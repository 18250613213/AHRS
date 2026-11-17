"""COM15 mode negotiation and incremental UBX/NMEA monitoring."""
import threading
import time
from collections import Counter

import serial
from serial.tools import list_ports
from pyubx2 import UBXReader
from pynmeagps import NMEAReader
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QGroupBox, QVBoxLayout, QHBoxLayout, QComboBox, QPushButton, QLabel, QTextEdit


class MixedParser:
    def __init__(self):
        self.buffer = b''
        self.errors = 0
        self.pending_since = None

    def feed(self, data):
        now = time.monotonic()
        if self.buffer and self.pending_since is not None and now - self.pending_since > 2:
            self.buffer = self.buffer[2:] if self.buffer.startswith(b'\xb5\x62') else self.buffer[1:]
            self.errors += 1
            self.pending_since = None
        self.buffer += data
        messages = []
        while self.buffer:
            candidates = [index for index in (self.buffer.find(b'\xb5\x62'), self.buffer.find(b'$')) if index >= 0]
            if not candidates:
                self.buffer = b'\xb5' if self.buffer.endswith(b'\xb5') else b''
                break
            self.buffer = self.buffer[min(candidates):]
            is_ubx = self.buffer.startswith(b'\xb5\x62')
            if is_ubx:
                if len(self.buffer) < 6:
                    break
                size = int.from_bytes(self.buffer[4:6], 'little') + 8
                if size > 65543:
                    self.buffer = self.buffer[2:]
                    self.errors += 1
                    continue
                if len(self.buffer) < size:
                    break
            else:
                end = self.buffer.find(b'\n')
                next_start = self.buffer.find(b'$', 1)
                ubx_start = self.buffer.find(b'\xb5\x62', 1)
                starts = [i for i in (next_start, ubx_start) if i >= 0]
                if starts and (end < 0 or min(starts) < end):
                    self.buffer = self.buffer[min(starts):]
                    self.errors += 1
                    continue
                if end < 0:
                    if len(self.buffer) > 4096:
                        self.buffer = b''
                        self.errors += 1
                    break
                size = end + 1
            raw = self.buffer[:size]
            try:
                parsed = UBXReader.parse(raw) if is_ubx else NMEAReader.parse(raw)
                if parsed is None:
                    raise ValueError('空报文')
                messages.append((raw, parsed))
                self.buffer = self.buffer[size:]
                self.pending_since = None
            except Exception:
                self.errors += 1
                # Rescan after a corrupt length/checksum instead of swallowing a following frame.
                self.buffer = self.buffer[2:] if is_ubx else self.buffer[size:]
        if self.buffer and self.pending_since is None:
            self.pending_since = now
        if not self.buffer:
            self.pending_since = None
        return messages


class RS232Reader:
    def __init__(self, port, serial_factory=serial.Serial):
        self.port = port
        self.serial_factory = serial_factory
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.thread = None
        self.state = '未连接'
        self.latest = {}
        self.counts = Counter()
        self.errors = 0
        self.last_received = None

    def start(self):
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def stop(self):
        self.stop_event.set()
        if self.thread:
            self.thread.join(timeout=2)
        return not self.thread or not self.thread.is_alive()

    def _state(self, text):
        with self.lock:
            self.state = text

    def snapshot(self):
        with self.lock:
            return self.state, dict(self.latest), dict(self.counts), self.errors, self.last_received

    def _consume(self, parser, data):
        messages = parser.feed(data)
        with self.lock:
            self.errors = parser.errors
            for raw, parsed in messages:
                identity = parsed.identity
                if identity not in self.latest and len(self.latest) >= 128:
                    continue
                self.latest[identity] = (raw, parsed, time.monotonic())
                self.counts[identity] += 1
                self.last_received = time.monotonic()
        return bool(messages)

    def _run(self):
        try:
            with self.serial_factory(self.port, 230400, timeout=.1, write_timeout=1) as port:
                self._state('检测 230400 混合输出…')
                parser = MixedParser()
                deadline = time.monotonic() + 1.5
                detected = False
                while not self.stop_event.is_set() and time.monotonic() < deadline:
                    if self._consume(parser, port.read(port.in_waiting or 1)):
                        detected = True
                        break
                if not detected and not self.stop_event.is_set():
                    port.baudrate = 460800
                    port.reset_input_buffer()
                    self._state('460800 — 发送 debug，等待 Debug mode…')
                    if port.write(b'debug\r') != 6:
                        raise OSError('debug 指令写入不完整')
                    port.flush()
                    response = b''
                    deadline = time.monotonic() + 4
                    while not self.stop_event.is_set() and time.monotonic() < deadline:
                        response = (response + port.read(port.in_waiting or 1))[-4096:]
                        if b'debug mode' in response.lower():
                            break
                    if self.stop_event.is_set():
                        return
                    if b'debug mode' not in response.lower():
                        raise TimeoutError('未收到 Debug mode，请检查端口及设备状态')
                    self._state('发送 uart 1，切换到 230400…')
                    if port.write(b'uart 1\r') != 7:
                        raise OSError('uart 1 指令写入不完整')
                    port.flush()
                    if self.stop_event.wait(.1):
                        return
                    port.baudrate = 230400
                    parser = MixedParser()
                self._state('230400 — 接收 UBX + NMEA')
                while not self.stop_event.is_set():
                    self._consume(parser, port.read(port.in_waiting or 1))
        except Exception as error:
            self._state(f'连接失败／已断开 — {error}')
        finally:
            if self.stop_event.is_set():
                self._state('已关闭')


class RS232Panel(QGroupBox):
    def __init__(self):
        super().__init__('RS232 — UBX / NMEA')
        self.reader = None
        layout = QVBoxLayout(self)
        controls = QHBoxLayout()
        self.port_combo = QComboBox()
        self.port_combo.setEditable(True)
        self.port_combo.addItems(sorted({p.device for p in list_ports.comports()} | {'COM15'}))
        self.port_combo.setCurrentText('COM15')
        self.button = QPushButton('打开 RS232')
        self.button.clicked.connect(self.toggle)
        self.message_combo = QComboBox()
        controls.addWidget(self.port_combo)
        controls.addWidget(self.button)
        controls.addWidget(self.message_combo, 1)
        layout.addLayout(controls)
        self.status = QLabel('460800 发送 debug / uart 1 → 230400 接收')
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        self.summary = QLabel('等待定位报文')
        self.summary.setWordWrap(True)
        layout.addWidget(self.summary)
        self.details = QTextEdit()
        self.details.setReadOnly(True)
        self.details.setMinimumHeight(90)
        layout.addWidget(self.details)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(200)

    def toggle(self):
        if self.reader and self.reader.thread and self.reader.thread.is_alive():
            self.shutdown()
            return
        self.reader = RS232Reader(self.port_combo.currentText().strip())
        self.message_combo.clear()
        self.details.clear()
        self.summary.setText('等待定位报文')
        self.port_combo.setEnabled(False)
        self.button.setText('关闭 RS232')
        self.reader.start()

    def shutdown(self):
        if self.reader and not self.reader.stop():
            self.status.setText('正在关闭串口，请稍后重试')
            return False
        self.port_combo.setEnabled(True)
        self.button.setText('打开 RS232')
        return True

    def refresh(self):
        if not self.reader:
            return
        state, latest, counts, errors, last_received = self.reader.snapshot()
        now = time.monotonic()
        age = now - last_received if last_received is not None else None
        stale = '；等待有效数据' if age is None else f'；最后接收 {age:.1f}s 前'
        if age is not None and age > 2:
            stale += '（数据过期）'
        self.status.setText(f'{state} | 有效 {sum(counts.values())} / 错误 {errors}{stale}')
        if self.reader.thread and not self.reader.thread.is_alive():
            self.port_combo.setEnabled(True)
            self.button.setText('打开 RS232')
        for identity in latest:
            if self.message_combo.findText(identity) < 0:
                self.message_combo.addItem(identity)
        nav = latest.get('NAV-PVT')
        if nav:
            _, msg, timestamp = nav
            self.summary.setText(
                f'NAV-PVT | 经度 {msg.lon:.7f}° 纬度 {msg.lat:.7f}° '
                f'海拔 {msg.hMSL / 1000:.3f}m | 卫星 {msg.numSV} '
                f'fixType={msg.fixType} 有效={msg.gnssFixOk} | {now - timestamp:.1f}s 前')
        selected = latest.get(self.message_combo.currentText())
        if selected:
            raw, msg, timestamp = selected
            fields = [f'{key} = {value}' for key, value in vars(msg).items() if not key.startswith('_')]
            raw_text = raw.hex(' ').upper() if raw.startswith(b'\xb5\x62') else raw.decode('ascii', errors='replace').strip()
            text = f'{msg.identity} | {counts[msg.identity]} 条 | {now - timestamp:.1f}s 前\n'
            text += '\n'.join(fields) + '\n原始报文\n' + raw_text
            scroll = self.details.verticalScrollBar().value()
            self.details.setPlainText(text)
            self.details.verticalScrollBar().setValue(scroll)
