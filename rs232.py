"""COM15 mode negotiation and incremental UBX/NMEA monitoring."""
import queue
import os
from pathlib import Path
import threading
import time
from collections import Counter

import serial
from serial.tools import list_ports
from pyubx2 import UBXReader
from pynmeagps import NMEAReader
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QGroupBox, QVBoxLayout, QHBoxLayout, QComboBox, QPushButton, QLabel, QTextEdit, QFileDialog, QMessageBox


class RawRecording:
    """Thread-safe, bounded-memory capture of received bytes before parsing."""
    def __init__(self):
        self.lock = threading.Lock()
        self.file = None
        self.path = ''
        self.size = 0
        self.error = ''

    def start(self, path):
        with self.lock:
            if self.file:
                raise RuntimeError('正在保存')
            self.file = open(path, 'wb')
            self.path, self.size, self.error = str(path), 0, ''

    def _close(self):
        handle, self.file = self.file, None
        if handle:
            try:
                handle.flush()
                os.fsync(handle.fileno())
            except OSError as error:
                self.error = str(error)
            finally:
                try:
                    handle.close()
                except OSError as error:
                    self.error = str(error)

    def stop(self):
        with self.lock:
            self._close()

    def append(self, data):
        with self.lock:
            if not self.file or not data:
                return
            try:
                written = self.file.write(data)
                self.file.flush()
                self.size += written
                if written != len(data):
                    raise OSError('原始数据写入不完整')
            except OSError as error:
                self.error = str(error)
                self._close()

    def snapshot(self):
        with self.lock:
            return self.file is not None, self.path, self.size, self.error


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
    def __init__(self, port, serial_factory=serial.Serial, uart=1):
        if uart not in (0, 1, 2):
            raise ValueError("UART 必须是 0、1 或 2")
        self.port = port
        self.uart = uart
        self.active_uart = None
        self.busy = True
        self.commands = queue.Queue(maxsize=1)
        self.serial_factory = serial_factory
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.thread = None
        self.state = '未连接'
        self.latest = {}
        self.counts = Counter()
        self.errors = 0
        self.last_received = None
        self.recording = RawRecording()

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
        self.recording.append(data)
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

    def request_mode(self, uart):
        if uart not in (None, 0, 1, 2):
            raise ValueError('UART 必须是 0、1 或 2')
        with self.lock:
            if self.busy or self.stop_event.is_set() or not self.thread or not self.thread.is_alive():
                return False
            self.busy = True
            self.commands.put_nowait(uart)
        return True

    @staticmethod
    def _send(port, command):
        if port.write(command) != len(command):
            raise OSError('串口命令写入不完整')
        port.flush()

    def _clear_data(self):
        with self.lock:
            self.latest.clear()
            self.counts.clear()
            self.errors = 0
            self.last_received = None

    def _exit_uart(self, port):
        self._state('460800 — 发送 exit…')
        port.baudrate = 460800
        self._send(port, b'exit\r')
        self.active_uart = None
        self._clear_data()
        if self.stop_event.wait(.5):
            return
        port.reset_input_buffer()
        self._state('exit 已发送；460800 命令通道，可选择 UART 后进入')

    def _enter_uart(self, port, uart):
        # Re-establish command mode even after reconnecting to a silent UART.
        self._exit_uart(port)
        if self.stop_event.is_set():
            return
        self._state(f'460800 — 发送 debug，准备进入 UART{uart}…')
        self._send(port, b'debug\r')
        response = b''
        deadline = time.monotonic() + 4
        while not self.stop_event.is_set() and time.monotonic() < deadline:
            response = (response + port.read(port.in_waiting or 1))[-4096:]
            if b'debug mode' in response.lower():
                break
        if self.stop_event.is_set():
            return
        if b'debug mode' not in response.lower():
            self._state('未收到 Debug mode 应答；可重试进入或发送 exit')
            return
        self._state(f'Debug mode 已确认；发送 uart {uart}…')
        self._send(port, f'uart {uart}\r'.encode('ascii'))
        if self.stop_event.wait(.1):
            return
        port.baudrate = 230400
        self.active_uart = uart
        self._state(f'UART{uart} 指令已发送 — 230400 等待数据（该通道可能无输出）')

    def _run(self):
        try:
            with self.serial_factory(self.port, 460800, timeout=.1, write_timeout=1) as port:
                self._enter_uart(port, self.uart)
                self.busy = False
                parser = MixedParser()
                while not self.stop_event.is_set():
                    try:
                        command = self.commands.get_nowait()
                    except queue.Empty:
                        command = 'receive'
                    if command != 'receive':
                        try:
                            if command is None:
                                self._exit_uart(port)
                            else:
                                self._enter_uart(port, command)
                        finally:
                            self.busy = False
                        parser = MixedParser()
                        continue
                    data = port.read(port.in_waiting or 1)
                    if self.active_uart is not None and self._consume(parser, data):
                        self._state(f'UART{self.active_uart} — 230400 接收 UBX + NMEA')
        except Exception as error:
            self._state(f'连接失败／已断开 — {error}')
        finally:
            self.recording.stop()
            self.busy = False
            if self.stop_event.is_set():
                self._state('已关闭串口')


class RS232Panel(QGroupBox):
    def __init__(self):
        super().__init__('卫导数据')
        self.reader = None
        layout = QVBoxLayout(self)
        controls = QHBoxLayout()
        self.port_combo = QComboBox()
        self.port_combo.setEditable(True)
        self.port_combo.addItems(sorted({p.device for p in list_ports.comports()} | {'COM15'}))
        self.port_combo.setCurrentText('COM15')
        self.button = QPushButton('打开 RS232')
        self.button.clicked.connect(self.toggle)
        self.uart_combo = QComboBox()
        self.uart_combo.addItems(['UART0', 'UART1', 'UART2'])
        self.uart_combo.setCurrentIndex(1)
        self.enter_button = QPushButton('进入 UART')
        self.enter_button.clicked.connect(self.enter_uart)
        self.exit_button = QPushButton('退出 UART')
        self.exit_button.clicked.connect(self.exit_uart)
        self.enter_button.setEnabled(False)
        self.exit_button.setEnabled(False)
        self.message_combo = QComboBox()
        controls.addWidget(self.port_combo)
        controls.addWidget(self.button)
        controls.addWidget(self.uart_combo)
        controls.addWidget(self.enter_button)
        controls.addWidget(self.exit_button)
        layout.addLayout(controls)
        recording_row = QHBoxLayout()
        self.save_button = QPushButton('保存原始数据')
        self.save_button.setEnabled(False)
        self.save_button.clicked.connect(self.toggle_recording)
        recording_row.addWidget(self.save_button)
        self.recording_status = QLabel('未保存 · UBX / NMEA 原始字节流')
        recording_row.addWidget(self.recording_status, 1)
        layout.addLayout(recording_row)
        controls = QHBoxLayout()
        controls.addWidget(self.message_combo, 1)
        layout.addLayout(controls)
        self.status = QLabel('选择 UART 后打开；exit 返回命令通道，无输出也可退出')
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        self.summary = QLabel('等待定位报文')
        self.summary.setWordWrap(True)
        layout.addWidget(self.summary)
        self.details = QTextEdit()
        self.details.setReadOnly(True)
        self.details.setMinimumHeight(130)
        layout.addWidget(self.details)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(200)

    def toggle_recording(self):
        if not self.reader:
            return
        if self.reader.recording.snapshot()[0]:
            self.reader.recording.stop()
        else:
            filename = str(Path.home() / f'卫导原始数据_{time.strftime("%Y%m%d_%H%M%S")}.bin')
            path, _ = QFileDialog.getSaveFileName(self, '保存卫导原始数据', filename, '二进制原始数据 (*.bin)')
            if not path:
                return
            if not Path(path).suffix:
                path += '.bin'
            # The port may disconnect while the file dialog is open.
            if not self.reader.thread or not self.reader.thread.is_alive():
                return
            try:
                self.reader.recording.start(path)
            except OSError as error:
                QMessageBox.critical(self, '保存失败', str(error))
        self.refresh()

    def toggle(self):
        if self.reader and self.reader.thread and self.reader.thread.is_alive():
            self.shutdown()
            return
        self.reader = RS232Reader(self.port_combo.currentText().strip(), uart=self.uart_combo.currentIndex())
        self.message_combo.clear()
        self.details.clear()
        self.summary.setText('等待定位报文')
        self.port_combo.setEnabled(False)
        self.button.setText('关闭 RS232')
        self.reader.start()

    def enter_uart(self):
        if self.reader:
            self.reader.request_mode(self.uart_combo.currentIndex())
            self.refresh()

    def exit_uart(self):
        if self.reader:
            self.reader.request_mode(None)
            self.refresh()

    def shutdown(self):
        if self.reader and not self.reader.stop():
            self.status.setText('正在关闭串口，请稍后重试')
            return False
        self.port_combo.setEnabled(True)
        self.button.setText('打开 RS232')
        self.enter_button.setEnabled(False)
        self.exit_button.setEnabled(False)
        self.save_button.setEnabled(False)
        self.refresh()
        return True

    def refresh(self):
        if not self.reader:
            return
        state, latest, counts, errors, last_received = self.reader.snapshot()
        connected = bool(self.reader.thread and self.reader.thread.is_alive())
        active, path, size, error = self.reader.recording.snapshot()
        if active and not connected:
            self.reader.recording.stop()
            active, path, size, error = self.reader.recording.snapshot()
        self.save_button.setEnabled(connected or active)
        self.save_button.setText('停止保存' if active else '保存原始数据')
        if path:
            state_text = '保存失败：' + error if error else ('正在保存' if active else '已保存')
            self.recording_status.setText(f'{state_text} · {size:,} 字节 · {Path(path).name}')
            self.recording_status.setToolTip(path + ('\n' + error if error else ''))
        else:
            self.recording_status.setText('未保存 · UBX / NMEA 原始字节流')
        self.enter_button.setEnabled(connected and not self.reader.busy)
        self.exit_button.setEnabled(connected and not self.reader.busy)
        if not latest:
            self.message_combo.clear()
            self.details.clear()
            self.summary.setText('当前通道暂无定位报文')
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
