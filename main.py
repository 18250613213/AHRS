import serial
from serial.tools import list_ports
import threading
import time
import os
import struct  
import sys
import math
import numpy as np
import trimesh
from collections import deque
from PIL import Image
from t02 import T02Parser
from recording import RecordingSession
from version import APP_VERSION
from decimal import Decimal, ROUND_HALF_UP, getcontext
getcontext().prec = 28

from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
                             QLabel, QComboBox, QPushButton, QCheckBox, QTextEdit,
                             QGroupBox, QGridLayout, QMessageBox, QOpenGLWidget, QSizePolicy,
                             QFileDialog, QProgressBar)
from PyQt5.QtCore import Qt, QTimer, QPointF, QCoreApplication, QMetaObject, Q_ARG, pyqtSignal
from PyQt5.QtGui import (QFont, QPainter, QPen, QBrush, QColor, QPolygonF,
                         QVector3D, QQuaternion, QMatrix4x4)
from OpenGL.GL import *
from OpenGL.GLU import *

try:
    import winreg
    HAS_WINREG = True
except ImportError:
    HAS_WINREG = False


class BinaryFrameParser:
    FRAME_HEADER = b'\xEB\x90'
    NAV_FRAME_ID = 0xD1
    NAV_FRAME_LENGTH = 124
    USE_VENDOR_ANGLE_CONVERSION = True
    MATH_RAD_TO_DEG = 180.0 / math.pi
    VENDOR_RAD_TO_DEG = 180.0 / 3.14

    @staticmethod
    def _read_f32(frame, offset):
        value = struct.unpack_from('<f', frame, offset)[0]
        return value, offset + 4

    @staticmethod
    def _read_i32(frame, offset):
        value = struct.unpack_from('<i', frame, offset)[0]
        return value, offset + 4

    @staticmethod
    def _read_u16(frame, offset):
        value = struct.unpack_from('<H', frame, offset)[0]
        return value, offset + 2

    @staticmethod
    def _read_i16(frame, offset):
        value = struct.unpack_from('<h', frame, offset)[0]
        return value, offset + 2

    @staticmethod
    def _get_rad_to_deg():
        return (
            BinaryFrameParser.VENDOR_RAD_TO_DEG
            if BinaryFrameParser.USE_VENDOR_ANGLE_CONVERSION
            else BinaryFrameParser.MATH_RAD_TO_DEG
        )

    @staticmethod
    def bytes_to_hex(data):
        return ' '.join(f'{b:02X}' for b in data)

    @staticmethod
    def find_frames(buffer):
        frames = []

        while True:
            index = buffer.find(BinaryFrameParser.FRAME_HEADER)

            if index < 0:
                if buffer.endswith(b'\xEB'):
                    buffer = b'\xEB'
                else:
                    buffer = b''
                break

            if index > 0:
                buffer = buffer[index:]

            if len(buffer) < 4:
                break

            frame_length = buffer[2]
            frame_id = buffer[3]

            if frame_length < 6 or frame_length > 255:
                buffer = buffer[1:]
                continue

            if len(buffer) < frame_length:
                break

            frame = buffer[:frame_length]

            if frame_id == BinaryFrameParser.NAV_FRAME_ID:
                frames.append(frame)

            buffer = buffer[frame_length:]

        return frames, buffer

    @staticmethod
    def _check_checksum(frame):
        if len(frame) < 6:
            return False

        calculated = sum(frame[:-2]) & 0xFFFF
        received = struct.unpack('<H', frame[-2:])[0]

        return calculated == received

    @staticmethod
    def _validate_frame(frame):
        if len(frame) < 6:
            return False, f'帧长度过短: {len(frame)}'
        
        if frame[0:2] != BinaryFrameParser.FRAME_HEADER:
            return False, f'帧头错误: {frame[:2].hex()}'
        
        if frame[2] != len(frame):
            return False, (
                f'帧长度字段错误: 字段={frame[2]}, 实际={len(frame)}'
            )
        
        if frame[3] != BinaryFrameParser.NAV_FRAME_ID:
            return False, f'不是导航帧: 0x{frame[3]:02X}'
        
        if not BinaryFrameParser._check_checksum(frame):
            calculated = sum(frame[:-2]) & 0xFFFF
            received = struct.unpack('<H', frame[-2:])[0]
            return False, (
                f'校验和错误: 计算值=0x{calculated:04X}, '
                f'接收值=0x{received:04X}'
            )
        
        try:
            counter = struct.unpack('<I', frame[4:8])[0]
            if counter > 0xFFFFFF:
                return False, f'计数器异常: {counter}'
        except:
            return False, '计数器解析失败'
        
        try:
            state_byte = frame[8]
            ahrs_status = state_byte & 0x07
            if ahrs_status not in [0, 1, 2]:
                return False, f'工作状态异常: {ahrs_status}'
        except:
            return False, '状态字节解析失败'
        
        try:
            pitch_raw = struct.unpack_from('<f', frame, 9)[0]
            if abs(pitch_raw) > 3.1416:
                return False, f'俯仰角原始值异常: {pitch_raw}'
        except:
            return False, '俯仰角解析失败'
        
        try:
            roll_raw = struct.unpack_from('<f', frame, 13)[0]
            if abs(roll_raw) > 3.1416:
                return False, f'滚转角原始值异常: {roll_raw}'
        except:
            return False, '滚转角解析失败'
        
        return True, '校验通过'

    @staticmethod
    def parse_frame(frame):
        try:
            is_valid, msg = BinaryFrameParser._validate_frame(frame)
            if not is_valid:
                return {'error': f'帧校验失败: {msg}', 'raw_hex': BinaryFrameParser.bytes_to_hex(frame)}

            parsed = {
                'frame_header': BinaryFrameParser.bytes_to_hex(frame[0:4]),
                'raw_hex': BinaryFrameParser.bytes_to_hex(frame),
            }

            rad_to_deg = BinaryFrameParser._get_rad_to_deg()
            offset = 4

            parsed['counter'] = struct.unpack_from('<I', frame, offset)[0]
            offset += 4

            state_byte = frame[offset]
            parsed['state'] = state_byte
            parsed['ahrs_status'] = state_byte & 0x07
            parsed['ahrs_status_desc'] = {0: '初始化', 1: '正常', 2: '错误'}.get(parsed['ahrs_status'], f'未知({parsed["ahrs_status"]})')
            parsed['compass_calibrate'] = (state_byte >> 3) & 0x01
            parsed['compass_calibrate_desc'] = '需要校准' if parsed['compass_calibrate'] else '正常'
            parsed['compass_healthy'] = (state_byte >> 4) & 0x01
            parsed['compass_healthy_desc'] = '故障' if parsed['compass_healthy'] else '正常'
            parsed['gyro_healthy'] = (state_byte >> 5) & 0x01
            parsed['gyro_healthy_desc'] = '故障' if parsed['gyro_healthy'] else '正常'
            parsed['accel_healthy'] = (state_byte >> 6) & 0x01
            parsed['accel_healthy_desc'] = '故障' if parsed['accel_healthy'] else '正常'
            parsed['baro_healthy'] = (state_byte >> 7) & 0x01
            parsed['baro_healthy_desc'] = '故障' if parsed['baro_healthy'] else '正常'
            offset += 1

            parsed['pitch_raw'] = struct.unpack_from('<f', frame, offset)[0]
            parsed['pitch'] = parsed['pitch_raw'] * rad_to_deg
            offset += 4

            parsed['roll_raw'] = struct.unpack_from('<f', frame, offset)[0]
            parsed['roll'] = parsed['roll_raw'] * rad_to_deg
            offset += 4

            parsed['yaw_raw'] = struct.unpack_from('<f', frame, offset)[0]
            parsed['yaw'] = parsed['yaw_raw'] * rad_to_deg
            offset += 4

            parsed['yaw_gps'] = struct.unpack_from('<f', frame, offset)[0]
            offset += 4

            parsed['pitch_rate_raw'] = struct.unpack_from('<f', frame, offset)[0]
            parsed['pitch_rate'] = parsed['pitch_rate_raw'] * rad_to_deg
            offset += 4

            parsed['roll_rate_raw'] = struct.unpack_from('<f', frame, offset)[0]
            parsed['roll_rate'] = parsed['roll_rate_raw'] * rad_to_deg
            offset += 4

            parsed['yaw_rate_raw'] = struct.unpack_from('<f', frame, offset)[0]
            parsed['yaw_rate'] = parsed['yaw_rate_raw'] * rad_to_deg
            offset += 4

            parsed['longitude'] = struct.unpack_from('<i', frame, offset)[0] * 1e-7
            offset += 4

            parsed['latitude'] = struct.unpack_from('<i', frame, offset)[0] * 1e-7
            offset += 4

            parsed['alt_baro'] = struct.unpack_from('<i', frame, offset)[0] * 0.01
            offset += 4

            parsed['alt_gps'] = struct.unpack_from('<i', frame, offset)[0] * 0.01
            offset += 4

            parsed['alt'] = struct.unpack_from('<i', frame, offset)[0] * 0.01
            offset += 4

            parsed['velocity_x'], offset = BinaryFrameParser._read_f32(frame, offset)
            parsed['velocity_y'], offset = BinaryFrameParser._read_f32(frame, offset)
            parsed['velocity_z'], offset = BinaryFrameParser._read_f32(frame, offset)
            parsed['velocity_air'], offset = BinaryFrameParser._read_f32(frame, offset)

            parsed['accel_x'], offset = BinaryFrameParser._read_f32(frame, offset)
            parsed['accel_y'], offset = BinaryFrameParser._read_f32(frame, offset)
            parsed['accel_z'], offset = BinaryFrameParser._read_f32(frame, offset)

            parsed['satellite_num'] = frame[offset]
            offset += 1

            parsed['hdop'] = struct.unpack_from('<H', frame, offset)[0] * 0.01
            offset += 2

            parsed['vdop'] = struct.unpack_from('<H', frame, offset)[0] * 0.01
            offset += 2

            parsed['gps_state'] = frame[offset]
            parsed['gps_state_desc'] = BinaryFrameParser._get_gps_state_desc(parsed['gps_state'])
            offset += 1

            parsed['gps_hh'] = frame[offset]
            offset += 1

            parsed['gps_mm'] = frame[offset]
            offset += 1

            parsed['gps_ss'] = frame[offset]
            offset += 1

            parsed['temperature'] = struct.unpack_from('<b', frame, offset)[0]
            offset += 1

            parsed['hdt'] = struct.unpack_from('<h', frame, offset)[0] * 0.1
            offset += 2

            parsed['hdg_dev'] = struct.unpack_from('<h', frame, offset)[0] * 0.1
            offset += 2

            redundancy_raw = frame[offset]
            offset += 1

            parsed['redundancy'] = redundancy_raw
            parsed['accel_source'] = (redundancy_raw >> 0) & 0x03
            parsed['gyro_source'] = (redundancy_raw >> 2) & 0x03
            parsed['compass_source'] = (redundancy_raw >> 4) & 0x03
            parsed['gps_source'] = (redundancy_raw >> 6) & 0x03

            parsed['accel_source_desc'] = {0: '外部', 1: '内部1', 2: '内部2'}.get(parsed['accel_source'], f'未知({parsed["accel_source"]})')
            parsed['gyro_source_desc'] = {0: '外部', 1: '内部1', 2: '内部2'}.get(parsed['gyro_source'], f'未知({parsed["gyro_source"]})')
            parsed['compass_source_desc'] = {0: '外部', 1: '内部'}.get(parsed['compass_source'], f'未知({parsed["compass_source"]})')
            parsed['gps_source_desc'] = {0: '外部', 1: '内部'}.get(parsed['gps_source'], f'未知({parsed["gps_source"]})')

            parsed['gps0_dt'] = frame[offset]
            offset += 1

            parsed['gps1_dt'] = frame[offset]
            offset += 1

            parsed['gps_vx'], offset = BinaryFrameParser._read_f32(frame, offset)
            parsed['gps_vy'], offset = BinaryFrameParser._read_f32(frame, offset)
            parsed['gps_vz'], offset = BinaryFrameParser._read_f32(frame, offset)

            parsed['gps_ms'] = struct.unpack_from('<H', frame, offset)[0]
            offset += 2

            parsed['gps_day'] = frame[offset]
            offset += 1

            parsed['gps_week'] = struct.unpack_from('<H', frame, offset)[0]
            offset += 2

            parsed['ahrs_state'] = frame[offset]
            ahrs_state_raw = parsed['ahrs_state']
            offset += 1
            bits = []
            if ahrs_state_raw & 0x10:
                bits.append('垂直姿态')
            if ahrs_state_raw & 0x20:
                bits.append('相对位置')
            if ahrs_state_raw & 0x40:
                bits.append('极坐标')
            if ahrs_state_raw & 0x80:
                bits.append('有效')
            else:
                bits.append('无效')
            parsed['ahrs_state_desc'] = '|'.join(bits) if bits else '未知'

            parsed['engine_rpm'] = struct.unpack_from('<H', frame, offset)[0]
            offset += 2

            parsed['gps_time_str'] = f"{parsed['gps_hh']:02d}:{parsed['gps_mm']:02d}:{parsed['gps_ss']:02d}.{parsed['gps_ms']:03d}"

            assert offset == len(frame) - 2, f'解析偏移错误: offset={offset}, frame_len={len(frame)}'

            return parsed

        except Exception as e:
            return {'error': f'解析异常: {str(e)}', 'raw_hex': BinaryFrameParser.bytes_to_hex(frame)}

    @staticmethod
    def _get_gps_state_desc(state):
        state_map = {
            0: 'NO_GPS',
            1: 'NO_FIX',
            2: 'GPS_FIX_2D',
            3: 'GPS_FIX_3D',
            4: 'GPS_FIX_3D_DGPS',
            5: 'GPS_FIX_3D_RTK_FLOAT',
            6: 'GPS_FIX_3D_RTK_FIXED',
        }
        return state_map.get(state, f'未知({state})')


class SerialBinaryReader:
    def __init__(self, port='COM3', baud_rate=9600, timeout=0.01, parser=BinaryFrameParser):
        self.parser = parser
        self.port = port
        self.baud_rate = baud_rate
        self.timeout = timeout
        self.serial_port = None
        self.running = False
        self.receive_thread = None
        self.data_callback = None
        self.last_error = ''
        self.raw_error = ''

        self.raw_lock = threading.Lock()
        self.raw_file = None
        self.raw_mode = None
        self.raw_path = ''
        self.raw_hex_pending = bytearray()
        self.raw_bytes_written = 0

    def open(self):
        try:
            self.serial_port = serial.Serial(
                port=self.port,
                baudrate=self.baud_rate,
                timeout=self.timeout,
                write_timeout=1.0,
                bytesize=8,
                parity='N',
                stopbits=1,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False
            )
            self.serial_port.set_buffer_size(rx_size=65536, tx_size=65536)
            return True
        except (serial.SerialException, OSError) as e:
            if self.serial_port:
                self.serial_port.close()
            return str(e)

    def close(self):
        self.stop_receiving()
        self.stop_raw_record()
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()

    def start_receiving(self, callback=None):
        if self.receive_thread and self.receive_thread.is_alive():
            raise RuntimeError('接收线程尚未退出')
        self.last_error = ''
        self.data_callback = callback
        self.running = True
        self.receive_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self.receive_thread.start()

    def stop_receiving(self):
        self.running = False
        if self.receive_thread and self.receive_thread is not threading.current_thread():
            self.receive_thread.join(timeout=2.0)
            if self.receive_thread.is_alive():
                raise RuntimeError('接收线程停止超时，请稍后重试')

    def _receive_loop(self):
        buffer = b''
        try:
            while self.running:
                if not self.serial_port or not self.serial_port.is_open:
                    raise OSError('串口已关闭')
                waiting = self.serial_port.in_waiting
                data = self.serial_port.read(waiting or 1)
                if not data:
                    continue
                self._write_raw(data)
                frames, buffer = self.parser.find_frames(buffer + data)
                for frame in frames:
                    if self.data_callback:
                        self.data_callback(self.parser.parse_frame(frame))
        except Exception as error:
            if self.running:
                self.last_error = f'串口接收失败: {error}'
                if self.data_callback:
                    self.data_callback({'error': self.last_error})
        finally:
            self.running = False

    def write_data(self, data):
        if self.serial_port and self.serial_port.is_open:
            written = self.serial_port.write(data)
            self.serial_port.flush()
            return written == len(data)
        return False

    def start_raw_record(self, path, mode='hex'):
        with self.raw_lock:
            if self.raw_file is not None:
                return '已在记录中'
            try:
                if mode == 'bin':
                    self.raw_file = open(path, 'wb')
                else:
                    self.raw_file = open(path, 'w', encoding='ascii', newline='\n')
            except OSError as e:
                self.raw_file = None
                return str(e)
            self.raw_mode = mode
            self.raw_error = ''
            self.raw_path = path
            self.raw_hex_pending = bytearray()
            self.raw_bytes_written = 0
            return True

    def _write_raw(self, data):
        with self.raw_lock:
            if self.raw_file is None:
                return
            try:
                if self.raw_mode == 'bin':
                    self.raw_file.write(data)
                else:
                    self.raw_hex_pending.extend(data)
                    while len(self.raw_hex_pending) >= 32:
                        chunk = bytes(self.raw_hex_pending[:32])
                        del self.raw_hex_pending[:32]
                        self.raw_file.write(' '.join(f'{b:02X}' for b in chunk) + '\n')
                self.raw_bytes_written += len(data)
                self.raw_file.flush()
            except OSError as error:
                self.raw_error = f'原始录制失败: {error}'
                try:
                    self.raw_file.close()
                except OSError:
                    pass
                self.raw_file = None

    def stop_raw_record(self):
        with self.raw_lock:
            if self.raw_file is None:
                return None
            path = self.raw_path
            size = self.raw_bytes_written
            try:
                if self.raw_mode == 'hex' and self.raw_hex_pending:
                    self.raw_file.write(' '.join(f'{b:02X}' for b in self.raw_hex_pending) + '\n')
                self.raw_file.flush()
            except OSError as error:
                self.raw_error = f'原始录制结束失败: {error}'
            try:
                self.raw_file.close()
            except OSError as error:
                self.raw_error = f'原始录制关闭失败: {error}'
            self.raw_file = None
            self.raw_mode = None
            self.raw_path = ''
            self.raw_hex_pending = bytearray()
            self.raw_bytes_written = 0
            return path, size


class MainWindow(QMainWindow):
    export_finished = pyqtSignal(str, str)
    transfer_progress = pyqtSignal(int)
    transfer_finished = pyqtSignal(object, str, int)

    def __init__(self):
        super().__init__()
        self.setWindowTitle(f"AHRS 数据解析器 V{APP_VERSION}")
        self.setGeometry(100, 100, 1200, 800)

        self.reader = None
        self.save_enabled = False
        self.recording = None
        self.save_lock = threading.Lock()
        self.recording_error = ''
        self.export_thread = None
        self._close_pending = False
        self._closing = False
        self.last_valid_time = None
        self.connected_time = None
        self.last_frame = None
        self._transfer_cancel = threading.Event()

        self._raw_recording = False

        self.last_update_time = "--:--:--"

        self.data_queue = deque(maxlen=2000)
        self.data_lock = threading.Lock()

        self.rx_frame_count = 0
        self.rx_frequency = 0.0
        self.rx_frequency_start = time.perf_counter()

        self.valid_frame_count = 0
        self.error_frame_count = 0

        self._send_file_running = False
        self._send_file_cancelled = False

        self.timer = QTimer()
        self.timer.setTimerType(Qt.PreciseTimer)
        self.timer.timeout.connect(self._process_data)
        self.timer.start(20)

        self._init_filter_queues()

        self._create_widgets()
        self._fill_serial_ports()
        self.export_finished.connect(self._on_export_finished)
        self.transfer_progress.connect(self.progress_bar.setValue)
        self.transfer_finished.connect(self._on_transfer_finished)

    def _init_filter_queues(self):
        self.filter_queues = {}
        filter_fields = ['pitch', 'roll', 'yaw', 'yaw_gps', 'pitch_rate', 'roll_rate', 'yaw_rate',
                         'velocity_x', 'velocity_y', 'velocity_z', 'accel_x', 'accel_y', 'accel_z']
        for field in filter_fields:
            self.filter_queues[field] = []
        self.FILTER_WINDOW_SIZE = 21

    def _create_widgets(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)

        config_group = QGroupBox("串口配置")
        config_layout = QHBoxLayout(config_group)
        config_layout.setSpacing(3)
        config_layout.setContentsMargins(10, 10, 10, 10)

        port_frame = QWidget()
        port_layout = QHBoxLayout(port_frame)
        port_layout.setSpacing(3)
        port_layout.setContentsMargins(0, 0, 0, 0)
        port_layout.addWidget(QLabel("串口:"))
        self.port_combo = QComboBox()
        self.port_combo.setFixedWidth(90)
        port_layout.addWidget(self.port_combo)
        config_layout.addWidget(port_frame)

        baud_frame = QWidget()
        baud_layout = QHBoxLayout(baud_frame)
        baud_layout.setSpacing(3)
        baud_layout.setContentsMargins(0, 0, 0, 0)
        baud_layout.addWidget(QLabel("波特率:"))
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["4800", "9600", "19200", "38400", "57600", "115200",
                                   "230400", "307200", "460800", "921600", "自定义"])
        self.baud_combo.setCurrentText("460800")
        self.baud_combo.setFixedWidth(90)
        baud_layout.addWidget(self.baud_combo)
        config_layout.addWidget(baud_frame)

        self.protocol_combo = QComboBox()
        self.protocol_combo.addItems(['T02 ($BDFPDL)', 'AHRS (EB90)'])
        self.protocol_combo.setToolTip('选择与设备输出一致的协议')
        config_layout.addWidget(self.protocol_combo)

        self.connect_btn = QPushButton("打开串口")
        self.connect_btn.clicked.connect(self.toggle_connection)
        config_layout.addWidget(self.connect_btn)

        self.refresh_btn = QPushButton("刷新端口")
        self.refresh_btn.clicked.connect(self._fill_serial_ports)
        config_layout.addWidget(self.refresh_btn)

        self.save_btn = QPushButton("开始保存")
        self.save_btn.clicked.connect(self.toggle_save)
        config_layout.addWidget(self.save_btn)

        self.save_count_label = QLabel("已保存: 0 条")
        config_layout.addWidget(self.save_count_label)

        self.export_btn = QPushButton('导出 Excel')
        self.export_btn.clicked.connect(self._export_to_excel)
        self.export_btn.setEnabled(False)
        config_layout.addWidget(self.export_btn)

        self.hex_display_check = QCheckBox("十六进制显示")
        self.hex_display_check.setChecked(True)
        self.hex_display_check.toggled.connect(self._refresh_raw_preview)
        config_layout.addWidget(self.hex_display_check)

        self.vendor_compat_check = QCheckBox("厂家角度兼容")
        self.vendor_compat_check.setChecked(BinaryFrameParser.USE_VENDOR_ANGLE_CONVERSION)
        self.vendor_compat_check.stateChanged.connect(self._on_vendor_compat_changed)
        config_layout.addWidget(self.vendor_compat_check)

        model_frame = QWidget()
        model_layout = QHBoxLayout(model_frame)
        model_layout.setSpacing(3)
        model_layout.setContentsMargins(0, 0, 0, 0)
        model_layout.addWidget(QLabel("模型:"))
        self.model_combo = QComboBox()
        self.model_combo.addItems([
            "歼-20",
            "阿帕奇", "陆军一号", "未来01", "坦克", "Shahed-136", "Lancet-3"
        ])
        self.model_combo.setFixedWidth(100)
        self.model_combo.currentTextChanged.connect(self._on_model_changed)
        model_layout.addWidget(self.model_combo)
        config_layout.addWidget(model_frame)

        main_layout.addWidget(config_group)

        file_group = QGroupBox("文件传输")
        file_layout = QHBoxLayout(file_group)
        file_layout.setSpacing(5)
        file_layout.setContentsMargins(10, 10, 10, 10)

        self.send_file_btn = QPushButton("发送文件")
        self.send_file_btn.clicked.connect(self._on_send_file)
        file_layout.addWidget(self.send_file_btn)

        self.burn_file_btn = QPushButton("烧录文件")
        self.burn_file_btn.clicked.connect(self._on_burn_file)
        file_layout.addWidget(self.burn_file_btn)

        self.cancel_send_btn = QPushButton("取消")
        self.cancel_send_btn.clicked.connect(self._on_cancel_send)
        self.cancel_send_btn.setEnabled(False)
        file_layout.addWidget(self.cancel_send_btn)

        self.file_path_label = QLabel("文件: 未选择")
        self.file_path_label.setFixedWidth(250)
        self.file_path_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        file_layout.addWidget(self.file_path_label)

        self.progress_bar = QProgressBar()
        self.progress_bar.setFixedWidth(200)
        self.progress_bar.setValue(0)
        self.progress_bar.setTextVisible(True)
        file_layout.addWidget(self.progress_bar)

        self.send_status_label = QLabel("")
        self.send_status_label.setFixedWidth(120)
        file_layout.addWidget(self.send_status_label)

        main_layout.addWidget(file_group)

        plane_group = QGroupBox("飞机姿态")
        plane_layout = QVBoxLayout(plane_group)
        self.plane_widget = PlaneGLWidget()
        self.plane_widget.setFixedHeight(250)
        plane_layout.addWidget(self.plane_widget)
        main_layout.addWidget(plane_group)

        self.smoothing_check = QCheckBox('平滑三维姿态（有显示延迟）')
        self.smoothing_check.toggled.connect(self._init_filter_queues)
        main_layout.addWidget(self.smoothing_check)

        parsed_group = QGroupBox("解析数据")
        parsed_layout = QHBoxLayout(parsed_group)

        col1_group = QGroupBox("姿态数据")
        col1_layout = QGridLayout(col1_group)
        col1_labels = [
            ("计数器", "counter"),
            ("俯仰角(°)", "pitch"),
            ("滚转角(°)", "roll"),
            ("航向角(°)", "yaw"),
            ("航迹角(°)", "yaw_gps"),
            ("俯仰角速率(°/s)", "pitch_rate"),
            ("滚转角速率(°/s)", "roll_rate"),
            ("航向角速率(°/s)", "yaw_rate"),
            ("北向加速度(m/s²)", "accel_x"),
            ("东向加速度(m/s²)", "accel_y"),
            ("地向加速度(m/s²)", "accel_z"),
        ]

        col2_group = QGroupBox("导航数据")
        col2_layout = QGridLayout(col2_group)
        col2_labels = [
            ("经度(°)", "longitude"),
            ("纬度(°)", "latitude"),
            ("北向速度(m/s)", "velocity_x"),
            ("东向速度(m/s)", "velocity_y"),
            ("地向速度(m/s)", "velocity_z"),
            ("GPS北向速度(m/s)", "gps_vx"),
            ("GPS东向速度(m/s)", "gps_vy"),
            ("GPS地向速度(m/s)", "gps_vz"),
            ("空速(m/s)", "velocity_air"),
            ("气压高度(m)", "alt_baro"),
            ("GPS高度(m)", "alt_gps"),
            ("融合高度(m)", "alt"),
            ("温度(°C)", "temperature"),
        ]

        col3_group = QGroupBox("GPS数据")
        col3_layout = QGridLayout(col3_group)
        col3_labels = [
            ("GPS时间", "gps_time_str"),
            ("定位状态", "gps_state_desc"),
            ("GPS星数", "satellite_num"),
            ("HDOP(m)", "hdop"),
            ("VDOP(m)", "vdop"),
            ("HDT(°)", "hdt"),
            ("HDG标准差(°)", "hdg_dev"),
            ("工作状态", "ahrs_status_desc"),
            ("当前加计", "accel_source_desc"),
            ("当前陀螺", "gyro_source_desc"),
            ("当前罗盘", "compass_source_desc"),
            ("当前GPS", "gps_source_desc"),
        ]

        LABEL_WIDTH = 138
        VALUE_WIDTH = 110

        def add_data_rows(layout, items):
            rows = []
            for row, (text, key) in enumerate(items):
                name_label = QLabel(text)
                name_label.setFont(QFont("Arial", 10, QFont.Bold))
                name_label.setFixedWidth(LABEL_WIDTH)
                name_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
                name_label.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Preferred)

                value_label = QLabel("--")
                value_label.setFont(QFont("Arial", 10))
                value_label.setFixedWidth(VALUE_WIDTH)
                value_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
                value_label.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Preferred)

                layout.addWidget(name_label, row, 0, Qt.AlignLeft | Qt.AlignVCenter)
                layout.addWidget(value_label, row, 1, Qt.AlignRight | Qt.AlignVCenter)

                self.value_labels[key] = value_label
                rows.append((name_label, value_label))
            self.field_rows.append(rows)

        for layout in (col1_layout, col2_layout, col3_layout):
            layout.setContentsMargins(8, 6, 8, 6)
            layout.setHorizontalSpacing(8)
            layout.setVerticalSpacing(3)
            layout.setColumnStretch(0, 0)
            layout.setColumnStretch(1, 0)
            layout.setColumnStretch(2, 1)
            layout.setAlignment(Qt.AlignLeft | Qt.AlignTop)

        self.value_labels = {}

        self.legacy_display_groups = (col1_labels, col2_labels, col3_labels)
        self.field_rows = []

        add_data_rows(col1_layout, col1_labels)
        add_data_rows(col2_layout, col2_labels)
        add_data_rows(col3_layout, col3_labels)
        self.protocol_combo.currentIndexChanged.connect(self._configure_protocol_fields)
        self._configure_protocol_fields()

        parsed_layout.addWidget(col1_group, 1)
        parsed_layout.addWidget(col2_group, 1)
        parsed_layout.addWidget(col3_group, 1)
        parsed_layout.setSpacing(6)
        parsed_layout.setContentsMargins(3, 3, 3, 3)
        main_layout.addWidget(parsed_group)

        self.raw_preview = QTextEdit()
        self.raw_preview.setReadOnly(True)
        self.raw_preview.setMaximumHeight(64)
        self.raw_preview.setPlaceholderText('最近一帧原始报文')
        main_layout.addWidget(self.raw_preview)
        self.recording_label = QLabel('采集记录保存到本地 CSV，可导出 Excel')
        self.recording_label.setWordWrap(True)
        main_layout.addWidget(self.recording_label)

        status_layout = QHBoxLayout()
        self.status_label = QLabel("状态: 未连接")
        self.status_label.setFont(QFont("Arial", 10))
        status_layout.addWidget(self.status_label)

        self.raw_save_btn = QPushButton("保存原始数据")
        self.raw_save_btn.setToolTip("把串口收到的原始字节写入 TXT(十六进制) 或 BIN 文件")
        self.raw_save_btn.clicked.connect(self.toggle_raw_record)
        status_layout.addWidget(self.raw_save_btn)

        self.raw_count_label = QLabel("原始数据: 未记录")
        self.raw_count_label.setFont(QFont("Arial", 10))
        status_layout.addWidget(self.raw_count_label)

        self.update_label = QLabel("")
        self.update_label.setFont(QFont("Arial", 10))
        status_layout.addWidget(self.update_label, 1, Qt.AlignRight)

        main_layout.addLayout(status_layout)

    def _configure_protocol_fields(self):
        is_t02 = self.protocol_combo.currentIndex() == 0
        groups = T02Parser.DISPLAY_GROUPS if is_t02 else self.legacy_display_groups
        self.value_labels.clear()
        for rows, fields in zip(self.field_rows, groups):
            for index, (name_label, value_label) in enumerate(rows):
                visible = index < len(fields)
                name_label.setVisible(visible)
                value_label.setVisible(visible)
                if visible:
                    title, key = fields[index]
                    name_label.setText(title)
                    value_label.setText('--')
                    self.value_labels[key] = value_label
        self.vendor_compat_check.setEnabled(not is_t02)
        self.vendor_compat_check.setToolTip('仅用于 EB90 二进制协议的弧度转角度')
        self.burn_file_btn.setEnabled(not is_t02 and not self._send_file_running)
        self.burn_file_btn.setToolTip('T02 烧录协议尚未提供' if is_t02 else '旧设备写入流程，设备应答尚未验证')
        self._init_filter_queues()

    def _fill_serial_ports(self):
        selected_port = self.port_combo.currentText()
        ports = []

        try:
            for port in list_ports.comports():
                ports.append(port.device)
        except:
            pass

        if HAS_WINREG:
            try:
                key = winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, r"HARDWARE\DEVICEMAP\SERIALCOMM")
                try:
                    for i in range(winreg.QueryInfoKey(key)[1]):
                        try:
                            value_name, value_data, _ = winreg.EnumValue(key, i)
                            if isinstance(value_data, str) and value_data.startswith('COM'):
                                ports.append(value_data)
                        except:
                            continue
                finally:
                    winreg.CloseKey(key)
            except:
                pass

        ports = sorted(list(set(ports)))
        self.port_combo.clear()
        self.port_combo.addItems(ports)
        if ports:
            preferred = selected_port if selected_port in ports else 'COM31'
            self.port_combo.setCurrentText(preferred if preferred in ports else ports[0])

    def _on_model_changed(self, model_name):
        self.plane_widget.load_model_by_name(model_name)

    def _on_vendor_compat_changed(self, state):
        BinaryFrameParser.USE_VENDOR_ANGLE_CONVERSION = (state == Qt.Checked)

    def toggle_connection(self):
        if self._send_file_running:
            self.status_label.setText('请先取消传输，等待串口恢复后再断开')
            return
        if self.reader:
            try:
                self.reader.close()
            except RuntimeError as error:
                self.status_label.setText(str(error))
                return
            if self._raw_recording:
                self._finish_raw_record()
            self.reader = None
            self._set_connection_controls(False)
            self.status_label.setText('状态: 已断开')
            self.rx_frequency = 0.0
            return
        port = self.port_combo.currentText()
        if not port:
            QMessageBox.warning(self, '警告', '请选择串口')
            return
        try:
            baud = int(self.baud_combo.currentText().strip())
            if not 0 < baud <= 921600:
                raise ValueError()
        except ValueError:
            QMessageBox.warning(self, '警告', '波特率必须为 1 到 921600 的整数')
            return
        parser = T02Parser if self.protocol_combo.currentIndex() == 0 else BinaryFrameParser
        reader = SerialBinaryReader(port=port, baud_rate=baud, parser=parser)
        result = reader.open()
        if result is not True:
            QMessageBox.critical(self, '连接失败', f'无法打开串口: {result}')
            return
        self.reader = reader
        with self.data_lock:
            self.data_queue.clear()
            self.last_valid_time = None
            self.connected_time = time.perf_counter()
            self.rx_frequency_start = self.connected_time
            self.rx_frame_count = 0
            self.rx_frequency = 0.0
        self._init_filter_queues()
        self._set_connection_controls(True)
        reader.start_receiving(callback=self.on_data_received)
        self.status_label.setText(f'状态: 已连接 {port} @ {baud}，等待有效报文')

    def _set_connection_controls(self, connected):
        self.connect_btn.setText('关闭串口' if connected else '打开串口')
        self.port_combo.setEnabled(not connected)
        self.baud_combo.setEnabled(not connected)
        self.refresh_btn.setEnabled(not connected)
        self.protocol_combo.setEnabled(not connected and not self.save_enabled)

    def on_data_received(self, data):
        now = time.perf_counter()
        valid = isinstance(data, dict) and 'error' not in data and 'frame_header' in data
        if valid:
            with self.save_lock:
                if self.save_enabled and self.recording:
                    try:
                        self.recording.append(self._build_save_record(data))
                    except (OSError, ValueError, RuntimeError) as error:
                        self.save_enabled = False
                        self.recording_error = f'记录失败: {error}'
        with self.data_lock:
            if valid:
                self.last_valid_time = now
                self.rx_frame_count += 1
                self.valid_frame_count += 1
            elif isinstance(data, dict) and data.get('error'):
                self.error_frame_count += 1
            self.data_queue.append(data)

    def _process_data(self):
        now = time.perf_counter()
        latest_data = None
        latest_error = None
        with self.data_lock:
            while self.data_queue:
                data = self.data_queue.popleft()
                if data.get('error'):
                    latest_error = data['error']
                elif data.get('type') != 'raw_hex':
                    latest_data = data
            elapsed = now - self.rx_frequency_start
            if elapsed >= 1.0:
                self.rx_frequency = self.rx_frame_count / elapsed
                self.rx_frame_count = 0
                self.rx_frequency_start = now
            last_valid = self.last_valid_time
        if latest_data is not None:
            self._update_ui(latest_data)
            self.last_update_time = time.strftime('%H:%M:%S')
        if latest_error:
            self.status_label.setText(latest_error)
        reader = self.reader
        if reader and reader.raw_error:
            self._raw_recording = False
            self.raw_save_btn.setText('保存原始数据')
            self.raw_count_label.setText(reader.raw_error)
        if reader and not reader.running and not self._send_file_running:
            error = reader.last_error or '接收已停止'
            try:
                reader.close()
            except RuntimeError:
                self.status_label.setText(error)
                return
            if reader.raw_error:
                self.raw_count_label.setText(reader.raw_error)
            self.reader = None
            self._raw_recording = False
            self.raw_save_btn.setText('保存原始数据')
            self._set_connection_controls(False)
            self.status_label.setText(f'{error}；可重新打开串口')
            reader = None
        last_activity = last_valid if last_valid is not None else self.connected_time
        stale = last_activity is not None and now - last_activity >= 2.0
        if stale or not reader or self._send_file_running:
            self.rx_frequency = 0.0
        if stale and reader and not self._send_file_running:
            self.status_label.setText('数据超时：连续 2 秒未收到有效报文')
        freshness = '（数据过期）' if stale or (not reader and last_valid is not None) else ''
        self.update_label.setText(
            f'更新时间: {self.last_update_time}{freshness}  '
            f'接收频率: {self.rx_frequency:.1f} Hz  '
            f'有效帧: {self.valid_frame_count}  错误帧: {self.error_frame_count}')
        with self.save_lock:
            count = self.recording.count if self.recording else 0
            recording_error = self.recording_error
        self.save_count_label.setText(f'已保存: {count} 条')
        if recording_error:
            self.recording_label.setText(recording_error + '；已有 CSV 仍保留')
            self.save_btn.setText('开始保存')
            self.export_btn.setEnabled(bool(count) and not self._exporting())
        if self._raw_recording and reader:
            self.raw_count_label.setText(f'原始数据: {reader.raw_bytes_written:,} B（记录中）')

    def _apply_filter(self, key, value):
        if key not in self.filter_queues:
            return value
        queue = self.filter_queues[key]
        # Unwrap circular angles before smoothing across 0/360 or -180/180.
        if key in ('yaw', 'roll') and queue:
            value = queue[-1] + (value - queue[-1] + 180.0) % 360.0 - 180.0
        queue.append(value)
        if len(queue) > self.FILTER_WINDOW_SIZE:
            queue.pop(0)
        if len(queue) == 0:
            return value
        
        if len(queue) < 3:
            total = sum(queue)
            return total / len(queue)
        
        sorted_vals = sorted(queue)
        n = len(sorted_vals)
        
        if n % 2 == 1:
            median = sorted_vals[n // 2]
        else:
            median = (sorted_vals[n // 2 - 1] + sorted_vals[n // 2]) / 2.0
        
        trimmed_queue = sorted_vals[1:-1]
        if len(trimmed_queue) >= 3:
            total = sum(trimmed_queue)
            trimmed_mean = total / len(trimmed_queue)
            return (median * 0.6 + trimmed_mean * 0.4)
        
        return median

    def _update_ui(self, data):
        if 'error' in data and data['error']:
            self.status_label.setText(data['error'])
            return

        if data.get('type') == 'raw_hex':
            return

        self.last_frame = data
        self._refresh_raw_preview()
        smooth = self.smoothing_check.isChecked()
        filtered_pitch = self._apply_filter('pitch', data.get('pitch', 0.0)) if smooth else data.get('pitch', 0.0)
        filtered_roll = self._apply_filter('roll', data.get('roll', 0.0)) if smooth else data.get('roll', 0.0)
        filtered_yaw = self._apply_filter('yaw', data.get('yaw', 0.0)) if smooth else data.get('yaw', 0.0)
        self.plane_widget.update_attitude(filtered_pitch, filtered_roll, filtered_yaw)
        protocol = data.get('protocol', 'EB90')
        self.status_label.setText(f"状态: 已接收 {protocol} | {self.port_combo.currentText()}")

        for key, label in self.value_labels.items():
            if key in data:
                value = data[key]
                if isinstance(value, float):
                    if key in ['longitude', 'latitude']:
                        label.setText(f"{value:.7f}")
                    elif key == 'gps_seconds':
                        label.setText(f"{value:.3f}")
                    elif key in ['alt_baro', 'alt_gps', 'alt']:
                        label.setText(f"{value:.3f}" if data.get('protocol') == 'T02' else f"{value:.2f}")
                    elif key == 'pitch':
                        label.setText(f"{value:.6f}")
                    elif key == 'roll':
                        label.setText(f"{value:.6f}")
                    elif key == 'yaw':
                        label.setText(f"{value:.6f}")
                    elif key == 'yaw_gps':
                        label.setText(f"{value:.6f}")
                    elif key in ['velocity_x', 'velocity_y', 'velocity_z', 'gps_vx', 'gps_vy', 'gps_vz',
                              'pitch_rate', 'roll_rate', 'yaw_rate', 'accel_x', 'accel_y', 'accel_z']:
                        label.setText(f"{value:.6f}")
                    elif key == 'hdop':
                        label.setText(f"{value:.2f}")
                    elif key == 'vdop':
                        label.setText(f"{value:.2f}")
                    elif key == 'temperature':
                        label.setText(f"{int(value)}")
                    else:
                        label.setText(f"{value:.4f}")
                elif key in ('system_state', 'flight_state'):
                    label.setText(f'{value} (0x{value:X})')
                else:
                    label.setText(str(value))
            else:
                label.setText("--")

        self.last_update_time = time.strftime('%H:%M:%S')

    def _refresh_raw_preview(self):
        if self.last_frame is None or not hasattr(self, 'raw_preview'):
            return
        data = self.last_frame
        text = data.get('raw_hex', '') if self.hex_display_check.isChecked() else data.get('raw_text', data.get('raw_hex', ''))
        self.raw_preview.setPlainText(text)

    def toggle_save(self):
        if self._exporting():
            return
        if self.save_enabled:
            if self._stop_recording():
                self._export_to_excel()
            return
        try:
            session = RecordingSession(self._default_raw_dir())
        except OSError as error:
            QMessageBox.critical(self, '保存失败', str(error))
            return
        with self.save_lock:
            self.recording = session
            self.recording_error = ''
            self.save_enabled = True
        self.protocol_combo.setEnabled(False)
        self.save_btn.setText('停止保存')
        self.export_btn.setEnabled(False)
        self.recording_label.setText(f'正在持续写入 CSV — {session.path}')

    def _stop_recording(self):
        with self.save_lock:
            self.save_enabled = False
            session = self.recording
            try:
                if session:
                    session.close()
            except OSError as error:
                self.recording_error = f'记录文件关闭失败: {error}'
        self.save_btn.setText('开始保存')
        self.protocol_combo.setEnabled(self.reader is None)
        self.export_btn.setEnabled(bool(session and session.count) and not self._exporting())
        if self.recording_error:
            self.recording_label.setText(self.recording_error + '；请检查磁盘，已有 CSV 仍保留')
            return False
        if session:
            self.recording_label.setText(f'CSV 已保存 — {session.path}')
        return True

    def _exporting(self):
        return self.export_thread is not None and self.export_thread.is_alive()

    def _build_save_record(self, data):
        if data.get('protocol') == 'T02':
            record = {title: data[key] for group in T02Parser.DISPLAY_GROUPS
                      for title, key in group}
            record['原始报文'] = data['raw_text']
            return record
        return {
            '计数器': data.get('counter', 0),
            '俯仰角(°)': data.get('pitch', 0.0),
            '滚转角(°)': data.get('roll', 0.0),
            '航向角(°)': data.get('yaw', 0.0),
            '航迹角(°)': data.get('yaw_gps', 0.0),
            '俯仰角速率(°/s)': data.get('pitch_rate', 0.0),
            '滚转角速率(°/s)': data.get('roll_rate', 0.0),
            '航向角速率(°/s)': data.get('yaw_rate', 0.0),
            '北向加速度(m/s²)': data.get('accel_x', 0.0),
            '东向加速度(m/s²)': data.get('accel_y', 0.0),
            '地向加速度(m/s²)': data.get('accel_z', 0.0),
            '经度(°)': data.get('longitude', 0.0),
            '纬度(°)': data.get('latitude', 0.0),
            '北向速度(m/s)': data.get('velocity_x', 0.0),
            '东向速度(m/s)': data.get('velocity_y', 0.0),
            '地向速度(m/s)': data.get('velocity_z', 0.0),
            'GPS北向速度(m/s)': data.get('gps_vx', 0.0),
            'GPS东向速度(m/s)': data.get('gps_vy', 0.0),
            'GPS地向速度(m/s)': data.get('gps_vz', 0.0),
            '空速(m/s)': data.get('velocity_air', 0.0),
            '气压高度(m)': data.get('alt_baro', 0.0),
            'GPS高度(m)': data.get('alt_gps', 0.0),
            '融合高度(m)': data.get('alt', 0.0),
            '温度(°C)': data.get('temperature', 0),
            'GPS时间': data.get('gps_time_str', ''),
            '定位状态': data.get('gps_state_desc', ''),
            'GPS星数': data.get('satellite_num', 0),
            'HDOP': data.get('hdop', 0.0),
            'VDOP': data.get('vdop', 0.0),
            'HDT(°)': data.get('hdt', 0.0),
            'HDG标准差(°)': data.get('hdg_dev', 0.0),
            '工作状态': data.get('ahrs_state', 0),
            '原始数据': data.get('raw_hex', ''),
        }

    def _export_to_excel(self):
        if self.save_enabled or self._exporting() or not self.recording or not self.recording.count:
            return
        session = self.recording
        self.export_btn.setEnabled(False)
        self.save_btn.setEnabled(False)
        self.recording_label.setText(f'正在后台生成 Excel — CSV 保存在 {session.path}')
        self.export_thread = threading.Thread(target=self._export_worker, args=(session,), daemon=False)
        self.export_thread.start()

    def _export_worker(self, session):
        try:
            path = session.export_excel()
            self.export_finished.emit(str(path), '')
        except Exception as error:
            self.export_finished.emit(str(session.path), str(error))

    def _on_export_finished(self, path, error):
        # The signal can arrive just before the worker returns.
        if self.export_thread:
            self.export_thread.join()
        self.export_thread = None
        self.save_btn.setEnabled(True)
        self.export_btn.setEnabled(True)
        if error:
            self.recording_label.setText(f'Excel 导出失败: {error}；CSV 保留在 {path}，可重试导出')
        else:
            self.recording_label.setText(f'Excel 已保存 — {path}')
        if self._close_pending:
            self.close()

    def _default_raw_dir(self):
        if getattr(sys, 'frozen', False):
            save_dir = os.path.join(os.path.expanduser('~'), 'Desktop')
            if not os.path.exists(save_dir):
                save_dir = os.path.expanduser('~')
        else:
            save_dir = os.path.dirname(os.path.abspath(__file__))
        return save_dir

    def toggle_raw_record(self):
        if self._raw_recording:
            path, size = self._finish_raw_record()
            if path:
                QMessageBox.information(
                    self, "保存完成",
                    f"原始数据已保存到:\n{path}\n\n共记录 {size:,} 字节"
                )
            return

        if not self.reader or not self.reader.running:
            QMessageBox.warning(self, "警告", "请先打开串口")
            return

        reader = self.reader
        filename = f"原始数据_{time.strftime('%Y_%m_%d_%H-%M-%S')}.txt"
        path, selected_filter = QFileDialog.getSaveFileName(
            self,
            "保存原始数据",
            os.path.join(self._default_raw_dir(), filename),
            "十六进制文本 (*.txt);;二进制文件 (*.bin)"
        )
        if not path:
            return

        ext = os.path.splitext(path)[1].lower()
        if not ext:
            ext = '.bin' if '二进制' in selected_filter else '.txt'
            path += ext
        mode = 'bin' if ext == '.bin' else 'hex'

        if self.reader is not reader or not reader.running:
            QMessageBox.warning(self, '连接已断开', '请重新连接串口后开始记录')
            return
        result = reader.start_raw_record(path, mode)
        if result is not True:
            QMessageBox.critical(self, "保存失败", f"无法创建文件:\n{result}")
            return

        self._raw_recording = True
        self.raw_save_btn.setText("停止记录原始数据")
        self.raw_count_label.setText("原始数据: 0 B（记录中）")

    def _finish_raw_record(self):
        self._raw_recording = False
        self.raw_save_btn.setText("保存原始数据")
        result = self.reader.stop_raw_record() if self.reader else None
        if self.reader and self.reader.raw_error:
            self.raw_count_label.setText(self.reader.raw_error)
            return None, 0
        if not result:
            self.raw_count_label.setText("原始数据: 未记录")
            return None, 0
        path, size = result
        self.raw_count_label.setText(f"原始数据: {size:,} B")
        return path, size

    def _on_send_file(self):
        self._start_file_transfer(burn=False)

    def _on_burn_file(self):
        if self.protocol_combo.currentIndex() == 0:
            QMessageBox.information(self, '暂不支持', '尚未提供 T02 的固件烧录协议')
            return
        reply = QMessageBox.question(
            self, '旧设备写入', '此流程没有设备应答校验。确认使用旧设备写入协议？',
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if reply == QMessageBox.Yes:
            self._start_file_transfer(burn=True)

    def _start_file_transfer(self, burn):
        if self._send_file_running:
            return
        reader = self.reader
        if not reader or not reader.running or not reader.serial_port or not reader.serial_port.is_open:
            QMessageBox.warning(self, '提示', '请先打开串口并启动接收')
            return
        path, _ = QFileDialog.getOpenFileName(self, '选择文件', '', '所有文件 (*.*)')
        if not path:
            return
        if os.path.getsize(path) == 0:
            QMessageBox.warning(self, '提示', '所选文件为空')
            return
        self._send_file_running = True
        self._send_file_cancelled = False
        self._transfer_cancel.clear()
        self.file_path_label.setText(f'文件: {os.path.basename(path)}')
        self.progress_bar.setValue(0)
        self.send_status_label.setText('准备写入串口')
        self.status_label.setText('文件传输期间暂停接收')
        self.send_file_btn.setEnabled(False)
        self.burn_file_btn.setEnabled(False)
        self.connect_btn.setEnabled(False)
        self.cancel_send_btn.setEnabled(True)
        self.transfer_thread = threading.Thread(
            target=self._transfer_file_worker, args=(path, reader, burn), daemon=False)
        self.transfer_thread.start()

    def _send_file_worker(self, file_path, reader=None):
        self._transfer_file_worker(file_path, reader or self.reader, False)

    def _burn_file_worker(self, file_path, reader=None):
        self._transfer_file_worker(file_path, reader or self.reader, True)

    def _transfer_file_worker(self, file_path, reader, burn):
        sent = 0
        total = 0
        message = ''
        progress = 0
        try:
            reader.stop_receiving()
            total = os.path.getsize(file_path)
            if total == 0:
                raise ValueError('文件为空')
            if burn and not reader.write_data(b'\x79\x79'):
                raise OSError('起始指令写入失败')
            if burn and self._transfer_cancel.wait(0.1):
                message = '已取消'
            else:
                with open(file_path, 'rb') as source:
                    while not self._transfer_cancel.is_set():
                        chunk = source.read(128 if burn else 1024)
                        if not chunk:
                            break
                        packet = b'\x31' + struct.pack('>I', sent) + bytes([len(chunk)]) + chunk if burn else chunk
                        if not reader.write_data(packet):
                            raise OSError('串口已关闭或发生不完整写入')
                        sent += len(chunk)
                        progress = min(99, int(sent * 100 / total))
                        self.transfer_progress.emit(progress)
                        if self._transfer_cancel.wait(0.005 if burn else 0.01):
                            break
                if self._transfer_cancel.is_set():
                    message = f'已取消，已写入 {sent}/{total} 字节'
                elif sent == total:
                    progress = 100
                    message = f'已写入串口 {sent} 字节（设备确认未验证）'
                else:
                    raise OSError('文件在传输期间发生变化')
        except Exception as error:
            message = f'发送失败: {error}（{sent}/{total} 字节）'
        finally:
            self.transfer_finished.emit(reader, message, progress)

    def _on_transfer_finished(self, reader, message, progress):
        self._send_file_running = False
        self._send_file_cancelled = False
        self.send_status_label.setText(message)
        self.send_status_label.setToolTip(message)
        self.progress_bar.setValue(progress)
        self.send_file_btn.setEnabled(True)
        self.burn_file_btn.setEnabled(self.protocol_combo.currentIndex() == 1)
        self.connect_btn.setEnabled(True)
        self.cancel_send_btn.setEnabled(False)
        if self.reader is reader and not self._closing and reader.serial_port and reader.serial_port.is_open:
            try:
                self.last_valid_time = None
                self.connected_time = time.perf_counter()
                self.rx_frequency_start = self.connected_time
                self.rx_frame_count = 0
                reader.start_receiving(callback=self.on_data_received)
                self.status_label.setText('文件传输结束，已恢复接收')
            except Exception as error:
                reader.last_error = f'恢复接收失败: {error}'
        if self._close_pending:
            self.close()

    def _on_cancel_send(self):
        if self._send_file_running:
            self._send_file_cancelled = True
            self._transfer_cancel.set()
            self.send_status_label.setText('正在取消，等待当前写入结束')

    def closeEvent(self, event):
        if self._send_file_running:
            self._closing = True
            self._close_pending = True
            self._on_cancel_send()
            event.ignore()
            return
        if self._exporting():
            self._close_pending = True
            self.recording_label.setText('正在导出 Excel，完成后自动关闭；CSV 已保存')
            event.ignore()
            return
        if self.reader:
            try:
                self.reader.close()
            except RuntimeError as error:
                self.status_label.setText(str(error))
                event.ignore()
                return
            if self.reader.raw_error:
                QMessageBox.warning(self, '原始记录写入异常', self.reader.raw_error)
            self.reader = None
        if not self._stop_recording():
            reply = QMessageBox.question(
                self, '记录写入异常', self.recording_error + '。已有 CSV 已保留，仍要退出？',
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            if reply != QMessageBox.Yes:
                event.ignore()
                return
        self.timer.stop()
        event.accept()


class ModelLoader:
    if getattr(sys, 'frozen', False):
        _SCRIPT_DIR = sys._MEIPASS
    else:
        _SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
    MODEL_PATHS = {
        "歼-20": os.path.join(_SCRIPT_DIR, "3D model", "Dorchester3D.com_歼20", "歼20.obj"),
        "阿帕奇": os.path.join(_SCRIPT_DIR, "3D model", "ImageToStl.com_阿帕奇1", "阿帕奇1.obj"),
        "陆军一号": os.path.join(_SCRIPT_DIR, "3D model", "ImageToStl.com_陆军一号（美国总统专用）", "陆军一号（美国总统专用）.obj"),
        "未来01": os.path.join(_SCRIPT_DIR, "3D model", "ImageToStl.com_未来01", "未来01.obj"),
        "坦克": os.path.join(_SCRIPT_DIR, "3D model", "ImageToStl.com_tank", "tank.obj"),
        "Shahed-136": os.path.join(_SCRIPT_DIR, "3D model", "shahed136", "shahed136.obj"),
        "Lancet-3": os.path.join(_SCRIPT_DIR, "3D model", "lancet3", "lancet3.obj")
    }

    MODEL_CONFIG = {
        "阿帕奇": {
            "axis_swap": [0, 2, 1],
            "flip_x": False,
            "flip_y": False,
            "flip_z": False,
            "rotate_x": 0.0,
            "rotate_y": 0.0,
            "rotate_z": 0.0
        },
        "陆军一号": {
            "axis_swap": [0, 2, 1],
            "flip_x": False,
            "flip_y": False,
            "flip_z": False,
            "rotate_x": 0.0,
            "rotate_y": 0.0,
            "rotate_z": 0.0
        },
        "歼-20": {
            "remove_detached": True,
            "axis_swap": [0, 2, 1],
            "flip_x": False,
            "flip_y": True,
            "flip_z": False,
            "rotate_x": 0.0,
            "rotate_y": 90.0,
            "rotate_z": 0.0
        },
        "未来01": {
            "axis_swap": [0, 2, 1],
            "flip_x": False,
            "flip_y": True,
            "flip_z": False,
            "rotate_x": 0.0,
            "rotate_y": 180.0,
            "rotate_z": 0.0,
            "remove_mirror": True,
            "mirror_axis": 1,
            "mirror_keep_positive": True,
            "mirror_tolerance": 0.05
        },
        "坦克": {
            "axis_swap": [0, 1, 2],
            "flip_x": False,
            "flip_y": False,
            "flip_z": True,
            "rotate_x": 0.0,
            "rotate_y": 180.0,
            "rotate_z": 0.0
        },
        "Shahed-136": {
            "axis_swap": [0, 2, 1],
            "flip_x": False,
            "flip_y": False,
            "flip_z": False,
            "rotate_x": 0.0,
            "rotate_y": 0.0,
            "rotate_z": 0.0
        },
        "Lancet-3": {
            "axis_swap": [0, 2, 1],
            "flip_x": False,
            "flip_y": False,
            "flip_z": False,
            "rotate_x": 0.0,
            "rotate_y": 0.0,
            "rotate_z": 0.0
        }
    }

    @staticmethod
    def load_model(model_name):
        obj_path = ModelLoader.MODEL_PATHS.get(model_name, ModelLoader.MODEL_PATHS["歼-20"])
        config = ModelLoader.MODEL_CONFIG.get(model_name, ModelLoader.MODEL_CONFIG["歼-20"])

        try:
            loaded = trimesh.load(
                obj_path,
                force="scene",
                process=False
            )
            if not isinstance(loaded, trimesh.Scene):
                loaded = trimesh.Scene(loaded)

            return ModelLoader._process_scene(loaded, config, model_name)

        except Exception as e:
            print(f"[ModelLoader] 模型加载失败: {str(e)}")
            import traceback
            traceback.print_exc()
            return None

    @staticmethod
    def _material_color(mesh, default=(0.6, 0.65, 0.75, 1.0)):
        try:
            visual = getattr(mesh, "visual", None)
            material = getattr(visual, "material", None)

            if material is not None:
                for attr in ("diffuse", "baseColorFactor", "main_color"):
                    value = getattr(material, attr, None)

                    if value is None:
                        continue

                    color = np.asarray(
                        value,
                        dtype=np.float32
                    ).reshape(-1)

                    if len(color) >= 3:
                        if np.max(color[:3]) > 1.0:
                            color[:3] /= 255.0

                        alpha = color[3] if len(color) >= 4 else 1.0

                        if alpha > 1.0:
                            alpha /= 255.0

                        return [
                            float(np.clip(color[0], 0, 1)),
                            float(np.clip(color[1], 0, 1)),
                            float(np.clip(color[2], 0, 1)),
                            float(np.clip(alpha, 0, 1))
                        ]

        except Exception as e:
            print(f"[ModelLoader] 读取材质颜色失败: {e}")

        return list(default)

    @staticmethod
    def _texture_image(mesh):
        try:
            visual = getattr(mesh, "visual", None)
            material = getattr(visual, "material", None)
            image = getattr(material, "image", None)

            if image is None:
                return None

            image = image.convert("RGBA")

            max_side = max(image.size)

            if max_side > 2048:
                ratio = 2048.0 / max_side

                new_width = max(1, int(image.width * ratio))
                new_height = max(1, int(image.height * ratio))

                image = image.resize(
                    (new_width, new_height),
                    Image.LANCZOS
                )

            return np.ascontiguousarray(
                np.asarray(image, dtype=np.uint8)
            )

        except Exception as e:
            print(f"[ModelLoader] 读取纹理失败: {e}")
            return None

    @staticmethod
    def _compute_vertex_normals(vertices, faces):
        normals = np.zeros_like(
            vertices,
            dtype=np.float32
        )

        if len(faces) == 0:
            return normals

        triangles = vertices[faces]

        face_normals = np.cross(
            triangles[:, 1] - triangles[:, 0],
            triangles[:, 2] - triangles[:, 0]
        )

        lengths = np.linalg.norm(
            face_normals,
            axis=1,
            keepdims=True
        )

        lengths[lengths < 1e-12] = 1.0
        face_normals /= lengths

        for corner in range(3):
            np.add.at(
                normals,
                faces[:, corner],
                face_normals
            )

        lengths = np.linalg.norm(
            normals,
            axis=1,
            keepdims=True
        )

        lengths[lengths < 1e-12] = 1.0

        return (
            normals / lengths
        ).astype(np.float32)

    @staticmethod
    def _filter_faces_by_axis(
        vertices,
        faces,
        uv=None,
        axis=1,
        keep_positive=True,
        tolerance=0.02
    ):
        if len(vertices) == 0 or len(faces) == 0:
            return vertices, faces, uv

        triangles = vertices[faces]

        face_centers = triangles.mean(axis=1)
        coordinate = face_centers[:, axis]

        if keep_positive:
            face_mask = coordinate >= -tolerance
        else:
            face_mask = coordinate <= tolerance

        filtered_faces = faces[face_mask]

        if len(filtered_faces) == 0:
            print("[ModelLoader] 警告：镜像过滤后没有剩余面片，取消过滤")
            return vertices, faces, uv

        used_vertex_indices = np.unique(filtered_faces.reshape(-1))

        index_map = np.full(
            len(vertices),
            -1,
            dtype=np.int32
        )

        index_map[used_vertex_indices] = np.arange(
            len(used_vertex_indices),
            dtype=np.int32
        )

        new_vertices = vertices[used_vertex_indices]
        new_faces = index_map[filtered_faces]

        new_uv = None

        if uv is not None and len(uv) == len(vertices):
            new_uv = uv[used_vertex_indices]

        return (
            new_vertices.astype(np.float32),
            new_faces.astype(np.int32),
            new_uv
        )

    @staticmethod
    def _compute_face_normals(vertices, faces):
        if len(faces) == 0:
            return np.empty((0, 3), dtype=np.float32)

        a = vertices[faces[:, 0]]
        b = vertices[faces[:, 1]]
        c = vertices[faces[:, 2]]

        normals = np.cross(b - a, c - a)

        lengths = np.linalg.norm(
            normals,
            axis=1,
            keepdims=True
        )

        lengths[lengths < 1e-12] = 1.0

        return (normals / lengths).astype(np.float32)

    @staticmethod
    def _remove_detached_parts(meshes):
        # Weld positions across material seams for component analysis only.
        vertices = np.vstack([mesh.vertices for mesh in meshes])
        offsets = np.cumsum([0] + [len(mesh.vertices) for mesh in meshes])
        faces = np.vstack([mesh.faces + offset for mesh, offset in zip(meshes, offsets)])
        _, inverse = np.unique(np.round(vertices, 6), axis=0, return_inverse=True)
        parent = np.arange(inverse.max() + 1)

        def root(index):
            while parent[index] != index:
                parent[index] = parent[parent[index]]
                index = parent[index]
            return index

        for face in inverse[faces]:
            first = root(face[0])
            for index in face[1:]:
                parent[root(index)] = first
        labels = np.array([root(index) for index in inverse[faces[:, 0]]])
        components = np.unique(labels)
        triangles = vertices[faces]
        areas = np.linalg.norm(np.cross(triangles[:, 1] - triangles[:, 0],
                                       triangles[:, 2] - triangles[:, 0]), axis=1)
        bounds = []
        sizes = []
        for component in components:
            mask = labels == component
            points = triangles[mask].reshape(-1, 3)
            bounds.append([points.min(axis=0), points.max(axis=0)])
            sizes.append(areas[mask].sum())
        bounds = np.asarray(bounds)
        tolerance = np.ptp(vertices, axis=0).max() * 0.001
        kept = {int(np.argmax(sizes))}
        pending = list(kept)
        while pending:
            current = pending.pop()
            overlaps = np.all(bounds[:, 1] + tolerance >= bounds[current, 0], axis=1)
            overlaps &= np.all(bounds[:, 0] - tolerance <= bounds[current, 1], axis=1)
            for index in np.flatnonzero(overlaps):
                if int(index) not in kept:
                    kept.add(int(index))
                    pending.append(int(index))
        face_mask = np.isin(labels, components[sorted(kept)])
        result = []
        offset = 0
        for mesh in meshes:
            mask = face_mask[offset:offset + len(mesh.faces)]
            offset += len(mesh.faces)
            if mask.any():
                mesh.update_faces(mask)
                mesh.remove_unreferenced_vertices()
                result.append(mesh)
        return result

    @staticmethod
    def _process_scene(scene, config, model_name):
        meshes = []
        for node in scene.graph.nodes_geometry:
            transform, geometry_name = scene.graph[node]
            geometry = scene.geometry[geometry_name]
            if isinstance(geometry, trimesh.Trimesh) and len(geometry.faces):
                mesh = geometry.copy()
                mesh.apply_transform(transform)
                mesh.remove_unreferenced_vertices()
                meshes.append(mesh)
        if not meshes:
            raise ValueError('Scene 中没有三角网格')
        if config.get('remove_detached', False):
            meshes = ModelLoader._remove_detached_parts(meshes)
        main_verts = np.vstack([mesh.vertices for mesh in meshes])
        center = (main_verts.max(axis=0) + main_verts.min(axis=0)) * 0.5
        max_dim = np.ptp(main_verts, axis=0).max()
        if max_dim <= 0:
            raise ValueError('模型尺寸为零')
        scale = 6.0 / max_dim
        submeshes = []
        for geom in meshes:
            vertices = geom.vertices.astype(np.float32)
            vertices = (vertices - center) * scale
            vertices = ModelLoader._apply_transforms(vertices, config)
            
            faces = geom.faces.astype(np.int32)
            
            uv = getattr(geom.visual, "uv", None)
            
            if uv is not None:
                uv = np.asarray(
                    uv,
                    dtype=np.float32
                )
            
            if config.get("remove_mirror", False):
                vertices, faces, uv = ModelLoader._filter_faces_by_axis(
                    vertices=vertices,
                    faces=faces,
                    uv=uv,
                    axis=int(config.get("mirror_axis", 1)),
                    keep_positive=bool(
                        config.get(
                            "mirror_keep_positive",
                            True
                        )
                    ),
                    tolerance=float(
                        config.get(
                            "mirror_tolerance",
                            0.05
                        )
                    )
                )
            
            normals = ModelLoader._compute_vertex_normals(
                vertices,
                faces
            )
            
            texture = ModelLoader._texture_image(geom)

            color = ModelLoader._material_color(geom)
            
            submeshes.append({
                'vertices': vertices,
                'faces': faces,
                'normals': normals,
                'uv': uv,
                'texture': texture,
                'color': color
            })

        return {'submeshes': submeshes}

    @staticmethod
    def _generate_ellipsoid(rx, ry, rz, center=(0.0, 0.0, 0.0), segments=24):
        """生成一个椭球体的顶点、面片和法线"""
        theta = np.linspace(0.0, math.pi, segments + 1, dtype=np.float32)
        phi = np.linspace(0.0, 2.0 * math.pi, segments + 1, dtype=np.float32)
        theta, phi = np.meshgrid(theta, phi, indexing="ij")

        x = rx * np.sin(theta) * np.cos(phi) + center[0]
        y = ry * np.sin(theta) * np.sin(phi) + center[1]
        z = rz * np.cos(theta) + center[2]

        vertices = np.stack([x.ravel(), y.ravel(), z.ravel()], axis=1).astype(np.float32)

        faces = []
        for i in range(segments):
            for j in range(segments):
                a = i * (segments + 1) + j
                b = a + 1
                c = a + (segments + 1)
                d = c + 1
                faces.append([a, c, b])
                faces.append([b, c, d])

        faces = np.array(faces, dtype=np.int32)
        normals = ModelLoader._compute_vertex_normals(vertices, faces)
        return vertices, faces, normals

    @staticmethod
    def _generate_cylinder(radius, length, center=(0.0, 0.0, 0.0), segments=24, axis='x'):
        """生成圆柱体（沿指定轴），返回顶点、面片、法线"""
        theta = np.linspace(0.0, 2.0 * math.pi, segments, dtype=np.float32)
        half = length / 2.0
        verts = []
        for t in theta:
            circ_x = radius * math.cos(t)
            circ_y = radius * math.sin(t)
            if axis == 'x':
                verts.append([center[0] - half, center[1] + circ_x, center[2] + circ_y])
                verts.append([center[0] + half, center[1] + circ_x, center[2] + circ_y])
            elif axis == 'y':
                verts.append([center[0] + circ_x, center[1] - half, center[2] + circ_y])
                verts.append([center[0] + circ_x, center[1] + half, center[2] + circ_y])
            else:
                verts.append([center[0] + circ_x, center[1] + circ_y, center[2] - half])
                verts.append([center[0] + circ_x, center[1] + circ_y, center[2] + half])

        vertices = np.array(verts, dtype=np.float32)
        faces = []
        n = len(verts)
        for i in range(0, n, 2):
            a, b = i, i + 1
            c, d = (i + 2) % n, (i + 3) % n
            faces.append([a, c, b])
            faces.append([b, c, d])

        faces = np.array(faces, dtype=np.int32)
        normals = ModelLoader._compute_vertex_normals(vertices, faces)
        return vertices, faces, normals

    @staticmethod
    def _generate_cone(radius, length, center=(0.0, 0.0, 0.0), segments=24, axis='x'):
        """生成圆锥体，返回顶点、面片、法线"""
        theta = np.linspace(0.0, 2.0 * math.pi, segments, dtype=np.float32)
        half = length / 2.0
        verts = []
        for t in theta:
            circ_x = radius * math.cos(t)
            circ_y = radius * math.sin(t)
            if axis == 'x':
                verts.append([center[0] - half, center[1] + circ_x, center[2] + circ_y])
            elif axis == 'y':
                verts.append([center[0] + circ_x, center[1] - half, center[2] + circ_y])
            else:
                verts.append([center[0] + circ_x, center[1] + circ_y, center[2] - half])

        if axis == 'x':
            verts.append([center[0] + half, center[1], center[2]])
        elif axis == 'y':
            verts.append([center[0], center[1] + half, center[2]])
        else:
            verts.append([center[0], center[1], center[2] + half])

        vertices = np.array(verts, dtype=np.float32)
        tip = len(verts) - 1
        faces = []
        for i in range(segments):
            a = i
            b = (i + 1) % segments
            faces.append([a, b, tip])

        faces = np.array(faces, dtype=np.int32)
        normals = ModelLoader._compute_vertex_normals(vertices, faces)
        return vertices, faces, normals

    @staticmethod
    def _generate_box(sx, sy, sz, center=(0.0, 0.0, 0.0)):
        """生成轴对齐长方体，返回顶点、面片、法线"""
        hx, hy, hz = sx / 2.0, sy / 2.0, sz / 2.0
        cx, cy, cz = center
        vertices = np.array([
            [cx - hx, cy - hy, cz - hz], [cx + hx, cy - hy, cz - hz],
            [cx + hx, cy + hy, cz - hz], [cx - hx, cy + hy, cz - hz],
            [cx - hx, cy - hy, cz + hz], [cx + hx, cy - hy, cz + hz],
            [cx + hx, cy + hy, cz + hz], [cx - hx, cy + hy, cz + hz],
        ], dtype=np.float32)

        faces = np.array([
            [0, 3, 2], [0, 2, 1], [4, 5, 6], [4, 6, 7],
            [0, 1, 5], [0, 5, 4], [2, 3, 7], [2, 7, 6],
            [0, 4, 7], [0, 7, 3], [1, 2, 6], [1, 6, 5],
        ], dtype=np.int32)

        normals = ModelLoader._compute_vertex_normals(vertices, faces)
        return vertices, faces, normals

    @staticmethod
    def _create_ellipsoid_airplane():
        """用多个椭球体拼接出一架简易飞机（还原最初的椭球体版本）"""
        parts = [
            # (rx, ry, rz, center, color)
            (2.2, 0.55, 0.55, (0.0, 0.0, 0.0), (0.78, 0.80, 0.85, 1.0)),      # 机身
            (0.45, 0.4, 0.4, (2.3, 0.0, 0.0), (0.70, 0.72, 0.78, 1.0)),       # 机头
            (0.30, 0.30, 0.30, (-2.2, 0.0, 0.0), (0.60, 0.62, 0.68, 1.0)),    # 机尾锥
            (0.9, 0.06, 0.7, (0.2, 0.0, 1.1), (0.75, 0.77, 0.82, 1.0)),       # 右机翼
            (0.9, 0.06, 0.7, (0.2, 0.0, -1.1), (0.75, 0.77, 0.82, 1.0)),      # 左机翼
            (0.5, 0.05, 0.35, (-1.8, 0.0, 0.7), (0.70, 0.72, 0.78, 1.0)),     # 右水平尾翼
            (0.5, 0.05, 0.35, (-1.8, 0.0, -0.7), (0.70, 0.72, 0.78, 1.0)),    # 左水平尾翼
            (0.25, 0.55, 0.08, (-1.6, 0.45, 0.0), (0.68, 0.70, 0.76, 1.0)),   # 垂直尾翼
            (0.35, 0.22, 0.22, (0.3, -0.15, 0.85), (0.45, 0.47, 0.52, 1.0)),  # 右引擎
            (0.35, 0.22, 0.22, (0.3, -0.15, -0.85), (0.45, 0.47, 0.52, 1.0)), # 左引擎
            (0.30, 0.18, 0.18, (1.2, 0.18, 0.0), (0.30, 0.45, 0.60, 1.0)),    # 驾驶舱
        ]

        submeshes = []
        for rx, ry, rz, center, color in parts:
            vertices, faces, normals = ModelLoader._generate_ellipsoid(
                rx, ry, rz, center=center
            )
            submeshes.append({
                'vertices': vertices,
                'faces': faces,
                'normals': normals,
                'uv': None,
                'texture': None,
                'color': list(color)
            })

        print(f"[ModelLoader] 椭球体飞机生成完成: {len(submeshes)} 个子网格")
        return {'submeshes': submeshes}

    @staticmethod
    def _create_handcrafted_airplane():
        """用数学公式手搓的精细飞机模型（还原椭球体之后、加载OBJ之前的阶段）"""
        submeshes = []

        def _add(verts, faces, normals, color):
            submeshes.append({
                'vertices': verts,
                'faces': faces,
                'normals': normals,
                'uv': None,
                'texture': None,
                'color': list(color)
            })

        # 机身（长椭球）
        v, f, n = ModelLoader._generate_ellipsoid(2.0, 0.45, 0.45, (0.0, 0.0, 0.0))
        _add(v, f, n, (0.82, 0.84, 0.88, 1.0))

        # 机头圆锥
        v, f, n = ModelLoader._generate_cone(0.45, 1.2, (2.5, 0.0, 0.0), axis='x')
        _add(v, f, n, (0.75, 0.77, 0.82, 1.0))

        # 驾驶舱（小椭球）
        v, f, n = ModelLoader._generate_ellipsoid(0.35, 0.25, 0.30, (0.8, 0.25, 0.0))
        _add(v, f, n, (0.30, 0.45, 0.60, 1.0))

        # 主机翼（左右两个扁平长方块）
        v, f, n = ModelLoader._generate_box(1.6, 0.05, 1.0, (0.2, 0.0, 1.1))
        _add(v, f, n, (0.80, 0.82, 0.86, 1.0))
        v, f, n = ModelLoader._generate_box(1.6, 0.05, 1.0, (0.2, 0.0, -1.1))
        _add(v, f, n, (0.80, 0.82, 0.86, 1.0))

        # 翼尖小翼
        v, f, n = ModelLoader._generate_box(0.15, 0.25, 0.12, (0.9, 0.12, 1.6))
        _add(v, f, n, (0.70, 0.72, 0.78, 1.0))
        v, f, n = ModelLoader._generate_box(0.15, 0.25, 0.12, (0.9, 0.12, -1.6))
        _add(v, f, n, (0.70, 0.72, 0.78, 1.0))

        # 双引擎（圆柱）
        v, f, n = ModelLoader._generate_cylinder(0.22, 1.0, (0.4, -0.15, 0.85), axis='x')
        _add(v, f, n, (0.45, 0.47, 0.52, 1.0))
        v, f, n = ModelLoader._generate_cylinder(0.22, 1.0, (0.4, -0.15, -0.85), axis='x')
        _add(v, f, n, (0.45, 0.47, 0.52, 1.0))

        # 引擎喷嘴（圆锥）
        v, f, n = ModelLoader._generate_cone(0.22, 0.3, (-0.1, -0.15, 0.85), axis='x')
        _add(v, f, n, (0.90, 0.55, 0.20, 1.0))
        v, f, n = ModelLoader._generate_cone(0.22, 0.3, (-0.1, -0.15, -0.85), axis='x')
        _add(v, f, n, (0.90, 0.55, 0.20, 1.0))

        # 水平尾翼
        v, f, n = ModelLoader._generate_box(0.8, 0.04, 0.6, (-1.8, 0.0, 0.65))
        _add(v, f, n, (0.78, 0.80, 0.84, 1.0))
        v, f, n = ModelLoader._generate_box(0.8, 0.04, 0.6, (-1.8, 0.0, -0.65))
        _add(v, f, n, (0.78, 0.80, 0.84, 1.0))

        # 垂直尾翼
        v, f, n = ModelLoader._generate_box(0.5, 0.55, 0.05, (-1.6, 0.35, 0.0))
        _add(v, f, n, (0.72, 0.74, 0.78, 1.0))

        # 前起落架
        v, f, n = ModelLoader._generate_cylinder(0.04, 0.35, (1.6, -0.35, 0.0), axis='y')
        _add(v, f, n, (0.35, 0.35, 0.38, 1.0))

        # 主起落架（左右）
        v, f, n = ModelLoader._generate_cylinder(0.05, 0.45, (0.2, -0.40, 0.7), axis='y')
        _add(v, f, n, (0.35, 0.35, 0.38, 1.0))
        v, f, n = ModelLoader._generate_cylinder(0.05, 0.45, (0.2, -0.40, -0.7), axis='y')
        _add(v, f, n, (0.35, 0.35, 0.38, 1.0))

        print(f"[ModelLoader] 手搓飞机生成完成: {len(submeshes)} 个子网格")
        return {'submeshes': submeshes}

    @staticmethod
    def _apply_transforms(vertices, config):
        if config.get("axis_swap"):
            vertices = vertices[:, config["axis_swap"]]

        if config.get("flip_x", False):
            vertices[:, 0] *= -1.0

        if config.get("flip_y", False):
            vertices[:, 1] *= -1.0

        if config.get("flip_z", False):
            vertices[:, 2] *= -1.0

        rotate_x = math.radians(float(config.get("rotate_x", 0.0)))
        rotate_y = math.radians(float(config.get("rotate_y", 0.0)))
        rotate_z = math.radians(float(config.get("rotate_z", 0.0)))

        if abs(rotate_x) > 1e-12:
            c = math.cos(rotate_x)
            s = math.sin(rotate_x)
            matrix_x = np.array([
                [1, 0, 0],
                [0, c, -s],
                [0, s, c]
            ], dtype=np.float32)
            vertices = vertices @ matrix_x.T

        if abs(rotate_y) > 1e-12:
            c = math.cos(rotate_y)
            s = math.sin(rotate_y)
            matrix_y = np.array([
                [c, 0, s],
                [0, 1, 0],
                [-s, 0, c]
            ], dtype=np.float32)
            vertices = vertices @ matrix_y.T

        if abs(rotate_z) > 1e-12:
            c = math.cos(rotate_z)
            s = math.sin(rotate_z)
            matrix_z = np.array([
                [c, -s, 0],
                [s, c, 0],
                [0, 0, 1]
            ], dtype=np.float32)
            vertices = vertices @ matrix_z.T

        return vertices.astype(np.float32)


class PlaneGLWidget(QOpenGLWidget):
    def __init__(self):
        super().__init__()
        
        self.pitch = 0.0
        self.roll = 0.0
        self.yaw = 0.0
        self.pitch_offset = 0.0
        self.roll_offset = 0.0
        self.yaw_offset = 0.0
        
        self.camera_pitch = 0.0
        self.camera_yaw = 0.0
        self.camera_distance = 8.0
        self.is_dragging = False
        self.last_pos = None
        
        self.model_data = None
        self.current_model = "歼-20"
        self.texture_cache = {}
        
        self._load_model(self.current_model)

    def load_model_by_name(self, model_name):
        if model_name != self.current_model:
            self.makeCurrent()

            for texture_id in self.texture_cache.values():
                try:
                    glDeleteTextures([texture_id])
                except Exception:
                    pass

            self.texture_cache.clear()
            self.doneCurrent()

            self.current_model = model_name
            self.model_data = ModelLoader.load_model(model_name)

            self.camera_pitch = 0.0
            self.camera_yaw = 0.0
            self.camera_distance = 8.0

            self.update()

    def reset_view(self):
        self.camera_pitch = 0.0
        self.camera_yaw = 0.0
        self.camera_distance = 8.0
        self.update()

    def _load_model(self, model_name):
        self.model_data = ModelLoader.load_model(model_name)

    def update_attitude(self, pitch, roll, yaw):
        self.pitch = float(pitch)
        self.roll = float(roll)

        yaw_val = float(yaw)
        yaw_error = (yaw_val - self.yaw + 180.0) % 360.0 - 180.0
        self.yaw += yaw_error
        self.yaw %= 360.0

        self.update()

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.is_dragging = True
            self.last_pos = event.pos()

    def mouseMoveEvent(self, event):
        if self.is_dragging and self.last_pos:
            dx = event.x() - self.last_pos.x()
            dy = event.y() - self.last_pos.y()
            self.camera_yaw += dx * 0.5
            self.camera_pitch += dy * 0.5
            self.camera_pitch = max(-85, min(85, self.camera_pitch))
            self.last_pos = event.pos()
            self.update()

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.is_dragging = False
            self.last_pos = None

    def wheelEvent(self, event):
        delta = event.angleDelta().y()
        self.camera_distance = max(6.0, min(15.0, self.camera_distance - delta * 0.005))
        self.update()

    def initializeGL(self):
        glEnable(GL_DEPTH_TEST)
        glDepthFunc(GL_LESS)
        
        glEnable(GL_LIGHTING)
        glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE)
        glEnable(GL_COLOR_MATERIAL)
        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)
        
        glEnable(GL_NORMALIZE)
        
        glLightfv(GL_LIGHT0, GL_POSITION, [10.0, 15.0, 10.0, 1.0])
        glLightfv(GL_LIGHT0, GL_DIFFUSE, [1.0, 1.0, 1.0, 1.0])
        glLightfv(GL_LIGHT0, GL_SPECULAR, [0.4, 0.4, 0.4, 1.0])
        glLightfv(GL_LIGHT0, GL_AMBIENT, [0.2, 0.2, 0.2, 1.0])
        glEnable(GL_LIGHT0)
        
        glLightfv(GL_LIGHT1, GL_POSITION, [-10.0, 5.0, -10.0, 1.0])
        glLightfv(GL_LIGHT1, GL_DIFFUSE, [0.5, 0.5, 0.5, 1.0])
        glLightfv(GL_LIGHT1, GL_AMBIENT, [0.1, 0.1, 0.1, 1.0])
        glEnable(GL_LIGHT1)
        
        glClearColor(0.74, 0.87, 0.97, 1.0)
        
        glShadeModel(GL_SMOOTH)

    def resizeGL(self, width, height):
        glViewport(0, 0, width, height)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(60.0, float(width)/max(height, 1), 0.1, 500.0)
        glMatrixMode(GL_MODELVIEW)

    def _draw_reference_grid(self):
        glDisable(GL_LIGHTING)
        glColor3f(0.66, 0.71, 0.77)

        glBegin(GL_LINES)
        for i in range(-20, 21):
            glVertex3f(i, -2.0, -20)
            glVertex3f(i, -2.0, 20)
            glVertex3f(-20, -2.0, i)
            glVertex3f(20, -2.0, i)
        glEnd()

        glColor3f(0.55, 0.61, 0.69)
        glBegin(GL_LINES)
        for i in range(-20, 21, 5):
            glVertex3f(i, -1.95, -20)
            glVertex3f(i, -1.95, 20)
            glVertex3f(-20, -1.95, i)
            glVertex3f(20, -1.95, i)
        glEnd()

        glColor3f(1.0, 0.3, 0.3)
        glBegin(GL_LINES)
        glVertex3f(0, -1.9, 0)
        glVertex3f(0, -1.9, 5)
        glEnd()

        glColor3f(0.3, 1.0, 0.3)
        glBegin(GL_LINES)
        glVertex3f(0, -1.9, 0)
        glVertex3f(0, 3.0, 0)
        glEnd()

        glColor3f(0.3, 0.5, 1.0)
        glBegin(GL_LINES)
        glVertex3f(0, -1.9, 0)
        glVertex3f(5, -1.9, 0)
        glEnd()

        self._draw_axis_arrow(0, -1.85, 5, 0, 0, 1, [1.0, 0.3, 0.3])
        self._draw_axis_arrow(0, 3.1, 0, -90, 1, 0, [0.3, 1.0, 0.3])
        self._draw_axis_arrow(5, -1.85, 0, 90, 0, 1, [0.3, 0.5, 1.0])
        glEnable(GL_LIGHTING)

    def _draw_axis_arrow(self, x, y, z, rot_angle, rot_x, rot_y, color):
        glColor3f(color[0], color[1], color[2])
        glPushMatrix()
        glTranslatef(x, y, z)
        glRotatef(rot_angle, rot_x, rot_y, 0)
        glScalef(0.15, 0.15, 0.15)
        
        quad = gluNewQuadric()
        gluQuadricNormals(quad, GLU_SMOOTH)
        gluCylinder(quad, 0.1, 0.1, 0.5, 16, 4)
        glTranslatef(0, 0, 0.5)
        gluCylinder(quad, 0.25, 0, 0.4, 16, 4)
        gluDeleteQuadric(quad)
        
        glPopMatrix()

    def _get_texture_id(self, submesh_index, image):
        if image is None:
            return None

        if submesh_index in self.texture_cache:
            return self.texture_cache[submesh_index]

        height, width = image.shape[:2]

        pixels = np.ascontiguousarray(np.flipud(image))

        texture_id = glGenTextures(1)

        glBindTexture(GL_TEXTURE_2D, texture_id)

        glPixelStorei(GL_UNPACK_ALIGNMENT, 1)

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT)

        glTexImage2D(
            GL_TEXTURE_2D,
            0,
            GL_RGBA,
            width,
            height,
            0,
            GL_RGBA,
            GL_UNSIGNED_BYTE,
            pixels
        )

        glGenerateMipmap(GL_TEXTURE_2D)

        glBindTexture(GL_TEXTURE_2D, 0)

        self.texture_cache[submesh_index] = texture_id

        return texture_id

    def _draw_model(self):
        if self.model_data is None:
            self._draw_default_plane()
            return

        glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, [0.12, 0.12, 0.12, 1.0])
        glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 12.0)

        submeshes = self.model_data.get("submeshes", [])

        for submesh_index, submesh in enumerate(submeshes):
            vertices = submesh["vertices"]
            faces = submesh["faces"]
            normals = submesh["normals"]

            uv = submesh.get("uv")
            image = submesh.get("texture")

            color = submesh.get("color", [0.6, 0.65, 0.75, 1.0])

            textured = (image is not None and uv is not None)

            if textured:
                texture_id = self._get_texture_id(submesh_index, image)
                glEnable(GL_TEXTURE_2D)
                glBindTexture(GL_TEXTURE_2D, texture_id)
                glColor4f(1.0, 1.0, 1.0, color[3] if len(color) > 3 else 1.0)
            else:
                glDisable(GL_TEXTURE_2D)
                glColor4f(color[0], color[1], color[2], color[3] if len(color) > 3 else 1.0)

            glBegin(GL_TRIANGLES)

            for face in faces:
                for vertex_index in face:
                    normal = normals[vertex_index]
                    glNormal3f(float(normal[0]), float(normal[1]), float(normal[2]))

                    if textured:
                        tex_coord = uv[vertex_index]
                        glTexCoord2f(float(tex_coord[0]), float(tex_coord[1]))

                    vertex = vertices[vertex_index]
                    glVertex3f(float(vertex[0]), float(vertex[1]), float(vertex[2]))

            glEnd()

            if textured:
                glBindTexture(GL_TEXTURE_2D, 0)
                glDisable(GL_TEXTURE_2D)

    def _draw_default_plane(self):
        glColor3f(0.6, 0.65, 0.75)
        glMaterialfv(GL_FRONT, GL_SPECULAR, [0.3, 0.3, 0.3, 1.0])
        
        glPushMatrix()
        glScalef(2.0, 0.5, 4.0)
        
        quad = gluNewQuadric()
        gluQuadricNormals(quad, GLU_SMOOTH)
        gluSphere(quad, 0.8, 32, 16)
        gluDeleteQuadric(quad)
        
        glPopMatrix()

    def paintGL(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()

        cam_x = self.camera_distance * math.sin(math.radians(self.camera_yaw)) * math.cos(math.radians(self.camera_pitch))
        cam_y = self.camera_distance * math.sin(math.radians(self.camera_pitch))
        cam_z = self.camera_distance * math.cos(math.radians(self.camera_yaw)) * math.cos(math.radians(self.camera_pitch))

        gluLookAt(cam_x, cam_y, cam_z, 0, 0, 0, 0, 1, 0)

        self._draw_reference_grid()
        
        glPushMatrix()
        glRotatef(-self.yaw, 0, 1, 0)
        glRotatef(-self.pitch, 1, 0, 0)
        glRotatef(-self.roll, 0, 0, 1)
        self._draw_model()
        glPopMatrix()

        painter = QPainter(self)
        painter.setPen(QColor(25, 45, 75))
        painter.setFont(QFont("Arial", 9))
        painter.drawText(10, 20, f"俯仰角: {self.pitch:.1f}°")
        painter.drawText(10, 40, f"滚转角: {self.roll:.1f}°")
        painter.drawText(10, 60, f"航向角: {self.yaw:.1f}°")
        painter.end()


def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
