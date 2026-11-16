"""T02V3 $BDFPDL navigation sentences, angles in degrees and ENU velocity."""
import math


class T02V3Parser:
    FRAME_HEADER = b'$BDFPDL,'
    MAX_FRAME_LENGTH = 1024
    FIELDS = (
        ('gps_week', int), ('gps_seconds', float),
        ('yaw', float), ('pitch', float), ('roll', float),
        ('latitude', float), ('longitude', float), ('alt', float),
        ('velocity_east', float), ('velocity_north', float), ('velocity_up', float),
        ('gyro_x', float), ('gyro_y', float), ('gyro_z', float),
        ('body_accel_x', float), ('body_accel_y', float), ('body_accel_z', float),
        ('satellite_num', int), ('satellite_num_2', int),
        ('position_type', int), ('heading_type', int),
        ('system_state', int), ('flight_state', int),
    )
    DISPLAY_GROUPS = (
        [('俯仰角(°)', 'pitch'), ('滚转角(°)', 'roll'), ('航向角(°)', 'yaw'),
         ('X轴角速率(°/s)', 'gyro_x'), ('Y轴角速率(°/s)', 'gyro_y'), ('Z轴角速率(°/s)', 'gyro_z'),
         ('X轴加速度(m/s²)', 'body_accel_x'), ('Y轴加速度(m/s²)', 'body_accel_y'),
         ('Z轴加速度(m/s²)', 'body_accel_z')],
        [('经度(°)', 'longitude'), ('纬度(°)', 'latitude'), ('高度(m)', 'alt'),
         ('东向速度(m/s)', 'velocity_east'), ('北向速度(m/s)', 'velocity_north'),
         ('天向速度(m/s)', 'velocity_up')],
        [('协议', 'protocol'), ('GPS周数', 'gps_week'), ('GPS周秒(s)', 'gps_seconds'),
         ('天线1卫星数', 'satellite_num'), ('天线2卫星数', 'satellite_num_2'),
         ('定位类型', 'position_type'), ('定向类型', 'heading_type'),
         ('系统状态字', 'system_state'), ('飞控通信状态字', 'flight_state')],
    )

    @staticmethod
    def find_frames(buffer):
        frames = []
        while buffer:
            start = buffer.find(b'$')
            if start < 0:
                return frames, b''
            buffer = buffer[start:]
            end = buffer.find(b'\n')
            next_start = buffer.find(b'$', 1)
            if next_start >= 0 and (end < 0 or next_start < end):
                buffer = buffer[next_start:]
                continue
            if end < 0:
                return frames, buffer if len(buffer) <= T02V3Parser.MAX_FRAME_LENGTH else b''
            line, buffer = buffer[:end + 1], buffer[end + 1:]
            if line.startswith(T02V3Parser.FRAME_HEADER):
                frames.append(line)
        return frames, buffer

    @staticmethod
    def parse_frame(frame):
        raw = frame.decode('ascii', errors='replace').strip()
        try:
            if len(frame) > T02V3Parser.MAX_FRAME_LENGTH:
                raise ValueError('报文过长')
            text = frame.decode('ascii').strip()
            body, checksum_text = text[1:].split('*')
            if not text.startswith('$BDFPDL,') or len(checksum_text) != 2:
                raise ValueError('报文头或校验字段错误')
            checksum = 0
            for byte in body.encode('ascii'):
                checksum ^= byte
            if checksum != int(checksum_text, 16):
                raise ValueError('异或校验和错误')
            fields = body.split(',')[1:]
            if len(fields) != len(T02V3Parser.FIELDS):
                raise ValueError(f'需要23个数据字段，收到{len(fields)}个')
            data = {key: convert(value) for (key, convert), value in zip(T02V3Parser.FIELDS, fields)}
            if not all(math.isfinite(value) for value in data.values()):
                raise ValueError('包含非有限数值')
            for key, low, high in [('pitch', -90, 90), ('roll', -180, 180),
                                   ('yaw', 0, 360), ('latitude', -90, 90),
                                   ('longitude', -180, 180), ('gps_seconds', 0, 604800)]:
                if not low <= data[key] <= high:
                    raise ValueError(f'{key}超出范围')
            for key, convert in T02V3Parser.FIELDS:
                if convert is int and data[key] < 0:
                    raise ValueError(f'{key}不能为负数')
            data.update(protocol='T02V3', frame_header='$BDFPDL', raw_text=raw,
                        raw_hex=' '.join(f'{byte:02X}' for byte in frame))
            return data
        except (ValueError, UnicodeError, OverflowError) as error:
            return {'error': f'T02V3解析失败: {error}', 'raw_text': raw}
