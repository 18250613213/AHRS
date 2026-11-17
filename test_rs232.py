import unittest
import struct
import os
import time
from unittest.mock import patch
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
from rs232 import MixedParser


def ubx_packet():
    payload = bytearray(92)
    struct.pack_into('<i', payload, 24, 1201234567)
    struct.pack_into('<i', payload, 28, 301234567)
    body = b'\x01\x07' + struct.pack('<H', len(payload)) + payload
    a = b = 0
    for value in body:
        a = (a + value) & 255
        b = (b + a) & 255
    return b'\xb5\x62' + body + bytes([a, b])


NMEA = b'$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n'


class MixedTests(unittest.TestCase):
    def test_corrupt_length_recovers_after_timeout(self):
        parser = MixedParser()
        parser.feed(b'\xb5\x62\x01\x07\xff\xff' + NMEA)
        parser.pending_since = time.monotonic() - 3
        result = parser.feed(b'')
        self.assertEqual(result[0][1].identity, 'GPGGA')

    def test_mode_commands_and_baud_change(self):
        from rs232 import RS232Reader
        writes = []
        class Port:
            baudrate = 230400
            in_waiting = 0
            def __enter__(self): return self
            def __exit__(self, *args): pass
            def reset_input_buffer(self): pass
            def flush(self): pass
            def write(self, data):
                writes.append((self.baudrate, data))
                return len(data)
            def read(self, size):
                if writes and writes[-1][1] == b'uart 1\r':
                    self.assert_baud()
                    reader.stop_event.set()
                    return ubx_packet()
                if writes: return b'Debug mode=>\r\n'
                return b''
            def assert_baud(self):
                if self.baudrate != 230400: raise AssertionError('incorrect receive baud')
        reader = RS232Reader('COM15', serial_factory=lambda *args, **kwargs: Port())
        ticks = iter(i * .5 for i in range(100))
        with patch('rs232.time.monotonic', side_effect=lambda: next(ticks)):
            reader._run()
        self.assertEqual(writes, [(460800, b'exit\r'), (460800, b'debug\r'), (460800, b'uart 1\r')])
        self.assertEqual(reader.counts['NAV-PVT'], 1)

    def test_silent_modes_can_exit(self):
        from rs232 import RS232Reader
        from unittest.mock import Mock
        for uart in (0, 1, 2):
            with self.subTest(uart=uart):
                port = Mock()
                port.in_waiting = 0
                port.read.return_value = b'Debug mode=>\r\n'
                port.write.side_effect = len
                reader = RS232Reader('COM15', uart=uart)
                reader._enter_uart(port, uart)
                self.assertEqual(reader.active_uart, uart)
                self.assertEqual(port.baudrate, 230400)
                self.assertEqual(reader.counts, {})
                self.assertIn('可能无输出', reader.state)
                self.assertEqual(port.write.call_args.args[0], f'uart {uart}\r'.encode())
                reader._exit_uart(port)
                self.assertIsNone(reader.active_uart)
                self.assertEqual(port.baudrate, 460800)
                self.assertEqual(port.write.call_args.args[0], b'exit\r')

    def test_panel_displays_pvt_fields(self):
        from PyQt5.QtWidgets import QApplication
        from rs232 import RS232Panel, RS232Reader
        app = QApplication.instance() or QApplication([])
        panel = RS232Panel()
        panel.reader = RS232Reader('COM15')
        panel.reader._consume(MixedParser(), ubx_packet())
        panel.refresh()
        self.assertIn('120.1234567', panel.summary.text())
        self.assertIn('lon = 120.1234567', panel.details.toPlainText())
        panel.timer.stop()
        panel.close()

    def test_every_split_mixed_stream(self):
        stream = b'console text\r\n' + ubx_packet() + NMEA
        for index in range(len(stream)):
            parser = MixedParser()
            messages = parser.feed(stream[:index]) + parser.feed(stream[index:])
            self.assertEqual([m[1].identity for m in messages], ['NAV-PVT', 'GPGGA'])
            self.assertAlmostEqual(messages[0][1].lon, 120.1234567)
            self.assertAlmostEqual(messages[1][1].lat, 48.1173)

    def test_bad_checksum_recovers(self):
        packet = bytearray(ubx_packet())
        packet[-1] ^= 1
        parser = MixedParser()
        messages = parser.feed(bytes(packet) + NMEA)
        self.assertEqual(len(messages), 1)
        self.assertEqual(parser.errors, 1)

    def test_buffer_is_bounded(self):
        parser = MixedParser()
        parser.feed(b'$' + b'x' * 10000)
        self.assertLessEqual(len(parser.buffer), 4096)
        self.assertEqual(len(parser.feed(NMEA)), 1)


if __name__ == '__main__':
    unittest.main()
