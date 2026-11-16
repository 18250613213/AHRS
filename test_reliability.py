import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

from main import QApplication, MainWindow, SerialBinaryReader
from t02v3 import T02V3Parser
from test_t02v3 import SAMPLE


class ReliabilityTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        self.window = MainWindow()
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.addCleanup(self.window.close)
        self.window._default_raw_dir = lambda: self.directory.name

    def test_receive_failure_marks_reader_stopped(self):
        reader = SerialBinaryReader()
        reader.running = True
        reader.serial_port = Mock(is_open=True)
        type(reader.serial_port).in_waiting = property(
            lambda self: (_ for _ in ()).throw(OSError('device removed')))
        reader.data_callback = Mock()
        reader._receive_loop()
        self.assertFalse(reader.running)
        self.assertIn('device removed', reader.last_error)

    def test_raw_recording_write_failure_is_visible(self):
        reader = SerialBinaryReader()
        reader.raw_file = Mock()
        reader.raw_file.write.side_effect = OSError('disk full')
        reader.raw_mode = 'bin'
        reader._write_raw(b'bytes')
        self.assertIn('disk full', reader.raw_error)

    def test_valid_data_timeout_zeroes_frequency_and_marks_values_stale(self):
        w = self.window
        w.reader = Mock(running=True, last_error='', raw_error='')
        data = T02V3Parser.parse_frame(SAMPLE)
        w.on_data_received(data)
        w.rx_frequency = 200.0
        w.last_valid_time = time.perf_counter() - 3
        w._process_data()
        self.assertEqual(w.rx_frequency, 0)
        self.assertIn('超时', w.status_label.text())
        self.assertIn('过期', w.update_label.text())
        w.reader = None

    def test_close_persists_recording_without_export(self):
        w = self.window
        w.toggle_save()
        w.on_data_received(T02V3Parser.parse_frame(SAMPLE))
        event = Mock()
        w.closeEvent(event)
        self.assertTrue(event.accept.called)
        files = list(Path(self.directory.name).glob('*.csv'))
        self.assertEqual(len(files), 1)
        self.assertIn('$BDFPDL', files[0].read_text(encoding='utf-8-sig'))

    def test_failed_send_never_reports_success(self):
        w = self.window
        reader = Mock()
        reader.serial_port.is_open = True
        reader.write_data.return_value = False
        w.reader = reader
        payload = Path(self.directory.name) / 'payload.bin'
        payload.write_bytes(b'payload')
        w._send_file_worker(str(payload), reader)
        self.app.processEvents()
        self.assertNotEqual(w.progress_bar.value(), 100)
        self.assertIn('失败', w.send_status_label.text())
        w.reader = None

    def test_t02_burning_is_disabled_and_display_is_direct(self):
        w = self.window
        self.assertFalse(w.burn_file_btn.isEnabled())
        first = T02V3Parser.parse_frame(SAMPLE)
        w._update_ui(first)
        second = dict(first, pitch=10.0)
        w._update_ui(second)
        self.assertEqual(w.plane_widget.pitch, 10.0)
        self.assertEqual(w.value_labels['alt'].text(), '50.664')


if __name__ == '__main__':
    unittest.main()
