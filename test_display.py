"""Integration checks for protocol switching, displayed units and export fields."""
import os
import unittest

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from main import QApplication, MainWindow, BinaryFrameParser
from t02 import T02Parser
from test_t02 import SAMPLE


class DisplayTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])
        cls.window = MainWindow()

    @classmethod
    def tearDownClass(cls):
        cls.window.close()

    def test_t02_display_and_export_preserve_axes(self):
        window = self.window
        window.protocol_combo.setCurrentIndex(0)
        data = T02Parser.parse_frame(SAMPLE)
        window._update_ui(data)
        self.assertEqual(window.value_labels['velocity_up'].text(), '-0.0002')
        self.assertEqual(window.value_labels['latitude'].text(), '30.1234567')
        self.assertEqual(window.value_labels['system_state'].text(), '48')
        record = window._build_save_record(data)
        self.assertEqual(record['天向速度(m/s)'], -.0002)
        self.assertEqual(record['X轴角速率(°/s)'], .029)
        self.assertEqual(len(record), 25)  # 23 fields, protocol, original sentence

    def test_ascii_protocol_and_preview(self):
        window = self.window
        self.assertEqual(window.protocol_combo.count(), 1)
        window._update_ui(T02Parser.parse_frame(SAMPLE))
        self.assertEqual(window.raw_preview.toPlainText(), SAMPLE.decode('ascii').strip())

    def test_yaw_filter_crosses_north_without_half_turn(self):
        window = self.window
        window._init_filter_queues()
        window._apply_filter('yaw', 359.9)
        value = window._apply_filter('yaw', .1)
        self.assertAlmostEqual(value % 360, 0)


if __name__ == '__main__':
    unittest.main()
