import tempfile
import unittest
from pathlib import Path

from diagnostics import Diagnostics, review_rows
from recording import RecordingSession


class DiagnosticTests(unittest.TestCase):
    def test_motion_crosses_dateline_then_jumps_in_height(self):
        analyzer = Diagnostics()
        for i, (lon, alt) in enumerate([(179.999, 1), (-180, 2), (-179.999, 30)]):
            result = analyzer.update(dict(longitude=lon, latitude=0, alt=alt, system_state=3),
                                     i, "运动", (1, 1, 1))
        self.assertEqual(result["经度判定"], "平滑")
        self.assertEqual(result["高度判定"], "跳变")
        self.assertEqual(result["系统状态字统计"], "3×3")

    def test_review_old_and_new_recordings(self):
        with tempfile.TemporaryDirectory() as directory:
            original = RecordingSession(directory)
            for stamp, alt in [(0, 10), (1, 20), (5, 20)]:
                original.append({"GPS周数": 100, "GPS周秒(s)": stamp, "经度(°)": 110,
                                 "纬度(°)": 30, "高度(m)": alt, "定位类型": 4})
            original.close()
            expected = list(review_rows(original.path, "静置", (2, 2, 2)))
            self.assertEqual([r["高度判定"] for r in expected], ["待判定", "跳变", "待判定"])
            self.assertEqual(expected[-1]["定位类型统计"], "4×3")
            self.assertEqual(list(review_rows(original.export_excel(), "静置", (2, 2, 2)))[1]["高度偏差(m)"], 10)
            session = RecordingSession(Path(directory) / "test")
            for row in expected:
                session.append(row)
            session.close()
            for path in (session.path, session.export_excel()):
                replay = list(review_rows(path, "运动", (100, 100, 100)))
                self.assertEqual(replay[1]["高度判定"], "跳变")
                self.assertEqual(replay[1]["测验场景"], "静置")
