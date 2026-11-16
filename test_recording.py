import csv
import importlib.util
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch


class RecordingTests(unittest.TestCase):
    def setUp(self):
        self.assertIsNotNone(importlib.util.find_spec('recording'))
        from recording import RecordingSession
        self.Session = RecordingSession
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)

    def test_close_preserves_all_rows_without_excel(self):
        session = self.Session(self.directory.name)
        for index in range(1000):
            session.append({'序号': index, '航向': 1.25, '原文': '$BDFPDL,abc'})
        session.close()
        with session.path.open(encoding='utf-8-sig', newline='') as file:
            rows = list(csv.DictReader(file))
        self.assertEqual(len(rows), 1000)
        self.assertEqual(rows[-1]['序号'], '999')
        self.assertEqual(session.count, 1000)

    def test_export_failure_retains_csv_and_can_retry(self):
        from openpyxl import load_workbook
        session = self.Session(self.directory.name)
        session.append({'角度': 1.25, '状态': 48, '原文': '=literal'})
        session.close()
        with patch('recording.os.replace', side_effect=OSError('disk full')):
            with self.assertRaises(OSError):
                session.export_excel()
        self.assertTrue(session.path.exists())
        target = session.export_excel()
        book = load_workbook(target)
        self.assertEqual(book.active['A2'].value, 1.25)
        self.assertEqual(book.active['B2'].value, 48)
        self.assertEqual(book.active['C2'].data_type, 's')
        book.close()

    def test_excel_splits_sheets_at_limit(self):
        from openpyxl import load_workbook
        session = self.Session(self.directory.name)
        for index in range(5):
            session.append({'序号': index})
        session.close()
        book = load_workbook(session.export_excel(max_rows=3))
        self.assertEqual([sheet.max_row for sheet in book], [3, 3, 2])
        self.assertEqual(book.worksheets[-1]['A2'].value, 4)
        book.close()

    def test_disk_error_is_exposed_and_stops_recording(self):
        session = self.Session(self.directory.name)
        with patch.object(session._file, 'write', side_effect=OSError('disk full')):
            with self.assertRaises(OSError):
                session.append({'value': 1})
        self.assertFalse(session.active)
        self.assertIn('disk full', session.error)
        session.close()


if __name__ == '__main__':
    unittest.main()
