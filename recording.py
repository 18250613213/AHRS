"""Incremental CSV capture and bounded-memory Excel export."""
import csv
import os
import threading
import uuid
from datetime import datetime
from pathlib import Path

from openpyxl import Workbook
from openpyxl.cell import WriteOnlyCell


class RecordingSession:
    def __init__(self, directory):
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        name = datetime.now().strftime('AHRS数据_%Y%m%d_%H%M%S_%f')
        self.path = directory / f'{name}_{uuid.uuid4().hex[:8]}.csv'
        self._file = self.path.open('x', encoding='utf-8-sig', newline='')
        self._lock = threading.Lock()
        self._writer = None
        self._numeric_types = {}
        self.count = 0
        self.active = True
        self.error = ''

    def append(self, record):
        with self._lock:
            if not self.active:
                raise RuntimeError(self.error or '记录已停止')
            try:
                if self._writer is None:
                    self._writer = csv.DictWriter(self._file, fieldnames=list(record))
                    self._numeric_types = {key: type(value) for key, value in record.items()
                                           if type(value) in (int, float)}
                    self._writer.writeheader()
                self._writer.writerow(record)
                self._file.flush()
                self.count += 1
            except (OSError, ValueError) as error:
                self.error = str(error)
                self.active = False
                try:
                    self._file.close()
                except OSError:
                    pass
                raise

    def close(self):
        with self._lock:
            self.active = False
            if self._file.closed:
                return
            try:
                self._file.flush()
                os.fsync(self._file.fileno())
            except OSError as error:
                self.error = str(error)
                raise
            finally:
                self._file.close()

    def export_excel(self, max_rows=1_048_576):
        if self.active:
            raise RuntimeError('请先停止记录')
        if max_rows < 2:
            raise ValueError('工作表必须能容纳表头和数据')
        target = self.path.with_suffix('.xlsx')
        temporary = target.with_name(f'.{target.stem}_{uuid.uuid4().hex}.tmp.xlsx')
        book = Workbook(write_only=True)
        try:
            with self.path.open(encoding='utf-8-sig', newline='') as source:
                reader = csv.DictReader(source)
                headers = reader.fieldnames
                if not headers:
                    raise ValueError('记录为空')
                sheet = book.create_sheet('数据1')
                sheet.append(headers)
                row_count = 1
                for row in reader:
                    if row_count >= max_rows:
                        sheet = book.create_sheet(f'数据{len(book.worksheets) + 1}')
                        sheet.append(headers)
                        row_count = 1
                    cells = []
                    for key in headers:
                        value = row[key]
                        if value is None:
                            raise ValueError('CSV 存在不完整的记录，请检查源文件')
                        convert = self._numeric_types.get(key)
                        cell = WriteOnlyCell(sheet, value=convert(value) if convert else value)
                        if not convert:
                            cell.data_type = 's'
                        cells.append(cell)
                    sheet.append(cells)
                    row_count += 1
            book.save(temporary)
            os.replace(temporary, target)
            return target
        finally:
            book.close()
            if temporary.exists():
                temporary.unlink()
