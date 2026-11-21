"""Per-frame diagnostics and review of AHRS recordings."""
import csv
import math
from collections import Counter, deque
from datetime import time
from pathlib import Path
from statistics import median

from openpyxl import load_workbook

AXES = ("经度", "纬度", "高度")
TYPES = {"position_type": "定位类型", "heading_type": "定向类型", "system_state": "系统状态字"}


class Diagnostics:
    def __init__(self):
        self.counts = {key: Counter() for key in TYPES}
        self.history = deque(maxlen=3)
        self.previous = None

    def update(self, data, stamp, mode, limits):
        result = {"测验场景": mode}
        for key, title in TYPES.items():
            if key in data:
                self.counts[key][data[key]] += 1
            result[title + "统计"] = "，".join(
                f"{value}×{count}" for value, count in self.counts[key].items())
        position = tuple(float(data[k]) for k in ("longitude", "latitude", "alt"))
        delta = None
        if self.previous and stamp is not None and self.previous[0] is not None:
            last, (lon, lat, height) = self.previous
            dt = stamp - last
            if 0 < dt < 2:
                metres = math.pi * 6371000 / 180
                step = (((position[0] - lon + 180) % 360 - 180) * metres *
                        math.cos(math.radians((lat + position[1]) / 2)),
                        (position[1] - lat) * metres, position[2] - height)
                if mode == "静置":
                    delta = step
                elif self.history:
                    delta = tuple(x - dt * median(v[i] for v in self.history)
                                  for i, x in enumerate(step))
                self.history.append(tuple(x / dt for x in step))
            else:
                self.history.clear()
        self.previous = stamp, position
        for i, axis in enumerate(AXES):
            diff = abs(delta[i]) if delta is not None else None
            result[axis + "偏差(m)"] = diff
            result[axis + "阈值(m)"] = limits[i]
            result[axis + "判定"] = "待判定" if diff is None else "跳变" if diff > limits[i] else "平滑"
        return result


def text_records(path, parsers, make_record):
    with open(path, encoding="utf-8-sig", newline="") as source:
        first = next((line.strip() for line in source if line.strip()), "")
        source.seek(0)
        if not first:
            return
        if "经度(°)" in first:
            delimiter = "\t" if "\t" in first else ","
            yield from csv.DictReader((line for line in source if line.strip()), delimiter=delimiter)
            return

        plain = first.startswith("$")
        buffers = {parser: b"" for parser in parsers}
        matched = None
        for number, line in enumerate(source, 1):
            try:
                chunk = (line.rstrip("\r\n").encode("ascii") + b"\n"
                         if plain else bytes.fromhex(line))
            except ValueError as error:
                raise ValueError(f"第 {number} 行无法识别，请使用十六进制原始记录、T02 报文或带表头的数据表") from error
            for parser in ([matched] if matched else parsers):
                frames, buffers[parser] = parser.find_frames(buffers[parser] + chunk)
                for frame in frames:
                    data = parser.parse_frame(frame)
                    if "error" not in data:
                        matched = parser
                        yield make_record(data)
                if matched:
                    break
        if not matched:
            raise ValueError("文件中没有通过校验的 T02 或 EB90 导航帧")


def review_rows(path, mode, limits, *, parsers=(), make_record=None):
    diagnostic = Diagnostics()

    def analyze(row):
        if "测验场景" in row and "高度判定" in row:
            return row
        data = {key: row[title] for key, title in TYPES.items() if title in row}
        for key, title in zip(("longitude", "latitude", "alt"), ("经度(°)", "纬度(°)", "高度(m)")):
            data[key] = float(row.get(title, row.get("融合高度(m)") if key == "alt" else None))
        stamp = None
        if "GPS周秒(s)" in row:
            stamp = int(row["GPS周数"]) * 604800 + float(row["GPS周秒(s)"])
        elif row.get("GPS时间"):
            t = time.fromisoformat(str(row["GPS时间"]))
            stamp = 3600*t.hour + 60*t.minute + t.second + t.microsecond/1e6
        return {**row, **diagnostic.update(data, stamp, mode, limits)}

    if Path(path).suffix.lower() == ".txt":
        for row in text_records(path, parsers, make_record):
            yield analyze(row)
    elif Path(path).suffix.lower() == ".xlsx":
        book = load_workbook(path, read_only=True, data_only=True)
        try:
            for sheet in book.worksheets:
                rows = sheet.values
                headers = next(rows, ())
                for values in rows:
                    yield analyze(dict(zip(headers, values)))
        finally:
            book.close()
    else:
        with open(path, encoding="utf-8-sig", newline="") as source:
            for row in csv.DictReader(source):
                yield analyze(row)
