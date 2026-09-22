"""Bounded JSONL reporting for long-running CARLA scenes."""

import json
from pathlib import Path


class RotatingReport:
  def __init__(self, directory: Path, *, max_bytes: int = 10 * 1024 * 1024):
    if max_bytes <= 0:
      raise ValueError("max_bytes must be positive")
    self.directory = Path(directory)
    self.directory.mkdir(parents=True, exist_ok=True)
    self.max_bytes = max_bytes
    self.index = 1
    self.file = self._open_file()

  def _path(self) -> Path:
    return self.directory / ("events.jsonl" if self.index == 1 else f"events-{self.index:04d}.jsonl")

  def _open_file(self):
    return self._path().open("w", encoding="utf-8")

  def write(self, record: dict):
    line = json.dumps(record, allow_nan=False) + "\n"
    if self.file.tell() and self.file.tell() + len(line.encode("utf-8")) > self.max_bytes:
      self.file.close()
      self.index += 1
      self.file = self._open_file()
    self.file.write(line)
    self.file.flush()

  def close(self):
    self.file.close()

  def __enter__(self):
    return self

  def __exit__(self, _type, _value, _traceback):
    self.close()
