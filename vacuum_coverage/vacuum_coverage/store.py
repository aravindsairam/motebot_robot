"""Saved cleaning plans, the record of every clean, and lifetime totals, on disk. No ROS imports.

Everything belongs to one map and lives in a folder next to it:

    <maps>/<map>_cleaning/plans/<name>.yaml         a saved plan (the name is %-encoded in the file name)
    <maps>/<map>_cleaning/records/<id>.yaml + .png  one clean: its numbers, and its picture
    <maps>/<map>_cleaning/records/<id>_cleaned.png  the floor it cleaned: one bit per map cell (row 0 at the top
                                                    of the image is the map's row 0); older records have none
    <maps>/<map>_cleaning/totals.yaml               sums over every clean ever recorded

Only the cleaning node writes here. Every file is written whole or not at all: to a hidden
temporary file first, then renamed over the old one. A file that cannot be read (cut short by a
crash, edited by hand) is skipped and reported in `problems`, never fatal.

The newest `history_size` records are kept. The totals are kept separately, so dropping old
records does not change them; if the totals file is lost they are rebuilt from the records left.
"""

import os
import re
import tempfile
import time
import urllib.parse
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
import yaml

from .planner import Piece

_NAME = re.compile(r"^[A-Za-z0-9][A-Za-z0-9 _-]{0,39}$")
_ID = re.compile(r"^[0-9]{8}-[0-9]{6}-[0-9]+$")
_TOTAL_KEYS = ("cleans", "area", "time", "distance")


class StoreError(Exception):
    def __init__(self, message: str, name_taken: bool = False):
        super().__init__(message)
        self.name_taken = name_taken


@dataclass
class SavedPlan:
    name: str
    job: dict
    pieces: List[Piece]
    estimate: dict
    map_name: str
    map_checksum: str
    saved: float  # seconds since 1970


def _write_atomic(path: str, data: bytes) -> None:
    folder = os.path.dirname(path)
    fd, tmp = tempfile.mkstemp(prefix=".tmp-", dir=folder)
    try:
        with os.fdopen(fd, "wb") as f:
            f.write(data)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, path)
    except BaseException:
        if os.path.exists(tmp):
            os.unlink(tmp)
        raise


def _mask_png(mask: np.ndarray) -> bytes:
    import io

    from PIL import Image

    out = io.BytesIO()
    Image.fromarray(np.asarray(mask, dtype=bool)).save(out, format="PNG", optimize=True)
    return out.getvalue()


def _dump(data) -> bytes:
    return yaml.safe_dump(data, sort_keys=False, allow_unicode=True).encode()


def piece_to_list(p: Piece) -> list:
    return [p.kind, float(p.start[0]), float(p.start[1]), float(p.end[0]), float(p.end[1]), p.number, p.pass_number, p.region]


def piece_from_list(v: Sequence) -> Piece:
    return Piece(str(v[0]), (float(v[1]), float(v[2])), (float(v[3]), float(v[4])), int(v[5]), int(v[6]), int(v[7]))


class CleaningStore:
    def __init__(self, maps_dir: str, map_name: str, history_size: int = 50):
        self.map_name = map_name
        self.folder = os.path.join(maps_dir, f"{map_name}_cleaning")
        self.plans_dir = os.path.join(self.folder, "plans")
        self.records_dir = os.path.join(self.folder, "records")
        self.totals_path = os.path.join(self.folder, "totals.yaml")
        self.history_size = history_size
        self.problems: List[str] = []  # files skipped since the last look, for the log
        os.makedirs(self.plans_dir, exist_ok=True)
        os.makedirs(self.records_dir, exist_ok=True)

    # ------------------------------------------------------------------ plans
    @staticmethod
    def check_name(name: str) -> str:
        name = name.strip()
        if not _NAME.match(name):
            raise StoreError("plan names have 1 to 40 letters, digits, spaces, '-' or '_', starting with a letter or digit")
        return name

    def _plan_path(self, name: str) -> str:
        return os.path.join(self.plans_dir, urllib.parse.quote(name, safe="") + ".yaml")

    def save_plan(self, name: str, job: dict, pieces: Sequence[Piece], estimate: dict, map_checksum: str,
                  replace: bool = False) -> SavedPlan:
        name = self.check_name(name)
        path = self._plan_path(name)
        if os.path.exists(path) and not replace:
            raise StoreError(f"a plan named '{name}' already exists; save with replace to overwrite it", name_taken=True)
        plan = SavedPlan(name, job, list(pieces), dict(estimate), self.map_name, map_checksum, time.time())
        _write_atomic(path, _dump({
            "name": plan.name,
            "saved": plan.saved,
            "map": plan.map_name,
            "map_checksum": plan.map_checksum,
            "job": plan.job,
            "estimate": plan.estimate,
            "pieces": [piece_to_list(p) for p in plan.pieces],
        }))
        return plan

    def _read_plan(self, path: str) -> Optional[SavedPlan]:
        try:
            with open(path) as f:
                d = yaml.safe_load(f)
            return SavedPlan(str(d["name"]), dict(d["job"]), [piece_from_list(v) for v in d["pieces"]], dict(d["estimate"]),
                             str(d["map"]), str(d["map_checksum"]), float(d["saved"]))
        except Exception as exc:  # cut short, edited by hand: skip it, say so
            self.problems.append(f"skipped plan file {os.path.basename(path)}: {exc}")
            return None

    def load_plan(self, name: str) -> Optional[SavedPlan]:
        try:
            path = self._plan_path(self.check_name(name))
        except StoreError:
            return None
        return self._read_plan(path) if os.path.isfile(path) else None

    def list_plans(self) -> List[SavedPlan]:
        plans = [self._read_plan(os.path.join(self.plans_dir, f)) for f in sorted(os.listdir(self.plans_dir))
                 if f.endswith(".yaml") and not f.startswith(".")]
        return sorted((p for p in plans if p is not None), key=lambda p: p.name.lower())

    def delete_plan(self, name: str) -> bool:
        try:
            path = self._plan_path(self.check_name(name))
        except StoreError:
            return False
        if not os.path.isfile(path):
            return False
        os.unlink(path)
        return True

    # ------------------------------------------------------------------ records
    def add_record(self, record: dict, picture_png: bytes, cleaned: Optional[np.ndarray] = None) -> str:
        """Keep one clean's record, picture and cleaned floor (bool per map cell); update the totals;
        drop the oldest beyond history_size."""
        stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime(float(record.get("started", time.time()))))
        rid = f"{stamp}-{int(record.get('job_id', 0))}"
        n = 0
        while os.path.exists(os.path.join(self.records_dir, rid + ".yaml")):
            n += 1
            rid = f"{stamp}-{int(record.get('job_id', 0))}{n}"
        record = dict(record, id=rid)
        totals = self.totals()  # before this record exists: a rebuild must not count it twice
        # The picture first: a record that exists always has its picture.
        _write_atomic(os.path.join(self.records_dir, rid + ".png"), picture_png)
        if cleaned is not None:
            _write_atomic(os.path.join(self.records_dir, rid + "_cleaned.png"), _mask_png(cleaned))
        _write_atomic(os.path.join(self.records_dir, rid + ".yaml"), _dump(record))
        status = record.get("status", {})
        totals["cleans"] += 1
        totals["area"] += float(status.get("cleaned_area", 0.0))
        totals["time"] += float(status.get("cleaning_time", 0.0))
        totals["distance"] += float(status.get("distance", 0.0))
        _write_atomic(self.totals_path, _dump(totals))
        self._prune()
        return rid

    def _record_ids(self) -> List[str]:
        ids = [f[:-5] for f in os.listdir(self.records_dir) if f.endswith(".yaml") and not f.startswith(".")]
        return sorted((i for i in ids if _ID.match(i)), key=self._order_key)

    @staticmethod
    def _order_key(rid: str) -> Tuple[str, int]:
        stamp, job = rid.rsplit("-", 1)
        return stamp, int(job)

    def _prune(self) -> None:
        ids = self._record_ids()
        for rid in ids[: max(len(ids) - self.history_size, 0)]:
            for ext in (".yaml", ".png", "_cleaned.png"):
                path = os.path.join(self.records_dir, rid + ext)
                if os.path.exists(path):
                    os.unlink(path)

    def _read_record(self, rid: str) -> Optional[dict]:
        path = os.path.join(self.records_dir, rid + ".yaml")
        try:
            with open(path) as f:
                d = yaml.safe_load(f)
            if not isinstance(d, dict) or "outcome" not in d:
                raise ValueError("not a cleaning record")
            d["id"] = rid
            return d
        except Exception as exc:
            self.problems.append(f"skipped record {rid}: {exc}")
            return None

    def history(self, limit: Optional[int] = None) -> List[dict]:
        """Records, newest first."""
        out = []
        for rid in reversed(self._record_ids()):
            r = self._read_record(rid)
            if r is not None:
                out.append(r)
                if limit is not None and len(out) >= limit:
                    break
        return out

    def get_record(self, rid: str) -> Optional[Tuple[dict, bytes]]:
        if not _ID.match(rid or ""):
            return None
        if not os.path.isfile(os.path.join(self.records_dir, rid + ".yaml")):
            return None
        record = self._read_record(rid)
        if record is None:
            return None
        try:
            with open(os.path.join(self.records_dir, rid + ".png"), "rb") as f:
                picture = f.read()
        except OSError:
            picture = b""
        return record, picture

    def cleaned(self, rid: str) -> Optional[np.ndarray]:
        """The floor a clean cleaned (bool per map cell), or None if the record has none."""
        if not _ID.match(rid or ""):
            return None
        path = os.path.join(self.records_dir, rid + "_cleaned.png")
        if not os.path.isfile(path):
            return None
        try:
            from PIL import Image

            with Image.open(path) as image:
                return np.asarray(image.convert("1"), dtype=bool).copy()
        except Exception as exc:
            self.problems.append(f"skipped the cleaned floor of record {rid}: {exc}")
            return None

    def totals(self) -> Dict[str, float]:
        try:
            with open(self.totals_path) as f:
                d = yaml.safe_load(f)
            totals = {k: float(d[k]) for k in _TOTAL_KEYS}
            totals["cleans"] = int(totals["cleans"])
            return totals
        except FileNotFoundError:
            if not self._record_ids():
                return {"cleans": 0, "area": 0.0, "time": 0.0, "distance": 0.0}
            reason = "missing"
        except Exception as exc:
            reason = f"unreadable ({exc})"
        # Lost: rebuild from the records that are left (older ones are gone for good) and say so.
        totals = {"cleans": 0, "area": 0.0, "time": 0.0, "distance": 0.0}
        for r in self.history():
            status = r.get("status", {})
            totals["cleans"] += 1
            totals["area"] += float(status.get("cleaned_area", 0.0))
            totals["time"] += float(status.get("cleaning_time", 0.0))
            totals["distance"] += float(status.get("distance", 0.0))
        self.problems.append(f"totals file {reason}; rebuilt from the {totals['cleans']} records kept")
        _write_atomic(self.totals_path, _dump(totals))
        return totals
