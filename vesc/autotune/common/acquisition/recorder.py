from __future__ import annotations

from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List
import csv
import json
import time


@dataclass
class Sample:
    t_s: float
    rpm: float
    current_a: float
    pos_deg: float


class TrialRecorder:
    def __init__(self) -> None:
        self.samples: List[Sample] = []
        self.started_at = time.time()

    def append(self, rpm: float, current_a: float, pos_deg: float) -> None:
        self.samples.append(
            Sample(
                t_s=time.time() - self.started_at,
                rpm=float(rpm),
                current_a=float(current_a),
                pos_deg=float(pos_deg),
            )
        )

    def reset(self) -> None:
        self.samples.clear()
        self.started_at = time.time()

    def dump_csv(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", encoding="utf-8", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=["t_s", "rpm", "current_a", "pos_deg"])
            writer.writeheader()
            for s in self.samples:
                writer.writerow(asdict(s))


def write_json(path: Path, payload: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")


def new_run_dir(root: Path, tuner_name: str) -> Path:
    timestamp = datetime.now().strftime("%Y-%m-%d_%H%M%S")
    run_dir = root / tuner_name / timestamp
    run_dir.mkdir(parents=True, exist_ok=True)
    return run_dir

