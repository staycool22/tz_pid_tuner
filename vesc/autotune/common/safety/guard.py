from __future__ import annotations

from dataclasses import dataclass


@dataclass
class SafetyLimits:
    pos_min_deg: float
    pos_max_deg: float
    peak_current_limit_a: float


class SafetyViolation(RuntimeError):
    pass


class SafetyGuard:
    def __init__(self, limits: SafetyLimits):
        self.limits = limits

    def validate_feedback(self, pid_pos_now_deg: float, current_a: float) -> None:
        if not (self.limits.pos_min_deg <= pid_pos_now_deg <= self.limits.pos_max_deg):
            raise SafetyViolation(
                f"pid_pos={pid_pos_now_deg:.2f}° 超出安全窗口 [{self.limits.pos_min_deg}, {self.limits.pos_max_deg}]"
            )
        if abs(current_a) > self.limits.peak_current_limit_a:
            raise SafetyViolation(
                f"current={current_a:.2f}A 超过阈值 {self.limits.peak_current_limit_a:.2f}A"
            )

