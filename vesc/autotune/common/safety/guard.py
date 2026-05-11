from __future__ import annotations

from dataclasses import dataclass


def unwrap_angle_deg(angle_deg: float, reference_deg: float) -> float:
    delta = (float(angle_deg) - float(reference_deg) + 180.0) % 360.0 - 180.0
    return float(reference_deg) + delta


@dataclass
class SafetyLimits:
    pos_min_deg: float
    pos_max_deg: float
    peak_current_limit_a: float
    enforce_position_limits: bool = True


class SafetyViolation(RuntimeError):
    pass


class SafetyGuard:
    def __init__(self, limits: SafetyLimits):
        self.limits = limits
        self._window_center_deg = (float(limits.pos_min_deg) + float(limits.pos_max_deg)) * 0.5

    def normalize_feedback_angle(self, pid_pos_now_deg: float) -> float:
        if not bool(self.limits.enforce_position_limits):
            return float(pid_pos_now_deg)
        return unwrap_angle_deg(pid_pos_now_deg, self._window_center_deg)

    def validate_feedback(self, pid_pos_now_deg: float, current_a: float) -> None:
        if bool(self.limits.enforce_position_limits):
            normalized_pos = self.normalize_feedback_angle(pid_pos_now_deg)
            if not (self.limits.pos_min_deg <= normalized_pos <= self.limits.pos_max_deg):
                raise SafetyViolation(
                    f"pid_pos={pid_pos_now_deg:.2f}° (归一化 {normalized_pos:.2f}°) 超出安全窗口 "
                    f"[{self.limits.pos_min_deg}, {self.limits.pos_max_deg}]"
                )
        if abs(current_a) > self.limits.peak_current_limit_a:
            raise SafetyViolation(
                f"current={current_a:.2f}A 超过阈值 {self.limits.peak_current_limit_a:.2f}A"
            )
