from __future__ import annotations

from typing import Dict, List

from autotune.common.acquisition.recorder import Sample


DEFAULT_STEADY_SPIKE_MARGIN_RPM = 3000.0


def _mean(values: List[float]) -> float:
    return float(sum(values) / len(values)) if values else 0.0


def evaluate_speed_trial(
    samples: List[Sample],
    target_rpm: float,
    use_abs_rpm: bool = False,
) -> Dict[str, float]:
    if not samples:
        return {
            "steady_error": 1e9,
            "signed_steady_error": 1e9,
            "overshoot": 1e9,
            "settling_time": 1e9,
            "current_efficiency": 1e9,
        }

    rpms = [abs(s.rpm) if use_abs_rpm else s.rpm for s in samples]
    target = abs(target_rpm) if use_abs_rpm else target_rpm
    currents = [abs(s.current_a) for s in samples]
    spike_upper_bound = abs(target) + DEFAULT_STEADY_SPIKE_MARGIN_RPM
    steady_slice = [x for x in rpms if abs(x) <= spike_upper_bound]
    if not steady_slice:
        steady_slice = rpms

    signed_steady_error = _mean(steady_slice) - target
    # 使用逐点绝对误差平均值（MAE）作为稳态误差，避免正负偏差互相抵消。
    steady_error = _mean([abs(x - target) for x in steady_slice])
    overshoot = max(0.0, max(rpms) - target)

    tolerance = max(30.0, abs(target) * 0.01)
    settling_time = samples[-1].t_s
    for s in samples:
        rpm_for_cmp = abs(s.rpm) if use_abs_rpm else s.rpm
        if abs(rpm_for_cmp - target) <= tolerance:
            settling_time = s.t_s
            break

    return {
        "steady_error": steady_error,
        "signed_steady_error": signed_steady_error,
        "overshoot": overshoot,
        "settling_time": settling_time,
        "current_efficiency": _mean(currents),
    }


def evaluate_position_trial(samples: List[Sample], target_deg: float) -> Dict[str, float]:
    if not samples:
        return {
            "position_stability": 1e9,
            "load_hold": 1e9,
            "step_response": 1e9,
            "rebound": 1e9,
            "current_efficiency": 1e9,
            "steady_state_entered": 0.0,
            "steady_points_count": 0.0,
            "min_abs_error": 1e9,
            "min_error_signed": 1e9,
            "closest_point_assessment": "unknown",
        }

    steady_tolerance_deg = 1.0
    pos = [s.pos_deg for s in samples]
    signed_errors = [p - target_deg for p in pos]
    abs_errors = [abs(e) for e in signed_errors]
    currents = [abs(s.current_a) for s in samples]
    steady_errors = [e for e in signed_errors if abs(e) <= steady_tolerance_deg]
    steady_points_count = float(len(steady_errors))
    steady_state_entered = 1.0 if steady_errors else 0.0

    min_idx = min(range(len(abs_errors)), key=lambda i: abs_errors[i])
    min_error_signed = float(signed_errors[min_idx])
    min_abs_error = float(abs_errors[min_idx])
    if min_error_signed > 0.0:
        closest_point_assessment = "overshoot"
    elif min_error_signed < 0.0:
        closest_point_assessment = "undershoot"
    else:
        closest_point_assessment = "on_target"

    step_response = samples[-1].t_s
    for s in samples:
        if abs(s.pos_deg - target_deg) <= steady_tolerance_deg:
            step_response = s.t_s
            break

    if steady_errors:
        # 用全局落入 ±1° 的点集统计稳态误差。
        position_stability = abs(_mean(steady_errors))
        load_hold = position_stability
        rebound = max(abs(e) for e in steady_errors)
    else:
        # 没有进入稳态范围时，直接判为无效稳态并给予高惩罚。
        position_stability = 1e9
        load_hold = 1e9
        rebound = max(abs_errors) if abs_errors else 1e9

    return {
        "position_stability": position_stability,
        "load_hold": load_hold,
        "step_response": step_response,
        "rebound": rebound,
        "current_efficiency": _mean(currents),
        "steady_state_entered": steady_state_entered,
        "steady_points_count": steady_points_count,
        "min_abs_error": min_abs_error,
        "min_error_signed": min_error_signed,
        "closest_point_assessment": closest_point_assessment,
    }


def weighted_score(metrics: Dict[str, float], weights: Dict[str, float]) -> float:
    return float(sum(metrics.get(k, 0.0) * w for k, w in weights.items()))
