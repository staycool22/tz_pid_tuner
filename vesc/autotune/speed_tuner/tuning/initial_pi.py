from __future__ import annotations

import math
from typing import Any, Dict, Tuple

from autotune.common.config.models import RunConfig


def _clamp(value: float, v_min: float, v_max: float) -> float:
    return max(v_min, min(v_max, value))


def _quantize(value: float, quantum: float) -> float:
    if quantum <= 0.0:
        return value
    return round(value / quantum) * quantum


def _as_float(data: Dict[str, Any], key: str) -> float | None:
    value = data.get(key)
    if value is None or value == "":
        return None
    return float(value)


def _positive_float(data: Dict[str, Any], key: str) -> float:
    value = _as_float(data, key)
    if value is None or value <= 0.0:
        raise ValueError(f"speed_tuner.initial_pi_formula.{key} 必须为正数")
    return value


def _build_bandwidth_rad_s(formula: Dict[str, Any]) -> Tuple[float, Dict[str, float]]:
    target_bandwidth = _as_float(formula, "target_bandwidth_rad_s")
    if target_bandwidth is not None and target_bandwidth > 0.0:
        return float(target_bandwidth), {"target_bandwidth_rad_s": float(target_bandwidth)}

    current_bandwidth = _as_float(formula, "current_loop_bandwidth_rad_s")
    if current_bandwidth is not None and current_bandwidth > 0.0:
        ratio = float(formula.get("speed_to_current_bandwidth_ratio", 8.0))
        if ratio <= 0.0:
            raise ValueError("speed_tuner.initial_pi_formula.speed_to_current_bandwidth_ratio 必须为正数")
        return float(current_bandwidth) / ratio, {
            "current_loop_bandwidth_rad_s": float(current_bandwidth),
            "speed_to_current_bandwidth_ratio": ratio,
        }

    target_tau = _as_float(formula, "target_time_constant_s")
    if target_tau is not None and target_tau > 0.0:
        return 1.0 / float(target_tau), {"target_time_constant_s": float(target_tau)}

    raise ValueError(
        "speed_tuner.initial_pi_formula 需要提供 "
        "target_bandwidth_rad_s、current_loop_bandwidth_rad_s 或 target_time_constant_s 之一"
    )


def _build_torque_constant(formula: Dict[str, Any]) -> Tuple[float, str]:
    torque_constant = _as_float(formula, "torque_constant_nm_per_a")
    if torque_constant is not None and torque_constant > 0.0:
        return float(torque_constant), "torque_constant_nm_per_a"

    kv_rpm_per_v = _as_float(formula, "kv_rpm_per_v")
    if kv_rpm_per_v is not None and kv_rpm_per_v > 0.0:
        return 60.0 / (2.0 * math.pi * float(kv_rpm_per_v)), "kv_rpm_per_v"

    ke_v_per_rad_s = _as_float(formula, "ke_v_per_rad_s")
    if ke_v_per_rad_s is not None and ke_v_per_rad_s > 0.0:
        return float(ke_v_per_rad_s), "ke_v_per_rad_s"

    raise ValueError(
        "speed_tuner.initial_pi_formula 需要提供 "
        "torque_constant_nm_per_a、kv_rpm_per_v 或 ke_v_per_rad_s 之一"
    )


def _convert_from_rad_per_s(
    kp_rad: float,
    ki_rad: float,
    controller_speed_unit: str,
    pole_pairs: float | None,
) -> Tuple[float, float, float]:
    unit = controller_speed_unit.strip().lower()
    if unit in {"rad_s", "rad/s", "rads"}:
        return kp_rad, ki_rad, 1.0
    if unit == "rpm":
        scale = 2.0 * math.pi / 60.0
        return kp_rad * scale, ki_rad * scale, scale
    if unit == "erpm":
        if pole_pairs is None or pole_pairs <= 0.0:
            raise ValueError("controller_speed_unit=erpm 时必须提供正数 pole_pairs")
        scale = 2.0 * math.pi / (60.0 * float(pole_pairs))
        return kp_rad * scale, ki_rad * scale, scale
    raise ValueError(
        "speed_tuner.initial_pi_formula.controller_speed_unit 仅支持 rad_s、rpm 或 erpm"
    )


def resolve_initial_pi_from_formula(
    formula: Dict[str, Any],
    config: RunConfig,
    source: str = "initial_pi_formula",
) -> Tuple[Dict[str, float], Dict[str, Any]]:
    kt, kt_source = _build_torque_constant(formula)
    inertia = _positive_float(formula, "inertia_kgm2")
    damping = _positive_float(formula, "damping_nms_per_rad")
    bandwidth_rad_s, bandwidth_meta = _build_bandwidth_rad_s(formula)
    controller_speed_unit = str(formula.get("controller_speed_unit", "erpm"))
    pole_pairs = _as_float(formula, "pole_pairs")
    if pole_pairs is None and float(config.motor.pole_pairs) > 0.0:
        pole_pairs = float(config.motor.pole_pairs)

    kp_rad = inertia * bandwidth_rad_s / kt
    ki_rad = damping * bandwidth_rad_s / kt
    kp_value, ki_value, unit_scale = _convert_from_rad_per_s(
        kp_rad=kp_rad,
        ki_rad=ki_rad,
        controller_speed_unit=controller_speed_unit,
        pole_pairs=pole_pairs,
    )

    limits = config.speed_tuner.param_limits
    quantum_kp = float(config.speed_tuner.param_quantum.get("kp", 0.00001))
    quantum_ki = float(config.speed_tuner.param_quantum.get("ki", 0.00001))
    kp_value = _quantize(
        _clamp(kp_value, float(limits["kp_min"]), float(limits["kp_max"])),
        quantum_kp,
    )
    ki_value = _quantize(
        _clamp(ki_value, float(limits["ki_min"]), float(limits["ki_max"])),
        quantum_ki,
    )
    kp_value = _clamp(kp_value, float(limits["kp_min"]), float(limits["kp_max"]))
    ki_value = _clamp(ki_value, float(limits["ki_min"]), float(limits["ki_max"]))

    details: Dict[str, Any] = {
        "source": source,
        "estimate_kind": str(formula.get("estimate_kind", "unloaded_first_pass")),
        "controller_speed_unit": controller_speed_unit,
        "pole_pairs": pole_pairs,
        "gear_ratio": float(config.motor.gear_ratio),
        "torque_constant_nm_per_a": kt,
        "torque_constant_source": kt_source,
        "inertia_kgm2": inertia,
        "damping_nms_per_rad": damping,
        "target_bandwidth_rad_s": bandwidth_rad_s,
        "unit_scale_from_rad_per_s": unit_scale,
        "formula_inputs": {**bandwidth_meta},
        "design_method": "pole_cancellation_pi",
        "raw_initial_pi": {
            "s_pid_kp": kp_value,
            "s_pid_ki": ki_value,
        },
    }
    return {"s_pid_kp": kp_value, "s_pid_ki": ki_value}, details


def resolve_initial_pi(config: RunConfig) -> Tuple[Dict[str, float], Dict[str, Any]]:
    formula = dict(config.speed_tuner.initial_pi_formula)
    if formula:
        return resolve_initial_pi_from_formula(formula=formula, config=config)

    initial = dict(config.speed_tuner.initial_pi)
    if "s_pid_kp" in initial and "s_pid_ki" in initial:
        return initial, {"source": "speed_tuner.initial_pi"}

    raise ValueError(
        "无法确定速度环初始 PI。请配置 speed_tuner.initial_pi_formula，"
        "或提供 speed_tuner.initial_pi。"
    )
