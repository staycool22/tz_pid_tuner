from __future__ import annotations

from typing import Any, Dict, Tuple

from autotune.common.config.models import RunConfig


def _clamp(value: float, v_min: float, v_max: float) -> float:
    return max(v_min, min(v_max, value))


def _as_float(data: Dict[str, Any], key: str) -> float | None:
    value = data.get(key)
    if value is None or value == "":
        return None
    return float(value)


def _positive_float(data: Dict[str, Any], key: str) -> float:
    value = _as_float(data, key)
    if value is None or value <= 0.0:
        raise ValueError(f"position_tuner.initial_params_formula.{key} 必须为正数")
    return value


def _resolve_wb_speed(formula: Dict[str, Any]) -> Tuple[float, Dict[str, float]]:
    wb_speed = _as_float(formula, "speed_bandwidth_rad_s")
    if wb_speed is not None and wb_speed > 0.0:
        return float(wb_speed), {"speed_bandwidth_rad_s": float(wb_speed)}

    tau_speed = _as_float(formula, "speed_time_constant_s")
    if tau_speed is not None and tau_speed > 0.0:
        return 1.0 / float(tau_speed), {"speed_time_constant_s": float(tau_speed)}

    raise ValueError(
        "position_tuner.initial_params_formula 需要提供 speed_bandwidth_rad_s 或 speed_time_constant_s"
    )


def _resolve_wb_pos(formula: Dict[str, Any], wb_speed: float) -> Tuple[float, Dict[str, float]]:
    wb_pos = _as_float(formula, "position_bandwidth_rad_s")
    if wb_pos is not None and wb_pos > 0.0:
        return float(wb_pos), {"position_bandwidth_rad_s": float(wb_pos)}

    ratio = _as_float(formula, "position_to_speed_bandwidth_ratio")
    if ratio is None:
        ratio = 8.0
    if ratio <= 0.0:
        raise ValueError("position_tuner.initial_params_formula.position_to_speed_bandwidth_ratio 必须为正数")
    return float(wb_speed) / float(ratio), {"position_to_speed_bandwidth_ratio": float(ratio)}


def resolve_initial_position_params(config: RunConfig) -> Tuple[Dict[str, float], Dict[str, Any]]:
    formula = dict(config.position_tuner.initial_params_formula)
    if formula:
        wb_speed, wb_speed_meta = _resolve_wb_speed(formula)
        wb_pos, wb_pos_meta = _resolve_wb_pos(formula, wb_speed=wb_speed)
        pole_pairs = float(config.motor.pole_pairs)
        gear_ratio = float(config.motor.gear_ratio)
        l_max_erpm = _as_float(formula, "l_max_erpm")
        if l_max_erpm is None or l_max_erpm <= 0.0:
            l_max_erpm = float(config.motor.l_max_erpm)
        if l_max_erpm <= 0.0:
            raise ValueError(
                "position_tuner.initial_params_formula.l_max_erpm 必须为正数，"
                "或在 motor.l_max_erpm 中提供"
            )

        pos_interpretation = str(formula.get("position_interpretation", "output_shaft")).strip().lower()
        if pos_interpretation not in {"output_shaft", "motor_shaft"}:
            raise ValueError(
                "position_tuner.initial_params_formula.position_interpretation 仅支持 output_shaft 或 motor_shaft"
            )

        kp_pos_model = wb_pos
        if pos_interpretation == "output_shaft":
            k_eq = wb_pos * gear_ratio * pole_pairs / 6.0
        else:
            k_eq = wb_pos * pole_pairs / 6.0
        p_pid_kp = k_eq / (0.8 * l_max_erpm)
        kd_proc = _as_float(formula, "p_pid_kd_proc")
        if kd_proc is None:
            kd_proc = 0.0

        limits = config.position_tuner.param_limits
        p_pid_kp = _clamp(p_pid_kp, float(limits["kp_min"]), float(limits["kp_max"]))
        kd_proc = _clamp(kd_proc, float(limits["kd_proc_min"]), float(limits["kd_proc_max"]))

        details: Dict[str, Any] = {
            "source": "initial_params_formula",
            "position_interpretation": pos_interpretation,
            "wb_speed_rad_s": wb_speed,
            "wb_pos_rad_s": wb_pos,
            "kp_pos_model_1_per_s": kp_pos_model,
            "k_eq_erpm_per_deg": k_eq,
            "l_max_erpm": l_max_erpm,
            "pole_pairs": pole_pairs,
            "gear_ratio": gear_ratio,
            "formula_inputs": {**wb_speed_meta, **wb_pos_meta},
            "raw_initial_params": {
                "p_pid_kp": p_pid_kp,
                "p_pid_kd_proc": kd_proc,
            },
        }
        return {"p_pid_kp": p_pid_kp, "p_pid_kd_proc": kd_proc}, details

    initial = dict(config.position_tuner.initial_params)
    if "p_pid_kp" in initial and "p_pid_kd_proc" in initial:
        return initial, {"source": "position_tuner.initial_params"}

    raise ValueError(
        "无法确定位置环初始参数。请配置 position_tuner.initial_params_formula，"
        "或提供 position_tuner.initial_params。"
    )
