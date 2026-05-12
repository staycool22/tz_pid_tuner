from __future__ import annotations

import math
import threading
import time
from pathlib import Path
from typing import Any, Dict, Iterable, List, Tuple

from autotune.common.acquisition.recorder import Sample, TrialRecorder, write_json
from autotune.common.config.models import RunConfig
from autotune.common.io.vesc_client import VESCBusClient
from autotune.common.safety.guard import SafetyGuard, SafetyLimits, SafetyViolation
from autotune.speed_tuner.tuning.initial_pi import resolve_initial_pi_from_formula

DEFAULT_IDENTIFICATION_SAMPLE_TIMEOUT_S = 0.05


def _cfg_value(data: Dict[str, Any], key: str, default: Any) -> Any:
    value = data.get(key, default)
    return default if value is None else value


def _cfg_float(data: Dict[str, Any], key: str, default: float) -> float:
    return float(_cfg_value(data, key, default))


def _cfg_bool(data: Dict[str, Any], key: str, default: bool) -> bool:
    return bool(_cfg_value(data, key, default))


def _mean(values: Iterable[float]) -> float:
    values_list = list(values)
    return float(sum(values_list) / len(values_list)) if values_list else 0.0


def _linear_regression(x_values: List[float], y_values: List[float]) -> Tuple[float, float]:
    if len(x_values) != len(y_values) or len(x_values) < 2:
        raise ValueError("线性拟合至少需要 2 个样本")
    x_mean = _mean(x_values)
    y_mean = _mean(y_values)
    denom = sum((x - x_mean) ** 2 for x in x_values)
    if denom <= 0.0:
        raise ValueError("线性拟合失败：时间样本无变化")
    slope = sum((x - x_mean) * (y - y_mean) for x, y in zip(x_values, y_values)) / denom
    intercept = y_mean - slope * x_mean
    return float(slope), float(intercept)


def _build_guard(config: RunConfig) -> SafetyGuard:
    limits = SafetyLimits(
        pos_min_deg=float(config.motor.pos_window_deg[0]),
        pos_max_deg=float(config.motor.pos_window_deg[1]),
        peak_current_limit_a=float(config.motor.peak_current_limit_a),
        enforce_position_limits=bool(config.motor.uses_position_limits),
    )
    return SafetyGuard(limits)


def _torque_constant_from_identification(config: RunConfig) -> Tuple[float, str]:
    ident = dict(config.speed_tuner.identification)
    if ident.get("torque_constant_nm_per_a") not in (None, ""):
        return float(ident["torque_constant_nm_per_a"]), "torque_constant_nm_per_a"
    if ident.get("kv_rpm_per_v") not in (None, ""):
        kv = float(ident["kv_rpm_per_v"])
        if kv <= 0.0:
            raise ValueError("speed_tuner.identification.kv_rpm_per_v 必须为正数")
        return 60.0 / (2.0 * math.pi * kv), "kv_rpm_per_v"
    if ident.get("ke_v_per_rad_s") not in (None, ""):
        ke = float(ident["ke_v_per_rad_s"])
        if ke <= 0.0:
            raise ValueError("speed_tuner.identification.ke_v_per_rad_s 必须为正数")
        return ke, "ke_v_per_rad_s"
    raise ValueError(
        "speed_tuner.identification 需要提供 "
        "torque_constant_nm_per_a、kv_rpm_per_v 或 ke_v_per_rad_s 之一"
    )


def _feedback_to_motor_rad_s(config: RunConfig, speed_value: float, unit: str) -> float:
    pole_pairs = float(config.motor.pole_pairs)
    if pole_pairs <= 0.0:
        raise ValueError("motor.pole_pairs 必须为正数")
    unit_norm = unit.strip().lower()
    if unit_norm == "erpm":
        motor_rpm = float(speed_value) / pole_pairs
        return motor_rpm * 2.0 * math.pi / 60.0
    if unit_norm == "rpm":
        return float(speed_value) * 2.0 * math.pi / 60.0
    if unit_norm in {"rad_s", "rad/s", "rads"}:
        return float(speed_value)
    raise ValueError("feedback_speed_unit 仅支持 erpm、rpm 或 rad_s")


def _feedback_to_output_rad_s(config: RunConfig, speed_value: float, unit: str) -> float:
    gear_ratio = max(1.0, float(config.motor.gear_ratio))
    return _feedback_to_motor_rad_s(config, speed_value, unit) / gear_ratio


def _speed_threshold_to_motor_rad_s(config: RunConfig, speed_value: float, unit: str) -> float:
    return abs(_feedback_to_motor_rad_s(config, speed_value, unit))


def _collect_current_trial(
    client: VESCBusClient,
    guard: SafetyGuard,
    current_a: float,
    duration_s: float,
    command_hz: float,
    read_hz: float,
    sample_timeout_s: float,
) -> TrialRecorder:
    recorder = TrialRecorder()
    read_period_s = 1.0 / max(1.0, float(read_hz))
    send_period_s = 1.0 / max(1.0, float(command_hz))
    stop_event = threading.Event()
    violation: List[SafetyViolation] = []

    def _monitor_feedback() -> None:
        while not stop_event.is_set():
            pack = client.receive_decode(min(sample_timeout_s, read_period_s))
            if pack is None:
                continue
            pos_now = guard.normalize_feedback_angle(float(pack.pid_pos_now))
            try:
                guard.validate_feedback(pid_pos_now_deg=float(pack.pid_pos_now), current_a=float(pack.current))
            except SafetyViolation as exc:
                violation.append(exc)
                stop_event.set()
                break
            recorder.append(rpm=float(pack.rpm), current_a=float(pack.current), pos_deg=pos_now)

    monitor = threading.Thread(target=_monitor_feedback, name="speed-ident-feedback-monitor", daemon=True)
    monitor.start()
    start = time.perf_counter()
    next_send_tick = start
    try:
        while time.perf_counter() - start <= duration_s and not stop_event.is_set():
            now = time.perf_counter()
            if now >= next_send_tick:
                client.send_current(current_a)
                while next_send_tick <= now:
                    next_send_tick += send_period_s
            else:
                time.sleep(min(0.001, max(0.0, next_send_tick - now)))
    finally:
        stop_event.set()
        monitor.join(timeout=max(0.5, sample_timeout_s * 2.0))
        client.send_current(0.0)
    if violation:
        raise violation[0]
    return recorder


def _slice_samples(samples: List[Sample], start_s: float, end_s: float) -> List[Sample]:
    return [s for s in samples if start_s <= float(s.t_s) <= end_s]


def _fit_acceleration(
    config: RunConfig,
    recorder: TrialRecorder,
    fit_samples: List[Sample],
    feedback_speed_unit: str,
) -> Dict[str, Any]:
    if len(fit_samples) < 3:
        raise ValueError("加速度拟合样本不足，请增大 accel_fit_window_s 或提高 read_hz")
    times = [float(s.t_s) for s in fit_samples]
    omega_motor = [_feedback_to_motor_rad_s(config, float(s.rpm), feedback_speed_unit) for s in fit_samples]
    slope, intercept = _linear_regression(times, omega_motor)
    iq_mean = _mean(float(s.current_a) for s in fit_samples)
    pos_start = float(fit_samples[0].pos_deg)
    pos_end = float(fit_samples[-1].pos_deg)
    return {
        "alpha_rad_s2": slope,
        "omega_intercept_rad_s": intercept,
        "iq_mean_a": iq_mean,
        "fit_sample_count": len(fit_samples),
        "fit_window_s": [float(times[0]), float(times[-1])],
        "pos_window_deg": [pos_start, pos_end],
        "rpm_window": [float(fit_samples[0].rpm), float(fit_samples[-1].rpm)],
        "output_speed_window_rad_s": [
            _feedback_to_output_rad_s(config, float(fit_samples[0].rpm), feedback_speed_unit),
            _feedback_to_output_rad_s(config, float(fit_samples[-1].rpm), feedback_speed_unit),
        ],
    }


def _estimate_inertia(
    config: RunConfig,
    client: VESCBusClient,
    guard: SafetyGuard,
    ident_dir: Path,
    kt_nm_per_a: float,
    feedback_speed_unit: str,
) -> Tuple[float, Dict[str, Any]]:
    ident = dict(config.speed_tuner.identification)
    iq_test_a = ident.get("iq_test_a")
    if iq_test_a in (None, ""):
        iq_test_a = float(config.motor.peak_current_limit_a) * _cfg_float(ident, "iq_test_ratio", 0.10)
    iq_test_a = abs(float(iq_test_a))
    if iq_test_a <= 0.0:
        raise ValueError("辨识电流 iq_test_a 必须大于 0")

    step_duration_s = _cfg_float(ident, "step_duration_s", 0.30)
    fit_start_s = _cfg_float(ident, "settle_time_s", 0.05) + _cfg_float(ident, "transient_discard_s", 0.02)
    fit_end_s = fit_start_s + _cfg_float(ident, "accel_fit_window_s", 0.12)
    rest_s = _cfg_float(ident, "rest_between_steps_s", 0.20)

    plus_rec = _collect_current_trial(
        client=client,
        guard=guard,
        current_a=iq_test_a,
        duration_s=step_duration_s,
        command_hz=float(config.speed_tuner.command_hz),
        read_hz=float(config.speed_tuner.read_hz),
        sample_timeout_s=DEFAULT_IDENTIFICATION_SAMPLE_TIMEOUT_S,
    )
    plus_rec.dump_csv(ident_dir / "j_plus_raw.csv")
    time.sleep(rest_s)

    minus_rec = _collect_current_trial(
        client=client,
        guard=guard,
        current_a=-iq_test_a,
        duration_s=step_duration_s,
        command_hz=float(config.speed_tuner.command_hz),
        read_hz=float(config.speed_tuner.read_hz),
        sample_timeout_s=DEFAULT_IDENTIFICATION_SAMPLE_TIMEOUT_S,
    )
    minus_rec.dump_csv(ident_dir / "j_minus_raw.csv")
    time.sleep(rest_s)

    plus_fit_samples = _slice_samples(plus_rec.samples, fit_start_s, fit_end_s)
    minus_fit_samples = _slice_samples(minus_rec.samples, fit_start_s, fit_end_s)
    plus_fit = _fit_acceleration(config, plus_rec, plus_fit_samples, feedback_speed_unit)
    minus_fit = _fit_acceleration(config, minus_rec, minus_fit_samples, feedback_speed_unit)

    alpha_delta = float(plus_fit["alpha_rad_s2"]) - float(minus_fit["alpha_rad_s2"])
    iq_delta = float(plus_fit["iq_mean_a"]) - float(minus_fit["iq_mean_a"])
    if abs(alpha_delta) <= 1e-9:
        raise ValueError("J 辨识失败：正负电流得到的加速度差过小")
    inertia = kt_nm_per_a * iq_delta / alpha_delta
    if inertia <= 0.0:
        raise ValueError("J 辨识失败：得到的惯量非正，请检查电流方向、速度方向和 pole_pairs")
    details = {
        "iq_test_a": iq_test_a,
        "fit_start_s": fit_start_s,
        "fit_end_s": fit_end_s,
        "plus": plus_fit,
        "minus": minus_fit,
        "formula": "J = Kt * (iq_plus - iq_minus) / (alpha_plus - alpha_minus)",
        "inertia_kgm2": inertia,
    }
    return inertia, details


def _estimate_damping_from_coast_down(
    config: RunConfig,
    client: VESCBusClient,
    guard: SafetyGuard,
    ident_dir: Path,
    inertia_kgm2: float,
    kt_nm_per_a: float,
    feedback_speed_unit: str,
) -> Tuple[float, Dict[str, Any]]:
    ident = dict(config.speed_tuner.identification)
    spinup_current = ident.get("coast_down_spinup_current_a")
    if spinup_current in (None, ""):
        spinup_current = float(config.motor.peak_current_limit_a) * _cfg_float(ident, "coast_down_spinup_ratio", 0.15)
    spinup_current = abs(float(spinup_current))
    if spinup_current <= 0.0:
        raise ValueError("coast_down_spinup_current_a 必须大于 0")

    spinup_rec = _collect_current_trial(
        client=client,
        guard=guard,
        current_a=spinup_current,
        duration_s=_cfg_float(ident, "coast_down_spinup_duration_s", 0.60),
        command_hz=float(config.speed_tuner.command_hz),
        read_hz=float(config.speed_tuner.read_hz),
        sample_timeout_s=DEFAULT_IDENTIFICATION_SAMPLE_TIMEOUT_S,
    )
    spinup_rec.dump_csv(ident_dir / "b_spinup_raw.csv")
    time.sleep(_cfg_float(ident, "rest_between_steps_s", 0.20))

    coast_rec = _collect_current_trial(
        client=client,
        guard=guard,
        current_a=0.0,
        duration_s=_cfg_float(ident, "coast_down_duration_s", 0.80),
        command_hz=float(config.speed_tuner.command_hz),
        read_hz=float(config.speed_tuner.read_hz),
        sample_timeout_s=DEFAULT_IDENTIFICATION_SAMPLE_TIMEOUT_S,
    )
    coast_rec.dump_csv(ident_dir / "b_coast_down_raw.csv")

    speed_threshold = _speed_threshold_to_motor_rad_s(
        config,
        _cfg_float(ident, "coast_down_min_speed_for_fit", 200.0),
        str(_cfg_value(ident, "coast_down_min_speed_unit", "erpm")),
    )
    discard_s = _cfg_float(ident, "coast_down_discard_s", 0.02)
    fit_samples = [
        s
        for s in coast_rec.samples
        if float(s.t_s) >= discard_s
        and abs(_feedback_to_motor_rad_s(config, float(s.rpm), feedback_speed_unit)) >= speed_threshold
    ]
    if len(fit_samples) < 3:
        raise ValueError("coast-down 样本不足，请提高 spinup 电流/时长或降低最小拟合速度阈值")

    times = [float(s.t_s) for s in fit_samples]
    ln_omega = [
        math.log(abs(_feedback_to_motor_rad_s(config, float(s.rpm), feedback_speed_unit)))
        for s in fit_samples
    ]
    slope, intercept = _linear_regression(times, ln_omega)
    if slope >= 0.0:
        raise ValueError("coast-down 拟合失败：速度未呈指数衰减")
    damping = -inertia_kgm2 * slope
    if damping <= 0.0:
        raise ValueError("B 辨识失败：得到的阻尼非正")

    details = {
        "spinup_current_a": spinup_current,
        "discard_s": discard_s,
        "fit_sample_count": len(fit_samples),
        "fit_window_s": [float(times[0]), float(times[-1])],
        "ln_omega_intercept": intercept,
        "ln_omega_slope": slope,
        "formula": "B = -J * d(ln(omega))/dt",
        "damping_nms_per_rad": damping,
        "speed_threshold_motor_rad_s": speed_threshold,
        "kt_nm_per_a": kt_nm_per_a,
    }
    return damping, details


def _estimate_damping_from_steady_state(
    config: RunConfig,
    client: VESCBusClient,
    guard: SafetyGuard,
    ident_dir: Path,
    kt_nm_per_a: float,
    feedback_speed_unit: str,
) -> Tuple[float, Dict[str, Any]]:
    ident = dict(config.speed_tuner.identification)
    iq_test_a = ident.get("coast_down_spinup_current_a")
    if iq_test_a in (None, ""):
        iq_test_a = float(config.motor.peak_current_limit_a) * _cfg_float(ident, "coast_down_spinup_ratio", 0.15)
    iq_test_a = abs(float(iq_test_a))
    rec = _collect_current_trial(
        client=client,
        guard=guard,
        current_a=iq_test_a,
        duration_s=_cfg_float(ident, "steady_state_duration_s", 1.00),
        command_hz=float(config.speed_tuner.command_hz),
        read_hz=float(config.speed_tuner.read_hz),
        sample_timeout_s=DEFAULT_IDENTIFICATION_SAMPLE_TIMEOUT_S,
    )
    rec.dump_csv(ident_dir / "b_steady_state_raw.csv")
    eval_window_s = _cfg_float(ident, "steady_state_eval_window_s", 0.25)
    fit_samples = [s for s in rec.samples if float(s.t_s) >= max(0.0, float(rec.samples[-1].t_s) - eval_window_s)]
    if len(fit_samples) < 3:
        raise ValueError("稳态 B 估计样本不足")
    iq_mean = abs(_mean(float(s.current_a) for s in fit_samples))
    omega_ss = abs(
        _mean(_feedback_to_motor_rad_s(config, float(s.rpm), feedback_speed_unit) for s in fit_samples)
    )
    if omega_ss <= 1e-9:
        raise ValueError("稳态 B 估计失败：稳态速度过小")
    damping = kt_nm_per_a * iq_mean / omega_ss
    if damping <= 0.0:
        raise ValueError("稳态 B 估计失败：得到的阻尼非正")
    details = {
        "iq_mean_a": iq_mean,
        "omega_ss_motor_rad_s": omega_ss,
        "fit_sample_count": len(fit_samples),
        "formula": "B = Kt * iq / omega_ss",
        "damping_nms_per_rad": damping,
    }
    return damping, details


def run_initial_pi_identification(config: RunConfig, run_dir: Path) -> Dict[str, Any]:
    ident = dict(config.speed_tuner.identification)
    if not _cfg_bool(ident, "enabled", False):
        return {}

    ident_dir = run_dir / "identification"
    ident_dir.mkdir(parents=True, exist_ok=True)
    guard = _build_guard(config)
    kt_nm_per_a, kt_source = _torque_constant_from_identification(config)
    feedback_speed_unit = str(_cfg_value(ident, "feedback_speed_unit", "erpm"))

    if _cfg_bool(ident, "manual_confirm_before_identification", True):
        print("[speed_tuner] 即将执行 initial_pi 辨识流程。")
        print("[speed_tuner] 请确认: 电流环已稳定、当前为首轮空载测试、机械运动空间安全。")
        user_in = input("[speed_tuner] 按回车开始辨识；输入 q 终止: ").strip().lower()
        if user_in == "q":
            raise RuntimeError("用户取消了 initial_pi 辨识流程")

    with VESCBusClient(config.can, config.motor) as client:
        try:
            inertia_kgm2, inertia_details = _estimate_inertia(
                config=config,
                client=client,
                guard=guard,
                ident_dir=ident_dir,
                kt_nm_per_a=kt_nm_per_a,
                feedback_speed_unit=feedback_speed_unit,
            )
            try:
                damping_nms_per_rad, damping_details = _estimate_damping_from_coast_down(
                    config=config,
                    client=client,
                    guard=guard,
                    ident_dir=ident_dir,
                    inertia_kgm2=inertia_kgm2,
                    kt_nm_per_a=kt_nm_per_a,
                    feedback_speed_unit=feedback_speed_unit,
                )
                damping_method = "coast_down"
            except Exception as coast_exc:
                if not _cfg_bool(ident, "allow_steady_state_b_fallback", True):
                    raise
                damping_nms_per_rad, damping_details = _estimate_damping_from_steady_state(
                    config=config,
                    client=client,
                    guard=guard,
                    ident_dir=ident_dir,
                    kt_nm_per_a=kt_nm_per_a,
                    feedback_speed_unit=feedback_speed_unit,
                )
                damping_method = "steady_state_fallback"
                damping_details["coast_down_error"] = str(coast_exc)
        finally:
            client.send_current(0.0)

    formula: Dict[str, Any] = {
        "estimate_kind": str(_cfg_value(ident, "estimate_kind", "unloaded_first_pass")),
        "controller_speed_unit": str(_cfg_value(ident, "controller_speed_unit", "erpm")),
        "pole_pairs": float(config.motor.pole_pairs),
        "gear_ratio": float(config.motor.gear_ratio),
        "torque_constant_nm_per_a": kt_nm_per_a,
        "inertia_kgm2": inertia_kgm2,
        "damping_nms_per_rad": damping_nms_per_rad,
    }
    if ident.get("target_bandwidth_rad_s") not in (None, ""):
        formula["target_bandwidth_rad_s"] = float(ident["target_bandwidth_rad_s"])
    if ident.get("current_loop_bandwidth_rad_s") not in (None, ""):
        formula["current_loop_bandwidth_rad_s"] = float(ident["current_loop_bandwidth_rad_s"])
    if ident.get("speed_to_current_bandwidth_ratio") not in (None, ""):
        formula["speed_to_current_bandwidth_ratio"] = float(ident["speed_to_current_bandwidth_ratio"])
    if ident.get("target_time_constant_s") not in (None, ""):
        formula["target_time_constant_s"] = float(ident["target_time_constant_s"])

    result = {
        "source": "identification",
        "formula": formula,
        "torque_constant_source": kt_source,
        "feedback_speed_unit": feedback_speed_unit,
        "motor_pole_pairs": float(config.motor.pole_pairs),
        "gear_ratio": float(config.motor.gear_ratio),
        "inertia_identification": inertia_details,
        "damping_identification": {
            "method": damping_method,
            **damping_details,
        },
    }
    parameter_generation_method = str(_cfg_value(ident, "parameter_generation_method", "pole_cancellation_pi"))
    if parameter_generation_method != "pole_cancellation_pi":
        raise ValueError(
            "speed_tuner.identification.parameter_generation_method 当前仅支持 pole_cancellation_pi"
        )
    generated_speed_pi, generated_speed_pi_details = resolve_initial_pi_from_formula(
        formula=formula,
        config=config,
        source="identification.pole_cancellation_pi",
    )
    result["parameter_generation_method"] = parameter_generation_method
    result["generated_speed_pi"] = generated_speed_pi
    result["generated_speed_pi_details"] = generated_speed_pi_details
    write_json(ident_dir / "result.json", result)
    print(
        "[speed_tuner] 辨识完成: "
        f"Kt={kt_nm_per_a:.6f} Nm/A, J={inertia_kgm2:.8e} kg*m^2, B={damping_nms_per_rad:.8e} N*m*s/rad"
    )
    print(
        "[speed_tuner] 极点对消初值: "
        f"kp={generated_speed_pi['s_pid_kp']:.8f}, ki={generated_speed_pi['s_pid_ki']:.8f}"
    )
    return result
