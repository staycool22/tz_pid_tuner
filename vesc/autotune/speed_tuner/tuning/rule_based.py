from __future__ import annotations

import time
import threading
from pathlib import Path
from typing import Dict, List, Tuple

from autotune.common.acquisition.recorder import TrialRecorder, write_json
from autotune.common.analysis.metrics import evaluate_speed_trial, weighted_score
from autotune.common.config.models import RunConfig
from autotune.common.io.vesc_client import VESCBusClient
from autotune.common.safety.guard import SafetyGuard, SafetyLimits, SafetyViolation
from autotune.speed_tuner.tuning.initial_pi import resolve_initial_pi


def _run_single_speed_trial(
    client: VESCBusClient,
    guard: SafetyGuard,
    target_rpm: float,
    trial_seconds: float,
    sample_timeout_s: float,
    command_hz: float,
    read_hz: float,
    pos_min_deg: float,
    pos_max_deg: float,
    reverse_at_pos_limits: bool,
    reverse_margin_deg: float,
) -> TrialRecorder:
    recorder = TrialRecorder()
    read_period_s = 1.0 / max(1.0, float(read_hz))
    send_period_s = 1.0 / max(1.0, float(command_hz))
    cmd_abs_rpm = abs(float(target_rpm))
    cmd_rpm = cmd_abs_rpm if float(target_rpm) >= 0 else -cmd_abs_rpm
    pos_rev_min = pos_min_deg + max(0.0, float(reverse_margin_deg))
    pos_rev_max = pos_max_deg - max(0.0, float(reverse_margin_deg))
    latest_pos = 0.0
    has_latest_pos = False
    state_lock = threading.Lock()
    stop_event = threading.Event()
    violation: List[SafetyViolation] = []

    def _monitor_feedback() -> None:
        nonlocal latest_pos, has_latest_pos
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
            with state_lock:
                latest_pos = pos_now
                has_latest_pos = True

    monitor = threading.Thread(target=_monitor_feedback, name="speed-feedback-monitor", daemon=True)
    monitor.start()
    start = time.perf_counter()
    next_send_tick = start
    try:
        while time.perf_counter() - start <= trial_seconds and not stop_event.is_set():
            now = time.perf_counter()
            if reverse_at_pos_limits and cmd_abs_rpm > 0.0:
                with state_lock:
                    pos_now = latest_pos
                    pos_ready = has_latest_pos
                if pos_ready:
                    switched = False
                    if cmd_rpm > 0.0 and pos_now >= pos_rev_max:
                        cmd_rpm = -cmd_abs_rpm
                        switched = True
                    elif cmd_rpm < 0.0 and pos_now <= pos_rev_min:
                        cmd_rpm = cmd_abs_rpm
                        switched = True
                    if switched:
                        # 命中限位反向阈值后立刻下发反向速度，避免 10Hz 发送节拍带来的方向切换滞后。
                        client.send_rpm(cmd_rpm)
                        next_send_tick = time.perf_counter() + send_period_s
            if now >= next_send_tick:
                client.send_rpm(cmd_rpm)
                while next_send_tick <= now:
                    next_send_tick += send_period_s
            else:
                time.sleep(min(0.001, max(0.0, next_send_tick - now)))
    finally:
        stop_event.set()
        monitor.join(timeout=max(0.5, sample_timeout_s * 2.0))
        client.send_rpm(0.0)
    if violation:
        raise violation[0]
    return recorder


def _mean(values: List[float]) -> float:
    return float(sum(values) / len(values)) if values else 0.0


def _clamp(value: float, v_min: float, v_max: float) -> float:
    return max(v_min, min(v_max, value))


def _quantize(value: float, quantum: float) -> float:
    if quantum <= 0.0:
        return value
    return round(value / quantum) * quantum


def _is_close_pi(a: Dict[str, float], b: Dict[str, float], quantum_kp: float, quantum_ki: float) -> bool:
    return (
        abs(float(a["s_pid_kp"]) - float(b["s_pid_kp"])) <= max(1e-12, quantum_kp * 0.5)
        and abs(float(a["s_pid_ki"]) - float(b["s_pid_ki"])) <= max(1e-12, quantum_ki * 0.5)
    )


def _small_step_towards(
    base_pi: Dict[str, float],
    target_pi: Dict[str, float],
    config: RunConfig,
    ratio: float = 0.25,
) -> Dict[str, float]:
    quantum_kp = float(config.speed_tuner.param_quantum.get("kp", 0.00001))
    quantum_ki = float(config.speed_tuner.param_quantum.get("ki", 0.00001))
    limits = config.speed_tuner.param_limits

    def _step(base: float, target: float, quantum: float) -> float:
        if target == base:
            return base
        delta = (target - base) * ratio
        if abs(delta) < quantum:
            delta = quantum if target > base else -quantum
        return base + delta

    kp = _step(float(base_pi["s_pid_kp"]), float(target_pi["s_pid_kp"]), quantum_kp)
    ki = _step(float(base_pi["s_pid_ki"]), float(target_pi["s_pid_ki"]), quantum_ki)
    kp = _quantize(_clamp(kp, float(limits["kp_min"]), float(limits["kp_max"])), quantum_kp)
    ki = _quantize(_clamp(ki, float(limits["ki_min"]), float(limits["ki_max"])), quantum_ki)
    kp = _clamp(kp, float(limits["kp_min"]), float(limits["kp_max"]))
    ki = _clamp(ki, float(limits["ki_min"]), float(limits["ki_max"]))
    return {"s_pid_kp": kp, "s_pid_ki": ki}


def _build_initial_pi(config: RunConfig) -> Tuple[Dict[str, float], Dict[str, object]]:
    initial_pi, details = resolve_initial_pi(config)
    if "s_pid_kp" not in initial_pi or "s_pid_ki" not in initial_pi:
        raise ValueError("速度环初始 PI 必须包含 s_pid_kp 和 s_pid_ki")
    return initial_pi, details


def _resolve_speed_tuning_phases(config: RunConfig) -> List[Dict[str, object]]:
    coarse_iterations = max(0, int(getattr(config.speed_tuner, "coarse_iterations", 0)))
    fine_iterations = max(0, int(getattr(config.speed_tuner, "fine_iterations", 0)))
    if coarse_iterations <= 0 and fine_iterations <= 0:
        return [
            {
                "name": "single",
                "iterations": max(1, int(config.speed_tuner.max_iterations)),
                "step_scale": dict(config.speed_tuner.step_scale),
                "restart_from_best": False,
            }
        ]

    phases: List[Dict[str, object]] = []
    if coarse_iterations > 0:
        phases.append(
            {
                "name": "coarse",
                "iterations": coarse_iterations,
                "step_scale": dict(config.speed_tuner.step_scale),
                "restart_from_best": False,
            }
        )
    if fine_iterations > 0:
        phases.append(
            {
                "name": "fine",
                "iterations": fine_iterations,
                "step_scale": dict(config.speed_tuner.fine_step_scale),
                "restart_from_best": bool(phases),
            }
        )
    return phases


def _next_pi(
    current_pi: Dict[str, float],
    agg: Dict[str, float],
    config: RunConfig,
    step_scale_override: Dict[str, float] | None = None,
) -> Dict[str, float]:
    step_scale = step_scale_override or config.speed_tuner.step_scale
    step_kp = float(step_scale.get("kp", 0.10))
    step_ki = float(step_scale.get("ki", 0.15))
    quantum_kp = float(config.speed_tuner.param_quantum.get("kp", 0.00001))
    quantum_ki = float(config.speed_tuner.param_quantum.get("ki", 0.00001))
    limits = config.speed_tuner.param_limits
    trial_seconds = float(config.speed_tuner.trial_seconds)
    avg_target_abs = max(1.0, _mean([abs(x) for x in config.speed_tuner.test_rpms]))

    kp = float(current_pi["s_pid_kp"])
    ki = float(current_pi["s_pid_ki"])
    steady_error = float(agg.get("steady_error", 0.0))
    signed_error = float(agg.get("signed_steady_error", 0.0))
    overshoot = float(agg.get("overshoot", 0.0))
    settling_time = float(agg.get("settling_time", trial_seconds))
    current_eff = float(agg.get("current_efficiency", 0.0))

    high_overshoot = overshoot > max(40.0, avg_target_abs * 0.08)
    if bool(config.speed_tuner.steady_state_priority):
        # 稳态优先模式：参数更新阶段忽略超调峰值，避免过度压低 kp。
        high_overshoot = False
    steady_error_high = steady_error > max(30.0, avg_target_abs * 0.05)
    steady_oscillation = steady_error_high and (
        abs(signed_error) <= max(20.0, avg_target_abs * 0.02)
    )
    steady_overshoot = signed_error > max(20.0, avg_target_abs * 0.04)
    too_slow = settling_time > trial_seconds * 0.75

    if steady_error_high:
        # 优先处理稳态误差：误差偏大时先增 kp/ki，再考虑其它约束。
        kp *= 1.0 + step_kp * 0.6
        ki *= 1.0 + step_ki
        if steady_oscillation:
            kp *= 1.0 + step_kp * 0.3
        if too_slow:
            kp *= 1.0 + step_kp * 0.3
    elif high_overshoot:
        kp *= 1.0 - step_kp
        ki *= 1.0 - step_ki * 0.5
    elif too_slow:
        kp *= 1.0 + step_kp * 0.5

    if steady_overshoot:
        # 用 signed_steady_error 约束稳态超调：超目标时回拉 kp/ki，抑制继续抬升引发的过冲与振荡。
        kp *= 1.0 - step_kp * 0.4
        ki *= 1.0 - step_ki * 0.6

    if current_eff > config.motor.peak_current_limit_a * 0.70:
        kp *= 1.0 - step_kp * 0.5

    kp = _clamp(kp, float(limits["kp_min"]), float(limits["kp_max"]))
    ki = _clamp(ki, float(limits["ki_min"]), float(limits["ki_max"]))
    kp = _quantize(kp, quantum_kp)
    ki = _quantize(ki, quantum_ki)
    kp = _clamp(kp, float(limits["kp_min"]), float(limits["kp_max"]))
    ki = _clamp(ki, float(limits["ki_min"]), float(limits["ki_max"]))
    return {"s_pid_kp": kp, "s_pid_ki": ki}


def run_speed_tuning(
    config: RunConfig,
    run_dir: Path,
) -> Tuple[Dict[str, float], List[Dict[str, float]], Dict[str, object]]:
    use_position_limits = bool(config.motor.uses_position_limits)
    pos_min_cmd = float(config.motor.pos_window_deg[0])
    pos_max_cmd = float(config.motor.pos_window_deg[1])
    soft_tol = (
        float(config.speed_tuner.reverse_soft_limit_tolerance_deg)
        if use_position_limits and bool(config.speed_tuner.reverse_at_pos_limits)
        else 0.0
    )
    limits = SafetyLimits(
        pos_min_deg=pos_min_cmd - soft_tol,
        pos_max_deg=pos_max_cmd + soft_tol,
        peak_current_limit_a=float(config.motor.peak_current_limit_a),
        enforce_position_limits=use_position_limits,
    )
    guard = SafetyGuard(limits)

    all_trials: List[Dict[str, float]] = []
    best_params: Dict[str, float] = {}
    best_score = float("inf")
    current_pi, initial_pi_details = _build_initial_pi(config)
    quantum_kp = float(config.speed_tuner.param_quantum.get("kp", 0.00001))
    quantum_ki = float(config.speed_tuner.param_quantum.get("ki", 0.00001))
    tried_history: List[Dict[str, object]] = []
    phases = _resolve_speed_tuning_phases(config)
    total_iterations = sum(int(phase["iterations"]) for phase in phases)

    with VESCBusClient(config.can, config.motor) as client:
        print(
            "[speed_tuner] 初始 PI 来源: "
            f"{initial_pi_details.get('source', 'unknown')}, "
            f"kp={current_pi['s_pid_kp']:.8f}, ki={current_pi['s_pid_ki']:.8f}"
        )
        global_iter = 0
        for phase_index, phase in enumerate(phases):
            phase_name = str(phase["name"])
            phase_iterations = int(phase["iterations"])
            phase_step_scale = dict(phase["step_scale"])
            if phase_iterations <= 0:
                continue
            if phase_index > 0 and bool(phase.get("restart_from_best")) and best_params:
                current_pi = dict(best_params)
                print(
                    f"[speed_tuner] 切换到{phase_name}阶段，"
                    f"从历史最优参数继续: kp={current_pi['s_pid_kp']:.8f}, ki={current_pi['s_pid_ki']:.8f}"
                )
            print(
                f"[speed_tuner] 阶段 {phase_name}: {phase_iterations} 轮，"
                f"step_scale(kp={phase_step_scale.get('kp')}, ki={phase_step_scale.get('ki')})"
            )
            for phase_iter in range(phase_iterations):
                idx = global_iter
                print(
                    f"[speed_tuner][iter {idx}][{phase_name} {phase_iter + 1}/{phase_iterations}] "
                    f"params: kp={current_pi['s_pid_kp']:.8f}, ki={current_pi['s_pid_ki']:.8f}"
                )
                if bool(config.speed_tuner.auto_write_params):
                    wrote_params = client.write_speed_pi(
                        kp=float(current_pi["s_pid_kp"]),
                        ki=float(current_pi["s_pid_ki"]),
                        save_to_flash=False,
                    )
                    if wrote_params:
                        print("[speed_tuner] 已自动写入 s_pid_kp / s_pid_ki。")
                    else:
                        print("[speed_tuner] 当前 s_pid_kp / s_pid_ki 与上次写入一致，跳过重复写入。")
                elif config.speed_tuner.manual_confirm_each_iteration:
                    user_in = input(
                        "[speed_tuner] 请先把以上参数人工写入 VESC，再按回车继续；输入 q 终止: "
                    ).strip().lower()
                    if user_in == "q":
                        print("[speed_tuner] 用户终止迭代。")
                        break

                # 当前版本 PID 写入默认人工执行：每轮记录建议参数并据此采样评分。
                trial_metrics_list: List[Dict[str, float]] = []
                for target_rpm in config.speed_tuner.test_rpms:
                    try:
                        rec = _run_single_speed_trial(
                            client=client,
                            guard=guard,
                            target_rpm=target_rpm,
                            trial_seconds=config.speed_tuner.trial_seconds,
                            sample_timeout_s=config.speed_tuner.sample_timeout_s,
                            command_hz=config.speed_tuner.command_hz,
                            read_hz=config.speed_tuner.read_hz,
                            pos_min_deg=pos_min_cmd,
                            pos_max_deg=pos_max_cmd,
                            reverse_at_pos_limits=use_position_limits and bool(config.speed_tuner.reverse_at_pos_limits),
                            reverse_margin_deg=float(config.speed_tuner.reverse_margin_deg),
                        )
                    except SafetyViolation as exc:
                        client.stop_motion()
                        raise RuntimeError(f"速度环试验触发安全保护: {exc}") from exc
                    if len(rec.samples) < int(config.speed_tuner.min_valid_samples):
                        raise RuntimeError(
                            "速度环试验有效采样不足。"
                            f" target_rpm={target_rpm}, valid_samples={len(rec.samples)},"
                            f" min_required={config.speed_tuner.min_valid_samples}。"
                            " 请检查: vesc_id 是否正确、VESC 是否开启状态上报、CAN 收发是否通、sample_timeout_s 是否过小。"
                        )

                    metrics = evaluate_speed_trial(
                        rec.samples,
                        target_rpm=target_rpm,
                        use_abs_rpm=True,
                        steady_spike_margin_rpm=float(config.speed_tuner.steady_spike_margin_rpm),
                    )
                    score = weighted_score(metrics, config.speed_tuner.kpi_weights)
                    trial_metrics_list.append({"target_rpm": target_rpm, "score": score, **metrics})
                    rec.dump_csv(run_dir / "trials" / f"trial_{idx:03d}_{int(target_rpm)}rpm_raw.csv")

                candidate_score = sum(x["score"] for x in trial_metrics_list) / max(1, len(trial_metrics_list))
                agg_metrics = {
                    "steady_error": _mean([x["steady_error"] for x in trial_metrics_list]),
                    "signed_steady_error": _mean([x["signed_steady_error"] for x in trial_metrics_list]),
                    "overshoot": _mean([x["overshoot"] for x in trial_metrics_list]),
                    "settling_time": _mean([x["settling_time"] for x in trial_metrics_list]),
                    "current_efficiency": _mean([x["current_efficiency"] for x in trial_metrics_list]),
                }
                proposed_next_pi = _next_pi(
                    current_pi=current_pi,
                    agg=agg_metrics,
                    config=config,
                    step_scale_override=phase_step_scale,
                )
                if candidate_score < best_score:
                    best_score = candidate_score
                    best_params = dict(current_pi)
                next_pi = dict(proposed_next_pi)
                revisited_candidate = False
                if tried_history:
                    for ratio in (0.25, 0.5, 1.0, 1.5, 2.0):
                        candidate = _small_step_towards(
                            base_pi=current_pi,
                            target_pi=proposed_next_pi,
                            config=config,
                            ratio=ratio,
                        )
                        already_tried = any(
                            _is_close_pi(
                                candidate,
                                item["params"],
                                quantum_kp=quantum_kp,
                                quantum_ki=quantum_ki,
                            )
                            for item in tried_history
                        )
                        if not already_tried:
                            next_pi = candidate
                            break
                    else:
                        revisited_candidate = True
                        next_pi = dict(proposed_next_pi)

                trial_info = {
                    "iteration": idx,
                    "stage": phase_name,
                    "stage_iteration": phase_iter,
                    "stage_iterations": phase_iterations,
                    "stage_step_scale": phase_step_scale,
                    "params_used": dict(current_pi),
                    "score": candidate_score,
                    "agg_metrics": agg_metrics,
                    "next_params_suggestion": next_pi,
                    "proposed_next_params": proposed_next_pi,
                    "revisited_candidate": revisited_candidate,
                    "details": trial_metrics_list,
                }
                all_trials.append(trial_info)
                tried_history.append({"params": dict(current_pi), "score": candidate_score})
                write_json(run_dir / "trials" / f"trial_{idx:03d}_metrics.json", trial_info)
                is_last_iteration = idx >= total_iterations - 1
                is_last_phase_iteration = phase_iter >= phase_iterations - 1
                if is_last_iteration:
                    print(
                        f"[speed_tuner][iter {idx}] score={candidate_score:.6f}, "
                        f"{total_iterations}轮两阶段探索完成。"
                    )
                    print(
                        "[speed_tuner] 历史最优参数: "
                        f"kp={best_params['s_pid_kp']:.8f}, ki={best_params['s_pid_ki']:.8f}"
                    )
                elif is_last_phase_iteration:
                    print(
                        f"[speed_tuner][iter {idx}] score={candidate_score:.6f}, "
                        f"{phase_name}阶段完成，当前历史最优: "
                        f"kp={best_params['s_pid_kp']:.8f}, ki={best_params['s_pid_ki']:.8f}"
                    )
                elif revisited_candidate:
                    print(
                        f"[speed_tuner][iter {idx}] score={candidate_score:.6f}, "
                        f"当前方向候选已探索，继续沿误差趋势外推: "
                        f"kp={next_pi['s_pid_kp']:.8f}, ki={next_pi['s_pid_ki']:.8f}"
                    )
                else:
                    print(
                        f"[speed_tuner][iter {idx}] score={candidate_score:.6f}, "
                        f"next: kp={next_pi['s_pid_kp']:.8f}, ki={next_pi['s_pid_ki']:.8f}"
                    )

                current_pi = next_pi
                global_iter += 1
            else:
                continue
            break

        if bool(config.speed_tuner.auto_write_params) and best_params:
            client.write_speed_pi(
                kp=float(best_params["s_pid_kp"]),
                ki=float(best_params["s_pid_ki"]),
                save_to_flash=True,
            )
            print(
                "[speed_tuner] 已将历史最优参数保存写入: "
                f"kp={best_params['s_pid_kp']:.8f}, ki={best_params['s_pid_ki']:.8f}"
            )

    return best_params, all_trials, initial_pi_details
