from __future__ import annotations

import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

from autotune.common.acquisition.recorder import TrialRecorder, write_json
from autotune.common.analysis.metrics import evaluate_position_trial, weighted_score
from autotune.common.config.models import RunConfig
from autotune.common.io.vesc_client import VESCBusClient
from autotune.common.safety.guard import SafetyGuard, SafetyLimits, SafetyViolation
from autotune.position_tuner.tuning.initial_params import resolve_initial_position_params


def _run_single_position_trial(
    client: VESCBusClient,
    guard: SafetyGuard,
    target_pos_deg: float,
    trial_seconds: float,
    sample_timeout_s: float,
    command_hz: float,
    read_hz: float,
    ui_like_pass_through: bool,
    motion_duration_s: float,
    ff_rpm_bias: float,
    ff_current_a: float,
) -> TrialRecorder:
    recorder = TrialRecorder()
    read_period_s = 1.0 / max(1.0, float(read_hz))
    send_period_s = 1.0 / max(1.0, float(command_hz))
    max_drain_reads = 50

    def _clamp_pass_through(pos_deg: float, rpm: float, current_a: float) -> Tuple[float, float, float]:
        # 与 UI 保持一致：pass_through 底层按 uint16 编码，负值会回绕，这里做安全裁剪。
        safe_pos = min(max(float(pos_deg), 0.0), 655.35)
        safe_rpm = min(max(float(rpm), 0.0), 65535.0)
        safe_cur = min(max(float(current_a), 0.0), 65.535)
        return safe_pos, safe_rpm, safe_cur

    def _drain_feedback(first_timeout_s: float, max_reads: int, record: bool) -> Optional[float]:
        latest_pos: Optional[float] = None
        reads = 0
        while reads < max_reads:
            reads += 1
            timeout = first_timeout_s if reads == 1 else 0.0
            pack = client.receive_decode(timeout)
            if pack is None:
                break
            try:
                guard.validate_feedback(pid_pos_now_deg=float(pack.pid_pos_now), current_a=float(pack.current))
            except SafetyViolation as exc:
                raise exc
            pos_now = guard.normalize_feedback_angle(float(pack.pid_pos_now))
            if record:
                recorder.append(rpm=float(pack.rpm), current_a=float(pack.current), pos_deg=pos_now)
            latest_pos = pos_now
        return latest_pos

    # 新目标开始前先清掉上一目标遗留反馈，但不把这段过渡数据记入当前 trial。
    latest_pos = _drain_feedback(
        first_timeout_s=min(sample_timeout_s, read_period_s),
        max_reads=max_drain_reads,
        record=False,
    )
    cmd_pos_seed = float(latest_pos) if latest_pos is not None else float(target_pos_deg)
    start = time.perf_counter()
    next_send_tick = start
    plan_start_pos = float(cmd_pos_seed)
    plan_start_t = start
    try:
        while time.perf_counter() - start <= trial_seconds:
            now = time.perf_counter()
            if now >= next_send_tick:
                cmd_pos = float(target_pos_deg)
                cmd_rpm = 0.0
                cmd_cur = 0.0
                if ui_like_pass_through:
                    duration_s = max(float(motion_duration_s), 1e-6)
                    tau = min(max((now - plan_start_t) / duration_s, 0.0), 1.0)
                    delta_deg = float(target_pos_deg) - float(plan_start_pos)
                    s = 10.0 * tau**3 - 15.0 * tau**4 + 6.0 * tau**5
                    cmd_pos = float(plan_start_pos) + delta_deg * s
                    if 0.0 < tau < 1.0:
                        ds_dt = (30.0 * tau**2 - 60.0 * tau**3 + 30.0 * tau**4) / duration_s
                        cmd_rpm = abs(delta_deg * ds_dt) * (60.0 / 360.0)
                    else:
                        cmd_rpm = 0.0
                    cmd_rpm += float(ff_rpm_bias)
                    cmd_cur = float(ff_current_a)
                safe_pos, safe_rpm, safe_cur = _clamp_pass_through(cmd_pos, cmd_rpm, cmd_cur)
                client.send_pass_through(pos_deg=safe_pos, rpm=safe_rpm, current_a=safe_cur)

                latest_pos = _drain_feedback(
                    first_timeout_s=min(sample_timeout_s, read_period_s),
                    max_reads=max_drain_reads,
                    record=True,
                )
                next_send_tick += send_period_s
                remain = next_send_tick - time.perf_counter()
                if remain > 0:
                    time.sleep(min(remain, 0.01))
                else:
                    next_send_tick = time.perf_counter()
            else:
                time.sleep(min(0.001, max(0.0, next_send_tick - now)))
    finally:
        client.send_pass_through(pos_deg=target_pos_deg, rpm=0.0, current_a=0.0)
    return recorder


def _mean(values: List[float]) -> float:
    return float(sum(values) / len(values)) if values else 0.0


def _clamp(value: float, v_min: float, v_max: float) -> float:
    return max(v_min, min(v_max, value))


def _build_initial_params(config: RunConfig) -> Tuple[Dict[str, float], Dict[str, object]]:
    initial_params, details = resolve_initial_position_params(config)
    if "p_pid_kp" not in initial_params or "p_pid_kd_proc" not in initial_params:
        raise ValueError("位置环初始参数必须包含 p_pid_kp 和 p_pid_kd_proc")
    return initial_params, details


def _next_position_params(
    current_params: Dict[str, float],
    agg: Dict[str, float],
    config: RunConfig,
) -> Dict[str, float]:
    step_kp = float(config.position_tuner.step_scale.get("kp", 0.10))
    step_kd = float(config.position_tuner.step_scale.get("kd_proc", 0.20))
    limits = config.position_tuner.param_limits
    trial_seconds = float(config.position_tuner.trial_seconds)

    kp = float(current_params["p_pid_kp"])
    kd_proc = float(current_params["p_pid_kd_proc"])
    position_stability = float(agg.get("position_stability", 0.0))
    rebound = float(agg.get("rebound", 0.0))
    step_response = float(agg.get("step_response", trial_seconds))
    current_eff = float(agg.get("current_efficiency", 0.0))

    rebound_high = rebound > 2.0
    too_slow = step_response > trial_seconds * 0.75
    hold_error_high = position_stability > 0.01

    if rebound_high:
        kd_proc *= 1.0 + step_kd
        kp *= 1.0 - step_kp * 0.4
    else:
        if hold_error_high:
            kp *= 1.0 + step_kp
        elif too_slow:
            kp *= 1.0 + step_kp * 0.6
        else:
            kd_proc *= 1.0 - step_kd * 0.2

    if current_eff > config.motor.peak_current_limit_a * 0.70:
        kp *= 1.0 - step_kp * 0.5

    kp = _clamp(kp, float(limits["kp_min"]), float(limits["kp_max"]))
    kd_proc = _clamp(kd_proc, float(limits["kd_proc_min"]), float(limits["kd_proc_max"]))
    return {"p_pid_kp": kp, "p_pid_kd_proc": kd_proc}


def run_position_tuning(
    config: RunConfig,
    run_dir: Path,
) -> Tuple[Dict[str, float], List[Dict[str, float]], Dict[str, object]]:
    limits = SafetyLimits(
        pos_min_deg=float(config.motor.pos_window_deg[0]),
        pos_max_deg=float(config.motor.pos_window_deg[1]),
        peak_current_limit_a=float(config.motor.peak_current_limit_a),
        enforce_position_limits=bool(config.motor.uses_position_limits),
    )
    guard = SafetyGuard(limits)

    all_trials: List[Dict[str, float]] = []
    best_params: Dict[str, float] = {}
    best_score = float("inf")
    current_params, initial_params_details = _build_initial_params(config)
    no_improve_rounds = 0

    with VESCBusClient(config.can, config.motor) as client:
        print(
            "[position_tuner] 初始参数来源: "
            f"{initial_params_details.get('source', 'unknown')}, "
            f"kp={current_params['p_pid_kp']:.8f}, kd_proc={current_params['p_pid_kd_proc']:.8f}"
        )
        for idx in range(int(config.position_tuner.max_iterations)):
            print(
                f"[position_tuner][iter {idx}] params: "
                f"kp={current_params['p_pid_kp']:.8f}, kd_proc={current_params['p_pid_kd_proc']:.8f}"
            )
            if bool(config.position_tuner.auto_write_params):
                wrote_params = client.write_position_params(
                    kp=float(current_params["p_pid_kp"]),
                    kd_proc=float(current_params["p_pid_kd_proc"]),
                    save_to_flash=False,
                )
                if wrote_params:
                    print("[position_tuner] 已自动写入 p_pid_kp / p_pid_kd_proc。")
                else:
                    print("[position_tuner] 当前 p_pid_kp / p_pid_kd_proc 与上次写入一致，跳过重复写入。")
            elif config.position_tuner.manual_confirm_each_iteration:
                user_in = input(
                    "[position_tuner] 请先把以上参数人工写入 VESC，再按回车继续；输入 q 终止: "
                ).strip().lower()
                if user_in == "q":
                    print("[position_tuner] 用户终止迭代。")
                    break
            # 当前版本 PID 写入默认人工执行：每轮记录建议参数并据此采样评分。
            trial_metrics_list: List[Dict[str, float]] = []
            for target_pos in config.position_tuner.target_positions_deg:
                try:
                    rec = _run_single_position_trial(
                        client=client,
                        guard=guard,
                        target_pos_deg=target_pos,
                        trial_seconds=config.position_tuner.trial_seconds,
                        sample_timeout_s=config.position_tuner.sample_timeout_s,
                        command_hz=config.position_tuner.command_hz,
                        read_hz=config.position_tuner.read_hz,
                        ui_like_pass_through=bool(config.position_tuner.ui_like_pass_through),
                        motion_duration_s=float(config.position_tuner.motion_duration_s),
                        ff_rpm_bias=float(config.position_tuner.ff_rpm_bias),
                        ff_current_a=float(config.position_tuner.ff_current_a),
                    )
                except SafetyViolation as exc:
                    client.send_pass_through(pos_deg=target_pos, rpm=0.0, current_a=0.0)
                    raise RuntimeError(f"位置环试验触发安全保护: {exc}") from exc

                metrics = evaluate_position_trial(rec.samples, target_deg=target_pos)
                score = weighted_score(metrics, config.position_tuner.kpi_weights)
                trial_metrics_list.append({"target_pos_deg": target_pos, "score": score, **metrics})
                rec.dump_csv(run_dir / "trials" / f"trial_{idx:03d}_{int(target_pos)}deg_raw.csv")

            candidate_score = sum(x["score"] for x in trial_metrics_list) / max(1, len(trial_metrics_list))
            agg_metrics = {
                "position_stability": _mean([x["position_stability"] for x in trial_metrics_list]),
                "load_hold": _mean([x["load_hold"] for x in trial_metrics_list]),
                "step_response": _mean([x["step_response"] for x in trial_metrics_list]),
                "rebound": _mean([x["rebound"] for x in trial_metrics_list]),
                "current_efficiency": _mean([x["current_efficiency"] for x in trial_metrics_list]),
            }
            next_params = _next_position_params(current_params=current_params, agg=agg_metrics, config=config)
            trial_info = {
                "iteration": idx,
                "params_used": dict(current_params),
                "score": candidate_score,
                "agg_metrics": agg_metrics,
                "next_params_suggestion": next_params,
                "details": trial_metrics_list,
            }
            all_trials.append(trial_info)
            write_json(run_dir / "trials" / f"trial_{idx:03d}_metrics.json", trial_info)
            print(
                f"[position_tuner][iter {idx}] score={candidate_score:.6f}, "
                f"next: kp={next_params['p_pid_kp']:.8f}, kd_proc={next_params['p_pid_kd_proc']:.8f}"
            )

            if candidate_score < best_score:
                improve = best_score - candidate_score if best_score < float("inf") else float("inf")
                best_score = candidate_score
                best_params = dict(current_params)
                no_improve_rounds = 0 if improve >= float(config.position_tuner.improve_threshold) else no_improve_rounds + 1
            else:
                no_improve_rounds += 1

            current_params = next_params
            if no_improve_rounds >= 2:
                print("[position_tuner] 连续改进不足，提前停止。")
                if float(agg_metrics.get("position_stability", 1e9)) > 0.01:
                    print("[position_tuner] 位置误差仍大于 0.02°，建议先回退并重新整定速度环后再继续位置环。")
                break

        if bool(config.position_tuner.auto_write_params) and best_params:
            client.write_position_params(
                kp=float(best_params["p_pid_kp"]),
                kd_proc=float(best_params["p_pid_kd_proc"]),
                save_to_flash=True,
            )
            print(
                "[position_tuner] 已将历史最优参数保存写入: "
                f"kp={best_params['p_pid_kp']:.8f}, kd_proc={best_params['p_pid_kd_proc']:.8f}"
            )

    return best_params, all_trials, initial_params_details
