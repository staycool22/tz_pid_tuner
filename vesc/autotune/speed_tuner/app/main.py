from __future__ import annotations

import argparse
from pathlib import Path

from autotune.common.acquisition.recorder import new_run_dir, write_json
from autotune.common.config.loader import load_run_config
from autotune.common.reporting.report_writer import write_summary_md
from autotune.speed_tuner.tuning.rule_based import run_speed_tuning


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="pass_through 速度环独立整定程序（仅 send_rpm）")
    parser.add_argument(
        "--config",
        default=str(Path(__file__).resolve().parents[3] / "config" / "default_config.yaml"),
        help="配置文件路径（yaml/json）",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    cfg = load_run_config(args.config)
    run_root = (Path(__file__).resolve().parents[3] / cfg.output_root_path).resolve()
    run_dir = new_run_dir(run_root, "speed_tuner")

    best_params, trials = run_speed_tuning(cfg, run_dir=run_dir)
    if not trials:
        raise RuntimeError("未产生有效试验轮次，请检查人工确认流程或设备连接状态。")
    write_json(run_dir / "best_params.json", best_params)
    write_json(
        run_dir / "session_meta.json",
        {
            "tuner": "speed_tuner",
            "vesc_id": cfg.motor.vesc_id,
            "test_rpms": cfg.speed_tuner.test_rpms,
            "command_hz": cfg.speed_tuner.command_hz,
            "read_hz": cfg.speed_tuner.read_hz,
            "reverse_at_pos_limits": cfg.speed_tuner.reverse_at_pos_limits,
            "reverse_margin_deg": cfg.speed_tuner.reverse_margin_deg,
            "reverse_soft_limit_tolerance_deg": cfg.speed_tuner.reverse_soft_limit_tolerance_deg,
            "steady_state_priority": cfg.speed_tuner.steady_state_priority,
            "steady_spike_margin_rpm": cfg.speed_tuner.steady_spike_margin_rpm,
            "param_quantum": cfg.speed_tuner.param_quantum,
            "initial_pi": cfg.speed_tuner.initial_pi,
            "max_iterations": cfg.speed_tuner.max_iterations,
            "improve_threshold": cfg.speed_tuner.improve_threshold,
            "manual_confirm_each_iteration": cfg.speed_tuner.manual_confirm_each_iteration,
            "min_valid_samples": cfg.speed_tuner.min_valid_samples,
            "pos_window_deg": cfg.motor.pos_window_deg,
            "current_forced_zero": True,
            "allowed_cmd": "send_rpm",
            "forbidden_param": "s_pid_kd",
        },
    )
    write_json(run_dir / "trials" / "all_trials.json", {"trials": trials})

    write_summary_md(
        path=run_dir / "summary.md",
        current_symptom=["已完成速度设定点响应采样、自动迭代评分与参数更新建议。"],
        likely_causes=["速度稳态误差、超调与收敛速度主要受 s_pid_kp / s_pid_ki 组合影响。"],
        manual_check_items=["s_pid_min_erpm", "s_pid_allow_braking", "l_current_max_scale", "current 前馈是否为零"],
        suggest_steps=[
            "保持 current=0，按每轮建议参数人工写入 s_pid_kp / s_pid_ki 并复测。",
            "若低速抖动上升或超调增大，优先减小 s_pid_kp，并抑制 s_pid_ki 增长。",
            "确认整个测试期间 pid_pos 始终处于 5°-85°。",
        ],
        acceptance=["速度稳态误差下降", "无明显持续振荡", "峰值电流不越界", "位置保持稳定"],
        best_params=best_params,
    )
    print(f"[speed_tuner] 完成，输出目录: {run_dir}")


if __name__ == "__main__":
    main()
