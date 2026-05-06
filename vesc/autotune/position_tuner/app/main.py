from __future__ import annotations

import argparse
from pathlib import Path

from autotune.common.acquisition.recorder import new_run_dir, write_json
from autotune.common.config.loader import load_run_config
from autotune.common.reporting.report_writer import write_summary_md
from autotune.position_tuner.tuning.rule_based import run_position_tuning


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="pass_through 位置环独立整定程序（支持 UI 同款轨迹+前馈配置）")
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
    run_dir = new_run_dir(run_root, "position_tuner")

    best_params, trials = run_position_tuning(cfg, run_dir=run_dir)
    best_trial = min(trials, key=lambda x: float(x.get("score", 1e9))) if trials else {}
    best_pos_stability = float(best_trial.get("agg_metrics", {}).get("position_stability", 1e9))
    recommend_speed_retune = best_pos_stability > 0.02
    write_json(run_dir / "best_params.json", best_params)
    write_json(
        run_dir / "session_meta.json",
        {
            "tuner": "position_tuner",
            "vesc_id": cfg.motor.vesc_id,
            "target_positions_deg": cfg.position_tuner.target_positions_deg,
            "initial_params": cfg.position_tuner.initial_params,
            "command_hz": cfg.position_tuner.command_hz,
            "read_hz": cfg.position_tuner.read_hz,
            "ui_like_pass_through": cfg.position_tuner.ui_like_pass_through,
            "motion_duration_s": cfg.position_tuner.motion_duration_s,
            "ff_rpm_bias": cfg.position_tuner.ff_rpm_bias,
            "ff_current_a": cfg.position_tuner.ff_current_a,
            "max_iterations": cfg.position_tuner.max_iterations,
            "improve_threshold": cfg.position_tuner.improve_threshold,
            "pos_window_deg": cfg.motor.pos_window_deg,
            "allowed_cmd": "send_pass_through",
        },
    )
    write_json(run_dir / "trials" / "all_trials.json", {"trials": trials})

    likely_causes = ["目标附近回摆与 p_pid_kp / p_pid_kd_proc 组合相关。"]
    suggest_steps = [
        "按每轮建议参数人工写入位置环参数后复测，建议保持与 UI 一致的 pass_through 控制方式。",
        "若接近目标点回摆，先小幅增加 p_pid_kd_proc，再评估是否需要 p_pid_kd。",
        "若保持阶段滑落，再评估 p_pid_ki 是否需要引入。",
    ]
    if recommend_speed_retune:
        likely_causes.append("速度环稳态/动态基础不足，导致位置环难以在 0.02° 误差内收敛。")
        suggest_steps.insert(0, "当前最佳 position_stability 仍大于 0.02°，建议先回退整定速度环，再继续位置环整定。")
        print(
            "[position_tuner] 提示：当前最佳位置误差仍大于 0.02°，建议先回退整定速度环，再继续位置环。"
        )

    write_summary_md(
        path=run_dir / "summary.md",
        current_symptom=["已完成小步进和保持数据采样、自动迭代评分与参数更新建议。"],
        likely_causes=likely_causes,
        manual_check_items=["p_pid_ang_div", "p_pid_gain_dec_angle", "current 前馈是否为零", "s_pid_min_erpm"],
        suggest_steps=suggest_steps,
        acceptance=["小步进可稳定回位", "目标附近无明显往返摆动", "轻载保持不滑落", "测试期间 pid_pos 始终在 5°-85°"],
        best_params=best_params,
    )
    print(f"[position_tuner] 完成，输出目录: {run_dir}")


if __name__ == "__main__":
    main()
