from __future__ import annotations

import argparse
import csv
import json
import textwrap
from pathlib import Path
from typing import Dict, List, Tuple

try:
    import matplotlib.pyplot as plt
except Exception as exc:  # pragma: no cover
    raise SystemExit(
        "matplotlib 未安装，无法绘图。请先执行: pip install matplotlib"
    ) from exc


def _read_json(path: Path) -> Dict:
    return json.loads(path.read_text(encoding="utf-8"))


def _load_trials(run_dir: Path) -> List[Dict]:
    trials_dir = run_dir / "trials"
    all_trials = trials_dir / "all_trials.json"
    if all_trials.exists():
        payload = _read_json(all_trials)
        return list(payload.get("trials", []))

    metrics_files = sorted(trials_dir.glob("trial_*_metrics.json"))
    out: List[Dict] = []
    for p in metrics_files:
        out.append(_read_json(p))
    return out


def _is_position_run(trials: List[Dict]) -> bool:
    if not trials:
        return True
    agg = trials[0].get("agg_metrics", {})
    return "position_stability" in agg


def _safe_float(v: object, default: float = 0.0) -> float:
    try:
        return float(v)
    except Exception:
        return default


def _format_pid(params: Dict[str, object]) -> str:
    if not params:
        return "N/A"
    chunks: List[str] = []
    for k, v in params.items():
        if isinstance(v, (int, float)):
            chunks.append(f"{k}={float(v):.8g}")
        else:
            chunks.append(f"{k}={v}")
    return ", ".join(chunks)


def _collect_targets(trials: List[Dict], is_position: bool) -> List[float]:
    values: List[float] = []
    for t in trials:
        for d in t.get("details", []):
            if is_position:
                if "target_pos_deg" in d:
                    values.append(_safe_float(d.get("target_pos_deg", 0.0)))
            else:
                if "target_erpm" in d:
                    values.append(_safe_float(d.get("target_erpm", 0.0)))
                elif "target_rpm" in d:
                    values.append(_safe_float(d.get("target_rpm", 0.0)))
    uniq = sorted(set(values))
    return uniq


def _wrap_text(text: str, width: int = 120) -> str:
    if not text:
        return ""
    return "\n".join(textwrap.wrap(text, width=width, break_long_words=False, break_on_hyphens=False))


def _plot_iteration_metrics(run_dir: Path, trials: List[Dict], is_position: bool, output_dir: Path) -> Path:
    iters = [_safe_float(x.get("iteration", i)) for i, x in enumerate(trials)]
    scores = [_safe_float(x.get("score", 0.0)) for x in trials]

    if is_position:
        m1_key, m2_key, m3_key = "position_stability", "step_response", "current_efficiency"
        y1_label = "Position Stability (deg)"
        y2_label = "Step Response (s)"
        y3_label = "Current (A)"
    else:
        m1_key, m2_key, m3_key = "steady_error", "settling_time", "current_efficiency"
        y1_label = "Steady Error (rpm)"
        y2_label = "Settling Time (s)"
        y3_label = "Current (A)"

    m1 = [_safe_float(x.get("agg_metrics", {}).get(m1_key, 0.0)) for x in trials]
    m2 = [_safe_float(x.get("agg_metrics", {}).get(m2_key, 0.0)) for x in trials]
    m3 = [_safe_float(x.get("agg_metrics", {}).get(m3_key, 0.0)) for x in trials]

    targets = _collect_targets(trials, is_position=is_position)
    target_unit = "deg" if is_position else "erpm"
    target_text = ", ".join(f"{x:g}" for x in targets) if targets else "N/A"
    pid_rows = [
        f"iter {int(_safe_float(t.get('iteration', i)))}: {_format_pid(dict(t.get('params_used', {})))}"
        for i, t in enumerate(trials)
    ]
    pid_text = " | ".join(pid_rows) if pid_rows else "N/A"

    fig, axes = plt.subplots(4, 1, figsize=(10, 10), sharex=True)
    fig.suptitle(
        f"Run Metrics: {run_dir.name}\n"
        f"Targets: [{target_text}] {target_unit}\n"
        f"PID:\n{_wrap_text(pid_text, width=110)}"
    )

    axes[0].plot(iters, scores, marker="o")
    axes[0].set_ylabel("Score")
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(iters, m1, marker="o")
    axes[1].set_ylabel(y1_label)
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(iters, m2, marker="o")
    axes[2].set_ylabel(y2_label)
    axes[2].grid(True, alpha=0.3)

    axes[3].plot(iters, m3, marker="o")
    axes[3].set_ylabel(y3_label)
    axes[3].set_xlabel("Iteration")
    axes[3].grid(True, alpha=0.3)

    fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.82))
    output_path = output_dir / "iteration_metrics.png"
    fig.savefig(output_path, dpi=150)
    return output_path


def _read_raw_csv(path: Path) -> Tuple[List[float], List[float], List[float], List[float]]:
    t_s: List[float] = []
    rpm: List[float] = []
    current_a: List[float] = []
    pos_deg: List[float] = []
    with path.open("r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            t_s.append(_safe_float(row.get("t_s", 0.0)))
            rpm.append(_safe_float(row.get("rpm", 0.0)))
            current_a.append(_safe_float(row.get("current_a", 0.0)))
            pos_deg.append(_safe_float(row.get("pos_deg", 0.0)))
    return t_s, rpm, current_a, pos_deg


def _find_raw_csv(trials_dir: Path, iteration: int, target: float, is_position: bool) -> Path | None:
    if is_position:
        preferred = trials_dir / f"trial_{iteration:03d}_{int(target)}deg_raw.csv"
    else:
        preferred = trials_dir / f"trial_{iteration:03d}_{int(target)}erpm_raw.csv"
    if preferred.exists():
        return preferred

    prefix = f"trial_{iteration:03d}_"
    suffixes = ["deg_raw.csv"] if is_position else ["erpm_raw.csv", "rpm_raw.csv"]
    candidates: List[Path] = []
    for suffix in suffixes:
        candidates.extend(trials_dir.glob(f"{prefix}*{suffix}"))
    return candidates[0] if candidates else None


def _plot_overlay_details(
    run_dir: Path,
    selected_trials: List[Dict],
    is_position: bool,
    output_dir: Path,
) -> List[Path]:
    trials_dir = run_dir / "trials"
    out_files: List[Path] = []
    if not selected_trials:
        return out_files

    unit = "deg" if is_position else "erpm"
    y_label = "Position (deg)" if is_position else "ERPM"
    value_key = "pos_deg" if is_position else "rpm"

    target_values: List[float] = []
    for trial in selected_trials:
        for detail in trial.get("details", []):
            if is_position:
                if "target_pos_deg" in detail:
                    target_values.append(_safe_float(detail.get("target_pos_deg", 0.0)))
            else:
                if "target_erpm" in detail:
                    target_values.append(_safe_float(detail.get("target_erpm", 0.0)))
                elif "target_rpm" in detail:
                    target_values.append(_safe_float(detail.get("target_rpm", 0.0)))
    unique_targets = sorted(set(target_values))

    for target in unique_targets:
        fig, ax = plt.subplots(1, 1, figsize=(10, 5))
        pid_rows: List[str] = []
        has_curve = False

        for trial in selected_trials:
            iteration = int(_safe_float(trial.get("iteration", 0)))
            raw_csv = _find_raw_csv(trials_dir, iteration, target, is_position=is_position)
            if raw_csv is None:
                continue
            t_s, rpm, _, pos_deg = _read_raw_csv(raw_csv)
            if not t_s:
                continue
            y = pos_deg if is_position else rpm
            pid_text = _format_pid(dict(trial.get("params_used", {})))
            pid_rows.append(f"iter {iteration}: {pid_text}")
            ax.plot(t_s, y, linewidth=1.2, label=f"iter {iteration}")
            has_curve = True

        if not has_curve:
            plt.close(fig)
            continue

        ax.axhline(target, linestyle="--", linewidth=1.0, label=f"target={target:g} {unit}")
        ax.set_ylabel(y_label)
        ax.set_xlabel("Time (s)")
        ax.grid(True, alpha=0.3)
        ax.legend()

        pid_title = " | ".join(pid_rows) if pid_rows else "N/A"
        fig.suptitle(
            f"Overlay Compare @ target={target:g} {unit}\n"
            f"PID:\n{_wrap_text(pid_title, width=110)}"
        )
        fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.80))

        out = output_dir / f"overlay_target_{int(target)}_{unit}.png"
        fig.savefig(out, dpi=150)
        out_files.append(out)

    return out_files


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="可视化 autotune runs 目录中的整定结果")
    parser.add_argument(
        "--run-dir",
        required=True,
        help="一次 run 的目录，例如: e:/tz_pid_tuner/vesc/runs/position_tuner/2026-04-28_162639",
    )
    parser.add_argument(
        "--iteration",
        type=int,
        default=-1,
        help="要绘制原始响应的迭代号，默认 -1 表示绘制全部迭代",
    )
    parser.add_argument(
        "--no-show",
        action="store_true",
        help="只保存图片，不弹出交互窗口",
    )
    return parser


def main() -> None:
    args = build_parser().parse_args()
    run_dir = Path(args.run_dir).resolve()
    trials = _load_trials(run_dir)
    if not trials:
        raise SystemExit(f"未找到 trial 数据: {run_dir / 'trials'}")

    is_position = _is_position_run(trials)
    output_dir = run_dir / "plots"
    output_dir.mkdir(parents=True, exist_ok=True)

    metric_png = _plot_iteration_metrics(run_dir=run_dir, trials=trials, is_position=is_position, output_dir=output_dir)

    selected_trials: List[Dict] = []
    if args.iteration < 0:
        selected_trials = list(trials)
    else:
        for t in trials:
            if int(_safe_float(t.get("iteration", -1))) == int(args.iteration):
                selected_trials = [t]
                break
        if not selected_trials:
            raise SystemExit(f"未找到 iteration={args.iteration}")

    detail_pngs = _plot_overlay_details(
        run_dir=run_dir,
        selected_trials=selected_trials,
        is_position=is_position,
        output_dir=output_dir,
    )

    print(f"[plot_runs] 已生成: {metric_png}")
    for p in detail_pngs:
        print(f"[plot_runs] 已生成: {p}")

    if not args.no_show:
        plt.show()
    plt.close("all")


if __name__ == "__main__":
    main()
