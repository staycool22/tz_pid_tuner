from __future__ import annotations

import argparse
from pathlib import Path

from autotune.common.acquisition.recorder import new_run_dir, write_json
from autotune.common.config.loader import load_run_config


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="current 前馈扫描工具（需速度环和位置环已冻结）")
    parser.add_argument(
        "--config",
        default=str(Path(__file__).resolve().parents[3] / "config" / "default_config.yaml"),
        help="配置文件路径（yaml/json）",
    )
    parser.add_argument("--currents", nargs="+", type=float, default=[0.2, 0.4, 0.6], help="前馈扫描电流(A)")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    cfg = load_run_config(args.config)
    run_root = (Path(__file__).resolve().parents[3] / cfg.output_root_path).resolve()
    run_dir = new_run_dir(run_root, "feedforward_tool")
    write_json(
        run_dir / "summary.json",
        {
            "status": "placeholder",
            "message": "前馈工具已预留。执行时请从小 current 起步，并人工复核位置保持和过推风险。",
            "candidate_currents_a": args.currents,
        },
    )
    print(f"[feedforward_tool] 已生成占位输出: {run_dir}")


if __name__ == "__main__":
    main()

