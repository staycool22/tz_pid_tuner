from __future__ import annotations

from pathlib import Path
from typing import Dict, List


def write_summary_md(
    path: Path,
    current_symptom: List[str],
    likely_causes: List[str],
    manual_check_items: List[str],
    suggest_steps: List[str],
    acceptance: List[str],
    best_params: Dict[str, float],
) -> None:
    lines = [
        "当前现象:",
        *[f"- {x}" for x in current_symptom],
        "",
        "最可能原因:",
        *[f"- {x}" for x in likely_causes],
        "",
        "需人工核查的配置项:",
        *[f"- {x}" for x in manual_check_items],
        "",
        "建议修改顺序:",
    ]
    lines.extend([f"{i + 1}. {step}" for i, step in enumerate(suggest_steps)])
    lines.extend(
        [
            "",
            "修改后的验收标准:",
            *[f"- {x}" for x in acceptance],
            "",
            "推荐参数:",
        ]
    )
    for k, v in best_params.items():
        lines.append(f"- {k}: {v}")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")

