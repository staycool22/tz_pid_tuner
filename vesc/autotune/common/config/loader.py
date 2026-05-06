from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict

from .models import RunConfig


def _load_raw_config(config_path: Path) -> Dict[str, Any]:
    if not config_path.exists():
        raise FileNotFoundError(f"配置文件不存在: {config_path}")

    suffix = config_path.suffix.lower()
    text = config_path.read_text(encoding="utf-8")

    if suffix in {".json"}:
        return json.loads(text)

    if suffix in {".yaml", ".yml"}:
        try:
            import yaml  # type: ignore
        except ImportError as exc:
            raise RuntimeError("读取 YAML 配置需要安装 PyYAML，请先 `pip install pyyaml`") from exc
        return yaml.safe_load(text) or {}

    raise ValueError(f"不支持的配置格式: {config_path.suffix}")


def load_run_config(config_path: str) -> RunConfig:
    data = _load_raw_config(Path(config_path))
    return RunConfig.from_dict(data)

