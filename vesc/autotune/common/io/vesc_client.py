from __future__ import annotations

import sys
import threading
from pathlib import Path
from typing import Optional

from autotune.common.config.models import CANConfig, MotorConfig


def _ensure_tzcan_import_path() -> None:
    repo_root = Path(__file__).resolve().parents[4]
    can_box_path = repo_root / "can_box"
    if str(can_box_path) not in sys.path:
        sys.path.insert(0, str(can_box_path))


_ensure_tzcan_import_path()

from tzcan import CANMessageTransmitter  # noqa: E402
from tzcan.protocols.vesc import VESC_CAN, VESC_PACK  # noqa: E402


class VESCBusClient:
    """统一封装 VESC 通讯与硬件资源生命周期。"""

    def __init__(self, can_cfg: CANConfig, motor_cfg: MotorConfig):
        self.can_cfg = can_cfg
        self.motor_cfg = motor_cfg
        self._tx_cls = None
        self._m_dev = None
        self._vesc: Optional[VESC_CAN] = None
        self._bus_channel = int(self.can_cfg.channels[0])
        self._io_lock = threading.Lock()

    def open(self) -> None:
        open_kwargs = {
            "device": self.can_cfg.device,
            "baud_rate": self.can_cfg.baud_rate,
            "dbit_baud_rate": self.can_cfg.dbit_baud_rate,
            "channels": self.can_cfg.channels,
            "fd": self.can_cfg.fd,
            "backend": self.can_cfg.backend,
        }
        if str(self.can_cfg.device).upper() == "TZUSB2CAN":
            if self.can_cfg.sp is not None:
                open_kwargs["sp"] = self.can_cfg.sp
            if self.can_cfg.dsp is not None:
                open_kwargs["dsp"] = self.can_cfg.dsp

        self._tx_cls, self._m_dev, _, _ = CANMessageTransmitter.open(**open_kwargs)
        bus = self._m_dev["buses"].get(self._bus_channel)
        if bus is None:
            raise RuntimeError(f"通道打开失败: {self._bus_channel}")
        self._vesc = VESC_CAN(self._tx_cls(bus))

    def close(self) -> None:
        if self._tx_cls is not None and self._m_dev is not None:
            self._tx_cls.close_can_device(self._m_dev)
        self._tx_cls = None
        self._m_dev = None
        self._vesc = None

    def __enter__(self) -> "VESCBusClient":
        self.open()
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

    def send_rpm(self, rpm: float) -> None:
        with self._io_lock:
            self._must_ready().send_rpm(self.motor_cfg.vesc_id, rpm)

    def send_pass_through(self, pos_deg: float, rpm: float, current_a: float) -> None:
        with self._io_lock:
            self._must_ready().send_pass_through(self.motor_cfg.vesc_id, pos_deg, rpm, current_a)

    def receive_decode(self, timeout_s: float) -> Optional[VESC_PACK]:
        with self._io_lock:
            _, pack = self._must_ready().receive_decode(timeout=timeout_s)
        if pack is None or int(pack.id) != int(self.motor_cfg.vesc_id):
            return None
        return pack

    def stop_motion(self) -> None:
        # 速度环入口统一使用 send_rpm 停止；位置环入口会发送 pass_through 零命令。
        self.send_rpm(0.0)

    def manual_pid_write_placeholder(self) -> None:
        raise NotImplementedError("PID 参数写入/回读默认人工执行。请在 VESC Tool 中写入并确认。")

    def _must_ready(self) -> VESC_CAN:
        if self._vesc is None:
            raise RuntimeError("CAN 总线未初始化，请先调用 open()")
        return self._vesc
