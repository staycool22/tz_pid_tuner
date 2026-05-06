from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional


@dataclass
class CANConfig:
    device: str = "TZUSB2CAN"
    backend: str = "candle"
    baud_rate: int = 1_000_000
    dbit_baud_rate: int = 4_000_000
    channels: List[int] = field(default_factory=lambda: [0])
    fd: bool = True
    sp: Optional[float] = 0.8
    dsp: Optional[float] = 0.75


@dataclass
class MotorConfig:
    vesc_id: int = 1
    pos_window_deg: List[float] = field(default_factory=lambda: [5.0, 85.0])
    peak_current_limit_a: float = 15.0


@dataclass
class SpeedTuneConfig:
    test_rpms: List[float] = field(default_factory=lambda: [300.0, 600.0, 900.0])
    trial_seconds: float = 2.0
    sample_timeout_s: float = 0.05
    command_hz: float = 10.0
    read_hz: float = 500.0
    reverse_at_pos_limits: bool = True
    reverse_margin_deg: float = 1.0
    reverse_soft_limit_tolerance_deg: float = 10.0
    steady_state_priority: bool = True
    steady_spike_margin_rpm: float = 2000.0
    param_quantum: Dict[str, float] = field(default_factory=lambda: {"kp": 0.00001, "ki": 0.00001})
    initial_pi: Dict[str, float] = field(default_factory=lambda: {"s_pid_kp": 0.0010, "s_pid_ki": 0.0000})
    max_iterations: int = 8
    improve_threshold: float = 0.5
    step_scale: Dict[str, float] = field(default_factory=lambda: {"kp": 0.10, "ki": 0.15})
    param_limits: Dict[str, float] = field(
        default_factory=lambda: {
            "kp_min": 0.0001,
            "kp_max": 0.0500,
            "ki_min": 0.0000,
            "ki_max": 0.5000,
        }
    )
    min_valid_samples: int = 5
    manual_confirm_each_iteration: bool = True
    kpi_weights: Dict[str, float] = field(
        default_factory=lambda: {
            "steady_error": 0.55,
            "overshoot": 0.05,
            "settling_time": 0.25,
            "current_efficiency": 0.15,
        }
    )
    # 兼容旧配置：若未设置 initial_pi，则回退使用 candidate_pi 第一组。
    candidate_pi: List[Dict[str, float]] = field(
        default_factory=lambda: [
            {"s_pid_kp": 0.0020, "s_pid_ki": 0.0080},
            {"s_pid_kp": 0.0030, "s_pid_ki": 0.0120},
            {"s_pid_kp": 0.0040, "s_pid_ki": 0.0160},
        ]
    )


@dataclass
class PositionTuneConfig:
    target_positions_deg: List[float] = field(default_factory=lambda: [20.0, 35.0, 50.0, 70.0])
    trial_seconds: float = 2.0
    sample_timeout_s: float = 0.05
    command_hz: float = 100.0
    read_hz: float = 300.0
    ui_like_pass_through: bool = True
    motion_duration_s: float = 0.5
    ff_rpm_bias: float = 0.0
    ff_current_a: float = 0.0
    initial_params: Dict[str, float] = field(
        default_factory=lambda: {
            "p_pid_kp": 0.0100,
            "p_pid_kd_proc": 0.0000,
        }
    )
    max_iterations: int = 8
    improve_threshold: float = 0.2
    manual_confirm_each_iteration: bool = True
    step_scale: Dict[str, float] = field(default_factory=lambda: {"kp": 0.10, "kd_proc": 0.20})
    param_limits: Dict[str, float] = field(
        default_factory=lambda: {
            "kp_min": 0.0010,
            "kp_max": 0.2000,
            "kd_proc_min": 0.0000,
            "kd_proc_max": 0.2000,
        }
    )
    kpi_weights: Dict[str, float] = field(
        default_factory=lambda: {
            "position_stability": 0.35,
            "load_hold": 0.20,
            "step_response": 0.25,
            "rebound": 0.10,
            "current_efficiency": 0.10,
        }
    )
    # 兼容旧配置：若未设置 initial_params，则回退使用 candidate_params 第一组。
    candidate_params: List[Dict[str, float]] = field(
        default_factory=lambda: [
            {"p_pid_kp": 0.020, "p_pid_kd_proc": 0.000},
            {"p_pid_kp": 0.030, "p_pid_kd_proc": 0.002},
            {"p_pid_kp": 0.040, "p_pid_kd_proc": 0.004},
        ]
    )


@dataclass
class RunConfig:
    can: CANConfig = field(default_factory=CANConfig)
    motor: MotorConfig = field(default_factory=MotorConfig)
    speed_tuner: SpeedTuneConfig = field(default_factory=SpeedTuneConfig)
    position_tuner: PositionTuneConfig = field(default_factory=PositionTuneConfig)
    output_root: str = "runs"

    @property
    def output_root_path(self) -> Path:
        return Path(self.output_root)

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "RunConfig":
        return cls(
            can=CANConfig(**data.get("can", {})),
            motor=MotorConfig(**data.get("motor", {})),
            speed_tuner=SpeedTuneConfig(**data.get("speed_tuner", {})),
            position_tuner=PositionTuneConfig(**data.get("position_tuner", {})),
            output_root=data.get("output_root", "runs"),
        )
