from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional


def _merge_default_dict(defaults: Dict[str, Any], overrides: Dict[str, Any] | None) -> Dict[str, Any]:
    merged = dict(defaults)
    if overrides:
        merged.update(overrides)
    return merged


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
    application_mode: str = "robotic_arm"
    pole_pairs: float = 1.0
    gear_ratio: float = 1.0
    l_max_erpm: float = 0.0

    @property
    def uses_position_limits(self) -> bool:
        return str(self.application_mode).strip().lower() in {"robotic_arm", "arm", "joint"}


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
    identification: Dict[str, Any] = field(
        default_factory=lambda: {
            "enabled": False,
            "estimate_kind": "unloaded_first_pass",
            "exit_after_parameter_generation": False,
            "parameter_generation_method": "pole_cancellation_pi",
            "controller_speed_unit": "erpm",
            "feedback_speed_unit": "erpm",
            "manual_confirm_before_identification": True,
            "command_hz": 100.0,
            "read_hz": 300.0,
            "sample_timeout_s": 0.05,
            "iq_test_ratio": 0.10,
            "iq_test_a": None,
            "settle_time_s": 0.05,
            "transient_discard_s": 0.02,
            "accel_fit_window_s": 0.12,
            "step_duration_s": 0.30,
            "rest_between_steps_s": 0.20,
            "coast_down_spinup_ratio": 0.15,
            "coast_down_spinup_current_a": None,
            "coast_down_spinup_duration_s": 0.60,
            "coast_down_duration_s": 0.80,
            "coast_down_discard_s": 0.02,
            "coast_down_min_speed_for_fit": 200.0,
            "coast_down_min_speed_unit": "erpm",
            "allow_steady_state_b_fallback": True,
            "steady_state_duration_s": 1.00,
            "steady_state_eval_window_s": 0.25,
            "target_bandwidth_rad_s": None,
            "current_loop_bandwidth_rad_s": None,
            "speed_to_current_bandwidth_ratio": 8.0,
            "target_time_constant_s": None,
            "torque_constant_nm_per_a": None,
            "kv_rpm_per_v": None,
            "ke_v_per_rad_s": None,
        }
    )
    # 若提供辨识结果，则优先按 Kt/J/B/wb 自动生成初始 PI，再进入 rule_based 微调。
    initial_pi_formula: Dict[str, Any] = field(default_factory=dict)
    initial_pi: Dict[str, float] = field(default_factory=lambda: {"s_pid_kp": 0.0010, "s_pid_ki": 0.0000})
    max_iterations: int = 8
    coarse_iterations: int = 0
    fine_iterations: int = 0
    improve_threshold: float = 0.5
    step_scale: Dict[str, float] = field(default_factory=lambda: {"kp": 0.10, "ki": 0.15})
    fine_step_scale: Dict[str, float] = field(default_factory=lambda: {"kp": 0.03, "ki": 0.05})
    param_limits: Dict[str, float] = field(
        default_factory=lambda: {
            "kp_min": 0.0001,
            "kp_max": 0.0500,
            "ki_min": 0.0000,
            "ki_max": 0.5000,
        }
    )
    min_valid_samples: int = 5
    auto_write_params: bool = False
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
    # 若提供 wb_speed/l_max_erpm 等信息，则可按 pos_tune_skill.md 自动生成初始参数。
    initial_params_formula: Dict[str, Any] = field(default_factory=dict)
    initial_params: Dict[str, float] = field(
        default_factory=lambda: {
            "p_pid_kp": 0.0100,
            "p_pid_kd_proc": 0.0000,
        }
    )
    max_iterations: int = 8
    improve_threshold: float = 0.2
    auto_write_params: bool = False
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
        speed_tuner_data = dict(data.get("speed_tuner", {}))
        position_tuner_data = dict(data.get("position_tuner", {}))

        speed_defaults = SpeedTuneConfig()
        position_defaults = PositionTuneConfig()

        speed_tuner_data["param_quantum"] = _merge_default_dict(
            speed_defaults.param_quantum,
            speed_tuner_data.get("param_quantum"),
        )
        speed_tuner_data["identification"] = _merge_default_dict(
            speed_defaults.identification,
            speed_tuner_data.get("identification"),
        )
        speed_tuner_data["initial_pi_formula"] = _merge_default_dict(
            speed_defaults.initial_pi_formula,
            speed_tuner_data.get("initial_pi_formula"),
        )
        speed_tuner_data["initial_pi"] = _merge_default_dict(
            speed_defaults.initial_pi,
            speed_tuner_data.get("initial_pi"),
        )
        speed_tuner_data["step_scale"] = _merge_default_dict(
            speed_defaults.step_scale,
            speed_tuner_data.get("step_scale"),
        )
        speed_tuner_data["fine_step_scale"] = _merge_default_dict(
            speed_defaults.fine_step_scale,
            speed_tuner_data.get("fine_step_scale"),
        )
        speed_tuner_data["param_limits"] = _merge_default_dict(
            speed_defaults.param_limits,
            speed_tuner_data.get("param_limits"),
        )
        speed_tuner_data["kpi_weights"] = _merge_default_dict(
            speed_defaults.kpi_weights,
            speed_tuner_data.get("kpi_weights"),
        )

        position_tuner_data["initial_params_formula"] = _merge_default_dict(
            position_defaults.initial_params_formula,
            position_tuner_data.get("initial_params_formula"),
        )
        position_tuner_data["initial_params"] = _merge_default_dict(
            position_defaults.initial_params,
            position_tuner_data.get("initial_params"),
        )
        position_tuner_data["step_scale"] = _merge_default_dict(
            position_defaults.step_scale,
            position_tuner_data.get("step_scale"),
        )
        position_tuner_data["param_limits"] = _merge_default_dict(
            position_defaults.param_limits,
            position_tuner_data.get("param_limits"),
        )
        position_tuner_data["kpi_weights"] = _merge_default_dict(
            position_defaults.kpi_weights,
            position_tuner_data.get("kpi_weights"),
        )

        return cls(
            can=CANConfig(**data.get("can", {})),
            motor=MotorConfig(**data.get("motor", {})),
            speed_tuner=SpeedTuneConfig(**speed_tuner_data),
            position_tuner=PositionTuneConfig(**position_tuner_data),
            output_root=data.get("output_root", "runs"),
        )
