#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
7轴机械臂 VESC CAN 控制 UI（PySide6）

功能：
1) 1~7 轴目标角度滑条控制
2) 基于时间窗的线性路径规划（连续模式）
3) 使用 VESC send_pass_through 发送目标角 + 前馈速度 + 前馈电流
4) UI 实时显示每轴反馈角度/转速/电流
"""

from __future__ import annotations

import argparse
import csv
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path

try:
    from PySide6.QtCore import Qt, QTimer
    from PySide6.QtWidgets import (
        QApplication,
        QCheckBox,
        QDoubleSpinBox,
        QGridLayout,
        QHBoxLayout,
        QLabel,
        QMainWindow,
        QMessageBox,
        QPushButton,
        QSlider,
        QVBoxLayout,
        QWidget,
    )
    PYSIDE6_IMPORT_ERROR = None
except Exception as exc:  # pragma: no cover - import guard
    PYSIDE6_IMPORT_ERROR = exc

try:
    from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
    from matplotlib.figure import Figure

    MATPLOTLIB_IMPORT_ERROR = None
except Exception as exc:  # pragma: no cover - import guard
    MATPLOTLIB_IMPORT_ERROR = exc


def _ensure_tzcan_import_path() -> None:
    """对齐 autotune 的导入策略，支持在当前目录直接运行 UI。"""
    project_root = Path(__file__).resolve().parent
    candidates = [
        project_root / "can_box",
        project_root.parent / "can_box",
        project_root / "can-box",
        project_root.parent / "can-box",
    ]
    for p in candidates:
        if p.exists():
            p_str = str(p)
            if p_str not in sys.path:
                sys.path.insert(0, p_str)
            return


_ensure_tzcan_import_path()

from tzcan import CANMessageTransmitter  # noqa: E402
from tzcan.protocols.vesc import VESC_CAN  # noqa: E402


def parse_bitrate_token(token: str) -> int:
    value = token.strip().lower()
    if value.endswith("k"):
        return int(float(value[:-1]) * 1000)
    if value.endswith("m"):
        return int(float(value[:-1]) * 1_000_000)
    return int(value)


def parse_int_list(text: str) -> list[int]:
    out: list[int] = []
    for item in text.split(","):
        raw = item.strip()
        if not raw:
            continue
        out.append(int(raw, 0))
    if not out:
        raise ValueError("joint id list is empty")
    return out


def _decode_int16_be(data: list[int], offset: int) -> int:
    if offset + 2 > len(data):
        raise ValueError("not enough bytes for int16")
    return int.from_bytes(bytes(data[offset : offset + 2]), byteorder="big", signed=True)


def _decode_int32_be(data: list[int], offset: int) -> int:
    if offset + 4 > len(data):
        raise ValueError("not enough bytes for int32")
    return int.from_bytes(bytes(data[offset : offset + 4]), byteorder="big", signed=True)


def _decode_vesc_status1_safe(arbitration_id: int, data: list[int]) -> tuple[int, float, float, float] | None:
    if len(data) < 8:
        return None
    status_id = (int(arbitration_id) >> 8) & 0xFF
    if status_id != 0x09:
        return None
    motor_id = int(arbitration_id) & 0xFF
    rpm = float(_decode_int32_be(data, 0))
    current_a = float(_decode_int16_be(data, 4)) / 100.0
    pos_deg = float(_decode_int16_be(data, 6)) / 50.0
    return motor_id, rpm, current_a, pos_deg


@dataclass
class JointState:
    joint_idx: int
    vesc_id: int
    target_deg: float = 0.0
    cmd_deg: float = 0.0
    ff_rpm: float = 0.0
    plan_rpm: float = 0.0
    ff_current_a: float = 0.0
    feedback_deg: float = 0.0
    feedback_rpm: float = 0.0
    feedback_current_a: float = 0.0
    has_feedback: bool = False
    plan_start_deg: float = 0.0
    plan_end_deg: float = 0.0
    plan_start_t: float = 0.0
    plan_end_t: float = 0.0

    def set_target(self, target_deg: float, duration_s: float, is_continue: bool) -> None:
        now = time.perf_counter()
        self.target_deg = float(target_deg)
        self.plan_start_deg = float(self.cmd_deg)
        self.plan_end_deg = float(target_deg)
        self.plan_start_t = now
        if is_continue:
            self.plan_end_t = now + max(float(duration_s), 1e-6)
        else:
            self.plan_end_t = now
            self.cmd_deg = float(target_deg)
            self.plan_rpm = 0.0

    def update_plan(self, now: float) -> None:
        if self.plan_end_t <= self.plan_start_t:
            self.cmd_deg = self.plan_end_deg
            self.plan_rpm = 0.0
            return
        total_t = self.plan_end_t - self.plan_start_t
        delta_deg = self.plan_end_deg - self.plan_start_deg
        tau = (now - self.plan_start_t) / total_t
        tau = min(max(tau, 0.0), 1.0)
        # Minimum-jerk 5th polynomial:
        # s(tau)=10*tau^3-15*tau^4+6*tau^5, ds/dt=(30*tau^2-60*tau^3+30*tau^4)/T
        s = 10.0 * tau**3 - 15.0 * tau**4 + 6.0 * tau**5
        self.cmd_deg = self.plan_start_deg + delta_deg * s
        if 0.0 < tau < 1.0:
            ds_dt = (30.0 * tau**2 - 60.0 * tau**3 + 30.0 * tau**4) / total_t
            self.plan_rpm = abs(delta_deg * ds_dt) * (60.0 / 360.0)
        else:
            self.plan_rpm = 0.0


class JointPosPlotWindow(QWidget):
    def __init__(self, joint_ids: list[int], max_points: int = 4000) -> None:
        super().__init__()
        self.joint_ids = list(joint_ids)
        self.max_points = max(200, int(max_points))
        self._t: list[float] = []
        self._y: dict[int, list[float]] = {jid: [] for jid in self.joint_ids}

        self.setWindowTitle("Joint pid_pos 曲线")
        self.resize(1000, 480)
        layout = QVBoxLayout(self)
        toolbar = QHBoxLayout()
        self.btn_save_png = QPushButton("保存PNG")
        self.btn_save_csv = QPushButton("保存CSV")
        self.lbl_save_status = QLabel("保存目录: runs/ui_plots")
        toolbar.addWidget(self.btn_save_png)
        toolbar.addWidget(self.btn_save_csv)
        toolbar.addWidget(self.lbl_save_status, 1)
        layout.addLayout(toolbar)
        self._fig = Figure(figsize=(10, 4.6))
        self._canvas = FigureCanvas(self._fig)
        layout.addWidget(self._canvas)

        self._ax = self._fig.add_subplot(111)
        self._lines = {}
        for jid in self.joint_ids:
            line, = self._ax.plot([], [], linewidth=1.2, label=f"id={jid} (0x{jid:X})")
            self._lines[jid] = line
        self._ax.set_xlabel("Time (s)")
        self._ax.set_ylabel("pid_pos (deg)")
        self._ax.grid(True, alpha=0.3)
        self._ax.legend(loc="upper right")
        self._fig.tight_layout()
        self.btn_save_png.clicked.connect(self.save_png)
        self.btn_save_csv.clicked.connect(self.save_csv)

    def _make_output_dir(self) -> Path:
        out_dir = Path(__file__).resolve().parent / "runs" / "ui_plots"
        out_dir.mkdir(parents=True, exist_ok=True)
        return out_dir

    @staticmethod
    def _ts_token() -> str:
        return time.strftime("%Y%m%d_%H%M%S", time.localtime())

    def save_png(self) -> None:
        if not self._t:
            QMessageBox.information(self, "无数据", "当前还没有曲线数据可保存。")
            return
        out_path = self._make_output_dir() / f"joint_pid_pos_{self._ts_token()}.png"
        self._fig.savefig(out_path, dpi=150)
        self.lbl_save_status.setText(f"已保存 PNG: {out_path}")

    def save_csv(self) -> None:
        if not self._t:
            QMessageBox.information(self, "无数据", "当前还没有曲线数据可保存。")
            return
        out_path = self._make_output_dir() / f"joint_pid_pos_{self._ts_token()}.csv"
        headers = ["t_s"] + [f"joint_{jid}_deg" for jid in self.joint_ids]
        with out_path.open("w", encoding="utf-8", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(headers)
            for i, t in enumerate(self._t):
                row = [t]
                for jid in self.joint_ids:
                    row.append(self._y[jid][i])
                writer.writerow(row)
        self.lbl_save_status.setText(f"已保存 CSV: {out_path}")

    def clear(self) -> None:
        self._t.clear()
        for jid in self.joint_ids:
            self._y[jid].clear()
            self._lines[jid].set_data([], [])
        self._canvas.draw_idle()

    def append_snapshot(self, t_s: float, pos_by_joint: dict[int, float]) -> None:
        self._t.append(float(t_s))
        for jid in self.joint_ids:
            self._y[jid].append(float(pos_by_joint.get(jid, float("nan"))))

        overflow = len(self._t) - self.max_points
        if overflow > 0:
            del self._t[:overflow]
            for jid in self.joint_ids:
                del self._y[jid][:overflow]

        for jid in self.joint_ids:
            self._lines[jid].set_data(self._t, self._y[jid])
        self._ax.relim()
        self._ax.autoscale_view()
        self._canvas.draw_idle()


class VESC7DofWindow(QMainWindow):
    SLIDER_SCALE = 100

    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__()
        self.args = args
        self.joint_ids = args.joint_ids
        self.n = len(self.joint_ids)
        self.pos_min_deg = float(args.pos_min_deg)
        self.pos_max_deg = float(args.pos_max_deg)
        self.is_continue = bool(args.is_continue)
        self.duration_s = max(float(args.duration), 1e-6)
        self.send_hz = max(float(args.send_hz), 1.0)

        self.tx_cls = None
        self.m_dev = None
        self.vesc: VESC_CAN | None = None
        self.connected = False
        self._running = False
        self._loop_thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self._plot_start_t = time.perf_counter()
        self.pos_plot_window: JointPosPlotWindow | None = None

        self.joints = [JointState(joint_idx=i, vesc_id=jid) for i, jid in enumerate(self.joint_ids)]

        self.target_sliders: list[QSlider] = []
        self.target_value_labels: list[QLabel] = []
        self.ff_rpm_spins: list[QDoubleSpinBox] = []
        self.ff_cur_spins: list[QDoubleSpinBox] = []
        self.fb_deg_labels: list[QLabel] = []
        self.fb_rpm_labels: list[QLabel] = []
        self.fb_cur_labels: list[QLabel] = []
        self._slider_dragging: list[bool] = [False for _ in range(self.n)]
        self._pending_target_deg: list[float | None] = [None for _ in range(self.n)]

        self._build_ui()
        self._ui_timer = QTimer(self)
        self._ui_timer.timeout.connect(self._refresh_ui_loop)
        self._ui_timer.start(100)

    def _build_ui(self) -> None:
        self.setWindowTitle("7DOF VESC Arm Control (PySide6)")
        self.resize(1380, 560)

        central = QWidget(self)
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(12, 10, 12, 10)
        root.setSpacing(10)

        # 顶部控制区：拆为两行，避免横向拥挤
        panel = QWidget(self)
        panel_layout = QGridLayout(panel)
        panel_layout.setContentsMargins(8, 8, 8, 8)
        panel_layout.setHorizontalSpacing(10)
        panel_layout.setVerticalSpacing(8)
        root.addWidget(panel, 0)

        self.btn_connect = QPushButton("连接 CAN")
        self.btn_disconnect = QPushButton("断开 CAN")
        self.btn_zero = QPushButton("全部置零")
        self.btn_sync = QPushButton("反馈同步目标")
        self.btn_apply_range = QPushButton("应用角度范围")
        self.btn_plot_pos = QPushButton("位置曲线")
        for btn in (
            self.btn_connect,
            self.btn_disconnect,
            self.btn_zero,
            self.btn_sync,
            self.btn_apply_range,
            self.btn_plot_pos,
        ):
            btn.setMinimumHeight(30)

        panel_layout.addWidget(self.btn_connect, 0, 0)
        panel_layout.addWidget(self.btn_disconnect, 0, 1)
        panel_layout.addWidget(self.btn_zero, 0, 2)
        panel_layout.addWidget(self.btn_sync, 0, 3)
        panel_layout.addWidget(self.btn_apply_range, 0, 4)
        panel_layout.addWidget(self.btn_plot_pos, 0, 5)

        self.lbl_status = QLabel("未连接")
        self.lbl_status.setStyleSheet("color:#1f6f43; font-weight:600;")
        panel_layout.addWidget(self.lbl_status, 0, 6, 1, 7, Qt.AlignRight)

        self.cb_continue = QCheckBox("连续规划")
        self.cb_continue.setChecked(self.is_continue)
        panel_layout.addWidget(self.cb_continue, 1, 0, 1, 1)

        panel_layout.addWidget(QLabel("规划时长(s)"), 1, 1)
        self.spin_duration = QDoubleSpinBox()
        self.spin_duration.setRange(0.001, 30.0)
        self.spin_duration.setDecimals(3)
        self.spin_duration.setValue(self.duration_s)
        self.spin_duration.setFixedWidth(90)
        panel_layout.addWidget(self.spin_duration, 1, 2)

        panel_layout.addWidget(QLabel("发送频率(Hz)"), 1, 3)
        self.spin_send_hz = QDoubleSpinBox()
        self.spin_send_hz.setRange(1.0, 1000.0)
        self.spin_send_hz.setDecimals(1)
        self.spin_send_hz.setValue(self.send_hz)
        self.spin_send_hz.setFixedWidth(90)
        panel_layout.addWidget(self.spin_send_hz, 1, 4)

        panel_layout.addWidget(QLabel("角度范围[deg]"), 1, 5)
        self.spin_pos_min = QDoubleSpinBox()
        self.spin_pos_min.setRange(0.0, 655.35)
        self.spin_pos_min.setDecimals(2)
        self.spin_pos_min.setValue(self.pos_min_deg)
        self.spin_pos_min.setFixedWidth(90)
        panel_layout.addWidget(self.spin_pos_min, 1, 6)

        self.lbl_to = QLabel("~")
        panel_layout.addWidget(self.lbl_to, 1, 7)

        self.spin_pos_max = QDoubleSpinBox()
        self.spin_pos_max.setRange(0.0, 655.35)
        self.spin_pos_max.setDecimals(2)
        self.spin_pos_max.setValue(self.pos_max_deg)
        self.spin_pos_max.setFixedWidth(90)
        panel_layout.addWidget(self.spin_pos_max, 1, 8)
        panel_layout.setColumnStretch(9, 1)

        # 关节控制区：紧凑网格 + 末行弹性撑开，避免行间被拉大
        table_container = QWidget(self)
        table = QGridLayout(table_container)
        table.setContentsMargins(4, 6, 4, 4)
        table.setHorizontalSpacing(8)
        table.setVerticalSpacing(6)
        root.addWidget(table_container, 0)

        headers = [
            "Joint",
            "VESC ID",
            "Target(deg)",
            "Target Value",
            "FF rpm bias",
            "FF current(A)",
            "Actual(deg)",
            "Actual rpm",
            "Actual current(A)",
        ]
        for col, title in enumerate(headers):
            header = QLabel(title)
            header.setStyleSheet("font-weight:600;")
            table.addWidget(header, 0, col)

        for i, joint in enumerate(self.joints, start=1):
            table.addWidget(QLabel(f"J{i}"), i, 0)
            table.addWidget(QLabel(f"{joint.vesc_id} (0x{joint.vesc_id:X})"), i, 1)

            slider = QSlider(Qt.Horizontal)
            slider.setRange(self._deg_to_slider(self.pos_min_deg), self._deg_to_slider(self.pos_max_deg))
            slider.setValue(self._deg_to_slider(0.0))
            slider.valueChanged.connect(lambda value, idx=i - 1: self.on_slider_changed(idx, value))
            slider.sliderPressed.connect(lambda idx=i - 1: self.on_slider_pressed(idx))
            slider.sliderReleased.connect(lambda idx=i - 1: self.on_slider_released(idx))
            table.addWidget(slider, i, 2)
            self.target_sliders.append(slider)

            target_value_label = QLabel("0.00")
            target_value_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            table.addWidget(target_value_label, i, 3)
            self.target_value_labels.append(target_value_label)

            ff_rpm_spin = QDoubleSpinBox()
            ff_rpm_spin.setRange(0.0, 65535.0)
            ff_rpm_spin.setDecimals(1)
            ff_rpm_spin.setSingleStep(10.0)
            ff_rpm_spin.setValue(float(self.args.default_ff_rpm))
            ff_rpm_spin.setFixedWidth(95)
            ff_rpm_spin.valueChanged.connect(lambda _value, idx=i - 1: self.on_ff_changed(idx))
            table.addWidget(ff_rpm_spin, i, 4)
            self.ff_rpm_spins.append(ff_rpm_spin)

            ff_cur_spin = QDoubleSpinBox()
            ff_cur_spin.setRange(0.0, 65.535)
            ff_cur_spin.setDecimals(3)
            ff_cur_spin.setSingleStep(0.1)
            ff_cur_spin.setValue(float(self.args.default_current_a))
            ff_cur_spin.setFixedWidth(95)
            ff_cur_spin.valueChanged.connect(lambda _value, idx=i - 1: self.on_ff_changed(idx))
            table.addWidget(ff_cur_spin, i, 5)
            self.ff_cur_spins.append(ff_cur_spin)

            fb_deg = QLabel("--")
            fb_rpm = QLabel("--")
            fb_cur = QLabel("--")
            fb_deg.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            fb_rpm.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            fb_cur.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            table.addWidget(fb_deg, i, 6)
            table.addWidget(fb_rpm, i, 7)
            table.addWidget(fb_cur, i, 8)
            self.fb_deg_labels.append(fb_deg)
            self.fb_rpm_labels.append(fb_rpm)
            self.fb_cur_labels.append(fb_cur)

        table.setColumnStretch(2, 1)
        table.setColumnMinimumWidth(2, 420)
        table.setColumnMinimumWidth(1, 100)
        table.setColumnMinimumWidth(3, 80)
        table.setColumnMinimumWidth(6, 90)
        table.setColumnMinimumWidth(7, 90)
        table.setColumnMinimumWidth(8, 105)
        table.setRowStretch(self.n + 1, 1)
        root.addStretch(1)

        self.btn_connect.clicked.connect(self.connect_can)
        self.btn_disconnect.clicked.connect(self.disconnect_can)
        self.btn_zero.clicked.connect(self.zero_all_targets)
        self.btn_sync.clicked.connect(self.sync_target_from_feedback)
        self.btn_apply_range.clicked.connect(self.apply_slider_range)
        self.btn_plot_pos.clicked.connect(self.open_joint_pos_plot)
        self.cb_continue.toggled.connect(self._on_continue_toggled)
        self.spin_duration.valueChanged.connect(self._on_duration_changed)
        self.spin_send_hz.valueChanged.connect(self._on_send_hz_changed)

    def open_joint_pos_plot(self) -> None:
        if MATPLOTLIB_IMPORT_ERROR is not None:
            QMessageBox.warning(
                self,
                "依赖缺失",
                "未检测到 matplotlib Qt 后端，无法打开位置曲线窗口。\n"
                "请安装: pip install matplotlib\n"
                f"导入错误: {MATPLOTLIB_IMPORT_ERROR}",
            )
            return
        if self.pos_plot_window is None:
            self.pos_plot_window = JointPosPlotWindow(joint_ids=self.joint_ids)
        self.pos_plot_window.show()
        self.pos_plot_window.raise_()
        self.pos_plot_window.activateWindow()

    def _deg_to_slider(self, deg: float) -> int:
        return int(round(float(deg) * self.SLIDER_SCALE))

    def _slider_to_deg(self, value: int) -> float:
        return float(value) / self.SLIDER_SCALE

    def _on_continue_toggled(self, checked: bool) -> None:
        with self._lock:
            self.is_continue = bool(checked)

    def _on_duration_changed(self, value: float) -> None:
        with self._lock:
            self.duration_s = max(float(value), 1e-6)

    def _on_send_hz_changed(self, value: float) -> None:
        with self._lock:
            self.send_hz = max(float(value), 1.0)

    @staticmethod
    def _clamp_pass_through(pos_deg: float, rpm: float, current_a: float) -> tuple[float, float, float]:
        # send_pass_through 底层编码为 uint16:
        # pos*100, rpm, current*1000。负值会回绕，因此这里做安全裁剪。
        safe_pos = min(max(float(pos_deg), 0.0), 655.35)
        safe_rpm = min(max(float(rpm), 0.0), 65535.0)
        safe_cur = min(max(float(current_a), 0.0), 65.535)
        return safe_pos, safe_rpm, safe_cur

    def apply_slider_range(self) -> None:
        new_min = float(self.spin_pos_min.value())
        new_max = float(self.spin_pos_max.value())
        if new_max <= new_min:
            QMessageBox.warning(self, "范围错误", "角度上限必须大于下限。")
            return
        with self._lock:
            self.pos_min_deg = new_min
            self.pos_max_deg = new_max

        s_min = self._deg_to_slider(new_min)
        s_max = self._deg_to_slider(new_max)
        for slider in self.target_sliders:
            cur = slider.value()
            slider.setRange(s_min, s_max)
            slider.setValue(min(max(cur, s_min), s_max))

    def on_slider_changed(self, idx: int, slider_value: int) -> None:
        deg = self._slider_to_deg(slider_value)
        self.target_value_labels[idx].setText(f"{deg:.2f}")
        if self._slider_dragging[idx]:
            self._pending_target_deg[idx] = deg
            return
        with self._lock:
            self.joints[idx].set_target(
                target_deg=deg,
                duration_s=self.duration_s,
                is_continue=self.is_continue,
            )

    def on_slider_pressed(self, idx: int) -> None:
        self._slider_dragging[idx] = True
        self._pending_target_deg[idx] = None

    def on_slider_released(self, idx: int) -> None:
        deg = self._pending_target_deg[idx]
        if deg is None:
            deg = self._slider_to_deg(self.target_sliders[idx].value())
        self._slider_dragging[idx] = False
        self._pending_target_deg[idx] = None
        with self._lock:
            self.joints[idx].set_target(
                target_deg=deg,
                duration_s=self.duration_s,
                is_continue=self.is_continue,
            )

    def on_ff_changed(self, idx: int) -> None:
        with self._lock:
            self.joints[idx].ff_rpm = float(self.ff_rpm_spins[idx].value())
            self.joints[idx].ff_current_a = float(self.ff_cur_spins[idx].value())

    def zero_all_targets(self) -> None:
        target = min(max(0.0, self.pos_min_deg), self.pos_max_deg)
        slider_value = self._deg_to_slider(target)
        for slider in self.target_sliders:
            slider.setValue(slider_value)

    def sync_target_from_feedback(self) -> None:
        with self._lock:
            values = []
            for joint in self.joints:
                if joint.has_feedback:
                    values.append(joint.feedback_deg)
                else:
                    values.append(None)
        for idx, value in enumerate(values):
            if value is None:
                continue
            clamped = min(max(value, self.pos_min_deg), self.pos_max_deg)
            self.target_sliders[idx].setValue(self._deg_to_slider(clamped))

    def connect_can(self) -> None:
        if self.connected:
            return
        try:
            baud = parse_bitrate_token(self.args.can_br)
            data_baud = parse_bitrate_token(self.args.data_can_br)
            self.tx_cls, self.m_dev, _, _ = CANMessageTransmitter.open(
                device="TZUSB2CAN",
                baud_rate=baud,
                dbit_baud_rate=data_baud,
                channels=[self.args.iface],
                fd=bool(self.args.canfd),
                backend=self.args.backend,
                sp=0.8,
                dsp=0.75,
            )
            bus = self.m_dev["buses"].get(self.args.iface)
            if bus is None:
                raise RuntimeError(f"Failed to open CAN channel {self.args.iface}")

            self.vesc = VESC_CAN(self.tx_cls(bus))
            with self._lock:
                for idx, joint in enumerate(self.joints):
                    joint.ff_rpm = float(self.ff_rpm_spins[idx].value())
                    joint.ff_current_a = float(self.ff_cur_spins[idx].value())
                    joint.set_target(
                        target_deg=self._slider_to_deg(self.target_sliders[idx].value()),
                        duration_s=0.0,
                        is_continue=False,
                    )

            self._running = True
            self._loop_thread = threading.Thread(target=self._control_loop, daemon=True)
            self._loop_thread.start()
            self.connected = True
            self._plot_start_t = time.perf_counter()
            if self.pos_plot_window is not None:
                self.pos_plot_window.clear()
            self.lbl_status.setText("已连接，控制循环运行中")
        except Exception as exc:
            self.lbl_status.setText(f"连接失败: {exc}")
            self._safe_close_bus()

    def disconnect_can(self) -> None:
        self._running = False
        if self._loop_thread is not None:
            self._loop_thread.join(timeout=1.0)
            self._loop_thread = None
        self._safe_close_bus()
        self.connected = False
        self.lbl_status.setText("未连接")

    def _safe_close_bus(self) -> None:
        try:
            if self.tx_cls is not None and self.m_dev is not None:
                self.tx_cls.close_can_device(self.m_dev)
        except Exception:
            pass
        self.tx_cls = None
        self.m_dev = None
        self.vesc = None

    def _control_loop(self) -> None:
        assert self.vesc is not None
        next_t = time.perf_counter()
        while self._running:
            now_plan = time.perf_counter()
            with self._lock:
                dt = 1.0 / max(self.send_hz, 1.0)
                for joint in self.joints:
                    joint.update_plan(now_plan)
                    rpm_cmd = joint.plan_rpm + joint.ff_rpm
                    safe_pos, safe_rpm, safe_cur = self._clamp_pass_through(
                        joint.cmd_deg,
                        rpm_cmd,
                        joint.ff_current_a,
                    )
                    self.vesc.send_pass_through(
                        _id=joint.vesc_id,
                        _pos=safe_pos,
                        _rpm=safe_rpm,
                        _cur=safe_cur,
                    )

            self._drain_feedback(timeout=0.001, max_reads=50)

            next_t += dt
            remain = next_t - time.perf_counter()
            if remain > 0:
                time.sleep(min(remain, 0.01))
            else:
                next_t = time.perf_counter()

    def _drain_feedback(self, timeout: float, max_reads: int) -> None:
        if self.vesc is None:
            return
        reads = 0
        while reads < max_reads:
            reads += 1
            result = self.vesc.transmitter._receive_can_data(
                timeout=timeout if reads == 1 else 0,
                target_id=None,
                is_ext_frame=None,
                is_fd=False,
                return_msg=True,
            )
            if result is None:
                break
            msg = result[-1] if isinstance(result, tuple) and len(result) >= 1 else None
            if msg is None:
                break

            arb_id = int(getattr(msg, "arbitration_id", 0)) & 0x1FFFFFFF
            data_bytes = list(getattr(msg, "data", b"") or b"")
            decoded = _decode_vesc_status1_safe(arb_id, data_bytes)
            if decoded is None:
                continue

            motor_id, rpm, current_a, pos_deg = decoded
            with self._lock:
                for joint in self.joints:
                    if joint.vesc_id == motor_id:
                        joint.feedback_deg = pos_deg
                        joint.feedback_rpm = rpm
                        joint.feedback_current_a = current_a
                        joint.has_feedback = True
                        break

    def _refresh_ui_loop(self) -> None:
        with self._lock:
            snapshot = list(self.joints)
        pos_by_joint: dict[int, float] = {}
        for idx, joint in enumerate(snapshot):
            if joint.has_feedback:
                self.fb_deg_labels[idx].setText(f"{joint.feedback_deg:8.3f}")
                self.fb_rpm_labels[idx].setText(f"{joint.feedback_rpm:8.1f}")
                self.fb_cur_labels[idx].setText(f"{joint.feedback_current_a:8.3f}")
                pos_by_joint[joint.vesc_id] = joint.feedback_deg
            else:
                self.fb_deg_labels[idx].setText("--")
                self.fb_rpm_labels[idx].setText("--")
                self.fb_cur_labels[idx].setText("--")
        if self.pos_plot_window is not None and self.pos_plot_window.isVisible():
            now_s = time.perf_counter() - self._plot_start_t
            self.pos_plot_window.append_snapshot(t_s=now_s, pos_by_joint=pos_by_joint)

    def closeEvent(self, event) -> None:  # type: ignore[override]
        if self.pos_plot_window is not None:
            self.pos_plot_window.close()
            self.pos_plot_window = None
        self.disconnect_can()
        super().closeEvent(event)

    def run(self) -> int:
        self.show()
        return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="7DOF VESC arm UI using send_pass_through")
    parser.add_argument(
        "--joint-ids",
        default="0x21,0x25,0x29,0x2C,0x2D,0x2E,0x2F",
        help="Comma separated VESC IDs, support decimal/hex",
    )
    parser.add_argument("--iface", type=int, default=0, help="CAN interface channel")
    parser.add_argument("--can-br", default="1m", help="CAN nominal bitrate token")
    parser.add_argument("--canfd", action=argparse.BooleanOptionalAction, default=True, help="Enable CAN FD transport")
    parser.add_argument("--data-can-br", default="4m", help="CAN FD data bitrate token")
    parser.add_argument("--backend", default="candle", help="CAN backend")
    parser.add_argument("--send-hz", type=float, default=100.0, help="Send loop frequency")
    parser.add_argument("--is-continue", action=argparse.BooleanOptionalAction, default=True, help="Enable continuous ramp planning")
    parser.add_argument("--duration", type=float, default=0.5, help="Planning duration in seconds")
    parser.add_argument("--default-ff-rpm", type=float, default=0.0, help="Default feedforward rpm (>=0)")
    parser.add_argument("--default-current-a", type=float, default=0.0, help="Default feedforward current (>=0)")
    parser.add_argument("--pos-min-deg", type=float, default=0.0, help="Slider min angle (>=0 for pass_through uint16)")
    parser.add_argument("--pos-max-deg", type=float, default=360.0, help="Slider max angle")
    return parser


def main() -> None:
    if PYSIDE6_IMPORT_ERROR is not None:
        raise SystemExit(
            "PySide6 未安装或不可用，请先执行: pip install pyside6\n"
            f"导入错误: {PYSIDE6_IMPORT_ERROR}"
        )

    parser = build_parser()
    args = parser.parse_args()
    args.joint_ids = parse_int_list(args.joint_ids)
    if not (1 <= len(args.joint_ids) <= 7):
        raise SystemExit(f"joint ids must be 1~7 items, got {len(args.joint_ids)}")
    app = QApplication(sys.argv)
    window = VESC7DofWindow(args)
    window.run()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
