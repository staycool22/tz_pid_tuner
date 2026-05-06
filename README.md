# TZ PID Tuner

本仓库用于 VESC 相关 PID 调试，包含两部分：

- `vesc/`：7 自由度机械臂控制与 PID 自整定工具。
- `can_box/`：CAN 通信与协议支持（以 submodule 方式接入）。

快速跳转：

- `autotune` 详细说明：[`vesc/autotune/README.md`](vesc/autotune/README.md)

## 目录总览

```text
tz_pid_tuner/
  vesc/
    autotune/                 # PID 自整定主模块
      tools/                  # 调试入口与可视化脚本
        speed_debug_tool.py
        position_debug_tool.py
        plot_runs.py
      config/default_config.yaml
    vesc_7dof_ui.py
  can_box/                    # 子模块: CAN 设备与协议支持
```

## PID 自整定方法

`vesc/autotune` 采用规则式迭代整定（rule-based）：

- 每轮使用当前参数执行一组目标点试验并采样原始数据。
- 从采样中计算指标（误差、超调、响应时间、电流效率等）。
- 按加权评分选择更优方向，给出下一轮参数建议。
- 支持回退与小步探索，避免长期卡在同一组参数。

### 速度环（Speed Tuner）

- 搜索参数：`s_pid_kp`、`s_pid_ki`
- 控制方式：`send_rpm`
- 关键指标：
- `steady_error`：基于逐点绝对误差平均值（MAE）
- `signed_steady_error`：带符号稳态偏差（均值-目标）
- `overshoot`、`settling_time`、`current_efficiency`

### 位置环（Position Tuner）

- 搜索参数：`p_pid_kp`、`p_pid_kd_proc`
- 控制方式：`send_pass_through`
- 关键指标：
- `position_stability`、`load_hold`、`step_response`、`rebound`
- 稳态点采用全局 `±1°` 条件筛选；未进入稳态会被高惩罚。

详细实现说明见：

- `vesc/autotune/README.md`

## 工具使用说明

在 `e:\tz_pid_tuner\vesc` 目录执行：

```bash
python -m autotune.tools.speed_debug_tool --config autotune/config/default_config.yaml
python -m autotune.tools.position_debug_tool --config autotune/config/default_config.yaml
```

结果可视化（示例）：

```bash
python -m autotune.tools.plot_runs --run-dir runs/speed_tuner/2026-04-28_181130
```

前馈占位工具：

```bash
python -m autotune.feedforward_tool.app.main --config autotune/config/default_config.yaml
```

## 输出结果说明

默认输出目录：`e:\tz_pid_tuner\vesc\runs`

每次运行会生成时间戳目录，常见文件：

- `session_meta.json`：本次配置与元信息
- `trials/*_raw.csv`：原始采样数据
- `trials/*_metrics.json`：每轮指标与建议参数
- `best_params.json`：当前会话最佳参数
- `summary.md`：自动摘要

## 使用建议

- 每轮建议参数需人工写入 VESC 后再继续。
- 优先确保 CAN 通信、VESC ID、状态上报正常。
- 调参期间避免触碰机械限位，关注电流保护日志。
- `runs/` 为运行产物，默认不建议提交到仓库。

## 子模块说明

本仓库使用 `can_box` 子模块。首次克隆后请执行：

```bash
git submodule update --init --recursive
```
