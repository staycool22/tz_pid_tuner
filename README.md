# TZ PID Tuner

本仓库用于 VESC 相关 PID 调试，包含两部分：

- `vesc/`：7 自由度机械臂控制与 PID 自整定工具。
- `can_box/`：CAN 通信与协议支持（以 submodule 方式接入）。

快速跳转：

- `autotune` 总览：[`docs/autotune_overview.md`](docs/autotune_overview.md)
- `autotune` 流程：[`docs/autotune_workflow.md`](docs/autotune_workflow.md)
- `autotune` 评分标准：[`docs/autotune_scoring.md`](docs/autotune_scoring.md)
- `autotune` 速度环完整说明：[`docs/speed_tuning_full.md`](docs/speed_tuning_full.md)
- `autotune` 位置环完整说明：[`docs/position_tuning_full.md`](docs/position_tuning_full.md)

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

`vesc/autotune` 当前采用“首轮参数生成 + 分阶段规则式微调”的流程：

- 速度环可先基于 `Kt / J / B` 辨识生成一组极点对消 PI。
- 速度环支持两种工作模式：辨识后直接退出，或在同一次运行中继续执行粗调 + 精调。
- 速度环默认可按“粗调阶段 + 精调阶段”连续运行，便于单次命令内完成大步搜索和局部细化。
- 位置环可先基于速度环带宽推导首轮参数，再执行目标点采样与 rule-based 微调。
- 每轮使用当前参数执行一组目标点试验并采样原始数据。
- 从采样中计算指标（误差、超调、响应时间、电流效率等）。
- 按加权评分选择更优方向，`signed_steady_error` 等辅助指标用于决定下一轮更新方向。
- 支持回退与小步探索，避免长期卡在同一组参数。

### 速度环（Speed Tuner）

- 搜索参数：`s_pid_kp`、`s_pid_ki`
- 控制方式：`send_rpm`
- 首轮参数：可由辨识阶段基于极点对消法直接生成
- 调试模式：支持 `identification -> generate PI -> exit`
- 调试模式：也支持 `identification -> coarse tuning -> fine tuning`
- 关键指标：
- `steady_error`：基于逐点绝对误差平均值（MAE）
- `signed_steady_error`：带符号稳态偏差（均值-目标）
- `overshoot`、`settling_time`、`current_efficiency`
- 默认单次流程可配置为 `15` 轮粗调 + `5` 轮精调；trial 记录中会带 `stage` 字段区分阶段

### 位置环（Position Tuner）

- 搜索参数：`p_pid_kp`、`p_pid_kd_proc`
- 控制方式：`send_pass_through`
- 关键指标：
- `position_stability`、`load_hold`、`step_response`、`rebound`
- 稳态点采用全局 `±1°` 条件筛选；未进入稳态会被高惩罚。
- 若最佳 `position_stability` 仍大于 `0.02°`，会提示先回退检查速度环基础。

详细实现说明见：

- `docs/autotune_overview.md`
- `docs/autotune_workflow.md`
- `docs/autotune_scoring.md`
- `docs/speed_tuning_full.md`
- `docs/position_tuning_full.md`

## 文档指引

建议按以下顺序阅读文档：

1. `docs/autotune_overview.md`
   - 先了解模块划分、运行入口、配置入口和输出产物。
2. `docs/autotune_workflow.md`
   - 再理解整个工程如何从速度环走到位置环和前馈。
3. `docs/autotune_scoring.md`
   - 需要解释 trial 分数、稳态判定和加权规则时查看。
4. `docs/speed_tuning_full.md`
   - 需要深入速度环首轮辨识、极点对消模型和微调逻辑时查看。
5. `docs/position_tuning_full.md`
   - 需要深入位置环模型、`pass_through` 映射和微调逻辑时查看。

文档入口统一收敛在本文件，`docs/` 

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
- `identification/result.json`：速度环辨识结果、极点对消生成参数与相关细节

## 使用建议

- 默认配置已开启自动写参；如关闭 `auto_write_params`，可切回每轮人工确认模式。
- 优先确保 CAN 通信、VESC ID、状态上报正常。
- 调参期间避免触碰机械限位，关注电流保护日志。
- `runs/` 为运行产物，默认不建议提交到仓库。

## 子模块说明

本仓库使用 `can_box` 子模块。首次克隆后请执行：

```bash
git submodule update --init --recursive
```
