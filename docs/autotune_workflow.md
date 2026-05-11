# VESC 自整定工程流程

本文档只说明整个自整定工程如何运作、各模块之间如何串联，以及在什么阶段应该查阅哪份详细文档。速度环和位置环的完整理论与流程拆解已迁移到单独说明文件。

返回总工程说明：[`../README.md`](../README.md)

## 总体顺序

建议按以下顺序执行：

1. 确认电流环已闭环、编码器与 CAN 通信正常。
2. 整定速度环，得到稳定的 `s_pid_kp / s_pid_ki`。
3. 在速度环稳定的前提下整定位置环，得到首版 `p_pid_kp / p_pid_kd_proc`。
4. 最后再做前馈电流扫描与人工验证。

不要直接跳到位置环或前馈阶段，否则会把内环问题误判成外环参数问题。

## 通用前提

### 安全边界

- `motor.pos_window_deg` 用于位置安全窗口。
- `motor.peak_current_limit_a` 用于电流越界保护。
- 一旦 `SafetyGuard` 检测到位置或电流异常，当前试验会立即中断。

### 统一配置

所有流程都从以下文件读取配置：

`e:\tz_pid_tuner\vesc\autotune\config\default_config.yaml`

### 统一输出

默认输出目录：

`e:\tz_pid_tuner\vesc\runs`

每次运行都会创建带时间戳的子目录，并在其中保存原始采样、每轮评分、最佳参数和摘要。

## 速度环模块

速度环模块负责：

1. 必要时执行首轮辨识
2. 生成首版 `s_pid_kp / s_pid_ki`
3. 对多个目标转速执行 trial
4. 根据评分结果给出下一轮建议参数
5. 保存历史最优参数与摘要

对应详细文档：

- 评分标准：[`autotune_scoring.md`](autotune_scoring.md)
- 速度环完整说明：[`speed_tuning_full.md`](speed_tuning_full.md)

运行命令：

在 `e:\tz_pid_tuner\vesc` 下执行：

```bash
python -m autotune.tools.speed_debug_tool --config autotune/config/default_config.yaml
```

## 位置环模块

位置环模块负责：

1. 从速度环带宽信息生成首轮 `p_pid_kp / p_pid_kd_proc`
2. 通过 `send_pass_through` 执行多目标点 trial
3. 统计保持误差、回摆和响应速度
4. 根据评分结果生成下一轮位置环参数建议
5. 当位置误差仍偏大时提示先回退检查速度环

对应详细文档：

- 评分标准：[`autotune_scoring.md`](autotune_scoring.md)
- 位置环完整说明：[`position_tuning_full.md`](position_tuning_full.md)

运行命令：

在 `e:\tz_pid_tuner\vesc` 下执行：

```bash
python -m autotune.tools.position_debug_tool --config autotune/config/default_config.yaml
```

## 前馈占位模块

当前前馈工具仍是占位入口，不做自动闭环优化，只生成候选前馈电流列表。

用途：

1. 在速度环和位置环冻结后，记录待人工验证的前馈电流。
2. 提醒从小电流开始，观察位置保持与过推风险。

运行命令：

```bash
python -m autotune.feedforward_tool.app.main --config autotune/config/default_config.yaml
```

可自定义候选电流：

```bash
python -m autotune.feedforward_tool.app.main --config autotune/config/default_config.yaml --currents 0.2 0.4 0.6
```

## 结果查看

典型结果目录内容：

1. `session_meta.json`
2. `best_params.json`
3. `trials/*.csv`
4. `trials/trial_XXX_metrics.json`
5. `trials/all_trials.json`
6. `summary.md` 或 `summary.json`

可视化命令：

```bash
python -m autotune.tools.plot_runs --run-dir runs/speed_tuner/2026-04-28_181130
```

## 推荐阅读顺序

1. 先看 [`autotune_overview.md`](autotune_overview.md)，了解目录、入口和产物。
2. 再看本文，理解整个工程如何从速度环走到位置环和前馈。
3. 调速度环时进入 [`speed_tuning_full.md`](speed_tuning_full.md)。
4. 调位置环时进入 [`position_tuning_full.md`](position_tuning_full.md)。
5. 需要解释分数时再查 [`autotune_scoring.md`](autotune_scoring.md)。
