# pass_through PID 自整定工具

该目录包含速度环、位置环与前馈占位工具，统一复用 `common/` 模块完成配置加载、通信、采样、评分与报告输出。

## 目录结构

```text
autotune/
  common/
    acquisition/      # 采样记录、run 目录创建、json/csv 写入
    analysis/         # 速度环/位置环指标计算与加权评分
    config/           # 配置数据模型与 loader
    io/               # VESC 总线客户端封装
    reporting/        # summary.md 生成
    safety/           # 位置/电流安全边界检查
  config/
    default_config.yaml
  speed_tuner/
    app/main.py
    tuning/rule_based.py
  position_tuner/
    app/main.py
    tuning/rule_based.py
  feedforward_tool/
    app/main.py
  tools/
    plot_runs.py      # 运行结果可视化脚本
```

## 子模块说明

1. 速度环整定：`speed_tuner`
   - 控制方式：`send_rpm`
   - 搜索参数：`s_pid_kp`、`s_pid_ki`
   - 评分核心：`steady_error`（MAE）、`overshoot`、`settling_time`、`current_efficiency`
2. 位置环整定：`position_tuner`
   - 控制方式：`send_pass_through`
   - 搜索参数：`p_pid_kp`、`p_pid_kd_proc`
   - 评分核心：`position_stability`、`load_hold`、`step_response`、`rebound`、`current_efficiency`
3. 前馈工具：`feedforward_tool`
   - 当前为占位入口，生成 `summary.json` 并记录候选电流
4. 可视化：`tools/plot_runs.py`
   - 用于绘制 `runs/*` 下的迭代指标图与响应曲线图

## 运行方式

在 `e:\tz_pid_tuner\vesc` 下执行（推荐使用 tools 入口）：

```bash
python -m autotune.tools.speed_debug_tool --config autotune/config/default_config.yaml
python -m autotune.tools.position_debug_tool --config autotune/config/default_config.yaml
```

前馈占位工具仍可按原入口执行：

```bash
python -m autotune.feedforward_tool.app.main --config autotune/config/default_config.yaml
```

可视化某次运行目录（示例）：

```bash
python -m autotune.tools.plot_runs --run-dir runs/speed_tuner/2026-04-28_181130
```

## 输出目录

输出根目录由 `output_root` 控制，默认是 `runs`，因此默认落在：

`e:\tz_pid_tuner\vesc\runs`

每次运行会创建时间戳子目录（如 `runs/speed_tuner/2026-...`），典型内容包括：

1. `session_meta.json`
2. `trials/*.csv` 与 `trials/*_metrics.json`
3. `best_params.json`
4. `summary.md`（speed/position）
5. `summary.json`（feedforward 占位输出）

## 关键约束

1. 配置统一从 `autotune/config/default_config.yaml` 读取。
2. 反馈采样统一通过 `receive_decode()`。
3. 安全检查统一由 `common/safety/guard.py` 执行（位置/电流越界即中断）。
4. PID 写入流程默认人工确认，每轮会给出下一轮建议参数。
