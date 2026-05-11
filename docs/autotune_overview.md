# VESC 自整定工具总览

本文档汇总 `vesc/autotune/` 的模块划分、当前能力边界、运行入口和输出目录，作为当前自整定文档体系中的总览页。

返回总工程说明：[`../README.md`](../README.md)

详细说明跳转：

- 工程流程：[`autotune_workflow.md`](autotune_workflow.md)
- 评分标准：[`autotune_scoring.md`](autotune_scoring.md)
- 速度环完整说明：[`speed_tuning_full.md`](speed_tuning_full.md)
- 位置环完整说明：[`position_tuning_full.md`](position_tuning_full.md)

## 适用范围

当前自整定工具面向 VESC `pass_through` 相关调试，统一复用 `autotune/common/` 完成：

- 配置加载
- VESC 通信
- 采样记录
- 指标计算与加权评分
- 安全边界检查
- 运行摘要输出

## 目录结构

```text
vesc/
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
      tuning/identification.py
      tuning/initial_pi.py
      tuning/rule_based.py
    position_tuner/
      app/main.py
      tuning/initial_params.py
      tuning/rule_based.py
    feedforward_tool/
      app/main.py
    tools/
      speed_debug_tool.py
      position_debug_tool.py
      plot_runs.py
```

## 子模块说明

### 1. 速度环整定

- 模块：`autotune/speed_tuner`
- 控制方式：`send_rpm`
- 搜索参数：`s_pid_kp`、`s_pid_ki`
- 最新流程：先做首轮辨识生成初值，再进入 rule-based 微调
- 评分核心：`steady_error`、`signed_steady_error`、`overshoot`、`settling_time`、`current_efficiency`

当前实现重点：

- 首轮辨识使用 `set_current()` 采样，估计 `J`、`B`，并结合 `Kt` 与目标带宽生成初始 PI。
- 迭代阶段只调 `s_pid_kp / s_pid_ki`，保持 `current=0`，并要求整个过程处于安全角度窗口内。
- `auto_write_params=true` 时会自动写入参数并回读校验；关闭后可改为人工确认每轮写参。

### 2. 位置环整定

- 模块：`autotune/position_tuner`
- 控制方式：`send_pass_through`
- 搜索参数：`p_pid_kp`、`p_pid_kd_proc`
- 最新流程：先根据速度环带宽推导位置环初值，再做小步进/保持阶段采样与 rule-based 微调
- 评分核心：`position_stability`、`load_hold`、`step_response`、`rebound`、`current_efficiency`

当前实现重点：

- 位置环初值优先从 `initial_params_formula` 自动生成，不再只依赖手工填写首轮参数。
- `ui_like_pass_through=true` 时，测试轨迹会尽量贴近 UI 的平滑运动方式。
- 若最佳 `position_stability` 仍高于阈值，摘要中会提示先回退重整速度环。

### 3. 前馈工具

- 模块：`autotune/feedforward_tool`
- 当前状态：占位工具
- 输出内容：`summary.json`
- 用途：记录候选前馈电流，提醒人工从小电流开始验证位置保持与过推风险

### 4. 可视化

- 脚本：`autotune/tools/plot_runs.py`
- 用途：绘制 `runs/*` 下的迭代指标图和响应曲线图

## 配置入口

统一配置文件：

`e:\tz_pid_tuner\vesc\autotune\config\default_config.yaml`

重点配置分区：

1. `can`
   - CAN 设备、后端、波特率与通道
2. `motor`
   - `vesc_id`
   - `application_mode`
   - `pos_window_deg`
   - `peak_current_limit_a`
   - `pole_pairs`
   - `gear_ratio`
   - `l_max_erpm`
3. `speed_tuner`
   - 测试转速、采样频率、首轮辨识参数、初始 PI 公式、rule-based 步长与参数上下限
4. `position_tuner`
   - 目标位置、UI 风格轨迹、初值公式、rule-based 步长与参数上下限
5. `output_root`
   - 输出根目录，默认是 `runs`

## 运行入口

推荐在 `e:\tz_pid_tuner\vesc` 下执行：

```bash
python -m autotune.tools.speed_debug_tool --config autotune/config/default_config.yaml
python -m autotune.tools.position_debug_tool --config autotune/config/default_config.yaml
```

前馈占位工具：

```bash
python -m autotune.feedforward_tool.app.main --config autotune/config/default_config.yaml
```

可视化某次运行目录：

```bash
python -m autotune.tools.plot_runs --run-dir runs/speed_tuner/2026-04-28_181130
```

## 输出目录

输出根目录由 `output_root` 控制，默认落在：

`e:\tz_pid_tuner\vesc\runs`

每次运行会创建时间戳子目录，例如：

- `runs/speed_tuner/2026-...`
- `runs/position_tuner/2026-...`
- `runs/feedforward_tool/2026-...`

典型产物包括：

1. `session_meta.json`
2. `best_params.json`
3. `trials/*.csv`
4. `trials/trial_XXX_metrics.json`
5. `trials/all_trials.json`
6. `summary.md` 或 `summary.json`

速度环首轮辨识还会额外生成：

- `identification/j_plus_raw.csv`
- `identification/j_minus_raw.csv`
- 与惯量/阻尼拟合相关的辨识结果文件

## 关键约束

1. 配置统一从 `autotune/config/default_config.yaml` 读取。
2. 反馈采样统一通过 `receive_decode()`。
3. 安全检查统一由 `common/safety/guard.py` 执行，位置或电流越界会立即中断。
4. 默认配置已开启 `auto_write_params`，会自动写参并在会话末写入历史最优值。
5. 若关闭自动写参，可通过 `manual_confirm_each_iteration` 切换到每轮人工确认。

## 相关文档

- 工程流程：[`autotune_workflow.md`](autotune_workflow.md)
- 评分标准：[`autotune_scoring.md`](autotune_scoring.md)
- 速度环完整说明：[`speed_tuning_full.md`](speed_tuning_full.md)
- 位置环完整说明：[`position_tuning_full.md`](position_tuning_full.md)
- 仓库总览：[`../README.md`](../README.md)
