# 速度环整定完整说明

本文档面向 `autotune/speed_tuner`，完整说明速度环初值构造、极点对消模型、首轮辨识流程、分阶段 rule-based 微调逻辑和运行产物。

返回总工程说明：[`../README.md`](../README.md)

## 文档目标

速度环文档重点回答四个问题：

1. 首轮 `s_pid_kp / s_pid_ki` 是怎么来的
2. 为什么这里采用极点对消模型
3. 代码里如何把辨识、建模、换算和分阶段微调串起来
4. 评分结果和下一轮参数建议分别由什么驱动

## 适用前提

使用本流程前，默认满足以下条件：

- 电流环已经闭环且稳定
- 速度反馈可靠
- 编码器、CAN 总线、VESC 状态上报正常
- 机械运动空间和电流上限已经配置到安全范围
- 第一轮最好在空载或轻载条件下进行

## 总流程图

```text
[确认电流环已闭环、速度反馈可靠]
                |
                v
[准备配置]
设置 Kt 来源、辨识电流、采样频率、安全边界
                |
                v
[首轮辨识]
set_current(+iq) / set_current(-iq) 估计 J
coast-down 或稳态法估计 B
                |
                v
[建立速度环模型]
G_speed(s) = Kt / (J s + B)
                |
                v
[极点对消生成初始 PI]
Ki / Kp = B / J
Kp = J * wb / Kt
Ki = B * wb / Kt
                |
                v
[单位换算与参数裁剪]
rad/s -> rpm 或 erpm
再量化到配置步长
                |
                v
[可选分支 A：辨识后直接输出参数并退出]
生成 `generated_speed_pi`
写入 result.json / summary.md
                |
                v
[可选分支 B：继续执行分阶段微调]
粗调阶段 -> 精调阶段
每轮 trial 记录 rpm / current / pos
计算指标与评分
rule-based 更新下一轮 PI
保存历史最优参数与摘要
```

## 速度环对象模型

### 控制对象

当电流环足够快时，速度环看到的机械对象可以近似写成：

```text
G_speed(s) = ω(s) / iq(s) = Kt / (J s + B)
```

其中：

- `Kt`：力矩常数，单位 `Nm/A`
- `J`：折算到电机轴侧的总惯量，单位 `kg*m^2`
- `B`：粘性阻尼系数，单位 `N*m*s/rad`
- `ω`：机械角速度，单位 `rad/s`

这也是当前速度环首轮 PI 公式的建模基础。

### 为什么可以这样建模

这里的关键假设不是“电机没有复杂性”，而是：

- 电流环足够快
- `iq` 能近似代表实际产力电流
- 速度环关注的是机械侧动态，而不是电气侧瞬态

在这个假设下，速度环主要要处理的是：

- 惯量带来的慢响应
- 阻尼带来的稳态速度损失

## 极点对消模型

### 控制器结构

当前速度环采用 PI：

```text
C(s) = Kp + Ki / s
```

其零点位置为：

```text
s = -Ki / Kp
```

### 被控对象极点

机械对象：

```text
G(s) = Kt / (J s + B)
```

对象极点位置为：

```text
s = -B / J
```

### 极点对消条件

若希望 PI 零点与对象极点对齐，则需要：

```text
Ki / Kp = B / J
```

这就是“极点对消”的核心。

### 目标带宽引入

在完成零极点对齐后，再选取目标速度环带宽 `wb`：

```text
Kp = J * wb / Kt
Ki = B * wb / Kt
```

含义可以理解为：

- `J` 越大，需要更大的 `Kp`
- `B` 越大，需要更大的 `Ki`
- 目标带宽 `wb` 越高，整套 PI 越激进

### 这不是最终参数

极点对消模型只负责生成首轮可用 PI，它并不承诺最终最优。

原因是实际系统还会受到这些因素影响：

- 负载变化
- 机构柔性
- 摩擦非线性
- 测量噪声
- VESC 内部实现和单位缩放

所以代码里的完整流程是：

```text
首轮模型给初值 -> 实机 trial -> 评分 -> 分阶段 rule-based 微调
```

## 首轮辨识流程拆解

### 步骤 1：确定 `Kt`

程序优先从以下三种来源之一得到 `Kt`：

1. `torque_constant_nm_per_a`
2. `kv_rpm_per_v`
3. `ke_v_per_rad_s`

常见换算：

```text
Kt ≈ 60 / (2π * Kv)
```

### 步骤 2：通过对称电流台阶辨识 `J`

代码会发出：

- `set_current(+iq_test)`
- `set_current(-iq_test)`

并只取电流建立完成后的早期短窗做线性拟合。

使用原因：

- 只看低速近线性加速段，尽量减小阻尼和高转速损失的影响
- 对称正负电流有助于抵消未知负载偏置

核心公式：

```text
J = Kt * (iq_plus - iq_minus) / (alpha_plus - alpha_minus)
```

其中：

- `alpha = dω/dt`
- `iq_plus` 和 `iq_minus` 使用拟合窗内的平均电流

### 步骤 3：通过滑行或稳态法辨识 `B`

优先方法是 coast-down：

1. 用小电流把电机拉到一定速度
2. 下发 `iq = 0`
3. 记录自然减速过程
4. 拟合 `ln(ω)` 的斜率

公式：

```text
B = -J * d(ln(ω)) / dt
```

如果滑行法样本不足，代码可回退到稳态法：

```text
B = Kt * iq / ωss
```

### 步骤 4：选择目标带宽 `wb`

代码支持三种输入方式：

1. 直接给 `target_bandwidth_rad_s`
2. 给 `current_loop_bandwidth_rad_s` 和 `speed_to_current_bandwidth_ratio`
3. 给 `target_time_constant_s`

对应关系：

```text
wb = target_bandwidth_rad_s
wb = current_loop_bandwidth_rad_s / speed_to_current_bandwidth_ratio
wb = 1 / target_time_constant_s
```

如果已经知道电流环带宽，更推荐第二种，因为它把“速度环应慢于电流环”的设计意图直接写在配置里。

### 步骤 5：生成初始 PI 并换算单位

模型公式默认使用 `rad/s`。

如果控制器内部用 `rpm` 或 `erpm`，代码会自动换算：

- `rpm`：乘 `2π / 60`
- `erpm`：再除以 `pole_pairs`

最后还会做：

- 参数上下限裁剪
- 量化到 `param_quantum`

若开启 `identification.exit_after_parameter_generation`：

- 该初始 PI 会直接写入 `identification/result.json`
- 程序直接退出，不进入后续试验轮次

## 运行时流程拆解

### 阶段 1：建立安全边界

速度环每轮试验前都会建立：

- 位置窗口
- 电流峰值上限
- 软限位回转容差

如果反馈越界，trial 立即中断。

### 阶段 2：写入当前 PI

若 `auto_write_params=true`：

- 自动写入 `s_pid_kp / s_pid_ki`
- 仅本轮运行中暂存，不立即保存到 flash
- 整个会话结束后再把历史最优参数带保存标志写入

若关闭自动写参：

- 由 `manual_confirm_each_iteration` 控制每轮是否等待人工确认

### 阶段 3：执行 speed trial

每个目标转速都会：

1. 周期性发送 `send_rpm`
2. 周期性读取反馈
3. 记录 `rpm / current / pos`
4. 在位置窗口附近按配置决定是否自动反向

原始数据会写入：

```text
trials/trial_XXX_YYYYrpm_raw.csv
```

### 阶段 4：计算指标与评分

每轮会计算：

- `steady_error`
- `signed_steady_error`
- `overshoot`
- `settling_time`
- `current_efficiency`

再按 `kpi_weights` 做加权求和。

默认权重以 `autotune/config/default_config.yaml` 为准，目前速度环默认：

```text
steady_error: 0.60
overshoot: 0.15
settling_time: 0.25
current_efficiency: 0.00
```

评分细节见：[`autotune_scoring.md`](autotune_scoring.md)

### 阶段 5：rule-based 微调

速度环下一轮参数更新规则可概括为：

1. 稳态误差偏大时，优先增加 `kp / ki`
2. 峰值超调大时，回拉 `kp`，并小幅回拉 `ki`
3. 稳态已经高于目标时，再次回拉 `kp / ki`
4. 电流利用过高时，额外回拉 `kp`

当前策略特点：

- 明显偏向先把误差压下来
- 对 `ki` 调整较积极
- 通过 `signed_steady_error` 区分“只是尖峰超调”还是“稳态已经超目标”

当前实现支持单次运行内的两阶段微调：

- 粗调阶段：使用 `step_scale` 做较大步长搜索，优先把参数推入可用区间
- 精调阶段：使用 `fine_step_scale`，并从历史最优参数重新起步做局部细化
- 若未配置 `coarse_iterations / fine_iterations`，则回退到旧的单阶段 `max_iterations` 模式
- 每轮 `trial_XXX_metrics.json` 会额外记录 `stage`、`stage_iteration`、`stage_step_scale`

### 阶段 6：避免重复候选

速度环相较位置环多了一层“小步探索”逻辑：

- 先得到理论上的 `proposed_next_pi`
- 若该候选与历史已试参数过近，则按多个比例做小步过渡
- 目的是避免不同轮次长期卡在完全相同的参数点

### 阶段 7：保存历史最优

会话结束后，程序会：

1. 找到评分最低的那一轮参数
2. 写入 `best_params.json`
3. 在自动写参模式下把这组参数保存写回 VESC
4. 生成 `summary.md`

如果运行在“辨识后直接退出”模式：

- 不会进入 trial 采样与评分阶段
- `best_params.json` 会直接保存辨识生成的极点对消 PI
- `trials/all_trials.json` 为空列表

## 典型产物

一次速度环运行通常会得到：

1. `session_meta.json`
2. `best_params.json`
3. `summary.md`
4. `trials/trial_XXX_metrics.json`
5. `trials/trial_XXX_YYYYrpm_raw.csv`
6. `identification/result.json`
7. `identification/j_plus_raw.csv`
8. `identification/j_minus_raw.csv`
9. `identification/b_coast_down_raw.csv` 或 `identification/b_steady_state_raw.csv`

## 实操建议

1. 第一轮辨识优先空载，不要直接带强外载开始
2. `iq_test` 先小后大，能量化出加速度即可，不要一开始就逼近限流
3. 如果已知 VESC 电流环带宽，优先用 `current_loop_bandwidth_rad_s` 驱动速度环带宽选取
4. 如果首轮 PI 明显过慢，可以增大 `wb`
5. 如果首轮 PI 明显振荡、冲电流或撞位置窗口，应减小 `wb` 或降低 `iq_test`

## 相关文档

- 工程流程：[`autotune_workflow.md`](autotune_workflow.md)
- 评分标准：[`autotune_scoring.md`](autotune_scoring.md)
- 位置环完整说明：[`position_tuning_full.md`](position_tuning_full.md)
