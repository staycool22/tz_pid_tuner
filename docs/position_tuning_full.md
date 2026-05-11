# 位置环整定完整说明

本文档面向 `autotune/position_tuner`，完整说明位置环初值构造、模型层与 VESC 参数层的映射、trial 执行过程、评分逻辑和微调策略。

返回总工程说明：[`../README.md`](../README.md)

## 文档目标

位置环文档重点回答四个问题：

1. 首轮 `p_pid_kp / p_pid_kd_proc` 是怎么生成的
2. 为什么位置环模型要建立在速度环之上
3. 为什么 `Kp_pos_model` 不能直接等同于 VESC 的 `p_pid_kp`
4. 位置环 trial、评分和下一轮建议参数是怎样串起来的

## 适用前提

位置环整定默认建立在以下前提上：

- 电流环已经闭环
- 速度环已经闭环且基本稳定
- 位置反馈可靠
- 机械运动方向、量程、软限位已经确认
- 第一轮最好在空载或轻载条件下进行

如果速度环本身不稳定，位置环常见问题会被放大为：

- 目标点附近反复摆动
- 轻载保持误差偏大
- 位置环参数看似不收敛，实际是内环基础不稳

## 总流程图

```text
[确认速度环已稳定]
            |
            v
[确认位置反馈和安全边界]
            |
            v
[读取速度环参考信息]
speed_bandwidth_rad_s 或 speed_time_constant_s
            |
            v
[选择位置环带宽]
wb_pos = wb_speed / ratio
            |
            v
[建立位置环模型]
G_pos(s) ≈ 1 / s
必要时考虑闭合速度环近似
            |
            v
[生成模型层位置增益]
Kp_pos_model = wb_pos
            |
            v
[映射到 pass_through 等效增益]
K_eq -> p_pid_kp
            |
            v
[写入当前参数并执行 trial]
send_pass_through
            |
            v
[计算评分]
position_stability / step_response / rebound / current_efficiency
            |
            v
[rule-based 微调]
kp 或 kd_proc
            |
            v
[保存历史最优参数与摘要]
```

## 位置环模型

### 为什么位置环建立在速度环之上

位置环不直接驱动电流，而是通过内部速度调节能力去逼近目标位置。

如果速度环相对位置环足够快，可以先把位置侧对象简化成：

```text
G_pos(s) ≈ 1 / s
```

这表示：

- 位置是速度的积分
- 位置环天然已经含有一个积分关系
- 因此首轮不宜一上来就加太强的积分项

### 更完整的近似

如果把闭合速度环近似成一阶系统：

```text
G_speed_cl(s) ≈ ωbw_speed / (s + ωbw_speed)
```

那么从速度指令到位置输出可写成：

```text
G_pos_eff(s) ≈ ωbw_speed / ( s (s + ωbw_speed) )
```

但在“位置环带宽显著低于速度环”的前提下，工程上通常仍可简化回：

```text
G_pos(s) ≈ 1 / s
```

## 首轮参数构造

### 步骤 1：拿到速度环参考量

位置环初值生成有两个速度环输入选项：

1. `speed_bandwidth_rad_s`
2. `speed_time_constant_s`

若给的是时间常数，则程序内部会转成：

```text
wb_speed = 1 / tau_speed
```

### 步骤 2：选择位置环带宽

位置环带宽也有两种方式：

1. 直接给 `position_bandwidth_rad_s`
2. 给 `position_to_speed_bandwidth_ratio`

若使用比例法：

```text
wb_pos = wb_speed / position_to_speed_bandwidth_ratio
```

默认推荐思路是让位置环明显慢于速度环。

### 步骤 3：生成模型层位置增益

在最简化模型里，位置环首轮从 P 开始：

```text
ω_ref = Kp_pos_model * (θ_ref - θ)
```

因此：

```text
Kp_pos_model = wb_pos
```

这时 `Kp_pos_model` 的物理单位是 `1/s`。

### 步骤 4：映射到 VESC `pass_through`

这里必须区分三层含义：

1. 模型层增益 `Kp_pos_model`
2. 接口等效增益 `K_eq`
3. VESC 内部配置参数 `p_pid_kp`

对于 `send_pass_through(pos, rpm, current)`，内部关系近似为：

```text
position error * p_pid_kp
-> normalized output
-> 0.8 * l_max_erpm
-> speed correction in ERPM
```

因此：

```text
K_eq = p_pid_kp * 0.8 * l_max_erpm
p_pid_kp = K_eq / (0.8 * l_max_erpm)
```

### 输出轴角度解释

若位置指令按输出轴角度解释：

```text
K_eq = wb_pos * gear_ratio * pole_pairs / 6
p_pid_kp = wb_pos * gear_ratio * pole_pairs / (4.8 * l_max_erpm)
```

### 电机轴角度解释

若位置指令按电机轴角度解释：

```text
K_eq = wb_pos * pole_pairs / 6
p_pid_kp = wb_pos * pole_pairs / (4.8 * l_max_erpm)
```

### 为什么不能把 `Kp_pos_model` 直接写进 VESC

因为：

- `Kp_pos_model` 是控制理论层的 `1/s`
- `p_pid_kp` 是 VESC 内部归一化后的无量纲参数
- 中间还隔着 `ERPM/deg` 的接口映射和 `0.8 * l_max_erpm` 的内部放大

所以必须先做映射，不能直接照抄。

## `p_pid_kd_proc` 的角色

首轮位置环默认更偏向：

- 先用 `p_pid_kp` 建立基本回位能力
- 如果目标点附近存在回摆，再增加 `p_pid_kd_proc`

因此在默认配置里：

- `p_pid_kd_proc` 可以从 `0.0` 起步
- 后续由 rule-based 根据 `rebound` 再决定是否抬升

## 运行时流程拆解

### 阶段 1：建立安全边界

位置环会根据配置建立：

- 位置窗口
- 峰值电流限制

一旦反馈越界，trial 会被中断。

### 阶段 2：写入当前参数

若 `auto_write_params=true`：

- 自动写入 `p_pid_kp / p_pid_kd_proc`
- 若与上次写入一致，则跳过重复写参

这并不等于跳过 trial，只是避免对同一组参数反复下发相同写入命令。

### 阶段 3：执行 pass_through trial

每个目标点 trial 都会：

1. 发出 `send_pass_through`
2. 记录位置、速度、电流
3. 把原始数据写入 csv

若 `ui_like_pass_through=true`：

- 命令位置不是瞬时跳到目标点
- 而是按平滑轨迹逐步逼近目标点
- 速度前馈和电流前馈也会一并带入

这样可以让离线整定更接近 UI 中的真实使用方式。

### 阶段 4：计算评分

每个目标点都会计算：

- `position_stability`
- `load_hold`
- `step_response`
- `rebound`
- `current_efficiency`

再对多个目标点取均值，组成当前 iteration 的聚合指标。

评分细节见：[`autotune_scoring.md`](autotune_scoring.md)

### 阶段 5：rule-based 微调

位置环的下一轮更新逻辑比较直接：

1. `rebound > 2.0` 时
   - 增加 `kd_proc`
   - 略微降低 `kp`
2. `position_stability > 0.05` 时
   - 增加 `kp`
3. `step_response > trial_seconds * 0.75` 时
   - 小幅增加 `kp`
4. 若上面都没命中
   - 会轻微衰减 `kd_proc`

这套规则的意图是：

- 回摆优先靠阻尼抑制
- 保持误差和响应过慢优先靠 `kp`
- 不轻易在首轮就把 `kd_proc` 拉高

### 阶段 6：连续改进不足提前停止

位置环当前实现会统计连续改进不足次数：

- 若连续两轮改进都不足 `improve_threshold`
- 则提前停止

若停止时 `position_stability` 仍偏大，程序会给出：

```text
建议先回退并重新整定速度环后再继续位置环
```

这体现了位置环流程对内环基础的依赖。

## 典型产物

一次位置环运行通常会得到：

1. `session_meta.json`
2. `best_params.json`
3. `summary.md`
4. `trials/trial_XXX_metrics.json`
5. `trials/trial_XXX_20deg_raw.csv`
6. `trials/trial_XXX_40deg_raw.csv`
7. `trials/trial_XXX_70deg_raw.csv`

## 实操建议

1. 若速度环带宽还不确定，不要急着放大位置环带宽
2. 首轮建议从 `p_pid_kd_proc = 0` 或很小值起步
3. 若目标附近明显摆动，先看 `rebound` 再决定是否加阻尼
4. 若多轮都停在同一组参数，通常表示当前阈值判断认为“问题不够严重”或“首轮参数已在可接受区间”
5. 若位置环一直无法压低稳态误差，先回头检查速度环是否真的稳

## 相关文档

- 工程流程：[`autotune_workflow.md`](autotune_workflow.md)
- 评分标准：[`autotune_scoring.md`](autotune_scoring.md)
- 速度环完整说明：[`speed_tuning_full.md`](speed_tuning_full.md)
