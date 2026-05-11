# 自整定评分标准

本文档说明 `autotune/common/analysis/metrics.py` 中速度环、位置环指标的定义方式，以及 `default_config.yaml` 中默认加权评分的意义。

返回总工程说明：[`../README.md`](../README.md)

## 总体规则

每次 trial 会先计算一组基础指标，再通过加权求和得到单轮分数：

```text
score = Σ(metric_i * weight_i)
```

分数越小越好。

实现位置：

- 指标计算：`autotune/common/analysis/metrics.py`
- 权重配置：`autotune/config/default_config.yaml`

## 速度环评分

### 指标定义

速度环入口使用：

```text
evaluate_speed_trial(samples, target_rpm, use_abs_rpm=True)
```

当前指标包括：

1. `steady_error`
   - 定义：稳态误差的 MAE
   - 公式：

```text
steady_error = mean(|rpm_i - target|)
```

2. `signed_steady_error`
   - 定义：带符号稳态偏差
   - 公式：

```text
signed_steady_error = mean(rpm_i) - target
```

3. `overshoot`
   - 定义：采样期间最高转速相对目标的超调量
   - 公式：

```text
overshoot = max(0, max(rpm_i) - target)
```

4. `settling_time`
   - 定义：第一次进入误差容差带的时间
   - 当前容差：

```text
tolerance = max(30 rpm, 1% * |target|)
```

### 速度环稳态容差如何确定

这里需要区分两个容易混淆的概念：

1. `settling_time` 使用的容差带
2. `steady_error / signed_steady_error` 统计时使用的稳态样本筛选范围

#### 1. `settling_time` 的稳态容差带

速度环当前把“进入稳态”的判定写成：

```text
|rpm - target| <= tolerance
```

其中：

```text
tolerance = max(30 rpm, 1% * |target|)
```

这表示容差带由两部分共同决定：

- 下限是固定的 `30 rpm`
- 当目标转速较高时，容差带随目标转速按 `1%` 放大

这样设计的原因是：

- 低速段如果只按比例容差，容差会过小，容易被量化误差、采样噪声和摩擦扰动放大
- 高速段如果只用固定 `30 rpm`，容差又会过严，不利于反映“相对误差已经足够小”的情况

因此代码采用“固定下限 + 相对比例”的混合判定。

几个直观例子：

```text
target = 200 rpm   -> tolerance = max(30, 2)   = 30 rpm
target = 2000 rpm  -> tolerance = max(30, 20)  = 30 rpm
target = 5000 rpm  -> tolerance = max(30, 50)  = 50 rpm
target = 10000 rpm -> tolerance = max(30, 100) = 100 rpm
```

也就是说：

- 在低中速段，当前实现更偏向使用固定 `30 rpm` 容差
- 在更高转速段，才逐渐切换到按 `1%` 相对误差判定

#### 2. `steady_error` 不是直接靠这个容差带算出来的

当前实现中：

- `settling_time` 依赖上面的容差带
- `steady_error` 和 `signed_steady_error` 则不是用这个容差带筛样本

它们使用的是“尖峰过滤”范围：

```text
spike_upper_bound = |target| + steady_spike_margin_rpm
steady_slice = {rpm_i | |rpm_i| <= spike_upper_bound}
```

所以：

- `settling_time` 关注“什么时候第一次算进入可接受误差带”
- `steady_error` 关注“在去掉异常尖峰后，整体稳态误差平均有多大”

这两个规则分别服务于：

- 动态收敛判定
- 稳态误差评估

#### 3. 这套容差的工程含义

当前速度环的稳态容差不是从电机模型严格推导出来的，而是一个偏工程化、便于通用调参的经验判定：

- 对低速目标不过分苛刻
- 对高速目标不失去比例意义
- 允许 rule-based 在一套统一口径下判断是否“明显过慢”

如果后续发现：

- 低速段判定太宽松
- 高速段判定太严格或太宽松

则可以把这条规则进一步参数化，而不是固定写死在代码里。

5. `current_efficiency`
   - 定义：采样期间电流绝对值平均值
   - 公式：

```text
current_efficiency = mean(|current_i|)
```

### 稳态取样规则

速度环不会直接用全量数据计算稳态误差，而是先做一个“尖峰过滤”：

```text
spike_upper_bound = |target| + steady_spike_margin_rpm
steady_slice = {rpm_i | |rpm_i| <= spike_upper_bound}
```

默认 `steady_spike_margin_rpm` 在配置中可调，用于避免极端尖峰把稳态误差统计拉坏。

### 默认权重

默认配置：

```text
steady_error: 0.55
overshoot: 0.15
settling_time: 0.25
current_efficiency: 0.05
```

可见当前策略更偏向：

- 先解决稳态误差
- 其次关注收敛速度
- 再处理超调
- 最后兼顾电流占用

### 与调参规则的关系

虽然 `signed_steady_error` 不直接进入加权分数，但它会参与下一轮参数更新判断：

- `signed_steady_error > 0`：说明稳态落在目标上方，倾向于判定为稳态超目标
- `signed_steady_error < 0`：说明稳态落在目标下方，倾向于判定为稳态偏低

因此速度环是“评分”和“更新规则”共同驱动的：

- 评分决定这一轮好不好
- 规则决定下一轮往哪个方向改

## 位置环评分

### 指标定义

位置环入口使用：

```text
evaluate_position_trial(samples, target_deg)
```

当前指标包括：

1. `position_stability`
   - 定义：落入稳态容差带后的带符号误差均值绝对值
   - 公式：

```text
position_stability = |mean(steady_errors)|
```

2. `load_hold`
   - 当前实现与 `position_stability` 相同
   - 用于表示保持阶段误差

3. `step_response`
   - 定义：第一次进入稳态容差带的时间

4. `rebound`
   - 定义：稳态容差带内误差的最大绝对值
   - 用于反映目标附近来回摆动或回摆程度

5. `current_efficiency`
   - 定义：采样期间电流绝对值平均值

### 稳态判定规则

位置环当前采用全局固定稳态带：

```text
steady_tolerance_deg = 1.0 deg
```

也就是只有满足：

```text
|pos_i - target_deg| <= 1.0 deg
```

的点才会被视为稳态点。

如果存在稳态点：

- `steady_state_entered = 1`
- `steady_points_count = 稳态点数量`

如果完全没有进入稳态带：

- `position_stability = 1e9`
- `load_hold = 1e9`
- `rebound = max(abs_errors)`

这相当于对“始终未进入稳态”的 trial 给出极高惩罚。

### 最近点辅助指标

位置环还会额外保存几项辅助信息，便于判断是超目标还是没够到：

- `min_abs_error`
- `min_error_signed`
- `closest_point_assessment`

其中：

- `min_error_signed > 0` 表示最近点发生在目标上方，记为 `overshoot`
- `min_error_signed < 0` 表示最近点在目标下方，记为 `undershoot`

### 默认权重

默认配置：

```text
position_stability: 0.35
load_hold: 0.20
step_response: 0.25
rebound: 0.10
current_efficiency: 0.10
```

当前策略偏向：

- 先保证目标附近稳定
- 再保证保持阶段不漂
- 再兼顾响应速度
- 最后约束回摆和电流

## 如何解读分数

### 速度环

如果速度环总分高，优先看：

1. `steady_error` 是否明显偏大
2. `settling_time` 是否长期过长
3. `overshoot` 是否只是瞬时尖峰还是已经形成稳态超目标

### 位置环

如果位置环总分高，优先看：

1. 有没有进入 `±1 deg` 稳态带
2. `position_stability` 是否已经接近零
3. `step_response` 是否过慢
4. `rebound` 是否说明目标点附近存在反复摆动

## 相关文档

- 工程流程：[`autotune_workflow.md`](autotune_workflow.md)
- 速度环完整说明：[`speed_tuning_full.md`](speed_tuning_full.md)
- 位置环完整说明：[`position_tuning_full.md`](position_tuning_full.md)
