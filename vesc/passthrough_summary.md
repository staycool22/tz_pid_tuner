# pass_through Interface Summary

## 1. 接口本质

`pass_through(pos, rpm, current)` 不是单环 PID，而是一个三输入级联控制接口:

```text
pos      -> 位置环 -> 速度修正量
rpm      -> 速度基准
current  -> 电流前馈

speed_cmd = rpm + pos_loop_output
iq_cmd    = current + speed_loop_output
```

最终控制量是 `motor->m_iq_set`，它会进入 FOC 的 q 轴电流控制。

## 2. 入口调用路径

### 2.1 CAN 包入口

文件:

- `comm/comm_can.c`
- 搜索关键字: `CAN_PACKET_PASS_THROUGH`

关键逻辑:

```c
case CAN_PACKET_PASS_THROUGH:
    float pos_set = (float)buffer_get_uint16(data8, &ind) / 1e2;
    float vel_set = buffer_get_float16(data8, 1e0, &ind);
    float cur_set = buffer_get_float16(data8, 1e3, &ind);
    mc_interface_set_pass_through(pos_set, vel_set, cur_set);
    break;
```

含义:

- `pos_set`: 角度设定，单位度。
- `vel_set`: 速度设定，单位 ERPM。
- `cur_set`: 电流设定，单位 A。

### 2.2 mc_interface 层

文件:

- `motor/mc_interface.c`
- 搜索关键字: `mc_interface_set_pass_through`

关键逻辑:

```c
motor_now()->m_position_set = pos;
pos += motor_now()->m_conf.p_pid_offset;
pos *= DIR_MULT;

if (encoder_is_configured()) {
    if (conf->foc_encoder_inverted) {
        pos *= -1.0;
    }
}

utils_norm_angle(&pos);
mcpwm_foc_set_pass_through(pos, DIR_MULT * rpm, DIR_MULT * current);
```

作用:

- 对 `pos` 加 `p_pid_offset`
- 应用方向符号 `DIR_MULT`
- 结合编码器反向修正角度符号
- 将角度归一化后送入 FOC 层

### 2.3 FOC 设定层

文件:

- `motor/mcpwm_foc.c`
- 搜索关键字: `mcpwm_foc_set_pass_through`

关键逻辑:

```c
motor->m_pos_pid_set = pos;
motor->m_speed_command_rpm = rpm;
motor->m_i_outer_set = current;
motor->m_control_mode = CONTROL_MODE_PASS_THROUGH;
```

附加逻辑:

- 如果 `s_pid_ramp_erpms_s > 0`，会让真实速度环目标逐步逼近 `rpm`。
- `current` 前馈直接写入 `m_i_outer_set`，没有同等斜坡处理。

## 3. PID 线程路径

文件:

- `motor/mcpwm_foc.c`
- 搜索关键字: `pid_thread`

关键逻辑:

```c
float dt = timer_seconds_elapsed_since(last_time);
foc_run_pid_control_normal(encoder_index_found(), dt, (motor_all_state_t*)&m_motor_1);
foc_run_pid_control_pass_through(encoder_index_found(), dt, (motor_all_state_t*)&m_motor_1);
```

说明:

- `pass_through` 的位置环和速度环都在 `pid_thread` 中周期运行。
- 线程频率由 `sp_pid_loop_rate` 决定。

## 4. 主计算路径

文件:

- `motor/foc_math.c`
- 搜索关键字: `foc_run_pid_control_pass_through`

关键代码:

```c
float speed_cmd = motor->m_speed_command_rpm
        + foc_run_pid_control_pos_arm(index_found, motor->m_pos_pid_set, dt, motor);
float iq_cmd = motor->m_i_outer_set
        + foc_run_pid_control_speed_arm(index_found, speed_cmd, dt, motor);
utils_truncate_number_abs(&iq_cmd, conf_now->lo_current_max * conf_now->l_current_max_scale);
motor->m_iq_set = iq_cmd;
```

含义:

- 位置环输出的是速度修正量。
- 速度环输出的是电流修正量。
- `current` 前馈直接与速度环输出相加。
- 最终结果被限幅后写入 `m_iq_set`。

## 5. 位置环实现说明

函数:

```c
float foc_run_pid_control_pos_arm(bool index_found, float angle_set, float dt, motor_all_state_t *motor)
```

### 5.1 位置反馈来源

文件:

- `motor/mcpwm_foc.c`
- 搜索关键字: `m_pos_pid_now =`

关键逻辑:

```c
if (conf_now->p_pid_ang_div > 0.98 && conf_now->p_pid_ang_div < 1.02) {
    motor_now->m_pos_pid_now = angle_now;
} else {
    motor_now->m_pos_pid_now = motor_now->m_pid_div_angle_accumulator
            + angle_now / conf_now->p_pid_ang_div;
}
```

结论:

- 位置环闭环量是 `m_pos_pid_now`。
- `p_pid_ang_div` 会直接改变位置误差尺度，不只是显示含义。

### 5.2 误差与 PID 公式

文件:

- `motor/foc_math.c`
- 搜索关键字: `float error = utils_angle_difference`
- 搜索关键字: `p_term = error * kp`

关键代码:

```c
float error = utils_angle_difference(angle_set, angle_now);
...
p_term = error * kp;
motor->m_pos_i_term += error * (ki * dt);
...
d_term = (error - motor->m_pos_prev_error) * (kd / motor->m_pos_dt_int);
...
d_term_proc = -utils_angle_difference(angle_now, motor->m_pos_prev_proc)
        * error_sign * (kd_proc / motor->m_pos_dt_int_proc);
```

说明:

- `d_term` 基于误差变化。
- `d_term_proc` 基于过程量变化，更像对实际运动速度做阻尼。

### 5.3 小误差增益衰减

文件:

- `motor/foc_math.c`
- 搜索关键字: `p_pid_gain_dec_angle > 0.1`

关键代码:

```c
if (conf_now->p_pid_gain_dec_angle > 0.1) {
    float min_error = conf_now->p_pid_gain_dec_angle / conf_now->p_pid_ang_div;
    float error_abs = fabs(error);

    if (error_abs < min_error) {
        float scale = error_abs / min_error;
        kp *= scale;
        ki *= scale;
        kd *= scale;
        kd_proc *= scale;
    }
}
```

含义:

- 小误差区会同时削弱位置环 P/I/D。
- 有助于减轻靠近目标点时的抖动。
- 但过大时会让系统在接近目标时显得发软。

### 5.4 输出限幅与单位

文件:

- `motor/foc_math.c`
- 搜索关键字: `utils_truncate_number(&output, -1.0, 1.0)`
- 搜索关键字: `output *= 0.8 * conf_now->l_max_erpm`

关键代码:

```c
float output = p_term + motor->m_pos_i_term + d_term + d_term_proc;
utils_truncate_number(&output, -1.0, 1.0);
...
output *= 0.8 * conf_now->l_max_erpm;
return output;
```

结论:

- 位置环先输出 `-1 ~ 1` 的归一化量。
- 在 `pass_through` 中，它最终被转换成速度修正量，单位是 ERPM。

### 5.5 index 未找到时的特殊逻辑

文件:

- `motor/foc_math.c`
- 搜索关键字: `0.4 * 0.8 * conf_now->l_max_erpm`

关键代码:

```c
if (index_found) {
    output *= 0.8 * conf_now->l_max_erpm;
} else {
    output *= 0.4 * 0.8 * conf_now->l_max_erpm;
}
```

含义:

- 未找到 index 时，位置环输出会被缩小，而不是像普通位置模式那样直接给固定电流找 index。

## 6. 速度环实现说明

函数:

```c
float foc_run_pid_control_speed_arm(bool index_found, float speed_cmd, float dt, motor_all_state_t *motor)
```

### 6.1 速度目标的进入方式

文件:

- `motor/foc_math.c`
- 搜索关键字: `utils_truncate_number_abs(&speed_cmd`
- 搜索关键字: `s_pid_ramp_erpms_s`

关键代码:

```c
utils_truncate_number_abs(&speed_cmd, conf_now->l_max_erpm);

if (conf_now->s_pid_ramp_erpms_s > 0.0) {
    utils_step_towards((float*)&motor->m_speed_pid_set_rpm,
            speed_cmd, conf_now->s_pid_ramp_erpms_s * dt);
} else {
    motor->m_speed_pid_set_rpm = speed_cmd;
}
```

结论:

- `m_speed_command_rpm` 是上层输入。
- `m_speed_pid_set_rpm` 才是当前速度环真正使用的目标。

### 6.2 速度反馈与低速退出

文件:

- `motor/foc_math.c`
- 搜索关键字: `const float rpm = RADPS2RPM_f(motor->m_speed_est_fast)`
- 搜索关键字: `fabsf(motor->m_speed_pid_set_rpm) < conf_now->s_pid_min_erpm`

关键代码:

```c
const float rpm = RADPS2RPM_f(motor->m_speed_est_fast);
float error = motor->m_speed_pid_set_rpm - rpm;

if (fabsf(motor->m_speed_pid_set_rpm) < conf_now->s_pid_min_erpm) {
    motor->m_speed_i_term = 0.0;
    motor->m_speed_prev_error = error;
    motor->m_iq_set = 0.0;
    return 0;
}
```

结论:

- 速度反馈使用 `m_speed_est_fast`，因此比较灵敏，也更容易吃到低速噪声。
- 低于 `s_pid_min_erpm` 时，速度环直接退出。

### 6.3 PID 公式与输出单位

文件:

- `motor/foc_math.c`
- 搜索关键字: `p_term = error * conf_now->s_pid_kp`
- 搜索关键字: `return output * conf_now->lo_current_max`

关键代码:

```c
p_term = error * conf_now->s_pid_kp * (1.0 / 20.0);
d_term = (error - motor->m_speed_prev_error) * (conf_now->s_pid_kd / dt) * (1.0 / 20.0);
...
float output = p_term + motor->m_speed_i_term + d_term;
utils_truncate_number_abs(&output, 1.0);
...
motor->m_speed_i_term += error * conf_now->s_pid_ki * dt * (1.0 / 20.0);
...
return output * conf_now->lo_current_max * conf_now->l_current_max_scale;
```

结论:

- 速度环先输出归一化量，再映射成电流修正量。
- 在 `pass_through` 中，速度环最终输出单位是 A。

### 6.4 D 项滤波与刹车允许

文件:

- `motor/foc_math.c`
- 搜索关键字: `UTILS_LP_FAST(motor->m_speed_d_filter`
- 搜索关键字: `s_pid_allow_braking`

关键代码:

```c
UTILS_LP_FAST(motor->m_speed_d_filter, d_term, conf_now->s_pid_kd_filter);
...
if (!conf_now->s_pid_allow_braking) {
    if (rpm > 20.0 && output < 0.0) {
        output = 0.0;
    }
    if (rpm < -20.0 && output > 0.0) {
        output = 0.0;
    }
}
```

含义:

- `s_pid_kd_filter` 决定 D 项平滑程度。
- 如果不允许制动，速度环不能主动给反向制动力矩。

### 6.5 代码里有前馈痕迹，但当前未真正参与输出

文件:

- `motor/foc_math.c`
- 搜索关键字: `ff_term =`

关键代码:

```c
ff_term = RPM2RADPS_f(motor->m_speed_pid_set_rpm - last_set_rpm)/dt * inertial;
...
float output = p_term + motor->m_speed_i_term + d_term;
```

说明:

- 代码里计算了 `ff_term`。
- 但它没有被加到最终 `output` 中。
- 当前调参不能把动态改善寄希望于这个量。

## 7. 最终输出路径

文件:

- `motor/foc_math.c`
- 搜索关键字: `motor->m_iq_set = iq_cmd`
- `motor/mcpwm_foc.c`
- 搜索关键字: `float iq_set_tmp = motor_now->m_iq_set`

执行链路:

```text
pos / rpm / current
-> pass_through 级联计算
-> m_iq_set
-> FOC q 轴电流环
-> PWM / 电压矢量
-> 电机
```

## 8. 调参时最容易踩的坑

### 8.1 `current` 会掩盖速度 PID

如果 `current` 前馈过大，电机即使速度环调得一般，也可能看起来能动。

### 8.2 低速时速度环可能退出，但电机仍有力矩

原因是速度环返回 0 后，`current` 前馈仍然可能继续生效。

### 8.3 `p_pid_ang_div` 会改变参数手感

它会改变位置误差尺度，因此改它以后，位置环参数通常要重新看。

### 8.4 速度斜坡会让你误判 PID 太弱

如果 `s_pid_ramp_erpms_s` 很小，内部速度目标爬坡很慢，外观看起来像响应太钝。

### 8.5 电流限幅可能受 `l_current_max_scale` 的二次影响

当前 `pass_through` 里，速度环输出与最终限幅都使用了:

```c
lo_current_max * l_current_max_scale
```

如果运行时 `lo_current_max` 已经由 `l_current_max * l_current_max_scale` 得到，则等效上限可能进一步收紧，需要结合实机配置核查。

## 9. 简要调参顺序

推荐顺序:

1. 先确认接口方向、零位、编码器方向。
2. `current = 0`，先调速度环。
3. 速度环稳定后，再调位置环。
4. 再检查低速门限、制动、斜坡、限流。
5. 最后加入电流前馈补偿负载。

## 10. 一句话总结

`pass_through` 的本质是位置环出速度、速度环出电流、前馈电流直接叠加，理解这条主线之后，绝大多数调参现象都能找到对应的源码原因。
