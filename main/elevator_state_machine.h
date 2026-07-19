/*
 * elevator_state_machine.h
 * 电梯乘运质量检测仪 —— 运行状态识别状态机（v2：加入持续时间去抖动）
 *
 * v2 更新说明：
 *   v1 版本只要瞬间超过阈值就立刻切状态，导致手部轻微晃动、开关门碰撞
 *   都会被误判成"电梯启动/停止"，实测中会看到状态在几十毫秒内反复横跳。
 *
 *   v2 加入了"持续时间确认"：加速度超过阈值后，必须连续保持
 *   ES_DEBOUNCE_SAMPLES 次采样都满足条件，才真正切换状态；
 *   中途只要有一次不满足，计数器清零重新开始计。
 *   这样能过滤掉瞬时的碰撞/晃动，只有真正持续的运动才会被认定为电梯启停。
 *
 * 逻辑说明：
 *   静止(IDLE) --[|az变化|持续超过阈值达0.3秒]--> 运行中(RUNNING)
 *   运行中(RUNNING) --[|az变化|持续回落到平稳达0.3秒]--> 静止(IDLE)
 *
 * 使用方式：
 *   1. 在 loop() / task 循环里，每次读到新的加速度数据后调用 es_update()
 *   2. es_update() 返回本次是否发生了状态切换，方便你决定何时开始/结束记录SD卡数据
 *   3. 通过 es_get_state() 随时查询当前状态，用于LVGL界面显示
 */

#ifndef ELEVATOR_STATE_MACHINE_H
#define ELEVATOR_STATE_MACHINE_H

#include <stdbool.h>

// ---------- 可调参数 ----------
// 启动/制动加速度阈值（单位: m/s^2）
#define ES_START_THRESHOLD   0.35f
#define ES_STOP_THRESHOLD    0.15f

// 去抖动所需的连续采样次数。假设采样率约100Hz（每次约10ms），
// 30次 ≈ 0.3秒，即加速度变化必须持续0.3秒以上才算真正的状态切换。
// 如果实测中还是容易误触发，可以调大这个值（比如50，对应0.5秒）；
// 如果发现真实的电梯启动都判断得太慢，可以适当调小。
#define ES_DEBOUNCE_SAMPLES  30

// ---------- 状态定义 ----------
typedef enum {
    ES_STATE_IDLE = 0,     // 静止 / 等待
    ES_STATE_RUNNING = 1   // 运行中（含启动、匀速、制动，简单版本不细分）
} elevator_state_t;

// ---------- 状态机对象 ----------
typedef struct {
    elevator_state_t state;   // 当前已确认的状态
    float baseline_az;        // 静止时的 Z 轴基准加速度（重力，约9.8，需要开机时先标定）
    bool  baseline_ready;     // 基准值是否已经标定完成
    int   debounce_counter;   // 去抖动计数器：记录候选状态已经连续保持了多少次采样
} elevator_sm_t;

// 初始化状态机
void es_init(elevator_sm_t *sm);

// 用一次新的三轴加速度采样更新状态机
// 返回值: true 表示这次调用发生了"确认后的"状态切换，false 表示未切换（包括仍在去抖动计数中的情况）
bool es_update(elevator_sm_t *sm, float ax, float ay, float az);

// 获取当前状态
elevator_state_t es_get_state(const elevator_sm_t *sm);

// ---------- v3新增：手动干预接口（供按键任务调用） ----------

// 手动强制设置状态（用于按键手动开始/停止，覆盖掉自动阈值判断的结果）。
// 会重置去抖动计数器，避免手动切换后自动检测立刻又把状态切回去。
void es_force_state(elevator_sm_t *sm, elevator_state_t new_state);

// 手动触发重新标定基准值（用于设备挪动位置/角度后，不用重启就能重新校准静止基准）。
// 只应该在ES_STATE_IDLE时调用，运行中调用会导致基准值被运动中的数据污染。
void es_reset_calibration(elevator_sm_t *sm);

#endif // ELEVATOR_STATE_MACHINE_H
