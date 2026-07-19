/*
 * elevator_state_machine.c
 * 状态机实现（v2：加入持续时间去抖动）
 */

#include "elevator_state_machine.h"
#include <math.h>

#define ES_CALIBRATION_SAMPLES 20

static int calib_count = 0;
static float calib_sum_az = 0.0f;

void es_init(elevator_sm_t *sm)
{
    sm->state = ES_STATE_IDLE;
    sm->baseline_az = 9.8f;
    sm->baseline_ready = false;
    sm->debounce_counter = 0;
    calib_count = 0;
    calib_sum_az = 0.0f;
}

bool es_update(elevator_sm_t *sm, float ax, float ay, float az)
{
    // ---------- 第一步：开机自动标定基准 ----------
    if (!sm->baseline_ready) {
        calib_sum_az += az;
        calib_count++;
        if (calib_count >= ES_CALIBRATION_SAMPLES) {
            sm->baseline_az = calib_sum_az / ES_CALIBRATION_SAMPLES;
            sm->baseline_ready = true;
        }
        return false;
    }

    // ---------- 第二步：计算相对于基准的净加速度变化量 ----------
    float delta_az = fabsf(az - sm->baseline_az);

    bool changed = false;

    // ---------- 第三步：阈值判断 + 持续时间去抖动 ----------
    if (sm->state == ES_STATE_IDLE) {
        // 当前是静止状态，看是否"持续"超过启动阈值
        if (delta_az > ES_START_THRESHOLD) {
            sm->debounce_counter++;
            if (sm->debounce_counter >= ES_DEBOUNCE_SAMPLES) {
                // 连续满足条件达到要求的采样次数，确认状态切换
                sm->state = ES_STATE_RUNNING;
                sm->debounce_counter = 0;
                changed = true;
            }
            // 还没达到确认次数：先不切状态，继续累计计数（不return，让下面的写入正常发生）
        } else {
            // 条件中断了（比如只是瞬间碰了一下，加速度又回落了），清零重新计数
            sm->debounce_counter = 0;
        }
    } else { // ES_STATE_RUNNING
        // 当前是运行状态，看是否"持续"回落到停止阈值以下
        if (delta_az < ES_STOP_THRESHOLD) {
            sm->debounce_counter++;
            if (sm->debounce_counter >= ES_DEBOUNCE_SAMPLES) {
                sm->state = ES_STATE_IDLE;
                sm->debounce_counter = 0;
                changed = true;
            }
        } else {
            sm->debounce_counter = 0;
        }
    }

    return changed;
}

elevator_state_t es_get_state(const elevator_sm_t *sm)
{
    return sm->state;
}