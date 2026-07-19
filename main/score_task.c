/*
 * score_task.c
 * v3更新：新增"冲击持续时间"统计——跟踪本次运行中加速度持续偏离基准超过
 * 冲击阈值(SHOCK_THRESHOLD)的最长连续时长，作为客户需求文档里提到的
 * "启停冲击持续时间"特征参数，一并纳入主观评分计算。
 *
 * 工作流程：
 *   1. 监听 g_state_event_queue（sensor_task 在状态机确认切换时会往这里发事件）
 *   2. 收到 EVT_ELEVATOR_START：重置本次运行的统计量，开始记录
 *   3. 运行期间：每隔一小段时间从共享数据里取一次快照，更新"最大振动幅度"、
 *      "最大jerk"、"最长冲击持续时间"、噪声/温湿度的累加平均
 *   4. 收到 EVT_ELEVATOR_STOP：用统计到的数据算出6维主观评分+综合分，打印报告并更新屏幕
 *
 * 关于冲击持续时间统计精度的说明：
 *   这个任务的采样节拍是xQueueReceive的50ms超时决定的，不是sensor_task那边
 *   严格的10ms(100Hz)。所以这里测出来的"持续时间"是近似值，精度在±50ms左右，
 *   对于毫秒级要求非常严格的场合不够精确，但作为"数量级"层面的特征参数、
 *   用来跟"良好/拖沓"这种模糊等级挂钩是足够的。如果后续需要更精确的时间测量，
 *   要把这段逻辑挪到sensor_task里跟着100Hz的采样节拍一起做。
 */

#include <math.h>
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "score_task.h"
#include "shared_data.h"
#include "subjective_score.h"

static const char *TAG = "SCORE_TASK";

// 引用 shared_data.c 中定义的全局变量
extern sensor_data_t g_sensor_data;
extern SemaphoreHandle_t g_data_mutex;

// 冲击阈值：比状态机里"判定电梯启动"的阈值(0.35)更高一些，
// 专门用来标记"明显的冲击/顿挫感"这种更剧烈的加速度偏离，而不是普通的启停过程。
#define SHOCK_THRESHOLD  0.5f

void score_task(void *pvParameters)
{
    ESP_LOGI(TAG, "score_task 启动");

    elevator_event_t evt;

    bool collecting = false;
    float local_baseline_az = 0.0f;
    float max_amplitude = 0.0f;
    float max_jerk = 0.0f;
    float prev_az = 0.0f;
    bool has_prev = false;

    // 冲击持续时间统计相关状态
    bool in_shock = false;
    TickType_t shock_start_tick = 0;
    float max_shock_duration_ms = 0.0f;

    float noise_sum = 0.0f;
    int noise_count = 0;
    float temp_sum = 0.0f;
    float humi_sum = 0.0f;
    int env_count = 0;

    while (1) {
        // 等待最多50ms看有没有状态事件
        if (xQueueReceive(g_state_event_queue, &evt, pdMS_TO_TICKS(50)) == pdTRUE) {

            if (evt == EVT_ELEVATOR_START) {
                // 开始一次新的记录，所有统计量清零
                collecting = true;
                max_amplitude = 0.0f;
                max_jerk = 0.0f;
                has_prev = false;
                in_shock = false;
                max_shock_duration_ms = 0.0f;
                noise_sum = 0.0f; noise_count = 0;
                temp_sum = 0.0f; humi_sum = 0.0f; env_count = 0;

                sensor_data_t d = shared_data_read();
                local_baseline_az = d.accel_z; // 用运行开始那一刻的值作为本次的参照基准
                ESP_LOGI(TAG, "开始记录本次运行数据");

            } else { // EVT_ELEVATOR_STOP
                collecting = false;

                // 如果电梯停止时冲击状态还没结束(比如刚好卡在临界点)，
                // 也要把这最后一段时间计入，避免漏算
                if (in_shock) {
                    float duration_ms = (float)((xTaskGetTickCount() - shock_start_tick) * portTICK_PERIOD_MS);
                    if (duration_ms > max_shock_duration_ms) {
                        max_shock_duration_ms = duration_ms;
                    }
                    in_shock = false;
                }

                float avg_db   = (noise_count > 0) ? (noise_sum / noise_count) : 0.0f;
                float avg_temp = (env_count > 0) ? (temp_sum / env_count) : 25.0f;
                float avg_humi = (env_count > 0) ? (humi_sum / env_count) : 50.0f;

                subjective_scores_t s = compute_subjective_scores(
                    max_amplitude, max_jerk, max_shock_duration_ms, avg_db, avg_temp, avg_humi);

                const char *grade_str = score_to_grade(s.overall_score);

                ESP_LOGW(TAG, "========== 本次运行评分报告 ==========");
                ESP_LOGW(TAG, "原始统计: 振动峰值=%.3fm/s^2  峰值jerk=%.3fm/s^2  冲击时长=%.0fms  噪声均值=%.1fdB  温度均值=%.1fC  湿度均值=%.1f%%",
                          max_amplitude, max_jerk, max_shock_duration_ms, avg_db, avg_temp, avg_humi);
                ESP_LOGW(TAG, "分项评分: 振动=%.1f  平稳=%.1f  冲击时长=%.1f  噪声=%.1f  温度=%.1f  湿度=%.1f",
                          s.vibration_score, s.smoothness_score, s.shock_duration_score,
                          s.noise_score, s.thermal_score, s.humidity_score);
                ESP_LOGW(TAG, "综合评分: %.1f 分   等级: %s", s.overall_score, grade_str);
                ESP_LOGW(TAG, "=======================================");

                // 用全局互斥锁安全地把结果写进共享结构体，供屏幕显示
                if (g_data_mutex != NULL && xSemaphoreTake(g_data_mutex, portMAX_DELAY) == pdTRUE) {
                    g_sensor_data.has_score_report = true;
                    g_sensor_data.total_score = s.overall_score;
                    g_sensor_data.score_vibe = s.vibration_score;
                    g_sensor_data.score_noise = s.noise_score;

                    strncpy(g_sensor_data.score_level, grade_str, sizeof(g_sensor_data.score_level) - 1);
                    g_sensor_data.score_level[sizeof(g_sensor_data.score_level) - 1] = '\0';

                    xSemaphoreGive(g_data_mutex);
                }
            }
        }

        // 运行期间持续统计
        if (collecting) {
            sensor_data_t d = shared_data_read();

            float amplitude = fabsf(d.accel_z - local_baseline_az);
            if (amplitude > max_amplitude) {
                max_amplitude = amplitude;
            }

            if (has_prev) {
                float jerk = fabsf(d.accel_z - prev_az);
                if (jerk > max_jerk) {
                    max_jerk = jerk;
                }
            }
            prev_az = d.accel_z;
            has_prev = true;

            // ---- 冲击持续时间统计 ----
            if (amplitude > SHOCK_THRESHOLD) {
                if (!in_shock) {
                    // 刚开始进入冲击状态，记录起始时刻
                    in_shock = true;
                    shock_start_tick = xTaskGetTickCount();
                }
                // 如果已经在冲击状态里，这里不用做什么，等结束时统一结算时长
            } else {
                if (in_shock) {
                    // 冲击状态刚结束，结算这一段的持续时长
                    float duration_ms = (float)((xTaskGetTickCount() - shock_start_tick) * portTICK_PERIOD_MS);
                    if (duration_ms > max_shock_duration_ms) {
                        max_shock_duration_ms = duration_ms;
                    }
                    in_shock = false;
                }
            }

            if (d.audio_ready) {
                noise_sum += d.noise_db;
                noise_count++;
            }
            temp_sum += d.temperature;
            humi_sum += d.humidity;
            env_count++;
        }
    }
}
