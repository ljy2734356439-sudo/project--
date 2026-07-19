/*
 * shared_data.h
 * 多任务共享的数据结构定义
 *
 * 设计思路：
 *   各个采集任务（IMU、麦克风）各自独立运行，采集到新数据后，
 *   通过互斥锁(mutex)保护地写入这个全局结构体。
 *   显示任务(display_task)需要画面时，同样通过互斥锁读取最新值。
 *
 *   这样各任务之间不需要互相等待，只在读写这一小块内存时短暂加锁，
 *   不会因为屏幕刷新慢而拖累传感器采样速率。
 */

#ifndef SHARED_DATA_H
#define SHARED_DATA_H

#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "elevator_state_machine.h"

// ---------- 共享数据结构 ----------
typedef struct {
    // IMU 数据（由 sensor_task 写入）
    float accel_x, accel_y, accel_z;   // m/s^2（经过IIR低通滤波）
    float accel_magnitude;             // 3轴合成加速度，用于冲击强度评估 (m/s^2)
    float temperature;                 // 摄氏度
    float humidity;                    // %RH
    elevator_state_t elevator_state;   // 当前电梯运行状态

    // 音频数据（由 audio_task 写入）
    float noise_db;                    // A计权声压级估算值 (dB)

    // 新增：用于给屏幕读取的最新评分结果
    bool has_score_report;     // 是否有可用的评分报告
    float total_score;         // 综合评分（例如 2.4）
    char score_level[8];       // 等级（例如 "差" 或 "BAD"）
    float score_vibe;          // 振动分项分
    float score_noise;         // 噪声分项分
    // 数据有效性标志，各任务首次成功采样后置true，
    // display_task 可以据此判断"是否所有传感器都已经ready"
    bool imu_ready;
    bool audio_ready;
} sensor_data_t;

// ---------- 全局共享实例与互斥锁 ----------
extern sensor_data_t g_sensor_data;
extern SemaphoreHandle_t g_data_mutex;

// ---------- 状态切换事件队列 ----------
// sensor_task 检测到电梯 静止<->运行 切换时，往这个队列丢一个事件，
// 其他关心"何时开始/结束一次检测"的任务（比如未来的SD卡记录任务、评分任务）
// 从队列里取，而不是不断轮询共享结构体里的state字段，更省CPU也更及时。
typedef enum {
    EVT_ELEVATOR_START = 0,
    EVT_ELEVATOR_STOP  = 1,
} elevator_event_t;

extern QueueHandle_t g_state_event_queue;

// ---------- 初始化函数 ----------
// 必须在创建各个任务之前调用一次，创建mutex和queue
void shared_data_init(void);

// ---------- 便捷读写封装（内部处理加锁/解锁） ----------
void shared_data_write_imu(float ax, float ay, float az, float accel_mag, float temp, float humi, elevator_state_t state);
void shared_data_write_audio(float noise_db);
sensor_data_t shared_data_read(void); // 返回一份快照副本，读取时不用担心被其他任务改到一半

#endif // SHARED_DATA_H
