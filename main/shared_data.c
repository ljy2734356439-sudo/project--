/*
 * shared_data.c
 */

#include "shared_data.h"
#include <string.h>

sensor_data_t g_sensor_data;
SemaphoreHandle_t g_data_mutex = NULL;
QueueHandle_t g_state_event_queue = NULL;

void shared_data_init(void)
{
    memset(&g_sensor_data, 0, sizeof(g_sensor_data));

    g_data_mutex = xSemaphoreCreateMutex();

    // 队列长度设为5，防止极端情况下事件来不及处理导致丢事件
    g_state_event_queue = xQueueCreate(5, sizeof(elevator_event_t));
}

void shared_data_write_imu(float ax, float ay, float az, float accel_mag, float temp, float humi, elevator_state_t state)
{
    if (xSemaphoreTake(g_data_mutex, portMAX_DELAY) == pdTRUE) {
        g_sensor_data.accel_x = ax;
        g_sensor_data.accel_y = ay;
        g_sensor_data.accel_z = az;
        g_sensor_data.accel_magnitude = accel_mag;
        g_sensor_data.temperature = temp;
        g_sensor_data.humidity = humi;
        g_sensor_data.elevator_state = state;
        g_sensor_data.imu_ready = true;
        xSemaphoreGive(g_data_mutex);
    }
}

void shared_data_write_audio(float noise_db)
{
    if (xSemaphoreTake(g_data_mutex, portMAX_DELAY) == pdTRUE) {
        g_sensor_data.noise_db = noise_db;
        g_sensor_data.audio_ready = true;
        xSemaphoreGive(g_data_mutex);
    }
}

sensor_data_t shared_data_read(void)
{
    sensor_data_t snapshot;
    if (xSemaphoreTake(g_data_mutex, portMAX_DELAY) == pdTRUE) {
        snapshot = g_sensor_data; // 结构体整体拷贝，拿到的是一致的快照
        xSemaphoreGive(g_data_mutex);
    }
    return snapshot;
}
