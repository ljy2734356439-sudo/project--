/*
 * sensor_task.h / sensor_task.c 的头文件
 * I2C传感器采集任务（IMU + 温湿度 + 状态机）
 */

#ifndef SENSOR_TASK_H
#define SENSOR_TASK_H

// FreeRTOS 任务函数，创建任务时作为入口传入
// 参数 pvParameters 不使用，传 NULL 即可
void sensor_task(void *pvParameters);

#endif // SENSOR_TASK_H
