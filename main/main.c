/*
 * main.c
 * 电梯乘运质量检测仪 —— 多任务主程序入口
 *
 * 任务分配：
 *   sensor_task  : I2C IMU+温湿度采集，优先级最高，绑定Core1（避开WiFi/BT协议栈）
 *   audio_task   : I2S 麦克风采集，优先级中等，绑定Core1
 *   display_task : 屏幕刷新（当前占位用日志），优先级最低，放Core0
 *
 * 为什么这样分核：
 *   ESP32-S3是双核，Core0默认跑WiFi/蓝牙协议栈相关的系统任务，
 *   如果把实时性要求高的传感器采集也放Core0，可能被协议栈任务抢占导致采样抖动。
 *   所以把两个"数据源"任务(sensor/audio)都锁在Core1，
 *   把"数据消费者"(display)放Core0，互不干扰。
 */

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "shared_data.h"
#include "sensor_task.h"
#include "audio_task.h"
#include "display_task.h"
#include "score_task.h"

static const char *TAG = "MAIN";

// 任务优先级（数字越大优先级越高，ESP-IDF默认最高可用到 configMAX_PRIORITIES-1）
#define PRIORITY_SENSOR   5
#define PRIORITY_AUDIO    4
#define PRIORITY_SCORE    3
#define PRIORITY_DISPLAY  2

// 任务栈大小（字节）。IMU任务逻辑简单给4KB足够；
// 音频任务涉及浮点RMS计算给4KB；评分任务涉及指数函数计算给4KB；
// 显示任务未来接入LVGL后需要加大（先给4KB占位，接LVGL时再调）
#define STACK_SENSOR   4096
#define STACK_AUDIO    4096
#define STACK_SCORE    4096
#define STACK_DISPLAY  4096

void app_main(void)
{
    ESP_LOGI(TAG, "=== 电梯乘运质量检测仪 多任务系统启动 ===");

    // 必须先初始化共享数据结构（mutex、queue），再创建任务，否则任务里用到mutex时会是NULL
    shared_data_init();

    xTaskCreatePinnedToCore(sensor_task, "sensor_task", STACK_SENSOR,
                             NULL, PRIORITY_SENSOR, NULL, 1);

    xTaskCreatePinnedToCore(audio_task, "audio_task", STACK_AUDIO,
                             NULL, PRIORITY_AUDIO, NULL, 1);

    // score_task只是定期读快照+做数学计算，不直接碰硬件，放Core0跟display一起，不占Core1的实时性资源
    xTaskCreatePinnedToCore(score_task, "score_task", STACK_SCORE,
                             NULL, PRIORITY_SCORE, NULL, 0);

    xTaskCreatePinnedToCore(display_task, "display_task", STACK_DISPLAY,
                             NULL, PRIORITY_DISPLAY, NULL, 0);

    ESP_LOGI(TAG, "所有任务创建完成");

    // app_main 本身作为一个任务，这里可以直接返回（ESP-IDF允许），
    // 也可以留一个空循环监控系统状态，这里选择让它退出，交给FreeRTOS调度器管理其他任务
}
