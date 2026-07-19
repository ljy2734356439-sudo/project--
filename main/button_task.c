/*
 * button_task.c
 *
 * 硬件说明：
 *   两个按键都是"一端接GPIO，一端接GND"的简单接法，配置GPIO内部上拉电阻，
 *   按键松开时GPIO读到高电平(1)，按下时短接到GND读到低电平(0)，即"按下=低电平有效"。
 *
 *   BTN1 (GPIO6)：手动开始/停止检测
 *   BTN2 (GPIO7)：短按=手动重新标定基准；长按(>=800ms)=切换屏幕显示页面
 *
 * 去抖动说明：
 *   机械按键在按下/松开的瞬间，电平会有几毫秒的抖动(不是干净的一次性跳变，
 *   而是快速地在高低电平之间弹跳几次)，如果不处理，一次按下可能被误判成
 *   好几次按键事件。这里用最简单的"延时确认法"：检测到电平变化后，
 *   延时20ms再读一次确认电平真的稳定了，才算一次有效按键。
 *
 * 长按检测说明：
 *   BTN2需要区分"短按"和"长按"两种操作，做法是：检测到按下(下降沿)后，
 *   不立刻上报事件，而是持续轮询直到松开(上升沿)，用两次时刻的差值算出
 *   按住了多久，超过LONG_PRESS_MS就判定为长按，否则算短按。
 */

#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "button_task.h"
#include "shared_data.h"

static const char *TAG = "BUTTON_TASK";

#define BTN1_START_STOP_GPIO  6
#define BTN2_RECALIBRATE_GPIO 7

#define DEBOUNCE_DELAY_MS  20   // 去抖动确认延时
#define POLL_INTERVAL_MS   20   // 轮询周期，按键操作不需要跟传感器一样高频
#define LONG_PRESS_MS      800  // 按住超过这个时长算长按

static void button_gpio_init(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << BTN1_START_STOP_GPIO) | (1ULL << BTN2_RECALIBRATE_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,   // 内部上拉，按键松开时默认高电平
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,     // 用轮询而不是中断，逻辑更简单，按键场景对延迟不敏感
    };
    gpio_config(&cfg);
}

// 检测单个按键是否发生了一次"有效按下"（带去抖动），只区分"有没有按下瞬间"，不管时长
static bool check_button_pressed(int gpio_num, int *last_level)
{
    int current_level = gpio_get_level(gpio_num);

    if (current_level != *last_level) {
        vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_DELAY_MS));
        int confirmed_level = gpio_get_level(gpio_num);

        if (confirmed_level == current_level) {
            bool is_press_edge = (*last_level == 1 && confirmed_level == 0);
            *last_level = confirmed_level;
            return is_press_edge;
        }
    }
    return false;
}

// BTN2专用：检测到按下后，阻塞等待松开，返回本次按住的时长(ms)
// 用阻塞方式是可以接受的，因为按键任务本身逻辑简单，就算这里等一下也不影响其他任务
static uint32_t wait_for_release_and_get_duration(int gpio_num)
{
    TickType_t press_tick = xTaskGetTickCount();

    // 持续轮询直到松开(高电平)，同样做一次去抖确认
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(POLL_INTERVAL_MS));
        if (gpio_get_level(gpio_num) == 1) {
            vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_DELAY_MS));
            if (gpio_get_level(gpio_num) == 1) {
                break; // 确认真的松开了
            }
        }
    }

    TickType_t release_tick = xTaskGetTickCount();
    return (uint32_t)((release_tick - press_tick) * portTICK_PERIOD_MS);
}

void button_task(void *pvParameters)
{
    ESP_LOGI(TAG, "button_task 启动");
    button_gpio_init();

    int btn1_last_level = 1;
    int btn2_last_level = 1;

    while (1) {
        if (check_button_pressed(BTN1_START_STOP_GPIO, &btn1_last_level)) {
            ESP_LOGW(TAG, "按键1按下：手动切换开始/停止检测");
            button_event_t evt = BTN_EVT_TOGGLE_START_STOP;
            xQueueSend(g_button_event_queue, &evt, 0);
        }

        if (check_button_pressed(BTN2_RECALIBRATE_GPIO, &btn2_last_level)) {
            // 检测到按下瞬间后，接着等它松开，量出这次按住了多久
            uint32_t duration_ms = wait_for_release_and_get_duration(BTN2_RECALIBRATE_GPIO);
            btn2_last_level = 1; // 上面的等待循环已经确认松开了，同步更新电平记录

            if (duration_ms >= LONG_PRESS_MS) {
                ESP_LOGW(TAG, "按键2长按(%lums)：切换显示页面", (unsigned long)duration_ms);
                shared_data_toggle_page();
            } else {
                ESP_LOGW(TAG, "按键2短按(%lums)：手动触发重新标定基准", (unsigned long)duration_ms);
                button_event_t evt = BTN_EVT_RECALIBRATE;
                xQueueSend(g_button_event_queue, &evt, 0);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(POLL_INTERVAL_MS));
    }
}
