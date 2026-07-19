/*
 * audio_task.c
 * I2S 麦克风(INMP441)采集任务 —— 经过 Slow 时间计权优化的稳定版
 */

#include <math.h>
#include <string.h>
#include "esp_log.h"
#include "driver/i2s_std.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "audio_task.h"
#include "shared_data.h"

static const char *TAG = "AUDIO_TASK";

// ---------- I2S 引脚 ----------
#define I2S_BCK_IO      15
#define I2S_WS_IO       16
#define I2S_DATA_IN_IO  17

#define I2S_SAMPLE_RATE   16000
#define I2S_READ_SAMPLES  512  

// 规格书理论推导的 94dB SPL 对应 RMS
#define REF_AMPLITUDE   297000.0f

// 一阶高通滤波器系数，消除硬件直流偏置
#define HPF_ALPHA        0.995f

static i2s_chan_handle_t s_rx_handle = NULL;

// ================= I2S 初始化 =================
static esp_err_t i2s_mic_init(void)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    esp_err_t err = i2s_new_channel(&chan_cfg, NULL, &s_rx_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2s_new_channel 失败: %d", err);
        return err;
    }

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(I2S_SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_BCK_IO,
            .ws   = I2S_WS_IO,
            .dout = I2S_GPIO_UNUSED,
            .din  = I2S_DATA_IN_IO,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };

    err = i2s_channel_init_std_mode(s_rx_handle, &std_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2s_channel_init_std_mode 失败: %d", err);
        return err;
    }

    return i2s_channel_enable(s_rx_handle);
}

// ================= 任务主体 =================
void audio_task(void *pvParameters)
{
    ESP_LOGI(TAG, "audio_task 启动");

    if (i2s_mic_init() != ESP_OK) {
        ESP_LOGE(TAG, "I2S麦克风初始化失败，任务退出");
        vTaskDelete(NULL);
        return;
    }

    int32_t *buffer = (int32_t *)malloc(I2S_READ_SAMPLES * sizeof(int32_t) * 2);
    if (buffer == NULL) {
        ESP_LOGE(TAG, "音频缓冲区内存分配失败");
        vTaskDelete(NULL);
        return;
    }

    float smoothed_db = 0.0f;
    bool first_reading = true;

    double last_input = 0.0;
    double last_output = 0.0;

    while (1) {
        size_t bytes_read = 0;
        esp_err_t err = i2s_channel_read(s_rx_handle, buffer, I2S_READ_SAMPLES * sizeof(int32_t) * 2,
                                          &bytes_read, portMAX_DELAY);

        if (err == ESP_OK && bytes_read > 0) {
            int total_samples = bytes_read / sizeof(int32_t);

            double sum_sq = 0.0;
            int valid_count = 0;

            for (int i = 0; i < total_samples; i += 2) {
                int32_t sample = buffer[i] >> 8;
                if (sample & 0x00800000) {
                    sample |= 0xFF000000;
                } else {
                    sample &= 0x00FFFFFF;
                }

                double current_input = (double)sample;
                double filtered_output = HPF_ALPHA * (last_output + current_input - last_input);
                last_input = current_input;
                last_output = filtered_output;

                sum_sq += filtered_output * filtered_output;
                valid_count++;
            }

            if (valid_count > 0) {
                double rms = sqrt(sum_sq / valid_count);

                float raw_db = 0.0f;
                if (rms > 0.0) {
                    raw_db = 20.0f * log10f((float)rms / REF_AMPLITUDE) + 94.0f;
                }

                // 统一线性校准：减去硬件电气固定偏置量，使其回归标准的室内环境声压
                float calibrated_db = raw_db - 31.0f;

                if (calibrated_db < 30.0f)  calibrated_db = 30.0f;
                if (calibrated_db > 120.0f) calibrated_db = 120.0f;

                // 强力 Slow 慢速滤波滤波：平滑系数采用 0.1，让分贝的变化非常优雅稳定，消灭毛刺
                if (first_reading) {
                    smoothed_db = calibrated_db;
                    first_reading = false;
                } else {
                    float alpha = 0.10f; 
                    if (calibrated_db - smoothed_db > 12.0f) {
                        alpha = 0.35f; // 仅对极大的突发尖锐爆音快速响应
                    }
                    smoothed_db = alpha * calibrated_db + (1.0f - alpha) * smoothed_db;
                }

                shared_data_write_audio(smoothed_db);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}