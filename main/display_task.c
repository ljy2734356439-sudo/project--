#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_ili9341.h"
#include "esp_lvgl_port.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "lvgl.h"

#include "display_task.h"
#include "shared_data.h"

#define LCD_HOST        SPI2_HOST
#define PIN_NUM_MISO    13
#define PIN_NUM_MOSI    11
#define PIN_NUM_CLK     12
#define PIN_NUM_CS      10
#define PIN_NUM_DC      18
#define PIN_NUM_RST     21

#define LCD_H_RES       240
#define LCD_V_RES       320

// 声明你在 menuconfig 里勾选的内置宋体字库
LV_FONT_DECLARE(lv_font_simsun_16_cjk);

// ---------- UI 控件定义 ----------
static lv_obj_t *s_label_state = NULL;
static lv_obj_t *s_label_accel = NULL;
static lv_obj_t *s_label_env   = NULL;
static lv_obj_t *s_label_noise = NULL;

// 评分区域控件
static lv_obj_t *s_report_container = NULL; 
static lv_obj_t *s_label_report_title = NULL;
static lv_obj_t *s_label_score_total  = NULL;
static lv_obj_t *s_label_score_level  = NULL;
static lv_obj_t *s_label_score_detail = NULL;

// 过滤函数：确保传给标签的一定是纯中文（优/良/中/差）
static const char* translate_to_pure_chinese(const char* c_grade)
{
    if (strstr(c_grade, "优") != NULL || strstr(c_grade, "EXCELLENT") != NULL) return "优";
    if (strstr(c_grade, "良") != NULL || strstr(c_grade, "GOOD") != NULL)      return "良";
    if (strstr(c_grade, "中") != NULL || strstr(c_grade, "FAIR") != NULL)      return "中";
    if (strstr(c_grade, "差") != NULL || strstr(c_grade, "BAD") != NULL)       return "差";
    return c_grade;
}

// 界面控件初始化创建（所有固定标签保持纯英文大字）
static void create_ui(lv_disp_t *disp)
{
    lv_obj_t *scr = lv_disp_get_scr_act(disp);
    lv_obj_set_style_bg_color(scr, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);

    // --- 1. 实时数据区（全英文大字，稳定不乱码） ---
    s_label_state = lv_label_create(scr);
    lv_obj_set_style_text_font(s_label_state, &lv_font_montserrat_22, 0); 
    lv_obj_set_style_text_color(s_label_state, lv_color_make(0, 255, 0), 0);
    lv_obj_align(s_label_state, LV_ALIGN_TOP_LEFT, 15, 15);
    lv_label_set_text(s_label_state, "State: --");

    s_label_accel = lv_label_create(scr);
    lv_obj_set_style_text_font(s_label_accel, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(s_label_accel, lv_color_make(180, 180, 180), 0); 
    lv_obj_align(s_label_accel, LV_ALIGN_TOP_LEFT, 15, 52); 
    lv_label_set_text(s_label_accel, "Accel: --");

    s_label_env = lv_label_create(scr);
    lv_obj_set_style_text_font(s_label_env, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(s_label_env, lv_color_make(180, 180, 180), 0); 
    lv_obj_align(s_label_env, LV_ALIGN_TOP_LEFT, 15, 80);  
    lv_label_set_text(s_label_env, "Env: --");

    s_label_noise = lv_label_create(scr);
    lv_obj_set_style_text_font(s_label_noise, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(s_label_noise, lv_color_make(180, 180, 180), 0); 
    lv_obj_align(s_label_noise, LV_ALIGN_TOP_LEFT, 15, 108); 
    lv_label_set_text(s_label_noise, "Noise: --");

    // --- 2. 报告区域容器卡片 ---
    s_report_container = lv_obj_create(scr);
    lv_obj_set_size(s_report_container, 224, 175);
    lv_obj_align(s_report_container, LV_ALIGN_BOTTOM_MID, 0, -8);
    
    lv_obj_set_style_bg_color(s_report_container, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(s_report_container, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(s_report_container, lv_color_make(140, 140, 140), 0); 
    lv_obj_set_style_border_width(s_report_container, 1, 0);
    lv_obj_set_style_radius(s_report_container, 4, 0);
    
    lv_obj_set_style_pad_all(s_report_container, 10, 0);
    lv_obj_set_scrollbar_mode(s_report_container, LV_SCROLLBAR_MODE_OFF);

    s_label_report_title = lv_label_create(s_report_container);
    lv_obj_set_style_text_font(s_label_report_title, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(s_label_report_title, lv_color_make(255, 255, 0), 0); 
    lv_obj_align(s_label_report_title, LV_ALIGN_TOP_MID, 0, -4);
    lv_label_set_text(s_label_report_title, "=== REPORT ===");

    s_label_score_total = lv_label_create(s_report_container);
    lv_obj_set_style_text_font(s_label_score_total, &lv_font_montserrat_24, 0);
    lv_obj_set_style_text_color(s_label_score_total, lv_color_make(255, 128, 0), 0); 
    lv_obj_align(s_label_score_total, LV_ALIGN_TOP_LEFT, 5, 22);
    lv_label_set_text(s_label_score_total, "Score: --");

    // 【核心关键】等级标签默认初始化为英文大字库
    s_label_score_level = lv_label_create(s_report_container);
    lv_obj_set_style_text_font(s_label_score_level, &lv_font_montserrat_22, 0);
    lv_obj_set_style_text_color(s_label_score_level, lv_color_make(0, 192, 255), 0); 
    lv_obj_align(s_label_score_level, LV_ALIGN_TOP_LEFT, 5, 58);
    lv_label_set_text(s_label_score_level, "Level: --");

    s_label_score_detail = lv_label_create(s_report_container);
    lv_obj_set_style_text_font(s_label_score_detail, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(s_label_score_detail, lv_color_white(), 0);
    lv_obj_align(s_label_score_detail, LV_ALIGN_TOP_LEFT, 5, 98); 
    lv_label_set_text(s_label_score_detail, "Waiting for run...");
}

void display_task(void *pvParameters)
{
    // SPI 及屏幕硬件驱动初始化保持不变
    spi_bus_config_t buscfg = {
        .sclk_io_num = PIN_NUM_CLK,
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_H_RES * 40 * sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = PIN_NUM_DC,
        .cs_gpio_num = PIN_NUM_CS,
        .pclk_hz = 20 * 1000 * 1000,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle));

    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = PIN_NUM_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_ili9341(io_handle, &panel_config, &panel_handle));

    esp_lcd_panel_reset(panel_handle);
    esp_lcd_panel_init(panel_handle);
    esp_lcd_panel_invert_color(panel_handle, false);
    esp_lcd_panel_swap_xy(panel_handle, false);
    esp_lcd_panel_mirror(panel_handle, false, true);
    esp_lcd_panel_disp_on_off(panel_handle, true);

    const lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    ESP_ERROR_CHECK(lvgl_port_init(&lvgl_cfg));

    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = io_handle,
        .panel_handle = panel_handle,
        .buffer_size = LCD_H_RES * 40,
        .double_buffer = true,
        .hres = LCD_H_RES,
        .vres = LCD_V_RES,
        .monochrome = false,
    };
    lv_disp_t *disp = lvgl_port_add_disp(&disp_cfg);

    if (lvgl_port_lock(0)) {
        create_ui(disp);
        lvgl_port_unlock();
    }

    while (1) {
        sensor_data_t data = shared_data_read();

        if (lvgl_port_lock(0)) {
            // === 1. 动态刷新：实时传感器数值 ===
            lv_label_set_text_fmt(s_label_state, "State: %s",
                                    data.elevator_state == ES_STATE_RUNNING ? "RUNNING" : "IDLE");

            lv_label_set_text_fmt(s_label_accel, "Acc: %.2f, %.2f, %.2f",
                                    data.accel_x, data.accel_y, data.accel_z);

            lv_label_set_text_fmt(s_label_env, "T: %.1f C  H: %.1f %%",
                                    data.temperature, data.humidity);

            lv_label_set_text_fmt(s_label_noise, "Noise: %.1f dB",
                                    data.audio_ready ? data.noise_db : 0.0f);

            // === 2. 动态刷新：下方的卡片评分数据 ===
            if (data.elevator_state == ES_STATE_RUNNING) {
                lv_obj_add_flag(s_report_container, LV_OBJ_FLAG_HIDDEN);
            } 
            else {
                lv_obj_clear_flag(s_report_container, LV_OBJ_FLAG_HIDDEN);
                
                if (data.has_score_report) {
                    // 获取过滤后的纯中文汉字 ("优"/"良"/"中"/"差")
                    const char* chinese_grade = translate_to_pure_chinese(data.score_level);

                    lv_label_set_text_fmt(s_label_score_total, "Score: %.1f", data.total_score);
                    
                    // 【终极方案】动态将字库临时切换到内置 CJK 字库，这样“Level:”和汉字都可以共存显示！
                    lv_obj_set_style_text_font(s_label_score_level, &lv_font_simsun_16_cjk, 0);
                    lv_label_set_text_fmt(s_label_score_level, "Level: %s", chinese_grade);
                                            
                    lv_label_set_text_fmt(s_label_score_detail, "Vibe Score: %.1f\nNoise Score: %.1f", 
                                            data.score_vibe, data.score_noise);
                } else {
                    // 等待状态时切回大英文字体
                    lv_obj_set_style_text_font(s_label_score_level, &lv_font_montserrat_22, 0);
                    lv_label_set_text(s_label_score_total, "Score: Wait...");
                    lv_label_set_text(s_label_score_level, "Level: Wait...");
                    lv_label_set_text(s_label_score_detail, "Waiting for run...");
                }
            }

            lvgl_port_unlock();
        }

        vTaskDelay(pdMS_TO_TICKS(300));
    }
}