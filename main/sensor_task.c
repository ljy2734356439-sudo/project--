/*
 * sensor_task.c
 * I2C 传感器采集任务：LSM6DS3TR-C (加速度) + SHT30 (温湿度) + 状态机
 * —— 适配 ESP-IDF v5.5.4 新版 I2C 驱动 (i2c_master.h)
 *
 * 关于新驱动API的说明：
 *
 *   新版: i2c_new_master_bus() 创建总线 + i2c_master_bus_add_device() 给每个器件建立"设备句柄"
 *         之后每个器件用自己的句柄读写，不用每次都传地址，更不容易出错。
 *
 * 这个任务独占I2C总线，104Hz采样率对应约10ms周期，
 * 是整个系统里实时性要求最高的任务，优先级设最高、绑定Core1。
 */

#include <math.h>
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "sensor_task.h"
#include "shared_data.h"
#include "elevator_state_machine.h"

static const char *TAG = "SENSOR_TASK";

// ---------- I2C 参数 ----------
#define I2C_MASTER_SDA_IO        4
#define I2C_MASTER_SCL_IO        5
#define I2C_MASTER_NUM           I2C_NUM_0
#define I2C_MASTER_FREQ_HZ       400000

// ---------- LSM6DS3TR-C ----------
#define LSM6DS3_ADDR             0x6A
#define LSM6DS3_WHO_AM_I         0x0F
#define LSM6DS3_CTRL1_XL         0x10
#define LSM6DS3_OUTX_L_XL        0x28
#define ACCEL_SENSITIVITY_2G     (0.061f * 9.80665f / 1000.0f)

// ---------- SHT30 ----------
#define SHT30_ADDR                0x44
#define SHT30_CMD_MEASURE_HIGH_H  0x24
#define SHT30_CMD_MEASURE_HIGH_L  0x00

// ---------- 设备句柄（新API核心：每个I2C器件对应一个句柄） ----------
static i2c_master_bus_handle_t s_bus_handle = NULL;
static i2c_master_dev_handle_t s_lsm6ds3_handle = NULL;
static i2c_master_dev_handle_t s_sht30_handle = NULL;

// ================= I2C 总线与设备初始化 =================
static esp_err_t i2c_bus_init(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true, // 内部上拉兜底，模块自带上拉的话也不冲突
    };
    esp_err_t err = i2c_new_master_bus(&bus_cfg, &s_bus_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_new_master_bus 失败: %d", err);
        return err;
    }

    // 挂载 LSM6DS3TR-C 设备
    i2c_device_config_t lsm6ds3_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = LSM6DS3_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    err = i2c_master_bus_add_device(s_bus_handle, &lsm6ds3_cfg, &s_lsm6ds3_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "挂载 LSM6DS3TR-C 设备失败: %d", err);
        return err;
    }

    // 挂载 SHT30 设备（同一条总线，不同地址，互不干扰）
    i2c_device_config_t sht30_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SHT30_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    err = i2c_master_bus_add_device(s_bus_handle, &sht30_cfg, &s_sht30_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "挂载 SHT30 设备失败: %d", err);
        return err;
    }

    return ESP_OK;
}

// ================= LSM6DS3TR-C 读写封装 =================
static esp_err_t lsm6ds3_write_reg(uint8_t reg, uint8_t data)
{
    uint8_t buf[2] = { reg, data };
    return i2c_master_transmit(s_lsm6ds3_handle, buf, sizeof(buf), -1); // -1表示用默认超时
}

static esp_err_t lsm6ds3_read_regs(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(s_lsm6ds3_handle, &reg, 1, data, len, -1);
}

static esp_err_t lsm6ds3_init(void)
{
    uint8_t who_am_i = 0;
    esp_err_t err = lsm6ds3_read_regs(LSM6DS3_WHO_AM_I, &who_am_i, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "LSM6DS3TR-C 未响应，请检查接线");
        return err;
    }
    ESP_LOGI(TAG, "LSM6DS3TR-C WHO_AM_I = 0x%02X", who_am_i);

    // 104Hz, ±2g
    return lsm6ds3_write_reg(LSM6DS3_CTRL1_XL, 0x40);
}

// ================= IIR 低通滤波器（一阶指数移动平均） =================
#define IIR_ALPHA  0.4f  // 滤波系数：0~1，越小滤波效果越强但延迟越大

typedef struct {
    float filtered_x, filtered_y, filtered_z;
    bool initialized;
} iir_filter_t;

static iir_filter_t s_iir_filter = {0.0f, 0.0f, 0.0f, false};

static void iir_filter_update(float raw_x, float raw_y, float raw_z,
                              float *out_x, float *out_y, float *out_z)
{
    if (!s_iir_filter.initialized) {
        s_iir_filter.filtered_x = raw_x;
        s_iir_filter.filtered_y = raw_y;
        s_iir_filter.filtered_z = raw_z;
        s_iir_filter.initialized = true;
    } else {
        s_iir_filter.filtered_x = IIR_ALPHA * raw_x + (1.0f - IIR_ALPHA) * s_iir_filter.filtered_x;
        s_iir_filter.filtered_y = IIR_ALPHA * raw_y + (1.0f - IIR_ALPHA) * s_iir_filter.filtered_y;
        s_iir_filter.filtered_z = IIR_ALPHA * raw_z + (1.0f - IIR_ALPHA) * s_iir_filter.filtered_z;
    }
    
    *out_x = s_iir_filter.filtered_x;
    *out_y = s_iir_filter.filtered_y;
    *out_z = s_iir_filter.filtered_z;
}

// ================= 3轴合成加速度计算 =================
static float compute_resultant_accel(float ax, float ay, float az)
{
    return sqrtf(ax * ax + ay * ay + az * az);
}

static esp_err_t lsm6ds3_read_accel(float *ax, float *ay, float *az)
{
    uint8_t raw[6];
    esp_err_t err = lsm6ds3_read_regs(LSM6DS3_OUTX_L_XL, raw, 6);
    if (err != ESP_OK) return err;

    int16_t raw_x = (int16_t)((raw[1] << 8) | raw[0]);
    int16_t raw_y = (int16_t)((raw[3] << 8) | raw[2]);
    int16_t raw_z = (int16_t)((raw[5] << 8) | raw[4]);

    float raw_ax = raw_x * ACCEL_SENSITIVITY_2G;
    float raw_ay = raw_y * ACCEL_SENSITIVITY_2G;
    float raw_az = raw_z * ACCEL_SENSITIVITY_2G;
    
    // 应用IIR低通滤波
    iir_filter_update(raw_ax, raw_ay, raw_az, ax, ay, az);
    
    return ESP_OK;
}

// ================= SHT30 读写封装 =================
static esp_err_t sht30_read(float *temperature, float *humidity)
{
    uint8_t cmd[2] = { SHT30_CMD_MEASURE_HIGH_H, SHT30_CMD_MEASURE_HIGH_L };
    esp_err_t err = i2c_master_transmit(s_sht30_handle, cmd, sizeof(cmd), -1);
    if (err != ESP_OK) return err;

    vTaskDelay(pdMS_TO_TICKS(15)); // 等待SHT30内部转换完成

    uint8_t raw[6];
    err = i2c_master_receive(s_sht30_handle, raw, sizeof(raw), -1);
    if (err != ESP_OK) return err;

    uint16_t raw_temp = (raw[0] << 8) | raw[1];
    uint16_t raw_humi = (raw[3] << 8) | raw[4];
    // CRC校验(raw[2], raw[5])这里先省略，稳定后可以加

    *temperature = -45.0f + 175.0f * ((float)raw_temp / 65535.0f);
    *humidity = 100.0f * ((float)raw_humi / 65535.0f);
    return ESP_OK;
}

// ================= 任务主体 =================
void sensor_task(void *pvParameters)
{
    ESP_LOGI(TAG, "sensor_task 启动");

    if (i2c_bus_init() != ESP_OK) {
        ESP_LOGE(TAG, "I2C总线/设备初始化失败，任务仍继续运行以便调试");
    }
    if (lsm6ds3_init() != ESP_OK) {
        ESP_LOGE(TAG, "IMU初始化失败，任务仍继续运行以便调试");
    }

    elevator_sm_t sm;
    es_init(&sm);

    int sht30_counter = 0;
    float last_temp = 25.0f, last_humi = 50.0f;

    while (1) {
        float ax, ay, az;
        if (lsm6ds3_read_accel(&ax, &ay, &az) == ESP_OK) {

            bool changed = es_update(&sm, ax, ay, az);

            // 计算3轴合成加速度（用于冲击强度评估）
            float accel_magnitude = compute_resultant_accel(ax, ay, az);

            // 每20个周期（约200ms）读一次温湿度
            if (++sht30_counter >= 20) {
                sht30_counter = 0;
                float t, h;
                if (sht30_read(&t, &h) == ESP_OK) {
                    last_temp = t;
                    last_humi = h;
                } else {
                    ESP_LOGW(TAG, "SHT30 读取失败，沿用上一次的值");
                }
            }

            shared_data_write_imu(ax, ay, az, accel_magnitude, last_temp, last_humi, es_get_state(&sm));

            if (changed) {
                elevator_event_t evt = (es_get_state(&sm) == ES_STATE_RUNNING)
                                        ? EVT_ELEVATOR_START : EVT_ELEVATOR_STOP;
                xQueueSend(g_state_event_queue, &evt, 0);
                ESP_LOGW(TAG, "状态切换: %s", evt == EVT_ELEVATOR_START ? "开始运行" : "运行结束");
            }
        } else {
            ESP_LOGE(TAG, "IMU读取失败");
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // 约100Hz
    }
}
