#include "esp_log.h"
#include <stdlib.h>
#include "nixie_ctr.h"
// I2C相关头文件
#include "driver/i2c.h"
#include <stdio.h>

#include "esp_sleep.h"
#include "driver/rtc_io.h"
#include "soc/rtc.h"
#include "esp_wifi.h"
#include "esp_pm.h"

// 添加NVS相关头文件
#include "nvs_flash.h"
#include "nvs.h"

// 添加WiFi管理器头文件（用于恢复出厂设置）
#include "wifi_manager.h"

static const char *IOTAG = "IO";
static const char *RTCTAG = "RTC_Example";
static const char *BUTTONTAG = "BUTTON";

// 按键事件类型定义
typedef enum {
    BUTTON_EVENT_NONE = 0,
    BUTTON_EVENT_SINGLE_CLICK,
    BUTTON_EVENT_DOUBLE_CLICK,
    BUTTON_EVENT_LONG_PRESS,
    BUTTON_EVENT_FACTORY_RESET_LONG_PRESS
} button_event_t;

// 按键状态和时间参数
#define DEBOUNCE_TIME_MS        50      // 消抖时间
#define LONG_PRESS_TIME_MS      3000    // 长按时间阈值
#define DOUBLE_CLICK_TIME_MS    300     // 双击时间窗口
#define FACTORY_RESET_LONG_PRESS_MS  15000  // 恢复出厂设置长按时间阈值（15秒）

// 按键相关变量
static TaskHandle_t button_task_handle = NULL;
static int64_t last_press_time = 0;
static int64_t last_release_time = 0;
static int click_count = 0;
static bool button_pressed = false;
static esp_timer_handle_t button_timer = NULL;

// AuxKey按键相关变量
static TaskHandle_t auxkey_task_handle = NULL;
static int64_t auxkey_last_press_time = 0;
static int64_t auxkey_last_release_time = 0;
static int auxkey_click_count = 0;
static bool auxkey_pressed = false;
static esp_timer_handle_t auxkey_timer = NULL;

adc_oneshot_unit_handle_t adc1_handle;

// Deep Sleep相关变量
static esp_timer_handle_t deep_sleep_timer = NULL;
// static httpd_handle_t current_server_handle = NULL;
static const char *SLEEPTAG = "POWER_MGMT";

// 总运行时间相关变量和常量
static const char *NVSTAG = "NVS_RUNTIME";
static const char *NVS_NAMESPACE = "nixie_clock";
static const char *NVS_RUNTIME_KEY = "total_runtime";
static const char *NVS_RTC_TIME_KEY __attribute__((unused)) = "last_rtc_time";
static nvs_handle_t runtime_nvs_handle = 0;

// 总运行时间相关变量
static uint32_t total_runtime_minutes = 0;  // 总运行时间（分钟）
static time_t last_saved_rtc_time = 0;      // 最后保存的RTC时间
static bool hv_enabled = false;             // 高压开启状态
static int64_t hv_start_time = 0;           // 高压开启时间戳
// static esp_timer_handle_t runtime_save_timer = NULL;  // 保存定时器
//static const uint64_t RUNTIME_SAVE_INTERVAL = 60 * 1000000; // 1分钟保存一次（微秒）

// 唤醒模式枚举（简化版）
typedef enum {
    WAKEUP_MODE_SECOND = 0,    // 秒跳变唤醒（保持所有服务）
    WAKEUP_MODE_MINUTE = 1     // 分跳变唤醒（关闭网络服务）
} wakeup_mode_t;

// 当前唤醒模式（默认为秒跳变模式，保持网络服务稳定）
static wakeup_mode_t current_wakeup_mode = WAKEUP_MODE_SECOND;

// 恢复出厂设置标志位
static bool factory_reset_pending = false;

// 显示配置全局变量
static char current_display_type[64] = "time";  // 默认显示时间
static char current_custom_content[64] = "";     // 自定义内容
static char current_refresh_rate[16] = "1";      // 默认刷新率1Hz

// 模式名称
static const char* wakeup_mode_names[] = {
    "秒跳变（保持所有服务）",
    "分跳变（关闭网络服务）"
};

// 静态缓冲区用于str2BitPos，避免频繁malloc/free
static uint8_t bit_pos_buffer[4][4];
static uint8_t* bit_pos_rows[4] = {
    bit_pos_buffer[0],
    bit_pos_buffer[1],
    bit_pos_buffer[2],
    bit_pos_buffer[3]
};

// 按键事件回调函数声明
static void button_single_click_callback(void);
static void button_double_click_callback(void);
static void button_long_press_callback(void);
static void button_isr_handler(void* arg);
static void button_task(void* arg);

// AuxKey按键事件回调函数声明
static void auxkey_double_click_callback(void);
static void auxkey_factory_reset_long_press_callback(void);
static void auxkey_isr_handler(void* arg);
static void auxkey_task(void* arg);

// 电源管理和电压保护函数声明
static bool check_battery_voltage_protection(void) __attribute__((unused));

// 总运行时间管理函数声明
static esp_err_t init_runtime_nvs(void);
void load_refresh_rate_from_nvs(void);
static esp_err_t save_runtime_to_nvs(void);
// static esp_err_t save_runtime_only_to_nvs(void);
// static esp_err_t save_rtc_time_only_to_nvs(void);
static void start_hv_runtime_counter(void);
static void stop_hv_runtime_counter(void);
static void update_runtime_counter(void);

// 恢复出厂设置函数声明
esp_err_t perform_factory_reset(void);
static void clear_factory_reset_flag(void);
// I2C配置参数
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_SCL_IO           0      // GPIO0
#define I2C_MASTER_SDA_IO           1      // GPIO1
#define I2C_MASTER_FREQ_HZ          100000 // 100kHz
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0
// BQ27220相关定义已移动到nixie_ctr.h

// 电池、BMP580和AHT20传感器相关变量
static float last_temperature_bmp = 0.0;
static float last_pressure = 0.0;
static float last_temperature_aht = 0.0;
static float last_humidity = 0.0;
static float last_battery_voltage = 0.0;
static float last_battery_percentage = 0.0;
static float last_battery_current = 0.0;


// 初始化I2C主机
void i2c_master_init(void)
{
    ESP_LOGI(IOTAG, "初始化I2C主机模式...");
    ESP_LOGI(IOTAG, "I2C配置: SCL=GPIO%d, SDA=GPIO%d, 频率=%dHz", I2C_MASTER_SCL_IO, I2C_MASTER_SDA_IO, I2C_MASTER_FREQ_HZ);
    
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,  // 禁用内部上拉，需要外部上拉电阻
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_DISABLE,  // 禁用内部上拉，需要外部上拉电阻
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    
    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(IOTAG, "I2C参数配置失败: %s", esp_err_to_name(ret));
        return;
    }
    
    ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(IOTAG, "I2C驱动安装失败: %s", esp_err_to_name(ret));
        return;
    }
    
    ESP_LOGI(IOTAG, "I2C主机初始化成功");
}

// 检查BQ27220电池是否存在
esp_err_t bq27220_check_presence(void)
{
    ESP_LOGI(IOTAG, "检查BQ27220YZFR电池芯片是否存在...");
    esp_err_t ret;
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (cmd == NULL) {
        ESP_LOGE(IOTAG, "I2C命令链创建失败");
        return ESP_ERR_NO_MEM;
    }
    
    // 尝试向BQ27220发送地址，检查ACK响应
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BQ27220_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);
    
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    if (ret == ESP_OK) {
        ESP_LOGI(IOTAG, "BQ27220电池芯片检测成功");
    } else {
        ESP_LOGW(IOTAG, "BQ27220电池芯片未检测到: %s", esp_err_to_name(ret));
    }
    
    return ret;
}

// 获取BQ27220电池完整状态信息
esp_err_t bq27220_get_battery_info(uint8_t *soc, uint16_t *voltage, int16_t *current)
{
    ESP_LOGI(IOTAG, "读取BQ27220电池完整状态信息...");
    
    esp_err_t ret;
    
    // 读取SOC（电量百分比）- 寄存器0x2C
    uint8_t soc_reg = 0x2C;
    uint8_t soc_data[2] = {0};
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (cmd == NULL) {
        ESP_LOGE(IOTAG, "I2C命令链创建失败");
        return ESP_ERR_NO_MEM;
    }
    
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BQ27220_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, soc_reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BQ27220_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, soc_data, 2, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(IOTAG, "读取SOC失败: %s", esp_err_to_name(ret));
        *soc = 0;
        *voltage = 0;
        *current = 0;
        return ret;
    }
    
    *soc = soc_data[0];
    
    // 读取电压 - 寄存器0x08
    uint8_t volt_reg = 0x08;
    uint8_t volt_data[2] = {0};
    
    cmd = i2c_cmd_link_create();
    if (cmd == NULL) {
        ESP_LOGE(IOTAG, "I2C命令链创建失败");
        return ESP_ERR_NO_MEM;
    }
    
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BQ27220_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, volt_reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BQ27220_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, volt_data, 2, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(IOTAG, "读取电压失败: %s", esp_err_to_name(ret));
        *voltage = 0;
    } else {
        *voltage = (volt_data[1] << 8) | volt_data[0]; // 小端序
    }
    
    // 读取电流 - 寄存器0x0C
    uint8_t curr_reg = 0x0C;
    uint8_t curr_data[2] = {0};
    
    cmd = i2c_cmd_link_create();
    if (cmd == NULL) {
        ESP_LOGE(IOTAG, "I2C命令链创建失败");
        return ESP_ERR_NO_MEM;
    }
    
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BQ27220_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, curr_reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BQ27220_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, curr_data, 2, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(IOTAG, "读取电流失败: %s", esp_err_to_name(ret));
        *current = 0;
    } else {
        *current = (int16_t)((curr_data[1] << 8) | curr_data[0]); // 小端序，有符号
    }
    
    ESP_LOGI(IOTAG, "电池状态 - 电量: %d%%, 电压: %dmV, 电流: %dmA", *soc, *voltage, *current);
    
    return ESP_OK;
}

// BMP580气压计初始化
esp_err_t bmp580_init(void)
{
    ESP_LOGI(IOTAG, "初始化BMP580气压计...");
    esp_err_t ret;
    
    // 软复位
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (cmd == NULL) {
        ESP_LOGE(IOTAG, "BMP580: I2C命令链创建失败");
        return ESP_ERR_NO_MEM;
    }
    
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMP580_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x7E, true);  // 软复位寄存器
    i2c_master_write_byte(cmd, 0xB6, true);  // 软复位命令
    i2c_master_stop(cmd);
    
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(IOTAG, "BMP580: 软复位失败: %s", esp_err_to_name(ret));
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(10)); // 等待复位完成
    
    // 检查芯片ID
    uint8_t chip_id = 0;
    cmd = i2c_cmd_link_create();
    if (cmd == NULL) {
        ESP_LOGE(IOTAG, "BMP580: I2C命令链创建失败");
        return ESP_ERR_NO_MEM;
    }
    
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMP580_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, BMP580_CHIP_ID_REG, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMP580_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &chip_id, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(IOTAG, "BMP580: 读取芯片ID失败: %s", esp_err_to_name(ret));
        return ret;
    }
    
    if (chip_id != BMP580_CHIP_ID) {
        ESP_LOGE(IOTAG, "BMP580: 芯片ID不匹配，期望0x%02X，实际0x%02X", BMP580_CHIP_ID, chip_id);
        return ESP_ERR_NOT_FOUND;
    }
    
    ESP_LOGI(IOTAG, "BMP580: 芯片ID验证成功 (0x%02X)", chip_id);
    
    // 配置电源控制寄存器 - 启用温度和气压测量
    cmd = i2c_cmd_link_create();
    if (cmd == NULL) {
        ESP_LOGE(IOTAG, "BMP580: I2C命令链创建失败");
        return ESP_ERR_NO_MEM;
    }
    
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMP580_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, BMP580_PWR_CTRL_REG, true);
    i2c_master_write_byte(cmd, 0x30, true); // 启用温度和气压测量，睡眠模式
    i2c_master_stop(cmd);
    
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(IOTAG, "BMP580: 配置电源控制失败: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 配置过采样率 - 温度和气压都使用x2过采样
    cmd = i2c_cmd_link_create();
    if (cmd == NULL) {
        ESP_LOGE(IOTAG, "BMP580: I2C命令链创建失败");
        return ESP_ERR_NO_MEM;
    }
    
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMP580_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, BMP580_OSR_CONFIG_REG, true);
    i2c_master_write_byte(cmd, 0x11, true); // 温度x2，气压x2过采样
    i2c_master_stop(cmd);
    
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(IOTAG, "BMP580: 配置过采样率失败: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 配置输出数据率 - 25Hz
    cmd = i2c_cmd_link_create();
    if (cmd == NULL) {
        ESP_LOGE(IOTAG, "BMP580: I2C命令链创建失败");
        return ESP_ERR_NO_MEM;
    }
    
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMP580_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, BMP580_ODR_CONFIG_REG, true);
    i2c_master_write_byte(cmd, 0x04, true); // 25Hz输出数据率
    i2c_master_stop(cmd);
    
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(IOTAG, "BMP580: 配置输出数据率失败: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(IOTAG, "BMP580气压计初始化成功");
    return ESP_OK;
}

// BMP580读取温度和气压
esp_err_t bmp580_read_temperature_pressure(float *temperature, float *pressure)
{
    esp_err_t ret;
    
    // 触发强制测量模式
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (cmd == NULL) {
        ESP_LOGE(IOTAG, "BMP580: I2C命令链创建失败");
        return ESP_ERR_NO_MEM;
    }
    
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMP580_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, BMP580_PWR_CTRL_REG, true);
    i2c_master_write_byte(cmd, 0x60, true);  // 启用温度和气压测量 (OSR配置)
    i2c_master_stop(cmd);
    
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(IOTAG, "BMP580: OSR配置失败: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 配置ODR寄存器启动测量
    cmd = i2c_cmd_link_create();
    if (cmd == NULL) {
        ESP_LOGE(IOTAG, "BMP580: I2C命令链创建失败");
        return ESP_ERR_NO_MEM;
    }
    
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMP580_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, BMP580_ODR_CONFIG_REG, true);
    i2c_master_write_byte(cmd, 0x02, true);  // 设置为正常模式，ODR=25Hz
    i2c_master_stop(cmd);
    
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(IOTAG, "BMP580: ODR配置失败: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 等待测量完成，增加等待时间
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // 多次检查数据是否准备好
    uint8_t status = 0;
    int retry_count = 0;
    const int max_retries = 10;
    
    do {
        cmd = i2c_cmd_link_create();
        if (cmd == NULL) {
            ESP_LOGE(IOTAG, "BMP580: I2C命令链创建失败");
            return ESP_ERR_NO_MEM;
        }
        
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (BMP580_ADDR << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, BMP580_STATUS_REG, true);
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (BMP580_ADDR << 1) | I2C_MASTER_READ, true);
        i2c_master_read_byte(cmd, &status, I2C_MASTER_LAST_NACK);
        i2c_master_stop(cmd);
        
        ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
        i2c_cmd_link_delete(cmd);
        
        if (ret != ESP_OK) {
            ESP_LOGE(IOTAG, "BMP580: 读取状态寄存器失败: %s", esp_err_to_name(ret));
            return ret;
        }
        
        // 检查核心是否准备好 (bit 0: core_rdy)
        if (status & 0x01) {
            break;  // 核心准备好了，可以读取数据
        }
        
        ESP_LOGD(IOTAG, "BMP580: 等待数据准备，状态: 0x%02X，重试: %d", status, retry_count);
        vTaskDelay(pdMS_TO_TICKS(10));
        retry_count++;
        
    } while (retry_count < max_retries);
    
    if (retry_count >= max_retries) {
        ESP_LOGW(IOTAG, "BMP580: 数据未准备好，最终状态: 0x%02X", status);
        return ESP_ERR_NOT_FINISHED;
    }
    
    uint8_t temp_data[3] = {0};
    uint8_t press_data[3] = {0};
    
    // 读取温度数据
    cmd = i2c_cmd_link_create();
    if (cmd == NULL) {
        ESP_LOGE(IOTAG, "BMP580: I2C命令链创建失败");
        return ESP_ERR_NO_MEM;
    }
    
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMP580_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, BMP580_TEMP_DATA_REG, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMP580_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, temp_data, 3, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(IOTAG, "BMP580: 读取温度数据失败: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 读取气压数据
    cmd = i2c_cmd_link_create();
    if (cmd == NULL) {
        ESP_LOGE(IOTAG, "BMP580: I2C命令链创建失败");
        return ESP_ERR_NO_MEM;
    }
    
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMP580_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, BMP580_PRESS_DATA_REG, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMP580_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, press_data, 3, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(IOTAG, "BMP580: 读取气压数据失败: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 转换温度数据 (24位，有符号)
    int32_t temp_raw = (temp_data[2] << 16) | (temp_data[1] << 8) | temp_data[0];
    if (temp_raw & 0x800000) {
        temp_raw |= 0xFF000000; // 符号扩展
    }
    *temperature = (float)temp_raw / 65536.0f; // BMP580温度分辨率
    
    // 转换气压数据 (24位，无符号)
    uint32_t press_raw = (press_data[2] << 16) | (press_data[1] << 8) | press_data[0];
    *pressure = (float)press_raw / 64.0f; // BMP580气压分辨率，单位Pa
    *pressure = *pressure / 100.0f; // 转换为hPa
    
    last_temperature_bmp = *temperature;
    last_pressure = *pressure;
    
    return ESP_OK;
}

// AHT20实际工作地址
uint8_t aht20_working_addr = AHT20_ADDR;

// I2C扫描函数
void i2c_scan(void)
{
    ESP_LOGI(IOTAG, "开始I2C扫描...");
    
    for (uint8_t addr = 1; addr < 127; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        
        esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(50));
        i2c_cmd_link_delete(cmd);
        
        if (ret == ESP_OK) {
            ESP_LOGI(IOTAG, "发现I2C设备地址: 0x%02X", addr);
        }
    }
    
    ESP_LOGI(IOTAG, "I2C扫描完成");
}

// AHT20温湿度传感器初始化
esp_err_t aht20_init(void)
{
    ESP_LOGI(IOTAG, "初始化AHT20温湿度传感器...");
    
    // 根据官方文档，AHT20不需要复杂的初始化过程
    // 只需要在读取时按照特定的I2C操作顺序即可
    
    // 先进行I2C扫描，查找可能的AHT20设备
    ESP_LOGI(IOTAG, "开始I2C扫描...");
    i2c_scan();
    ESP_LOGI(IOTAG, "I2C扫描完成");
    
    // 尝试AHT20标准地址
    uint8_t possible_addresses[] = {0x38, 0x39};
    size_t num_addresses = sizeof(possible_addresses) / sizeof(possible_addresses[0]);
    
    for (size_t i = 0; i < num_addresses; i++) {
        uint8_t test_addr = possible_addresses[i];
        ESP_LOGI(IOTAG, "尝试AHT20地址: 0x%02X", test_addr);
        
        // 测试设备是否响应
        i2c_cmd_handle_t test_cmd = i2c_cmd_link_create();
        if (test_cmd == NULL) {
            ESP_LOGE(IOTAG, "AHT20: 测试命令链创建失败");
            continue;
        }
        
        i2c_master_start(test_cmd);
        i2c_master_write_byte(test_cmd, (test_addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(test_cmd);
        esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, test_cmd, pdMS_TO_TICKS(100));
        i2c_cmd_link_delete(test_cmd);
        
        if (ret == ESP_OK) {
            ESP_LOGI(IOTAG, "在地址0x%02X找到AHT20设备", test_addr);
            aht20_working_addr = test_addr;
            ESP_LOGI(IOTAG, "AHT20温湿度传感器初始化成功，地址: 0x%02X", test_addr);
            return ESP_OK;
        } else {
            ESP_LOGW(IOTAG, "地址0x%02X无响应: %s", test_addr, esp_err_to_name(ret));
        }
    }
    
    aht20_working_addr = 0;  // 标记初始化失败
    return ESP_ERR_NOT_FOUND;
}

// AHT20读取温度和湿度
esp_err_t aht20_read_temperature_humidity(float *temperature, float *humidity)
{
    // 检查AHT20是否已正确初始化（通过检查工作地址是否有效）
    if (aht20_working_addr == 0) {
        ESP_LOGE(IOTAG, "AHT20: 传感器未初始化或初始化失败");
        return ESP_ERR_INVALID_STATE;
    }
    
    // 按照官方流程：发送测量命令 0xAC 0x33 0x00
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (cmd == NULL) {
        ESP_LOGE(IOTAG, "AHT20: I2C命令链创建失败");
        return ESP_ERR_NO_MEM;
    }
    
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (aht20_working_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0xAC, true);  // 测量命令
    i2c_master_write_byte(cmd, 0x33, true);  // 参数1
    i2c_master_write_byte(cmd, 0x00, true);  // 参数2
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(IOTAG, "AHT20: 发送测量命令失败: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 按照官方文档：等待80ms测量完成
    vTaskDelay(pdMS_TO_TICKS(80));
    
    // 按照官方文档：读取7字节数据（状态字 + 温湿度数据 + CRC）
    uint8_t data[7] = {0};
    cmd = i2c_cmd_link_create();
    if (cmd == NULL) {
        ESP_LOGE(IOTAG, "AHT20: I2C命令链创建失败");
        return ESP_ERR_NO_MEM;
    }
    
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (aht20_working_addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 7, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(IOTAG, "AHT20: 读取数据失败: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 检查状态字（第0字节），Bit7为忙状态位
    ESP_LOGI(IOTAG, "AHT20状态字: 0x%02X", data[0]);
    if (data[0] & 0x80) {
        ESP_LOGW(IOTAG, "AHT20: 传感器仍处于忙状态");
        return ESP_ERR_INVALID_RESPONSE;
    }
    
    // 打印原始数据用于调试
    ESP_LOGI(IOTAG, "AHT20原始数据: %02X %02X %02X %02X %02X %02X %02X", 
             data[0], data[1], data[2], data[3], data[4], data[5], data[6]);
    
    // 解析湿度数据 (20位)
    uint32_t humidity_raw = ((uint32_t)data[1] << 12) | ((uint32_t)data[2] << 4) | ((uint32_t)data[3] >> 4);
    *humidity = ((float)humidity_raw / 1048576.0f) * 100.0f;
    
    // 解析温度数据 (20位)
    uint32_t temperature_raw = (((uint32_t)data[3] & 0x0F) << 16) | ((uint32_t)data[4] << 8) | (uint32_t)data[5];
    *temperature = ((float)temperature_raw / 1048576.0f) * 200.0f - 50.0f;
    
    ESP_LOGI(IOTAG, "AHT20解析: 湿度原始=%lu, 温度原始=%lu", humidity_raw, temperature_raw);
    
    last_temperature_aht = *temperature;
    last_humidity = *humidity;
    
    return ESP_OK;
}

// 检查BQ27220是否存在并读取电量（SOC寄存器0x0D）
esp_err_t bq27220_check_and_read_soc(uint8_t *soc, uint16_t *voltage, int16_t *current)
{
    esp_err_t ret;    
    // 读取SOC（电量百分比）- 寄存器0x2C
    uint8_t soc_reg = 0x2C;
    uint8_t soc_data[2] = {0};
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (cmd == NULL) {
        ESP_LOGE(IOTAG, "I2C命令链创建失败");
        *soc = 0;
        *voltage = 0;
        *current = 0;
        return ESP_ERR_NO_MEM;
    }
    
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BQ27220_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, soc_reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BQ27220_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, soc_data, 2, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(250));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        *soc = 0;
        *voltage = 0;
        *current = 0;
        if (ret == ESP_ERR_TIMEOUT) {
            ESP_LOGW(IOTAG, "BQ27220通信超时，可能未连接电池");
        } else {
            ESP_LOGE(IOTAG, "BQ27220通信失败: %s (0x%02X)", esp_err_to_name(ret), ret);
        }
        return ret;
    }
    
    *soc = soc_data[0]; // SOC百分比
    
    // 读取电压 - 寄存器0x08
    uint8_t volt_reg = 0x08;
    uint8_t volt_data[2] = {0};
    
    cmd = i2c_cmd_link_create();
    if (cmd == NULL) {
        ESP_LOGE(IOTAG, "I2C命令链创建失败");
        *voltage = 0;
    } else {
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (BQ27220_ADDR << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, volt_reg, true);
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (BQ27220_ADDR << 1) | I2C_MASTER_READ, true);
        i2c_master_read(cmd, volt_data, 2, I2C_MASTER_LAST_NACK);
        i2c_master_stop(cmd);
        
        ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
        i2c_cmd_link_delete(cmd);
        
        if (ret == ESP_OK) {
            *voltage = (volt_data[1] << 8) | volt_data[0]; // 小端序
        } else {
            ESP_LOGW(IOTAG, "读取电压失败: %s", esp_err_to_name(ret));
            *voltage = 0;
        }
    }
    
    // 读取电流 - 寄存器0x0C
    uint8_t curr_reg = 0x0C;
    uint8_t curr_data[2] = {0};
    
    cmd = i2c_cmd_link_create();
    if (cmd == NULL) {
        ESP_LOGE(IOTAG, "I2C命令链创建失败");
        *current = 0;
    } else {
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (BQ27220_ADDR << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, curr_reg, true);
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (BQ27220_ADDR << 1) | I2C_MASTER_READ, true);
        i2c_master_read(cmd, curr_data, 2, I2C_MASTER_LAST_NACK);
        i2c_master_stop(cmd);
        
        ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
        i2c_cmd_link_delete(cmd);
        
        if (ret == ESP_OK) {
            *current = (int16_t)((curr_data[1] << 8) | curr_data[0]); // 小端序，有符号
        } else {
            ESP_LOGW(IOTAG, "读取电流失败: %s", esp_err_to_name(ret));
            *current = 0;
        }
    }
    
    ESP_LOGI(IOTAG, "BQ27220检测成功 - 电量: %d%%, 电压: %dmV, 电流: %dmA", *soc, *voltage, *current);
    return ESP_OK;
}

// 配置BQ27220设计容量和FCC参数
esp_err_t bq27220_configure_design_capacity(uint16_t design_capacity_mah)
{
    ESP_LOGI(IOTAG, "配置BQ27220设计容量: %dmAh", design_capacity_mah);
    
    esp_err_t ret;
    
    // 步骤1: 解锁BQ27220
    ESP_LOGI(IOTAG, "解锁BQ27220...");
    
    // 发送第一个解锁密钥
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (cmd == NULL) {
        ESP_LOGE(IOTAG, "I2C命令链创建失败");
        return ESP_ERR_NO_MEM;
    }
    
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BQ27220_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, BQ27220_CONTROL_REG, true);
    i2c_master_write_byte(cmd, (BQ27220_UNSEAL_KEY1 & 0xFF), true);        // 低字节
    i2c_master_write_byte(cmd, ((BQ27220_UNSEAL_KEY1 >> 8) & 0xFF), true); // 高字节
    i2c_master_stop(cmd);
    
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(IOTAG, "发送第一个解锁密钥失败: %s", esp_err_to_name(ret));
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // 发送第二个解锁密钥
    cmd = i2c_cmd_link_create();
    if (cmd == NULL) {
        ESP_LOGE(IOTAG, "I2C命令链创建失败");
        return ESP_ERR_NO_MEM;
    }
    
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BQ27220_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, BQ27220_CONTROL_REG, true);
    i2c_master_write_byte(cmd, (BQ27220_UNSEAL_KEY2 & 0xFF), true);        // 低字节
    i2c_master_write_byte(cmd, ((BQ27220_UNSEAL_KEY2 >> 8) & 0xFF), true); // 高字节
    i2c_master_stop(cmd);
    
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(IOTAG, "发送第二个解锁密钥失败: %s", esp_err_to_name(ret));
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // 步骤2: 进入CFG_UPDATE模式
    ESP_LOGI(IOTAG, "进入CFG_UPDATE模式...");
    
    cmd = i2c_cmd_link_create();
    if (cmd == NULL) {
        ESP_LOGE(IOTAG, "I2C命令链创建失败");
        return ESP_ERR_NO_MEM;
    }
    
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BQ27220_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, BQ27220_CONTROL_REG, true);
    i2c_master_write_byte(cmd, 0x90, true); // CFG_UPDATE命令低字节
    i2c_master_write_byte(cmd, 0x00, true); // CFG_UPDATE命令高字节
    i2c_master_stop(cmd);
    
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(IOTAG, "进入CFG_UPDATE模式失败: %s", esp_err_to_name(ret));
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(100)); // 等待进入CFG_UPDATE模式
    
    // 步骤3: 配置设计容量 (Design Capacity) - 地址0x4A4A
    ESP_LOGI(IOTAG, "配置设计容量: %dmAh", design_capacity_mah);
    
    // 设置数据内存地址 (0x4A4A) 到寄存器0x3E/0x3F
    cmd = i2c_cmd_link_create();
    if (cmd == NULL) {
        ESP_LOGE(IOTAG, "I2C命令链创建失败");
        return ESP_ERR_NO_MEM;
    }
    
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BQ27220_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x3E, true); // 地址寄存器低字节
    i2c_master_write_byte(cmd, 0x4A, true); // 设计容量地址低字节
    i2c_master_stop(cmd);
    
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(IOTAG, "设置设计容量地址低字节失败: %s", esp_err_to_name(ret));
        return ret;
    }
    
    cmd = i2c_cmd_link_create();
    if (cmd == NULL) {
        ESP_LOGE(IOTAG, "I2C命令链创建失败");
        return ESP_ERR_NO_MEM;
    }
    
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BQ27220_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x3F, true); // 地址寄存器高字节
    i2c_master_write_byte(cmd, 0x4A, true); // 设计容量地址高字节
    i2c_master_stop(cmd);
    
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(IOTAG, "设置设计容量地址高字节失败: %s", esp_err_to_name(ret));
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // 写入设计容量数据到0x40寄存器 (大端序)
    cmd = i2c_cmd_link_create();
    if (cmd == NULL) {
        ESP_LOGE(IOTAG, "I2C命令链创建失败");
        return ESP_ERR_NO_MEM;
    }
    
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BQ27220_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x40, true); // 数据寄存器
    i2c_master_write_byte(cmd, ((design_capacity_mah >> 8) & 0xFF), true); // 高字节
    i2c_master_write_byte(cmd, (design_capacity_mah & 0xFF), true);        // 低字节
    i2c_master_stop(cmd);
    
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(IOTAG, "写入设计容量数据失败: %s", esp_err_to_name(ret));
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // 步骤4: 配置FCC (Full Charge Capacity) - 地址0x4A4C
    ESP_LOGI(IOTAG, "配置FCC: %dmAh", design_capacity_mah);
    
    // 设置FCC地址 (0x4A4C) 到寄存器0x3E/0x3F
    cmd = i2c_cmd_link_create();
    if (cmd == NULL) {
        ESP_LOGE(IOTAG, "I2C命令链创建失败");
        return ESP_ERR_NO_MEM;
    }
    
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BQ27220_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x3E, true); // 地址寄存器低字节
    i2c_master_write_byte(cmd, 0x4C, true); // FCC地址低字节
    i2c_master_stop(cmd);
    
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(IOTAG, "设置FCC地址低字节失败: %s", esp_err_to_name(ret));
        return ret;
    }
    
    cmd = i2c_cmd_link_create();
    if (cmd == NULL) {
        ESP_LOGE(IOTAG, "I2C命令链创建失败");
        return ESP_ERR_NO_MEM;
    }
    
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BQ27220_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x3F, true); // 地址寄存器高字节
    i2c_master_write_byte(cmd, 0x4A, true); // FCC地址高字节
    i2c_master_stop(cmd);
    
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(IOTAG, "设置FCC地址高字节失败: %s", esp_err_to_name(ret));
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // 写入FCC数据到0x40寄存器 (大端序)
    cmd = i2c_cmd_link_create();
    if (cmd == NULL) {
        ESP_LOGE(IOTAG, "I2C命令链创建失败");
        return ESP_ERR_NO_MEM;
    }
    
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BQ27220_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x40, true); // 数据寄存器
    i2c_master_write_byte(cmd, ((design_capacity_mah >> 8) & 0xFF), true); // 高字节
    i2c_master_write_byte(cmd, (design_capacity_mah & 0xFF), true);        // 低字节
    i2c_master_stop(cmd);
    
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(IOTAG, "写入FCC数据失败: %s", esp_err_to_name(ret));
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // 步骤5: 退出CFG_UPDATE模式
    ESP_LOGI(IOTAG, "退出CFG_UPDATE模式...");
    
    cmd = i2c_cmd_link_create();
    if (cmd == NULL) {
        ESP_LOGE(IOTAG, "I2C命令链创建失败");
        return ESP_ERR_NO_MEM;
    }
    
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BQ27220_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, BQ27220_CONTROL_REG, true);
    i2c_master_write_byte(cmd, 0x92, true); // EXIT_CFG_UPDATE命令低字节
    i2c_master_write_byte(cmd, 0x00, true); // EXIT_CFG_UPDATE命令高字节
    i2c_master_stop(cmd);
    
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(IOTAG, "退出CFG_UPDATE模式失败: %s", esp_err_to_name(ret));
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(100)); // 等待退出CFG_UPDATE模式
    
    ESP_LOGI(IOTAG, "BQ27220设计容量配置完成: %dmAh", design_capacity_mah);
    ESP_LOGI(IOTAG, "注意: 配置生效后需要进行完整的充放电循环以校准SOC");
    
    return ESP_OK;
}

// 配置BQ27220 GPOUT低电量警报功能
esp_err_t bq27220_configure_gpout_low_battery_alert(uint8_t soc_threshold)
{
    ESP_LOGI(IOTAG, "配置BQ27220 GPOUT低电量警报，阈值: %d%%", soc_threshold);
    
    esp_err_t ret;
    
    // 步骤1: 解锁BQ27220
    ESP_LOGI(IOTAG, "解锁BQ27220...");
    
    // 发送第一个解锁密钥
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (cmd == NULL) {
        ESP_LOGE(IOTAG, "I2C命令链创建失败");
        return ESP_ERR_NO_MEM;
    }
    
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BQ27220_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, BQ27220_CONTROL_REG, true);
    i2c_master_write_byte(cmd, (BQ27220_UNSEAL_KEY1 & 0xFF), true);        // 低字节
    i2c_master_write_byte(cmd, ((BQ27220_UNSEAL_KEY1 >> 8) & 0xFF), true); // 高字节
    i2c_master_stop(cmd);
    
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(IOTAG, "发送第一个解锁密钥失败: %s", esp_err_to_name(ret));
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // 发送第二个解锁密钥
    cmd = i2c_cmd_link_create();
    if (cmd == NULL) {
        ESP_LOGE(IOTAG, "I2C命令链创建失败");
        return ESP_ERR_NO_MEM;
    }
    
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BQ27220_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, BQ27220_CONTROL_REG, true);
    i2c_master_write_byte(cmd, (BQ27220_UNSEAL_KEY2 & 0xFF), true);
    i2c_master_write_byte(cmd, ((BQ27220_UNSEAL_KEY2 >> 8) & 0xFF), true);
    i2c_master_stop(cmd);
    
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(IOTAG, "发送第二个解锁密钥失败: %s", esp_err_to_name(ret));
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // 步骤2: 进入CFG_UPDATE模式
    ESP_LOGI(IOTAG, "进入CFG_UPDATE模式...");
    
    cmd = i2c_cmd_link_create();
    if (cmd == NULL) {
        ESP_LOGE(IOTAG, "I2C命令链创建失败");
        return ESP_ERR_NO_MEM;
    }
    
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BQ27220_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, BQ27220_CONTROL_REG, true);
    i2c_master_write_byte(cmd, 0x90, true); // ENTER_CFG_UPDATE 低字节
    i2c_master_write_byte(cmd, 0x00, true); // ENTER_CFG_UPDATE 高字节
    i2c_master_stop(cmd);
    
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(IOTAG, "进入CFG_UPDATE模式失败: %s", esp_err_to_name(ret));
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(100)); // 等待进入CFG_UPDATE模式
    
    // 步骤3: 配置SOC Set Threshold (0x48D2)
    ESP_LOGI(IOTAG, "配置SOC Set Threshold为 %d%%...", soc_threshold);
    
    // 写入数据内存地址和数据
    cmd = i2c_cmd_link_create();
    if (cmd == NULL) {
        ESP_LOGE(IOTAG, "I2C命令链创建失败");
        return ESP_ERR_NO_MEM;
    }
    
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BQ27220_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x3E, true); // 数据内存访问寄存器
    i2c_master_write_byte(cmd, 0xD2, true); // 0x48D2 地址低字节
    i2c_master_write_byte(cmd, 0x48, true); // 0x48D2 地址高字节
    i2c_master_write_byte(cmd, soc_threshold, true); // SOC阈值
    i2c_master_stop(cmd);
    
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(IOTAG, "配置SOC Set Threshold失败: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 计算校验和并写入
    uint8_t checksum = (0xFF - (0xD2 + 0x48 + soc_threshold)) & 0xFF;
    
    cmd = i2c_cmd_link_create();
    if (cmd == NULL) {
        ESP_LOGE(IOTAG, "I2C命令链创建失败");
        return ESP_ERR_NO_MEM;
    }
    
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BQ27220_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x60, true); // 校验和寄存器
    i2c_master_write_byte(cmd, checksum, true); // 校验和
    i2c_master_write_byte(cmd, 0x05, true); // 数据长度
    i2c_master_stop(cmd);
    
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(IOTAG, "写入SOC Set Threshold校验和失败: %s", esp_err_to_name(ret));
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // 步骤4: 配置SOC Clear Threshold (0x48D3)
    uint8_t clear_threshold = soc_threshold + 3;
    ESP_LOGI(IOTAG, "配置SOC Clear Threshold为 %d%%...", clear_threshold);
    
    cmd = i2c_cmd_link_create();
    if (cmd == NULL) {
        ESP_LOGE(IOTAG, "I2C命令链创建失败");
        return ESP_ERR_NO_MEM;
    }
    
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BQ27220_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x3E, true); // 数据内存访问寄存器
    i2c_master_write_byte(cmd, 0xD3, true); // 0x48D3 地址低字节
    i2c_master_write_byte(cmd, 0x48, true); // 0x48D3 地址高字节
    i2c_master_write_byte(cmd, clear_threshold, true); // SOC清除阈值
    i2c_master_stop(cmd);
    
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(IOTAG, "配置SOC Clear Threshold失败: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 计算校验和并写入
    checksum = (0xFF - (0xD3 + 0x48 + clear_threshold)) & 0xFF;
    
    cmd = i2c_cmd_link_create();
    if (cmd == NULL) {
        ESP_LOGE(IOTAG, "I2C命令链创建失败");
        return ESP_ERR_NO_MEM;
    }
    
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BQ27220_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x60, true); // 校验和寄存器
    i2c_master_write_byte(cmd, checksum, true); // 校验和
    i2c_master_write_byte(cmd, 0x05, true); // 数据长度
    i2c_master_stop(cmd);
    
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(IOTAG, "写入SOC Clear Threshold校验和失败: %s", esp_err_to_name(ret));
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // 步骤5: 退出CFG_UPDATE模式并重新初始化
    ESP_LOGI(IOTAG, "退出CFG_UPDATE模式...");
    
    cmd = i2c_cmd_link_create();
    if (cmd == NULL) {
        ESP_LOGE(IOTAG, "I2C命令链创建失败");
        return ESP_ERR_NO_MEM;
    }
    
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BQ27220_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, BQ27220_CONTROL_REG, true);
    i2c_master_write_byte(cmd, 0x91, true); // EXIT_CFG_UPDATE_REINIT 低字节
    i2c_master_write_byte(cmd, 0x00, true); // EXIT_CFG_UPDATE_REINIT 高字节
    i2c_master_stop(cmd);
    
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(IOTAG, "退出CFG_UPDATE模式失败: %s", esp_err_to_name(ret));
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(10)); // 短暂延迟
    
    // 步骤6: 配置IO Config寄存器启用BATLOWEN功能
    ESP_LOGI(IOTAG, "配置IO Config寄存器启用BATLOWEN...");
    
    // 写入IO Config寄存器地址0x4857
    cmd = i2c_cmd_link_create();
    if (cmd == NULL) {
        ESP_LOGE(IOTAG, "I2C命令链创建失败");
        return ESP_ERR_NO_MEM;
    }
    
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BQ27220_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x3E, true);  // 数据内存访问命令
    i2c_master_write_byte(cmd, 0x57, true);  // IO Config地址低字节
    i2c_master_write_byte(cmd, 0x48, true);  // IO Config地址高字节
    i2c_master_stop(cmd);
    
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(IOTAG, "设置IO Config地址失败: %s", esp_err_to_name(ret));
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // 读取当前IO Config值
    uint8_t io_config_data[2] = {0};
    cmd = i2c_cmd_link_create();
    if (cmd == NULL) {
        ESP_LOGE(IOTAG, "I2C命令链创建失败");
        return ESP_ERR_NO_MEM;
    }
    
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BQ27220_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x40, true);  // 数据内存读取命令
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BQ27220_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, io_config_data, 2, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(IOTAG, "读取IO Config失败: %s", esp_err_to_name(ret));
        return ret;
    }
    
    uint16_t io_config_value = (io_config_data[1] << 8) | io_config_data[0];
    ESP_LOGI(IOTAG, "当前IO Config值: 0x%04X", io_config_value);
    
    // 设置BATLOWEN位(bit 2)和GPIOPOL位(bit 11)来启用低电量警报
    // BATLOWEN = 1: 启用电池低电量功能
    // GPIOPOL = 0: GPOUT低电平有效(默认)
    io_config_value |= (1 << 2);   // 设置BATLOWEN位
    io_config_value &= ~(1 << 11); // 清除GPIOPOL位(低电平有效)
    
    ESP_LOGI(IOTAG, "新的IO Config值: 0x%04X", io_config_value);
    
    // 计算校验和
    uint8_t io_config_checksum = (uint8_t)(0xFF - ((0x57 + 0x48 + (io_config_value & 0xFF) + ((io_config_value >> 8) & 0xFF)) & 0xFF));
    
    // 写入新的IO Config值
    cmd = i2c_cmd_link_create();
    if (cmd == NULL) {
        ESP_LOGE(IOTAG, "I2C命令链创建失败");
        return ESP_ERR_NO_MEM;
    }
    
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BQ27220_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x40, true);  // 数据内存写入命令
    i2c_master_write_byte(cmd, (io_config_value & 0xFF), true);        // 低字节
    i2c_master_write_byte(cmd, ((io_config_value >> 8) & 0xFF), true); // 高字节
    i2c_master_write_byte(cmd, io_config_checksum, true);              // 校验和
    i2c_master_stop(cmd);
    
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(IOTAG, "写入IO Config失败: %s", esp_err_to_name(ret));
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_LOGI(IOTAG, "IO Config寄存器配置完成，BATLOWEN已启用");
    
    ESP_LOGI(IOTAG, "BQ27220 GPOUT低电量警报配置完成");
    ESP_LOGI(IOTAG, "触发阈值: %d%%, 清除阈值: %d%%", soc_threshold, clear_threshold);
    ESP_LOGI(IOTAG, "IO Config: 0x%04X (BATLOWEN已启用)", io_config_value);
    
    return ESP_OK;
}

// =========================== 总运行时间管理系统 ===========================

// 从NVS加载刷新率设置
void load_refresh_rate_from_nvs(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("display_config", NVS_READONLY, &nvs_handle);
    if (err == ESP_OK) {
        size_t required_size = sizeof(current_refresh_rate);
        err = nvs_get_str(nvs_handle, "refresh_rate", current_refresh_rate, &required_size);
        if (err != ESP_OK) {
            strcpy(current_refresh_rate, "1"); // 默认1Hz
        }
        nvs_close(nvs_handle);
        ESP_LOGI(SLEEPTAG, "从NVS加载刷新率设置: %s", current_refresh_rate);
    } else {
        strcpy(current_refresh_rate, "1"); // 默认1Hz
        ESP_LOGI(SLEEPTAG, "使用默认刷新率设置: %s", current_refresh_rate);
    }
}

// 初始化NVS并分别读取总运行时间
static esp_err_t init_runtime_nvs(void)
{
    ESP_LOGI(NVSTAG, "初始化运行时间NVS存储...");
    
    // 打开NVS命名空间
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &runtime_nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(NVSTAG, "NVS打开失败: %s", esp_err_to_name(err));
        return err;
    }
    
    // 读取总运行时间
    size_t runtime_size = sizeof(total_runtime_minutes);
    err = nvs_get_blob(runtime_nvs_handle, NVS_RUNTIME_KEY, &total_runtime_minutes, &runtime_size);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        total_runtime_minutes = 0;
        ESP_LOGI(NVSTAG, "首次运行，总运行时间初始化为0");
    } else if (err == ESP_OK) {
        ESP_LOGI(NVSTAG, "从NVS读取总运行时间: %lu分钟", total_runtime_minutes);
    } else {
        ESP_LOGE(NVSTAG, "总运行时间读取失败: %s", esp_err_to_name(err));
        total_runtime_minutes = 0;
    }
    
    // 如果是首次运行，保存初始值
    if (total_runtime_minutes == 0 && last_saved_rtc_time == 0) {
        save_runtime_to_nvs();
    }
    
    return ESP_OK;
}

// 分别保存总运行时间和RTC时间到NVS
static esp_err_t save_runtime_to_nvs(void)
{
    if (runtime_nvs_handle == 0) {
        ESP_LOGE(NVSTAG, "NVS未初始化");
        return ESP_FAIL;
    }
    
    esp_err_t err = ESP_OK;
    
    // 保存总运行时间
    err = nvs_set_blob(runtime_nvs_handle, NVS_RUNTIME_KEY, &total_runtime_minutes, sizeof(total_runtime_minutes));
    if (err != ESP_OK) {
        ESP_LOGE(NVSTAG, "总运行时间保存失败: %s", esp_err_to_name(err));
        return err;
    }
    
    // 提交更改
    err = nvs_commit(runtime_nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(NVSTAG, "NVS提交失败: %s", esp_err_to_name(err));
        return err;
    }
    
    // 更新缓存的时间
    // last_saved_rtc_time = current_time;
    
    ESP_LOGI(NVSTAG, "✓ 运行时间已保存: %lu分钟", total_runtime_minutes);
    // ESP_LOGI(NVSTAG, "✓ RTC时间已保存: %04d-%02d-%02d %02d:%02d:%02d", 
    //          timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
    //          timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
    return ESP_OK;
}

// 开启高压运行时间计数
static void start_hv_runtime_counter(void)
{
    if (!hv_enabled) {
        hv_enabled = true;
        hv_start_time = esp_timer_get_time(); // 微秒
        ESP_LOGI(NVSTAG, "高压运行时间计数开始（无独立定时器，依赖显示定时器）");
        
        // 不创建独立的运行时间保存定时器
        // 运行时间更新将与显示定时器同步进行
    }
}

// 停止高压运行时间计数
static void stop_hv_runtime_counter(void)
{
    if (hv_enabled) {
        // 更新运行时间
        update_runtime_counter();
        
        hv_enabled = false;
        hv_start_time = 0;
        ESP_LOGI(NVSTAG, "高压运行时间计数停止");
        // 最后保存一次
        save_runtime_to_nvs();
    }
}

// 更新运行时间计数
static void update_runtime_counter(void)
{
    if (hv_enabled && hv_start_time > 0) {
        int64_t current_time = esp_timer_get_time(); // 微秒
        int64_t elapsed_microseconds = current_time - hv_start_time;
        uint32_t elapsed_minutes = elapsed_microseconds / (60 * 1000000); // 转换为分钟
        
        if (elapsed_minutes > 0) {
            total_runtime_minutes += elapsed_minutes;
            hv_start_time = current_time; // 重置起始时间
            ESP_LOGI(NVSTAG, "运行时间更新: +%lu分钟, 总计: %lu分钟", elapsed_minutes, total_runtime_minutes);
        }
    }
}



// 获取格式化的总运行时间字符串（供web服务器调用）
void get_total_runtime_string(char* buffer, size_t buffer_size)
{
    if (buffer == NULL || buffer_size < 16) {
        ESP_LOGE(NVSTAG, "缓冲区无效");
        return;
    }
    
    // 先更新当前计数（如果正在运行）
    if (hv_enabled) {
        update_runtime_counter();
    }
    
    uint32_t hours = total_runtime_minutes / 60;
    uint32_t minutes = total_runtime_minutes % 60;
    
    // 格式化为不带前导零的格式：xH xM
    snprintf(buffer, buffer_size, "%luH%luM", hours, minutes);
}

// 获取总运行时间（分钟）
uint32_t get_total_runtime_minutes(void)
{
    // 先更新当前计数（如果正在运行）
    if (hv_enabled) {
        update_runtime_counter();
    }
    return total_runtime_minutes;
}

// 获取最后保存的RTC时间信息
void get_last_saved_rtc_info(char* buffer, size_t buffer_size)
{
    if (buffer == NULL || buffer_size < 32) {
        ESP_LOGE(NVSTAG, "缓冲区无效");
        return;
    }
    
    if (last_saved_rtc_time == 0) {
        snprintf(buffer, buffer_size, "未保存RTC时间");
        return;
    }
    
    struct tm timeinfo;
    localtime_r(&last_saved_rtc_time, &timeinfo);
    
    snprintf(buffer, buffer_size, "%04d-%02d-%02d %02d:%02d:%02d", 
             timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
             timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
}

// 获取最后保存的RTC时间戳
time_t get_last_saved_rtc_timestamp(void)
{
    return last_saved_rtc_time;
}

// 测试函数：显示当前运行时间和最后保存的RTC时间信息
void print_runtime_and_rtc_info(void)
{
    // 显示当前运行时间
    char runtime_str[32];
    get_total_runtime_string(runtime_str, sizeof(runtime_str));
    
    // 显示最后保存的RTC时间
    char rtc_info[64];
    get_last_saved_rtc_info(rtc_info, sizeof(rtc_info));
    
    // 显示当前系统时间
    time_t current_time;
    struct tm current_timeinfo;
    time(&current_time);
    localtime_r(&current_time, &current_timeinfo);
    
    ESP_LOGI(NVSTAG, "==== 运行时间和RTC时间信息 ====");
    ESP_LOGI(NVSTAG, "总运行时间: %s", runtime_str);
    ESP_LOGI(NVSTAG, "最后保存的RTC时间: %s", rtc_info);
    ESP_LOGI(NVSTAG, "当前系统时间: %04d-%02d-%02d %02d:%02d:%02d",
             current_timeinfo.tm_year + 1900, current_timeinfo.tm_mon + 1, current_timeinfo.tm_mday,
             current_timeinfo.tm_hour, current_timeinfo.tm_min, current_timeinfo.tm_sec);
    ESP_LOGI(NVSTAG, "================================");
}

// // 对外接口：仅保存运行时间
// esp_err_t save_runtime_only(void)
// {
//     return save_runtime_only_to_nvs();
// }

// 对外接口：仅保存RTC时间
// esp_err_t save_rtc_time_only(void)
// {
//     return save_rtc_time_only_to_nvs();
// }

// =========================== 恢复出厂设置系统 ===========================

// 执行恢复出厂设置
esp_err_t perform_factory_reset(void)
{
    ESP_LOGW(NVSTAG, "==>> 开始执行恢复出厂设置 <<==");
    
    esp_err_t ret = ESP_OK;
    
    // 1. 清除NVS中的所有用户数据
    ESP_LOGI(NVSTAG, "正在清除NVS用户数据...");
    
    if (runtime_nvs_handle != 0) {
        // 关闭当前的NVS句柄
        nvs_close(runtime_nvs_handle);
        runtime_nvs_handle = 0;
    }
    
    // 清除所有相关的NVS命名空间（与Web界面保持一致）
    const char* nvs_namespaces[] = {
        "wifi_config",   // WiFi管理器配置
        "nvs.net80211",  // ESP32内部WiFi配置 
        "ntp_config",    // NTP配置
        "nixie_clock",   // 辉光管控制器配置
        "nixie_storage", // 运行时间等
        "storage",       // 其他配置
        NULL
    };
    
    nvs_handle_t nvs_handle;
    for (int i = 0; nvs_namespaces[i] != NULL; i++) {
        esp_err_t ret = nvs_open(nvs_namespaces[i], NVS_READWRITE, &nvs_handle);
        if (ret == ESP_OK) {
            esp_err_t erase_ret = nvs_erase_all(nvs_handle);
            if (erase_ret == ESP_OK) {
                esp_err_t commit_ret = nvs_commit(nvs_handle);
                if (commit_ret == ESP_OK) {
                    ESP_LOGI(NVSTAG, "✓ 已清除命名空间: %s", nvs_namespaces[i]);
                } else {
                    ESP_LOGW(NVSTAG, "⚠️ 提交清除失败 %s: %s", nvs_namespaces[i], esp_err_to_name(commit_ret));
                }
            } else {
                ESP_LOGW(NVSTAG, "⚠️ 清除失败 %s: %s", nvs_namespaces[i], esp_err_to_name(erase_ret));
            }
            nvs_close(nvs_handle);
        } else {
            if (ret != ESP_ERR_NVS_NOT_FOUND) {
                ESP_LOGW(NVSTAG, "⚠️ 无法打开命名空间 %s: %s", nvs_namespaces[i], esp_err_to_name(ret));
            } else {
                ESP_LOGI(NVSTAG, "命名空间 %s 不存在，跳过", nvs_namespaces[i]);
            }
        }
    }
    
    // 2. 创建完全禁用的WiFi配置
    ESP_LOGI(NVSTAG, "创建完全禁用的WiFi配置...");
    wifi_config_persistent_t disabled_config;
    memset(&disabled_config, 0, sizeof(wifi_config_persistent_t));
    
    // 设置默认AP名称（带MAC后缀）
    char mac_suffix[5];
    esp_err_t mac_ret = wifi_manager_get_mac_suffix(mac_suffix, sizeof(mac_suffix));
    if (mac_ret == ESP_OK) {
        snprintf(disabled_config.ap_ssid, sizeof(disabled_config.ap_ssid), "%s_%s", 
                WIFI_MANAGER_DEFAULT_SSID, mac_suffix);
    } else {
        strcpy(disabled_config.ap_ssid, WIFI_MANAGER_DEFAULT_SSID);
    }
    
    strcpy(disabled_config.ap_password, WIFI_MANAGER_DEFAULT_PASSWORD);
    disabled_config.ap_channel = WIFI_MANAGER_DEFAULT_CHANNEL;
    
    // 重要：启用WiFi功能（APSTA模式为默认模式）
    disabled_config.sta_enabled = true;
    disabled_config.ap_enabled = true;
    disabled_config.mode = WIFI_MODE_AP_STA;
    
    // 设置默认AP IP配置
    strcpy(disabled_config.ap_ip, "192.168.4.1");
    strcpy(disabled_config.ap_gateway, "192.168.4.1");
    strcpy(disabled_config.ap_netmask, "255.255.255.0");
    
    ESP_LOGI(NVSTAG, "恢复出厂设置配置详情:");
    ESP_LOGI(NVSTAG, "  AP_SSID: %s", disabled_config.ap_ssid);
    ESP_LOGI(NVSTAG, "  STA_ENABLED: %s", disabled_config.sta_enabled ? "true" : "false");
    ESP_LOGI(NVSTAG, "  AP_ENABLED: %s", disabled_config.ap_enabled ? "true" : "false");
    ESP_LOGI(NVSTAG, "  MODE: %d (APSTA - 默认WiFi模式)", disabled_config.mode);
    
    // 保存恢复出厂设置配置到NVS
    esp_err_t save_ret = wifi_manager_save_config(&disabled_config);
    if (save_ret == ESP_OK) {
        ESP_LOGI(NVSTAG, "✓ 已保存恢复出厂设置WiFi配置（APSTA模式）");
    } else {
        ESP_LOGW(NVSTAG, "⚠️ 保存恢复出厂设置配置失败: %s", esp_err_to_name(save_ret));
    }
    
    // 3. 创建恢复出厂设置标记
    ESP_LOGI(NVSTAG, "创建恢复出厂设置标记...");
    nvs_handle_t marker_handle;
    esp_err_t marker_ret = nvs_open("system", NVS_READWRITE, &marker_handle);
    if (marker_ret == ESP_OK) {
        uint8_t factory_reset_flag = 1;
        esp_err_t set_ret = nvs_set_u8(marker_handle, "factory_reset", factory_reset_flag);
        if (set_ret == ESP_OK) {
            nvs_commit(marker_handle);
            ESP_LOGI(NVSTAG, "✓ 恢复出厂设置标记已创建");
        }
        nvs_close(marker_handle);
    }
    
    // 4. 重置运行时间变量
    ESP_LOGI(NVSTAG, "重置运行时间计数器...");
    total_runtime_minutes = 0;
    last_saved_rtc_time = 0;
    hv_enabled = false;
    hv_start_time = 0;
    ESP_LOGI(NVSTAG, "✓ 运行时间计数器已重置");
    
    // 5. 重置唤醒模式到默认值
    ESP_LOGI(NVSTAG, "重置唤醒模式...");
    current_wakeup_mode = WAKEUP_MODE_SECOND;
    ESP_LOGI(NVSTAG, "✓ 唤醒模式已重置为: %s", wakeup_mode_names[current_wakeup_mode]);
    
    // 6. 重新初始化NVS系统
    ESP_LOGI(NVSTAG, "重新初始化NVS系统...");
    ret = init_runtime_nvs();
    if (ret == ESP_OK) {
        ESP_LOGI(NVSTAG, "✓ NVS系统重新初始化完成");
    } else {
        ESP_LOGE(NVSTAG, "NVS系统重新初始化失败: %s", esp_err_to_name(ret));
    }
    
    // 7. 保存默认设置
    ESP_LOGI(NVSTAG, "保存默认设置...");
    save_runtime_to_nvs();
    
    ESP_LOGW(NVSTAG, "==>> 恢复出厂设置完成 <<==");
    ESP_LOGW(NVSTAG, "系统将在关机后重启至初始状态");
    
    return ESP_OK;
}

// 清除恢复出厂设置标志
static void clear_factory_reset_flag(void)
{
    if (factory_reset_pending) {
        factory_reset_pending = false;
        ESP_LOGW(NVSTAG, "恢复出厂设置标志已清除，取消恢复操作");
    }
}

// 对外接口：检查是否有恢复出厂设置标志
bool is_factory_reset_pending(void)
{
    return factory_reset_pending;
}

// 对外接口：取消恢复出厂设置
void cancel_factory_reset(void)
{
    clear_factory_reset_flag();
}

// 对外接口：获取当前唤醒模式
int get_current_wakeup_mode(void)
{
    return (int)current_wakeup_mode;
}

// 设置唤醒模式
void set_wakeup_mode(int mode)
{
    if (mode >= 0 && mode <= 1) {
        current_wakeup_mode = (wakeup_mode_t)mode;
        ESP_LOGI("WAKEUP", "唤醒模式已切换为: %s", wakeup_mode_names[mode]);
        
        // 停止当前定时器
        if (deep_sleep_timer != NULL) {
            esp_timer_stop(deep_sleep_timer);
            esp_timer_delete(deep_sleep_timer);
            deep_sleep_timer = NULL;
        }
        
        // 重新初始化定时器系统
        deep_sleep_timer_init();
    } else {
        ESP_LOGW("WAKEUP", "无效的唤醒模式: %d", mode);
    }
}

// 设置显示配置
void set_display_config(const char* display_type, const char* custom_content)
{
    if (display_type != NULL) {
        strncpy(current_display_type, display_type, sizeof(current_display_type) - 1);
        current_display_type[sizeof(current_display_type) - 1] = '\0';
    }
    
    if (custom_content != NULL) {
        strncpy(current_custom_content, custom_content, sizeof(current_custom_content) - 1);
        current_custom_content[sizeof(current_custom_content) - 1] = '\0';
    }
    
    ESP_LOGI("DISPLAY", "显示配置已更新: type=%s, custom=%s", current_display_type, current_custom_content);
}

// 获取显示配置
void get_display_config(char* display_type, size_t display_type_size, char* custom_content, size_t custom_content_size)
{
    if (display_type != NULL && display_type_size > 0) {
        strncpy(display_type, current_display_type, display_type_size - 1);
        display_type[display_type_size - 1] = '\0';
    }
    
    if (custom_content != NULL && custom_content_size > 0) {
        strncpy(custom_content, current_custom_content, custom_content_size - 1);
        custom_content[custom_content_size - 1] = '\0';
    }
}

// 电池电流读取函数实现
float get_battery_current(void)
{
    return (float)last_battery_current;
}

// 电池电压读取函数实现
float get_battery_voltage(void)
{
    return last_battery_voltage;
}

// 电池电量百分比读取函数实现
int get_battery_percentage(void)
{
    return last_battery_percentage;    
}

float get_bmp580_temperature(void)
{
    return last_temperature_bmp;
}

float get_bmp580_pressure(void)
{
    return last_pressure;
}

float get_aht20_temperature(void)
{
    return last_temperature_aht;
}

float get_aht20_humidity(void)
{
    return last_humidity;
}

// bool are_sensors_initialized(void)
// {
//     return sensors_initialized;
// }

esp_err_t gpio_set_od_level(gpio_num_t optIO, uint32_t level)
{
    esp_err_t ret = ESP_OK;
    if(level == 1)
    {
        // 对于开漏输出，设置高电平时：禁用下拉，使能上拉，设置输出高
        ret = gpio_pulldown_dis(optIO);
        if (ret != ESP_OK) return ret;
        
        ret = gpio_pullup_en(optIO);
        if (ret != ESP_OK) return ret;
        
        ret = gpio_set_level(optIO, 1);
        if (ret != ESP_OK) return ret;
        
        return ESP_OK;
    }
    else if (level == 0)
    {
        // 对于开漏输出，设置低电平时：禁用上拉，使能下拉，设置输出低
        ret = gpio_pullup_dis(optIO);
        if (ret != ESP_OK) return ret;
        
        ret = gpio_pulldown_en(optIO);
        if (ret != ESP_OK) return ret;
        
        ret = gpio_set_level(optIO, 0);
        if (ret != ESP_OK) return ret;
        
        return ESP_OK;        
    }   
    return ESP_FAIL; 
}

uint8_t** str2BitPos(const char* str) 
{    
    // 修复内存分配错误：应该分配4个指针，不是8个
    uint8_t** result = (uint8_t**)malloc(4 * sizeof(uint8_t*));
    if (result == NULL) {
        ESP_LOGE(IOTAG, "Failed to allocate memory for result");
        return NULL;
    }
    for (int i = 0; i < 4; i++) {  // 修正：只分配4行
        result[i] = (uint8_t*)malloc(4 * sizeof(uint8_t)); // 每行分配4个uint8_t
        if (result[i] == NULL) {
            ESP_LOGE(IOTAG, "Failed to allocate memory for result[%d]", i);
            // 如果分配失败，释放已分配的内存
            for (int j = 0; j < i; j++) {
                free(result[j]);
            }
            free(result);
            return NULL;
        }
    }
    // 初始化数组的值（可根据需求修改）
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            result[i][j] = -1; // 示例初始化
        }
    }

    // 变量用于存储小数点的位置
    int digitCount = 0;
    // 遍历输入字符串
    for (int i = 0; str[i] != '\0' && digitCount < 4; i++) 
    {
        if (str[i] >= '1' && str[i] <= '9') 
        {
            result[2][digitCount] = str[i] - 48; // 存储前4个数字
            digitCount++;
        } 
        if(str[i] == '0')
        {            
            result[2][digitCount] = 10; // 存储前4个数字
            digitCount++;
        }        
        if (str[i] == '.') 
        {
            result[3][digitCount] = 0; // 记录小数点位置            
        }
        else
        {            
            result[3][digitCount] = -1; // 记录小数点位置      
        }  
        //ESP_LOGI(TAG, "atoi %c,%d\n",str[i],digitCount);  
        //ESP_LOGI(TAG, "strBitPos %d,%d,%d,%d,%d,%d,%d,%d \r",result[2][0],result[2][1],result[2][2],result[2][3],result[3][0],result[3][1],result[3][2],result[3][3]);  
    }
    
    result[0][3] = 11 - result[2][0];
    result[0][2] = 11 - result[2][1];
    result[0][1] = 11 - result[2][2];
    result[0][0] = 11 - result[2][3];
    result[1][3] = result[3][0];
    result[1][2] = result[3][1];
    result[1][1] = result[3][2];
    result[1][0] = result[3][3];

    return result; // 返回数组的指针
}

// 使用静态缓冲区的str2BitPos版本，避免动态内存分配
uint8_t** str2BitPos_static(const char* str) 
{    
    // 初始化数组的值
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            bit_pos_buffer[i][j] = -1; // 示例初始化
        }
    }

    // 变量用于存储小数点的位置
    int digitCount = 0;
    // 遍历输入字符串
    for (int i = 0; str[i] != '\0' && digitCount < 4; i++) 
    {
        if (str[i] >= '1' && str[i] <= '9') {
            bit_pos_buffer[2][digitCount] = str[i] - 48; // 存储前4个数字
            digitCount++;
        } else if(str[i] == '0') {            
            bit_pos_buffer[2][digitCount] = 10; // 存储前4个数字
            digitCount++;
        } 
        // else{
        //     bit_pos_buffer[2][digitCount] = 0; // 存储前非数值字符
        //     digitCount++;
        // }    
        if (str[i] == '.') {
            bit_pos_buffer[3][digitCount] = 0; // 记录小数点位置            
        } else {            
            bit_pos_buffer[3][digitCount] = -1; // 记录小数点位置      
        }  
    }
    
    bit_pos_buffer[0][3] = 11 - bit_pos_buffer[2][0];
    bit_pos_buffer[0][2] = 11 - bit_pos_buffer[2][1];
    bit_pos_buffer[0][1] = 11 - bit_pos_buffer[2][2];
    bit_pos_buffer[0][0] = 11 - bit_pos_buffer[2][3];
    bit_pos_buffer[1][3] = bit_pos_buffer[3][0];
    bit_pos_buffer[1][2] = bit_pos_buffer[3][1];
    bit_pos_buffer[1][1] = bit_pos_buffer[3][2];
    bit_pos_buffer[1][0] = bit_pos_buffer[3][3];

    return bit_pos_rows; // 返回静态数组的指针
}

// 释放str2BitPos分配的内存
void free_bit_pos(uint8_t** bitPos)
{
    if (bitPos == NULL) {
        return;
    }
    
    // 释放每一行
    for (int i = 0; i < 4; i++) {
        if (bitPos[i] != NULL) {
            free(bitPos[i]);
        }
    }
    
    // 释放行指针数组
    free(bitPos);
}

// 注意：此函数返回的指针需要调用者负责释放
uint8_t** send_shift(uint8_t regDa[])
{
    uint8_t **SBP = str2BitPos_static((char *)regDa);     
    
    gpio_set_level(OE_GPIO,0);     
    esp_err_t ret = ESP_OK;
    
    for(int k = 0; k < 50; k++)
    {
        ret = gpio_set_level(DO_GPIO,0);
        ret = gpio_set_level(CLK_GPIO,0);        
        ret = gpio_set_level(OE_GPIO,0);        
        vTaskDelay(pdMS_TO_TICKS(1));
        
        ret = gpio_set_level(CLK_GPIO,1);
    } 
    for(int i = 0; i < 4; i++)
    {
        for(int j = 0; j < 11; j++)
        {
            uint8_t bitPos = 0;
            ret = ESP_OK;
            if(j == SBP[0][i] || j == SBP[1][i])
                bitPos = 1;
                
            ret = gpio_set_level(DO_GPIO,bitPos);
            ret = gpio_set_level(CLK_GPIO,0);
            ret = gpio_set_level(OE_GPIO,0);
            vTaskDelay(pdMS_TO_TICKS(1));
            
            ret = gpio_set_level(CLK_GPIO,1);
            vTaskDelay(pdMS_TO_TICKS(1));

            gpio_set_level(OE_GPIO,1); 
        }
    }

    vTaskDelay(pdMS_TO_TICKS(1));
    gpio_set_level(OE_GPIO,0);   
    return SBP;  // 调用者需要释放此内存
}

time_t dispTime(bool isLightSleep)
{
    static char last_display_content[10] = ""; // 保存上次显示内容
    
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);

    uint8_t batSoc;
    uint16_t batVoltage;
    int16_t batCurrent;
    bq27220_check_and_read_soc(&batSoc, &batVoltage, &batCurrent);
    last_battery_current = batCurrent;
    last_battery_percentage = batSoc;
    last_battery_voltage = batVoltage;

    // 读取传感器数据（如果传感器已初始化）
    if (true) {
        // 读取BMP580温度和气压数据
        esp_err_t bmp_ret = bmp580_read_temperature_pressure(&last_temperature_bmp, &last_pressure);
        if (bmp_ret == ESP_OK) {
            ESP_LOGI(RTCTAG, "BMP580 - 温度: %.2f°C, 气压: %.2f hPa", last_temperature_bmp, last_pressure);
        } else {
            ESP_LOGW(RTCTAG, "BMP580数据读取失败: %s", esp_err_to_name(bmp_ret));
        }
        
        // // 读取AHT20温度和湿度数据
        // esp_err_t aht_ret = aht20_read_temperature_humidity(&last_temperature_aht, &last_humidity);
        // if (aht_ret == ESP_OK) {
        //     ESP_LOGI(RTCTAG, "AHT20 - 温度: %.2f°C, 湿度: %.2f%%", last_temperature_aht, last_humidity);
        // } else {
        //     ESP_LOGW(RTCTAG, "AHT20数据读取失败: %s", esp_err_to_name(aht_ret));
        // }
    }
    
    if(batSoc<15 && batCurrent<0)
    {
        // 关闭高压和显示电路
        // gpio_set_od_level(HVEN_GPIO, 0);  // 关闭高压
        // gpio_set_level(OE_GPIO, 1);       // 关闭输出使能
        // gpio_set_level(DO_GPIO, 0);       // 数据引脚置低
        // gpio_set_level(CLK_GPIO, 0);      // 时钟引脚置低
        
        if(batSoc<5 && false)
        {
            ESP_LOGW(SLEEPTAG, "检测到低电压(%d%% < 10%%)，启动电池保护模式", batSoc);
            
            // 确保系统供电引脚保持高电平
            gpio_set_od_level(SYSPWEN_GPIO, 1);
            
            // 配置深度睡眠唤醒源
            esp_deep_sleep_enable_gpio_wakeup((1ULL << BUTTON_GPIO), ESP_GPIO_WAKEUP_GPIO_HIGH);
            
            ESP_LOGW(SLEEPTAG, "电池保护模式：系统将进入深度睡眠");
            ESP_LOGW(SLEEPTAG, "按下按键可唤醒系统，唤醒后请检查电池电压");
            
            // 延迟确保日志输出完成
            vTaskDelay(pdMS_TO_TICKS(2000));
            
            // 进入深度睡眠
            esp_deep_sleep_start();
            
            // 代码不会执行到这里
            return false;
        }
    }

    char nots[10] = {0}; // 初始化为全零，避免未初始化内存
    
    // 根据全局显示配置决定显示内容
    if (strcmp(current_display_type, "time") == 0) {
        // 显示时间
        if(!isLightSleep && timeinfo.tm_sec%2 == 0) {
            snprintf(nots, sizeof(nots), "%02d.%02d",timeinfo.tm_hour, timeinfo.tm_min);
        }  else {
            snprintf(nots, sizeof(nots), "%02d%02d",timeinfo.tm_hour, timeinfo.tm_min);            
        }
    } else if (strcmp(current_display_type, "bmp580_temperature") == 0) {
        // 显示BMP580温度，保留小数点
        snprintf(nots, sizeof(nots), "%.2f", last_temperature_bmp);
    } else if (strcmp(current_display_type, "bmp580_pressure") == 0) {
        // 显示BMP580气压，显示整数
        snprintf(nots, sizeof(nots), "%.0f", last_pressure);
    } else if (strcmp(current_display_type, "aht20_temperature") == 0) {
        // 显示AHT20温度，保留小数点
        snprintf(nots, sizeof(nots), "%.2f", last_temperature_aht);
    } else if (strcmp(current_display_type, "aht20_humidity") == 0) {
        // 显示AHT20湿度，保留小数点
        snprintf(nots, sizeof(nots), "%.2f", last_humidity);
    } else if (strcmp(current_display_type, "custom") == 0 && strlen(current_custom_content) > 0) {
        // 显示自定义内容，按原样传入
        // ESP_LOGI(RTCTAG, "自定义原始内容: %s", custom_content);
        // ESP_LOGI(RTCTAG, "自定义显示内容: %s", current_custom_content);
        // ESP_LOGI(RTCTAG, "自定义内容十六进制：%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
        //     current_custom_content[0], current_custom_content[1], current_custom_content[2], current_custom_content[3], 
        //     current_custom_content[4], current_custom_content[5], current_custom_content[6], current_custom_content[7], 
        //     current_custom_content[8], current_custom_content[9]);
        strncpy(nots, current_custom_content, sizeof(nots) - 1);
        nots[sizeof(nots) - 1] = '\0'; // 确保字符串结束
    } else {
        // 默认显示时间
        snprintf(nots, sizeof(nots), "%02d%02d",timeinfo.tm_hour, timeinfo.tm_min);
    }
    
    ESP_LOGI(RTCTAG, "传入显示内容: %s", nots);
    ESP_LOGI(RTCTAG, "内容十六进制：%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
        nots[0], nots[1], nots[2], nots[3], nots[4], nots[5], nots[6], nots[7], nots[8], nots[9]);

    // 检查显示内容是否与上次相同，如果相同则跳过刷新
    if (strcmp(nots, last_display_content) != 0) 
    {
        // 内容有变化，更新显示并保存当前内容
        strcpy(last_display_content, nots);
        uint8_t **prst = send_shift((uint8_t*)nots);
        prst = send_shift((uint8_t*)nots);
            
        //打印当前时间
        ESP_LOGI(RTCTAG, "Datetime: %04d-%02d-%02d %02d:%02d:%02d, %s, Stringbit %d,%d,%d,%d,%d,%d,%d,%d",
            timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
            timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec, nots,
            prst[0][0],prst[0][1],prst[0][2],prst[0][3],prst[1][0],prst[1][1],prst[1][2],prst[1][3]);
    }
    return now;
}

void set_rtc(uint16_t year, uint8_t mon, uint8_t tm_mday, uint8_t tm_hour, uint8_t tm_min, uint8_t tm_sec)
{      
    // 设置 RTC 时间
    struct tm timeinfo = {0};
    timeinfo.tm_year = year - 1900; // 年份从 1900 开始
    timeinfo.tm_mon = mon - 1;        // 月份从 0 开始
    timeinfo.tm_mday = tm_mday;           // 日期
    timeinfo.tm_hour = tm_hour;          // 小时
    timeinfo.tm_min = tm_min;            // 分钟
    timeinfo.tm_sec = tm_sec;            // 秒

    // 设置 RTC 时间
    time_t now = mktime(&timeinfo);
    struct timeval now_tv = {
        .tv_sec = now,
        .tv_usec = 0
    };
    settimeofday(&now_tv, NULL);
}

void setDeepSleeep()
{
    ESP_LOGW(BUTTONTAG, "电池电压过低(< 3.3V)，进入电池保护模式");
    
    // 1. 关闭高压和显示电路
    ESP_LOGI(BUTTONTAG, "关闭高压电路...");
    gpio_set_od_level(HVEN_GPIO, 0);  // 关闭高压
    stop_hv_runtime_counter(); // 关闭高压时停止运行时间计数
    
    // 2. 关闭显示输出
    ESP_LOGI(BUTTONTAG, "关闭显示电路...");
    gpio_set_level(OE_GPIO, 1);   // 关闭输出使能
    gpio_set_level(DO_GPIO, 0);   // 数据引脚置低
    gpio_set_level(CLK_GPIO, 0);  // 时钟引脚置低
    
    // 3. 停止所有定时器
    if (deep_sleep_timer != NULL) {
        ESP_LOGI(BUTTONTAG, "停止定时器...");
        esp_timer_stop(deep_sleep_timer);
        esp_timer_delete(deep_sleep_timer);
        deep_sleep_timer = NULL;
    }
    
    // // 4. 停止Web服务器
    // if (current_server_handle != NULL) {
    //     ESP_LOGI(BUTTONTAG, "停止Web服务器...");
    //     stop_webserver(current_server_handle);
    //     current_server_handle = NULL;
    //     vTaskDelay(pdMS_TO_TICKS(1000));
    // }
    
    // // 5. 关闭所有无线射频
    // ESP_LOGI(BUTTONTAG, "关闭WiFi和射频...");
    // wifi_mode_t mode;
    // if (esp_wifi_get_mode(&mode) == ESP_OK) {
    //     if (mode != WIFI_MODE_NULL) {
    //         esp_wifi_disconnect();
    //         vTaskDelay(pdMS_TO_TICKS(500));
    //         esp_wifi_set_mode(WIFI_MODE_NULL);
    //         vTaskDelay(pdMS_TO_TICKS(500));
    //         esp_wifi_stop();
    //         vTaskDelay(pdMS_TO_TICKS(1000));
    //         esp_wifi_deinit(); // 完全去初始化WiFi
    //         ESP_LOGI(BUTTONTAG, "WiFi已完全关闭");
    //     }
    // }
    
    // // 6. 保持最大CPU频率（移除频率调整）
    // ESP_LOGI(BUTTONTAG, "保持最大CPU频率...");
    
    // 7. 确保系统供电引脚保持高电平
    ESP_LOGI(BUTTONTAG, "确保系统供电引脚保持高电平...");
    gpio_set_od_level(SYSPWEN_GPIO, 1);
    
    // 8. 禁用不必要的外设
    ESP_LOGI(BUTTONTAG, "禁用不必要的外设...");
    
    // 9. 设置深度睡眠唤醒源
    esp_deep_sleep_enable_gpio_wakeup((1ULL << BUTTON_GPIO), ESP_GPIO_WAKEUP_GPIO_HIGH);
    
    ESP_LOGW(SLEEPTAG, "电池保护模式：系统将进入深度睡眠");
    ESP_LOGW(SLEEPTAG, "按下按键可唤醒系统，唤醒后请检查电池电压");
    
    // 延迟确保日志输出完成
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // 10. 进入深度睡眠
    esp_deep_sleep_start();
    
    // 注意：代码不会执行到这里，系统已进入深度睡眠
    

}

void nixie_ctr_init()
{    
    gpio_config_t io_conf;
    
    ESP_LOGI(IOTAG, "开始配置GPIO引脚...");
    
    // 初始化总运行时间系统
    ESP_LOGI(IOTAG, "初始化总运行时间系统...");
    esp_err_t nvs_ret = init_runtime_nvs();
    if (nvs_ret != ESP_OK) {
        ESP_LOGE(IOTAG, "总运行时间系统初始化失败: %s", esp_err_to_name(nvs_ret));
    } else {
        ESP_LOGI(IOTAG, "总运行时间系统初始化成功");
    }
    
    // 配置系统供电GPIO
    ESP_LOGI(IOTAG, "配置 SYSPWEN_GPIO (GPIO%d)...", SYSPWEN_GPIO);
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT_OD;
    io_conf.pin_bit_mask = (1ULL << SYSPWEN_GPIO);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
    gpio_set_level(SYSPWEN_GPIO, 1);
    ESP_LOGI(IOTAG, "SYSPWEN_GPIO配置完成");
    
    // 配置 OE_GPIO (GPIO8)
    ESP_LOGI(IOTAG, "配置 OE_GPIO (GPIO%d)...", OE_GPIO);
    io_conf.intr_type = GPIO_INTR_DISABLE; // 禁用中断
    io_conf.mode = GPIO_MODE_OUTPUT;       // 设置为输出模式
    io_conf.pin_bit_mask = (1ULL << OE_GPIO);  // 设置要配置的引脚
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE; // 禁用下拉
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;     // 禁用上拉
    gpio_config(&io_conf);

    // 配置 CLK_GPIO (GPIO7)
    ESP_LOGI(IOTAG, "配置 CLK_GPIO (GPIO%d)...", CLK_GPIO);
    io_conf.intr_type = GPIO_INTR_DISABLE; // 禁用中断
    io_conf.mode = GPIO_MODE_OUTPUT;       // 设置为输出模式
    io_conf.pin_bit_mask = (1ULL << CLK_GPIO);  // 设置要配置的引脚
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE; // 禁用下拉
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;     // 禁用上拉
    gpio_config(&io_conf);

    // 配置 DO_GPIO (GPIO6)
    ESP_LOGI(IOTAG, "配置 DO_GPIO (GPIO%d)...", DO_GPIO);
    io_conf.intr_type = GPIO_INTR_DISABLE; // 禁用中断
    io_conf.mode = GPIO_MODE_OUTPUT;       // 设置为输出模式
    io_conf.pin_bit_mask = (1ULL << DO_GPIO);  // 设置要配置的引脚
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE; // 禁用下拉
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;     // 禁用上拉
    gpio_config(&io_conf);

    // 配置 HVEN_GPIO (GPIO10) 170v和3.3V对外电源
    ESP_LOGI(IOTAG, "配置 HVEN_GPIO (GPIO%d)...", HVEN_GPIO);
    io_conf.intr_type = GPIO_INTR_DISABLE; // 禁用中断
    io_conf.mode = GPIO_MODE_OUTPUT;       // 设置为输出模式
    io_conf.pin_bit_mask = (1ULL << HVEN_GPIO);  // 设置要配置的引脚
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE; // 禁用下拉
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;     // 禁用上拉    
        gpio_config(&io_conf);    
    ESP_LOGI(IOTAG, "HVEN_GPIO配置完成");

    // 注意：SYSPWEN_GPIO已经在最开始配置，这里不需要重复配置
    
    // 配置按钮引脚GPIO5   MCU启动按钮
    // 注意：该按钮有外部弱下拉，按下时为高电平
    ESP_LOGI(IOTAG, "配置 BUTTON_GPIO (GPIO%d)...", BUTTON_GPIO);
    io_conf.intr_type = GPIO_INTR_ANYEDGE; // 检测上升和下降沿
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << BUTTON_GPIO);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE; // 禁用内部下拉（已有外部下拉）
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;     // 禁用内部上拉
    gpio_config(&io_conf);    
    ESP_LOGI(IOTAG, "BUTTON_GPIO配置完成");

    // 配置按钮引脚GPIO9   MCU唤醒按钮
    // 注意：该按钮没有上下拉直连io，按下时为低电平
    ESP_LOGI(IOTAG, "配置 AuxKey_GPIO (GPIO%d)...", AuxKey_GPIO);
    io_conf.intr_type = GPIO_INTR_ANYEDGE; // 检测上升和下降沿
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << AuxKey_GPIO);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE; // 禁用内部下拉（已有外部下拉）
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;     // 禁用内部上拉
    gpio_config(&io_conf);    
    ESP_LOGI(IOTAG, "AuxKey_GPIO配置完成");
    
    // 初始化I2C总线（GPIO0=SCL, GPIO1=SDA）
    ESP_LOGI(IOTAG, "初始化I2C总线用于电池管理和传感器...");
    i2c_master_init();

    // 初始化传感器
    ESP_LOGI(IOTAG, "初始化BMP580气压传感器...");
    esp_err_t bmp580_ret = bmp580_init();
    if (bmp580_ret == ESP_OK) {
        ESP_LOGI(IOTAG, "BMP580初始化成功");
    } else {
        ESP_LOGW(IOTAG, "BMP580初始化失败: %s", esp_err_to_name(bmp580_ret));
    }

    // ESP_LOGI(IOTAG, "初始化AHT20温湿度传感器...");
    // esp_err_t aht20_ret = aht20_init();
    // if (aht20_ret == ESP_OK) {
    //     ESP_LOGI(IOTAG, "AHT20初始化成功");
    // } else {
    //     ESP_LOGW(IOTAG, "AHT20初始化失败: %s", esp_err_to_name(aht20_ret));
    // }

    // 检测BQ27220YZFR电池管理芯片并读取完整状态
    while(1)
    {
        uint8_t batSoc = 0;
        uint16_t batVoltage = 0;
        int16_t batCurrent = 0;
        esp_err_t battery_ret = bq27220_check_and_read_soc(&batSoc, &batVoltage, &batCurrent);
        if (battery_ret == ESP_OK) {
            ESP_LOGI(IOTAG, "电池系统初始化成功 - 电量: %d%%, 电压: %dmV, 电流: %dmA", batSoc, batVoltage, batCurrent);
            
            // 配置BQ27220设计容量（假设使用1200mAh电池，可根据实际电池容量调整）
            esp_err_t design_cap_ret = bq27220_configure_design_capacity(1200);
            if (design_cap_ret == ESP_OK) {
                ESP_LOGI(IOTAG, "BQ27220设计容量配置成功");
            } else {
                ESP_LOGW(IOTAG, "BQ27220设计容量配置失败: %s", esp_err_to_name(design_cap_ret));
            }
            
            // 配置GPOUT低电量警报功能（阈值设为3%）
            esp_err_t gpout_ret = bq27220_configure_gpout_low_battery_alert(3);
            if (gpout_ret == ESP_OK) 
            {
                ESP_LOGI(IOTAG, "BQ27220 GPOUT低电量警报配置成功");
                break;
            } else {
                ESP_LOGW(IOTAG, "BQ27220 GPOUT低电量警报配置失败: %s", esp_err_to_name(gpout_ret));
            }
        } else {
            ESP_LOGW(IOTAG, "电池系统初始化失败，可能未连接电池或电池故障");
        }
    }
    gpio_set_od_level(HVEN_GPIO,1);


    vTaskDelay(pdMS_TO_TICKS(100)); // 延时 0.1 秒
    
    send_shift((uint8_t*)"89.64");

    // 初始化按键处理系统
    esp_err_t button_ret = button_init();
    if (button_ret != ESP_OK) 
    {
        ESP_LOGE(IOTAG, "按键系统初始化失败: %s", esp_err_to_name(button_ret));
    } 
    else 
    {
        ESP_LOGI(IOTAG, "按键系统初始化成功");        
    }

    // 初始化AuxKey按键处理系统
    esp_err_t auxkey_ret = auxkey_init();
    if (auxkey_ret != ESP_OK) 
    {
        ESP_LOGE(IOTAG, "AuxKey按键系统初始化失败: %s", esp_err_to_name(auxkey_ret));
    } 
    else 
    {
        ESP_LOGI(IOTAG, "AuxKey按键系统初始化成功");
    }

    deep_sleep_timer_init();
    
    //set_rtc(2025, 6, 26, 02, 40, 00);
    // 显示初始化完成后的运行时间和RTC信息
    ESP_LOGI(IOTAG, "==== 系统初始化完成 ====");
    vTaskDelay(pdMS_TO_TICKS(1000)); // 延迟1秒确保所有初始化完成
    print_runtime_and_rtc_info();
}

// 按键事件回调函数实现
static void button_single_click_callback(void)
{
    ESP_LOGI(BUTTONTAG, "GPIO%d 单击", BUTTON_GPIO);
    
    // 清除恢复出厂设置标志
    clear_factory_reset_flag();
    
    gpio_set_od_level(HVEN_GPIO,0);
    stop_hv_runtime_counter(); // 关闭高压时停止运行时间计数
    // TODO: 在这里添加单击事件的实际处理逻辑
}

static void button_double_click_callback(void)
{
    ESP_LOGI(BUTTONTAG, "GPIO%d 双击", BUTTON_GPIO);
    
    // 清除恢复出厂设置标志
    clear_factory_reset_flag();
    
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // 检测电池电压
    // int adc_value = 0;
    // float voltage = 0.0f;
    
    // if (adc1_handle != NULL) {
    //     esp_err_t adc_err = adc_oneshot_read(adc1_handle, ADC_CHANNEL_3, &adc_value);
    //     if (adc_err == ESP_OK) {
    //         voltage = (adc_value * 3.1f * BatADCDiv) / 4095.0f;
    //         ESP_LOGI(BUTTONTAG, "当前电池电压: %.2fV", voltage);
    //     } else {
    //         ESP_LOGW(BUTTONTAG, "ADC读取失败: %s", esp_err_to_name(adc_err));
    //     }
    // } else {
    //     ESP_LOGW(BUTTONTAG, "ADC未初始化");
    // }
    
    // // 电压低于3.3V时进入电池保护模式
    // if (voltage < 3.3f) 
    //     setDeepSleeep();
    // else
    {
        ESP_LOGI(IOTAG, "开启高压");
        esp_err_t hven_ret = gpio_set_od_level(HVEN_GPIO,1);
        if (hven_ret != ESP_OK) 
            ESP_LOGE(IOTAG, "设置HVEN_GPIO失败: %s", esp_err_to_name(hven_ret));
        else
            start_hv_runtime_counter(); // 开启高压成功后开始运行时间计数
        vTaskDelay(pdMS_TO_TICKS(100));
        dispTime(true);
    }
    
    // TODO: 在这里添加双击事件的实际处理逻辑
}

static void button_long_press_callback(void)
{
    ESP_LOGI(BUTTONTAG, "长按持续时间: >= %d ms,准备关机...", LONG_PRESS_TIME_MS);
    
    // 检查是否需要恢复出厂设置
    if (factory_reset_pending) {
        ESP_LOGW(BUTTONTAG, "检测到恢复出厂设置标志，执行恢复操作...");
        
        // 显示警告信息
        ESP_LOGW(BUTTONTAG, "==>> 正在执行恢复出厂设置 <<==");
        ESP_LOGW(BUTTONTAG, "请勿断电，恢复过程中...");
        
        // 执行恢复出厂设置
        esp_err_t reset_result = perform_factory_reset();
        if (reset_result == ESP_OK) {
            ESP_LOGW(BUTTONTAG, "✓ 恢复出厂设置完成");
        } else {
            ESP_LOGE(BUTTONTAG, "✗ 恢复出厂设置失败");
        }
        
        // 清除标志
        factory_reset_pending = false;
        
        // 延迟确保操作完成
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    
    ESP_LOGW(BUTTONTAG, "系统即将关机...");
    vTaskDelay(pdMS_TO_TICKS(1000));
    gpio_set_od_level(SYSPWEN_GPIO,0);
}

// 定时器回调函数
static void button_timer_callback(void* arg)
{
    button_event_t event = (button_event_t)(int)arg;
    
    if (event == BUTTON_EVENT_SINGLE_CLICK && click_count == 1) {
        // 只有在确实是单击（不是双击的一部分）时才触发
        button_single_click_callback();
        click_count = 0;
    } else if (event == BUTTON_EVENT_LONG_PRESS && button_pressed) {
        button_long_press_callback();
        // 标记已经处理了长按，避免触发单击
        click_count = -1;  // 使用特殊值标记长按已处理
    }
}

// GPIO中断处理函数
static void IRAM_ATTR button_isr_handler(void* arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    if (button_task_handle != NULL) {
        xTaskNotifyFromISR(button_task_handle, 1, eSetValueWithoutOverwrite, &xHigherPriorityTaskWoken);
    }
    
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

// 按键处理任务
static void button_task(void* arg)
{
    uint32_t notify_value;
    int64_t current_time;
    
    // ESP_LOGI(BUTTONTAG, "按键处理任务启动");
    // ESP_LOGI(BUTTONTAG, "按键参数 - 消抖: %dms, 长按: %dms, 双击窗口: %dms", 
            //  DEBOUNCE_TIME_MS, LONG_PRESS_TIME_MS, DOUBLE_CLICK_TIME_MS);
    
    // 显示初始GPIO状态
    // int initial_state = gpio_get_level(BUTTON_GPIO);
    // ESP_LOGI(BUTTONTAG, "初始GPIO%d状态: %d (%s)", BUTTON_GPIO, initial_state,
    //          initial_state ? "按下" : "未按下");
    
    while (1) {
        // 等待中断通知（阻塞等待，低功耗）
        if (xTaskNotifyWait(0, ULONG_MAX, &notify_value, portMAX_DELAY) == pdTRUE) {
            current_time = esp_timer_get_time() / 1000;  // 转换为毫秒
            
            // 消抖处理
            if ((current_time - last_press_time) < DEBOUNCE_TIME_MS && 
                (current_time - last_release_time) < DEBOUNCE_TIME_MS) {
                continue;
            }
            
            // 读取当前按键状态
            int button_level = gpio_get_level(BUTTON_GPIO);
            
            // ESP_LOGI(BUTTONTAG, "按键中断 - GPIO%d电平: %d, 时间: %lld ms", 
                    //  BUTTON_GPIO, button_level, current_time);
            
            if (button_level == 1) {  // 按键按下（高电平）
                // ESP_LOGI(BUTTONTAG, "检测到按键按下");
                last_press_time = current_time;
                button_pressed = true;
                
                // 创建长按检测定时器
                esp_timer_create_args_t timer_args = {
                    .callback = &button_timer_callback,
                    .arg = (void*)BUTTON_EVENT_LONG_PRESS,
                    .name = "long_press_timer"
                };
                
                if (button_timer != NULL) {
                    esp_timer_stop(button_timer);
                    esp_timer_delete(button_timer);
                }
                
                esp_timer_create(&timer_args, &button_timer);
                esp_timer_start_once(button_timer, LONG_PRESS_TIME_MS * 1000);
                
            } else {  // 按键释放（低电平）
                // ESP_LOGI(BUTTONTAG, "检测到按键释放");
                
                if (button_pressed) {
                    button_pressed = false;
                    
                    // 停止长按定时器
                    if (button_timer != NULL) {
                        esp_timer_stop(button_timer);
                        esp_timer_delete(button_timer);
                        button_timer = NULL;
                    }
                    
                    // 计算按压时间
                    int64_t press_duration = current_time - last_press_time;
                    // ESP_LOGI(BUTTONTAG, "按键按压时长: %lld ms", press_duration);
                    
                    // 检查是否已经处理了长按
                    if (click_count == -1) {
                        // 长按已处理，重置状态，不触发单击
                        click_count = 0;
                        // ESP_LOGI(BUTTONTAG, "长按事件已处理，跳过单击检测");
                    }
                    // 如果按压时间小于长按阈值，检查单击或双击
                    else if (press_duration < LONG_PRESS_TIME_MS) {
                        // 检查是否在双击时间窗口内
                        if ((current_time - last_release_time) < DOUBLE_CLICK_TIME_MS && click_count == 1) {
                            // 停止单击定时器，避免触发单击事件
                            if (button_timer != NULL) {
                                esp_timer_stop(button_timer);
                                esp_timer_delete(button_timer);
                                button_timer = NULL;
                            }
                            
                            click_count = 2;
                            button_double_click_callback();
                            click_count = 0;
                        } else {
                            click_count = 1;
                            
                            // 创建单击检测定时器
                            esp_timer_create_args_t timer_args = {
                                .callback = &button_timer_callback,
                                .arg = (void*)BUTTON_EVENT_SINGLE_CLICK,
                                .name = "single_click_timer"
                            };
                            
                            if (button_timer != NULL) {
                                esp_timer_stop(button_timer);
                                esp_timer_delete(button_timer);
                            }
                            
                            esp_timer_create(&timer_args, &button_timer);
                            esp_timer_start_once(button_timer, DOUBLE_CLICK_TIME_MS * 1000);
                        }
                        
                        last_release_time = current_time;
                    }
                }
            }
        }
    }
}

// 初始化按键处理系统
esp_err_t button_init(void)
{
    esp_err_t ret = ESP_OK;
    
    // 创建按键处理任务
    BaseType_t task_ret = xTaskCreate(button_task, "button_task", 
                                      2048, NULL, 10, &button_task_handle);
    if (task_ret != pdPASS) {
        ESP_LOGE(BUTTONTAG, "创建按键任务失败");
        return ESP_FAIL;
    }
    // ESP_LOGI(BUTTONTAG, "✓ 按键处理任务创建成功");
    
    // 安装GPIO中断服务
    ret = gpio_install_isr_service(0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        // ESP_LOGE(BUTTONTAG, "安装GPIO中断服务失败: %s", esp_err_to_name(ret));
        return ret;
    }
    // ESP_LOGI(BUTTONTAG, "✓ GPIO中断服务安装成功");
    
    // 添加中断处理函数
    ret = gpio_isr_handler_add(BUTTON_GPIO, button_isr_handler, NULL);
    if (ret != ESP_OK) {
        // ESP_LOGE(BUTTONTAG, "添加中断处理函数失败: %s", esp_err_to_name(ret));
        return ret;
    }
    // ESP_LOGI(BUTTONTAG, "✓ 按键中断处理函数添加成功");
    
    // 读取初始GPIO状态
    // int gpio_level = gpio_get_level(BUTTON_GPIO);
    // ESP_LOGI(BUTTONTAG, "当前GPIO%d状态: %s", BUTTON_GPIO, 
    //          gpio_level ? "高电平(按下)" : "低电平(未按下)");
    
    
    return ESP_OK;
}

// =========================== AuxKey按键系统 ===========================
// AuxKey双击事件回调函数实现

// AuxKey双击事件回调函数实现
static void auxkey_double_click_callback(void)
{    
    ESP_LOGI(BUTTONTAG, "GPIO%d 双击", AuxKey_GPIO);
    
    // 清除恢复出厂设置标志
    clear_factory_reset_flag();
    
    wakeup_mode_t old_mode = current_wakeup_mode;
    
    // 在两种模式间切换
    current_wakeup_mode = (current_wakeup_mode == WAKEUP_MODE_SECOND) ? 
                         WAKEUP_MODE_MINUTE : WAKEUP_MODE_SECOND;
    
    ESP_LOGI(BUTTONTAG, "从 %s 切换到: %s", 
             wakeup_mode_names[old_mode], wakeup_mode_names[current_wakeup_mode]);
    
    // 停止当前的定时器
    if (deep_sleep_timer != NULL) {
        esp_timer_stop(deep_sleep_timer);
        esp_timer_delete(deep_sleep_timer);
        deep_sleep_timer = NULL;
        ESP_LOGI(BUTTONTAG, "✓ 定时器已停止并删除");
    } else {
        ESP_LOGI(BUTTONTAG, "当前没有运行的定时器");
    }
        
    // 直接重新初始化定时器系统（不需要网络服务重新初始化的情况）
    ESP_LOGI(SLEEPTAG, "模式切换: %s -> %s", wakeup_mode_names[old_mode], wakeup_mode_names[current_wakeup_mode]);

    deep_sleep_timer_init();
    
    ESP_LOGI(BUTTONTAG, "==>> AuxKey双击事件处理完成 <<==");
}

// AuxKey长按事件回调函数（恢复出厂设置）
static void auxkey_factory_reset_long_press_callback(void)
{
    ESP_LOGW(BUTTONTAG, "GPIO%d 长按 >= %d ms - 恢复出厂设置模式", 
             AuxKey_GPIO, FACTORY_RESET_LONG_PRESS_MS);
    
    if (!factory_reset_pending) {
        factory_reset_pending = true;
        ESP_LOGW(BUTTONTAG, "==>> 恢复出厂设置模式已激活 <<==");
        ESP_LOGW(BUTTONTAG, "警告：下次关机时将恢复出厂设置");
        ESP_LOGW(BUTTONTAG, "取消方法：单击或双击任意按键");
        ESP_LOGW(BUTTONTAG, "确认方法：长按主按钮关机");
        
        // 可以在这里添加视觉或声音提示，比如闪烁显示
        // 临时显示警告信息
        send_shift((uint8_t*)"FA.CT");
        vTaskDelay(pdMS_TO_TICKS(2000));
        send_shift((uint8_t*)"RE.SE");
        vTaskDelay(pdMS_TO_TICKS(2000));
        send_shift((uint8_t*)"8.8.8.8");
        vTaskDelay(pdMS_TO_TICKS(1000));
        
        // 显示正常时间
        dispTime(true);
    } else {
        ESP_LOGW(BUTTONTAG, "恢复出厂设置模式已经激活");
    }
}


// AuxKey定时器回调函数
static void auxkey_timer_callback(void* arg)
{
    button_event_t event = (button_event_t)(int)arg;
    
    if (event == BUTTON_EVENT_DOUBLE_CLICK && auxkey_click_count == 2) {
        auxkey_double_click_callback();
        auxkey_click_count = 0;
    } else if (event == BUTTON_EVENT_FACTORY_RESET_LONG_PRESS && auxkey_pressed) {
        auxkey_factory_reset_long_press_callback();
        // 标记已经处理了长按，避免触发其他事件
        auxkey_click_count = -1;  // 使用特殊值标记长按已处理
    }
}

// AuxKey GPIO中断处理函数
static void IRAM_ATTR auxkey_isr_handler(void* arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    if (auxkey_task_handle != NULL) {
        xTaskNotifyFromISR(auxkey_task_handle, 1, eSetValueWithoutOverwrite, &xHigherPriorityTaskWoken);
    }
    
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

// AuxKey按键处理任务
static void auxkey_task(void* arg)
{
    uint32_t notify_value;
    int64_t current_time;
    
    while (1) {
        // 等待中断通知（阻塞等待，低功耗）
        if (xTaskNotifyWait(0, ULONG_MAX, &notify_value, portMAX_DELAY) == pdTRUE) {
            current_time = esp_timer_get_time() / 1000;  // 转换为毫秒
            
            // 消抖处理
            if ((current_time - auxkey_last_press_time) < DEBOUNCE_TIME_MS && 
                (current_time - auxkey_last_release_time) < DEBOUNCE_TIME_MS) {
                continue;
            }
            
            // 读取当前按键状态 (注意：AuxKey按下时为低电平)
            int auxkey_level = gpio_get_level(AuxKey_GPIO);
            
            if (auxkey_level == 0) {  // 按键按下（低电平）
                auxkey_last_press_time = current_time;
                auxkey_pressed = true;
                
                // 创建恢复出厂设置长按检测定时器
                esp_timer_create_args_t timer_args = {
                    .callback = &auxkey_timer_callback,
                    .arg = (void*)BUTTON_EVENT_FACTORY_RESET_LONG_PRESS,
                    .name = "auxkey_factory_reset_timer"
                };
                
                if (auxkey_timer != NULL) {
                    esp_timer_stop(auxkey_timer);
                    esp_timer_delete(auxkey_timer);
                }
                
                esp_timer_create(&timer_args, &auxkey_timer);
                esp_timer_start_once(auxkey_timer, FACTORY_RESET_LONG_PRESS_MS * 1000);
                
            } else {  // 按键释放（高电平）
                if (auxkey_pressed) {
                    auxkey_pressed = false;
                    
                    // 停止长按定时器
                    if (auxkey_timer != NULL) {
                        esp_timer_stop(auxkey_timer);
                        esp_timer_delete(auxkey_timer);
                        auxkey_timer = NULL;
                    }
                    
                    // 计算按压时间
                    int64_t press_duration = current_time - auxkey_last_press_time;
                    
                    // 检查是否已经处理了长按
                    if (auxkey_click_count == -1) {
                        // 长按已处理，重置状态，不触发其他事件
                        auxkey_click_count = 0;
                    }
                    // 只处理双击事件，忽略短按长按
                    else if (press_duration < FACTORY_RESET_LONG_PRESS_MS) {
                        // 检查是否在双击时间窗口内
                        if ((current_time - auxkey_last_release_time) < DOUBLE_CLICK_TIME_MS && auxkey_click_count == 1) {
                            // 停止之前的定时器
                            if (auxkey_timer != NULL) {
                                esp_timer_stop(auxkey_timer);
                                esp_timer_delete(auxkey_timer);
                                auxkey_timer = NULL;
                            }
                            
                            auxkey_click_count = 2;
                            auxkey_double_click_callback();
                            auxkey_click_count = 0;
                        } else {
                            auxkey_click_count = 1;
                            
                            // 创建双击检测定时器
                            esp_timer_create_args_t timer_args = {
                                .callback = &auxkey_timer_callback,
                                .arg = (void*)BUTTON_EVENT_DOUBLE_CLICK,
                                .name = "auxkey_double_click_timer"
                            };
                            
                            if (auxkey_timer != NULL) {
                                esp_timer_stop(auxkey_timer);
                                esp_timer_delete(auxkey_timer);
                            }
                            
                            esp_timer_create(&timer_args, &auxkey_timer);
                            esp_timer_start_once(auxkey_timer, DOUBLE_CLICK_TIME_MS * 1000);
                        }
                        
                        auxkey_last_release_time = current_time;
                    }
                }
            }
        }
    }
}

// 初始化AuxKey按键处理系统
esp_err_t auxkey_init(void)
{
    esp_err_t ret = ESP_OK;
    
    // 创建AuxKey按键处理任务（增加堆栈大小避免溢出）
    BaseType_t task_ret = xTaskCreate(auxkey_task, "auxkey_task", 
                                      4096, NULL, 10, &auxkey_task_handle);
    if (task_ret != pdPASS) {
        ESP_LOGE(BUTTONTAG, "创建AuxKey按键任务失败");
        return ESP_FAIL;
    }
    
    // 添加中断处理函数
    ret = gpio_isr_handler_add(AuxKey_GPIO, auxkey_isr_handler, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(BUTTONTAG, "添加AuxKey中断处理函数失败: %s", esp_err_to_name(ret));
        return ret;
    }
    
    return ESP_OK;
}

// =========================== 电源管理系统 ===========================

// 检查唤醒原因
void check_wakeup_reason(void)
{
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    
    switch(wakeup_reason) {
        case ESP_SLEEP_WAKEUP_TIMER:
            ESP_LOGI(SLEEPTAG, "从定时器唤醒（系统重启）");
            break;
        case ESP_SLEEP_WAKEUP_EXT0:
            ESP_LOGI(SLEEPTAG, "从外部中断唤醒（按键触发）");
            break;
        case ESP_SLEEP_WAKEUP_EXT1:
            ESP_LOGI(SLEEPTAG, "从外部唤醒源唤醒");
            break;
        case ESP_SLEEP_WAKEUP_TOUCHPAD:
            ESP_LOGI(SLEEPTAG, "从触摸板唤醒");
            break;
        case ESP_SLEEP_WAKEUP_ULP:
            ESP_LOGI(SLEEPTAG, "从ULP唤醒");
            break;
        default:
            ESP_LOGI(SLEEPTAG, "系统首次启动，原因: %d", wakeup_reason);
            break;
    }
}

// 电压保护检查函数
static bool check_battery_voltage_protection(void)
{
    // float voltage = 0.0f;
    uint8_t batSoc;
    uint16_t batVoltage;
    int16_t batCurrent;
    bq27220_check_and_read_soc(&batSoc, &batVoltage, &batCurrent);
    
    last_battery_current = batCurrent;
    last_battery_percentage = batSoc;
    last_battery_voltage = batVoltage;

    ESP_LOGI(IOTAG, "电池状态 - 电量: %d%%, 电压: %dmV, 电流: %dmA", batSoc, batVoltage, batCurrent);
    ESP_LOGW(SLEEPTAG, "检测到低电压(%d%% < 50%%)，系统正常启动", batSoc);
    return true;
}

// 秒跳变定时器回调函数（保持所有服务）
static void second_timer_callback(void* arg)
{    
    // 在秒跳变模式下自行管理运行时间计数
    // 每分钟（当秒数为0时）更新并保存运行时间，避免过于频繁的NVS写入
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    
    if (timeinfo.tm_sec == 0) {
        // 每分钟更新并保存运行时间
        update_runtime_counter();
        save_runtime_to_nvs();
        ESP_LOGI(NVSTAG, "秒跳变模式 - 数据已分别更新（%02d:%02d）", 
                 timeinfo.tm_hour, timeinfo.tm_min);
    }
    
    // 根据设置的刷新率计算下次触发间隔
    uint64_t next_interval_us = 1000000; // 默认1秒（1Hz）
    
    if (strcmp(current_refresh_rate, "2") == 0) {
        next_interval_us = 500000; // 0.5秒（2Hz）
    } else if (strcmp(current_refresh_rate, "5") == 0) {
        next_interval_us = 200000; // 0.2秒（5Hz）
    } else if (strcmp(current_refresh_rate, "10") == 0) {
        next_interval_us = 100000; // 0.1秒（10Hz）
    } else {
        // 默认或"1"：1Hz
        next_interval_us = 1000000; // 1秒
    }
    
    // 重新启动定时器
    if (deep_sleep_timer != NULL) {
        esp_timer_start_once(deep_sleep_timer, next_interval_us);
    }

    // 显示时间（秒跳变模式，保持服务运行）
    dispTime(false);
}

// 分跳变定时器回调函数（关闭网络服务）
static void minute_timer_callback(void* arg)
{    
    // 在分跳变模式下同步更新运行时间
    update_runtime_counter();
    save_runtime_to_nvs();
    ESP_LOGI(NVSTAG, "分跳变模式 - 数据已分别更新");
    
    // 计算到下一分钟的剩余秒数
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    
    int seconds_to_next_minute = 60 - timeinfo.tm_sec;
    if (seconds_to_next_minute == 60) seconds_to_next_minute = 60;
    
    ESP_LOGI(SLEEPTAG, "下次显示时间: %d秒后", seconds_to_next_minute);
    
    // 重新启动定时器到下一分钟
    if (deep_sleep_timer != NULL) {
        esp_timer_start_once(deep_sleep_timer, seconds_to_next_minute * 1000000); // 微秒
    }

    // 显示时间
    ESP_LOGI(SLEEPTAG, "分跳变定时器触发 - 显示时间并更新运行时间");    
    dispTime(true);
}

// 初始化电源管理定时器系统
esp_err_t deep_sleep_timer_init(void)
{
    ESP_LOGI(SLEEPTAG, "==>> 电源管理定时器系统初始化开始 <<==");
    
    // 检查电池电压，如果过低则进入保护模式
    if (!check_battery_voltage_protection()) 
    {
        // 函数内部已经进入深度睡眠，不会执行到这里
        return ESP_FAIL;
    }
    
    // 首先确保系统供电正常（特别是从Deep Sleep唤醒后）
    gpio_set_od_level(SYSPWEN_GPIO, 1);
    ESP_LOGI(SLEEPTAG, "确保系统供电正常 - SYSPWEN_GPIO拉高");
    
    // 检查唤醒原因
    // esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    check_wakeup_reason();
    
    // 加载刷新率设置
    load_refresh_rate_from_nvs();
    
    ESP_LOGI(SLEEPTAG, "当前模式: %s", wakeup_mode_names[current_wakeup_mode]);
    ESP_LOGI(SLEEPTAG, "当前刷新率: %s", current_refresh_rate);
    
    // 创建定时器
    esp_timer_create_args_t timer_args = {
        .arg = NULL,
        .name = "wakeup_timer"
    };
    
    if (current_wakeup_mode == WAKEUP_MODE_SECOND) 
    {
        // 秒跳变模式：保持所有服务运行
        ESP_LOGI(SLEEPTAG, "-->> 配置秒跳变模式 <<--");
        timer_args.callback = &second_timer_callback;
        
        esp_err_t ret = esp_timer_create(&timer_args, &deep_sleep_timer);
        if (ret != ESP_OK) {
            ESP_LOGE(SLEEPTAG, "创建秒跳变定时器失败: %s", esp_err_to_name(ret));
            return ret;
        }
        
        // 根据设置的刷新率计算初始触发间隔
        uint64_t initial_interval_us = 1000000; // 默认1秒（1Hz）
        
        if (strcmp(current_refresh_rate, "2") == 0) {
            initial_interval_us = 500000; // 0.5秒（2Hz）
        } else if (strcmp(current_refresh_rate, "5") == 0) {
            initial_interval_us = 200000; // 0.2秒（5Hz）
        } else if (strcmp(current_refresh_rate, "10") == 0) {
            initial_interval_us = 100000; // 0.1秒（10Hz）
        } else {
            // 默认或"1"：1Hz
            initial_interval_us = 1000000; // 1秒
        }
        
        ESP_LOGI(SLEEPTAG, "启动秒跳变定时器，刷新率%s，%llu微秒后触发", current_refresh_rate, initial_interval_us);
        
        // 启动秒跳变定时器
        ret = esp_timer_start_once(deep_sleep_timer, initial_interval_us);
        if (ret != ESP_OK) 
        {
            ESP_LOGE(SLEEPTAG, "启动秒跳变定时器失败: %s", esp_err_to_name(ret));
            esp_timer_delete(deep_sleep_timer);
            deep_sleep_timer = NULL;
            return ret;
        }
    } 
    else 
    {
        // 分跳变模式：关闭网络服务，节省功耗
        ESP_LOGI(SLEEPTAG, "-->> 配置分跳变模式 <<--");
        timer_args.callback = &minute_timer_callback;
        
        esp_err_t ret = esp_timer_create(&timer_args, &deep_sleep_timer);
        if (ret != ESP_OK) {
            ESP_LOGE(SLEEPTAG, "创建分跳变定时器失败: %s", esp_err_to_name(ret));
            return ret;
        }
        
        // 计算到下一分钟的剩余秒数
        time_t now;
        struct tm timeinfo;
        time(&now);
        localtime_r(&now, &timeinfo);
        
        int seconds_to_next_minute = 60 - timeinfo.tm_sec;
        if (seconds_to_next_minute == 60) seconds_to_next_minute = 60;
        
        ESP_LOGI(SLEEPTAG, "当前时间: %02d:%02d:%02d", 
                 timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
        ESP_LOGI(SLEEPTAG, "将在%d秒后显示下一分钟时间", seconds_to_next_minute);
        
        // 启动分跳变定时器
        ret = esp_timer_start_once(deep_sleep_timer, seconds_to_next_minute * 1000000);
        if (ret != ESP_OK) {
            ESP_LOGE(SLEEPTAG, "启动分跳变定时器失败: %s", esp_err_to_name(ret));
            esp_timer_delete(deep_sleep_timer);
            deep_sleep_timer = NULL;
            return ret;
        }
    }
    
    ESP_LOGI(SLEEPTAG, "==>> 电源管理定时器系统初始化完成 <<==");
    return ESP_OK;
}
