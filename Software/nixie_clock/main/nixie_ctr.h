#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "esp_err.h"
#include "hal/gpio_types.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include <esp_adc/adc_oneshot.h>
#include <time.h>
#include "esp_timer.h"
#include "esp_sleep.h"
#include "esp_pm.h"

#include "driver/rtc_io.h"
#include "nvs_flash.h"
#include <sys/time.h>

// 添加WiFi和Web服务器支持
// #include "wifi_manager.h"
// #include "web_server.h"

#define SYSPWEN_GPIO GPIO_NUM_4
#define BUTTON_GPIO GPIO_NUM_5
#define DO_GPIO GPIO_NUM_6
#define CLK_GPIO GPIO_NUM_7
#define OE_GPIO GPIO_NUM_8
#define AuxKey_GPIO GPIO_NUM_9
#define HVEN_GPIO GPIO_NUM_10

#define HVEN()     gpio_set_od_level(HVEN_GPIO,0)
#define HVDIS()     gpio_set_od_level(HVEN_GPIO,1)

#define BatADCDiv 2.0f

void nixie_ctr_init();
esp_err_t button_init();
esp_err_t auxkey_init();
esp_err_t gpio_set_od_level(gpio_num_t optIO, uint32_t level);
uint8_t** send_shift(uint8_t regDa[]); // 使用静态缓冲区，无需释放返回值
// void free_bit_pos(uint8_t** bitPos); // 释放str2BitPos分配的内存（已废弃）
void set_rtc(uint16_t year, uint8_t mon, uint8_t tm_mday, uint8_t tm_hour, uint8_t tm_min, uint8_t tm_sec);
time_t dispTime(bool isLightSleep);

// I2C相关函数声明
void i2c_master_init(void);

// 电池测试相关函数声明
void start_battery_test(void);

// 电源管理相关函数声明
esp_err_t deep_sleep_timer_init(void);
void check_wakeup_reason(void);
// void set_webserver_handle(httpd_handle_t server);
void setDeepSleeep();

// 电池电压读取相关函数声明
float get_battery_voltage(void);
int get_battery_percentage(void);
float get_battery_current(void);

// 获取传感器数据
float get_bmp580_temperature(void);
float get_bmp580_pressure(void);
float get_aht20_temperature(void);
float get_aht20_humidity(void);
bool are_sensors_initialized(void);

// 总运行时间相关函数声明
void get_total_runtime_string(char* buffer, size_t buffer_size);
uint32_t get_total_runtime_minutes(void);

// RTC时间保存相关函数声明
void get_last_saved_rtc_info(char* buffer, size_t buffer_size);
time_t get_last_saved_rtc_timestamp(void);
void print_runtime_and_rtc_info(void);  // 测试函数：显示运行时间和RTC信息

// 恢复出厂设置相关函数
bool is_factory_reset_pending(void);
void cancel_factory_reset(void);
esp_err_t perform_factory_reset(void);

// 唤醒模式相关函数
int get_current_wakeup_mode(void);
void set_wakeup_mode(int mode);
void load_refresh_rate_from_nvs(void);

// 显示配置相关函数
void set_display_config(const char* display_type, const char* custom_content);
void get_display_config(char* display_type, size_t display_type_size, char* custom_content, size_t custom_content_size);

// BMP580气压计相关定义和函数声明
#define BMP580_ADDR                 0x46   // BMP580 7位地址I2C地址
#define BMP580_CHIP_ID_REG          0x01   // 芯片ID寄存器
#define BMP580_CHIP_ID              0x50   // BMP580芯片ID
#define BMP580_STATUS_REG           0x28   // 状态寄存器
#define BMP580_TEMP_DATA_REG        0x1D   // 温度数据寄存器
#define BMP580_PRESS_DATA_REG       0x20   // 气压数据寄存器
#define BMP580_PWR_CTRL_REG         0x36   // 电源控制寄存器
#define BMP580_OSR_CONFIG_REG       0x37   // 过采样配置寄存器
#define BMP580_ODR_CONFIG_REG       0x38   // 输出数据率配置寄存器
#define BMP580_DSP_CONFIG_REG       0x30   // DSP配置寄存器
#define BMP580_INT_CONFIG_REG       0x14   // 中断配置寄存器

esp_err_t bmp580_init(void);
esp_err_t bmp580_read_temperature_pressure(float *temperature, float *pressure);

// AHT20温湿度传感器相关定义和函数声明
#define AHT20_ADDR                  0x70   // AHT20 7位I2C地址
extern uint8_t aht20_working_addr;             // AHT20实际工作地址
#define AHT20_INIT_CMD              0xBE   // 初始化命令
#define AHT20_TRIGGER_CMD           0xAC   // 触发测量命令
#define AHT20_SOFT_RESET_CMD        0xBA   // 软复位命令
#define AHT20_STATUS_BUSY           0x80   // 忙状态位
#define AHT20_STATUS_CALIBRATED     0x08   // 校准状态位

// I2C扫描函数声明
void i2c_scan(void);

// AHT20温湿度传感器函数声明
esp_err_t aht20_init(void);
esp_err_t aht20_read_temperature_humidity(float *temperature, float *humidity);

// BQ27220电池管理芯片相关定义和函数声明
#define BQ27220_ADDR                0x55   // BQ27220YZFR 7位地址
#define BQ27220_CONTROL_REG         0x00   // Control寄存器
#define BQ27220_CONTROL_STATUS_REG  0x00   // Control Status寄存器
#define BQ27220_SOC_SET_THRESHOLD   0x51   // SOC Set Threshold寄存器
#define BQ27220_SOC_CLR_THRESHOLD   0x52   // SOC Clear Threshold寄存器
#define BQ27220_OPCONFIG_REG        0x3A   // OpConfig寄存器
#define BQ27220_UNSEAL_KEY1         0x8000 // 解锁密钥1
#define BQ27220_UNSEAL_KEY2         0x8000 // 解锁密钥2
#define BQ27220_FULL_ACCESS_KEY1    0xFFFF // 完全访问密钥1
#define BQ27220_FULL_ACCESS_KEY2    0xFFFF // 完全访问密钥2

// BQ27220电池管理芯片函数声明
esp_err_t bq27220_check_presence(void);
esp_err_t bq27220_check_and_read_soc(uint8_t *soc, uint16_t *voltage, int16_t *current);
esp_err_t bq27220_get_battery_info(uint8_t *soc, uint16_t *voltage, int16_t *current);
esp_err_t bq27220_configure_gpout_low_battery_alert(uint8_t soc_threshold);
esp_err_t bq27220_configure_design_capacity(uint16_t design_capacity_mah);