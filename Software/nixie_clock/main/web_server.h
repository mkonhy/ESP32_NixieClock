#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include <esp_http_server.h>
#include <esp_err.h>

// Web服务器相关常量
#define WEB_SERVER_PORT 80
#define WEB_SERVER_MAX_URI_LEN 512
#define WEB_SERVER_MAX_BODY_SIZE 1024

// 系统信息结构体
typedef struct {
    // 时间信息
    char current_rtc_time[32];        // 当前RTC时间
    char last_calibration_time[64];   // 上次校准时间
    char ntp_server[64];              // NTP服务器地址
    char timezone_name[32];           // 当前时区名称
    int32_t timezone_offset_sec;      // 时区偏移（秒）
    
    // 系统状态
    int battery_percentage;           // 电量百分比
    float battery_voltage;            // 电池电压
    int battery_current;              // 电池电流(mA)
    int cpu_usage;                    // CPU使用率
    int memory_usage;                 // 内存使用率
    int flash_usage;                  // Flash使用率
    
    // 网络信息
    bool wifi_enabled;                // WiFi客户端是否启用
    bool ap_enabled;                  // AP热点是否启用
    char ap_ssid[32];                 // AP名称
    char ap_password[64];             // AP密码
    bool ap_password_hidden;          // 密码是否隐藏
    char ap_ip_address[16];           // AP的IP地址
    char ap_gateway[16];              // AP的网关地址
    char sta_ssid[32];                // 连接的WiFi名称
    char sta_ip_address[16];          // 获取的IP地址
    bool sta_connected;               // WiFi连接状态
    
    // NTP配置
    bool ntp_auto_sync;               // NTP自动同步开关
    int ntp_sync_interval;            // NTP同步间隔(秒)
    
    // 运行时间
    char total_runtime[16];           // 总运行时间
    
    // 传感器数据
    float bmp580_temperature;         // BMP580温度
    float bmp580_pressure;            // BMP580气压
    float aht20_temperature;          // AHT20温度
    float aht20_humidity;             // AHT20湿度
    
    // 唤醒模式
    int wakeup_mode;                  // 当前唤醒模式 (0=秒跳变, 1=分跳变)
    
    // 显示设置
    char current_display_content[64]; // 当前显示内容
    char current_refresh_rate[32];    // 当前刷新率
} system_info_t;

// Web服务器函数声明
esp_err_t start_webserver(void);
esp_err_t stop_webserver(void);
httpd_handle_t get_webserver_handle(void);

// HTTP处理函数
esp_err_t index_handler(httpd_req_t *req);
esp_err_t api_system_info_handler(httpd_req_t *req);
esp_err_t api_wifi_config_handler(httpd_req_t *req);
esp_err_t api_ap_config_handler(httpd_req_t *req);
esp_err_t api_ntp_sync_handler(httpd_req_t *req);
esp_err_t api_manual_time_handler(httpd_req_t *req);
esp_err_t api_timezone_config_handler(httpd_req_t *req);
esp_err_t api_reboot_handler(httpd_req_t *req);
esp_err_t api_factory_reset_handler(httpd_req_t *req);
esp_err_t api_wifi_scan_handler(httpd_req_t *req);
esp_err_t api_ntp_config_handler(httpd_req_t *req);
esp_err_t api_display_config_handler(httpd_req_t *req);
esp_err_t api_refresh_rate_handler(httpd_req_t *req);
esp_err_t api_reset_battery_handler(httpd_req_t *req);
esp_err_t api_set_battery_capacity_handler(httpd_req_t *req);

#endif // WEB_SERVER_H