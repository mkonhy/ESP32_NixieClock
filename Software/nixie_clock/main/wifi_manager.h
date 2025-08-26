#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include "esp_err.h"
#include "esp_wifi.h"
#include "esp_event.h"

#ifdef __cplusplus
extern "C" {
#endif

// WiFi模式枚举
typedef enum {
    WIFI_MODE_AP_STA,       // AP+STA模式（默认模式）
    WIFI_MODE_STA_ONLY,     // 仅STA模式
    WIFI_MODE_AP_ONLY,      // 仅AP模式  
    WIFI_MODE_OFF           // 关闭WiFi
} wifi_manager_mode_t;

// WiFi连接状态
typedef enum {
    WIFI_STATE_DISCONNECTED,
    WIFI_STATE_CONNECTING,
    WIFI_STATE_CONNECTED,
    WIFI_STATE_FAILED
} wifi_connection_state_t;

// WiFi配置参数
#define WIFI_MANAGER_DEFAULT_SSID       "NixieClock"
#define WIFI_MANAGER_DEFAULT_PASSWORD   "12345678"
#define WIFI_MANAGER_DEFAULT_CHANNEL    1
#define WIFI_MANAGER_MAX_STA_CONN       4
#define WIFI_MANAGER_SCAN_MAX_AP        20
#define WIFI_MANAGER_MAX_SSID_LEN       32
#define WIFI_MANAGER_MAX_PASSWORD_LEN   64

// WiFi配置结构体
typedef struct {
    char sta_ssid[WIFI_MANAGER_MAX_SSID_LEN];
    char sta_password[WIFI_MANAGER_MAX_PASSWORD_LEN];
    char ap_ssid[WIFI_MANAGER_MAX_SSID_LEN];
    char ap_password[WIFI_MANAGER_MAX_PASSWORD_LEN];
    uint8_t ap_channel;
    bool sta_enabled;
    bool ap_enabled;
    wifi_manager_mode_t mode;
    char ap_ip[16];        // AP IP地址
    char ap_gateway[16];   // AP网关
    char ap_netmask[16];   // AP子网掩码
} wifi_config_persistent_t;

// 函数声明
esp_err_t wifi_manager_init(void);
esp_err_t wifi_manager_deinit(void);
esp_err_t wifi_manager_set_mode(wifi_manager_mode_t mode);
esp_err_t wifi_manager_scan_networks(void);
esp_err_t wifi_manager_connect_sta(const char* ssid, const char* password);
esp_err_t wifi_manager_start_ap(const char* ssid, const char* password);
esp_err_t wifi_manager_stop_ap(void);
esp_err_t wifi_manager_disconnect_sta(void);
wifi_connection_state_t wifi_manager_get_sta_state(void);
wifi_manager_mode_t wifi_manager_get_current_mode(void);
void wifi_manager_print_scan_results(void);
esp_err_t wifi_manager_get_sta_ip(char* ip_str, size_t max_len);
esp_err_t wifi_manager_get_scan_results(wifi_ap_record_t* ap_records_out, uint16_t* ap_count_out);
esp_err_t wifi_manager_save_config(const wifi_config_persistent_t* config);
esp_err_t wifi_manager_load_config(wifi_config_persistent_t* config);
esp_err_t wifi_manager_get_default_config(wifi_config_persistent_t* config);
esp_err_t wifi_manager_get_default_config_with_mode(wifi_config_persistent_t* config, bool enable_wifi);
uint8_t wifi_manager_find_best_channel(void);
esp_err_t wifi_manager_get_mac_suffix(char* suffix, size_t max_len);
esp_err_t wifi_manager_set_ap_ip(const char* ip, const char* gateway, const char* netmask);
esp_err_t wifi_manager_get_ap_ip(char* ip_str, size_t max_len);
esp_err_t wifi_manager_apply_config(void);
esp_err_t wifi_manager_get_current_config(wifi_config_persistent_t* config);

#ifdef __cplusplus
}
#endif

#endif // WIFI_MANAGER_H 