#ifndef NTP_SYNC_H
#define NTP_SYNC_H

#include "esp_err.h"
#include <time.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// NTP配置常量
#define NTP_DEFAULT_SERVER1         "ntp.ntsc.ac.cn"
#define NTP_DEFAULT_SERVER2         "pool.ntp.org"
#define NTP_DEFAULT_SERVER3         "time.nist.gov"
#define NTP_MAX_SERVER_LEN          64
#define NTP_DEFAULT_SYNC_INTERVAL   3600    // 默认1小时同步一次（秒）
#define NTP_MIN_SYNC_INTERVAL       300     // 最小5分钟
#define NTP_MAX_SYNC_INTERVAL       86400   // 最大24小时
#define NTP_SYNC_TIMEOUT_MS         10000   // 10秒超时

// NTP同步状态枚举
typedef enum {
    NTP_SYNC_STATUS_IDLE = 0,
    NTP_SYNC_STATUS_IN_PROGRESS,
    NTP_SYNC_STATUS_SUCCESS,
    NTP_SYNC_STATUS_FAILED,
    NTP_SYNC_STATUS_NETWORK_ERROR,
    NTP_SYNC_STATUS_TIMEOUT
} ntp_sync_status_t;

// NTP配置结构体
typedef struct {
    char primary_server[NTP_MAX_SERVER_LEN];     // 主NTP服务器
    char secondary_server[NTP_MAX_SERVER_LEN];   // 备用NTP服务器
    char tertiary_server[NTP_MAX_SERVER_LEN];    // 第三NTP服务器
    uint32_t sync_interval_sec;                  // 同步间隔（秒）
    bool auto_sync_enabled;                      // 是否启用自动同步
    bool manual_sync_requested;                  // 手动同步请求标志
    int32_t timezone_offset_sec;                 // 时区偏移（秒），东八区为+28800
    char timezone_name[32];                      // 时区名称，如"Asia/Shanghai"
} ntp_config_t;

// NTP同步结果结构体
typedef struct {
    ntp_sync_status_t status;                    // 同步状态
    time_t sync_time;                            // 同步时间戳
    int32_t time_diff_sec;                       // 时间差（秒）
    int32_t time_diff_ms;                        // 时间差毫秒部分
    char server_used[NTP_MAX_SERVER_LEN];        // 使用的服务器
    char time_diff_str[16];                      // 格式化的时间差字符串 "xxSxxxMS"
} ntp_sync_result_t;

// 函数声明
esp_err_t ntp_sync_init(void);
esp_err_t ntp_sync_deinit(void);
esp_err_t ntp_sync_start_manual(void);
esp_err_t ntp_sync_stop(void);
esp_err_t ntp_sync_set_config(const ntp_config_t* config);
esp_err_t ntp_sync_get_config(ntp_config_t* config);
esp_err_t ntp_sync_load_config_from_nvs(void);
esp_err_t ntp_sync_save_config_to_nvs(void);
ntp_sync_result_t ntp_sync_get_last_result(void);
ntp_sync_status_t ntp_sync_get_status(void);
bool ntp_sync_is_network_available(void);
esp_err_t ntp_sync_check_and_perform(void);
void ntp_sync_auto_task(void* pvParameters);

// 默认配置获取
void ntp_sync_get_default_config(ntp_config_t* config);

// 格式化时间差字符串
void ntp_sync_format_time_diff(int32_t sec, int32_t ms, char* buffer, size_t buffer_size);

// 时区相关函数
esp_err_t ntp_sync_set_timezone(int32_t offset_sec, const char* timezone_name);
esp_err_t ntp_sync_get_timezone(int32_t* offset_sec, char* timezone_name, size_t name_size);
esp_err_t ntp_sync_apply_timezone(void);
esp_err_t ntp_sync_set_manual_time(time_t timestamp);

#ifdef __cplusplus
}
#endif

#endif // NTP_SYNC_H 