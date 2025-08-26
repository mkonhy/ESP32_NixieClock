#include "ntp_sync.h"
#include "wifi_manager.h"
#include "nixie_ctr.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_sntp.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "sys/time.h"
#include <string.h>

static const char *TAG = "NTP_SYNC";

// NVS相关
#define NVS_NAMESPACE_NTP           "ntp_config"
#define NVS_KEY_PRIMARY_SERVER      "primary_srv"
#define NVS_KEY_SECONDARY_SERVER    "secondary_srv"
#define NVS_KEY_TERTIARY_SERVER     "tertiary_srv"
#define NVS_KEY_SYNC_INTERVAL       "sync_interval"
#define NVS_KEY_AUTO_SYNC_ENABLED   "auto_enabled"
#define NVS_KEY_TIMEZONE_OFFSET     "tz_offset"
#define NVS_KEY_TIMEZONE_NAME       "tz_name"

// 全局变量
static ntp_config_t ntp_config;
static ntp_sync_result_t last_sync_result;
static nvs_handle_t ntp_nvs_handle = 0;
static bool ntp_initialized = false;
static TaskHandle_t ntp_auto_task_handle = NULL;
static EventGroupHandle_t ntp_event_group = NULL;

// 事件组位定义
#define NTP_SYNC_DONE_BIT       BIT0
#define NTP_SYNC_TIMEOUT_BIT    BIT1

// SNTP同步回调函数
static void sntp_sync_time_cb(struct timeval *tv)
{
    ESP_LOGI(TAG, "SNTP同步回调被调用");
    if (ntp_event_group != NULL) {
        xEventGroupSetBits(ntp_event_group, NTP_SYNC_DONE_BIT);
    }
}

// 获取默认配置
void ntp_sync_get_default_config(ntp_config_t* config)
{
    if (config == NULL) return;
    
    strncpy(config->primary_server, NTP_DEFAULT_SERVER1, NTP_MAX_SERVER_LEN - 1);
    config->primary_server[NTP_MAX_SERVER_LEN - 1] = '\0';
    
    strncpy(config->secondary_server, NTP_DEFAULT_SERVER2, NTP_MAX_SERVER_LEN - 1);
    config->secondary_server[NTP_MAX_SERVER_LEN - 1] = '\0';
    
    strncpy(config->tertiary_server, NTP_DEFAULT_SERVER3, NTP_MAX_SERVER_LEN - 1);
    config->tertiary_server[NTP_MAX_SERVER_LEN - 1] = '\0';
    
    config->sync_interval_sec = NTP_DEFAULT_SYNC_INTERVAL;
    config->auto_sync_enabled = true;
    config->manual_sync_requested = false;
    
    // 默认设置为东八区（北京时间）
    config->timezone_offset_sec = 28800;  // +8小时 = 28800秒
    strncpy(config->timezone_name, "Asia/Shanghai", 31);
    config->timezone_name[31] = '\0';
}

// 格式化时间差字符串
void ntp_sync_format_time_diff(int32_t sec, int32_t ms, char* buffer, size_t buffer_size)
{
    if (buffer == NULL || buffer_size < 16) return;
    
    // 处理负数
    bool negative = false;
    if (sec < 0 || (sec == 0 && ms < 0)) {
        negative = true;
        sec = abs(sec);
        ms = abs(ms);
    }
    
    snprintf(buffer, buffer_size, "%s%ldS%ldMS", 
             negative ? "-" : "", (long)sec, (long)ms);
}

// 检查网络是否可用
bool ntp_sync_is_network_available(void)
{
    // 检查WiFi管理器状态
    wifi_manager_mode_t wifi_mode = wifi_manager_get_current_mode();
    if (wifi_mode == WIFI_MODE_OFF) {
        ESP_LOGW(TAG, "WiFi已关闭，网络不可用");
        return false;
    }
    
    // 检查STA连接状态
    wifi_connection_state_t sta_state = wifi_manager_get_sta_state();
    if (sta_state != WIFI_STATE_CONNECTED) {
        ESP_LOGW(TAG, "STA未连接，网络不可用");
        return false;
    }
    
    // 检查是否获取到IP地址
    char ip_str[16];
    esp_err_t ip_ret = wifi_manager_get_sta_ip(ip_str, sizeof(ip_str));
    if (ip_ret != ESP_OK) {
        ESP_LOGW(TAG, "未获取到IP地址，网络不可用");
        return false;
    }
    
    ESP_LOGI(TAG, "网络可用，当前IP: %s", ip_str);
    return true;
}

// 从NVS加载配置
esp_err_t ntp_sync_load_config_from_nvs(void)
{
    esp_err_t ret = nvs_open(NVS_NAMESPACE_NTP, NVS_READONLY, &ntp_nvs_handle);
    if (ret != ESP_OK) {
        if (ret == ESP_ERR_NVS_NOT_FOUND) {
            ESP_LOGI(TAG, "NTP配置不存在，使用默认配置");
            ntp_sync_get_default_config(&ntp_config);
            return ESP_OK;
        }
        ESP_LOGE(TAG, "打开NVS失败: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 加载各个配置项
    size_t str_len = NTP_MAX_SERVER_LEN;
    ret = nvs_get_str(ntp_nvs_handle, NVS_KEY_PRIMARY_SERVER, ntp_config.primary_server, &str_len);
    if (ret != ESP_OK) {
        strncpy(ntp_config.primary_server, NTP_DEFAULT_SERVER1, NTP_MAX_SERVER_LEN - 1);
    }
    
    str_len = NTP_MAX_SERVER_LEN;
    ret = nvs_get_str(ntp_nvs_handle, NVS_KEY_SECONDARY_SERVER, ntp_config.secondary_server, &str_len);
    if (ret != ESP_OK) {
        strncpy(ntp_config.secondary_server, NTP_DEFAULT_SERVER2, NTP_MAX_SERVER_LEN - 1);
    }
    
    str_len = NTP_MAX_SERVER_LEN;
    ret = nvs_get_str(ntp_nvs_handle, NVS_KEY_TERTIARY_SERVER, ntp_config.tertiary_server, &str_len);
    if (ret != ESP_OK) {
        strncpy(ntp_config.tertiary_server, NTP_DEFAULT_SERVER3, NTP_MAX_SERVER_LEN - 1);
    }
    
    ret = nvs_get_u32(ntp_nvs_handle, NVS_KEY_SYNC_INTERVAL, &ntp_config.sync_interval_sec);
    if (ret != ESP_OK) {
        ntp_config.sync_interval_sec = NTP_DEFAULT_SYNC_INTERVAL;
    }
    
    uint8_t auto_enabled = 1;
    ret = nvs_get_u8(ntp_nvs_handle, NVS_KEY_AUTO_SYNC_ENABLED, &auto_enabled);
    if (ret != ESP_OK) {
        auto_enabled = 1;
    }
    ntp_config.auto_sync_enabled = (auto_enabled != 0);
    
    uint32_t tz_offset_uint;
    ret = nvs_get_u32(ntp_nvs_handle, NVS_KEY_TIMEZONE_OFFSET, &tz_offset_uint);
    if (ret != ESP_OK) {
        ntp_config.timezone_offset_sec = 28800;  // +8小时 = 28800秒
    } else {
        ntp_config.timezone_offset_sec = (int32_t)tz_offset_uint;
    }
    
    str_len = sizeof(ntp_config.timezone_name) - 1;
    ret = nvs_get_str(ntp_nvs_handle, NVS_KEY_TIMEZONE_NAME, ntp_config.timezone_name, &str_len);
    if (ret != ESP_OK) {
        strncpy(ntp_config.timezone_name, "Asia/Shanghai", sizeof(ntp_config.timezone_name) - 1);
        ntp_config.timezone_name[sizeof(ntp_config.timezone_name) - 1] = '\0';
    }
    
    nvs_close(ntp_nvs_handle);
    ntp_nvs_handle = 0;
    
    ESP_LOGI(TAG, "NTP配置已从NVS加载:");
    ESP_LOGI(TAG, "  主服务器: %s", ntp_config.primary_server);
    ESP_LOGI(TAG, "  备用服务器: %s", ntp_config.secondary_server);
    ESP_LOGI(TAG, "  第三服务器: %s", ntp_config.tertiary_server);
    ESP_LOGI(TAG, "  同步间隔: %lu秒", ntp_config.sync_interval_sec);
    ESP_LOGI(TAG, "  自动同步: %s", ntp_config.auto_sync_enabled ? "启用" : "禁用");
    ESP_LOGI(TAG, "  时区偏移: %ld秒", ntp_config.timezone_offset_sec);
    ESP_LOGI(TAG, "  时区名称: %s", ntp_config.timezone_name);
    
    return ESP_OK;
}

// 保存配置到NVS
esp_err_t ntp_sync_save_config_to_nvs(void)
{
    esp_err_t ret = nvs_open(NVS_NAMESPACE_NTP, NVS_READWRITE, &ntp_nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "打开NVS失败: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 保存各个配置项
    ret = nvs_set_str(ntp_nvs_handle, NVS_KEY_PRIMARY_SERVER, ntp_config.primary_server);
    if (ret != ESP_OK) goto save_error;
    
    ret = nvs_set_str(ntp_nvs_handle, NVS_KEY_SECONDARY_SERVER, ntp_config.secondary_server);
    if (ret != ESP_OK) goto save_error;
    
    ret = nvs_set_str(ntp_nvs_handle, NVS_KEY_TERTIARY_SERVER, ntp_config.tertiary_server);
    if (ret != ESP_OK) goto save_error;
    
    ret = nvs_set_u32(ntp_nvs_handle, NVS_KEY_SYNC_INTERVAL, ntp_config.sync_interval_sec);
    if (ret != ESP_OK) goto save_error;
    
    uint8_t auto_enabled = ntp_config.auto_sync_enabled ? 1 : 0;
    ret = nvs_set_u8(ntp_nvs_handle, NVS_KEY_AUTO_SYNC_ENABLED, auto_enabled);
    if (ret != ESP_OK) goto save_error;
    
    ret = nvs_set_u32(ntp_nvs_handle, NVS_KEY_TIMEZONE_OFFSET, (uint32_t)ntp_config.timezone_offset_sec);
    if (ret != ESP_OK) goto save_error;
    
    ret = nvs_set_str(ntp_nvs_handle, NVS_KEY_TIMEZONE_NAME, ntp_config.timezone_name);
    if (ret != ESP_OK) goto save_error;
    
    ret = nvs_commit(ntp_nvs_handle);
    if (ret != ESP_OK) goto save_error;
    
    nvs_close(ntp_nvs_handle);
    ntp_nvs_handle = 0;
    
    ESP_LOGI(TAG, "NTP配置已保存到NVS");
    return ESP_OK;
    
save_error:
    ESP_LOGE(TAG, "保存NTP配置失败: %s", esp_err_to_name(ret));
    nvs_close(ntp_nvs_handle);
    ntp_nvs_handle = 0;
    return ret;
}

// 执行NTP同步
static esp_err_t perform_ntp_sync(void)
{
    ESP_LOGI(TAG, "==>> 开始NTP时间同步 <<==");
    
    // 记录同步前的时间
    time_t time_before_sync;
    struct timeval tv_before;
    gettimeofday(&tv_before, NULL);
    time_before_sync = tv_before.tv_sec;
    
    struct tm timeinfo_before;
    localtime_r(&time_before_sync, &timeinfo_before);
    ESP_LOGI(TAG, "同步前时间: %04d-%02d-%02d %02d:%02d:%02d.%03ld", 
             timeinfo_before.tm_year + 1900, timeinfo_before.tm_mon + 1, timeinfo_before.tm_mday,
             timeinfo_before.tm_hour, timeinfo_before.tm_min, timeinfo_before.tm_sec,
             tv_before.tv_usec / 1000);
    
    // 清除事件位
    xEventGroupClearBits(ntp_event_group, NTP_SYNC_DONE_BIT | NTP_SYNC_TIMEOUT_BIT);
    
    // 配置SNTP
    esp_sntp_stop();
    esp_sntp_setoperatingmode(ESP_SNTP_OPMODE_POLL);
    
    // 设置服务器
    esp_sntp_setservername(0, ntp_config.primary_server);
    esp_sntp_setservername(1, ntp_config.secondary_server);
    esp_sntp_setservername(2, ntp_config.tertiary_server);
    
    ESP_LOGI(TAG, "使用NTP服务器:");
    ESP_LOGI(TAG, "  主: %s", ntp_config.primary_server);
    ESP_LOGI(TAG, "  备: %s", ntp_config.secondary_server);
    ESP_LOGI(TAG, "  三: %s", ntp_config.tertiary_server);
    
    // 设置同步回调
    esp_sntp_set_time_sync_notification_cb(sntp_sync_time_cb);
    
    // 启动SNTP
    esp_sntp_init();
    
    ESP_LOGI(TAG, "等待NTP同步完成...");
    
    // 等待同步完成或超时
    EventBits_t bits = xEventGroupWaitBits(ntp_event_group,
                                          NTP_SYNC_DONE_BIT,
                                          pdTRUE,
                                          pdFALSE,
                                          NTP_SYNC_TIMEOUT_MS / portTICK_PERIOD_MS);
    
    if (!(bits & NTP_SYNC_DONE_BIT)) {
        ESP_LOGE(TAG, "NTP同步超时");
        esp_sntp_stop();
        
        last_sync_result.status = NTP_SYNC_STATUS_TIMEOUT;
        strncpy(last_sync_result.time_diff_str, "TIMEOUT", sizeof(last_sync_result.time_diff_str) - 1);
        return ESP_ERR_TIMEOUT;
    }
    
    // 记录同步后的时间
    time_t time_after_sync;
    struct timeval tv_after;
    gettimeofday(&tv_after, NULL);
    time_after_sync = tv_after.tv_sec;
    
    struct tm timeinfo_after;
    localtime_r(&time_after_sync, &timeinfo_after);
    ESP_LOGI(TAG, "同步后时间: %04d-%02d-%02d %02d:%02d:%02d.%03ld", 
             timeinfo_after.tm_year + 1900, timeinfo_after.tm_mon + 1, timeinfo_after.tm_mday,
             timeinfo_after.tm_hour, timeinfo_after.tm_min, timeinfo_after.tm_sec,
             tv_after.tv_usec / 1000);
    
    // 计算时间差
    int64_t time_diff_us = (tv_after.tv_sec - tv_before.tv_sec) * 1000000LL + 
                           (tv_after.tv_usec - tv_before.tv_usec);
    
    int32_t time_diff_sec = time_diff_us / 1000000;
    int32_t time_diff_ms = (time_diff_us % 1000000) / 1000;
    
    // 更新同步结果
    last_sync_result.status = NTP_SYNC_STATUS_SUCCESS;
    last_sync_result.sync_time = time_after_sync;
    last_sync_result.time_diff_sec = time_diff_sec;
    last_sync_result.time_diff_ms = time_diff_ms;
    strncpy(last_sync_result.server_used, ntp_config.primary_server, NTP_MAX_SERVER_LEN - 1);
    ntp_sync_format_time_diff(time_diff_sec, time_diff_ms, 
                             last_sync_result.time_diff_str, 
                             sizeof(last_sync_result.time_diff_str));
    
    ESP_LOGI(TAG, "✓ NTP同步成功！");
    ESP_LOGI(TAG, "时间差: %s", last_sync_result.time_diff_str);
    ESP_LOGI(TAG, "使用服务器: %s", last_sync_result.server_used);
    
    esp_sntp_stop();
    
    return ESP_OK;
}

// 检查并执行NTP同步
esp_err_t ntp_sync_check_and_perform(void)
{
    if (!ntp_initialized) {
        ESP_LOGE(TAG, "NTP同步未初始化");
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "检查NTP同步条件...");
    
    // 检查网络是否可用
    if (!ntp_sync_is_network_available()) {
        ESP_LOGW(TAG, "网络不可用，跳过NTP同步");
        last_sync_result.status = NTP_SYNC_STATUS_NETWORK_ERROR;
        strncpy(last_sync_result.time_diff_str, "NET_ERR", sizeof(last_sync_result.time_diff_str) - 1);
        return ESP_ERR_INVALID_STATE;
    }
    
    // 执行同步
    esp_err_t ret = perform_ntp_sync();
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "NTP同步完成");
    } else {
        ESP_LOGE(TAG, "NTP同步失败: %s", esp_err_to_name(ret));
    }
    
    return ret;
}

// 手动启动NTP同步
esp_err_t ntp_sync_start_manual(void)
{
    ESP_LOGI(TAG, "手动启动NTP同步");
    ntp_config.manual_sync_requested = true;
    return ntp_sync_check_and_perform();
}

// NTP自动同步任务
void ntp_sync_auto_task(void* pvParameters)
{
    ESP_LOGI(TAG, "NTP自动同步任务启动");
    
    TickType_t last_sync_time = 0;
    TickType_t sync_interval_ticks = pdMS_TO_TICKS(ntp_config.sync_interval_sec * 1000);
    
    while (1) {
        TickType_t current_time = xTaskGetTickCount();
        
        // 检查是否需要同步
        bool should_sync = false;
        
        // 手动同步请求
        if (ntp_config.manual_sync_requested) {
            should_sync = true;
            ntp_config.manual_sync_requested = false;
            ESP_LOGI(TAG, "检测到手动同步请求");
        }
        // 自动同步检查
        else if (ntp_config.auto_sync_enabled) {
            // 检查是否在秒跳变模式
            // 通过外部函数获取当前唤醒模式，避免直接extern声明
            extern int get_current_wakeup_mode(void);
            int current_mode = get_current_wakeup_mode();
            
            if (current_mode == 0) { // WAKEUP_MODE_SECOND
                if ((current_time - last_sync_time) >= sync_interval_ticks) {
                    should_sync = true;
                    ESP_LOGI(TAG, "自动同步时间到（秒跳变模式）");
                }
            } else {
                // 非秒跳变模式，不进行自动同步
                if ((current_time - last_sync_time) >= sync_interval_ticks) {
                    ESP_LOGI(TAG, "自动同步时间到，但非秒跳变模式，跳过同步");
                    last_sync_time = current_time; // 更新时间，避免频繁日志
                }
            }
        }
        
        if (should_sync) {
            ESP_LOGI(TAG, "开始执行NTP同步...");
            esp_err_t ret = ntp_sync_check_and_perform();
            if (ret == ESP_OK) {
                last_sync_time = current_time;
                ESP_LOGI(TAG, "NTP同步成功，下次同步时间: %lu秒后", ntp_config.sync_interval_sec);
            } else {
                ESP_LOGW(TAG, "NTP同步失败，将在下个周期重试");
            }
        }
        
        // 每60秒检查一次
        vTaskDelay(pdMS_TO_TICKS(60000));
    }
}

// 初始化NTP同步
esp_err_t ntp_sync_init(void)
{
    ESP_LOGI(TAG, "==>> 初始化NTP同步系统 <<==");
    
    if (ntp_initialized) {
        ESP_LOGW(TAG, "NTP同步已初始化");
        return ESP_OK;
    }
    
    // 创建事件组
    ntp_event_group = xEventGroupCreate();
    if (ntp_event_group == NULL) {
        ESP_LOGE(TAG, "创建事件组失败");
        return ESP_FAIL;
    }
    
    // 初始化同步结果
    memset(&last_sync_result, 0, sizeof(last_sync_result));
    last_sync_result.status = NTP_SYNC_STATUS_IDLE;
    strncpy(last_sync_result.time_diff_str, "NONE", sizeof(last_sync_result.time_diff_str) - 1);
    
    // 加载配置
    esp_err_t ret = ntp_sync_load_config_from_nvs();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "加载NTP配置失败，使用默认配置");
        ntp_sync_get_default_config(&ntp_config);
    }
    
    // 应用时区设置
    ret = ntp_sync_apply_timezone();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "应用时区设置失败: %s", esp_err_to_name(ret));
    }
    
    // 创建自动同步任务
    BaseType_t task_ret = xTaskCreate(ntp_sync_auto_task, 
                                      "ntp_auto_sync", 
                                      4096, 
                                      NULL, 
                                      5, 
                                      &ntp_auto_task_handle);
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "创建NTP自动同步任务失败");
        vEventGroupDelete(ntp_event_group);
        ntp_event_group = NULL;
        return ESP_FAIL;
    }
    
    ntp_initialized = true;
    ESP_LOGI(TAG, "✓ NTP同步系统初始化完成");
    ESP_LOGI(TAG, "自动同步: %s", ntp_config.auto_sync_enabled ? "启用" : "禁用");
    ESP_LOGI(TAG, "同步间隔: %lu秒", ntp_config.sync_interval_sec);
    
    return ESP_OK;
}

// 反初始化NTP同步
esp_err_t ntp_sync_deinit(void)
{
    ESP_LOGI(TAG, "反初始化NTP同步系统");
    
    if (!ntp_initialized) {
        return ESP_OK;
    }
    
    // 停止SNTP
    esp_sntp_stop();
    
    // 删除任务
    if (ntp_auto_task_handle != NULL) {
        vTaskDelete(ntp_auto_task_handle);
        ntp_auto_task_handle = NULL;
    }
    
    // 删除事件组
    if (ntp_event_group != NULL) {
        vEventGroupDelete(ntp_event_group);
        ntp_event_group = NULL;
    }
    
    ntp_initialized = false;
    ESP_LOGI(TAG, "NTP同步系统已反初始化");
    
    return ESP_OK;
}

// 获取配置
esp_err_t ntp_sync_get_config(ntp_config_t* config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    memcpy(config, &ntp_config, sizeof(ntp_config_t));
    return ESP_OK;
}

// 设置配置
esp_err_t ntp_sync_set_config(const ntp_config_t* config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // 验证同步间隔
    if (config->sync_interval_sec < NTP_MIN_SYNC_INTERVAL || 
        config->sync_interval_sec > NTP_MAX_SYNC_INTERVAL) {
        ESP_LOGE(TAG, "同步间隔超出范围: %lu", config->sync_interval_sec);
        return ESP_ERR_INVALID_ARG;
    }
    
    memcpy(&ntp_config, config, sizeof(ntp_config_t));
    
    ESP_LOGI(TAG, "NTP配置已更新");
    return ntp_sync_save_config_to_nvs();
}

// 获取最后同步结果
ntp_sync_result_t ntp_sync_get_last_result(void)
{
    return last_sync_result;
}

// 获取同步状态
ntp_sync_status_t ntp_sync_get_status(void)
{
    return last_sync_result.status;
}

// 停止NTP同步
esp_err_t ntp_sync_stop(void)
{
    ESP_LOGI(TAG, "停止NTP同步");
    esp_sntp_stop();
    last_sync_result.status = NTP_SYNC_STATUS_IDLE;
    return ESP_OK;
}

// 设置时区
esp_err_t ntp_sync_set_timezone(int32_t offset_sec, const char* timezone_name)
{
    if (timezone_name == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "设置时区: %s (偏移: %ld秒)", timezone_name, offset_sec);
    
    ntp_config.timezone_offset_sec = offset_sec;
    strncpy(ntp_config.timezone_name, timezone_name, sizeof(ntp_config.timezone_name) - 1);
    ntp_config.timezone_name[sizeof(ntp_config.timezone_name) - 1] = '\0';
    
    // 应用时区设置
    esp_err_t ret = ntp_sync_apply_timezone();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "应用时区设置失败: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 保存到NVS
    ret = ntp_sync_save_config_to_nvs();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "保存时区设置到NVS失败: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "时区设置已保存并应用");
    return ESP_OK;
}

// 获取时区
esp_err_t ntp_sync_get_timezone(int32_t* offset_sec, char* timezone_name, size_t name_size)
{
    if (offset_sec == NULL || timezone_name == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    *offset_sec = ntp_config.timezone_offset_sec;
    strncpy(timezone_name, ntp_config.timezone_name, name_size - 1);
    timezone_name[name_size - 1] = '\0';
    
    return ESP_OK;
}

// 应用时区设置
esp_err_t ntp_sync_apply_timezone(void)
{
    ESP_LOGI(TAG, "应用时区设置: %s (偏移: %ld秒)", 
             ntp_config.timezone_name, ntp_config.timezone_offset_sec);
    
    // 设置时区环境变量 - 使用标准格式
    char tz_string[64];
    int32_t hours = ntp_config.timezone_offset_sec / 3600;
    int32_t minutes = (ntp_config.timezone_offset_sec % 3600) / 60;
    
    // 使用标准时区格式：GMT+8 或 GMT-5 等
    if (ntp_config.timezone_offset_sec >= 0) {
        snprintf(tz_string, sizeof(tz_string), "GMT-%ld:%02ld", hours, minutes);
    } else {
        snprintf(tz_string, sizeof(tz_string), "GMT+%ld:%02ld", -hours, minutes);
    }
    
    ESP_LOGI(TAG, "设置TZ环境变量: %s", tz_string);
    setenv("TZ", tz_string, 1);
    tzset();
    
    return ESP_OK;
}

// 手动设置时间
esp_err_t ntp_sync_set_manual_time(time_t timestamp)
{
    ESP_LOGI(TAG, "手动设置时间: %lld", (long long)timestamp);
    
    // 创建timeval结构
    struct timeval tv;
    tv.tv_sec = timestamp;
    tv.tv_usec = 0;
    
    // 设置系统时间
    esp_err_t ret = settimeofday(&tv, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "设置系统时间失败: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 应用时区设置
    ret = ntp_sync_apply_timezone();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "应用时区设置失败: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 更新同步结果
    last_sync_result.status = NTP_SYNC_STATUS_SUCCESS;
    last_sync_result.sync_time = timestamp;
    last_sync_result.time_diff_sec = 0;
    last_sync_result.time_diff_ms = 0;
    strncpy(last_sync_result.server_used, "MANUAL", sizeof(last_sync_result.server_used) - 1);
    strncpy(last_sync_result.time_diff_str, "MANUAL", sizeof(last_sync_result.time_diff_str) - 1);
    
    ESP_LOGI(TAG, "手动时间设置成功");
    return ESP_OK;
} 