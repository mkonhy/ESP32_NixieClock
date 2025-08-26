#include "wifi_manager.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_mac.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include <string.h>

static const char *TAG = "WIFI_MANAGER";

// WiFi事件组
#define WIFI_CONNECTED_BIT      BIT0
#define WIFI_FAIL_BIT          BIT1
#define WIFI_SCAN_DONE_BIT     BIT2

// NVS配置键名
#define NVS_NAMESPACE          "wifi_config"
#define NVS_CONFIG_KEY         "config"

static EventGroupHandle_t wifi_event_group;
static esp_netif_t *netif_sta = NULL;
static esp_netif_t *netif_ap = NULL;
static wifi_manager_mode_t current_mode = WIFI_MODE_OFF;
static wifi_connection_state_t sta_state = WIFI_STATE_DISCONNECTED;
static wifi_ap_record_t ap_records[WIFI_MANAGER_SCAN_MAX_AP];
static uint16_t ap_count = 0;
static int retry_count = 0;
static const int MAX_RETRY = 5;
static wifi_config_persistent_t persistent_config;

// 事件处理函数
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                              int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT) {
        switch (event_id) {
            case WIFI_EVENT_STA_START:
                ESP_LOGI(TAG, "STA模式启动");
                break;
                
            case WIFI_EVENT_STA_DISCONNECTED:
                ESP_LOGI(TAG, "STA断开连接");
                sta_state = WIFI_STATE_DISCONNECTED;
                if (retry_count < MAX_RETRY) {
                    esp_wifi_connect();
                    retry_count++;
                    ESP_LOGI(TAG, "重试连接中... (%d/%d)", retry_count, MAX_RETRY);
                    sta_state = WIFI_STATE_CONNECTING;
                } else {
                    ESP_LOGI(TAG, "连接失败，达到最大重试次数");
                    sta_state = WIFI_STATE_FAILED;
                    xEventGroupSetBits(wifi_event_group, WIFI_FAIL_BIT);
                }
                break;
                
            case WIFI_EVENT_AP_START:
                ESP_LOGI(TAG, "AP模式启动成功");
                break;
                
            case WIFI_EVENT_AP_STOP:
                ESP_LOGI(TAG, "AP模式停止");
                break;
                
            case WIFI_EVENT_AP_STACONNECTED:
                ESP_LOGI(TAG, "设备连接到AP");
                break;
                
            case WIFI_EVENT_AP_STADISCONNECTED:
                ESP_LOGI(TAG, "设备从AP断开");
                break;
                
            case WIFI_EVENT_SCAN_DONE:
                ESP_LOGI(TAG, "WiFi扫描完成");
                xEventGroupSetBits(wifi_event_group, WIFI_SCAN_DONE_BIT);
                break;
                
            default:
                break;
        }
    } else if (event_base == IP_EVENT) {
        switch (event_id) {
            case IP_EVENT_STA_GOT_IP:
                ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
                ESP_LOGI(TAG, "获得IP地址: " IPSTR, IP2STR(&event->ip_info.ip));
                retry_count = 0;
                sta_state = WIFI_STATE_CONNECTED;
                xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
                break;
                
            default:
                break;
        }
    }
}

esp_err_t wifi_manager_init(void)
{
    ESP_LOGI(TAG, "初始化WiFi管理器");
    
    // 创建事件组
    wifi_event_group = xEventGroupCreate();
    if (wifi_event_group == NULL) {
        ESP_LOGE(TAG, "创建事件组失败");
        return ESP_FAIL;
    }
    
    // 初始化网络接口
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    // 创建默认STA和AP网络接口
    netif_sta = esp_netif_create_default_wifi_sta();
    netif_ap = esp_netif_create_default_wifi_ap();
    
    if (netif_sta == NULL || netif_ap == NULL) {
        ESP_LOGE(TAG, "创建网络接口失败");
        return ESP_FAIL;
    }
    
    // 初始化WiFi
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    
    // 注册事件处理函数
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));
    
    // 设置WiFi存储为RAM
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    
    // 加载WiFi配置
    esp_err_t ret = wifi_manager_load_config(&persistent_config);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "首次启动或配置损坏，生成默认配置");
        
        // 生成带MAC后缀的默认配置
        memset(&persistent_config, 0, sizeof(wifi_config_persistent_t));
        
        // 生成带MAC后缀的AP名称
        char mac_suffix[5];
        esp_err_t mac_ret = wifi_manager_get_mac_suffix(mac_suffix, sizeof(mac_suffix));
        if (mac_ret == ESP_OK) {
            snprintf(persistent_config.ap_ssid, sizeof(persistent_config.ap_ssid), "%s_%s", 
                    WIFI_MANAGER_DEFAULT_SSID, mac_suffix);
        } else {
            strcpy(persistent_config.ap_ssid, WIFI_MANAGER_DEFAULT_SSID);
        }
        
        strcpy(persistent_config.ap_password, WIFI_MANAGER_DEFAULT_PASSWORD);
        persistent_config.ap_channel = WIFI_MANAGER_DEFAULT_CHANNEL;
        
        // 使用多重检查确保正确检测恢复出厂设置状态
        bool is_factory_reset = false;
        nvs_handle_t check_handle;
        
        // 方法1：检查恢复出厂设置标记
        nvs_handle_t marker_handle;
        esp_err_t marker_ret = nvs_open("system", NVS_READWRITE, &marker_handle);
        if (marker_ret == ESP_OK) {
            uint8_t factory_reset_flag = 0;
            esp_err_t get_ret = nvs_get_u8(marker_handle, "factory_reset", &factory_reset_flag);
            if (get_ret == ESP_OK && factory_reset_flag == 1) {
                is_factory_reset = true;
                ESP_LOGI(TAG, "检测到恢复出厂设置标记");
                // 清除标记
                nvs_erase_key(marker_handle, "factory_reset");
                nvs_commit(marker_handle);
                ESP_LOGI(TAG, "已清除恢复出厂设置标记");
            }
            nvs_close(marker_handle);
        }
        
        // 方法2：检查关键命名空间是否存在
        if (!is_factory_reset) {
            esp_err_t wifi_check = nvs_open("wifi_config", NVS_READONLY, &check_handle);
            if (wifi_check == ESP_OK) {
                nvs_close(check_handle);
            }
            
            esp_err_t nixie_check = nvs_open("nixie_clock", NVS_READONLY, &check_handle);
            if (nixie_check == ESP_OK) {
                nvs_close(check_handle);
            }
            
            // 如果两个关键命名空间都不存在，说明是恢复出厂设置
            if (wifi_check == ESP_ERR_NVS_NOT_FOUND && nixie_check == ESP_ERR_NVS_NOT_FOUND) {
                is_factory_reset = true;
                ESP_LOGI(TAG, "检测到恢复出厂设置后的首次启动（命名空间不存在）");
            }
        }
        
        // 方法3：检查配置完整性
        if (!is_factory_reset) {
            // 尝试读取之前保存的配置，如果读取失败但命名空间存在，可能是分区表变化
            nvs_handle_t config_handle;
            esp_err_t open_ret = nvs_open("wifi_config", NVS_READONLY, &config_handle);
            if (open_ret == ESP_OK) {
                size_t length = sizeof(wifi_config_persistent_t);
                esp_err_t get_ret = nvs_get_blob(config_handle, "config", &persistent_config, &length);
                nvs_close(config_handle);
                
                if (get_ret != ESP_OK || length != sizeof(wifi_config_persistent_t)) {
                    ESP_LOGW(TAG, "配置读取失败或大小不匹配，可能是分区表变化");
                    is_factory_reset = true;
                }
            }
        }
        
        if (is_factory_reset) {
            // 恢复出厂设置后，启用WiFi功能（APSTA模式为默认模式）
            persistent_config.sta_enabled = true;
            persistent_config.ap_enabled = true;
            persistent_config.mode = WIFI_MODE_AP_STA;  // 0 = APSTA模式
            ESP_LOGI(TAG, "恢复出厂设置模式：WiFi功能已启用（APSTA模式）");
        } else {
            // 正常首次启动，启用WiFi功能（APSTA模式为默认模式）
            persistent_config.sta_enabled = true;
            persistent_config.ap_enabled = true;
            persistent_config.mode = WIFI_MODE_AP_STA;  // 0 = APSTA模式
            ESP_LOGI(TAG, "正常首次启动模式：WiFi功能已启用（APSTA模式）");
        }
        
        // 设置默认AP IP配置
        strcpy(persistent_config.ap_ip, "192.168.4.1");
        strcpy(persistent_config.ap_gateway, "192.168.4.1");
        strcpy(persistent_config.ap_netmask, "255.255.255.0");
        
        // 保存默认配置到NVS
        esp_err_t save_ret = wifi_manager_save_config(&persistent_config);
        if (save_ret == ESP_OK) {
            const char* mode_str = (persistent_config.mode == WIFI_MODE_OFF) ? "OFF" : 
                                  (persistent_config.mode == WIFI_MODE_AP_STA) ? "APSTA" :
                                  (persistent_config.mode == WIFI_MODE_STA_ONLY) ? "STA_ONLY" : "AP_ONLY";
            ESP_LOGI(TAG, "默认配置已保存: AP_SSID=%s, 模式=%s (枚举值=%d)", 
                    persistent_config.ap_ssid, mode_str, persistent_config.mode);
        } else {
            ESP_LOGW(TAG, "保存默认配置失败: %s", esp_err_to_name(save_ret));
        }
    }
    
    ESP_LOGI(TAG, "WiFi管理器初始化完成");
    return ESP_OK;
}

esp_err_t wifi_manager_deinit(void)
{
    ESP_LOGI(TAG, "反初始化WiFi管理器");
    
    esp_wifi_stop();
    esp_wifi_deinit();
    
    if (wifi_event_group) {
        vEventGroupDelete(wifi_event_group);
        wifi_event_group = NULL;
    }
    
    return ESP_OK;
}

esp_err_t wifi_manager_set_mode(wifi_manager_mode_t mode)
{
    esp_err_t ret = ESP_OK;
    
    const char* mode_str = (mode == WIFI_MODE_OFF) ? "OFF" : 
                          (mode == WIFI_MODE_AP_STA) ? "APSTA" :
                          (mode == WIFI_MODE_STA_ONLY) ? "STA_ONLY" : "AP_ONLY";
    ESP_LOGI(TAG, "设置WiFi模式: %s (枚举值=%d)", mode_str, mode);
    
    // 安全地停止当前WiFi，增加更长的延迟确保完全停止
    esp_err_t stop_ret = esp_wifi_stop();
    if (stop_ret != ESP_OK && stop_ret != ESP_ERR_WIFI_NOT_STARTED) {
        ESP_LOGW(TAG, "停止WiFi警告: %s", esp_err_to_name(stop_ret));
    }
    vTaskDelay(200 / portTICK_PERIOD_MS);  // 增加延迟确保WiFi完全停止
    
    current_mode = mode;
    
    switch (mode) {
        case WIFI_MODE_OFF:
            ret = esp_wifi_set_mode(WIFI_MODE_NULL);
            break;
            
        case WIFI_MODE_STA_ONLY:
            ret = esp_wifi_set_mode(WIFI_MODE_STA);
            break;
            
        case WIFI_MODE_AP_ONLY:
            ret = esp_wifi_set_mode(WIFI_MODE_AP);
            break;
            
        case WIFI_MODE_AP_STA:
            ret = esp_wifi_set_mode(WIFI_MODE_APSTA);
            break;
            
        default:
            ESP_LOGE(TAG, "无效的WiFi模式");
            return ESP_ERR_INVALID_ARG;
    }
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "设置WiFi模式失败: %s", esp_err_to_name(ret));
        return ret;
    }
    
    if (mode == WIFI_MODE_OFF) {
        ESP_LOGI(TAG, "WiFi已关闭");
        return ESP_OK;
    }
    
    // 启动WiFi，增加重试机制
    int retry_attempts = 0;
    const int max_retries = 3;
    
    while (retry_attempts < max_retries) {
        ret = esp_wifi_start();
        if (ret == ESP_OK) {
            break;
        }
        
        retry_attempts++;
        ESP_LOGW(TAG, "启动WiFi失败 (尝试 %d/%d): %s", 
                retry_attempts, max_retries, esp_err_to_name(ret));
        
        if (retry_attempts < max_retries) {
            vTaskDelay(500 / portTICK_PERIOD_MS);  // 重试前等待
        }
    }
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "启动WiFi最终失败: %s", esp_err_to_name(ret));
        return ret;
    }
    
    vTaskDelay(800 / portTICK_PERIOD_MS);  // 增加延迟确保WiFi稳定启动
    ESP_LOGI(TAG, "WiFi模式设置完成");
    
    return ESP_OK;
}

esp_err_t wifi_manager_scan_networks(void)
{
    ESP_LOGI(TAG, "开始扫描WiFi网络");
    
    // 检查WiFi事件组是否有效
    if (wifi_event_group == NULL) {
        ESP_LOGE(TAG, "WiFi事件组未初始化");
        return ESP_ERR_INVALID_STATE;
    }
    
    // 检查当前WiFi模式是否支持STA功能
    wifi_mode_t wifi_mode;
    esp_err_t ret = esp_wifi_get_mode(&wifi_mode);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "获取WiFi模式失败: %s", esp_err_to_name(ret));
        return ret;
    }
    
    if (wifi_mode != WIFI_MODE_STA && wifi_mode != WIFI_MODE_APSTA) {
        ESP_LOGE(TAG, "当前WiFi模式不支持扫描: %d (需要STA或APSTA模式)", wifi_mode);
        return ESP_ERR_INVALID_STATE;
    }
    
    // 清除之前的扫描完成位
    xEventGroupClearBits(wifi_event_group, WIFI_SCAN_DONE_BIT);
    
    // 清空之前的扫描结果
    memset(ap_records, 0, sizeof(ap_records));
    ap_count = 0;
    
    wifi_scan_config_t scan_config = {
        .ssid = NULL,
        .bssid = NULL,
        .channel = 0,
        .show_hidden = true,
        .scan_type = WIFI_SCAN_TYPE_ACTIVE,
        .scan_time = {
            .active = {
                .min = 100,
                .max = 300
            }
        }
    };
    
    ret = esp_wifi_scan_start(&scan_config, false);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "开始扫描失败: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 等待扫描完成
    EventBits_t bits = xEventGroupWaitBits(wifi_event_group,
                                          WIFI_SCAN_DONE_BIT,
                                          pdTRUE,  // 清除位
                                          pdFALSE,
                                          15000 / portTICK_PERIOD_MS);  // 延长超时时间
    
    if (!(bits & WIFI_SCAN_DONE_BIT)) {
        ESP_LOGE(TAG, "扫描超时");
        esp_wifi_scan_stop();  // 停止扫描
        return ESP_ERR_TIMEOUT;
    }
    
    // 获取扫描结果
    ap_count = WIFI_MANAGER_SCAN_MAX_AP;
    ret = esp_wifi_scan_get_ap_records(&ap_count, ap_records);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "获取扫描结果失败: %s", esp_err_to_name(ret));
        ap_count = 0;  // 重置计数
        return ret;
    }
    
    ESP_LOGI(TAG, "扫描完成，发现 %d 个网络", ap_count);
    return ESP_OK;
}

void wifi_manager_print_scan_results(void)
{
    ESP_LOGI(TAG, "扫描结果:");
    ESP_LOGI(TAG, "序号  SSID                          信号强度  加密方式");
    ESP_LOGI(TAG, "----  ----------------------------  --------  --------");
    
    if (ap_count == 0) {
        ESP_LOGI(TAG, "未发现任何WiFi网络");
        return;
    }
    
    for (int i = 0; i < ap_count; i++) {
        const char* auth_mode;
        switch (ap_records[i].authmode) {
            case WIFI_AUTH_OPEN:
                auth_mode = "开放";
                break;
            case WIFI_AUTH_WEP:
                auth_mode = "WEP";
                break;
            case WIFI_AUTH_WPA_PSK:
                auth_mode = "WPA";
                break;
            case WIFI_AUTH_WPA2_PSK:
                auth_mode = "WPA2";
                break;
            case WIFI_AUTH_WPA_WPA2_PSK:
                auth_mode = "WPA/WPA2";
                break;
            case WIFI_AUTH_WPA3_PSK:
                auth_mode = "WPA3";
                break;
            default:
                auth_mode = "其他";
                break;
        }
        
        // 安全地处理SSID字符串，确保以NULL结尾
        char ssid_str[33];  // WiFi SSID最大长度为32字节
        memset(ssid_str, 0, sizeof(ssid_str));
        memcpy(ssid_str, ap_records[i].ssid, 
               ap_records[i].ssid[32] == 0 ? strlen((char*)ap_records[i].ssid) : 32);
        
        ESP_LOGI(TAG, "%4d  %-28s  %4d dBm   %s", 
                i + 1, 
                ssid_str,
                ap_records[i].rssi,
                auth_mode);
    }
}

esp_err_t wifi_manager_connect_sta(const char* ssid, const char* password)
{
    if (ssid == NULL) {
        ESP_LOGE(TAG, "SSID不能为空");
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "连接到WiFi: %s", ssid);
    
    // 清除事件位
    xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT);
    
    wifi_config_t wifi_config = {0};
    strncpy((char*)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid) - 1);
    if (password != NULL) {
        strncpy((char*)wifi_config.sta.password, password, sizeof(wifi_config.sta.password) - 1);
    }
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_config.sta.pmf_cfg.capable = true;
    wifi_config.sta.pmf_cfg.required = false;
    
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    
    retry_count = 0;
    sta_state = WIFI_STATE_CONNECTING;
    esp_err_t ret = esp_wifi_connect();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "WiFi连接失败: %s", esp_err_to_name(ret));
        sta_state = WIFI_STATE_FAILED;
        return ret;
    }
    
    // 等待连接结果
    EventBits_t bits = xEventGroupWaitBits(wifi_event_group,
                                          WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                          pdFALSE,
                                          pdFALSE,
                                          30000 / portTICK_PERIOD_MS);
    
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "WiFi连接成功");
        return ESP_OK;
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "WiFi连接失败");
        return ESP_FAIL;
    } else {
        ESP_LOGE(TAG, "WiFi连接超时");
        return ESP_ERR_TIMEOUT;
    }
}

esp_err_t wifi_manager_start_ap(const char* ssid, const char* password)
{
    const char* ap_ssid = ssid ? ssid : persistent_config.ap_ssid;
    const char* ap_password = password ? password : persistent_config.ap_password;
    
    ESP_LOGI(TAG, "启动AP模式: %s", ap_ssid);
    
    // 扫描网络以选择最优信道
    if (ap_count == 0) {
        ESP_LOGI(TAG, "执行快速扫描以选择最优信道");
        wifi_manager_scan_networks();
        
        // 显示扫描到的WiFi信息
        ESP_LOGI(TAG, "=== 扫描到的WiFi网络信息 ===");
        if (ap_count > 0) {
            ESP_LOGI(TAG, "发现 %d 个WiFi网络:", ap_count);
            ESP_LOGI(TAG, "序号  SSID                          频段(信道)  信号强度(dBm)  加密方式");
            ESP_LOGI(TAG, "----  ----------------------------  ----------  ------------  --------");
            
            for (int i = 0; i < ap_count; i++) {
                const char* auth_mode;
                switch (ap_records[i].authmode) {
                    case WIFI_AUTH_OPEN:
                        auth_mode = "开放";
                        break;
                    case WIFI_AUTH_WEP:
                        auth_mode = "WEP";
                        break;
                    case WIFI_AUTH_WPA_PSK:
                        auth_mode = "WPA";
                        break;
                    case WIFI_AUTH_WPA2_PSK:
                        auth_mode = "WPA2";
                        break;
                    case WIFI_AUTH_WPA_WPA2_PSK:
                        auth_mode = "WPA/WPA2";
                        break;
                    case WIFI_AUTH_WPA3_PSK:
                        auth_mode = "WPA3";
                        break;
                    default:
                        auth_mode = "其他";
                        break;
                }
                
                // 安全地处理SSID字符串，确保以NULL结尾
                char ssid_str[33];  // WiFi SSID最大长度为32字节
                memset(ssid_str, 0, sizeof(ssid_str));
                memcpy(ssid_str, ap_records[i].ssid, 
                       ap_records[i].ssid[32] == 0 ? strlen((char*)ap_records[i].ssid) : 32);
                
                ESP_LOGI(TAG, "%4d  %-28s  %2d (CH%d)      %4d dBm       %s", 
                        i + 1, 
                        ssid_str,
                        ap_records[i].primary,
                        ap_records[i].primary,
                        ap_records[i].rssi,
                        auth_mode);
            }
        } else {
            ESP_LOGI(TAG, "未发现任何WiFi网络");
        }
        ESP_LOGI(TAG, "=== 扫描信息显示完成 ===");
    }
    
    // 选择最优信道
    uint8_t best_channel = wifi_manager_find_best_channel();
    
    wifi_config_t wifi_config = {
        .ap = {
            .channel = best_channel,
            .max_connection = WIFI_MANAGER_MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .required = false,
            },
        },
    };
    
    strncpy((char*)wifi_config.ap.ssid, ap_ssid, sizeof(wifi_config.ap.ssid) - 1);
    wifi_config.ap.ssid_len = strlen((char*)wifi_config.ap.ssid);
    
    if (ap_password && strlen(ap_password) > 0) {
        strncpy((char*)wifi_config.ap.password, ap_password, sizeof(wifi_config.ap.password) - 1);
    } else {
        strncpy((char*)wifi_config.ap.password, WIFI_MANAGER_DEFAULT_PASSWORD, sizeof(wifi_config.ap.password) - 1);
    }
    
    if (strlen((char*)wifi_config.ap.password) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }
    
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    
    // 更新持久化配置中的信道
    persistent_config.ap_channel = best_channel;
    
    ESP_LOGI(TAG, "AP启动完成 - SSID: %s, 信道: %d, 密码: %s", 
            wifi_config.ap.ssid, 
            best_channel,
            wifi_config.ap.authmode == WIFI_AUTH_OPEN ? "无" : (char*)wifi_config.ap.password);
    
    return ESP_OK;
}

esp_err_t wifi_manager_stop_ap(void)
{
    ESP_LOGI(TAG, "停止AP模式");
    // AP会在模式切换时自动停止
    return ESP_OK;
}

esp_err_t wifi_manager_disconnect_sta(void)
{
    ESP_LOGI(TAG, "断开STA连接");
    sta_state = WIFI_STATE_DISCONNECTED;
    return esp_wifi_disconnect();
}

wifi_connection_state_t wifi_manager_get_sta_state(void)
{
    return sta_state;
}

wifi_manager_mode_t wifi_manager_get_current_mode(void)
{
    return current_mode;
}

esp_err_t wifi_manager_get_sta_ip(char* ip_str, size_t max_len)
{
    if (ip_str == NULL || max_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (netif_sta == NULL) {
        ESP_LOGE(TAG, "STA网络接口未初始化");
        return ESP_ERR_INVALID_STATE;
    }
    
    esp_netif_ip_info_t ip_info;
    esp_err_t ret = esp_netif_get_ip_info(netif_sta, &ip_info);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "获取IP信息失败: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 检查是否获取到有效IP地址
    if (ip_info.ip.addr == 0) {
        //ESP_LOGW(TAG, "未获取到有效IP地址");
        snprintf(ip_str, max_len, "未分配");
        return ESP_ERR_INVALID_STATE;
    }
    
    // 格式化IP地址字符串
    snprintf(ip_str, max_len, IPSTR, IP2STR(&ip_info.ip));
    
    return ESP_OK;
}

esp_err_t wifi_manager_get_scan_results(wifi_ap_record_t* ap_records_out, uint16_t* ap_count_out)
{
    if (ap_records_out == NULL || ap_count_out == NULL) {
        ESP_LOGE(TAG, "获取扫描结果的参数无效");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (ap_count == 0) {
        ESP_LOGW(TAG, "没有可用的扫描结果");
        *ap_count_out = 0;
        return ESP_OK;
    }
    
    // 复制扫描结果到输出缓冲区
    uint16_t copy_count = (*ap_count_out < ap_count) ? *ap_count_out : ap_count;
    memcpy(ap_records_out, ap_records, copy_count * sizeof(wifi_ap_record_t));
    *ap_count_out = copy_count;
    
    ESP_LOGI(TAG, "返回 %d 个扫描结果", copy_count);
    return ESP_OK;
}

esp_err_t wifi_manager_get_mac_suffix(char* suffix, size_t max_len)
{
    if (suffix == NULL || max_len < 5) {
        return ESP_ERR_INVALID_ARG;
    }
    
    uint8_t mac_addr[6];
    esp_err_t ret = esp_read_mac(mac_addr, ESP_MAC_WIFI_STA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "获取MAC地址失败: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 提取MAC地址后两个字节（4位十六进制）
    snprintf(suffix, max_len, "%02X%02X", mac_addr[4], mac_addr[5]);
    
    ESP_LOGI(TAG, "MAC地址后四位: %s", suffix);
    return ESP_OK;
}

esp_err_t wifi_manager_get_default_config(wifi_config_persistent_t* config)
{
    return wifi_manager_get_default_config_with_mode(config, true); // 默认启用WiFi
}

esp_err_t wifi_manager_get_default_config_with_mode(wifi_config_persistent_t* config, bool enable_wifi)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    memset(config, 0, sizeof(wifi_config_persistent_t));
    
    // 生成带MAC后缀的默认AP名称，而不是使用基础名称
    char mac_suffix[5];
    esp_err_t mac_ret = wifi_manager_get_mac_suffix(mac_suffix, sizeof(mac_suffix));
    if (mac_ret == ESP_OK) {
        snprintf(config->ap_ssid, sizeof(config->ap_ssid), "%s_%s", 
                WIFI_MANAGER_DEFAULT_SSID, mac_suffix);
    } else {
        strcpy(config->ap_ssid, WIFI_MANAGER_DEFAULT_SSID);
    }
    
    strcpy(config->ap_password, WIFI_MANAGER_DEFAULT_PASSWORD);
    config->ap_channel = WIFI_MANAGER_DEFAULT_CHANNEL;
    
    // 根据参数决定是否启用WiFi功能
    if (enable_wifi) {
        // 默认启用APSTA模式（枚举值为0）
        config->sta_enabled = true;
        config->ap_enabled = true;
        config->mode = WIFI_MODE_AP_STA;  // 0 = APSTA模式
        ESP_LOGI(TAG, "默认配置: AP_SSID=%s, 模式=APSTA (枚举值=%d)", config->ap_ssid, config->mode);
    } else {
        // 完全禁用WiFi功能（枚举值为3）
        config->sta_enabled = false;
        config->ap_enabled = false;
        config->mode = WIFI_MODE_OFF;  // 3 = OFF模式
        ESP_LOGI(TAG, "默认配置: AP_SSID=%s, 模式=OFF (枚举值=%d, WiFi已禁用)", config->ap_ssid, config->mode);
    }
    
    // 设置默认AP IP配置
    strcpy(config->ap_ip, "192.168.4.1");
    strcpy(config->ap_gateway, "192.168.4.1");
    strcpy(config->ap_netmask, "255.255.255.0");
    
    return ESP_OK;
}

uint8_t wifi_manager_find_best_channel(void)
{
    ESP_LOGI(TAG, "扫描信道以寻找最优信道");
    
    // 统计各信道上的网络数量
    uint8_t channel_count[14] = {0}; // 信道1-13 (下标0-12对应信道1-13)
    
    for (int i = 0; i < ap_count; i++) {
        uint8_t ch = ap_records[i].primary;
        if (ch >= 1 && ch <= 13) {
            channel_count[ch - 1]++;
        }
    }
    
    // 寻找使用最少的信道
    uint8_t best_channel = 1;
    uint8_t min_count = channel_count[0];
    
    for (int i = 1; i < 13; i++) {
        if (channel_count[i] < min_count) {
            min_count = channel_count[i];
            best_channel = i + 1;
        }
    }
    
    ESP_LOGI(TAG, "最优信道: %d (检测到 %d 个网络)", best_channel, min_count);
    return best_channel;
}

esp_err_t wifi_manager_save_config(const wifi_config_persistent_t* config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "打开NVS失败: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = nvs_set_blob(nvs_handle, NVS_CONFIG_KEY, config, sizeof(wifi_config_persistent_t));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "保存WiFi配置失败: %s", esp_err_to_name(ret));
        nvs_close(nvs_handle);
        return ret;
    }
    
    ret = nvs_commit(nvs_handle);
    nvs_close(nvs_handle);
    
    if (ret == ESP_OK) {
        // 更新内存中的配置
        memcpy(&persistent_config, config, sizeof(wifi_config_persistent_t));
        ESP_LOGI(TAG, "WiFi配置已保存到NVS");
    } else {
        ESP_LOGE(TAG, "提交NVS失败: %s", esp_err_to_name(ret));
    }
    
    return ret;
}

esp_err_t wifi_manager_load_config(wifi_config_persistent_t* config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "打开NVS失败，使用默认配置: %s", esp_err_to_name(ret));
        return wifi_manager_get_default_config(config);
    }
    
    size_t required_size = sizeof(wifi_config_persistent_t);
    ret = nvs_get_blob(nvs_handle, NVS_CONFIG_KEY, config, &required_size);
    nvs_close(nvs_handle);
    
    if (ret == ESP_OK) {
        // 更新内存中的配置
        memcpy(&persistent_config, config, sizeof(wifi_config_persistent_t));
        ESP_LOGI(TAG, "从NVS加载WiFi配置成功");
        ESP_LOGI(TAG, "  AP_SSID: %s", config->ap_ssid);
        ESP_LOGI(TAG, "  STA_SSID: %s", config->sta_ssid);
        const char* mode_str = (config->mode == WIFI_MODE_OFF) ? "OFF" : 
                              (config->mode == WIFI_MODE_AP_STA) ? "APSTA" :
                              (config->mode == WIFI_MODE_STA_ONLY) ? "STA_ONLY" : "AP_ONLY";
        ESP_LOGI(TAG, "  模式: %s (枚举值=%d)", mode_str, config->mode);
    } else {
        ESP_LOGW(TAG, "从NVS加载配置失败，使用默认配置: %s", esp_err_to_name(ret));
        return wifi_manager_get_default_config(config);
    }
    
    return ESP_OK;
}

esp_err_t wifi_manager_set_ap_ip(const char* ip, const char* gateway, const char* netmask)
{
    if (!ip || !gateway || !netmask) {
        ESP_LOGE(TAG, "IP配置参数不能为空");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!netif_ap) {
        ESP_LOGE(TAG, "AP网络接口未初始化");
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "设置AP IP配置: IP=%s, 网关=%s, 子网掩码=%s", ip, gateway, netmask);
    
    // 记录当前STA连接状态
    wifi_connection_state_t original_sta_state = sta_state;
    wifi_config_t sta_config = {0};
    bool should_reconnect_sta = false;
    
    // 如果STA已连接，保存当前配置用于重连
    if (original_sta_state == WIFI_STATE_CONNECTED) {
        esp_err_t config_ret = esp_wifi_get_config(WIFI_IF_STA, &sta_config);
        if (config_ret == ESP_OK && strlen((char*)sta_config.sta.ssid) > 0) {
            should_reconnect_sta = true;
            ESP_LOGI(TAG, "保存STA配置用于重连: %s", sta_config.sta.ssid);
        }
    }
    
    // 停止DHCP服务器
    esp_netif_dhcps_stop(netif_ap);
    
    // 配置IP信息
    esp_netif_ip_info_t ip_info;
    memset(&ip_info, 0, sizeof(esp_netif_ip_info_t));
    
    // 转换IP地址
    ip_info.ip.addr = esp_ip4addr_aton(ip);
    ip_info.gw.addr = esp_ip4addr_aton(gateway);
    ip_info.netmask.addr = esp_ip4addr_aton(netmask);
    
    if (ip_info.ip.addr == 0 || ip_info.gw.addr == 0 || ip_info.netmask.addr == 0) {
        ESP_LOGE(TAG, "无效的IP地址格式");
        return ESP_ERR_INVALID_ARG;
    }
    
    // 设置静态IP
    esp_err_t ret = esp_netif_set_ip_info(netif_ap, &ip_info);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "设置AP IP信息失败: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 重新启动DHCP服务器
    ret = esp_netif_dhcps_start(netif_ap);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "启动DHCP服务器失败: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 如果之前STA已连接，尝试重新连接
    if (should_reconnect_sta) {
        ESP_LOGI(TAG, "尝试重新连接STA: %s", sta_config.sta.ssid);
        vTaskDelay(pdMS_TO_TICKS(1000)); // 等待AP稳定
        
        // 重新设置STA配置并连接
        esp_wifi_set_config(WIFI_IF_STA, &sta_config);
        esp_wifi_connect();
    }
    
    // 保存AP IP配置到NVS
    strcpy(persistent_config.ap_ip, ip);
    strcpy(persistent_config.ap_gateway, gateway);
    strcpy(persistent_config.ap_netmask, netmask);
    
    esp_err_t save_ret = wifi_manager_save_config(&persistent_config);
    if (save_ret == ESP_OK) {
        ESP_LOGI(TAG, "AP IP配置已保存到NVS");
    } else {
        ESP_LOGW(TAG, "保存AP IP配置失败: %s", esp_err_to_name(save_ret));
    }
    
    ESP_LOGI(TAG, "AP IP配置设置成功");
    return ESP_OK;
}

esp_err_t wifi_manager_get_ap_ip(char* ip_str, size_t max_len)
{
    if (ip_str == NULL || max_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (netif_ap == NULL) {
        ESP_LOGE(TAG, "AP网络接口未初始化");
        return ESP_ERR_INVALID_STATE;
    }
    
    esp_netif_ip_info_t ip_info;
    esp_err_t ret = esp_netif_get_ip_info(netif_ap, &ip_info);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "获取AP IP信息失败: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 格式化IP地址字符串
    snprintf(ip_str, max_len, IPSTR, IP2STR(&ip_info.ip));
    
    return ESP_OK;
}

esp_err_t wifi_manager_apply_config(void)
{
    ESP_LOGI(TAG, "应用保存的WiFi配置");
    
    wifi_config_persistent_t config;
    esp_err_t ret = wifi_manager_load_config(&config);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "无法加载WiFi配置，使用默认配置");
        ret = wifi_manager_get_default_config(&config);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "获取默认配置失败");
            return ret;
        }
    }
    
    // 根据配置设置WiFi模式
    if (config.sta_enabled && config.ap_enabled) {
        ret = wifi_manager_set_mode(WIFI_MODE_AP_STA);
    } else if (config.sta_enabled) {
        ret = wifi_manager_set_mode(WIFI_MODE_STA_ONLY);
    } else if (config.ap_enabled) {
        ret = wifi_manager_set_mode(WIFI_MODE_AP_ONLY);
    } else {
        ret = wifi_manager_set_mode(WIFI_MODE_OFF);
        ESP_LOGI(TAG, "WiFi配置显示所有功能都已禁用");
        return ESP_OK;
    }
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "设置WiFi模式失败: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 启动AP（如果启用）
    if (config.ap_enabled) {
        ret = wifi_manager_start_ap(config.ap_ssid, config.ap_password);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "启动AP失败: %s", esp_err_to_name(ret));
        } else {
            ESP_LOGI(TAG, "✓ AP已启动: %s", config.ap_ssid);
            
            // 应用保存的AP IP配置（如果不是默认值）
            if (strlen(config.ap_ip) > 0 && 
                strcmp(config.ap_ip, "192.168.4.1") != 0) {
                
                ESP_LOGI(TAG, "应用保存的AP IP配置: %s", config.ap_ip);
                
                // 直接设置网络接口，避免递归保存配置
                if (netif_ap) {
                    esp_netif_dhcps_stop(netif_ap);
                    
                    esp_netif_ip_info_t ip_info;
                    memset(&ip_info, 0, sizeof(esp_netif_ip_info_t));
                    ip_info.ip.addr = esp_ip4addr_aton(config.ap_ip);
                    ip_info.gw.addr = esp_ip4addr_aton(config.ap_gateway);
                    ip_info.netmask.addr = esp_ip4addr_aton(config.ap_netmask);
                    
                    esp_err_t ip_ret = esp_netif_set_ip_info(netif_ap, &ip_info);
                    if (ip_ret == ESP_OK) {
                        esp_netif_dhcps_start(netif_ap);
                        ESP_LOGI(TAG, "✓ AP IP配置已应用");
                    } else {
                        ESP_LOGW(TAG, "应用AP IP配置失败: %s", esp_err_to_name(ip_ret));
                    }
                }
            }
        }
    }
    
    // 连接STA（如果启用且有配置）
    if (config.sta_enabled && strlen(config.sta_ssid) > 0) {
        ret = wifi_manager_connect_sta(config.sta_ssid, config.sta_password);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "STA连接失败: %s", esp_err_to_name(ret));
        } else {
            ESP_LOGI(TAG, "✓ STA连接启动: %s", config.sta_ssid);
        }
    }
    
    ESP_LOGI(TAG, "WiFi配置应用完成");
    return ESP_OK;
}

esp_err_t wifi_manager_get_current_config(wifi_config_persistent_t* config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // 返回内存中的配置副本，避免频繁NVS读取
    memcpy(config, &persistent_config, sizeof(wifi_config_persistent_t));
    return ESP_OK;
} 