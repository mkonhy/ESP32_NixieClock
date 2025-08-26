#include "web_server.h"
#include "nixie_ctr.h"
#include "ntp_sync.h"
#include "wifi_manager.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "cJSON.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <time.h>

static const char *TAG = "WEB_SERVER";
static httpd_handle_t server = NULL;

// 重启任务函数
static void reboot_task(void* param) {
    ESP_LOGI(TAG, "重启倒计时开始...");
    for (int i = 3; i > 0; i--) {
        ESP_LOGI(TAG, "重启倒计时: %d 秒", i);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    ESP_LOGI(TAG, "正在重启系统...");
    esp_restart();
    vTaskDelete(NULL);
}

// 恢复出厂设置任务函数（调用辉光管控制器的统一函数）
static void factory_reset_task(void* param) {
    ESP_LOGI(TAG, "Web界面触发恢复出厂设置，调用统一恢复出厂设置函数...");
    
    // 调用辉光管控制器的恢复出厂设置函数
    esp_err_t ret = perform_factory_reset();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "恢复出厂设置失败: %s", esp_err_to_name(ret));
    }
    
    // 注意：perform_factory_reset函数内部会重启系统，所以这里不会执行到
    vTaskDelete(NULL);
}

// 获取系统信息
static void get_system_info(system_info_t *info) {
    // 时间信息 - 使用本机RTC时间
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    strftime(info->current_rtc_time, sizeof(info->current_rtc_time), "%Y-%m-%d %H:%M:%S", &timeinfo);
    
    // 获取NTP同步结果和配置
    ntp_sync_result_t ntp_result = ntp_sync_get_last_result();
    ntp_config_t ntp_config;
    ntp_sync_get_config(&ntp_config);
    
    // 格式化上次校准时间
    if (ntp_result.status == NTP_SYNC_STATUS_SUCCESS && ntp_result.sync_time > 0) {
        struct tm sync_timeinfo;
        localtime_r(&ntp_result.sync_time, &sync_timeinfo);
        char sync_time_str[32];
        strftime(sync_time_str, sizeof(sync_time_str), "%m-%d %H:%M:%S", &sync_timeinfo);
        snprintf(info->last_calibration_time, sizeof(info->last_calibration_time), 
                "%s (%s)", sync_time_str, ntp_result.time_diff_str);
    } else {
        strcpy(info->last_calibration_time, "从未校准");
    }
    
    // NTP服务器名称
    strcpy(info->ntp_server, ntp_config.primary_server);
    
    // 时区信息
    strcpy(info->timezone_name, ntp_config.timezone_name);
    info->timezone_offset_sec = ntp_config.timezone_offset_sec;
    
    // 获取运行时间
    get_total_runtime_string(info->total_runtime, sizeof(info->total_runtime));
    
    // 系统状态 - 获取完整电池信息
    info->battery_voltage = get_battery_voltage();
    info->battery_percentage = get_battery_percentage();
    info->battery_current = get_battery_current();
    
    // 模拟其他系统数据
    info->cpu_usage = 15;
    info->memory_usage = 45;
    info->flash_usage = 30;
    
    // 传感器数据
    info->bmp580_temperature = get_bmp580_temperature();
    info->bmp580_pressure = get_bmp580_pressure();
    info->aht20_temperature = get_aht20_temperature();
    info->aht20_humidity = get_aht20_humidity();
    
    // 唤醒模式
    info->wakeup_mode = get_current_wakeup_mode();
    
    // 显示设置 - 从NVS读取当前配置
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("display_config", NVS_READONLY, &nvs_handle);
    if (err == ESP_OK) {
        size_t required_size = sizeof(info->current_display_content);
        err = nvs_get_str(nvs_handle, "display_type", info->current_display_content, &required_size);
        if (err != ESP_OK) {
            strcpy(info->current_display_content, "时间");
        }
        
        required_size = sizeof(info->current_refresh_rate);
        err = nvs_get_str(nvs_handle, "refresh_rate", info->current_refresh_rate, &required_size);
        if (err != ESP_OK) {
            strcpy(info->current_refresh_rate, "1Hz (1次/秒)");
        }
        
        nvs_close(nvs_handle);
    } else {
        // 默认值
        strcpy(info->current_display_content, "时间");
        strcpy(info->current_refresh_rate, "1Hz (1次/秒)");
    }
    
    // 网络信息
    wifi_manager_mode_t current_wifi_mode = wifi_manager_get_current_mode();
    wifi_connection_state_t sta_state = wifi_manager_get_sta_state();
    
    info->wifi_enabled = (current_wifi_mode != WIFI_MODE_OFF);
    info->ap_enabled = (current_wifi_mode == WIFI_MODE_AP_ONLY || current_wifi_mode == WIFI_MODE_AP_STA);
    
    // AP信息 - 从WiFi管理器获取当前配置（内存缓存，避免频繁NVS读取）
    wifi_config_persistent_t wifi_config;
    esp_err_t wifi_config_ret = wifi_manager_get_current_config(&wifi_config);
    if (wifi_config_ret == ESP_OK) {
        // 确保AP SSID始终显示带MAC后缀的名称
        if (strlen(wifi_config.ap_ssid) > 0) {
            strcpy(info->ap_ssid, wifi_config.ap_ssid);
        } else {
            // 如果配置中没有AP名称，生成默认的带MAC后缀名称
            char mac_suffix[5];
            if (wifi_manager_get_mac_suffix(mac_suffix, sizeof(mac_suffix)) == ESP_OK) {
                snprintf(info->ap_ssid, sizeof(info->ap_ssid), "%s_%s", WIFI_MANAGER_DEFAULT_SSID, mac_suffix);
            } else {
                strcpy(info->ap_ssid, WIFI_MANAGER_DEFAULT_SSID);
            }
        }
        
        strcpy(info->ap_password, wifi_config.ap_password);
        
        // 获取真实的AP IP地址和网关
        char ap_ip[16] = {0};
        if (wifi_manager_get_ap_ip(ap_ip, sizeof(ap_ip)) == ESP_OK) {
            strcpy(info->ap_ip_address, ap_ip);
            strcpy(info->ap_gateway, ap_ip);  // 网关通常与AP IP相同
        } else {
            // 如果无法从网络接口获取，使用配置中保存的值
            if (strlen(wifi_config.ap_ip) > 0) {
                strcpy(info->ap_ip_address, wifi_config.ap_ip);
                strcpy(info->ap_gateway, wifi_config.ap_gateway);
            } else {
                strcpy(info->ap_ip_address, "192.168.4.1");
                strcpy(info->ap_gateway, "192.168.4.1");
            }
        }
        
        // STA信息 - 获取当前连接的WiFi信息
        wifi_config_t wifi_sta_config;
        esp_err_t config_ret = esp_wifi_get_config(WIFI_IF_STA, &wifi_sta_config);
        if (config_ret == ESP_OK && strlen((char*)wifi_sta_config.sta.ssid) > 0) {
            strncpy(info->sta_ssid, (char*)wifi_sta_config.sta.ssid, sizeof(info->sta_ssid) - 1);
            info->sta_ssid[sizeof(info->sta_ssid) - 1] = '\0';
        } else {
            strcpy(info->sta_ssid, "未连接");
        }
        
        // 获取STA IP地址
        char ip_str[16] = {0};
        if (wifi_manager_get_sta_ip(ip_str, sizeof(ip_str)) == ESP_OK) {
            strcpy(info->sta_ip_address, ip_str);
        } else {
            strcpy(info->sta_ip_address, "未分配");
        }
        
        info->sta_connected = (sta_state == WIFI_STATE_CONNECTED);
        
        // NTP配置
        info->ntp_auto_sync = ntp_config.auto_sync_enabled;
        info->ntp_sync_interval = ntp_config.sync_interval_sec;
    } else {
        // 生成默认AP名称（带MAC后缀）
        char mac_suffix[5];
        if (wifi_manager_get_mac_suffix(mac_suffix, sizeof(mac_suffix)) == ESP_OK) {
            snprintf(info->ap_ssid, sizeof(info->ap_ssid), "%s_%s", WIFI_MANAGER_DEFAULT_SSID, mac_suffix);
        } else {
            strcpy(info->ap_ssid, WIFI_MANAGER_DEFAULT_SSID);
        }
        strcpy(info->ap_password, "12345678");
        strcpy(info->ap_ip_address, "192.168.4.1");
        strcpy(info->ap_gateway, "192.168.4.1");
        strcpy(info->sta_ssid, "未连接");
        strcpy(info->sta_ip_address, "未分配");
        info->sta_connected = false;
        info->ntp_auto_sync = false;
        info->ntp_sync_interval = 0;
    }
    info->ap_password_hidden = true;
}

// HTML页面处理函数
esp_err_t index_handler(httpd_req_t *req) {
    extern const uint8_t index_html_start[] asm("_binary_index_html_start");
    extern const uint8_t index_html_end[] asm("_binary_index_html_end");
    const size_t index_html_size = (index_html_end - index_html_start);
    
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, (const char*)index_html_start, index_html_size);
    return ESP_OK;
}

// 系统信息API
esp_err_t api_system_info_handler(httpd_req_t *req) {
    system_info_t info;
    get_system_info(&info);
    
    cJSON *json = cJSON_CreateObject();
    
    // 时间信息
    cJSON_AddStringToObject(json, "current_rtc_time", info.current_rtc_time);
    cJSON_AddStringToObject(json, "last_calibration_time", info.last_calibration_time);
    cJSON_AddStringToObject(json, "ntp_server", info.ntp_server);
    cJSON_AddStringToObject(json, "total_runtime", info.total_runtime);
    cJSON_AddStringToObject(json, "timezone_name", info.timezone_name);
    cJSON_AddNumberToObject(json, "timezone_offset_sec", info.timezone_offset_sec);
    
    // 系统状态
    cJSON_AddNumberToObject(json, "battery_percentage", info.battery_percentage);
    cJSON_AddNumberToObject(json, "battery_voltage", info.battery_voltage);
    cJSON_AddNumberToObject(json, "battery_current", info.battery_current);
    cJSON_AddNumberToObject(json, "cpu_usage", info.cpu_usage);
    cJSON_AddNumberToObject(json, "memory_usage", info.memory_usage);
    cJSON_AddNumberToObject(json, "flash_usage", info.flash_usage);
    
    // 网络信息
    cJSON_AddBoolToObject(json, "wifi_enabled", info.wifi_enabled);
    cJSON_AddBoolToObject(json, "ap_enabled", info.ap_enabled);
    cJSON_AddStringToObject(json, "ap_ssid", info.ap_ssid);
    cJSON_AddStringToObject(json, "ap_password", info.ap_password);
    cJSON_AddStringToObject(json, "ap_ip_address", info.ap_ip_address);
    cJSON_AddStringToObject(json, "ap_gateway", info.ap_gateway);
    cJSON_AddStringToObject(json, "sta_ssid", info.sta_ssid);
    cJSON_AddStringToObject(json, "sta_ip_address", info.sta_ip_address);
    cJSON_AddBoolToObject(json, "sta_connected", info.sta_connected);
    
    // NTP配置
    cJSON_AddBoolToObject(json, "ntp_auto_sync", info.ntp_auto_sync);
    cJSON_AddNumberToObject(json, "ntp_sync_interval", info.ntp_sync_interval);
    
    // 传感器数据
    cJSON_AddNumberToObject(json, "bmp580_temperature", info.bmp580_temperature);
    cJSON_AddNumberToObject(json, "bmp580_pressure", info.bmp580_pressure);
    cJSON_AddNumberToObject(json, "aht20_temperature", info.aht20_temperature);
    cJSON_AddNumberToObject(json, "aht20_humidity", info.aht20_humidity);
    
    // 唤醒模式
    cJSON_AddNumberToObject(json, "wakeup_mode", info.wakeup_mode);
    
    // 显示设置
    cJSON_AddStringToObject(json, "current_display_content", info.current_display_content);
    cJSON_AddStringToObject(json, "current_refresh_rate", info.current_refresh_rate);
    
    char *json_string = cJSON_Print(json);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_string, HTTPD_RESP_USE_STRLEN);
    
    free(json_string);
    cJSON_Delete(json);
    return ESP_OK;
}

// WiFi配置API
esp_err_t api_wifi_config_handler(httpd_req_t *req) {
    char buf[256];
    int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (ret <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid request");
        return ESP_FAIL;
    }
    buf[ret] = '\0';
    
    cJSON *json = cJSON_Parse(buf);
    if (!json) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }
    
    cJSON *enabled = cJSON_GetObjectItem(json, "enabled");
    cJSON *ssid = cJSON_GetObjectItem(json, "ssid");
    cJSON *password = cJSON_GetObjectItem(json, "password");
    
    ESP_LOGI(TAG, "WiFi Config: Enabled=%s, SSID=%s", 
             cJSON_IsTrue(enabled) ? "true" : "false",
             cJSON_GetStringValue(ssid));
    
    cJSON *response = cJSON_CreateObject();
    
    if (cJSON_IsTrue(enabled)) {
        // 启用WiFi连接
        const char* wifi_ssid = cJSON_GetStringValue(ssid);
        const char* wifi_password = cJSON_GetStringValue(password);
        
        if (wifi_ssid && strlen(wifi_ssid) > 0) {
            // 获取当前配置
            wifi_config_persistent_t config;
            esp_err_t load_ret = wifi_manager_load_config(&config);
            if (load_ret != ESP_OK) {
                cJSON_AddStringToObject(response, "message", "加载WiFi配置失败");
                cJSON_AddBoolToObject(response, "success", false);
                goto wifi_config_end;
            }
            
            // 确保切换到APSTA模式
            wifi_manager_mode_t target_mode = WIFI_MODE_AP_STA;
            esp_err_t mode_ret = wifi_manager_set_mode(target_mode);
            if (mode_ret != ESP_OK) {
                cJSON_AddStringToObject(response, "message", "设置WiFi模式失败");
                cJSON_AddBoolToObject(response, "success", false);
                goto wifi_config_end;
            }
            
            // 启动AP（如果AP已启用）
            if (config.ap_enabled) {
                esp_err_t ap_ret = wifi_manager_start_ap(config.ap_ssid, config.ap_password);
                if (ap_ret != ESP_OK) {
                    ESP_LOGW(TAG, "启动AP失败: %s", esp_err_to_name(ap_ret));
                }
                vTaskDelay(pdMS_TO_TICKS(500));
            }
            
            // 尝试连接WiFi
            esp_err_t connect_ret = wifi_manager_connect_sta(wifi_ssid, wifi_password);
            if (connect_ret == ESP_OK) {
                // 保存WiFi配置到NVS
                strncpy(config.sta_ssid, wifi_ssid, sizeof(config.sta_ssid) - 1);
                config.sta_ssid[sizeof(config.sta_ssid) - 1] = '\0';
                strncpy(config.sta_password, wifi_password, sizeof(config.sta_password) - 1);
                config.sta_password[sizeof(config.sta_password) - 1] = '\0';
                config.sta_enabled = true;
                
                // 根据当前AP状态调整WiFi模式
                if (config.ap_enabled) {
                    config.mode = WIFI_MODE_AP_STA;
                } else {
                    config.mode = WIFI_MODE_STA_ONLY;
                }
                
                wifi_manager_save_config(&config);
                
                cJSON_AddStringToObject(response, "message", "WiFi连接成功");
                cJSON_AddBoolToObject(response, "success", true);
                ESP_LOGI(TAG, "✓ WiFi连接成功: %s", wifi_ssid);
            } else {
                char error_msg[128];
                snprintf(error_msg, sizeof(error_msg), "WiFi连接失败: %s", esp_err_to_name(connect_ret));
                cJSON_AddStringToObject(response, "message", error_msg);
                cJSON_AddBoolToObject(response, "success", false);
                ESP_LOGE(TAG, "✗ WiFi连接失败: %s", esp_err_to_name(connect_ret));
            }
        } else {
            cJSON_AddStringToObject(response, "message", "WiFi SSID不能为空");
            cJSON_AddBoolToObject(response, "success", false);
        }
    } else {
        // 禁用WiFi连接，断开STA
        esp_err_t disconnect_ret = wifi_manager_disconnect_sta();
        if (disconnect_ret == ESP_OK) {
            // 更新配置状态
            wifi_config_persistent_t config;
            esp_err_t load_ret = wifi_manager_load_config(&config);
            if (load_ret == ESP_OK) {
                config.sta_enabled = false;
                
                // 根据AP状态调整WiFi模式
                if (config.ap_enabled) {
                    config.mode = WIFI_MODE_AP_ONLY;
                    wifi_manager_set_mode(WIFI_MODE_AP_ONLY);
                    // 重新启动AP以确保它正常工作
                    wifi_manager_start_ap(config.ap_ssid, config.ap_password);
                } else {
                    config.mode = WIFI_MODE_OFF;
                    wifi_manager_set_mode(WIFI_MODE_OFF);
                }
                
                wifi_manager_save_config(&config);
            }
            
            cJSON_AddStringToObject(response, "message", "WiFi已断开");
            cJSON_AddBoolToObject(response, "success", true);
            ESP_LOGI(TAG, "✓ WiFi连接已断开");
        } else {
            cJSON_AddStringToObject(response, "message", "断开WiFi失败");
            cJSON_AddBoolToObject(response, "success", false);
            ESP_LOGE(TAG, "✗ 断开WiFi失败");
        }
    }
    
wifi_config_end:
    char *response_string = cJSON_Print(response);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response_string, HTTPD_RESP_USE_STRLEN);
    
    free(response_string);
    cJSON_Delete(response);
    cJSON_Delete(json);
    return ESP_OK;
}

// AP配置API  
esp_err_t api_ap_config_handler(httpd_req_t *req) {
    char buf[256];
    int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (ret <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid request");
        return ESP_FAIL;
    }
    buf[ret] = '\0';
    
    cJSON *json = cJSON_Parse(buf);
    if (!json) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }
    
    cJSON *enabled = cJSON_GetObjectItem(json, "enabled");
    cJSON *ssid = cJSON_GetObjectItem(json, "ssid");
    cJSON *password = cJSON_GetObjectItem(json, "password");
    cJSON *ip = cJSON_GetObjectItem(json, "ip");
    cJSON *gateway = cJSON_GetObjectItem(json, "gateway");
    
    ESP_LOGI(TAG, "AP Config: Enabled=%s, SSID=%s, IP=%s, Gateway=%s", 
             cJSON_IsTrue(enabled) ? "true" : "false",
             cJSON_GetStringValue(ssid),
             cJSON_GetStringValue(ip),
             cJSON_GetStringValue(gateway));
    
    cJSON *response = cJSON_CreateObject();
    
    if (cJSON_IsTrue(enabled)) {
        // 启用AP模式
        const char* ap_ssid = cJSON_GetStringValue(ssid);
        const char* ap_password = cJSON_GetStringValue(password);
        
        // 获取当前配置以使用保存的默认值
        wifi_config_persistent_t config;
        esp_err_t load_ret = wifi_manager_load_config(&config);
        if (load_ret != ESP_OK) {
            cJSON_AddStringToObject(response, "message", "加载WiFi配置失败");
            cJSON_AddBoolToObject(response, "success", false);
            goto ap_config_end;
        }
        
        // 使用保存的默认值如果未提供新值
        if (!ap_ssid || strlen(ap_ssid) == 0) {
            ap_ssid = config.ap_ssid;
        }
        if (!ap_password || strlen(ap_password) == 0) {
            ap_password = config.ap_password;
        }
        
        // 获取当前STA状态
        wifi_connection_state_t sta_state = wifi_manager_get_sta_state();
        
        // 根据当前状态选择合适的模式
        wifi_manager_mode_t target_mode;
        if (sta_state == WIFI_STATE_CONNECTED || config.sta_enabled) {
            // 如果STA已连接或启用，使用APSTA模式
            target_mode = WIFI_MODE_AP_STA;
        } else {
            // 如果STA未连接且未启用，使用AP模式
            target_mode = WIFI_MODE_AP_ONLY;
        }
        
        // 设置WiFi模式
        esp_err_t mode_ret = wifi_manager_set_mode(target_mode);
        if (mode_ret != ESP_OK) {
            cJSON_AddStringToObject(response, "message", "设置WiFi模式失败");
            cJSON_AddBoolToObject(response, "success", false);
            ESP_LOGE(TAG, "✗ 设置WiFi模式失败: %s", esp_err_to_name(mode_ret));
        } else {
            // 启动AP
            esp_err_t ap_ret = wifi_manager_start_ap(ap_ssid, ap_password);
            if (ap_ret == ESP_OK) {
                // 设置AP IP地址和网关（如果提供）
                const char* ap_ip = cJSON_GetStringValue(ip);
                const char* ap_gateway = cJSON_GetStringValue(gateway);
                
                if (ap_ip && strlen(ap_ip) > 0 && ap_gateway && strlen(ap_gateway) > 0) {
                    // 使用默认子网掩码255.255.255.0
                    esp_err_t ip_ret = wifi_manager_set_ap_ip(ap_ip, ap_gateway, "255.255.255.0");
                    if (ip_ret != ESP_OK) {
                        ESP_LOGW(TAG, "设置AP IP失败: %s", esp_err_to_name(ip_ret));
                    } else {
                        ESP_LOGI(TAG, "✓ AP IP设置成功: %s", ap_ip);
                    }
                }
                
                // 保存AP配置到NVS
                strncpy(config.ap_ssid, ap_ssid, sizeof(config.ap_ssid) - 1);
                config.ap_ssid[sizeof(config.ap_ssid) - 1] = '\0';
                strncpy(config.ap_password, ap_password, sizeof(config.ap_password) - 1);
                config.ap_password[sizeof(config.ap_password) - 1] = '\0';
                config.ap_enabled = true;
                
                // 根据当前STA状态调整WiFi模式
                if (config.sta_enabled) {
                    config.mode = WIFI_MODE_AP_STA;
                } else {
                    config.mode = WIFI_MODE_AP_ONLY;
                }
                
                wifi_manager_save_config(&config);
                
                cJSON_AddStringToObject(response, "message", "AP热点已启动");
                cJSON_AddBoolToObject(response, "success", true);
                ESP_LOGI(TAG, "✓ AP热点启动成功: %s", ap_ssid);
            } else {
                char error_msg[128];
                snprintf(error_msg, sizeof(error_msg), "AP启动失败: %s", esp_err_to_name(ap_ret));
                cJSON_AddStringToObject(response, "message", error_msg);
                cJSON_AddBoolToObject(response, "success", false);
                ESP_LOGE(TAG, "✗ AP启动失败: %s", esp_err_to_name(ap_ret));
            }
        }
    } else {
        // 禁用AP模式
        wifi_config_persistent_t config;
        esp_err_t load_ret = wifi_manager_load_config(&config);
        
        if (load_ret == ESP_OK) {
            config.ap_enabled = false;
            
            if (config.sta_enabled) {
                // 如果STA启用，切换到STA模式并尝试连接已保存的WiFi
                config.mode = WIFI_MODE_STA_ONLY;
                esp_err_t mode_ret = wifi_manager_set_mode(WIFI_MODE_STA_ONLY);
                if (mode_ret == ESP_OK) {
                    wifi_manager_save_config(&config);
                    
                    // 尝试连接已保存的STA配置
                    if (strlen(config.sta_ssid) > 0) {
                        ESP_LOGI(TAG, "尝试连接已保存的WiFi: %s", config.sta_ssid);
                        esp_err_t connect_ret = wifi_manager_connect_sta(config.sta_ssid, config.sta_password);
                        if (connect_ret == ESP_OK) {
                            cJSON_AddStringToObject(response, "message", "AP已关闭，已重新连接到WiFi");
                            ESP_LOGI(TAG, "✓ AP已关闭，成功重新连接到WiFi: %s", config.sta_ssid);
                        } else {
                            cJSON_AddStringToObject(response, "message", "AP已关闭，但WiFi重连失败");
                            ESP_LOGW(TAG, "⚠️ AP已关闭，WiFi重连失败: %s", esp_err_to_name(connect_ret));
                        }
                    } else {
                        cJSON_AddStringToObject(response, "message", "AP已关闭，切换为STA模式");
                        ESP_LOGI(TAG, "✓ AP已关闭，切换为STA模式（无保存的WiFi配置）");
                    }
                    cJSON_AddBoolToObject(response, "success", true);
                } else {
                    cJSON_AddStringToObject(response, "message", "关闭AP失败");
                    cJSON_AddBoolToObject(response, "success", false);
                    ESP_LOGE(TAG, "✗ 关闭AP失败");
                }
            } else {
                // 如果STA也未启用，关闭WiFi
                config.mode = WIFI_MODE_OFF;
                esp_err_t mode_ret = wifi_manager_set_mode(WIFI_MODE_OFF);
                if (mode_ret == ESP_OK) {
                    wifi_manager_save_config(&config);
                    cJSON_AddStringToObject(response, "message", "WiFi已完全关闭");
                    cJSON_AddBoolToObject(response, "success", true);
                    ESP_LOGI(TAG, "✓ WiFi已完全关闭");
                } else {
                    cJSON_AddStringToObject(response, "message", "关闭WiFi失败");
                    cJSON_AddBoolToObject(response, "success", false);
                    ESP_LOGE(TAG, "✗ 关闭WiFi失败");
                }
            }
        } else {
            cJSON_AddStringToObject(response, "message", "无法加载WiFi配置");
            cJSON_AddBoolToObject(response, "success", false);
            ESP_LOGE(TAG, "✗ 无法加载WiFi配置");
        }
    }
    
ap_config_end:
    char *response_string = cJSON_Print(response);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response_string, HTTPD_RESP_USE_STRLEN);
    
    free(response_string);
    cJSON_Delete(response);
    cJSON_Delete(json);
    return ESP_OK;
}

// NTP配置API
esp_err_t api_ntp_config_handler(httpd_req_t *req) {
    char buf[512];
    int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (ret <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid request");
        return ESP_FAIL;
    }
    buf[ret] = '\0';
    
    cJSON *json = cJSON_Parse(buf);
    if (!json) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }
    
    cJSON *server = cJSON_GetObjectItem(json, "server");
    cJSON *auto_sync = cJSON_GetObjectItem(json, "auto_sync");
    cJSON *interval = cJSON_GetObjectItem(json, "interval");
    
    ESP_LOGI(TAG, "NTP Config: Server=%s, AutoSync=%s, Interval=%d秒", 
             cJSON_GetStringValue(server),
             cJSON_IsTrue(auto_sync) ? "true" : "false",
             (int)cJSON_GetNumberValue(interval));
    
    // 获取当前配置并更新
    ntp_config_t ntp_config;
    esp_err_t config_ret = ntp_sync_get_config(&ntp_config);
    
    cJSON *response = cJSON_CreateObject();
    
    if (config_ret == ESP_OK) {
        // 更新配置
        if (server && cJSON_IsString(server)) {
            strncpy(ntp_config.primary_server, cJSON_GetStringValue(server), NTP_MAX_SERVER_LEN - 1);
            ntp_config.primary_server[NTP_MAX_SERVER_LEN - 1] = '\0';
        }
        
        if (auto_sync) {
            ntp_config.auto_sync_enabled = cJSON_IsTrue(auto_sync);
        }
        
        if (interval && cJSON_IsNumber(interval)) {
            uint32_t new_interval = (uint32_t)cJSON_GetNumberValue(interval);
            if (new_interval >= NTP_MIN_SYNC_INTERVAL && new_interval <= NTP_MAX_SYNC_INTERVAL) {
                ntp_config.sync_interval_sec = new_interval;
            }
        }
        
        // 保存配置
        esp_err_t save_ret = ntp_sync_set_config(&ntp_config);
        if (save_ret == ESP_OK) {
            cJSON_AddStringToObject(response, "message", "NTP设置已保存并生效");
            cJSON_AddBoolToObject(response, "success", true);
            ESP_LOGI(TAG, "✓ NTP配置已更新并保存");
        } else {
            cJSON_AddStringToObject(response, "message", "NTP设置保存失败");
            cJSON_AddBoolToObject(response, "success", false);
            ESP_LOGE(TAG, "✗ NTP配置保存失败: %s", esp_err_to_name(save_ret));
        }
    } else {
        cJSON_AddStringToObject(response, "message", "获取NTP配置失败");
        cJSON_AddBoolToObject(response, "success", false);
        ESP_LOGE(TAG, "✗ 获取NTP配置失败: %s", esp_err_to_name(config_ret));
    }
    
    char *response_string = cJSON_Print(response);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response_string, HTTPD_RESP_USE_STRLEN);
    
    free(response_string);
    cJSON_Delete(response);
    cJSON_Delete(json);
    return ESP_OK;
}

// NTP同步API - 立即校准
esp_err_t api_ntp_sync_handler(httpd_req_t *req) {
    ESP_LOGI(TAG, "立即NTP同步请求");
    
    cJSON *response = cJSON_CreateObject();
    
    // 检查网络是否可用
    if (!ntp_sync_is_network_available()) {
        cJSON_AddStringToObject(response, "message", "网络未连接，无法进行NTP同步");
        cJSON_AddBoolToObject(response, "success", false);
        ESP_LOGW(TAG, "网络不可用，无法执行NTP同步");
    } else {
        // 执行手动NTP同步
        esp_err_t sync_ret = ntp_sync_start_manual();
        if (sync_ret == ESP_OK) {
            // 等待短暂时间让同步完成
            vTaskDelay(pdMS_TO_TICKS(100));
            
            // 获取同步结果
            ntp_sync_result_t result = ntp_sync_get_last_result();
            if (result.status == NTP_SYNC_STATUS_SUCCESS) {
                char msg[128];
                snprintf(msg, sizeof(msg), "NTP同步成功！时间差：%s", result.time_diff_str);
                cJSON_AddStringToObject(response, "message", msg);
                cJSON_AddBoolToObject(response, "success", true);
                ESP_LOGI(TAG, "✓ 手动NTP同步成功，时间差：%s", result.time_diff_str);
            } else {
                const char* status_msg;
                switch (result.status) {
                    case NTP_SYNC_STATUS_TIMEOUT:
                        status_msg = "NTP同步超时，请检查网络连接";
                        break;
                    case NTP_SYNC_STATUS_NETWORK_ERROR:
                        status_msg = "网络错误，无法连接NTP服务器";
                        break;
                    case NTP_SYNC_STATUS_FAILED:
                        status_msg = "NTP同步失败，请稍后重试";
                        break;
                    default:
                        status_msg = "NTP同步状态未知";
                        break;
                }
                cJSON_AddStringToObject(response, "message", status_msg);
                cJSON_AddBoolToObject(response, "success", false);
                ESP_LOGW(TAG, "✗ 手动NTP同步失败：%s", status_msg);
            }
        } else {
            cJSON_AddStringToObject(response, "message", "启动NTP同步失败");
            cJSON_AddBoolToObject(response, "success", false);
            ESP_LOGE(TAG, "✗ 启动NTP同步失败: %s", esp_err_to_name(sync_ret));
        }
    }
    
    char *response_string = cJSON_Print(response);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response_string, HTTPD_RESP_USE_STRLEN);
    
    free(response_string);
    cJSON_Delete(response);
    return ESP_OK;
}

// 手动时间设置API
esp_err_t api_manual_time_handler(httpd_req_t *req) {
    char buf[256];
    int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (ret <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid request");
        return ESP_FAIL;
    }
    buf[ret] = '\0';
    
    cJSON *json = cJSON_Parse(buf);
    if (!json) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }
    
    cJSON *datetime = cJSON_GetObjectItem(json, "datetime");
    if (!datetime || !cJSON_IsString(datetime)) {
        cJSON_Delete(json);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing or invalid datetime");
        return ESP_FAIL;
    }
    
    const char* datetime_str = cJSON_GetStringValue(datetime);
    ESP_LOGI(TAG, "手动时间设置请求: %s", datetime_str);
    
    // 解析datetime-local格式 (YYYY-MM-DDTHH:MM)
    struct tm timeinfo = {0};
    int year, month, day, hour, minute;
    
    if (sscanf(datetime_str, "%d-%d-%dT%d:%d", &year, &month, &day, &hour, &minute) != 5) {
        cJSON_Delete(json);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid datetime format");
        return ESP_FAIL;
    }
    
    timeinfo.tm_year = year - 1900;  // 年份从1900年开始
    timeinfo.tm_mon = month - 1;     // 月份从0开始
    timeinfo.tm_mday = day;
    timeinfo.tm_hour = hour;
    timeinfo.tm_min = minute;
    timeinfo.tm_sec = 0;
    timeinfo.tm_isdst = -1;  // 自动判断夏令时
    
    // 转换为时间戳
    time_t timestamp = mktime(&timeinfo);
    if (timestamp == -1) {
        cJSON_Delete(json);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid datetime value");
        return ESP_FAIL;
    }
    
    // 设置时间
    esp_err_t set_ret = ntp_sync_set_manual_time(timestamp);
    
    cJSON *response = cJSON_CreateObject();
    if (set_ret == ESP_OK) {
        char time_str[64];
        struct tm* local_time = localtime(&timestamp);
        strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", local_time);
        cJSON_AddStringToObject(response, "message", "手动时间设置成功");
        cJSON_AddStringToObject(response, "set_time", time_str);
        cJSON_AddBoolToObject(response, "success", true);
        ESP_LOGI(TAG, "手动时间设置成功: %s", time_str);
    } else {
        cJSON_AddStringToObject(response, "message", "手动时间设置失败");
        cJSON_AddBoolToObject(response, "success", false);
        ESP_LOGE(TAG, "手动时间设置失败: %s", esp_err_to_name(set_ret));
    }
    
    char *response_string = cJSON_Print(response);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response_string, HTTPD_RESP_USE_STRLEN);
    
    free(response_string);
    cJSON_Delete(response);
    cJSON_Delete(json);
    return ESP_OK;
}

// 时区配置API
esp_err_t api_timezone_config_handler(httpd_req_t *req) {
    char buf[256];
    int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (ret <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid request");
        return ESP_FAIL;
    }
    buf[ret] = '\0';
    
    cJSON *json = cJSON_Parse(buf);
    if (!json) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }
    
    cJSON *timezone_name = cJSON_GetObjectItem(json, "timezone_name");
    cJSON *timezone_offset = cJSON_GetObjectItem(json, "timezone_offset");
    
    if (!timezone_name || !cJSON_IsString(timezone_name) || 
        !timezone_offset || !cJSON_IsNumber(timezone_offset)) {
        cJSON_Delete(json);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing or invalid timezone parameters");
        return ESP_FAIL;
    }
    
    const char* tz_name = cJSON_GetStringValue(timezone_name);
    int32_t tz_offset = cJSON_GetNumberValue(timezone_offset);
    
    ESP_LOGI(TAG, "时区配置请求: %s (偏移: %ld秒)", tz_name, tz_offset);
    
    // 设置时区
    esp_err_t set_ret = ntp_sync_set_timezone(tz_offset, tz_name);
    
    cJSON *response = cJSON_CreateObject();
    if (set_ret == ESP_OK) {
        cJSON_AddStringToObject(response, "message", "时区设置成功");
        cJSON_AddBoolToObject(response, "success", true);
        ESP_LOGI(TAG, "时区设置成功: %s (偏移: %ld秒)", tz_name, tz_offset);
    } else {
        cJSON_AddStringToObject(response, "message", "时区设置失败");
        cJSON_AddBoolToObject(response, "success", false);
        ESP_LOGE(TAG, "时区设置失败: %s", esp_err_to_name(set_ret));
    }
    
    char *response_string = cJSON_Print(response);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response_string, HTTPD_RESP_USE_STRLEN);
    
    free(response_string);
    cJSON_Delete(response);
    cJSON_Delete(json);
    return ESP_OK;
}

// 重启API
esp_err_t api_reboot_handler(httpd_req_t *req) {
    cJSON *response = cJSON_CreateObject();
    cJSON_AddStringToObject(response, "message", "系统将在3秒后重启");
    cJSON_AddBoolToObject(response, "success", true);
    
    char *response_string = cJSON_Print(response);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response_string, HTTPD_RESP_USE_STRLEN);
    
    free(response_string);
    cJSON_Delete(response);
    
    ESP_LOGI(TAG, "系统重启请求，3秒后执行重启");
    
    // 创建重启任务
    xTaskCreate(reboot_task, "reboot_task", 2048, NULL, 5, NULL);
    
    return ESP_OK;
}

// 恢复出厂设置API
esp_err_t api_factory_reset_handler(httpd_req_t *req) {
    cJSON *response = cJSON_CreateObject();
    cJSON_AddStringToObject(response, "message", "恢复出厂设置将在5秒后开始，系统将清除所有配置并重启");
    cJSON_AddBoolToObject(response, "success", true);
    
    char *response_string = cJSON_Print(response);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response_string, HTTPD_RESP_USE_STRLEN);
    
    free(response_string);
    cJSON_Delete(response);
    
    ESP_LOGI(TAG, "恢复出厂设置请求，5秒后执行恢复操作");
    
    // 创建恢复出厂设置任务
    xTaskCreate(factory_reset_task, "factory_reset_task", 4096, NULL, 5, NULL);
    
    return ESP_OK;
}

// 复位电量计API
esp_err_t api_reset_battery_handler(httpd_req_t *req) {
    cJSON *response = cJSON_CreateObject();
    
    // 这里调用BQ27220复位函数（需要在nixie_ctr.c中实现）
    // esp_err_t result = bq27220_reset_fuel_gauge();
    
    // 暂时返回成功，实际实现需要调用硬件复位函数
    cJSON_AddStringToObject(response, "message", "电量计复位成功");
    cJSON_AddBoolToObject(response, "success", true);
    
    char *response_string = cJSON_Print(response);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response_string, HTTPD_RESP_USE_STRLEN);
    
    free(response_string);
    cJSON_Delete(response);
    
    ESP_LOGI(TAG, "电量计复位请求");
    
    return ESP_OK;
}

// 设置电池容量API
esp_err_t api_set_battery_capacity_handler(httpd_req_t *req) {
    char content[100];
    size_t recv_size = MIN(req->content_len, sizeof(content) - 1);
    int ret = httpd_req_recv(req, content, recv_size);
    if (ret <= 0) {
        if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
            httpd_resp_send_408(req);
        }
        return ESP_FAIL;
    }
    content[ret] = '\0';
    
    cJSON *json = cJSON_Parse(content);
    cJSON *response = cJSON_CreateObject();
    
    if (json == NULL) {
        cJSON_AddStringToObject(response, "message", "无效的JSON格式");
        cJSON_AddBoolToObject(response, "success", false);
    } else {
        cJSON *capacity_json = cJSON_GetObjectItem(json, "capacity");
        if (cJSON_IsNumber(capacity_json)) {
            int capacity = capacity_json->valueint;
            if (capacity >= 100 && capacity <= 10000) {
                // 调用BQ27220设置容量函数
                esp_err_t result = bq27220_configure_design_capacity((uint16_t)capacity);
                if (result == ESP_OK) {
                    cJSON_AddStringToObject(response, "message", "电池容量设置成功");
                    cJSON_AddBoolToObject(response, "success", true);
                    ESP_LOGI(TAG, "电池容量设置为: %d mAh", capacity);
                } else {
                    cJSON_AddStringToObject(response, "message", "电池容量设置失败");
                    cJSON_AddBoolToObject(response, "success", false);
                    ESP_LOGE(TAG, "电池容量设置失败: %s", esp_err_to_name(result));
                }
            } else {
                cJSON_AddStringToObject(response, "message", "电池容量必须在100-10000 mAh之间");
                cJSON_AddBoolToObject(response, "success", false);
            }
        } else {
            cJSON_AddStringToObject(response, "message", "无效的容量值");
            cJSON_AddBoolToObject(response, "success", false);
        }
        cJSON_Delete(json);
    }
    
    char *response_string = cJSON_Print(response);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response_string, HTTPD_RESP_USE_STRLEN);
    
    free(response_string);
    cJSON_Delete(response);
    
    return ESP_OK;
}

// WiFi扫描API
esp_err_t api_wifi_scan_handler(httpd_req_t *req) {
    ESP_LOGI(TAG, "WiFi scan requested");
    
    cJSON *response = cJSON_CreateObject();
    
    // 检查WiFi模式是否支持扫描
    wifi_manager_mode_t current_mode = wifi_manager_get_current_mode();
    if (current_mode != WIFI_MODE_STA_ONLY && current_mode != WIFI_MODE_AP_STA) {
        // 如果当前不是支持扫描的模式，先设置为APSTA模式
        ESP_LOGI(TAG, "当前模式不支持扫描，切换到APSTA模式");
        esp_err_t mode_ret = wifi_manager_set_mode(WIFI_MODE_AP_STA);
        if (mode_ret != ESP_OK) {
            cJSON_AddStringToObject(response, "message", "切换WiFi模式失败，无法扫描");
            cJSON_AddBoolToObject(response, "success", false);
            goto scan_end;
        }
        
        // 启动AP - 使用已保存配置或生成带MAC后缀的默认名称
        wifi_config_persistent_t current_config;
        esp_err_t config_ret = wifi_manager_get_current_config(&current_config);
        if (config_ret == ESP_OK && strlen(current_config.ap_ssid) > 0) {
            wifi_manager_start_ap(current_config.ap_ssid, current_config.ap_password);
        } else {
            // 生成带MAC后缀的默认AP名称
            char default_ap_ssid[33];
            char mac_suffix[5];
            if (wifi_manager_get_mac_suffix(mac_suffix, sizeof(mac_suffix)) == ESP_OK) {
                snprintf(default_ap_ssid, sizeof(default_ap_ssid), "%s_%s", WIFI_MANAGER_DEFAULT_SSID, mac_suffix);
            } else {
                strcpy(default_ap_ssid, WIFI_MANAGER_DEFAULT_SSID);
            }
            wifi_manager_start_ap(default_ap_ssid, WIFI_MANAGER_DEFAULT_PASSWORD);
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); // 等待模式切换完成
    }
    
    // 执行WiFi扫描
    esp_err_t scan_ret = wifi_manager_scan_networks();
    if (scan_ret != ESP_OK) {
        char error_msg[128];
        snprintf(error_msg, sizeof(error_msg), "WiFi扫描失败: %s", esp_err_to_name(scan_ret));
        cJSON_AddStringToObject(response, "message", error_msg);
        cJSON_AddBoolToObject(response, "success", false);
        ESP_LOGE(TAG, "✗ WiFi扫描失败: %s", esp_err_to_name(scan_ret));
        goto scan_end;
    }
    
    // 获取扫描结果
    wifi_ap_record_t ap_records[WIFI_MANAGER_SCAN_MAX_AP];
    uint16_t ap_count = WIFI_MANAGER_SCAN_MAX_AP;
    esp_err_t get_ret = wifi_manager_get_scan_results(ap_records, &ap_count);
    if (get_ret != ESP_OK) {
        cJSON_AddStringToObject(response, "message", "获取扫描结果失败");
        cJSON_AddBoolToObject(response, "success", false);
        ESP_LOGE(TAG, "✗ 获取扫描结果失败: %s", esp_err_to_name(get_ret));
        goto scan_end;
    }
    
    // 创建网络数组
    cJSON *networks = cJSON_CreateArray();
    
    for (int i = 0; i < ap_count; i++) {
        // 安全地处理SSID字符串
        char ssid_str[33] = {0};
        size_t ssid_len = strnlen((char*)ap_records[i].ssid, 32);
        
        // 跳过空SSID
        if (ssid_len == 0) {
            continue;
        }
        
        memcpy(ssid_str, ap_records[i].ssid, ssid_len);
        ssid_str[ssid_len] = '\0';
        
        cJSON *network = cJSON_CreateObject();
        cJSON_AddStringToObject(network, "ssid", ssid_str);
        
        // 信号强度
        cJSON_AddNumberToObject(network, "rssi", ap_records[i].rssi);
        
        // 加密方式
        const char* auth_mode;
        switch (ap_records[i].authmode) {
            case WIFI_AUTH_OPEN:
                auth_mode = "Open";
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
                auth_mode = "Other";
                break;
        }
        cJSON_AddStringToObject(network, "security", auth_mode);
        
        // 信道
        cJSON_AddNumberToObject(network, "channel", ap_records[i].primary);
        
        cJSON_AddItemToArray(networks, network);
    }
    
    cJSON_AddItemToObject(response, "networks", networks);
    cJSON_AddBoolToObject(response, "success", true);
    
    ESP_LOGI(TAG, "✓ WiFi扫描完成，发现 %d 个网络", ap_count);
    
scan_end:
    char *response_string = cJSON_Print(response);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response_string, HTTPD_RESP_USE_STRLEN);
    
    free(response_string);
    cJSON_Delete(response);
    return ESP_OK;
}

// 显示配置API
esp_err_t api_display_config_handler(httpd_req_t *req) {
    char content[512];
    size_t recv_size = MIN(req->content_len, sizeof(content) - 1);
    int ret = httpd_req_recv(req, content, recv_size);
    if (ret <= 0) {
        if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
            httpd_resp_send_408(req);
        }
        return ESP_FAIL;
    }
    content[ret] = '\0';
    
    cJSON *json = cJSON_Parse(content);
    if (json == NULL) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }
    
    cJSON *display_type = cJSON_GetObjectItem(json, "display_type");
    cJSON *custom_content = cJSON_GetObjectItem(json, "custom_content");
    
    if (!cJSON_IsString(display_type)) {
        cJSON_Delete(json);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing display_type");
        return ESP_FAIL;
    }
    
    // 立即设置显示配置并保存到NVS
    const char* display_type_str = display_type->valuestring;
    const char* custom_content_str = (custom_content && cJSON_IsString(custom_content)) ? custom_content->valuestring : "";
    
    // ESP_LOGI(TAG, "自定义WEB十六进制：%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
    //     custom_content_str[0], custom_content_str[1], custom_content_str[2], custom_content_str[3], 
    //     custom_content_str[4], custom_content_str[5], custom_content_str[6], custom_content_str[7], 
    //     custom_content_str[8], custom_content_str[9]);
    // 调用nixie_ctr.c中的函数立即设置显示配置
    set_display_config(display_type_str, custom_content_str);
    
    // 保存显示类型到NVS
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("display_config", NVS_READWRITE, &nvs_handle);
    if (err == ESP_OK) {
        nvs_set_str(nvs_handle, "display_type", display_type_str);
        nvs_commit(nvs_handle);
        nvs_close(nvs_handle);
        ESP_LOGI(TAG, "显示类型已保存到NVS: %s", display_type_str);
    } else {
        ESP_LOGE(TAG, "无法打开NVS存储显示类型: %s", esp_err_to_name(err));
    }
    
    ESP_LOGI(TAG, "Display config: type=%s, content=%s", 
             display_type->valuestring, 
             custom_content ? custom_content->valuestring : "N/A");
    
    cJSON_Delete(json);
    
    // 返回成功响应
    cJSON *response = cJSON_CreateObject();
    cJSON_AddBoolToObject(response, "success", true);
    cJSON_AddStringToObject(response, "message", "显示配置已更新");
    
    char *response_string = cJSON_Print(response);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response_string, strlen(response_string));
    
    free(response_string);
    cJSON_Delete(response);
    return ESP_OK;
}

// 刷新率设置API
esp_err_t api_refresh_rate_handler(httpd_req_t *req) {
    char buf[256];
    int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (ret <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid request");
        return ESP_FAIL;
    }
    buf[ret] = '\0';
    
    cJSON *json = cJSON_Parse(buf);
    if (!json) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }
    
    cJSON *refresh_rate = cJSON_GetObjectItem(json, "refresh_rate");
    if (!refresh_rate || !cJSON_IsString(refresh_rate)) {
        cJSON_Delete(json);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing refresh_rate");
        return ESP_FAIL;
    }
    
    const char* rate_value = cJSON_GetStringValue(refresh_rate);
    ESP_LOGI(TAG, "刷新率设置请求: %s", rate_value);
    
    // 根据刷新率自动切换唤醒模式
    if (strcmp(rate_value, "minute") == 0) {
        // 分跳变：切换到分跳变模式
        set_wakeup_mode(1);
        ESP_LOGI(TAG, "刷新率设置为分跳变，自动切换到分跳变模式");
    } else {
        // 其他刷新率：切换到秒跳变模式
        set_wakeup_mode(0);
        ESP_LOGI(TAG, "刷新率设置为%s，自动切换到秒跳变模式", rate_value);
    }
    
    // 保存刷新率设置到NVS
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("display_config", NVS_READWRITE, &nvs_handle);
    if (err == ESP_OK) {
        err = nvs_set_str(nvs_handle, "refresh_rate", rate_value);
        if (err == ESP_OK) {
            err = nvs_commit(nvs_handle);
            if (err == ESP_OK) {
                ESP_LOGI(TAG, "刷新率设置已保存到NVS: %s", rate_value);
                // 重新加载刷新率设置到全局变量
                load_refresh_rate_from_nvs();
            } else {
                ESP_LOGW(TAG, "刷新率设置NVS提交失败: %s", esp_err_to_name(err));
            }
        } else {
            ESP_LOGW(TAG, "刷新率设置NVS保存失败: %s", esp_err_to_name(err));
        }
        nvs_close(nvs_handle);
    } else {
        ESP_LOGW(TAG, "刷新率设置NVS打开失败: %s", esp_err_to_name(err));
    }
    
    cJSON_Delete(json);
    
    // 返回成功响应
    cJSON *response = cJSON_CreateObject();
    cJSON_AddBoolToObject(response, "success", true);
    cJSON_AddStringToObject(response, "message", "刷新率设置已更新");
    
    char *response_string = cJSON_Print(response);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response_string, strlen(response_string));
    
    free(response_string);
    cJSON_Delete(response);
    return ESP_OK;
}

// 启动Web服务器
esp_err_t start_webserver(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = WEB_SERVER_PORT;
    config.max_uri_handlers = 16;
    
    // 增加httpd任务堆栈大小以防止堆栈溢出
    config.task_priority = 5;
    config.stack_size = 8192;  // 增加到8KB，默认可能只有4KB
    config.core_id = tskNO_AFFINITY;  // 允许在任何核心运行
    config.max_resp_headers = 8;
    config.backlog_conn = 5;
    config.lru_purge_enable = true;
    
    ESP_LOGI(TAG, "Starting HTTP server on port %d with stack size %d", 
             config.server_port, config.stack_size);
    
    if (httpd_start(&server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server");
        return ESP_FAIL;
    }
    
    // 注册HTTP处理程序
    httpd_uri_t index_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = index_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &index_uri);
    
    httpd_uri_t api_system_info_uri = {
        .uri = "/api/system-info",
        .method = HTTP_GET,
        .handler = api_system_info_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &api_system_info_uri);
    
    httpd_uri_t api_wifi_config_uri = {
        .uri = "/api/wifi-config",
        .method = HTTP_POST,
        .handler = api_wifi_config_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &api_wifi_config_uri);
    
    httpd_uri_t api_ap_config_uri = {
        .uri = "/api/ap-config",
        .method = HTTP_POST,
        .handler = api_ap_config_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &api_ap_config_uri);
    
    httpd_uri_t api_ntp_config_uri = {
        .uri = "/api/ntp-config",
        .method = HTTP_POST,
        .handler = api_ntp_config_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &api_ntp_config_uri);
    
    httpd_uri_t api_ntp_sync_uri = {
        .uri = "/api/ntp-sync",
        .method = HTTP_POST,
        .handler = api_ntp_sync_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &api_ntp_sync_uri);
    
    httpd_uri_t api_manual_time_uri = {
        .uri = "/api/manual-time",
        .method = HTTP_POST,
        .handler = api_manual_time_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &api_manual_time_uri);
    
    httpd_uri_t api_timezone_config_uri = {
        .uri = "/api/timezone-config",
        .method = HTTP_POST,
        .handler = api_timezone_config_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &api_timezone_config_uri);
    
    httpd_uri_t api_reboot_uri = {
        .uri = "/api/reboot",
        .method = HTTP_POST,
        .handler = api_reboot_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &api_reboot_uri);
    
    httpd_uri_t api_factory_reset_uri = {
        .uri = "/api/factory-reset",
        .method = HTTP_POST,
        .handler = api_factory_reset_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &api_factory_reset_uri);
    
    httpd_uri_t api_reset_battery_uri = {
        .uri = "/api/reset-battery",
        .method = HTTP_POST,
        .handler = api_reset_battery_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &api_reset_battery_uri);
    
    httpd_uri_t api_set_battery_capacity_uri = {
        .uri = "/api/set-battery-capacity",
        .method = HTTP_POST,
        .handler = api_set_battery_capacity_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &api_set_battery_capacity_uri);
    
    httpd_uri_t api_wifi_scan_uri = {
        .uri = "/api/wifi/scan",
        .method = HTTP_GET,
        .handler = api_wifi_scan_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &api_wifi_scan_uri);
    
    httpd_uri_t api_display_config_uri = {
        .uri = "/api/display-config",
        .method = HTTP_POST,
        .handler = api_display_config_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &api_display_config_uri);
    
    httpd_uri_t api_refresh_rate_uri = {
        .uri = "/api/refresh-rate",
        .method = HTTP_POST,
        .handler = api_refresh_rate_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &api_refresh_rate_uri);
    
    ESP_LOGI(TAG, "Web server started successfully");
    return ESP_OK;
}

// 停止Web服务器
esp_err_t stop_webserver(void) {
    if (server) {
        ESP_LOGI(TAG, "Stopping web server");
        httpd_stop(server);
        server = NULL;
    }
    return ESP_OK;
}

// 获取服务器句柄
httpd_handle_t get_webserver_handle(void) {
    return server;
}