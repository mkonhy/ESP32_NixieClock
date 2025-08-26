#include "nixie_ctr.h"
#include "web_server.h"
#include "wifi_manager.h"
#include "ntp_sync.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_partition.h"
#include <inttypes.h>

static const char *TAG = "MAIN";

// 分区表兼容性检查函数声明
static void check_partition_table_compatibility(void);

// 分区表兼容性检查函数实现
static void check_partition_table_compatibility(void)
{
    ESP_LOGI(TAG, "检查分区表兼容性...");
    
    // 获取分区表信息
    const esp_partition_t* nvs_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_NVS, NULL);
    if (nvs_partition == NULL) {
        ESP_LOGE(TAG, "未找到NVS分区，分区表可能有问题");
        return;
    }
    
    ESP_LOGI(TAG, "NVS分区信息:");
    ESP_LOGI(TAG, "  地址: 0x%08" PRIx32, (uint32_t)nvs_partition->address);
    ESP_LOGI(TAG, "  大小: %" PRIu32 " bytes", (uint32_t)nvs_partition->size);
    ESP_LOGI(TAG, "  标签: %s", nvs_partition->label);
    
    // 检查NVS分区大小是否合理
    if (nvs_partition->size < 0x4000) { // 小于16KB
        ESP_LOGW(TAG, "NVS分区大小较小 (%" PRIu32 " bytes)，可能影响配置存储", (uint32_t)nvs_partition->size);
    } else if (nvs_partition->size > 0x10000) { // 大于64KB
        ESP_LOGW(TAG, "NVS分区大小较大 (%" PRIu32 " bytes)，可能浪费Flash空间", (uint32_t)nvs_partition->size);
    } else {
        ESP_LOGI(TAG, "NVS分区大小正常 (%" PRIu32 " bytes)", (uint32_t)nvs_partition->size);
    }
    
    // 检查是否有其他重要分区
    const esp_partition_t* app_partition = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_FACTORY, NULL);
    if (app_partition != NULL) {
        ESP_LOGI(TAG, "应用程序分区:");
        ESP_LOGI(TAG, "  地址: 0x%08" PRIx32, (uint32_t)app_partition->address);
        ESP_LOGI(TAG, "  大小: %" PRIu32 " bytes", (uint32_t)app_partition->size);
    }
    
    // 检查存储分区
    const esp_partition_t* storage_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_FAT, NULL);
    if (storage_partition != NULL) {
        ESP_LOGI(TAG, "存储分区:");
        ESP_LOGI(TAG, "  地址: 0x%08" PRIx32, (uint32_t)storage_partition->address);
        ESP_LOGI(TAG, "  大小: %" PRIu32 " bytes", (uint32_t)storage_partition->size);
    }
    
    ESP_LOGI(TAG, "分区表兼容性检查完成");
}

void app_main(void)
{    
    // 初始化NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // 检查分区表兼容性
    ESP_LOGI(TAG, "检查分区表兼容性...");
    check_partition_table_compatibility();
    
    // 初始化辉光管控制
    ESP_LOGI(TAG, "初始化辉光管控制器...");
    nixie_ctr_init();
    
    // 初始化WiFi管理器
    ESP_LOGI(TAG, "初始化WiFi管理器...");
    ret = wifi_manager_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "WiFi管理器初始化失败: %s", esp_err_to_name(ret));
        return;
    }
    
    // 初始化NTP同步系统
    ESP_LOGI(TAG, "初始化NTP同步系统...");
    ret = ntp_sync_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NTP同步系统初始化失败: %s", esp_err_to_name(ret));
        // NTP初始化失败不影响主要功能，继续运行
    }
    
    ESP_LOGI(TAG, "==>> 系统初始化完成 <<==");
    
    // 应用保存的WiFi配置
    ESP_LOGI(TAG, "应用保存的WiFi配置...");
    ret = wifi_manager_apply_config();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "应用WiFi配置失败，降级到默认AP模式");
        wifi_manager_set_mode(WIFI_MODE_AP_ONLY);
        vTaskDelay(pdMS_TO_TICKS(1000));
        wifi_manager_start_ap(NULL, NULL);
    }
    
    // 检查WiFi是否被启用
    wifi_manager_mode_t current_wifi_mode = wifi_manager_get_current_mode();
    if (current_wifi_mode == WIFI_MODE_OFF) {
        ESP_LOGI(TAG, "WiFi功能已被禁用，跳过Web服务器启动");
        ESP_LOGI(TAG, "系统启动完成，进入离线运行状态");
        return;
    }
    
    // 启动web服务器
    ESP_LOGI(TAG, "启动Web服务器...");
    ret = start_webserver();
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Web服务器启动成功！");
        ESP_LOGI(TAG, "可通过 http://192.168.4.1 访问web界面");
    } else {
        ESP_LOGE(TAG, "Web服务器启动失败: %s", esp_err_to_name(ret));
    }
    
    ESP_LOGI(TAG, "系统启动完成，进入运行状态");
}