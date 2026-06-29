/**
 * @file data_store.c
 * @brief NVS 持久化存储实现
 *
 * 使用 ESP-IDF NVS API，命名空间: "app_store"。
 */
#include "data_store.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"

static const char *TAG = "data_store";
#define NVS_NAMESPACE "app_store"

esp_err_t data_store_init(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS 分区异常，擦除后重新初始化");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NVS 初始化失败: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "NVS 初始化完成");
    }
    return ret;
}

esp_err_t data_store_get_str(const char *key, char *value, size_t max_len)
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (ret != ESP_OK) return ret;
    ret = nvs_get_str(handle, key, value, &max_len);
    nvs_close(handle);
    return ret;
}

esp_err_t data_store_set_str(const char *key, const char *value)
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) return ret;
    ret = nvs_set_str(handle, key, value);
    if (ret == ESP_OK) ret = nvs_commit(handle);
    nvs_close(handle);
    return ret;
}

esp_err_t data_store_get_i32(const char *key, int32_t *value)
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (ret != ESP_OK) return ret;
    ret = nvs_get_i32(handle, key, value);
    nvs_close(handle);
    return ret;
}

esp_err_t data_store_set_i32(const char *key, int32_t value)
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) return ret;
    ret = nvs_set_i32(handle, key, value);
    if (ret == ESP_OK) ret = nvs_commit(handle);
    nvs_close(handle);
    return ret;
}

esp_err_t data_store_get_blob(const char *key, void *data, size_t *length)
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (ret != ESP_OK) return ret;
    ret = nvs_get_blob(handle, key, data, length);
    nvs_close(handle);
    return ret;
}

esp_err_t data_store_set_blob(const char *key, const void *data, size_t length)
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) return ret;
    ret = nvs_set_blob(handle, key, data, length);
    if (ret == ESP_OK) ret = nvs_commit(handle);
    nvs_close(handle);
    return ret;
}
