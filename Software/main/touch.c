/*
 * Windsensor Firmware 
 * Author: Kristof Katzenberger
 * File: touch.c
 */

#include "touch.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Global variables
static const char *TAG = "TOUCH";
static i2c_master_dev_handle_t touch_handle = NULL;

// Initialize touch controller
esp_err_t touch_init(i2c_master_bus_handle_t bus_handle) {
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = TOUCH_I2C_ADDR,
        .scl_speed_hz = 400000,
    };

    esp_err_t ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, &touch_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add touch device");
        return ret;
    }

    // Configure IRQ pin
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << TOUCH_IRQ_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE
    };
    gpio_config(&io_conf);

    ESP_LOGI(TAG, "Touch initialized");
    return ESP_OK;
}

// Read touch sample
bool touch_read(touch_data_t *data) {
    if (touch_handle == NULL) return false;

    uint8_t read_buf[6];
    uint8_t reg_addr = 0x01; 

    esp_err_t ret = i2c_master_transmit_receive(touch_handle, &reg_addr, 1, read_buf, 6, 100);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read touch data");
        return false;
    }

    data->gesture_id = read_buf[0];
    data->points = read_buf[1];
    data->x = ((read_buf[2] & 0x0F) << 8) | read_buf[3];
    data->y = ((read_buf[4] & 0x0F) << 8) | read_buf[5];

    return true;
}
