/*
 * Windsensor Firmware 
 * Author: Kristof Katzenberger
 * File: touch.h
 */

#ifndef TOUCH_H
#define TOUCH_H

#include "esp_err.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"

#define TOUCH_I2C_ADDR 0x15
#define TOUCH_IRQ_PIN  GPIO_NUM_5

typedef struct {
    uint8_t gesture_id;
    uint8_t points;
    uint16_t x;
    uint16_t y;
} touch_data_t;

esp_err_t touch_init(i2c_master_bus_handle_t bus_handle);
bool touch_read(touch_data_t *data);

#endif
