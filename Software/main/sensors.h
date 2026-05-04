/*
 * Windsensor Firmware 
 * Author: Kristof Katzenberger
 * File: sensors.h
 */

#ifndef SENSORS_H
#define SENSORS_H

#include "esp_err.h"
#include "driver/i2c_master.h"
#include "bmi270.h"

// DPS368 Registers & Constants
#define DPS368_ADDR_0           0x76
#define DPS368_ADDR_1           0x77

typedef struct {
    int16_t c0;
    int16_t c1;
    int32_t c00;
    int32_t c10;
    int16_t c01;
    int16_t c11;
    int16_t c20;
    int16_t c21;
    int16_t c30;
} dps368_coeffs_t;

typedef struct {
    uint8_t channel;
    uint8_t address;
    dps368_coeffs_t coeffs;
    float temperature;
    float pressure;
    float offset;
} dps368_t;

typedef struct {
    struct bmi2_dev dev;
    i2c_master_dev_handle_t i2c_dev_handle;
    struct bmi2_sens_data sensor_data;
} bmi270_sensor_t;

typedef struct {
    i2c_master_dev_handle_t i2c_dev_handle;
    float mag_x;
    float mag_y;
    float mag_z;
    float temperature;
} bmm350_sensor_t;

esp_err_t tca9548_select_channel(uint8_t channel);
esp_err_t tca9548_disable_channels(void);
void run_i2c_scan(const char* bus_name);

esp_err_t dps368_init_driver(i2c_master_bus_handle_t bus_handle);
esp_err_t dps368_init_sensor(dps368_t *sensor);
esp_err_t dps368_measure(dps368_t *sensor);
void dps368_calibrate_offsets(dps368_t *sensors, int count);

esp_err_t bmi270_init_driver(i2c_master_bus_handle_t bus_handle, bmi270_sensor_t *bmi270);
esp_err_t bmi270_read_data(bmi270_sensor_t *bmi270);

esp_err_t bmm350_init_driver(i2c_master_bus_handle_t bus_handle, bmm350_sensor_t *sensor);
esp_err_t bmm350_reinit(bmm350_sensor_t *sensor);
esp_err_t bmm350_read_data(bmm350_sensor_t *sensor);

#endif