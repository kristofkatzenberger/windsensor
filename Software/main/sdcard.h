/*
 * Windsensor Firmware 
 * Author: Kristof Katzenberger
 * File: sdcard.h
 */

#ifndef SDCARD_H
#define SDCARD_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

esp_err_t sdcard_init(void);
esp_err_t sdcard_start_recording(char *filename_buf, size_t filename_buf_size);
esp_err_t sdcard_stop_recording(void);
bool sdcard_is_recording(void);
esp_err_t sdcard_log_data(uint64_t uptime_ms,
						  float wind_speed_ms,
						  int relative_angle_deg,
						  float compass_angle_deg,
						  const float *pressures,
						  size_t pressure_count,
						  int32_t accel_x_raw,
						  int32_t accel_y_raw,
						  int32_t accel_z_raw,
						  int32_t gyro_x_raw,
						  int32_t gyro_y_raw,
						  int32_t gyro_z_raw);
esp_err_t sdcard_unmount(void);

#endif // SDCARD_H
