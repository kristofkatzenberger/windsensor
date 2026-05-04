/*
 * Windsensor Firmware 
 * Author: Kristof Katzenberger
 * File: sensors.c
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "sensors.h"
#include "config.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_rom_sys.h"

// Global logging tag and handles
static const char *TAG = "SENSORS";
static i2c_master_dev_handle_t tca9548_handle = NULL;
static i2c_master_dev_handle_t dps368_handle_76 = NULL;
static i2c_master_dev_handle_t dps368_handle_77 = NULL;

// DPS368 registers
#define DPS310_REG_PSR_B2       0x00
#define DPS310_REG_PSR_B1       0x01
#define DPS310_REG_PSR_B0       0x02
#define DPS310_REG_TMP_B2       0x03
#define DPS310_REG_TMP_B1       0x04
#define DPS310_REG_TMP_B0       0x05
#define DPS310_REG_PRS_CFG      0x06
#define DPS310_REG_TMP_CFG      0x07
#define DPS310_REG_MEAS_CFG     0x08
#define DPS310_REG_CFG_REG      0x09
#define DPS310_REG_INT_STS      0x0A
#define DPS310_REG_FIFO_STS     0x0B
#define DPS310_REG_RESET        0x0C
#define DPS310_REG_PRODUCT_ID   0x0D
#define DPS310_REG_COEF_START   0x10
#define DPS310_REG_COEF_END     0x21
#define DPS310_REG_COEF_SRCE    0x28

#define DPS310_RESET_CMD        0x09
#define DPS310_MEAS_CTRL_CONT   0x07

// BMI270 helpers

// BMI270 I2C read helper
static int8_t bmi2_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    i2c_master_dev_handle_t handle = (i2c_master_dev_handle_t)intf_ptr;
    esp_err_t ret = i2c_master_transmit_receive(handle, &reg_addr, 1, reg_data, len, 1000);
    return (ret == ESP_OK) ? BMI2_OK : BMI2_E_COM_FAIL;
}

// BMI270 I2C write helper
static int8_t bmi2_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    i2c_master_dev_handle_t handle = (i2c_master_dev_handle_t)intf_ptr;
    uint8_t *buffer = malloc(len + 1);
    if (!buffer) return BMI2_E_COM_FAIL;
    
    buffer[0] = reg_addr;
    for (uint32_t i = 0; i < len; i++) {
        buffer[i + 1] = reg_data[i];
    }
    
    esp_err_t ret = i2c_master_transmit(handle, buffer, len + 1, 1000);
    free(buffer);
    
    return (ret == ESP_OK) ? BMI2_OK : BMI2_E_COM_FAIL;
}

// BMI270 delay helper
static void bmi2_delay_us(uint32_t period, void *intf_ptr) {
    esp_rom_delay_us(period);
}

// BMI270 driver init
esp_err_t bmi270_init_driver(i2c_master_bus_handle_t bus_handle, bmi270_sensor_t *bmi270) {
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BMI270_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &bmi270->i2c_dev_handle));

    bmi270->dev.intf = BMI2_I2C_INTF;
    bmi270->dev.read = bmi2_i2c_read;
    bmi270->dev.write = bmi2_i2c_write;
    bmi270->dev.delay_us = bmi2_delay_us;
    bmi270->dev.intf_ptr = bmi270->i2c_dev_handle;
    bmi270->dev.read_write_len = 32;
    bmi270->dev.config_file_ptr = NULL;

    int8_t rslt = bmi270_init(&bmi270->dev);
    if (rslt != BMI2_OK) {
        ESP_LOGE(TAG, "BMI270 Init failed: %d", rslt);
        return ESP_FAIL;
    }

    // Enable Sensors (Accel + Gyro)
    uint8_t sens_list[2] = {BMI2_ACCEL, BMI2_GYRO};
    rslt = bmi270_sensor_enable(sens_list, 2, &bmi270->dev);
    if (rslt != BMI2_OK) {
        ESP_LOGE(TAG, "BMI270 Enable failed: %d", rslt);
        return ESP_FAIL;
    }

    // Configure Sensors 
    struct bmi2_sens_config config[2];
    config[0].type = BMI2_ACCEL;
    config[0].cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;
    config[0].cfg.acc.odr = BMI2_ACC_ODR_100HZ;
    config[0].cfg.acc.range = BMI2_ACC_RANGE_2G;
    config[0].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;

    config[1].type = BMI2_GYRO;
    config[1].cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;
    config[1].cfg.gyr.odr = BMI2_GYR_ODR_100HZ;
    config[1].cfg.gyr.range = BMI2_GYR_RANGE_2000;
    config[1].cfg.gyr.noise_perf = BMI2_PERF_OPT_MODE;
    config[1].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;

    rslt = bmi270_set_sensor_config(config, 2, &bmi270->dev);
    if (rslt != BMI2_OK) {
        ESP_LOGE(TAG, "BMI270 Config failed: %d", rslt);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "BMI270 Initialized successfully");
    return ESP_OK;
}

// BMI270 data read
esp_err_t bmi270_read_data(bmi270_sensor_t *bmi270) {
    int8_t rslt = bmi2_get_sensor_data(&bmi270->sensor_data, &bmi270->dev);
    if (rslt != BMI2_OK) {
        ESP_LOGE(TAG, "BMI270 Read failed: %d", rslt);
        return ESP_FAIL;
    }
    return ESP_OK;
}

// TCA9548 handle guard
static esp_err_t ensure_tca9548_handle(void) {
    if (tca9548_handle != NULL) {
        return ESP_OK;
    }
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = TCA9548_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    return i2c_master_bus_add_device(main_bus_handle, &dev_cfg, &tca9548_handle);
}

// DPS368 driver init
esp_err_t dps368_init_driver(i2c_master_bus_handle_t bus_handle) {
    i2c_device_config_t dev_cfg_76 = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = DPS368_ADDR_0,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg_76, &dps368_handle_76));

    i2c_device_config_t dev_cfg_77 = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = DPS368_ADDR_1,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg_77, &dps368_handle_77));
    return ESP_OK;
}

// DPS368 register write helper
static esp_err_t dps368_write_reg(i2c_master_dev_handle_t handle, uint8_t reg, uint8_t val) {
    uint8_t data[2] = {reg, val};
    return i2c_master_transmit(handle, data, 2, 1000);
}

// DPS368 register read helper
static esp_err_t dps368_read_regs(i2c_master_dev_handle_t handle, uint8_t reg, uint8_t *data, size_t len) {
    return i2c_master_transmit_receive(handle, &reg, 1, data, len, 1000);
}

// DPS368 sensor init
esp_err_t dps368_init_sensor(dps368_t *sensor) {
    tca9548_select_channel(sensor->channel);
    i2c_master_dev_handle_t handle = (sensor->address == DPS368_ADDR_0) ? dps368_handle_76 : dps368_handle_77;

    dps368_write_reg(handle, DPS310_REG_RESET, DPS310_RESET_CMD); // Soft reset the DPS368
    vTaskDelay(pdMS_TO_TICKS(50)); 

    uint8_t meas_cfg = 0;
    int retries = 20;
    while (retries--) {
        dps368_read_regs(handle, DPS310_REG_MEAS_CFG, &meas_cfg, 1);
        if ((meas_cfg & 0xC0) == 0xC0) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    if (retries < 0) {
        ESP_LOGE(TAG, "Sensor Ch %d Addr 0x%02X not ready (MEAS_CFG: 0x%02X)", sensor->channel, sensor->address, meas_cfg);
        return ESP_FAIL;
    }

    uint8_t val;
    if (dps368_read_regs(handle, DPS310_REG_PRODUCT_ID, &val, 1) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read ID Ch %d Addr 0x%02X", sensor->channel, sensor->address);
        return ESP_FAIL;
    }
    if (val != 0x10) {
        ESP_LOGE(TAG, "DPS368 ID mismatch on Ch %d Addr 0x%02X: 0x%02X", sensor->channel, sensor->address, val);
        return ESP_FAIL;
    }

    uint8_t raw_coeffs[18];
    if (dps368_read_regs(handle, DPS310_REG_COEF_START, raw_coeffs, 18) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read coeffs Ch %d Addr 0x%02X", sensor->channel, sensor->address);
        return ESP_FAIL;
    }

    sensor->coeffs.c0 = (raw_coeffs[0] << 4) | (raw_coeffs[1] >> 4);
    if (sensor->coeffs.c0 & 0x800) sensor->coeffs.c0 -= 0x1000;

    sensor->coeffs.c1 = ((raw_coeffs[1] & 0x0F) << 8) | raw_coeffs[2];
    if (sensor->coeffs.c1 & 0x800) sensor->coeffs.c1 -= 0x1000;

    sensor->coeffs.c00 = (raw_coeffs[3] << 12) | (raw_coeffs[4] << 4) | (raw_coeffs[5] >> 4);
    if (sensor->coeffs.c00 & 0x80000) sensor->coeffs.c00 -= 0x100000;

    sensor->coeffs.c10 = ((raw_coeffs[5] & 0x0F) << 16) | (raw_coeffs[6] << 8) | raw_coeffs[7];
    if (sensor->coeffs.c10 & 0x80000) sensor->coeffs.c10 -= 0x100000;

    sensor->coeffs.c01 = (raw_coeffs[8] << 8) | raw_coeffs[9];
    sensor->coeffs.c11 = (raw_coeffs[10] << 8) | raw_coeffs[11];
    sensor->coeffs.c20 = (raw_coeffs[12] << 8) | raw_coeffs[13];
    sensor->coeffs.c21 = (raw_coeffs[14] << 8) | raw_coeffs[15];
    sensor->coeffs.c30 = (raw_coeffs[16] << 8) | raw_coeffs[17];

    sensor->offset = 0.0f;

    ESP_LOGI(TAG, "Ch %d Addr 0x%02X Coeffs: c0=%d c1=%d c00=%ld c10=%ld", 
             sensor->channel, sensor->address, sensor->coeffs.c0, sensor->coeffs.c1, sensor->coeffs.c00, sensor->coeffs.c10);

    uint8_t coef_srce = 0;
    if (dps368_read_regs(handle, DPS310_REG_COEF_SRCE, &coef_srce, 1) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read COEF_SRCE");
        return ESP_FAIL;
    }
    bool use_external_temp = (coef_srce & 0x80);
    ESP_LOGI(TAG, "Ch %d Addr 0x%02X COEF_SRCE=0x%02X -> Using %s Temp", 
             sensor->channel, sensor->address, coef_srce, use_external_temp ? "External" : "Internal");

    dps368_write_reg(handle, DPS310_REG_PRS_CFG, 0x36); // Set pressure oversampling/filter config
    uint8_t tmp_cfg = 0x36;
    if (use_external_temp) tmp_cfg |= 0x80;
    dps368_write_reg(handle, DPS310_REG_TMP_CFG, tmp_cfg); // Configure temperature source/oversampling

    dps368_write_reg(handle, DPS310_REG_CFG_REG, 0x0C); // Enable pressure/temperature shift (interrupt cfg)
    dps368_write_reg(handle, DPS310_REG_MEAS_CFG, 0x07); // Start continuous pressure+temperature conversions

    return ESP_OK;
}

// BMM350 registers
#define BMM350_REG_CHIP_ID          0x00
#define BMM350_REG_PMU_CMD_AGGR_SET 0x04
#define BMM350_REG_PMU_CMD_AXIS_EN  0x05
#define BMM350_REG_PMU_CMD          0x06
#define BMM350_REG_MAG_X_XLSB       0x31
#define BMM350_REG_OTP_CMD_REG      0x50

#define BMM350_CHIP_ID              0x33
#define BMM350_CMD_OTP_BOOT_END     0x80
#define BMM350_PMU_CMD_NM           0x01

// BMM350 helpers
// BMM350 register write helper
static esp_err_t bmm350_write_reg(i2c_master_dev_handle_t handle, uint8_t reg, uint8_t val) {
    uint8_t data[2] = {reg, val};
    return i2c_master_transmit(handle, data, 2, 1000);
}

// BMM350 register read helper
static esp_err_t bmm350_read_regs(i2c_master_dev_handle_t handle, uint8_t reg, uint8_t *data, size_t len) {
    uint8_t buf[32];
    if (len + 2 > sizeof(buf)) {
        return ESP_ERR_INVALID_SIZE;
    }

    esp_err_t ret = i2c_master_transmit_receive(handle, &reg, 1, buf, len + 2, 1000);
    if (ret == ESP_OK) {
        memcpy(data, buf + 2, len);
    }
    return ret;
}

// BMM350 reinit
esp_err_t bmm350_reinit(bmm350_sensor_t *sensor) {
    uint8_t chip_id = 0;
    int retries = 5;
    while (retries--) {
        if (bmm350_read_regs(sensor->i2c_dev_handle, BMM350_REG_CHIP_ID, &chip_id, 1) == ESP_OK) {
            if (chip_id == BMM350_CHIP_ID) break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    if (chip_id != BMM350_CHIP_ID) {
        ESP_LOGE(TAG, "BMM350: Invalid Chip ID 0x%02X", chip_id);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "BMM350: Found Chip ID 0x%02X", chip_id);

    bmm350_write_reg(sensor->i2c_dev_handle, BMM350_REG_OTP_CMD_REG, BMM350_CMD_OTP_BOOT_END); // Finish OTP boot sequence
    vTaskDelay(pdMS_TO_TICKS(10));

    bmm350_write_reg(sensor->i2c_dev_handle, BMM350_REG_PMU_CMD_AGGR_SET, 0x22); // Set averaging for magnetic axes

    bmm350_write_reg(sensor->i2c_dev_handle, BMM350_REG_PMU_CMD_AXIS_EN, 0x07); // Enable XYZ magnetic axes

    bmm350_write_reg(sensor->i2c_dev_handle, BMM350_REG_PMU_CMD, BMM350_PMU_CMD_NM); // Switch PMU into normal measurement mode

    return ESP_OK;
}

// BMM350 driver init
esp_err_t bmm350_init_driver(i2c_master_bus_handle_t bus_handle, bmm350_sensor_t *sensor) {
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BMM350_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &sensor->i2c_dev_handle));

    return bmm350_reinit(sensor);
}

// BMM350 data read
esp_err_t bmm350_read_data(bmm350_sensor_t *sensor) {
    uint8_t data[12];
    if (bmm350_read_regs(sensor->i2c_dev_handle, BMM350_REG_MAG_X_XLSB, data, 12) != ESP_OK) {
        return ESP_FAIL;
    }

    // Helper to convert 24-bit (21-bit used) to signed 32-bit
    int32_t raw_x = (data[2] << 16) | (data[1] << 8) | data[0];
    int32_t raw_y = (data[5] << 16) | (data[4] << 8) | data[3];
    int32_t raw_z = (data[8] << 16) | (data[7] << 8) | data[6];
    int32_t raw_t = (data[11] << 16) | (data[10] << 8) | data[9];

    // Mask unused bits (23-21) and sign extend from bit 20
    raw_x &= 0x1FFFFF; if (raw_x & 0x100000) raw_x -= 0x200000;
    raw_y &= 0x1FFFFF; if (raw_y & 0x100000) raw_y -= 0x200000;
    raw_z &= 0x1FFFFF; if (raw_z & 0x100000) raw_z -= 0x200000;
    raw_t &= 0x1FFFFF; if (raw_t & 0x100000) raw_t -= 0x200000;

    // Conversion (Approximate, based on +/- 2000uT range)
    // 1 LSB = 2000 / 1048576 uT ~= 0.0019073 uT
    const float lsb_to_ut = 0.0019073f;
    
    sensor->mag_x = (float)raw_x * lsb_to_ut;
    sensor->mag_y = (float)raw_y * lsb_to_ut;
    sensor->mag_z = (float)raw_z * lsb_to_ut;
    
    sensor->temperature = (float)raw_t; 

    return ESP_OK;
}

// DPS368 measurement routine
esp_err_t dps368_measure(dps368_t *sensor) {
    tca9548_select_channel(sensor->channel);
    i2c_master_dev_handle_t handle = (sensor->address == DPS368_ADDR_0) ? dps368_handle_76 : dps368_handle_77;

    uint8_t data[6];
    if (dps368_read_regs(handle, DPS310_REG_PSR_B2, data, 6) != ESP_OK) {
        return ESP_FAIL;
    }

    int32_t raw_p = (data[0] << 16) | (data[1] << 8) | data[2];
    if (raw_p & 0x800000) raw_p -= 0x1000000;

    int32_t raw_t = (data[3] << 16) | (data[4] << 8) | data[5];
    if (raw_t & 0x800000) raw_t -= 0x1000000;

    float p_sc = (float)raw_p / 1040384.0f;
    float t_sc = (float)raw_t / 1040384.0f;

    sensor->temperature = (float)sensor->coeffs.c0 * 0.5f + (float)sensor->coeffs.c1 * t_sc;

    sensor->pressure = (float)sensor->coeffs.c00 + 
                      p_sc * ((float)sensor->coeffs.c10 + p_sc * ((float)sensor->coeffs.c20 + p_sc * (float)sensor->coeffs.c30)) +
                      t_sc * (float)sensor->coeffs.c01 +
                      t_sc * p_sc * ((float)sensor->coeffs.c11 + p_sc * (float)sensor->coeffs.c21);
    
    sensor->pressure += sensor->offset;

    return ESP_OK;
}

// TCA9548 channel select
esp_err_t tca9548_select_channel(uint8_t channel) {
    if (channel > 7) return ESP_FAIL;
    if (ensure_tca9548_handle() != ESP_OK) return ESP_FAIL;

    uint8_t data = (1 << channel);
    return i2c_master_transmit(tca9548_handle, &data, 1, 1000); // Enable requested TCA9548 channel
}

// TCA9548 channel disable
esp_err_t tca9548_disable_channels(void) {
    if (ensure_tca9548_handle() != ESP_OK) return ESP_FAIL;

    uint8_t data = 0x00;
    return i2c_master_transmit(tca9548_handle, &data, 1, 1000); // Disable all TCA9548 channels
}

// I2C bus scan
void run_i2c_scan(const char* bus_name) {
    printf("\n>> Scanning: %s\n", bus_name);
    uint8_t address;
    for (address = 1; address < 127; address++) {
        esp_err_t ret = i2c_master_probe(main_bus_handle, address, 50);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "-- Found device at 0x%02X", address);
        }
    }
}

// DPS368 offset calibration
void dps368_calibrate_offsets(dps368_t *sensors, int count) {
    if (count <= 1) return;

    if (count <= 1) return;

    float reference_pressure = sensors[1].pressure;
    ESP_LOGI(TAG, "Calibrating offsets. Reference (Sensor 1): %.2f Pa", reference_pressure);

    for (int i = 0; i < count; i++) {
        // Calculate difference needed to match reference
        float diff = reference_pressure - sensors[i].pressure;
        
        // Update offset (accumulate)
        sensors[i].offset += diff;
        
        ESP_LOGI(TAG, "Sensor %d: Val=%.2f Diff=%.2f NewOffset=%.2f", i, sensors[i].pressure, diff, sensors[i].offset);
        
        // Apply immediately so the current value reflects the calibration
        sensors[i].pressure += diff;
    }
}

