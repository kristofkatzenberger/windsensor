/*
 * Windsensor Firmware 
 * Author: Kristof Katzenberger
 * File: config.h
 */

#ifndef CONFIG_H
#define CONFIG_H

#include "driver/gpio.h"
#include "driver/i2c_master.h"

// --- I2C Configuration ---
#define I2C_MASTER_SCL_IO           GPIO_NUM_7      
#define I2C_MASTER_SDA_IO           GPIO_NUM_6      
#define I2C_MASTER_NUM              I2C_NUM_0        
#define I2C_MASTER_FREQ_HZ          100000           

extern i2c_master_bus_handle_t main_bus_handle;

// --- SD Card (SDMMC 4-bit) ---
#define SD_PIN_CMD                  GPIO_NUM_35
#define SD_PIN_CLK                  GPIO_NUM_36
#define SD_PIN_D0                   GPIO_NUM_37
#define SD_PIN_D1                   GPIO_NUM_38
#define SD_PIN_D2                   GPIO_NUM_39
#define SD_PIN_D3                   GPIO_NUM_40
#define SD_PIN_CD                   GPIO_NUM_NC 

// --- SPI Display Configuration (GC9A01) ---
#define LCD_HOST     SPI2_HOST
#define PIN_NUM_MISO GPIO_NUM_12  
#define PIN_NUM_MOSI GPIO_NUM_11
#define PIN_NUM_CLK  GPIO_NUM_10
#define PIN_NUM_CS   GPIO_NUM_9
#define PIN_NUM_DC   GPIO_NUM_8
#define PIN_NUM_RST  GPIO_NUM_14
#define PIN_NUM_BCKL GPIO_NUM_2

#define LCD_PIXEL_CLOCK_HZ (40 * 1000 * 1000) 
#define LCD_H_RES          240
#define LCD_V_RES          240
#define LCD_CMD_BITS       8
#define LCD_PARAM_BITS     8

// --- Device Addresses ---
#define AP33772S_ADDR               0x52
#define BQ25628_ADDR                0x6A
#define TCA9548_ADDR                0x70 
#define DPS368_0_ADDR               0x76
#define DPS368_1_ADDR               0x77
#define BMM350_ADDR                 0x14
#define BMI270_ADDR                 0x68
#define TOUCH_ADDR                  0x15

// --- Registers & GPIOs ---
// AP33772S Registers (User Provided)
#define AP33772S_REG_STATUS         0x01
#define AP33772S_REG_MASK           0x02
#define AP33772S_REG_OPMODE         0x03
#define AP33772S_REG_CONFIG         0x04
#define AP33772S_REG_PDCONFIG       0x05
#define AP33772S_REG_SYSTEM         0x06
#define AP33772S_REG_VOLTAGE        0x11
#define AP33772S_REG_CURRENT        0x12
#define AP33772S_REG_TEMP           0x13
#define AP33772S_REG_VREQ           0x14
#define AP33772S_REG_IREQ           0x15
#define AP33772S_REG_SRCPDO         0x20 // 26 bytes
#define AP33772S_REG_PD_REQMSG      0x31 // Write Only
#define AP33772S_REG_PD_CMDMSG      0x32 // Write Only
#define AP33772S_REG_PD_MSGRLT      0x33

#define BQ25628_STAT_PIN            GPIO_NUM_41

// BQ25628 Registers (User Provided)
#define BQ25628_REG_ICHG            0x02 // 16-bit
#define BQ25628_REG_VREG            0x04 // 16-bit
#define BQ25628_REG_IINDPM          0x06 // 16-bit
#define BQ25628_REG_VINDPM          0x08 // 16-bit
#define BQ25628_REG_VSYSMIN         0x0E // 16-bit
#define BQ25628_REG_IPRECHG         0x10 // 16-bit
#define BQ25628_REG_ITERM           0x12 // 16-bit

#define BQ25628_REG_CHG_CTRL        0x14
#define BQ25628_REG_CHG_TMR         0x15
#define BQ25628_REG_CHG_CTRL_0      0x16
#define BQ25628_REG_CHG_CTRL_1      0x17
#define BQ25628_REG_CHG_CTRL_2      0x18
#define BQ25628_REG_CHG_CTRL_3      0x19
#define BQ25628_REG_NTC_CTRL_0      0x1A
#define BQ25628_REG_NTC_CTRL_1      0x1B
#define BQ25628_REG_NTC_CTRL_2      0x1C

#define BQ25628_REG_STATUS_0        0x1D
#define BQ25628_REG_STATUS_1        0x1E
#define BQ25628_REG_FAULT_0         0x1F
#define BQ25628_REG_FLAG_0          0x20
#define BQ25628_REG_FLAG_1          0x21
#define BQ25628_REG_FAULT_FLAG_0    0x22

#define BQ25628_REG_ADC_CTRL        0x26
#define BQ25628_REG_ADC_DIS_0       0x27
#define BQ25628_REG_IBUS_ADC        0x28 // 16-bit
#define BQ25628_REG_IBAT_ADC        0x2A // 16-bit
#define BQ25628_REG_VBUS_ADC        0x2C // 16-bit
#define BQ25628_REG_VPMID_ADC       0x2E // 16-bit
#define BQ25628_REG_VBAT_ADC        0x30 // 16-bit
#define BQ25628_REG_VSYS_ADC        0x32 // 16-bit
#define BQ25628_REG_TS_ADC          0x34 // 16-bit
#define BQ25628_REG_TDIE_ADC        0x36 // 16-bit
#define BQ25628_REG_PART_INFO       0x38 

#endif // CONFIG_H