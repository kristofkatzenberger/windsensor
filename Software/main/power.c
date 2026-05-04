/*
 * Windsensor Firmware 
 * Author: Kristof Katzenberger
 * File: power.c
 */

#include <stdio.h>
#include "power.h"
#include "config.h"
#include "display.h" 
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Power driver globals
static const char *TAG = "POWER";
static i2c_master_dev_handle_t ap33772s_handle = NULL;
static i2c_master_dev_handle_t bq25628_handle = NULL;
static bool pd_negotiated = false;
static char pd_status_str[32] = "No PD";

// Forward declarations
void set_bq25628_input_current_limit(uint16_t current_ma);
void set_bq25628_charge_current_limit(uint16_t current_ma);

static bool power_debug_logs_enabled(void) {
    return display_get_terminal_output_mode() == TERMINAL_OUTPUT_DEBUG;
}

static esp_err_t ensure_ap33772s_handle(void) {
    if (ap33772s_handle != NULL) return ESP_OK;
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = AP33772S_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    return i2c_master_bus_add_device(main_bus_handle, &dev_cfg, &ap33772s_handle);
}

static esp_err_t ensure_bq25628_handle(void) {
    if (bq25628_handle != NULL) return ESP_OK;
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BQ25628_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    return i2c_master_bus_add_device(main_bus_handle, &dev_cfg, &bq25628_handle);
}

// Handle AP33772S PD negotiation
void manage_usb_pd(void) {
    esp_err_t ret;
    uint8_t status = 0;
    uint8_t pdos[26];
    uint8_t write_buf[4];

    if (ensure_ap33772s_handle() != ESP_OK) return;

    // Read PD status
    uint8_t reg_addr = AP33772S_REG_STATUS;
    ret = i2c_master_transmit_receive(ap33772s_handle, &reg_addr, 1, &status, 1, 100);

    if (ret != ESP_OK) {
        pd_negotiated = false;
        snprintf(pd_status_str, sizeof(pd_status_str), "Comm Err");
        return;
    }

    // Wait until controller ready
    if (!(status & 0x02)) {
        return;
    }

    // Refresh PDO descriptors when NEWPDO or no session
    if ((status & 0x04) || !pd_negotiated) {
        // Read PDO descriptors
        reg_addr = AP33772S_REG_SRCPDO;
        ret = i2c_master_transmit_receive(ap33772s_handle, &reg_addr, 1, pdos, 26, 100);
        
        if (ret == ESP_OK) {
            // Log PDO bytes
            if (power_debug_logs_enabled()) {
                ESP_LOGI(TAG, "PDOs: %02X %02X %02X %02X ...", pdos[0], pdos[1], pdos[2], pdos[3]);
            }
            
            // Ensure PDO1 present
            uint16_t pdo1 = (pdos[1] << 8) | pdos[0];
            if (pdo1 == 0) {
                snprintf(pd_status_str, sizeof(pd_status_str), "No PD (0)");
                pd_negotiated = false;
                return;
            }

            // Request fixed 5V at 3A
            if (!pd_negotiated) {
                if (power_debug_logs_enabled()) {
                    ESP_LOGI(TAG, "Requesting PDO 1 (5V), 3A...");
                }
                
                // PD_REQMSG (0x31)
                // Bits 12-15: PDO_INDEX = 1
                // Bits 8-11: CURRENT_SEL = 3 (3A)
                // Bits 0-7: VOLTAGE_SEL = 0 (Fixed)
                
                uint16_t req = (1 << 12) | (3 << 8) | 0;
                
                write_buf[0] = AP33772S_REG_PD_REQMSG;
                write_buf[1] = req & 0xFF;
                write_buf[2] = (req >> 8) & 0xFF;
                
                ret = i2c_master_transmit(ap33772s_handle, write_buf, 3, 100);
                
                if (ret == ESP_OK) {
                    pd_negotiated = true;
                    snprintf(pd_status_str, sizeof(pd_status_str), "Req Sent");
                    
                    // Update BQ25628 limits
                    set_bq25628_input_current_limit(3000);
                    set_bq25628_charge_current_limit(1000);
                }
            }
        }
    }
    
    // Read negotiated voltage and current
    uint8_t vreq_buf[2], ireq_buf[2];
    reg_addr = AP33772S_REG_VREQ;
    i2c_master_transmit_receive(ap33772s_handle, &reg_addr, 1, vreq_buf, 2, 100);
    reg_addr = AP33772S_REG_IREQ;
    i2c_master_transmit_receive(ap33772s_handle, &reg_addr, 1, ireq_buf, 2, 100);
    
    uint16_t vreq = (vreq_buf[1] << 8) | vreq_buf[0]; // 50mV LSB
    uint16_t ireq = (ireq_buf[1] << 8) | ireq_buf[0]; // 10mA LSB
    
    if (vreq > 0) {
        snprintf(pd_status_str, sizeof(pd_status_str), "%.1fV @ %.2fA", vreq * 0.05, ireq * 0.01);
    }
}

// Configure the BQ25628 charger
void configure_bq25628_init(void) {
    // Configure STAT pin
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BQ25628_STAT_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    if (ensure_bq25628_handle() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add BQ25628 device");
        return;
    }

    // Configure charger registers
    uint8_t val = 0;
    uint8_t write_buf[2];

    // Disable watchdog
    write_buf[0] = BQ25628_REG_CHG_CTRL_0;
    i2c_master_transmit_receive(bq25628_handle, write_buf, 1, &val, 1, 100);

    val &= ~0x03;

    write_buf[0] = BQ25628_REG_CHG_CTRL_0;
    write_buf[1] = val;
    i2c_master_transmit(bq25628_handle, write_buf, 2, 100);

    // Enable ADC
    write_buf[0] = BQ25628_REG_ADC_CTRL;
    write_buf[1] = 0x80;
    i2c_master_transmit(bq25628_handle, write_buf, 2, 100);

    // Apply default current limits
    set_bq25628_input_current_limit(500);
    set_bq25628_charge_current_limit(500);
}

// Set charger input current limit
void set_bq25628_input_current_limit(uint16_t current_ma) {
    if (ensure_bq25628_handle() != ESP_OK) return;

    if (current_ma < 100) current_ma = 100;
    if (current_ma > 3200) current_ma = 3200;

    // IINDPM is 16-bit register (0x06)
    // Bits 4-11: IINDPM (Step 20mA)
    // Value = current_ma / 20
    // Shifted left by 4 bits
    
    uint16_t reg_val = (current_ma / 20) << 4;
    uint8_t write_buf[3];
    write_buf[0] = BQ25628_REG_IINDPM;
    write_buf[1] = reg_val & 0xFF;        // LSB
    write_buf[2] = (reg_val >> 8) & 0xFF; // MSB
    
    i2c_master_transmit(bq25628_handle, write_buf, 3, 100);
    if (power_debug_logs_enabled()) {
        ESP_LOGI(TAG, "Set Input Current Limit: %d mA (Reg: 0x%04X)", current_ma, reg_val);
    }
}

// Set charger charge current limit
void set_bq25628_charge_current_limit(uint16_t current_ma) {
    if (ensure_bq25628_handle() != ESP_OK) return;

    if (current_ma < 40) current_ma = 40;
    if (current_ma > 2000) current_ma = 2000;

    // ICHG is 16-bit register (0x02)
    // Bits 5-10: ICHG (Step 40mA)
    // Value = current_ma / 40
    // Shifted left by 5 bits
    
    uint16_t reg_val = (current_ma / 40) << 5;
    uint8_t write_buf[3];
    write_buf[0] = BQ25628_REG_ICHG;
    write_buf[1] = reg_val & 0xFF;        // LSB
    write_buf[2] = (reg_val >> 8) & 0xFF; // MSB

    i2c_master_transmit(bq25628_handle, write_buf, 3, 100);
    if (power_debug_logs_enabled()) {
        ESP_LOGI(TAG, "Set Charge Current Limit: %d mA (Reg: 0x%04X)", current_ma, reg_val);
    }
}

// Check charger telemetry and battery status
void check_battery_status(void) {
    uint8_t data[2];
    uint8_t status_1 = 0;
    uint8_t fault_0 = 0; 
    uint8_t reg_addr;
    esp_err_t ret;

    if (ensure_bq25628_handle() != ESP_OK) return;

    // Read charger status register
    reg_addr = BQ25628_REG_STATUS_1;
    ret = i2c_master_transmit_receive(bq25628_handle, &reg_addr, 1, &status_1, 1, 100);

    if (ret != ESP_OK) {
        update_ui_text(0, 0, "Comm Error");
        return;
    }

    uint8_t chg_stat = (status_1 >> 3) & 0x03;
    const char* chg_str[] = {"Not Charging", "Charging", "Taper/Full", "Top-off"};

    // Read fault register
    reg_addr = BQ25628_REG_FAULT_0;
    i2c_master_transmit_receive(bq25628_handle, &reg_addr, 1, &fault_0, 1, 100);

    if (fault_0 != 0 && power_debug_logs_enabled()) {
        ESP_LOGE(TAG, "--- CHARGER FAULT DETECTED: 0x%02X ---", fault_0);
    }

    // Read STAT pin
    int stat_pin_level = gpio_get_level(BQ25628_STAT_PIN);
    const char* pin_str = (stat_pin_level == 0) ? "LOW (Active)" : "HIGH (Idle/Done)";

    // Read VBAT
    reg_addr = BQ25628_REG_VBAT_ADC;
    ret = i2c_master_transmit_receive(bq25628_handle, &reg_addr, 1, data, 2, 100);
    if (ret != ESP_OK) return;

    uint16_t vbat_raw = (data[1] << 8) | data[0];
    uint16_t vbat_code = (vbat_raw >> 1) & 0x0FFF; 
    float vbat_mv = vbat_code * 1.99;

    // Read IBAT
    reg_addr = BQ25628_REG_IBAT_ADC;
    ret = i2c_master_transmit_receive(bq25628_handle, &reg_addr, 1, data, 2, 100);
    if (ret != ESP_OK) return;

    int16_t ibat_raw = (int16_t)((data[1] << 8) | data[0]);
    // Interpret IBAT as signed current
    int ibat_ma = ibat_raw; 

    // Read current limit registers
    uint8_t iindpm_buf[2] = {0};
    uint8_t ichg_buf[2] = {0};
    
    reg_addr = BQ25628_REG_IINDPM;
    i2c_master_transmit_receive(bq25628_handle, &reg_addr, 1, iindpm_buf, 2, 100);
    reg_addr = BQ25628_REG_ICHG;
    i2c_master_transmit_receive(bq25628_handle, &reg_addr, 1, ichg_buf, 2, 100);

    uint16_t iindpm_reg = (iindpm_buf[1] << 8) | iindpm_buf[0];
    uint16_t ichg_reg = (ichg_buf[1] << 8) | ichg_buf[0];

    int iindpm_val = ((iindpm_reg >> 4) & 0xFF) * 20;
    int ichg_val = ((ichg_reg >> 5) & 0x3F) * 40;

    // Log telemetry
    if (power_debug_logs_enabled()) {
        ESP_LOGI(TAG, "Battery: %.0f mV | Current: %d mA | Reg: %s | Pin: %s | PD: %s | Lim: In=%d Chg=%d",
                 vbat_mv, ibat_ma, chg_str[chg_stat], pin_str, pd_status_str, iindpm_val, ichg_val);
    }

    // Update display text
    char status_buf[64];
    if (fault_0 != 0) {
        snprintf(status_buf, sizeof(status_buf), "FAULT: 0x%02X", fault_0);
    } else {
        snprintf(status_buf, sizeof(status_buf), "%s\n%s\nL: %d/%d", chg_str[chg_stat], pd_status_str, iindpm_val, ichg_val);
    }

    update_ui_text(vbat_mv, ibat_ma, status_buf);
}