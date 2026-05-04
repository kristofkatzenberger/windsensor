/*
 * Windsensor Firmware 
 * Author: Kristof Katzenberger
 * File: main.c
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "config.h"
#include "sensors.h"
#include "power.h"
#include "display.h"
#include "touch.h"
#include "wifi.h"
#include "sdcard.h"

// Module log tag.
static const char *TAG = "MAIN_APP";

// Shared I2C bus handles.
i2c_master_bus_handle_t main_bus_handle = NULL;
static SemaphoreHandle_t touch_sem = NULL;

// DPS368 sensor instances.
static dps368_t sensors[9];

// Last detected peak pressure.
static int last_max_idx = -1;
static float last_max_pressure = 0.0f;

// Compass calibration and heading filter state.
static float min_x = 10000.0f, max_x = -10000.0f;
static float min_y = 10000.0f, max_y = -10000.0f;
static float min_z = 10000.0f, max_z = -10000.0f;
static bool compass_cal_active = false;
static bool compass_cal_reset = false;
static float last_heading_deg = 0.0f;
static float heading_filter_state_deg = 0.0f;
static bool heading_filter_initialized = false;
static char recording_filename[32] = {0};

static const float compass_heading_filter_alpha = 0.30f;

// Convert pressure difference to estimated wind speed (m/s).
static float pressure_diff_to_wind_ms(float pressure_diff_pa) {
    if (pressure_diff_pa <= 0.0f) {
        return 0.0f;
    }

    return ((pressure_diff_pa * 0.55f) + 10.0f) / 3.6f;
}

// Wrap heading angle into the 0..360 degree range.
static float normalize_heading_deg(float heading_deg) {
    while (heading_deg < 0.0f) {
        heading_deg += 360.0f;
    }
    while (heading_deg >= 360.0f) {
        heading_deg -= 360.0f;
    }
    return heading_deg;
}

// Return shortest signed angular delta between two headings.
static float shortest_heading_delta_deg(float from_deg, float to_deg) {
    float delta = to_deg - from_deg;
    while (delta > 180.0f) {
        delta -= 360.0f;
    }
    while (delta < -180.0f) {
        delta += 360.0f;
    }
    return delta;
}

// Smooth heading with a circular EMA filter.
static float smooth_heading_deg(float raw_heading_deg) {
    raw_heading_deg = normalize_heading_deg(raw_heading_deg);

    if (!heading_filter_initialized || !isfinite(heading_filter_state_deg)) {
        heading_filter_state_deg = raw_heading_deg;
        heading_filter_initialized = true;
        return raw_heading_deg;
    }

    float delta = shortest_heading_delta_deg(heading_filter_state_deg, raw_heading_deg);
    heading_filter_state_deg = normalize_heading_deg(
        heading_filter_state_deg + (compass_heading_filter_alpha * delta)
    );

    return heading_filter_state_deg;
}

// Convert sensor heading into the UI reference frame.
static float heading_to_relative_frame(float sensor_heading_deg) {
    return normalize_heading_deg(sensor_heading_deg + 90.0f);
}

// Prime DPS sensors with a few startup measurement cycles.
static void warmup_dps_measurements(int cycles, TickType_t cycle_delay) {
    if (cycles <= 0) {
        return;
    }

    for (int c = 0; c < cycles; c++) {
        for (int i = 0; i < 9; i++) {
            (void)dps368_measure(&sensors[i]);
        }

        vTaskDelay(cycle_delay);
    }
}

// Check whether verbose debug logs are enabled.
static bool app_debug_logs_enabled(void) {
    return display_get_terminal_output_mode() == TERMINAL_OUTPUT_DEBUG;
}

// Apply per-module log levels for the selected terminal output mode.
static void apply_terminal_output_log_policy(terminal_output_mode_t mode) {
    esp_log_level_t wifi_log_level = (mode == TERMINAL_OUTPUT_DEBUG) ? ESP_LOG_INFO : ESP_LOG_WARN;
    esp_log_level_t peripheral_log_level = (mode == TERMINAL_OUTPUT_DEBUG) ? ESP_LOG_INFO : ESP_LOG_WARN;

    // Reduce Wi-Fi logs outside debug mode.
    esp_log_level_set("wifi", wifi_log_level);
    esp_log_level_set("esp_netif_lwip", wifi_log_level);
    esp_log_level_set("WIFI", wifi_log_level);

    // Reduce infrastructure logs outside debug mode.
    esp_log_level_set("gpio", peripheral_log_level);
    esp_log_level_set("SDCARD", peripheral_log_level);
    esp_log_level_set("sdmmc_common", peripheral_log_level);
    esp_log_level_set("sdmmc_sd", peripheral_log_level);
    esp_log_level_set("vfs_fat_sdmmc", peripheral_log_level);
}

// Map pressure sensor index to relative wind direction angle.
static int wind_direction_deg_from_sensor_idx(int sensor_idx) {
    switch (sensor_idx) {
        case 6: return 0;
        case 5: return 45;
        case 4: return 90;
        case 3: return 135;
        case 2: return 180;
        case 0: return 225;
        case 8: return 270;
        case 7: return 315;
        default: return 0;
    }
}

// Persist compass calibration.
static void save_compass_calibration(void) {
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return;

    float compass_data[6] = {min_x, max_x, min_y, max_y, min_z, max_z};
    err = nvs_set_blob(my_handle, "compass_cal", compass_data, sizeof(compass_data));
    if (err == ESP_OK) {
        err = nvs_commit(my_handle);
        ESP_LOGI(TAG, "Compass calibration saved");
    }
    nvs_close(my_handle);
}

// Persist pressure calibration.
static void save_pressure_calibration(void) {
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return;

    float offsets[9];
    for(int i=0; i<9; i++) {
        offsets[i] = sensors[i].offset;
    }

    err = nvs_set_blob(my_handle, "press_offsets", offsets, sizeof(offsets));
    if (err == ESP_OK) {
        err = nvs_commit(my_handle);
        ESP_LOGI(TAG, "Pressure offsets saved");
    }
    nvs_close(my_handle);
}

// Persist wind unit preference.
static void save_wind_unit_setting(void) {
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return;

    err = nvs_set_u8(my_handle, "wind_unit", (uint8_t)display_get_wind_unit());
    if (err == ESP_OK) {
        err = nvs_commit(my_handle);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Wind unit saved");
        }
    }

    nvs_close(my_handle);
}

// Persist terminal output mode preference.
static void save_terminal_output_setting(void) {
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return;

    err = nvs_set_u8(my_handle, "term_out_mode", (uint8_t)display_get_terminal_output_mode());
    if (err == ESP_OK) {
        err = nvs_commit(my_handle);
        if (err == ESP_OK && app_debug_logs_enabled()) {
            ESP_LOGI(TAG, "Terminal output mode saved");
        }
    }

    nvs_close(my_handle);
}

// Load persisted calibration and settings.
static void load_calibration(void) {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    nvs_handle_t my_handle;
    err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
        return;
    }

    // Load compass calibration (backward compatible format).
    size_t required_size = 0;
    err = nvs_get_blob(my_handle, "compass_cal", NULL, &required_size);
    if (err == ESP_OK && required_size > 0) {
        float *compass_data = malloc(required_size);
        err = nvs_get_blob(my_handle, "compass_cal", compass_data, &required_size);
        if (err == ESP_OK) {
            if (required_size >= sizeof(float) * 4) {
                min_x = compass_data[0]; max_x = compass_data[1];
                min_y = compass_data[2]; max_y = compass_data[3];
            }
            if (required_size >= sizeof(float) * 6) {
                min_z = compass_data[4]; max_z = compass_data[5];
                ESP_LOGI(TAG, "Compass cal loaded (3D): X[%.1f, %.1f] Y[%.1f, %.1f] Z[%.1f, %.1f]", min_x, max_x, min_y, max_y, min_z, max_z);
            } else {
                ESP_LOGI(TAG, "Compass cal loaded (2D - old format)");
            }
        }
        free(compass_data);
    } else {
        ESP_LOGW(TAG, "No compass calibration found");
    }

    // Load pressure offsets.
    required_size = sizeof(float) * 9;
    float offsets[9];
    err = nvs_get_blob(my_handle, "press_offsets", offsets, &required_size);
    if (err == ESP_OK) {
        for(int i=0; i<9; i++) {
            sensors[i].offset = offsets[i];
        }
        ESP_LOGI(TAG, "Pressure offsets loaded");
    } else {
        ESP_LOGW(TAG, "No pressure offsets found");
    }

    uint8_t saved_wind_unit = (uint8_t)WIND_UNIT_PA;
    err = nvs_get_u8(my_handle, "wind_unit", &saved_wind_unit);
    if (err == ESP_OK) {
        display_set_wind_unit((wind_unit_t)saved_wind_unit);
        ESP_LOGI(TAG, "Wind unit loaded: %d", saved_wind_unit);
    } else {
        display_set_wind_unit(WIND_UNIT_PA);
        ESP_LOGW(TAG, "No wind unit setting found");
    }

    uint8_t saved_terminal_output_mode = (uint8_t)TERMINAL_OUTPUT_DEBUG;
    err = nvs_get_u8(my_handle, "term_out_mode", &saved_terminal_output_mode);
    if (err == ESP_OK) {
        display_set_terminal_output_mode((terminal_output_mode_t)saved_terminal_output_mode);
        if (app_debug_logs_enabled()) {
            ESP_LOGI(TAG, "Terminal output mode loaded: %d", saved_terminal_output_mode);
        }
    } else {
        display_set_terminal_output_mode(TERMINAL_OUTPUT_DEBUG);
        ESP_LOGW(TAG, "No terminal output mode setting found");
    }

    nvs_close(my_handle);
}

// Return latest wind snapshot.
void get_wind_data(int *idx, float *diff) {
    *idx = last_max_idx;
    *diff = last_max_pressure;
}

// Return latest compass heading in UI reference frame.
void get_compass_heading_deg(float *heading_deg) {
    if (heading_deg == NULL) {
        return;
    }

    *heading_deg = heading_to_relative_frame(last_heading_deg);
}

static void enter_system_sleep(void);

// Execute action bound to a long press for the current display mode.
static void handle_long_press_action(bool *calibration_done) {
    if (!display_get_power()) {
        display_set_power(true);
        return;
    }

    display_mode_t mode = display_get_mode();

    if (mode == DISPLAY_MODE_SETTINGS) {
        display_enter_settings_menu();
        *calibration_done = false;
    } else if (mode == DISPLAY_MODE_SETTINGS_BACK) {
        display_exit_settings_menu();
    } else if (mode == DISPLAY_MODE_WIND_UNIT) {
        display_cycle_wind_unit();
        save_wind_unit_setting();
    } else if (mode == DISPLAY_MODE_TERMINAL_OUTPUT) {
        display_cycle_terminal_output_mode();
        save_terminal_output_setting();
    } else if (mode == DISPLAY_MODE_CALIBRATION) {
        if (!(*calibration_done)) {
            dps368_calibrate_offsets(sensors, 9);
            save_pressure_calibration();
            display_show_calibration_tick();
            *calibration_done = true;
        }
    } else if (mode == DISPLAY_MODE_COMPASS_CAL) {
        compass_cal_active = !compass_cal_active;
        if (compass_cal_active) {
            compass_cal_reset = true;
            display_show_compass_cal_text("Rotate Compass\nSlowly...");
        } else {
            save_compass_calibration();
            display_show_compass_cal_text("Calibrated!");
        }
    } else if (mode == DISPLAY_MODE_RECORDING) {
        if (sdcard_is_recording()) {
            esp_err_t stop_ret = sdcard_stop_recording();
            if (stop_ret == ESP_OK) {
                recording_filename[0] = '\0';
                display_set_recording_state(false, NULL);
            } else {
                ESP_LOGW(TAG, "Failed to stop recording: %s", esp_err_to_name(stop_ret));
            }
        } else {
            char next_filename[32] = {0};
            esp_err_t start_ret = sdcard_start_recording(next_filename, sizeof(next_filename));

            if (start_ret == ESP_OK) {
                strncpy(recording_filename, next_filename, sizeof(recording_filename) - 1);
                recording_filename[sizeof(recording_filename) - 1] = '\0';
                display_set_recording_state(true, recording_filename);
            } else {
                recording_filename[0] = '\0';
                display_set_recording_state(false, NULL);
                ESP_LOGW(TAG, "Failed to start recording: %s", esp_err_to_name(start_ret));
            }
        }
    } else if (mode == DISPLAY_MODE_POWER) {
        display_set_power(false);
    } else if (mode == DISPLAY_MODE_SLEEP) {
        enter_system_sleep();
    }
}

// Enter the light-sleep flow.
static void enter_system_sleep(void) {
    ESP_LOGI(TAG, "Entering Light Sleep Sequence");
    display_set_power(false);
    
    // Wait until touch is released.
    touch_data_t tdata;
    while (touch_read(&tdata) && tdata.points > 0) {
        vTaskDelay(pdMS_TO_TICKS(100));
        esp_task_wdt_reset();
    }

    // Suspend Wi-Fi so wake-up always follows a clean restart path.
    wifi_suspend_services();
    
    esp_sleep_enable_timer_wakeup(30 * 1000000);

    while (1) {
        esp_task_wdt_reset();

        gpio_wakeup_enable(TOUCH_IRQ_PIN, GPIO_INTR_LOW_LEVEL);
        esp_sleep_enable_gpio_wakeup();
        
        esp_light_sleep_start();
        
        if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_TIMER) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        // Device woke from GPIO.
        TickType_t wake_start = xTaskGetTickCount();
        bool long_press = false;
        
        while (xTaskGetTickCount() - wake_start < pdMS_TO_TICKS(1500)) {
            esp_task_wdt_reset();
            if (touch_read(&tdata) && tdata.points > 0) {
                if (xTaskGetTickCount() - wake_start > pdMS_TO_TICKS(1000)) {
                    long_press = true;
                    break;
                }
            } else {
                break; 
            }
            vTaskDelay(pdMS_TO_TICKS(50));
        }
        
        if (long_press) {
            display_set_power(true);
            wifi_resume_services();
            esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
            return;
        }
        
        // Wait until touch is released before sleeping again.
        while (touch_read(&tdata) && tdata.points > 0) {
            vTaskDelay(pdMS_TO_TICKS(100));
            esp_task_wdt_reset();
        }
    }
}

// Touch IRQ handler.
static void IRAM_ATTR touch_isr_handler(void* arg) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (touch_sem) {
        xSemaphoreGiveFromISR(touch_sem, &xHigherPriorityTaskWoken);
    }
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

// Process touch press, hold, and swipe gestures.
static void touch_task(void *arg) {
    esp_task_wdt_add(NULL);
    touch_data_t touch_data;
    bool is_pressed = false;
    TickType_t press_start_tick = 0;
    bool long_press_handled = false;
    static bool calibration_done = false;
    
    uint16_t start_x = 0;
    uint16_t last_x = 0;

    while (1) {
        esp_task_wdt_reset();
        // Poll with timeout so releases and long-presses are handled.
        if (xSemaphoreTake(touch_sem, pdMS_TO_TICKS(50)) == pdTRUE) {
            if (touch_read(&touch_data)) {
                if (touch_data.points > 0) {
                    last_x = touch_data.x;
                    if (!is_pressed) {
                        // New press started.
                        is_pressed = true;
                        press_start_tick = xTaskGetTickCount();
                        long_press_handled = false;
                        start_x = touch_data.x;
                    } else {
                        // Finger is still down.
                        if (!long_press_handled && (xTaskGetTickCount() - press_start_tick > pdMS_TO_TICKS(1000))) {
                            // Long press threshold reached (>1 s).
                            long_press_handled = true;
                            if (app_debug_logs_enabled()) {
                                ESP_LOGI(TAG, "Long Press Detected");
                            }
                            handle_long_press_action(&calibration_done);
                        }
                    }
                } else {
                    // Finger released.
                    if (is_pressed) {
                        is_pressed = false;
                        if (!long_press_handled) {
                            // Detect horizontal swipe.
                            if (display_get_power()) { // Swipe only when display is on.
                                int delta_x = (int)last_x - (int)start_x;
                                bool mode_changed = false;
                                
                                if (abs(delta_x) > 40) { // 40 px swipe threshold.
                                    if (delta_x > 0) {
                                        // Left-to-right: previous mode.
                                        display_prev_mode();
                                        mode_changed = true;
                                    } else {
                                        // Right-to-left: next mode.
                                        display_next_mode();
                                        mode_changed = true;
                                    }
                                }
                                
                                if (mode_changed) {
                                    display_mode_t mode = display_get_mode();
                                    if (mode == DISPLAY_MODE_CALIBRATION) {
                                        calibration_done = false; // Reset calibration state on mode entry.
                                    } else if (mode == DISPLAY_MODE_BATTERY) {
                                        check_battery_status();
                                    } else if (mode == DISPLAY_MODE_MAX_PRESSURE) {
                                        update_max_pressure_ui(last_max_idx, last_max_pressure);
                                    }
                                }
                            }
                        }
                    }
                }
            }
        } else {
            // Handle release/hold when no new IRQ arrived.
            if (is_pressed) {
                if (touch_read(&touch_data)) {
                    if (touch_data.points == 0) {
                        is_pressed = false;
                        if (!long_press_handled) {
                            // Detect swipe on timeout release path.
                            if (display_get_power()) {
                                int delta_x = (int)last_x - (int)start_x;
                                bool mode_changed = false;

                                if (abs(delta_x) > 40) {
                                    if (delta_x > 0) {
                                        display_prev_mode();
                                        mode_changed = true;
                                    } else {
                                        display_next_mode();
                                        mode_changed = true;
                                    }
                                }

                                if (mode_changed) {
                                    display_mode_t mode = display_get_mode();
                                    if (mode == DISPLAY_MODE_CALIBRATION) {
                                        calibration_done = false;
                                    } else if (mode == DISPLAY_MODE_BATTERY) {
                                        check_battery_status();
                                    } else if (mode == DISPLAY_MODE_MAX_PRESSURE) {
                                        update_max_pressure_ui(last_max_idx, last_max_pressure);
                                    }
                                }
                            }
                        }
                    } else {
                        // Still pressed: keep latest X position.
                        last_x = touch_data.x;
                        
                        // Still pressed: check long press timeout.
                         if (!long_press_handled && (xTaskGetTickCount() - press_start_tick > pdMS_TO_TICKS(1000))) {
                            long_press_handled = true;
                            if (app_debug_logs_enabled()) {
                                ESP_LOGI(TAG, "Long Press Detected (Timeout Path)");
                            }

                            handle_long_press_action(&calibration_done);
                        }
                    }
                }
            }
        }
    }
}

// Initialize shared I2C master bus.
static esp_err_t i2c_master_init(void)
{
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_MASTER_NUM,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    return i2c_new_master_bus(&i2c_mst_config, &main_bus_handle);
}

// Application entry point.
void app_main(void)
{
    // Configure task watchdog.
    esp_task_wdt_config_t twdt_config = {
        .timeout_ms = 60000,
        .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,
        .trigger_panic = true,
    };
    
    if (esp_task_wdt_reconfigure(&twdt_config) != ESP_OK) {
        ESP_ERROR_CHECK(esp_task_wdt_init(&twdt_config));
    }
    
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));

    ESP_ERROR_CHECK(i2c_master_init());
    
    gpio_set_direction(GPIO_NUM_13, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_13, 1);
    gpio_set_direction(BQ25628_STAT_PIN, GPIO_MODE_INPUT);
    
    vTaskDelay(pdMS_TO_TICKS(100));

    // Start LVGL task on core 1.
    setup_lvgl_display();
    display_set_recording_state(false, NULL);
    xTaskCreatePinnedToCore(lvgl_tick_task, "lvgl", 8192, NULL, 2, NULL, 1);

    // Initialize power and charger control.
    manage_usb_pd();
    vTaskDelay(pdMS_TO_TICKS(200));
    configure_bq25628_init();
    
    // Scan I2C bus for connected devices.
    run_i2c_scan("Main Bus");

    // Initialize touch controller.
    if (touch_init(main_bus_handle) == ESP_OK) {
        ESP_LOGI(TAG, "Touch Driver Initialized");
        
        // Setup touch interrupt task and ISR.
        touch_sem = xSemaphoreCreateBinary();
        xTaskCreate(touch_task, "touch_task", 4096, NULL, 5, NULL); // Priority 5: above main loop.
        
        gpio_install_isr_service(0);
        gpio_isr_handler_add(TOUCH_IRQ_PIN, touch_isr_handler, NULL);
    } else {
        ESP_LOGE(TAG, "Touch Driver Init Failed");
    }

    // Initialize DPS368 driver.
    dps368_init_driver(main_bus_handle);

    // Initialize BMI270 driver.
    static bmi270_sensor_t bmi270;
    if (bmi270_init_driver(main_bus_handle, &bmi270) != ESP_OK) {
        ESP_LOGE(TAG, "BMI270 Init Failed");
    }

    // Initialize BMM350 driver.
    static bmm350_sensor_t bmm350;
    if (bmm350_init_driver(main_bus_handle, &bmm350) != ESP_OK) {
        ESP_LOGE(TAG, "BMM350 Init Failed");
    }

    // Initialize all pressure sensors.
    int sensor_idx = 0;

    // Channels 0-3 each host two sensors (0x76 and 0x77).
    for (int ch = 0; ch < 4; ch++) {
        sensors[sensor_idx].channel = ch;
        sensors[sensor_idx].address = DPS368_ADDR_0;
        if (dps368_init_sensor(&sensors[sensor_idx]) == ESP_OK) {
            ESP_LOGI(TAG, "Sensor %d (Ch %d, 0x76) initialized", sensor_idx, ch);
        } else {
            ESP_LOGE(TAG, "Sensor %d (Ch %d, 0x76) init failed", sensor_idx, ch);
        }
        sensor_idx++;

        sensors[sensor_idx].channel = ch;
        sensors[sensor_idx].address = DPS368_ADDR_1;
        if (dps368_init_sensor(&sensors[sensor_idx]) == ESP_OK) {
            ESP_LOGI(TAG, "Sensor %d (Ch %d, 0x77) initialized", sensor_idx, ch);
        } else {
            ESP_LOGE(TAG, "Sensor %d (Ch %d, 0x77) init failed", sensor_idx, ch);
        }
        sensor_idx++;
    }

    // Channel 4 hosts one sensor (0x77).
    sensors[sensor_idx].channel = 4;
    sensors[sensor_idx].address = DPS368_ADDR_1;
    if (dps368_init_sensor(&sensors[sensor_idx]) == ESP_OK) {
        ESP_LOGI(TAG, "Sensor %d (Ch 4, 0x77) initialized", sensor_idx);
    } else {
        ESP_LOGE(TAG, "Sensor %d (Ch 4, 0x77) init failed", sensor_idx);
    }
    sensor_idx++;

    tca9548_disable_channels();

    load_calibration();

    // Discard first DPS frames to avoid startup outliers.
    warmup_dps_measurements(3, pdMS_TO_TICKS(125));

    apply_terminal_output_log_policy(display_get_terminal_output_mode());

    // Start Wi-Fi services.
    wifi_resume_services();

    ESP_LOGI(TAG, "Ready.");

    TickType_t last_batt_tick = 0;
    TickType_t last_bmi_tick = 0;
    TickType_t last_bmm_tick = 0;
    TickType_t last_dps_tick = xTaskGetTickCount();
    TickType_t last_log_tick = 0;

    const TickType_t batt_period = pdMS_TO_TICKS(1000);
    const TickType_t bmi_period = pdMS_TO_TICKS(200);   // 5 Hz.
    const TickType_t bmm_period = pdMS_TO_TICKS(100);   // 10 Hz compass.
    const TickType_t dps_period = pdMS_TO_TICKS(125);   // 8 Hz.
    const TickType_t log_period = pdMS_TO_TICKS(1000);  // 1 Hz SD logging.
    terminal_output_mode_t last_applied_mode = display_get_terminal_output_mode();
    
    while(1) {
        esp_task_wdt_reset();
        TickType_t now = xTaskGetTickCount();
        terminal_output_mode_t terminal_output_mode = display_get_terminal_output_mode();

        if (terminal_output_mode != last_applied_mode) {
            apply_terminal_output_log_policy(terminal_output_mode);
            last_applied_mode = terminal_output_mode;
        }

        if (now - last_batt_tick >= batt_period) {
            check_battery_status();
            manage_usb_pd();
            last_batt_tick = now;
        }

        if (now - last_bmi_tick >= bmi_period) {
            if (bmi270_read_data(&bmi270) == ESP_OK) {
                if (terminal_output_mode == TERMINAL_OUTPUT_DEBUG) {
                    ESP_LOGI(TAG, "BMI270: Acc[%.2f, %.2f, %.2f] Gyr[%.2f, %.2f, %.2f]",
                             bmi270.sensor_data.acc.x / 16384.0f,
                             bmi270.sensor_data.acc.y / 16384.0f,
                             bmi270.sensor_data.acc.z / 16384.0f,
                             bmi270.sensor_data.gyr.x * 2000.0f / 32768.0f,
                             bmi270.sensor_data.gyr.y * 2000.0f / 32768.0f,
                             bmi270.sensor_data.gyr.z * 2000.0f / 32768.0f);
                } else if (terminal_output_mode == TERMINAL_OUTPUT_RAW_DATA) {
                    printf("Acc: %.2f, %.2f, %.2f | Gyr: %.2f, %.2f, %.2f\n",
                           bmi270.sensor_data.acc.x / 16384.0f,
                           bmi270.sensor_data.acc.y / 16384.0f,
                           bmi270.sensor_data.acc.z / 16384.0f,
                           bmi270.sensor_data.gyr.x * 2000.0f / 32768.0f,
                           bmi270.sensor_data.gyr.y * 2000.0f / 32768.0f,
                           bmi270.sensor_data.gyr.z * 2000.0f / 32768.0f);
                }
            }
            last_bmi_tick = now;
        }

        if (now - last_bmm_tick >= bmm_period) {
            static int bmm350_fail_count = 0;
            if (bmm350_read_data(&bmm350) == ESP_OK) {
                bmm350_fail_count = 0;
                
                if (compass_cal_reset) {
                    min_x = max_x = bmm350.mag_x;
                    min_y = max_y = bmm350.mag_y;
                    min_z = max_z = bmm350.mag_z; // Reset Z-axis calibration bounds.
                    compass_cal_reset = false;
                    heading_filter_initialized = false;
                }

                if (compass_cal_active) {
                    if (bmm350.mag_x < min_x) min_x = bmm350.mag_x;
                    if (bmm350.mag_x > max_x) max_x = bmm350.mag_x;
                    if (bmm350.mag_y < min_y) min_y = bmm350.mag_y;
                    if (bmm350.mag_y > max_y) max_y = bmm350.mag_y;
                    if (bmm350.mag_z < min_z) min_z = bmm350.mag_z; // Update Z-axis calibration bounds.
                    if (bmm350.mag_z > max_z) max_z = bmm350.mag_z; 
                }

                // Compute hard-iron offsets from min/max midpoint per axis.
                float off_x = (min_x + max_x) / 2.0f;
                float off_y = (min_y + max_y) / 2.0f;
                float off_z = (min_z + max_z) / 2.0f; // Z-axis hard-iron offset.
                
                // Apply hard-iron compensation to raw magnetic field.
                float cal_x = bmm350.mag_x - off_x;
                float cal_y = bmm350.mag_y - off_y;
                float cal_z = bmm350.mag_z - off_z;

                // Tilt compensation using an NED frame (X=Forward, Y=Right, Z=Down).
                // Goal: remove roll/pitch influence so heading stays stable while tilted.

                // 1) Map accelerometer data to NED and convert to g-units.
                // These signs follow the validated board orientation.
                float g_x = -(bmi270.sensor_data.acc.y / 16384.0f); // Forward acceleration.
                float g_y = -(bmi270.sensor_data.acc.x / 16384.0f); // Right acceleration.
                float g_z =  (bmi270.sensor_data.acc.z / 16384.0f); // Down acceleration.

                // 2) Map calibrated magnetometer data to the same NED frame.
                // Matching frames is required before any tilt compensation math.
                float m_x = -cal_y; // Forward magnetic component.
                float m_y =  cal_x; // Right magnetic component.
                float m_z =  cal_z; // Down magnetic component.

                // 3) Estimate roll (phi) and pitch (theta) from gravity.
                // Gravity defines device tilt when linear acceleration is small.
                float phi = atan2f(g_y, g_z);
                float theta = atan2f(-g_x, sqrtf(g_y * g_y + g_z * g_z));

                // 4) Precompute trigonometric terms for rotation equations.
                float cos_phi = cosf(phi);
                float sin_phi = sinf(phi);
                float cos_theta = cosf(theta);
                float sin_theta = sinf(theta);

                // 5) Rotate the magnetic vector by roll/pitch to project it onto
                // the horizontal plane (tilt-compensated X/Y components).
                float b_fy = m_y * cos_phi - m_z * sin_phi;
                float b_fx = m_x * cos_theta + m_y * sin_theta * sin_phi + m_z * sin_theta * cos_phi;

                // 6) Compute heading from the compensated horizontal vector.
                // The sign in atan2 keeps the same clockwise convention as before.
                float heading = atan2f(-b_fx, b_fy) * 180.0f / (float)M_PI;

                // Normalize heading and smooth it with circular EMA (0/360 safe).
                float raw_heading = normalize_heading_deg(heading);
                float filtered_heading = smooth_heading_deg(raw_heading);
                last_heading_deg = filtered_heading;

                const char *dirs[] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
                int dir_idx = (int)((filtered_heading + 22.5) / 45.0) % 8;

                if (terminal_output_mode == TERMINAL_OUTPUT_DEBUG) {
                    ESP_LOGI(TAG, "BMM350 3D Cal: Off[%.1f, %.1f, %.1f] | Head Raw: %.1f | Head Filt: %.1f %s",
                             off_x, off_y, off_z, raw_heading, filtered_heading, dirs[dir_idx]);
                }

                display_update_heading(filtered_heading);
            } else {
                bmm350_fail_count++;
                if (bmm350_fail_count > 5) {
                    ESP_LOGW(TAG, "BMM350 failed %d times, re-initializing...", bmm350_fail_count);
                    if (bmm350_reinit(&bmm350) == ESP_OK) {
                        ESP_LOGI(TAG, "BMM350 re-init success");
                        bmm350_fail_count = 0;
                    } else {
                        ESP_LOGE(TAG, "BMM350 re-init failed");
                    }
                }
            }
            last_bmm_tick = now;
        }

        if (now - last_dps_tick >= dps_period) {
            bool dps_all_ok = true;

            // Read all pressure sensors.
            for (int i = 0; i < 9; i++) {
                if (dps368_measure(&sensors[i]) != ESP_OK) {
                    dps_all_ok = false;
                }

                if (terminal_output_mode == TERMINAL_OUTPUT_DEBUG) {
                    ESP_LOGI(TAG, "Sensor %d: %.2f C | %.2f Pa", i, sensors[i].temperature, sensors[i].pressure);
                } else if (terminal_output_mode == TERMINAL_OUTPUT_RAW_DATA) {
                    printf("P%d: %.2f Pa\n", i, sensors[i].pressure);
                }
            }

            if (!dps_all_ok) {
                if (terminal_output_mode == TERMINAL_OUTPUT_DEBUG) {
                    ESP_LOGW(TAG, "Skipping wind update due to DPS read error");
                }
                last_dps_tick = now;
                continue;
            }

            // Find maximum pressure delta relative to sensor 1.
            float max_diff = -100000.0f;
            int max_idx = -1;
            float ref_pressure = sensors[1].pressure;

            for (int i = 0; i < 9; i++) {
                float diff = sensors[i].pressure - ref_pressure;
                if (diff > max_diff) {
                    max_diff = diff;
                    max_idx = i;
                }
            }

            // Apply 2 Pa no-wind threshold.
            if (max_diff >= 2.0f) {
                last_max_idx = max_idx;
                last_max_pressure = max_diff;
            } else {
                last_max_idx = -2; // Sentinel for "No Wind".
                last_max_pressure = 0.0f;
            }

            update_max_pressure_ui(last_max_idx, last_max_pressure);

            float wind_ms = pressure_diff_to_wind_ms(last_max_pressure);
            int relative_angle_deg = (last_max_idx >= 0)
                                     ? wind_direction_deg_from_sensor_idx(last_max_idx)
                                     : -1;

            if (terminal_output_mode == TERMINAL_OUTPUT_PROCESSED_DATA) {
                int compass_heading_deg = (int)lroundf(heading_to_relative_frame(last_heading_deg));

                if (relative_angle_deg < 0) {
                    printf("Wind: %.1f m/s, No Wind, Compass: %d\xC2\xB0\n",
                           wind_ms,
                           compass_heading_deg);
                } else {
                    printf("Wind: %.1f m/s, %d\xC2\xB0, Compass: %d\xC2\xB0\n",
                           wind_ms,
                           relative_angle_deg,
                           compass_heading_deg);
                }
            }

            if (sdcard_is_recording() && (now - last_log_tick >= log_period)) {
                float pressures[9];
                for (int i = 0; i < 9; i++) {
                    pressures[i] = sensors[i].pressure;
                }

                uint64_t uptime_ms = esp_timer_get_time() / 1000ULL;
                esp_err_t log_ret = sdcard_log_data(uptime_ms,
                                                    wind_ms,
                                                    relative_angle_deg,
                                                    heading_to_relative_frame(last_heading_deg),
                                                    pressures,
                                                    9,
                                                    (int32_t)bmi270.sensor_data.acc.x,
                                                    (int32_t)bmi270.sensor_data.acc.y,
                                                    (int32_t)bmi270.sensor_data.acc.z,
                                                    (int32_t)bmi270.sensor_data.gyr.x,
                                                    (int32_t)bmi270.sensor_data.gyr.y,
                                                    (int32_t)bmi270.sensor_data.gyr.z);
                if (log_ret != ESP_OK) {
                    ESP_LOGW(TAG, "SD log failed: %s", esp_err_to_name(log_ret));
                }
                last_log_tick = now;
            }
            last_dps_tick = now;
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}