/*
 * Windsensor Firmware 
 * Author: Kristof Katzenberger
 * File: display.c
 */

#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <string.h>
#include "display.h"
#include "config.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_heap_caps.h"

#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_gc9a01.h"
#include "lvgl.h"

static const char *TAG = "DISPLAY";

// Global variables
static lv_display_t *display = NULL;
static lv_obj_t *label_status = NULL;
static lv_obj_t *compass_container = NULL;
static lv_obj_t *compass_labels[4] = {NULL};
static lv_obj_t *pressure_marker = NULL;
static lv_obj_t *calibration_tick = NULL;
static lv_obj_t *dots[6] = {NULL};
static lv_obj_t *recording_icon = NULL;
static SemaphoreHandle_t lvgl_mutex = NULL;
static display_mode_t current_mode = DISPLAY_MODE_MAX_PRESSURE;
static wind_unit_t current_wind_unit = WIND_UNIT_PA;
static terminal_output_mode_t current_terminal_output_mode = TERMINAL_OUTPUT_DEBUG;
static bool current_recording_active = false;
static char current_recording_filename[32] = {0};
static bool is_display_on = true;
static int last_pressure_sensor_idx = -1;
static float last_pressure_value = 0.0f;
static volatile float latest_heading_deg = 0.0f;
static volatile uint32_t heading_update_seq = 0;
static uint32_t heading_render_seq = 0;

// Cardinal label metadata
static const char *compass_letters[4] = {"N", "E", "S", "W"};
static const float compass_base_angles[4] = {0.0f, 270.0f, 180.0f, 90.0f};
static lv_coord_t compass_radius = 0;
static const float compass_heading_offset = -5.0f;

// forward declarations
static void create_compass_ring(lv_obj_t *parent);
static void position_compass_labels(float heading_deg);
static void create_dots_indicator(lv_obj_t *parent);
static void update_dots_indicator(void);
static void update_wind_unit_ui_locked(void);
static void update_terminal_output_ui_locked(void);
static void update_recording_ui_locked(void);
static void update_recording_icon_locked(bool active);
static void update_max_pressure_content_locked(int sensor_idx, float pressure);
static void set_menu_icon_locked(const char *symbol, lv_color_t color);
static int get_dot_count_for_mode(display_mode_t mode);
static int get_dot_index_for_mode(display_mode_t mode);

static const char *wind_unit_name(wind_unit_t unit) {
    switch (unit) {
        case WIND_UNIT_KMH:
            return "km/h";
        case WIND_UNIT_MS:
            return "m/s";
        case WIND_UNIT_PA:
        default:
            return "Pa";
    }
}

static const char *terminal_output_mode_name(terminal_output_mode_t mode) {
    switch (mode) {
        case TERMINAL_OUTPUT_RAW_DATA:
            return "Raw Data";
        case TERMINAL_OUTPUT_PROCESSED_DATA:
            return "Processed Data";
        case TERMINAL_OUTPUT_DEBUG:
        default:
            return "Debug";
    }
}

static float pressure_to_kmh(float pressure) {
    return (pressure * 0.55f) + 10.0f;
}

static float pressure_to_display_value(float pressure) {
    switch (current_wind_unit) {
        case WIND_UNIT_KMH:
            return pressure_to_kmh(pressure);
        case WIND_UNIT_MS:
            return pressure_to_kmh(pressure) / 3.6f;
        case WIND_UNIT_PA:
        default:
            return pressure;
    }
}

static int get_dot_count_for_mode(display_mode_t mode) {
    if (mode == DISPLAY_MODE_SETTINGS_BACK ||
        mode == DISPLAY_MODE_CALIBRATION ||
        mode == DISPLAY_MODE_COMPASS_CAL ||
        mode == DISPLAY_MODE_WIND_UNIT ||
        mode == DISPLAY_MODE_TERMINAL_OUTPUT) {
        return 5;
    }

    return 6;
}

static int get_dot_index_for_mode(display_mode_t mode) {
    switch (mode) {
        case DISPLAY_MODE_MAX_PRESSURE:
            return 0;
        case DISPLAY_MODE_RECORDING:
            return 1;
        case DISPLAY_MODE_BATTERY:
            return 2;
        case DISPLAY_MODE_SETTINGS:
            return 3;
        case DISPLAY_MODE_POWER:
            return 4;
        case DISPLAY_MODE_SLEEP:
            return 5;
        case DISPLAY_MODE_SETTINGS_BACK:
            return 0;
        case DISPLAY_MODE_CALIBRATION:
            return 1;
        case DISPLAY_MODE_COMPASS_CAL:
            return 2;
        case DISPLAY_MODE_WIND_UNIT:
            return 3;
        case DISPLAY_MODE_TERMINAL_OUTPUT:
            return 4;
        default:
            return 0;
    }
}

static void set_menu_icon_locked(const char *symbol, lv_color_t color) {
    lv_label_set_text(pressure_marker, symbol);
    lv_obj_set_style_text_font(pressure_marker, &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_color(pressure_marker, color, 0);
    lv_obj_set_style_transform_rotation(pressure_marker, 0, 0);
    lv_obj_set_style_transform_scale(pressure_marker, 256, 0);
    lv_obj_align(pressure_marker, LV_ALIGN_CENTER, 0, -40);
    lv_obj_remove_flag(pressure_marker, LV_OBJ_FLAG_HIDDEN);
}

void display_set_wind_unit(wind_unit_t unit) {
    wind_unit_t normalized_unit = unit;

    if (normalized_unit < WIND_UNIT_PA || normalized_unit > WIND_UNIT_MS) {
        normalized_unit = WIND_UNIT_PA;
    }

    if (lvgl_mutex && xSemaphoreTake(lvgl_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        current_wind_unit = normalized_unit;

        if (current_mode == DISPLAY_MODE_WIND_UNIT) {
            update_wind_unit_ui_locked();
        } else if (current_mode == DISPLAY_MODE_MAX_PRESSURE) {
            update_max_pressure_content_locked(last_pressure_sensor_idx, last_pressure_value);
        }

        xSemaphoreGive(lvgl_mutex);
    } else {
        current_wind_unit = normalized_unit;
    }
}

wind_unit_t display_get_wind_unit(void) {
    return current_wind_unit;
}

void display_set_terminal_output_mode(terminal_output_mode_t mode) {
    terminal_output_mode_t normalized_mode = mode;

    if (normalized_mode < TERMINAL_OUTPUT_DEBUG || normalized_mode > TERMINAL_OUTPUT_PROCESSED_DATA) {
        normalized_mode = TERMINAL_OUTPUT_DEBUG;
    }

    if (lvgl_mutex && xSemaphoreTake(lvgl_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        current_terminal_output_mode = normalized_mode;

        if (current_mode == DISPLAY_MODE_TERMINAL_OUTPUT) {
            update_terminal_output_ui_locked();
        }

        xSemaphoreGive(lvgl_mutex);
    } else {
        current_terminal_output_mode = normalized_mode;
    }
}

terminal_output_mode_t display_get_terminal_output_mode(void) {
    return current_terminal_output_mode;
}

// spi dma completion callback
static bool notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    // Signal LVGL that the buffer is free to be used again
    if (display) {
        lv_display_flush_ready(display);
    }
    return false;
}

// lvgl flush callback
static void lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) lv_display_get_user_data(disp);
    int x1 = area->x1;
    int x2 = area->x2;
    int y1 = area->y1;
    int y2 = area->y2;
    // optional byte swap for rgb565 little endian data
    uint16_t *p = (uint16_t *)px_map;
    size_t len = (x2 - x1 + 1) * (y2 - y1 + 1);
    for (int i = 0; i < len; i++) {
        p[i] = (p[i] << 8) | (p[i] >> 8);
    }

    // start the dma transfer
    esp_lcd_panel_draw_bitmap(panel_handle, x1, y1, x2 + 1, y2 + 1, px_map);

    // lvgl flush ready signal is sent by notify_lvgl_flush_ready
}

// initialize lvgl display hardware
void setup_lvgl_display(void) {
    lvgl_mutex = xSemaphoreCreateMutex();

    ESP_LOGI(TAG, "Initialize SPI bus");
    spi_bus_config_t buscfg = {
        .sclk_io_num = PIN_NUM_CLK,
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_H_RES * 80 * sizeof(uint16_t),
    };
    // use automatic dma channel for smooth updates
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = PIN_NUM_DC,
        .cs_gpio_num = PIN_NUM_CS,
        .pclk_hz = LCD_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits = LCD_CMD_BITS,
        .lcd_param_bits = LCD_PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = 10, // allows buffering up to 10 transfers
        .on_color_trans_done = notify_lvgl_flush_ready, // register callback
        .user_ctx = NULL, 
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle));

    ESP_LOGI(TAG, "Install GC9A01 panel driver");
    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = PIN_NUM_RST,
        .rgb_endian = LCD_RGB_ENDIAN_BGR, 
        .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_gc9a01(io_handle, &panel_config, &panel_handle));

    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    
    // Standard GC9A01 Orientation adjustments
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, true, false));
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, false)); // Usually false for GC9A01
    ESP_ERROR_CHECK(esp_lcd_panel_set_gap(panel_handle, 0, 0));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    if (PIN_NUM_BCKL >= 0) {
        gpio_set_direction(PIN_NUM_BCKL, GPIO_MODE_OUTPUT);
        gpio_set_level(PIN_NUM_BCKL, 1);
    }

    ESP_LOGI(TAG, "Initialize LVGL v9");
    lv_init();

    // create the lvgl display object
    display = lv_display_create(LCD_H_RES, LCD_V_RES);
    lv_display_set_flush_cb(display, lvgl_flush_cb);
    lv_display_set_user_data(display, panel_handle);

    lv_display_set_color_format(display, LV_COLOR_FORMAT_RGB565);
    
    // allocate buffer in dma-capable memory
    size_t buf_size = LCD_H_RES * 40 * sizeof(uint16_t); 
    void *buf1 = heap_caps_aligned_alloc(16, buf_size, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    
    lv_display_set_buffers(display, buf1, NULL, buf_size, LV_DISPLAY_RENDER_MODE_PARTIAL);

    // ui setup
    lv_obj_t *scr = lv_screen_active();
    lv_obj_set_style_bg_color(scr, lv_color_black(), 0);
    
    label_status = lv_label_create(scr);
    lv_label_set_text(label_status, "Reading Pressure...");
    lv_obj_set_width(label_status, LCD_H_RES - 24);
    lv_obj_set_style_text_align(label_status, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(label_status, lv_color_white(), 0);
    lv_obj_align(label_status, LV_ALIGN_CENTER, 0, 2);

    create_compass_ring(scr);
    // compass visible by default in max pressure mode

    pressure_marker = lv_label_create(scr);
    lv_label_set_text(pressure_marker, LV_SYMBOL_GPS);
    lv_obj_set_style_text_color(pressure_marker, lv_palette_main(LV_PALETTE_GREEN), 0);
    lv_obj_add_flag(pressure_marker, LV_OBJ_FLAG_HIDDEN);

    calibration_tick = lv_label_create(scr);
    lv_label_set_text(calibration_tick, LV_SYMBOL_OK);
    // lv_obj_set_style_text_font(calibration_tick, &lv_font_montserrat_48, 0); // commented out to avoid build error if not enabled
    lv_obj_set_style_transform_scale(calibration_tick, 400, 0); // Scale up 4x (256 is 1x)
    lv_obj_set_style_text_color(calibration_tick, lv_palette_main(LV_PALETTE_GREEN), 0);
    lv_obj_align(calibration_tick, LV_ALIGN_CENTER, 0, 40);
    lv_obj_add_flag(calibration_tick, LV_OBJ_FLAG_HIDDEN);

    recording_icon = lv_obj_create(scr);
    lv_obj_remove_style_all(recording_icon);
    lv_obj_set_size(recording_icon, 24, 24);
    lv_obj_set_style_bg_opa(recording_icon, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(recording_icon, lv_palette_main(LV_PALETTE_RED), 0);
    lv_obj_set_style_border_width(recording_icon, 0, 0);
    lv_obj_align(recording_icon, LV_ALIGN_CENTER, 0, -45);
    lv_obj_add_flag(recording_icon, LV_OBJ_FLAG_HIDDEN);

    create_dots_indicator(scr);
}

// build the mode indicator dots
static void create_dots_indicator(lv_obj_t *parent) {
    lv_obj_t *container = lv_obj_create(parent);
    lv_obj_remove_style_all(container);
    lv_obj_set_size(container, 120, 20);
    lv_obj_align(container, LV_ALIGN_BOTTOM_MID, 0, -50);
    lv_obj_set_flex_flow(container, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(container, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_gap(container, 10, 0);

    for (int i = 0; i < 6; i++) {
        dots[i] = lv_obj_create(container);
        lv_obj_set_size(dots[i], 6, 6);
        lv_obj_set_style_radius(dots[i], LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_bg_opa(dots[i], LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(dots[i], 0, 0);
    }
    update_dots_indicator();
}

// refresh active dot state
static void update_dots_indicator(void) {
    int active_index = get_dot_index_for_mode(current_mode);
    int visible_count = get_dot_count_for_mode(current_mode);

    for (int i = 0; i < 6; i++) {
        if (dots[i]) {
            if (i < visible_count) {
                lv_obj_remove_flag(dots[i], LV_OBJ_FLAG_HIDDEN);
                if (i == active_index) {
                    lv_obj_set_style_bg_color(dots[i], lv_color_white(), 0);
                } else {
                    lv_obj_set_style_bg_color(dots[i], lv_palette_main(LV_PALETTE_GREY), 0);
                }
            } else {
                lv_obj_add_flag(dots[i], LV_OBJ_FLAG_HIDDEN);
            }
        }
    }
}

// draw the compass ring labels
static void create_compass_ring(lv_obj_t *parent) {
    compass_container = lv_obj_create(parent);
    lv_obj_remove_style_all(compass_container);
    lv_obj_set_size(compass_container, LCD_H_RES, LCD_V_RES);
    lv_obj_center(compass_container);
    lv_obj_set_style_bg_opa(compass_container, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(compass_container, 0, 0);

    lv_coord_t min_dim = LV_MIN(LCD_H_RES, LCD_V_RES);
    lv_coord_t margin = 20;
    compass_radius = (min_dim / 2) - margin;
    if (compass_radius < 0) {
        compass_radius = 0;
    }

    for (int i = 0; i < 4; i++) {
        compass_labels[i] = lv_label_create(compass_container);
        lv_label_set_text(compass_labels[i], compass_letters[i]);
        lv_color_t label_color = (i == 0) ? lv_palette_main(LV_PALETTE_RED) : lv_color_white();
        lv_obj_set_style_text_color(compass_labels[i], label_color, 0);
    }

    position_compass_labels(0.0f);
}

// reposition compass labels based on heading
static void position_compass_labels(float heading_deg) {
    if (!compass_container) {
        return;
    }

    const float deg_to_rad = 0.0174532925f; // PI / 180
    lv_coord_t center_x = LCD_H_RES / 2;
    lv_coord_t center_y = LCD_V_RES / 2;

    float adjusted_heading = heading_deg + compass_heading_offset;

    for (int i = 0; i < 4; i++) {
        if (!compass_labels[i]) {
            continue;
        }

        float angle = (compass_base_angles[i] - adjusted_heading) * deg_to_rad;
        lv_coord_t x = center_x + (lv_coord_t)(cosf(angle) * compass_radius);
        lv_coord_t y = center_y - (lv_coord_t)(sinf(angle) * compass_radius);

        lv_coord_t label_w = lv_obj_get_width(compass_labels[i]);
        lv_coord_t label_h = lv_obj_get_height(compass_labels[i]);

        lv_obj_set_pos(compass_labels[i], x - (label_w / 2), y - (label_h / 2));
    }
}

// update heading indicator
void display_update_heading(float heading_deg) {
    if (!compass_container) {
        return;
    }

    while (heading_deg < 0.0f) {
        heading_deg += 360.0f;
    }
    while (heading_deg >= 360.0f) {
        heading_deg -= 360.0f;
    }

    // Buffer latest heading; LVGL task applies it under the mutex.
    latest_heading_deg = heading_deg;
    heading_update_seq++;
}

// run the lvgl tick handler
void lvgl_tick_task(void *arg) {
    TickType_t last_interaction = xTaskGetTickCount();
    bool last_state = true;
    
    while (1) {
        // lock access to lvgl while the timer runs
        if (lvgl_mutex && xSemaphoreTake(lvgl_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            lv_tick_inc(10);
            lv_timer_handler();

            uint32_t pending_seq = heading_update_seq;
            if (pending_seq != heading_render_seq && compass_container) {
                position_compass_labels((float)latest_heading_deg);
                heading_render_seq = pending_seq;
            }

            xSemaphoreGive(lvgl_mutex);
        }

        // detect if display was turned on externally
        if (is_display_on && !last_state) {
            last_interaction = xTaskGetTickCount();
        }
        last_state = is_display_on;

        // turn off display after ten minutes
        if (is_display_on && (xTaskGetTickCount() - last_interaction > pdMS_TO_TICKS(600000))) {
            ESP_LOGI(TAG, "Turning off display after 10m timeout");
            display_set_power(false);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void update_wind_unit_ui_locked(void) {
    char buf[64];

    snprintf(buf, sizeof(buf), "Units\nLong Press\n%s", wind_unit_name(current_wind_unit));
    lv_label_set_text(label_status, buf);
    lv_obj_align(label_status, LV_ALIGN_CENTER, 0, 0);
}

static void update_terminal_output_ui_locked(void) {
    char buf[64];

    snprintf(buf,
             sizeof(buf),
             "Terminal\nLong Press\n%s",
             terminal_output_mode_name(current_terminal_output_mode));
    lv_label_set_text(label_status, buf);
    lv_obj_align(label_status, LV_ALIGN_CENTER, 0, 0);
}

static void update_recording_ui_locked(void) {
    char buf[96];

    if (current_recording_active) {
        if (current_recording_filename[0] != '\0') {
            snprintf(buf,
                     sizeof(buf),
                     "Recording ON\n%s\nLong Press Stop",
                     current_recording_filename);
        } else {
            snprintf(buf, sizeof(buf), "Recording ON\nLong Press Stop");
        }
    } else {
        snprintf(buf, sizeof(buf), "Recording OFF\nLong Press Start");
    }

    lv_label_set_text(label_status, buf);
    lv_obj_align(label_status, LV_ALIGN_CENTER, 0, 0);
    update_recording_icon_locked(current_recording_active);
}

static void update_recording_icon_locked(bool active) {
    if (!recording_icon) {
        return;
    }

    lv_obj_set_size(recording_icon, active ? 22 : 24, active ? 22 : 24);
    lv_obj_set_style_radius(recording_icon,
                            active ? 2 : LV_RADIUS_CIRCLE,
                            0);
    lv_obj_set_style_bg_color(recording_icon, lv_palette_main(LV_PALETTE_RED), 0);
    lv_obj_align(recording_icon, LV_ALIGN_CENTER, 0, -45);
}

void display_set_recording_state(bool active, const char *filename) {
    if (lvgl_mutex && xSemaphoreTake(lvgl_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        current_recording_active = active;

        if (filename && filename[0] != '\0') {
            strncpy(current_recording_filename, filename, sizeof(current_recording_filename) - 1);
            current_recording_filename[sizeof(current_recording_filename) - 1] = '\0';
        } else {
            current_recording_filename[0] = '\0';
        }

        if (current_mode == DISPLAY_MODE_RECORDING) {
            update_recording_ui_locked();
        }

        xSemaphoreGive(lvgl_mutex);
    } else {
        current_recording_active = active;

        if (filename && filename[0] != '\0') {
            strncpy(current_recording_filename, filename, sizeof(current_recording_filename) - 1);
            current_recording_filename[sizeof(current_recording_filename) - 1] = '\0';
        } else {
            current_recording_filename[0] = '\0';
        }
    }
}

static void update_max_pressure_content_locked(int sensor_idx, float pressure) {
    char buf[64];

    if (sensor_idx >= 0) {
        snprintf(buf,
                 sizeof(buf),
                 "Max Pressure:\nSensor %d\n+%.2f %s",
                 sensor_idx,
                 pressure_to_display_value(pressure),
                 wind_unit_name(current_wind_unit));
    } else if (sensor_idx == -2) {
        snprintf(buf, sizeof(buf), "No Wind");
    } else {
        snprintf(buf, sizeof(buf), "Max Pressure:\nReading...");
    }

    lv_label_set_text(label_status, buf);
    lv_obj_align(label_status, LV_ALIGN_CENTER, 0, 0);

    if (sensor_idx >= 0 && sensor_idx != 1) {
        int angle = -1;
        switch(sensor_idx) {
            case 6: angle = 0; break;
            case 5: angle = 45; break;
            case 4: angle = 90; break;
            case 3: angle = 135; break;
            case 2: angle = 180; break;
            case 0: angle = 225; break;
            case 8: angle = 270; break;
            case 7: angle = 315; break;
            default: angle = -1; break;
        }

        if (angle != -1) {
            float rad_angle = angle * 0.0174532925f;
            lv_coord_t x = (lv_coord_t)(sinf(rad_angle) * compass_radius);
            lv_coord_t y = (lv_coord_t)(-cosf(rad_angle) * compass_radius);

            lv_obj_align(pressure_marker, LV_ALIGN_CENTER, x, y);

            int32_t rotation = (angle + 135) * 10;
            lv_obj_set_style_transform_rotation(pressure_marker, rotation, 0);

            lv_obj_remove_flag(pressure_marker, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_add_flag(pressure_marker, LV_OBJ_FLAG_HIDDEN);
        }
    } else {
        lv_obj_add_flag(pressure_marker, LV_OBJ_FLAG_HIDDEN);
    }
}

// show battery status text
void update_ui_text(float vbat, int ibat, const char* status) {
    if (current_mode == DISPLAY_MODE_BATTERY && label_status && lvgl_mutex) {
        if (xSemaphoreTake(lvgl_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            char buf[128];
            snprintf(buf, sizeof(buf), "Power:\n%.0f mV\n%d mA\n%s", vbat, ibat, status);
            lv_label_set_text(label_status, buf);
            lv_obj_align(label_status, LV_ALIGN_CENTER, 0, 0);
            xSemaphoreGive(lvgl_mutex);
        }
    }
}

// render max pressure status
void update_max_pressure_ui(int sensor_idx, float pressure) {
    last_pressure_sensor_idx = sensor_idx;
    last_pressure_value = pressure;

    if (current_mode == DISPLAY_MODE_MAX_PRESSURE && label_status && lvgl_mutex) {
        if (xSemaphoreTake(lvgl_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            update_max_pressure_content_locked(sensor_idx, pressure);

            xSemaphoreGive(lvgl_mutex);
        }
    }
}

// adjust ui for the selected mode
static void update_mode_ui(void) {
    lv_coord_t label_y_offset = 20;

    if (recording_icon) {
        lv_obj_add_flag(recording_icon, LV_OBJ_FLAG_HIDDEN);
    }

    if (current_mode == DISPLAY_MODE_BATTERY) {
        lv_label_set_text(label_status, "Reading Power...");
        lv_obj_add_flag(pressure_marker, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(compass_container, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(calibration_tick, LV_OBJ_FLAG_HIDDEN);
    } else if (current_mode == DISPLAY_MODE_MAX_PRESSURE) {
        // Reset pressure_marker for Wind Direction
        lv_label_set_text(pressure_marker, LV_SYMBOL_GPS);
        lv_obj_set_style_text_font(pressure_marker, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(pressure_marker, lv_palette_main(LV_PALETTE_GREEN), 0);
        lv_obj_set_style_transform_scale(pressure_marker, 256, 0); // Reset scale to 1x
        // We hide it initially until update_max_pressure_ui positions it correctly
        lv_obj_add_flag(pressure_marker, LV_OBJ_FLAG_HIDDEN);

        lv_obj_remove_flag(compass_container, LV_OBJ_FLAG_HIDDEN); // Show compass in pressure mode
        lv_obj_add_flag(calibration_tick, LV_OBJ_FLAG_HIDDEN);
        update_max_pressure_content_locked(last_pressure_sensor_idx, last_pressure_value);
    } else if (current_mode == DISPLAY_MODE_SETTINGS) {
        lv_label_set_text(label_status, "Settings\nLong Press");
        set_menu_icon_locked(LV_SYMBOL_SETTINGS, lv_color_white());
        lv_obj_add_flag(compass_container, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(calibration_tick, LV_OBJ_FLAG_HIDDEN);
    } else if (current_mode == DISPLAY_MODE_RECORDING) {
        update_recording_ui_locked();
        lv_obj_add_flag(pressure_marker, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(compass_container, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(calibration_tick, LV_OBJ_FLAG_HIDDEN);
        if (recording_icon) {
            lv_obj_remove_flag(recording_icon, LV_OBJ_FLAG_HIDDEN);
        }
        label_y_offset = 0;
    } else if (current_mode == DISPLAY_MODE_WIND_UNIT) {
        update_wind_unit_ui_locked();
        lv_obj_add_flag(pressure_marker, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(compass_container, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(calibration_tick, LV_OBJ_FLAG_HIDDEN);
        label_y_offset = 0;
    } else if (current_mode == DISPLAY_MODE_TERMINAL_OUTPUT) {
        update_terminal_output_ui_locked();
        lv_obj_add_flag(pressure_marker, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(compass_container, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(calibration_tick, LV_OBJ_FLAG_HIDDEN);
        label_y_offset = 0;
    } else if (current_mode == DISPLAY_MODE_SETTINGS_BACK) {
        lv_label_set_text(label_status, "Back\nLong Press");
        set_menu_icon_locked(LV_SYMBOL_LEFT, lv_color_white());
        lv_obj_add_flag(compass_container, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(calibration_tick, LV_OBJ_FLAG_HIDDEN);
    } else if (current_mode == DISPLAY_MODE_CALIBRATION) {
        lv_label_set_text(label_status, "Sensor\nCalibration\nLong Press");
        lv_obj_add_flag(pressure_marker, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(compass_container, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(calibration_tick, LV_OBJ_FLAG_HIDDEN);
        label_y_offset = 0;
    } else if (current_mode == DISPLAY_MODE_COMPASS_CAL) {
        lv_label_set_text(label_status, "Compass\nCalibration\nLong Press");
        lv_obj_add_flag(pressure_marker, LV_OBJ_FLAG_HIDDEN);
        lv_obj_remove_flag(compass_container, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(calibration_tick, LV_OBJ_FLAG_HIDDEN);
        label_y_offset = 0;
    } else if (current_mode == DISPLAY_MODE_POWER) {
        lv_label_set_text(label_status, "Screen Off\nLong Press");
        set_menu_icon_locked(LV_SYMBOL_IMAGE, lv_palette_main(LV_PALETTE_BLUE));

        lv_obj_add_flag(compass_container, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(calibration_tick, LV_OBJ_FLAG_HIDDEN);
    } else if (current_mode == DISPLAY_MODE_SLEEP) {
        lv_label_set_text(label_status, "Power Off\nLong Press");
        set_menu_icon_locked(LV_SYMBOL_POWER, lv_palette_main(LV_PALETTE_RED));

        lv_obj_add_flag(compass_container, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(calibration_tick, LV_OBJ_FLAG_HIDDEN);
    }
    lv_obj_align(label_status, LV_ALIGN_CENTER, 0, label_y_offset);
    update_dots_indicator();
}

// advance to the next display mode
void display_next_mode(void) {
    if (lvgl_mutex && xSemaphoreTake(lvgl_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        if (current_mode == DISPLAY_MODE_MAX_PRESSURE) current_mode = DISPLAY_MODE_RECORDING;
        else if (current_mode == DISPLAY_MODE_RECORDING) current_mode = DISPLAY_MODE_BATTERY;
        else if (current_mode == DISPLAY_MODE_BATTERY) current_mode = DISPLAY_MODE_SETTINGS;
        else if (current_mode == DISPLAY_MODE_SETTINGS) current_mode = DISPLAY_MODE_POWER;
        else if (current_mode == DISPLAY_MODE_POWER) current_mode = DISPLAY_MODE_SLEEP;
        else if (current_mode == DISPLAY_MODE_SLEEP) current_mode = DISPLAY_MODE_MAX_PRESSURE;
        else if (current_mode == DISPLAY_MODE_SETTINGS_BACK) current_mode = DISPLAY_MODE_CALIBRATION;
        else if (current_mode == DISPLAY_MODE_CALIBRATION) current_mode = DISPLAY_MODE_COMPASS_CAL;
        else if (current_mode == DISPLAY_MODE_COMPASS_CAL) current_mode = DISPLAY_MODE_WIND_UNIT;
        else if (current_mode == DISPLAY_MODE_WIND_UNIT) current_mode = DISPLAY_MODE_TERMINAL_OUTPUT;
        else current_mode = DISPLAY_MODE_SETTINGS_BACK;
        
        update_mode_ui();
        xSemaphoreGive(lvgl_mutex);
    }
}

// go to the previous display mode
void display_prev_mode(void) {
    if (lvgl_mutex && xSemaphoreTake(lvgl_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        if (current_mode == DISPLAY_MODE_MAX_PRESSURE) current_mode = DISPLAY_MODE_SLEEP;
        else if (current_mode == DISPLAY_MODE_SLEEP) current_mode = DISPLAY_MODE_POWER;
        else if (current_mode == DISPLAY_MODE_POWER) current_mode = DISPLAY_MODE_SETTINGS;
        else if (current_mode == DISPLAY_MODE_SETTINGS) current_mode = DISPLAY_MODE_BATTERY;
        else if (current_mode == DISPLAY_MODE_BATTERY) current_mode = DISPLAY_MODE_RECORDING;
        else if (current_mode == DISPLAY_MODE_RECORDING) current_mode = DISPLAY_MODE_MAX_PRESSURE;
        else if (current_mode == DISPLAY_MODE_SETTINGS_BACK) current_mode = DISPLAY_MODE_TERMINAL_OUTPUT;
        else if (current_mode == DISPLAY_MODE_WIND_UNIT) current_mode = DISPLAY_MODE_COMPASS_CAL;
        else if (current_mode == DISPLAY_MODE_COMPASS_CAL) current_mode = DISPLAY_MODE_CALIBRATION;
        else if (current_mode == DISPLAY_MODE_TERMINAL_OUTPUT) current_mode = DISPLAY_MODE_WIND_UNIT;
        else current_mode = DISPLAY_MODE_SETTINGS_BACK;
        
        update_mode_ui();
        xSemaphoreGive(lvgl_mutex);
    }
}

void display_enter_settings_menu(void) {
    if (lvgl_mutex && xSemaphoreTake(lvgl_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        current_mode = DISPLAY_MODE_CALIBRATION;
        update_mode_ui();
        xSemaphoreGive(lvgl_mutex);
    }
}

void display_exit_settings_menu(void) {
    if (lvgl_mutex && xSemaphoreTake(lvgl_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        current_mode = DISPLAY_MODE_SETTINGS;
        update_mode_ui();
        xSemaphoreGive(lvgl_mutex);
    }
}

void display_cycle_wind_unit(void) {
    wind_unit_t next_unit = WIND_UNIT_PA;

    if (current_wind_unit == WIND_UNIT_PA) {
        next_unit = WIND_UNIT_KMH;
    } else if (current_wind_unit == WIND_UNIT_KMH) {
        next_unit = WIND_UNIT_MS;
    }

    display_set_wind_unit(next_unit);
}

void display_cycle_terminal_output_mode(void) {
    terminal_output_mode_t next_mode = TERMINAL_OUTPUT_DEBUG;

    if (current_terminal_output_mode == TERMINAL_OUTPUT_DEBUG) {
        next_mode = TERMINAL_OUTPUT_RAW_DATA;
    } else if (current_terminal_output_mode == TERMINAL_OUTPUT_RAW_DATA) {
        next_mode = TERMINAL_OUTPUT_PROCESSED_DATA;
    }

    display_set_terminal_output_mode(next_mode);
}

// switch the display backlight and panel power
void display_set_power(bool on) {
    if (lvgl_mutex && xSemaphoreTake(lvgl_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        is_display_on = on;
        if (PIN_NUM_BCKL >= 0) {
            gpio_set_level(PIN_NUM_BCKL, on ? 1 : 0);
        }
        
        if (display) {
            esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) lv_display_get_user_data(display);
            if (panel_handle) {
                esp_lcd_panel_disp_on_off(panel_handle, on);
            }
        }
        xSemaphoreGive(lvgl_mutex);
    }
}

// report the current display power state
bool display_get_power(void) {
    return is_display_on;
}

// convenience toggle that advances the mode
void display_toggle_mode(void) {
    display_next_mode();
}

// placeholder for future calibration text updates
void update_calibration_ui(void) {
}

// show a confirmation checkmark after calibration
void display_show_calibration_tick(void) {
    if (lvgl_mutex && xSemaphoreTake(lvgl_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        lv_obj_remove_flag(calibration_tick, LV_OBJ_FLAG_HIDDEN);
        lv_label_set_text(label_status, "Calibrated!");
        lv_obj_align(label_status, LV_ALIGN_CENTER, 0, -20);
        xSemaphoreGive(lvgl_mutex);
    }
}

// show compass calibration instructions
void display_show_compass_cal_text(const char* text) {
    if (lvgl_mutex && xSemaphoreTake(lvgl_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        lv_label_set_text(label_status, text);
        lv_obj_align(label_status, LV_ALIGN_CENTER, 0, 20);
        xSemaphoreGive(lvgl_mutex);
    }
}

// expose the current mode to other modules
display_mode_t display_get_mode(void) {
    return current_mode;
}