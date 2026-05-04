/*
 * Windsensor Firmware 
 * Author: Kristof Katzenberger
 * File: display.h
 */

#ifndef DISPLAY_H
#define DISPLAY_H

#include <stdbool.h>

typedef enum {
    DISPLAY_MODE_MAX_PRESSURE,
    DISPLAY_MODE_BATTERY,
    DISPLAY_MODE_SETTINGS,
    DISPLAY_MODE_RECORDING,
    DISPLAY_MODE_POWER,
    DISPLAY_MODE_SLEEP,
    DISPLAY_MODE_SETTINGS_BACK,
    DISPLAY_MODE_CALIBRATION,
    DISPLAY_MODE_COMPASS_CAL,
    DISPLAY_MODE_WIND_UNIT,
    DISPLAY_MODE_TERMINAL_OUTPUT
} display_mode_t;

typedef enum {
    WIND_UNIT_PA,
    WIND_UNIT_KMH,
    WIND_UNIT_MS
} wind_unit_t;

typedef enum {
    TERMINAL_OUTPUT_DEBUG,
    TERMINAL_OUTPUT_RAW_DATA,
    TERMINAL_OUTPUT_PROCESSED_DATA
} terminal_output_mode_t;

void setup_lvgl_display(void);
void lvgl_tick_task(void *arg);
void update_ui_text(float vbat, int ibat, const char* status);
void update_max_pressure_ui(int sensor_idx, float pressure);
void update_calibration_ui(void);
void display_show_calibration_tick(void);
void display_show_compass_cal_text(const char* text);
void display_update_heading(float heading_deg);
void display_set_wind_unit(wind_unit_t unit);
wind_unit_t display_get_wind_unit(void);
void display_set_terminal_output_mode(terminal_output_mode_t mode);
terminal_output_mode_t display_get_terminal_output_mode(void);
void display_set_recording_state(bool active, const char *filename);
void display_enter_settings_menu(void);
void display_exit_settings_menu(void);
void display_toggle_mode(void);
void display_next_mode(void);
void display_prev_mode(void);
void display_cycle_wind_unit(void);
void display_cycle_terminal_output_mode(void);
display_mode_t display_get_mode(void);
void display_set_power(bool on);
bool display_get_power(void);

#endif