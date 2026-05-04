/*
 * Windsensor Firmware 
 * Author: Kristof Katzenberger
 * File: sdcard.c
 */

#include "sdcard.h"

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <errno.h>
#include <dirent.h>
#include <sys/stat.h>
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "driver/sdmmc_host.h"
#include "sdmmc_cmd.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "config.h"

// Global state
static const char *TAG = "SDCARD";
static sdmmc_card_t *s_card = NULL;
static bool s_mounted = false;
static bool s_recording_active = false;
static char s_active_log_path[64] = {0};
static SemaphoreHandle_t s_state_mutex = NULL;

// Mount point path
static const char *MOUNT_POINT = "/sdcard";

static void ensure_state_mutex(void) {
	if (!s_state_mutex) {
		s_state_mutex = xSemaphoreCreateMutex();
	}
}

static void write_log_header(FILE *f) {
	fprintf(f,
	        "uptime_ms,wind_speed_ms,relative_angle_deg,compass_angle_deg,"
	        "press0_raw_pa,press1_raw_pa,press2_raw_pa,press3_raw_pa,press4_raw_pa,"
	        "press5_raw_pa,press6_raw_pa,press7_raw_pa,press8_raw_pa,"
	        "accel_x_raw,accel_y_raw,accel_z_raw,gyro_x_raw,gyro_y_raw,gyro_z_raw\n");
}

static esp_err_t get_next_log_index(uint32_t *next_index) {
	DIR *dir = opendir(MOUNT_POINT);
	if (!dir) {
		ESP_LOGE(TAG, "Failed to open SD root directory");
		return ESP_FAIL;
	}

	uint32_t max_index = 0;
	struct dirent *entry = NULL;

	while ((entry = readdir(dir)) != NULL) {
		unsigned long index = 0;
		int consumed = 0;

		if (sscanf(entry->d_name, "wind_log_%lu.csv%n", &index, &consumed) == 1 &&
			consumed == (int)strlen(entry->d_name) &&
			index > (unsigned long)max_index) {
			max_index = (uint32_t)index;
		}
	}

	closedir(dir);
	*next_index = max_index + 1;
	return ESP_OK;
}

// Initialize SD card interface
esp_err_t sdcard_init(void) {
	if (s_mounted) {
		return ESP_OK;
	}

	esp_vfs_fat_sdmmc_mount_config_t mount_config = {
		.format_if_mount_failed = false,
		.max_files = 12,
		.allocation_unit_size = 16 * 1024,
		.disk_status_check_enable = false,
	};

	sdmmc_host_t host = SDMMC_HOST_DEFAULT();
	host.max_freq_khz = SDMMC_FREQ_DEFAULT;

	sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
	slot_config.width = 4;
	slot_config.clk = SD_PIN_CLK;
	slot_config.cmd = SD_PIN_CMD;
	slot_config.d0 = SD_PIN_D0;
	slot_config.d1 = SD_PIN_D1;
	slot_config.d2 = SD_PIN_D2;
	slot_config.d3 = SD_PIN_D3;
	slot_config.gpio_cd = SD_PIN_CD;
	slot_config.gpio_wp = GPIO_NUM_NC;
	slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

	esp_err_t ret = esp_vfs_fat_sdmmc_mount(MOUNT_POINT, &host, &slot_config, &mount_config, &s_card);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Failed to mount SD card: %s", esp_err_to_name(ret));
		return ret;
	}

	ESP_LOGI(TAG, "SD card mounted. Name: %s", s_card->cid.name);
	s_mounted = true;

	return ESP_OK;
}

esp_err_t sdcard_start_recording(char *filename_buf, size_t filename_buf_size) {
	if (!s_mounted) {
		esp_err_t init_ret = sdcard_init();
		if (init_ret != ESP_OK) {
			return init_ret;
		}
	}

	ensure_state_mutex();
	if (!s_state_mutex) {
		return ESP_ERR_NO_MEM;
	}

	if (xSemaphoreTake(s_state_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
		if (s_recording_active) {
			xSemaphoreGive(s_state_mutex);
			return ESP_ERR_INVALID_STATE;
		}
		xSemaphoreGive(s_state_mutex);
	} else {
		return ESP_ERR_TIMEOUT;
	}

	uint32_t next_index = 0;
	esp_err_t idx_ret = get_next_log_index(&next_index);
	if (idx_ret != ESP_OK) {
		return idx_ret;
	}

	char filename[32] = {0};
	char path[64] = {0};
	bool created = false;

	for (uint32_t offset = 0; offset < 1000; offset++) {
		unsigned long candidate_index = (unsigned long)next_index + (unsigned long)offset;
		snprintf(filename, sizeof(filename), "wind_log_%03lu.csv", candidate_index);
		snprintf(path, sizeof(path), "%s/%s", MOUNT_POINT, filename);

		struct stat st = {0};
		if (stat(path, &st) == 0) {
			continue;
		}

		FILE *f = fopen(path, "w");
		if (!f) {
			ESP_LOGE(TAG,
				     "Failed to create log file %s (errno=%d, %s)",
				     path,
				     errno,
				     strerror(errno));
			if (errno == EMFILE || errno == ENFILE) {
				vTaskDelay(pdMS_TO_TICKS(20));
				continue;
			}
			return ESP_FAIL;
		}

		write_log_header(f);
		if (fflush(f) != 0) {
			ESP_LOGE(TAG,
				     "Header flush failed for %s (errno=%d, %s)",
				     path,
				     errno,
				     strerror(errno));
			fclose(f);
			return ESP_FAIL;
		}

		fclose(f);
		created = true;
		break;
	}

	if (!created) {
		ESP_LOGE(TAG, "Failed to allocate unique log filename on SD card");
		return ESP_FAIL;
	}

	if (xSemaphoreTake(s_state_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
		s_recording_active = true;
		strncpy(s_active_log_path, path, sizeof(s_active_log_path) - 1);
		s_active_log_path[sizeof(s_active_log_path) - 1] = '\0';
		xSemaphoreGive(s_state_mutex);
	} else {
		return ESP_ERR_TIMEOUT;
	}

	if (filename_buf && filename_buf_size > 0) {
		strncpy(filename_buf, filename, filename_buf_size - 1);
		filename_buf[filename_buf_size - 1] = '\0';
	}

	ESP_LOGI(TAG, "Recording started: %s", filename);
	return ESP_OK;
}

esp_err_t sdcard_stop_recording(void) {
	ensure_state_mutex();
	if (!s_state_mutex) {
		return ESP_ERR_NO_MEM;
	}

	if (xSemaphoreTake(s_state_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
		s_recording_active = false;
		s_active_log_path[0] = '\0';
		xSemaphoreGive(s_state_mutex);
		ESP_LOGI(TAG, "Recording stopped");
		return ESP_OK;
	}

	return ESP_ERR_TIMEOUT;
}

bool sdcard_is_recording(void) {
	bool is_recording = false;

	ensure_state_mutex();
	if (!s_state_mutex) {
		return false;
	}

	if (xSemaphoreTake(s_state_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
		is_recording = s_recording_active;
		xSemaphoreGive(s_state_mutex);
	}

	return is_recording;
}

// Unmount SD card
esp_err_t sdcard_unmount(void) {
	if (!s_mounted) {
		return ESP_OK;
	}
	esp_vfs_fat_sdcard_unmount(MOUNT_POINT, s_card);
	s_card = NULL;
	s_mounted = false;

	ensure_state_mutex();
	if (s_state_mutex && xSemaphoreTake(s_state_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
		s_recording_active = false;
		s_active_log_path[0] = '\0';
		xSemaphoreGive(s_state_mutex);
	}

	ESP_LOGW(TAG, "SD card unmounted");
	return ESP_OK;
}

// Append measurement data
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
	                      int32_t gyro_z_raw) {
	if (!pressures || pressure_count == 0) {
		return ESP_ERR_INVALID_ARG;
	}

	if (!s_mounted) {
		if (sdcard_init() != ESP_OK) {
			ESP_LOGW(TAG, "SD card not mounted; skipping log");
			return ESP_ERR_INVALID_STATE;
		}
	}

	char path[64] = {0};

	ensure_state_mutex();
	if (!s_state_mutex) {
		return ESP_ERR_NO_MEM;
	}

	if (xSemaphoreTake(s_state_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
		if (!s_recording_active || s_active_log_path[0] == '\0') {
			xSemaphoreGive(s_state_mutex);
			return ESP_ERR_INVALID_STATE;
		}

		strncpy(path, s_active_log_path, sizeof(path) - 1);
		path[sizeof(path) - 1] = '\0';
		xSemaphoreGive(s_state_mutex);
	} else {
		return ESP_ERR_TIMEOUT;
	}

	FILE *f = fopen(path, "a");
	if (!f) {
		ESP_LOGE(TAG, "Failed to open log file %s", path);
		sdcard_unmount();
		return ESP_FAIL;
	}

	fprintf(f,
	        "%llu,%.2f,%d,%.2f",
	        (unsigned long long)uptime_ms,
	        wind_speed_ms,
	        relative_angle_deg,
	        compass_angle_deg);
	for (size_t i = 0; i < pressure_count; i++) {
		fprintf(f, ",%.2f", pressures[i]);
	}
	fprintf(f,
	        ",%ld,%ld,%ld,%ld,%ld,%ld",
	        (long)accel_x_raw,
	        (long)accel_y_raw,
	        (long)accel_z_raw,
	        (long)gyro_x_raw,
	        (long)gyro_y_raw,
	        (long)gyro_z_raw);
	fputc('\n', f);
	if (fflush(f) != 0) {
		ESP_LOGE(TAG, "fflush failed, unmounting SD");
		fclose(f);
		sdcard_unmount();
		return ESP_FAIL;
	}
	fclose(f);
	return ESP_OK;
}
