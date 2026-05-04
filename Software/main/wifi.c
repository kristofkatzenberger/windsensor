/*
 * Windsensor Firmware 
 * Author: Kristof Katzenberger
 * File: wifi.c
 */

#include <dirent.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <stdint.h>
#include <sys/stat.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_mac.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "mdns.h"
#include "sdcard.h"
#include "wifi.h"

static const char *TAG = "WIFI";

static httpd_handle_t s_server = NULL;
static esp_netif_t *s_ap_netif = NULL;
static bool s_wifi_started = false;
static bool s_mdns_started = false;
static bool s_wifi_event_handler_registered = false;

#define SDCARD_MOUNT_POINT "/sdcard"
#define MAX_QUERY_LENGTH   512
#define MAX_PATH_LENGTH    384
#define FILE_BUFFER_SIZE   1024
#define HTTP_SERVER_STACK_SIZE 8192

// Wi-Fi credentials
#define WIFI_SSID      "WindSensor"
#define WIFI_PASS      "12345678"
#define WIFI_CHANNEL   1
#define MAX_STA_CONN   4

// Sensor data accessor
extern void get_wind_data(int *idx, float *diff);
extern void get_compass_heading_deg(float *heading_deg);

// Embedded web assets (main/web/*)
extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[] asm("_binary_index_html_end");
extern const uint8_t main_css_start[] asm("_binary_main_css_start");
extern const uint8_t main_css_end[] asm("_binary_main_css_end");
extern const uint8_t main_js_start[] asm("_binary_main_js_start");
extern const uint8_t main_js_end[] asm("_binary_main_js_end");
extern const uint8_t files_html_start[] asm("_binary_files_html_start");
extern const uint8_t files_html_end[] asm("_binary_files_html_end");
extern const uint8_t files_css_start[] asm("_binary_files_css_start");
extern const uint8_t files_css_end[] asm("_binary_files_css_end");
extern const uint8_t files_js_start[] asm("_binary_files_js_start");
extern const uint8_t files_js_end[] asm("_binary_files_js_end");
extern const uint8_t fh_technikum_wien_logo_png_start[] asm("_binary_fh_technikum_wien_logo_png_start");
extern const uint8_t fh_technikum_wien_logo_png_end[] asm("_binary_fh_technikum_wien_logo_png_end");

typedef struct {
    const uint8_t *start;
    const uint8_t *end;
    const char *content_type;
} embedded_asset_t;

static int relative_angle_deg_from_sensor_idx(int sensor_idx)
{
    switch (sensor_idx) {
        case 6:
            return 0;
        case 5:
            return 45;
        case 4:
            return 90;
        case 3:
            return 135;
        case 2:
            return 180;
        case 0:
            return 225;
        case 8:
            return 270;
        case 7:
            return 315;
        default:
            return -1;
    }
}

static float pressure_to_wind_speed_kmh(float pressure_diff_pa)
{
    if (pressure_diff_pa <= 0.0f) {
        return 0.0f;
    }

    return (pressure_diff_pa * 0.55f) + 10.0f;
}

static float pressure_to_wind_speed_ms(float pressure_diff_pa)
{
    return pressure_to_wind_speed_kmh(pressure_diff_pa) / 3.6f;
}

static esp_err_t send_json_error(httpd_req_t *req, const char *status, const char *message)
{
    char response[192];

    snprintf(response, sizeof(response), "{\"error\":\"%s\"}", message);
    httpd_resp_set_status(req, status);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_sendstr(req, response);
}

static esp_err_t send_chunk(httpd_req_t *req, const char *chunk)
{
    return httpd_resp_sendstr_chunk(req, chunk);
}

static esp_err_t send_embedded_asset(httpd_req_t *req, const embedded_asset_t *asset)
{
    size_t length;

    if (asset == NULL || asset->start == NULL || asset->end == NULL || asset->content_type == NULL || asset->end < asset->start) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Invalid asset");
    }

    length = (size_t)(asset->end - asset->start);

    httpd_resp_set_type(req, asset->content_type);
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, (const char *)asset->start, length);
}

static esp_err_t send_json_escaped_chunk(httpd_req_t *req, const char *value)
{
    char raw_buffer[64];
    size_t raw_len = 0;

    for (const unsigned char *cursor = (const unsigned char *)value; *cursor != '\0'; cursor++) {
        const char *escaped = NULL;
        char control_escape[8];

        switch (*cursor) {
            case '"':
                escaped = "\\\"";
                break;
            case '\\':
                escaped = "\\\\";
                break;
            case '\b':
                escaped = "\\b";
                break;
            case '\f':
                escaped = "\\f";
                break;
            case '\n':
                escaped = "\\n";
                break;
            case '\r':
                escaped = "\\r";
                break;
            case '\t':
                escaped = "\\t";
                break;
            default:
                if (*cursor < 0x20) {
                    snprintf(control_escape, sizeof(control_escape), "\\u%04x", *cursor);
                    escaped = control_escape;
                }
                break;
        }

        if (escaped != NULL) {
            if (raw_len > 0) {
                raw_buffer[raw_len] = '\0';
                if (send_chunk(req, raw_buffer) != ESP_OK) {
                    return ESP_FAIL;
                }
                raw_len = 0;
            }

            if (send_chunk(req, escaped) != ESP_OK) {
                return ESP_FAIL;
            }
            continue;
        }

        raw_buffer[raw_len++] = (char)*cursor;
        if (raw_len == sizeof(raw_buffer) - 1) {
            raw_buffer[raw_len] = '\0';
            if (send_chunk(req, raw_buffer) != ESP_OK) {
                return ESP_FAIL;
            }
            raw_len = 0;
        }
    }

    if (raw_len > 0) {
        raw_buffer[raw_len] = '\0';
        if (send_chunk(req, raw_buffer) != ESP_OK) {
            return ESP_FAIL;
        }
    }

    return ESP_OK;
}

static esp_err_t ensure_sdcard_available(void)
{
    esp_err_t ret = sdcard_init();

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SD card unavailable: %s", esp_err_to_name(ret));
    }

    return ret;
}

static int hex_value(char c)
{
    if (c >= '0' && c <= '9') {
        return c - '0';
    }
    if (c >= 'a' && c <= 'f') {
        return c - 'a' + 10;
    }
    if (c >= 'A' && c <= 'F') {
        return c - 'A' + 10;
    }
    return -1;
}

static bool url_decode(const char *source, char *destination, size_t destination_size)
{
    size_t dest_index = 0;

    for (size_t index = 0; source[index] != '\0'; index++) {
        char decoded_char = source[index];

        if (decoded_char == '%') {
            int high;
            int low;

            if (source[index + 1] == '\0' || source[index + 2] == '\0') {
                return false;
            }

            high = hex_value(source[index + 1]);
            low = hex_value(source[index + 2]);
            if (high < 0 || low < 0) {
                return false;
            }

            decoded_char = (char)((high << 4) | low);
            index += 2;
        } else if (decoded_char == '+') {
            decoded_char = ' ';
        }

        if (dest_index + 1 >= destination_size) {
            return false;
        }

        destination[dest_index++] = decoded_char;
    }

    destination[dest_index] = '\0';
    return true;
}

static bool normalize_relative_path(const char *input, char *output, size_t output_size)
{
    char normalized_input[MAX_PATH_LENGTH];
    size_t input_length = 0;
    size_t output_length = 0;
    const char *cursor;

    if (output_size < 2) {
        return false;
    }

    if (input == NULL || input[0] == '\0' || strcmp(input, "/") == 0) {
        strncpy(output, "/", output_size);
        return true;
    }

    if (input[0] != '/') {
        normalized_input[input_length++] = '/';
    }

    for (size_t index = 0; input[index] != '\0'; index++) {
        if (input[index] == '\\') {
            return false;
        }
        if (input_length + 1 >= sizeof(normalized_input)) {
            return false;
        }
        normalized_input[input_length++] = input[index];
    }
    normalized_input[input_length] = '\0';

    output[output_length++] = '/';
    output[output_length] = '\0';
    cursor = normalized_input;

    while (*cursor != '\0') {
        char segment[MAX_PATH_LENGTH];
        size_t segment_length = 0;

        while (*cursor == '/') {
            cursor++;
        }

        if (*cursor == '\0') {
            break;
        }

        while (*cursor != '\0' && *cursor != '/') {
            if (segment_length + 1 >= sizeof(segment)) {
                return false;
            }
            segment[segment_length++] = *cursor;
            cursor++;
        }
        segment[segment_length] = '\0';

        if (strcmp(segment, ".") == 0) {
            continue;
        }
        if (strcmp(segment, "..") == 0) {
            return false;
        }

        if (output_length > 1) {
            if (output_length + 1 >= output_size) {
                return false;
            }
            output[output_length++] = '/';
        }

        if (output_length + segment_length >= output_size) {
            return false;
        }

        memcpy(output + output_length, segment, segment_length);
        output_length += segment_length;
        output[output_length] = '\0';
    }

    if (output_length == 0) {
        strncpy(output, "/", output_size);
    }

    return true;
}

static void get_parent_path(const char *path, char *parent_path, size_t parent_path_size)
{
    char *last_slash;

    if (parent_path_size < 2) {
        return;
    }

    if (path == NULL || strcmp(path, "/") == 0) {
        strncpy(parent_path, "/", parent_path_size);
        return;
    }

    strncpy(parent_path, path, parent_path_size - 1);
    parent_path[parent_path_size - 1] = '\0';

    last_slash = strrchr(parent_path, '/');
    if (last_slash == NULL || last_slash == parent_path) {
        strncpy(parent_path, "/", parent_path_size);
        return;
    }

    *last_slash = '\0';
}

static esp_err_t get_relative_path_from_request(httpd_req_t *req, const char *key, char *relative_path, size_t relative_path_size)
{
    size_t query_length = httpd_req_get_url_query_len(req);
    char decoded_path[MAX_PATH_LENGTH];
    char *encoded_path = NULL;
    char *query = NULL;
    esp_err_t ret = ESP_OK;

    strncpy(relative_path, "/", relative_path_size);

    if (query_length == 0) {
        return ESP_OK;
    }
    if (query_length >= MAX_QUERY_LENGTH) {
        return ESP_ERR_INVALID_SIZE;
    }

    query = calloc(query_length + 1, sizeof(char));
    encoded_path = calloc(query_length + 1, sizeof(char));
    if (query == NULL || encoded_path == NULL) {
        ret = ESP_ERR_NO_MEM;
        goto cleanup;
    }

    if (httpd_req_get_url_query_str(req, query, query_length + 1) != ESP_OK) {
        ret = ESP_FAIL;
        goto cleanup;
    }

    if (httpd_query_key_value(query, key, encoded_path, query_length + 1) != ESP_OK) {
        ret = ESP_OK;
        goto cleanup;
    }

    if (!url_decode(encoded_path, decoded_path, sizeof(decoded_path))) {
        ret = ESP_ERR_INVALID_ARG;
        goto cleanup;
    }
    if (!normalize_relative_path(decoded_path, relative_path, relative_path_size)) {
        ret = ESP_ERR_INVALID_ARG;
        goto cleanup;
    }

cleanup:
    free(encoded_path);
    free(query);
    return ret;
}

static esp_err_t build_sdcard_path(const char *relative_path, char *absolute_path, size_t absolute_path_size)
{
    int written;

    if (strcmp(relative_path, "/") == 0) {
        written = snprintf(absolute_path, absolute_path_size, "%s", SDCARD_MOUNT_POINT);
    } else {
        written = snprintf(absolute_path, absolute_path_size, "%s%s", SDCARD_MOUNT_POINT, relative_path);
    }

    if (written < 0 || (size_t)written >= absolute_path_size) {
        return ESP_ERR_INVALID_SIZE;
    }

    return ESP_OK;
}

static const char *get_filename_from_path(const char *path)
{
    const char *last_slash = strrchr(path, '/');

    return last_slash == NULL ? path : last_slash + 1;
}

static bool should_hide_entry(const char *name)
{
    if (name == NULL || name[0] == '\0') {
        return true;
    }

    if (name[0] == '.') {
        return true;
    }

    if (strncasecmp(name, "FSEVEN~", 7) == 0 ||
        strncasecmp(name, "SPOTLI~", 7) == 0 ||
        strncasecmp(name, "TRASHE~", 7) == 0) {
        return true;
    }

    if (name[0] == '_' && strchr(name, '~') != NULL) {
        return true;
    }

    return false;
}

static esp_err_t ensure_network_stack_ready(void)
{
    esp_err_t ret = esp_netif_init();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "esp_netif_init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_event_loop_create_default();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "esp_event_loop_create_default failed: %s", esp_err_to_name(ret));
        return ret;
    }

    if (s_ap_netif == NULL) {
        s_ap_netif = esp_netif_create_default_wifi_ap();
        if (s_ap_netif == NULL) {
            ESP_LOGE(TAG, "esp_netif_create_default_wifi_ap failed");
            return ESP_FAIL;
        }
    }

    return ESP_OK;
}

static void stop_webserver_internal(void)
{
    if (s_server != NULL) {
        ESP_LOGI(TAG, "Stopping webserver");
        httpd_stop(s_server);
        s_server = NULL;
    }
}

// Wi-Fi event handler
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                    int32_t event_id, void* event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" join, AID=%d",
                 MAC2STR(event->mac), event->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" leave, AID=%d",
                 MAC2STR(event->mac), event->aid);
    }
}

// SoftAP setup
void wifi_init_softap(void)
{
    esp_err_t ret;

    if (s_wifi_started) {
        ESP_LOGI(TAG, "Wi-Fi SoftAP already running");
        return;
    }

    ret = ensure_network_stack_ready();
    if (ret != ESP_OK) {
        return;
    }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ret = esp_wifi_init(&cfg);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "esp_wifi_init failed: %s", esp_err_to_name(ret));
        return;
    }

    if (!s_wifi_event_handler_registered) {
        ret = esp_event_handler_instance_register(WIFI_EVENT,
                                                  ESP_EVENT_ANY_ID,
                                                  &wifi_event_handler,
                                                  NULL,
                                                  NULL);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "event handler register failed: %s", esp_err_to_name(ret));
            return;
        }
        s_wifi_event_handler_registered = true;
    }

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = WIFI_SSID,
            .ssid_len = strlen(WIFI_SSID),
            .channel = WIFI_CHANNEL,
            .password = WIFI_PASS,
            .max_connection = MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK,
            .pmf_cfg = {
                .required = false,
            },
        },
    };
    if (strlen(WIFI_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ret = esp_wifi_set_mode(WIFI_MODE_AP);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_set_mode failed: %s", esp_err_to_name(ret));
        return;
    }

    ret = esp_wifi_set_config(WIFI_IF_AP, &wifi_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_set_config failed: %s", esp_err_to_name(ret));
        return;
    }

    ret = esp_wifi_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_start failed: %s", esp_err_to_name(ret));
        return;
    }

    s_wifi_started = true;

    ESP_LOGI(TAG, "wifi_init_softap finished. SSID:%s password:%s channel:%d",
             WIFI_SSID, WIFI_PASS, WIFI_CHANNEL);
             
    // mdns setup
    if (!s_mdns_started) {
        ret = mdns_init();
        if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
            ESP_LOGE(TAG, "mdns_init failed: %s", esp_err_to_name(ret));
            return;
        }

        ret = mdns_hostname_set("windsensor");
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "mdns_hostname_set failed: %s", esp_err_to_name(ret));
            return;
        }

        ret = mdns_instance_name_set("Wind Sensor Web Interface");
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "mdns_instance_name_set failed: %s", esp_err_to_name(ret));
            return;
        }

        // Service type _http._tcp
        ret = mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0);
        if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
            ESP_LOGE(TAG, "mdns_service_add failed: %s", esp_err_to_name(ret));
            return;
        }

        s_mdns_started = true;
        ESP_LOGI(TAG, "mDNS initialized: windsensor.local");
    }
}

// Web server handlers
// Data response handler
static esp_err_t data_get_handler(httpd_req_t *req)
{
    int idx;
    float diff;
    float compass_heading_deg = 0.0f;
    int relative_angle;
    float wind_kmh;
    float wind_ms;
    bool has_wind;
    get_wind_data(&idx, &diff);
    get_compass_heading_deg(&compass_heading_deg);

    char json_str[256];
    char sensor_info[64];

    if (idx == -2) {
        snprintf(sensor_info, sizeof(sensor_info), "No Wind Detected");
    } else {
        snprintf(sensor_info, sizeof(sensor_info), "%d", idx);
    }

    relative_angle = relative_angle_deg_from_sensor_idx(idx);
    wind_kmh = pressure_to_wind_speed_kmh(diff);
    wind_ms = pressure_to_wind_speed_ms(diff);
    has_wind = idx >= 0;

    snprintf(json_str,
             sizeof(json_str),
             "{\"idx\":\"%s\",\"diff\":%.2f,\"sensorIdx\":%d,\"pressureDiffPa\":%.2f,\"windSpeedKmh\":%.2f,\"windSpeedMs\":%.2f,\"relativeAngleDeg\":%d,\"compassHeadingDeg\":%.1f,\"hasWind\":%s}",
             sensor_info,
             diff,
             idx,
             diff,
             wind_kmh,
             wind_ms,
             relative_angle,
             compass_heading_deg,
             has_wind ? "true" : "false");

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_send(req, json_str, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// Data endpoint uri
static const httpd_uri_t data_uri = {
    .uri       = "/data",
    .method    = HTTP_GET,
    .handler   = data_get_handler,
    .user_ctx  = NULL
};

static esp_err_t files_list_get_handler(httpd_req_t *req)
{
    char relative_path[MAX_PATH_LENGTH];
    char absolute_path[MAX_PATH_LENGTH];
    char parent_path[MAX_PATH_LENGTH];
    struct stat directory_info;
    DIR *directory = NULL;
    esp_err_t ret;
    bool first_entry = true;

    ret = ensure_sdcard_available();
    if (ret != ESP_OK) {
        return send_json_error(req, "503 Service Unavailable", "SD card is not available");
    }

    ret = get_relative_path_from_request(req, "path", relative_path, sizeof(relative_path));
    if (ret != ESP_OK) {
        return send_json_error(req, "400 Bad Request", "Invalid path");
    }

    ret = build_sdcard_path(relative_path, absolute_path, sizeof(absolute_path));
    if (ret != ESP_OK) {
        return send_json_error(req, "400 Bad Request", "Path is too long");
    }

    if (stat(absolute_path, &directory_info) != 0 || !S_ISDIR(directory_info.st_mode)) {
        return send_json_error(req, "404 Not Found", "Folder not found");
    }

    directory = opendir(absolute_path);
    if (directory == NULL) {
        ESP_LOGE(TAG, "Failed to open directory %s", absolute_path);
        return send_json_error(req, "500 Internal Server Error", "Could not open folder");
    }

    get_parent_path(relative_path, parent_path, sizeof(parent_path));

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");

    ret = send_chunk(req, "{\"path\":\"");
    if (ret != ESP_OK) {
        goto cleanup;
    }
    ret = send_json_escaped_chunk(req, relative_path);
    if (ret != ESP_OK) {
        goto cleanup;
    }
    ret = send_chunk(req, "\",\"parentPath\":\"");
    if (ret != ESP_OK) {
        goto cleanup;
    }
    ret = send_json_escaped_chunk(req, parent_path);
    if (ret != ESP_OK) {
        goto cleanup;
    }
    ret = send_chunk(req, "\",\"entries\":[");
    if (ret != ESP_OK) {
        goto cleanup;
    }

    for (struct dirent *entry = readdir(directory); entry != NULL; entry = readdir(directory)) {
        char child_relative_path[MAX_PATH_LENGTH];
        char child_absolute_path[MAX_PATH_LENGTH];
        struct stat entry_info;
        bool is_directory;
        long long file_size;

        if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0 || should_hide_entry(entry->d_name)) {
            continue;
        }

        if (strcmp(relative_path, "/") == 0) {
            if (snprintf(child_relative_path, sizeof(child_relative_path), "/%s", entry->d_name) >= (int)sizeof(child_relative_path)) {
                ESP_LOGW(TAG, "Skipping long path entry %s", entry->d_name);
                continue;
            }
        } else if (snprintf(child_relative_path, sizeof(child_relative_path), "%s/%s", relative_path, entry->d_name) >= (int)sizeof(child_relative_path)) {
            ESP_LOGW(TAG, "Skipping long path entry %s", entry->d_name);
            continue;
        }

        ret = build_sdcard_path(child_relative_path, child_absolute_path, sizeof(child_absolute_path));
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Skipping path too long %s", child_relative_path);
            continue;
        }
        if (stat(child_absolute_path, &entry_info) != 0) {
            ESP_LOGW(TAG, "Skipping unreadable entry %s", child_absolute_path);
            continue;
        }

        is_directory = S_ISDIR(entry_info.st_mode);
        file_size = is_directory ? 0LL : (long long)entry_info.st_size;

        if (!first_entry) {
            ret = send_chunk(req, ",");
            if (ret != ESP_OK) {
                goto cleanup;
            }
        }
        first_entry = false;

        ret = send_chunk(req, "{\"name\":\"");
        if (ret != ESP_OK) {
            goto cleanup;
        }
        ret = send_json_escaped_chunk(req, entry->d_name);
        if (ret != ESP_OK) {
            goto cleanup;
        }
        ret = send_chunk(req, "\",\"path\":\"");
        if (ret != ESP_OK) {
            goto cleanup;
        }
        ret = send_json_escaped_chunk(req, child_relative_path);
        if (ret != ESP_OK) {
            goto cleanup;
        }
        ret = send_chunk(req, is_directory ? "\",\"type\":\"dir\",\"size\":" : "\",\"type\":\"file\",\"size\":");
        if (ret != ESP_OK) {
            goto cleanup;
        }

        {
            char size_buffer[32];

            snprintf(size_buffer, sizeof(size_buffer), "%lld}", file_size);
            ret = send_chunk(req, size_buffer);
            if (ret != ESP_OK) {
                goto cleanup;
            }
        }
    }

    ret = send_chunk(req, "]}");

cleanup:
    closedir(directory);
    if (ret == ESP_OK) {
        ret = httpd_resp_send_chunk(req, NULL, 0);
    }
    return ret;
}

static const httpd_uri_t files_list_uri = {
    .uri       = "/api/files",
    .method    = HTTP_GET,
    .handler   = files_list_get_handler,
    .user_ctx  = NULL
};

static esp_err_t file_download_get_handler(httpd_req_t *req)
{
    char relative_path[MAX_PATH_LENGTH];
    char absolute_path[MAX_PATH_LENGTH];
    char disposition[160];
    struct stat file_info;
    FILE *file = NULL;
    esp_err_t ret;

    ret = ensure_sdcard_available();
    if (ret != ESP_OK) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        return httpd_resp_sendstr(req, "SD card is not available");
    }

    ret = get_relative_path_from_request(req, "path", relative_path, sizeof(relative_path));
    if (ret != ESP_OK || strcmp(relative_path, "/") == 0) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid file path");
    }

    ret = build_sdcard_path(relative_path, absolute_path, sizeof(absolute_path));
    if (ret != ESP_OK) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Path is too long");
    }

    if (stat(absolute_path, &file_info) != 0 || !S_ISREG(file_info.st_mode)) {
        return httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "File not found");
    }

    file = fopen(absolute_path, "rb");
    if (file == NULL) {
        ESP_LOGE(TAG, "Failed to open file %s", absolute_path);
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Could not open file");
    }

    snprintf(disposition, sizeof(disposition), "attachment; filename=\"%s\"", get_filename_from_path(absolute_path));
    httpd_resp_set_type(req, "application/octet-stream");
    httpd_resp_set_hdr(req, "Content-Disposition", disposition);
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");

    while (!feof(file)) {
        char buffer[FILE_BUFFER_SIZE];
        size_t bytes_read = fread(buffer, 1, sizeof(buffer), file);

        if (bytes_read > 0 && httpd_resp_send_chunk(req, buffer, bytes_read) != ESP_OK) {
            fclose(file);
            return ESP_FAIL;
        }
        if (ferror(file)) {
            fclose(file);
            return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Error reading file");
        }
    }

    fclose(file);
    return httpd_resp_send_chunk(req, NULL, 0);
}

static const httpd_uri_t file_download_uri = {
    .uri       = "/download",
    .method    = HTTP_GET,
    .handler   = file_download_get_handler,
    .user_ctx  = NULL
};

static esp_err_t file_delete_handler(httpd_req_t *req)
{
    char relative_path[MAX_PATH_LENGTH];
    char absolute_path[MAX_PATH_LENGTH];
    struct stat file_info;
    esp_err_t ret;

    ret = ensure_sdcard_available();
    if (ret != ESP_OK) {
        return send_json_error(req, "503 Service Unavailable", "SD card is not available");
    }

    ret = get_relative_path_from_request(req, "path", relative_path, sizeof(relative_path));
    if (ret != ESP_OK || strcmp(relative_path, "/") == 0) {
        return send_json_error(req, "400 Bad Request", "Invalid file path");
    }

    ret = build_sdcard_path(relative_path, absolute_path, sizeof(absolute_path));
    if (ret != ESP_OK) {
        return send_json_error(req, "400 Bad Request", "Path is too long");
    }

    if (stat(absolute_path, &file_info) != 0 || !S_ISREG(file_info.st_mode)) {
        return send_json_error(req, "404 Not Found", "File not found");
    }

    if (remove(absolute_path) != 0) {
        ESP_LOGE(TAG, "Failed to delete file %s", absolute_path);
        return send_json_error(req, "500 Internal Server Error", "Could not delete file");
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_sendstr(req, "{\"ok\":true}");
}

static const httpd_uri_t file_delete_uri = {
    .uri       = "/api/file",
    .method    = HTTP_DELETE,
    .handler   = file_delete_handler,
    .user_ctx  = NULL
};

static const embedded_asset_t root_html_asset = {
    .start = index_html_start,
    .end = index_html_end,
    .content_type = "text/html"
};

static const embedded_asset_t files_html_asset = {
    .start = files_html_start,
    .end = files_html_end,
    .content_type = "text/html"
};

static const embedded_asset_t main_css_asset = {
    .start = main_css_start,
    .end = main_css_end,
    .content_type = "text/css"
};

static const embedded_asset_t main_js_asset = {
    .start = main_js_start,
    .end = main_js_end,
    .content_type = "application/javascript"
};

static const embedded_asset_t files_css_asset = {
    .start = files_css_start,
    .end = files_css_end,
    .content_type = "text/css"
};

static const embedded_asset_t files_js_asset = {
    .start = files_js_start,
    .end = files_js_end,
    .content_type = "application/javascript"
};

static const embedded_asset_t footer_logo_asset = {
    .start = fh_technikum_wien_logo_png_start,
    .end = fh_technikum_wien_logo_png_end,
    .content_type = "image/png"
};

static esp_err_t static_asset_get_handler(httpd_req_t *req)
{
    return send_embedded_asset(req, (const embedded_asset_t *)req->user_ctx);
}

static const httpd_uri_t files_page_uri = {
    .uri       = "/files",
    .method    = HTTP_GET,
    .handler   = static_asset_get_handler,
    .user_ctx  = (void *)&files_html_asset
};

static const httpd_uri_t main_css_uri = {
    .uri       = "/static/main.css",
    .method    = HTTP_GET,
    .handler   = static_asset_get_handler,
    .user_ctx  = (void *)&main_css_asset
};

static const httpd_uri_t main_js_uri = {
    .uri       = "/static/main.js",
    .method    = HTTP_GET,
    .handler   = static_asset_get_handler,
    .user_ctx  = (void *)&main_js_asset
};

static const httpd_uri_t files_css_uri = {
    .uri       = "/static/files.css",
    .method    = HTTP_GET,
    .handler   = static_asset_get_handler,
    .user_ctx  = (void *)&files_css_asset
};

static const httpd_uri_t files_js_uri = {
    .uri       = "/static/files.js",
    .method    = HTTP_GET,
    .handler   = static_asset_get_handler,
    .user_ctx  = (void *)&files_js_asset
};

static const httpd_uri_t footer_logo_uri = {
    .uri       = "/static/fh_technikum_wien_logo.png",
    .method    = HTTP_GET,
    .handler   = static_asset_get_handler,
    .user_ctx  = (void *)&footer_logo_asset
};

static esp_err_t favicon_get_handler(httpd_req_t *req)
{
    httpd_resp_set_status(req, "204 No Content");
    httpd_resp_set_type(req, "image/x-icon");
    httpd_resp_set_hdr(req, "Cache-Control", "max-age=86400");
    return httpd_resp_send(req, NULL, 0);
}

static const httpd_uri_t favicon_uri = {
    .uri       = "/favicon.ico",
    .method    = HTTP_GET,
    .handler   = favicon_get_handler,
    .user_ctx  = NULL
};

// Root endpoint uri
static const httpd_uri_t root = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = static_asset_get_handler,
    .user_ctx  = (void *)&root_html_asset
};

// Web server startup
void start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    if (s_server != NULL) {
        ESP_LOGI(TAG, "Webserver already running");
        return;
    }

    config.stack_size = HTTP_SERVER_STACK_SIZE;
    config.lru_purge_enable = true;
    // Keep room for static assets plus API routes.
    config.max_uri_handlers = 12;

    ESP_LOGI(TAG,
             "Starting server on port: '%d' with stack size %d, handlers %d",
             config.server_port,
             config.stack_size,
             config.max_uri_handlers);
    if (httpd_start(&s_server, &config) == ESP_OK) {
        ESP_LOGI(TAG, "Registering URI handlers");
        httpd_register_uri_handler(s_server, &root);
        httpd_register_uri_handler(s_server, &main_css_uri);
        httpd_register_uri_handler(s_server, &main_js_uri);
        httpd_register_uri_handler(s_server, &data_uri);
        httpd_register_uri_handler(s_server, &files_page_uri);
        httpd_register_uri_handler(s_server, &files_css_uri);
        httpd_register_uri_handler(s_server, &files_js_uri);
        httpd_register_uri_handler(s_server, &footer_logo_uri);
        httpd_register_uri_handler(s_server, &favicon_uri);
        httpd_register_uri_handler(s_server, &files_list_uri);
        httpd_register_uri_handler(s_server, &file_download_uri);
        httpd_register_uri_handler(s_server, &file_delete_uri);
    } else {
        s_server = NULL;
        ESP_LOGI(TAG, "Error starting server!");
    }
}

void wifi_suspend_services(void)
{
    stop_webserver_internal();

    if (s_mdns_started) {
        mdns_service_remove("_http", "_tcp");
        mdns_free();
        s_mdns_started = false;
    }

    if (s_wifi_started) {
        esp_err_t ret = esp_wifi_stop();
        if (ret != ESP_OK && ret != ESP_ERR_WIFI_NOT_STARTED && ret != ESP_ERR_WIFI_NOT_INIT) {
            ESP_LOGW(TAG, "esp_wifi_stop returned: %s", esp_err_to_name(ret));
        }

        ret = esp_wifi_deinit();
        if (ret != ESP_OK && ret != ESP_ERR_WIFI_NOT_INIT) {
            ESP_LOGW(TAG, "esp_wifi_deinit returned: %s", esp_err_to_name(ret));
        }

        s_wifi_started = false;
    }
}

void wifi_resume_services(void)
{
    wifi_init_softap();
    start_webserver();
}
