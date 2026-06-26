#include "rg_system.h"
#include "rg_input.h"
#include "rg_settings.h"
#include "rg_ble_gamepad.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define NS_BLE_GAMEPAD "ble_gamepad"
#define SETTING_ENABLE "Enable"
#define SETTING_DEVICE_NAME "DeviceName"
#define SETTING_DEVICE_ADDR "DeviceAddress"
#define SETTING_ADDR_TYPE "AddressType"

#if defined(ESP_PLATFORM) && RG_ENABLE_BLE_GAMEPAD

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <esp_bt.h>
#include <esp_bt_main.h>
#include <esp_bit_defs.h>
#include <esp_gap_ble_api.h>
#include <esp_gatt_defs.h>
#include <esp_gattc_api.h>
#include <esp_hid_common.h>
#include <esp_hidh.h>
#include <esp_hidh_gattc.h>
#include <esp_log.h>
#include <nvs.h>
#include <nvs_flash.h>

#define BLE_GAMEPAD_SCAN_SECONDS 6
#define BLE_GAMEPAD_STARTUP_SCAN_SECONDS 5
#define BLE_GAMEPAD_SAVED_CONNECT_MS 1600
#define BLE_GAMEPAD_AUTO_CONNECT_MS 2200
#define BLE_GAMEPAD_PREFERRED_NAME "Q36 for Android"
#define AXIS_LOW_THRESHOLD 0x40
#define AXIS_HIGH_THRESHOLD 0xC0
#define HID_LOG_MAX_BYTES 20
#define BLE_GAMEPAD_VERBOSE_REPORTS 0

typedef struct
{
    esp_bd_addr_t bda;
    esp_ble_addr_type_t addr_type;
    char name[32];
    int rssi;
    uint16_t appearance;
} ble_gamepad_scan_result_t;

typedef struct
{
    bool initialized;
    bool enabled;
    rg_ble_gamepad_status_t status;
    uint32_t state;
    esp_hidh_dev_t *dev;
    SemaphoreHandle_t scan_done;
    SemaphoreHandle_t scan_lock;
    uint32_t scan_seconds;
    ble_gamepad_scan_result_t results[RG_BLE_GAMEPAD_MAX_DEVICES];
    size_t result_count;
    char active_name[32];
    char active_addr[18];
    char status_text[48];
    esp_ble_addr_type_t active_addr_type;
} ble_gamepad_state_t;

static ble_gamepad_state_t ble_pad;

static uint32_t map_buttons(uint32_t buttons);
static bool has_saved_device(void);
static bool wait_for_startup_connection(uint32_t timeout_ms);
static int find_startup_device_index(int count);
static int scan_for_devices(rg_ble_gamepad_device_t *devices, size_t max_devices, uint32_t scan_seconds);

static const char *status_to_text(rg_ble_gamepad_status_t status)
{
    switch (status)
    {
    case RG_BLE_GAMEPAD_STATUS_DISABLED: return "Off";
    case RG_BLE_GAMEPAD_STATUS_IDLE: return "Disconnected";
    case RG_BLE_GAMEPAD_STATUS_SCANNING: return "Scanning";
    case RG_BLE_GAMEPAD_STATUS_CONNECTING: return "Connecting";
    case RG_BLE_GAMEPAD_STATUS_CONNECTED:
        if (ble_pad.active_name[0])
        {
            snprintf(ble_pad.status_text, sizeof(ble_pad.status_text), "Connected: %.32s", ble_pad.active_name);
            return ble_pad.status_text;
        }
        return "Connected";
    case RG_BLE_GAMEPAD_STATUS_ERROR: return "Error";
    default: return "Unknown";
    }
}

static void bda_to_str(const uint8_t *bda, char *out, size_t out_len)
{
    snprintf(out, out_len, "%02x:%02x:%02x:%02x:%02x:%02x",
        bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
}

static bool str_to_bda(const char *str, uint8_t *bda)
{
    unsigned int value[6];
    if (!str || sscanf(str, "%02x:%02x:%02x:%02x:%02x:%02x",
        &value[0], &value[1], &value[2], &value[3], &value[4], &value[5]) != 6)
        return false;
    for (int i = 0; i < 6; ++i)
        bda[i] = value[i] & 0xFF;
    return true;
}

static uint32_t map_hat(uint8_t hat)
{
    switch (hat & 0x0F)
    {
    case 0: return RG_KEY_UP;
    case 1: return RG_KEY_UP | RG_KEY_RIGHT;
    case 2: return RG_KEY_RIGHT;
    case 3: return RG_KEY_RIGHT | RG_KEY_DOWN;
    case 4: return RG_KEY_DOWN;
    case 5: return RG_KEY_DOWN | RG_KEY_LEFT;
    case 6: return RG_KEY_LEFT;
    case 7: return RG_KEY_LEFT | RG_KEY_UP;
    default: return 0;
    }
}

static uint32_t map_axes(uint8_t x, uint8_t y)
{
    uint32_t state = 0;
    if (x < AXIS_LOW_THRESHOLD)
        state |= RG_KEY_LEFT;
    else if (x > AXIS_HIGH_THRESHOLD)
        state |= RG_KEY_RIGHT;
    if (y < AXIS_LOW_THRESHOLD)
        state |= RG_KEY_UP;
    else if (y > AXIS_HIGH_THRESHOLD)
        state |= RG_KEY_DOWN;
    return state;
}

static bool is_centered_axis(uint8_t value)
{
    return value >= 0x60 && value <= 0xA0;
}

static uint32_t parse_gamepad_candidate(const uint8_t *data, size_t len, size_t offset)
{
    if (!data || len < offset + 4)
        return 0;

    uint32_t state = 0;
    uint8_t x = data[offset + 0];
    uint8_t y = data[offset + 1];
    uint8_t hat = data[offset + 2] & 0x0F;

    if (!is_centered_axis(x) || !is_centered_axis(y))
        state |= map_axes(x, y);

    if (hat <= 8)
        state |= map_hat(hat);

    uint32_t buttons = data[offset + 3];
    if (len > offset + 4)
        buttons |= ((uint32_t)data[offset + 4]) << 8;
    if (len > offset + 5)
        buttons |= ((uint32_t)data[offset + 5]) << 16;
    state |= map_buttons(buttons);

    return state;
}

static uint32_t map_buttons(uint32_t buttons)
{
    uint32_t state = 0;
    // Android/HID gamepads commonly report buttons as A, B, X, Y, L1, R1, L2, R2, Select, Start, Home.
    if (buttons & BIT(0)) state |= RG_KEY_A;
    if (buttons & BIT(1)) state |= RG_KEY_B;
    if (buttons & BIT(2)) state |= RG_KEY_X;
    if (buttons & BIT(3)) state |= RG_KEY_Y;
    if (buttons & BIT(4)) state |= RG_KEY_L;
    if (buttons & BIT(5)) state |= RG_KEY_R;
    if (buttons & BIT(6)) state |= RG_KEY_L;
    if (buttons & BIT(7)) state |= RG_KEY_R;
    if (buttons & BIT(8)) state |= RG_KEY_SELECT;
    if (buttons & BIT(9)) state |= RG_KEY_START;
    if (buttons & BIT(10)) state |= RG_KEY_MENU;
    return state;
}

static bool is_q36_android_report(const uint8_t *data, size_t len)
{
    if (!data || len != 10)
        return false;

    return data[0] == 0x80 && data[1] == 0x80 && data[2] == 0x80 && data[3] == 0x80;
}

static uint32_t parse_q36_android_report(const uint8_t *data, size_t len)
{
    if (!is_q36_android_report(data, len))
        return 0;

    uint32_t state = 0;

    if (data[4] != 0xFF)
        state |= map_hat(data[4]);

    uint8_t buttons = data[5];
    if (buttons & BIT(0)) state |= RG_KEY_A;
    if (buttons & BIT(1)) state |= RG_KEY_B;
    if (buttons & BIT(3)) state |= RG_KEY_X;
    if (buttons & BIT(4)) state |= RG_KEY_Y;
    if (buttons & BIT(6)) state |= RG_KEY_L;
    if (buttons & BIT(7)) state |= RG_KEY_R;

    uint8_t system = data[6];
    if (system & BIT(0)) state |= RG_KEY_L;
    if (system & BIT(1)) state |= RG_KEY_R;
    if (system & BIT(2)) state |= RG_KEY_MENU;
    if (system & BIT(3)) state |= RG_KEY_OPTION;
    if (system & BIT(4)) state |= RG_KEY_MENU;

    return state;
}

static bool looks_like_gamepad_report(const uint8_t *data, size_t len)
{
    if (!data || len < 2)
        return false;

    bool all_zero = true;
    for (size_t i = 0; i < len; ++i)
        all_zero &= data[i] == 0;
    if (all_zero)
        return false;

    if (len >= 4)
    {
        uint8_t hat = data[2] & 0x0F;
        return hat <= 8;
    }

    return (data[0] & 0x0F) <= 8;
}

static uint32_t parse_gamepad_report(const uint8_t *data, size_t len)
{
    if (!data || len < 2)
        return 0;

    uint32_t state = 0;

    if (len >= 10)
    {
        if (is_q36_android_report(data, len))
            return parse_q36_android_report(data, len);

        for (size_t offset = 0; offset <= 4; ++offset)
        {
            uint32_t candidate = parse_gamepad_candidate(data, len, offset);
            if (!state && candidate)
                state = candidate;
        }
        return state;
    }

    if (!looks_like_gamepad_report(data, len))
        return 0;

    // Common BLE gamepad shape: X, Y, hat, buttons...
    if (len >= 4)
        return parse_gamepad_candidate(data, len, 0);

    if (len >= 2)
    {
        state |= map_hat(data[0] & 0x0F);
        state |= map_buttons(data[1]);
    }

    return state;
}

#if BLE_GAMEPAD_VERBOSE_REPORTS
static void log_hid_data(const uint8_t *data, size_t len)
{
    char hex[HID_LOG_MAX_BYTES * 3 + 1] = {0};
    size_t count = RG_MIN(len, (size_t)HID_LOG_MAX_BYTES);
    char *ptr = hex;

    for (size_t i = 0; i < count; ++i)
        ptr += snprintf(ptr, sizeof(hex) - (ptr - hex), "%02x%s", data[i], i + 1 < count ? " " : "");

    RG_LOGI("BLE HID data: %s%s", hex, len > count ? " ..." : "");
}
#endif

static uint32_t map_keyboard_usage(uint8_t usage)
{
    switch (usage)
    {
    case 0x1A: return RG_KEY_UP;     // w
    case 0x16: return RG_KEY_DOWN;   // s
    case 0x04: return RG_KEY_LEFT;   // a
    case 0x07: return RG_KEY_RIGHT;  // d
    case 0x28: return RG_KEY_A;      // Enter
    case 0x29: return RG_KEY_B;      // Escape
    case 0x2B: return RG_KEY_OPTION; // Tab
    case 0x2C: return RG_KEY_A;      // Space
    case 0x65: return RG_KEY_MENU;   // Application/Menu
    case 0x4A: return RG_KEY_MENU;   // Home
    case 0x76: return RG_KEY_MENU;   // Keyboard Menu
    case 0x4F: return RG_KEY_RIGHT;  // Right Arrow
    case 0x50: return RG_KEY_LEFT;   // Left Arrow
    case 0x51: return RG_KEY_DOWN;   // Down Arrow
    case 0x52: return RG_KEY_UP;     // Up Arrow
    default: return 0;
    }
}

static uint32_t parse_keyboard_report(const uint8_t *data, size_t len)
{
    if (!data)
        return 0;

    uint32_t state = 0;

    // BLE HID boot keyboard input report: modifiers, reserved, six key usages.
    if (len >= 8)
    {
        for (size_t i = 2; i < 8; ++i)
            state |= map_keyboard_usage(data[i]);
        return state;
    }

    for (size_t i = 0; i < len; ++i)
        state |= map_keyboard_usage(data[i]);

    return state;
}

static void make_device_label(const ble_gamepad_scan_result_t *result, char *out, size_t out_len)
{
    char addr[18];
    bda_to_str(result->bda, addr, sizeof(addr));
    if (result->name[0])
        snprintf(out, out_len, "%.24s  %ddBm", result->name, result->rssi);
    else
        snprintf(out, out_len, "%s  %ddBm", addr, result->rssi);
}

static void save_active_device(const uint8_t *bda, esp_ble_addr_type_t addr_type, const char *name)
{
    bda_to_str(bda, ble_pad.active_addr, sizeof(ble_pad.active_addr));
    snprintf(ble_pad.active_name, sizeof(ble_pad.active_name), "%s", name && name[0] ? name : ble_pad.active_addr);
    ble_pad.active_addr_type = addr_type;
    rg_settings_set_string(NS_BLE_GAMEPAD, SETTING_DEVICE_ADDR, ble_pad.active_addr);
    rg_settings_set_string(NS_BLE_GAMEPAD, SETTING_DEVICE_NAME, ble_pad.active_name);
    rg_settings_set_number(NS_BLE_GAMEPAD, SETTING_ADDR_TYPE, addr_type);
    rg_settings_commit();

    nvs_handle_t nvs;
    if (nvs_open(NS_BLE_GAMEPAD, NVS_READWRITE, &nvs) == ESP_OK)
    {
        nvs_set_str(nvs, SETTING_DEVICE_ADDR, ble_pad.active_addr);
        nvs_set_str(nvs, SETTING_DEVICE_NAME, ble_pad.active_name);
        nvs_set_u8(nvs, SETTING_ADDR_TYPE, addr_type);
        nvs_commit(nvs);
        nvs_close(nvs);
    }
}

static char *load_saved_string(const char *key)
{
    nvs_handle_t nvs;
    if (nvs_open(NS_BLE_GAMEPAD, NVS_READONLY, &nvs) == ESP_OK)
    {
        size_t len = 0;
        esp_err_t ret = nvs_get_str(nvs, key, NULL, &len);
        if (ret == ESP_OK && len)
        {
            char *value = malloc(len);
            if (value && nvs_get_str(nvs, key, value, &len) == ESP_OK)
            {
                nvs_close(nvs);
                return value;
            }
            free(value);
        }
        nvs_close(nvs);
    }

    return rg_settings_get_string(NS_BLE_GAMEPAD, key, NULL);
}

static esp_ble_addr_type_t load_saved_addr_type(void)
{
    nvs_handle_t nvs;
    uint8_t addr_type = BLE_ADDR_TYPE_PUBLIC;
    if (nvs_open(NS_BLE_GAMEPAD, NVS_READONLY, &nvs) == ESP_OK)
    {
        if (nvs_get_u8(nvs, SETTING_ADDR_TYPE, &addr_type) == ESP_OK)
        {
            nvs_close(nvs);
            return (esp_ble_addr_type_t)addr_type;
        }
        nvs_close(nvs);
    }

    return (esp_ble_addr_type_t)rg_settings_get_number(NS_BLE_GAMEPAD, SETTING_ADDR_TYPE, BLE_ADDR_TYPE_PUBLIC);
}

static bool load_saved_bda(esp_bd_addr_t bda)
{
    char *addr = load_saved_string(SETTING_DEVICE_ADDR);
    bool valid = addr && str_to_bda(addr, bda);
    free(addr);
    return valid;
}

static bool saved_device_matches(const ble_gamepad_scan_result_t *result)
{
    esp_bd_addr_t saved_bda;
    return result && load_saved_bda(saved_bda) &&
        memcmp(result->bda, saved_bda, sizeof(esp_bd_addr_t)) == 0;
}

static bool adv_has_hid_service(uint8_t *adv_data, uint8_t adv_len)
{
    uint8_t len = 0;
    uint8_t *data = esp_ble_resolve_adv_data_by_type(adv_data, adv_len, ESP_BLE_AD_TYPE_16SRV_CMPL, &len);
    if (!data)
        data = esp_ble_resolve_adv_data_by_type(adv_data, adv_len, ESP_BLE_AD_TYPE_16SRV_PART, &len);

    for (int i = 0; data && i + 1 < len; i += 2)
    {
        uint16_t uuid = data[i] | (data[i + 1] << 8);
        if (uuid == ESP_GATT_UUID_HID_SVC)
            return true;
    }
    return false;
}

static void add_scan_result(esp_ble_gap_cb_param_t *param)
{
    esp_ble_gap_cb_param_t *scan = param;
    const uint8_t *bda = scan->scan_rst.bda;

    for (size_t i = 0; i < ble_pad.result_count; ++i)
    {
        if (memcmp(ble_pad.results[i].bda, bda, sizeof(esp_bd_addr_t)) == 0)
            return;
    }
    if (ble_pad.result_count >= RG_BLE_GAMEPAD_MAX_DEVICES)
        return;

    ble_gamepad_scan_result_t *result = &ble_pad.results[ble_pad.result_count++];
    memcpy(result->bda, bda, sizeof(esp_bd_addr_t));
    result->addr_type = scan->scan_rst.ble_addr_type;
    result->rssi = scan->scan_rst.rssi;

    uint8_t name_len = 0;
    uint8_t *name = esp_ble_resolve_adv_data_by_type(scan->scan_rst.ble_adv,
        scan->scan_rst.adv_data_len + scan->scan_rst.scan_rsp_len,
        ESP_BLE_AD_TYPE_NAME_CMPL, &name_len);
    if (!name)
        name = esp_ble_resolve_adv_data_by_type(scan->scan_rst.ble_adv,
            scan->scan_rst.adv_data_len + scan->scan_rst.scan_rsp_len,
            ESP_BLE_AD_TYPE_NAME_SHORT, &name_len);
    if (name && name_len)
    {
        size_t copy_len = RG_MIN((size_t)name_len, sizeof(result->name) - 1);
        memcpy(result->name, name, copy_len);
        result->name[copy_len] = 0;
    }

    uint8_t appearance_len = 0;
    uint8_t *appearance = esp_ble_resolve_adv_data_by_type(scan->scan_rst.ble_adv,
        scan->scan_rst.adv_data_len + scan->scan_rst.scan_rsp_len,
        ESP_BLE_AD_TYPE_APPEARANCE, &appearance_len);
    if (appearance && appearance_len >= 2)
        result->appearance = appearance[0] | (appearance[1] << 8);

    char addr[18];
    bda_to_str(result->bda, addr, sizeof(addr));
    RG_LOGI("Found BLE HID device %s (%s), RSSI=%d", result->name[0] ? result->name : "unknown", addr, result->rssi);
}

static void ble_gap_callback(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event)
    {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
        esp_ble_gap_start_scanning(ble_pad.scan_seconds ? ble_pad.scan_seconds : BLE_GAMEPAD_SCAN_SECONDS);
        break;
    case ESP_GAP_BLE_SCAN_RESULT_EVT:
        if (param->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT)
        {
            if (adv_has_hid_service(param->scan_rst.ble_adv, param->scan_rst.adv_data_len + param->scan_rst.scan_rsp_len))
                add_scan_result(param);
        }
        else if (param->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_CMPL_EVT)
        {
            if (ble_pad.scan_done)
                xSemaphoreGive(ble_pad.scan_done);
        }
        break;
    case ESP_GAP_BLE_SEC_REQ_EVT:
        esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
        break;
    case ESP_GAP_BLE_AUTH_CMPL_EVT:
        if (!param->ble_security.auth_cmpl.success)
            RG_LOGW("BLE gamepad auth failed: 0x%x", param->ble_security.auth_cmpl.fail_reason);
        break;
    case ESP_GAP_BLE_NC_REQ_EVT:
        esp_ble_confirm_reply(param->ble_security.key_notif.bd_addr, true);
        break;
    default:
        break;
    }
}

static void hidh_callback(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    esp_hidh_event_data_t *param = (esp_hidh_event_data_t *)event_data;
    uint32_t new_state = ble_pad.state;

    switch ((esp_hidh_event_t)id)
    {
    case ESP_HIDH_OPEN_EVENT:
        if (param->open.status == ESP_OK)
        {
            ble_pad.dev = param->open.dev;
            ble_pad.status = RG_BLE_GAMEPAD_STATUS_CONNECTED;
            const uint8_t *bda = esp_hidh_dev_bda_get(param->open.dev);
            save_active_device(bda, ble_pad.active_addr_type, esp_hidh_dev_name_get(param->open.dev));
            RG_LOGI("BLE gamepad connected: %s", ble_pad.active_name);
        }
        else
        {
            ble_pad.status = ble_pad.enabled ? RG_BLE_GAMEPAD_STATUS_IDLE : RG_BLE_GAMEPAD_STATUS_DISABLED;
            RG_LOGE("BLE gamepad open failed: %s", esp_err_to_name(param->open.status));
        }
        break;
    case ESP_HIDH_INPUT_EVENT:
        if (param->input.usage == ESP_HID_USAGE_GAMEPAD ||
            param->input.usage == ESP_HID_USAGE_JOYSTICK)
        {
            new_state = parse_gamepad_report(param->input.data, param->input.length);
        }
        else if (param->input.usage == ESP_HID_USAGE_KEYBOARD)
        {
            new_state = parse_keyboard_report(param->input.data, param->input.length);
        }
        else if (param->input.usage == ESP_HID_USAGE_GENERIC)
        {
            RG_LOGW("Ignoring generic HID input report until its descriptor is mapped.");
        }
        if (new_state != ble_pad.state)
        {
            RG_LOGI("BLE HID input: usage=%s map=%u id=%u len=%u",
                esp_hid_usage_str(param->input.usage),
                param->input.map_index,
                param->input.report_id,
                param->input.length);
#if BLE_GAMEPAD_VERBOSE_REPORTS
            log_hid_data(param->input.data, param->input.length);
#endif
            ble_pad.state = new_state;
            RG_LOGI("BLE HID state: 0x%08lx", (unsigned long)ble_pad.state);
        }
        break;
    case ESP_HIDH_CLOSE_EVENT:
        ble_pad.state = 0;
        if (ble_pad.dev == param->close.dev)
            ble_pad.dev = NULL;
        ble_pad.status = ble_pad.enabled ? RG_BLE_GAMEPAD_STATUS_IDLE : RG_BLE_GAMEPAD_STATUS_DISABLED;
        esp_hidh_dev_free(param->close.dev);
        break;
    default:
        break;
    }
}

static bool init_stack(void)
{
    if (ble_pad.initialized)
        return true;

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        if (nvs_flash_erase() == ESP_OK)
            ret = nvs_flash_init();
    }
    if (ret != ESP_OK)
    {
        RG_LOGE("nvs_flash_init failed: %s", esp_err_to_name(ret));
        return false;
    }

    esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE)
    {
        RG_LOGE("esp_bt_controller_init failed: %s", esp_err_to_name(ret));
        return false;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE)
    {
        RG_LOGE("esp_bt_controller_enable failed: %s", esp_err_to_name(ret));
        return false;
    }

    esp_bluedroid_config_t bluedroid_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
    bluedroid_cfg.ssp_en = false;
    ret = esp_bluedroid_init_with_cfg(&bluedroid_cfg);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE)
    {
        RG_LOGE("esp_bluedroid_init failed: %s", esp_err_to_name(ret));
        return false;
    }

    ret = esp_bluedroid_enable();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE)
    {
        RG_LOGE("esp_bluedroid_enable failed: %s", esp_err_to_name(ret));
        return false;
    }

    esp_ble_gap_register_callback(ble_gap_callback);
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_BOND;
    esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE;
    uint8_t key_size = 16;
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(auth_req));
    esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(iocap));
    esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(key_size));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(init_key));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(rsp_key));
    esp_ble_gattc_register_callback(esp_hidh_gattc_event_handler);

    esp_hidh_config_t config = {
        .callback = hidh_callback,
        .event_stack_size = 4096,
        .callback_arg = NULL,
    };
    ret = esp_hidh_init(&config);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE)
    {
        RG_LOGE("esp_hidh_init failed: %s", esp_err_to_name(ret));
        return false;
    }

    ble_pad.scan_done = xSemaphoreCreateBinary();
    ble_pad.scan_lock = xSemaphoreCreateMutex();
    ble_pad.initialized = ble_pad.scan_done != NULL && ble_pad.scan_lock != NULL;
    return ble_pad.initialized;
}

static bool has_saved_device(void)
{
    char *addr = load_saved_string(SETTING_DEVICE_ADDR);
    bool found = addr && addr[0];
    free(addr);
    return found;
}

static bool open_saved_device(void)
{
    esp_bd_addr_t bda;
    if (!load_saved_bda(bda))
        return false;

    ble_pad.status = RG_BLE_GAMEPAD_STATUS_CONNECTING;
    ble_pad.active_addr_type = load_saved_addr_type();
    return esp_hidh_dev_open(bda, ESP_HID_TRANSPORT_BLE, ble_pad.active_addr_type) != NULL;
}

void rg_ble_gamepad_init(void)
{
    ble_pad.enabled = rg_settings_get_boolean(NS_BLE_GAMEPAD, SETTING_ENABLE, true);
    ble_pad.status = ble_pad.enabled ? RG_BLE_GAMEPAD_STATUS_IDLE : RG_BLE_GAMEPAD_STATUS_DISABLED;
    if (!ble_pad.enabled)
        return;

    if (!init_stack())
    {
        ble_pad.status = RG_BLE_GAMEPAD_STATUS_ERROR;
        return;
    }
}

void rg_ble_gamepad_deinit(void)
{
    ble_pad.state = 0;
    if (ble_pad.dev)
        esp_hidh_dev_close(ble_pad.dev);
}

uint32_t rg_ble_gamepad_read(void)
{
    return ble_pad.enabled ? ble_pad.state : 0;
}

bool rg_ble_gamepad_is_enabled(void)
{
    return ble_pad.enabled;
}

void rg_ble_gamepad_set_enabled(bool enabled)
{
    ble_pad.enabled = enabled;
    rg_settings_set_boolean(NS_BLE_GAMEPAD, SETTING_ENABLE, enabled);
    ble_pad.status = enabled ? RG_BLE_GAMEPAD_STATUS_IDLE : RG_BLE_GAMEPAD_STATUS_DISABLED;
    if (enabled)
        rg_ble_gamepad_init();
    else
    {
        ble_pad.state = 0;
        if (ble_pad.dev)
            esp_hidh_dev_close(ble_pad.dev);
    }
}

bool rg_ble_gamepad_is_available(void)
{
    return ble_pad.initialized;
}

rg_ble_gamepad_status_t rg_ble_gamepad_get_status(void)
{
    return ble_pad.status;
}

const char *rg_ble_gamepad_status_text(void)
{
    return status_to_text(ble_pad.status);
}

static bool wait_for_startup_connection(uint32_t timeout_ms)
{
    int64_t timeout = rg_system_timer() + timeout_ms * 1000;
    while (rg_system_timer() < timeout)
    {
        if (ble_pad.status == RG_BLE_GAMEPAD_STATUS_CONNECTED)
            return true;
        if (ble_pad.status != RG_BLE_GAMEPAD_STATUS_CONNECTING)
            return false;
        rg_task_delay(50);
    }
    return ble_pad.status == RG_BLE_GAMEPAD_STATUS_CONNECTED;
}

static int find_startup_device_index(int count)
{
    for (int i = 0; i < count; ++i)
    {
        if (saved_device_matches(&ble_pad.results[i]))
            return i;
    }

    for (int i = 0; i < count; ++i)
    {
        if (strcmp(ble_pad.results[i].name, BLE_GAMEPAD_PREFERRED_NAME) == 0)
            return i;
    }

    return -1;
}

bool rg_ble_gamepad_startup_connect(void)
{
    if (!ble_pad.enabled || !ble_pad.initialized)
        return false;
    if (ble_pad.status == RG_BLE_GAMEPAD_STATUS_CONNECTED)
        return true;
    if (ble_pad.dev)
        return true;

    if (has_saved_device() && open_saved_device() && wait_for_startup_connection(BLE_GAMEPAD_SAVED_CONNECT_MS))
        return true;

    rg_ble_gamepad_device_t devices[RG_BLE_GAMEPAD_MAX_DEVICES] = {0};
    int count = scan_for_devices(devices, RG_COUNT(devices), BLE_GAMEPAD_STARTUP_SCAN_SECONDS);
    int selected = find_startup_device_index(count);
    if (selected >= 0)
    {
        RG_LOGI("Auto-connecting preferred BLE HID device: %s", devices[selected].label);
        if (rg_ble_gamepad_connect_index(selected) && wait_for_startup_connection(BLE_GAMEPAD_AUTO_CONNECT_MS))
            return true;
    }
    else if (count == 1)
    {
        RG_LOGI("Auto-connecting BLE HID device: %s", devices[0].label);
        if (rg_ble_gamepad_connect_index(0) && wait_for_startup_connection(BLE_GAMEPAD_AUTO_CONNECT_MS))
            return true;
    }
    else if (count > 1)
    {
        RG_LOGI("Found %d BLE HID devices; use Pair controller to choose.", count);
    }

    if (ble_pad.status != RG_BLE_GAMEPAD_STATUS_CONNECTED)
        ble_pad.status = RG_BLE_GAMEPAD_STATUS_IDLE;
    return false;
}

static int scan_for_devices(rg_ble_gamepad_device_t *devices, size_t max_devices, uint32_t scan_seconds)
{
    if (!ble_pad.enabled)
        rg_ble_gamepad_set_enabled(true);
    if (!init_stack())
    {
        ble_pad.status = RG_BLE_GAMEPAD_STATUS_ERROR;
        return -1;
    }

    if (xSemaphoreTake(ble_pad.scan_lock, pdMS_TO_TICKS(50)) != pdTRUE)
        return -1;

    memset(ble_pad.results, 0, sizeof(ble_pad.results));
    ble_pad.result_count = 0;
    ble_pad.scan_seconds = scan_seconds ? scan_seconds : BLE_GAMEPAD_SCAN_SECONDS;
    xSemaphoreTake(ble_pad.scan_done, 0);
    ble_pad.status = RG_BLE_GAMEPAD_STATUS_SCANNING;

    esp_ble_scan_params_t params = {
        .scan_type = BLE_SCAN_TYPE_ACTIVE,
        .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
        .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
        .scan_interval = 0x50,
        .scan_window = 0x30,
        .scan_duplicate = BLE_SCAN_DUPLICATE_ENABLE,
    };

    if (esp_ble_gap_set_scan_params(&params) != ESP_OK ||
        xSemaphoreTake(ble_pad.scan_done, pdMS_TO_TICKS((ble_pad.scan_seconds + 2) * 1000)) != pdTRUE)
    {
        ble_pad.status = RG_BLE_GAMEPAD_STATUS_ERROR;
        xSemaphoreGive(ble_pad.scan_lock);
        return -1;
    }

    if (ble_pad.result_count == 0)
    {
        ble_pad.status = RG_BLE_GAMEPAD_STATUS_IDLE;
        xSemaphoreGive(ble_pad.scan_lock);
        return 0;
    }

    size_t count = RG_MIN(ble_pad.result_count, max_devices);
    for (size_t i = 0; devices && i < count; ++i)
    {
        make_device_label(&ble_pad.results[i], devices[i].label, sizeof(devices[i].label));
        devices[i].rssi = ble_pad.results[i].rssi;
    }

    ble_pad.status = RG_BLE_GAMEPAD_STATUS_IDLE;
    xSemaphoreGive(ble_pad.scan_lock);
    return (int)count;
}

int rg_ble_gamepad_scan(rg_ble_gamepad_device_t *devices, size_t max_devices)
{
    return scan_for_devices(devices, max_devices, BLE_GAMEPAD_SCAN_SECONDS);
}

bool rg_ble_gamepad_connect_index(size_t index)
{
    if (index >= ble_pad.result_count)
        return false;

    if (ble_pad.dev)
    {
        esp_hidh_dev_close(ble_pad.dev);
        ble_pad.dev = NULL;
    }

    ble_gamepad_scan_result_t *result = &ble_pad.results[index];
    ble_pad.status = RG_BLE_GAMEPAD_STATUS_CONNECTING;
    ble_pad.active_addr_type = result->addr_type;
    bda_to_str(result->bda, ble_pad.active_addr, sizeof(ble_pad.active_addr));
    snprintf(ble_pad.active_name, sizeof(ble_pad.active_name), "%s", result->name[0] ? result->name : ble_pad.active_addr);
    return esp_hidh_dev_open(result->bda, ESP_HID_TRANSPORT_BLE, result->addr_type) != NULL;
}

void rg_ble_gamepad_forget(void)
{
    rg_settings_delete(NS_BLE_GAMEPAD, SETTING_DEVICE_ADDR);
    rg_settings_delete(NS_BLE_GAMEPAD, SETTING_DEVICE_NAME);
    rg_settings_delete(NS_BLE_GAMEPAD, SETTING_ADDR_TYPE);
    rg_settings_commit();

    nvs_handle_t nvs;
    if (nvs_open(NS_BLE_GAMEPAD, NVS_READWRITE, &nvs) == ESP_OK)
    {
        nvs_erase_key(nvs, SETTING_DEVICE_ADDR);
        nvs_erase_key(nvs, SETTING_DEVICE_NAME);
        nvs_erase_key(nvs, SETTING_ADDR_TYPE);
        nvs_commit(nvs);
        nvs_close(nvs);
    }

    memset(ble_pad.active_addr, 0, sizeof(ble_pad.active_addr));
    memset(ble_pad.active_name, 0, sizeof(ble_pad.active_name));
    ble_pad.active_addr_type = BLE_ADDR_TYPE_PUBLIC;
    ble_pad.state = 0;
    if (ble_pad.dev)
        esp_hidh_dev_close(ble_pad.dev);
    ble_pad.status = ble_pad.enabled ? RG_BLE_GAMEPAD_STATUS_IDLE : RG_BLE_GAMEPAD_STATUS_DISABLED;
}

#else

static bool ble_gamepad_enabled = false;

void rg_ble_gamepad_init(void) {}
void rg_ble_gamepad_deinit(void) {}
uint32_t rg_ble_gamepad_read(void) { return 0; }
bool rg_ble_gamepad_is_enabled(void) { return ble_gamepad_enabled; }
void rg_ble_gamepad_set_enabled(bool enabled) { ble_gamepad_enabled = enabled; }
bool rg_ble_gamepad_is_available(void) { return false; }
rg_ble_gamepad_status_t rg_ble_gamepad_get_status(void) { return RG_BLE_GAMEPAD_STATUS_DISABLED; }
const char *rg_ble_gamepad_status_text(void) { return "Unavailable"; }
bool rg_ble_gamepad_startup_connect(void) { return false; }
int rg_ble_gamepad_scan(rg_ble_gamepad_device_t *devices, size_t max_devices) { return -1; }
bool rg_ble_gamepad_connect_index(size_t index) { return false; }
void rg_ble_gamepad_forget(void) {}

#endif
