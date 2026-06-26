#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum
{
    RG_BLE_GAMEPAD_STATUS_DISABLED,
    RG_BLE_GAMEPAD_STATUS_IDLE,
    RG_BLE_GAMEPAD_STATUS_SCANNING,
    RG_BLE_GAMEPAD_STATUS_CONNECTING,
    RG_BLE_GAMEPAD_STATUS_CONNECTED,
    RG_BLE_GAMEPAD_STATUS_ERROR,
} rg_ble_gamepad_status_t;

#define RG_BLE_GAMEPAD_MAX_DEVICES 8
#define RG_BLE_GAMEPAD_LABEL_LEN 48

typedef struct
{
    char label[RG_BLE_GAMEPAD_LABEL_LEN];
    int rssi;
} rg_ble_gamepad_device_t;

void rg_ble_gamepad_init(void);
void rg_ble_gamepad_deinit(void);
uint32_t rg_ble_gamepad_read(void);
bool rg_ble_gamepad_is_enabled(void);
void rg_ble_gamepad_set_enabled(bool enabled);
bool rg_ble_gamepad_is_available(void);
rg_ble_gamepad_status_t rg_ble_gamepad_get_status(void);
const char *rg_ble_gamepad_status_text(void);
bool rg_ble_gamepad_startup_connect(void);
int rg_ble_gamepad_scan(rg_ble_gamepad_device_t *devices, size_t max_devices);
bool rg_ble_gamepad_connect_index(size_t index);
void rg_ble_gamepad_forget(void);
