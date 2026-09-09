#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#define OFT_BLE_CHOICES 4
typedef enum {OFT_BLE_OFF,OFT_BLE_SCANNING,OFT_BLE_CONNECTING,OFT_BLE_DISCOVERING,
    OFT_BLE_AUTHENTICATING,OFT_BLE_CONFIRM,OFT_BLE_CREDENTIALS,OFT_BLE_READY,
    OFT_BLE_IDLE,OFT_BLE_FAULT} oft_ble_state_t;
typedef struct {char name[32];uint8_t address[6],address_type;int8_t rssi;} oft_ble_choice_t;
typedef struct {
    oft_ble_state_t state;
    char detail[64];
    oft_ble_choice_t choices[OFT_BLE_CHOICES];
    unsigned choice_count,advertisements,dji_advertisements,fff0_advertisements,notifications,frames,writes,errors,disconnects;
    uint16_t mtu;
    unsigned matched_replies;
    uint8_t pairing_status,last_reply_flags;
    bool battery_valid;
    uint8_t battery;
    int64_t battery_updated_us; /* provenance: decoded 0D/02, offset20, uint8 */
} oft_ble_snapshot_t;
esp_err_t oft_ble_start(void);
void oft_ble_snapshot(oft_ble_snapshot_t *out);
bool oft_ble_select(unsigned index);
void oft_ble_disconnect(void);
void oft_ble_rescan(void);
