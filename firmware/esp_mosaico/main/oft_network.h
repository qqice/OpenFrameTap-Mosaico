#pragma once
#include <stdbool.h>
#include "esp_err.h"
typedef enum {OFT_NETWORK_OFF,OFT_NETWORK_JOINING,OFT_NETWORK_CONNECTED,OFT_NETWORK_FAULT} oft_network_state_t;
typedef struct {oft_network_state_t state;unsigned attempts;int last_reason;char ip[16];} oft_network_snapshot_t;
esp_err_t oft_network_start(void);
bool oft_network_join(const char *ssid,const char *password);
void oft_network_stop(void);
void oft_network_snapshot(oft_network_snapshot_t *out);
