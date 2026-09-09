#pragma once
#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>
typedef struct {
    bool battery_valid,gauge_configured;unsigned percent,millivolts,design_mah,read_errors;
    int milliamps;int64_t battery_us;unsigned button_presses,button_queued;
} oft_peripherals_snapshot_t;
esp_err_t oft_peripherals_start(void);
void oft_peripherals_snapshot(oft_peripherals_snapshot_t *out);
