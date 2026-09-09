#pragma once
#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>
#include "driver/i2c_types.h"
typedef struct {
    uint32_t flushes,unaligned_flushes;
    uint32_t render_last_us,render_max_us;
    bool alignment_selftest_passed;
} oft_display_stats_t;
esp_err_t oft_board_display_start(void);
oft_display_stats_t oft_board_display_stats(void);
i2c_master_bus_handle_t oft_board_i2c(void);
