#pragma once
#include "esp_lcd_touch.h"
#include "lvgl.h"
#include <stdint.h>
#include <stdbool.h>
typedef struct {
    uint32_t samples,errors,max_gap_us,head_presses,head_releases;
    uint32_t shadow_presses,shadow_releases,shadow_early_stops,shadow_max_gap_us;
} oft_touch_stats_t;
esp_err_t oft_touch_start(esp_lcd_touch_handle_t touch,lv_display_t *display);
void oft_touch_head_region(int x1,int y1,int x2,int y2,bool enabled);
void oft_touch_shadow_start(int64_t end_us);
void oft_touch_stats(oft_touch_stats_t *out);
void oft_touch_selftests(void);
