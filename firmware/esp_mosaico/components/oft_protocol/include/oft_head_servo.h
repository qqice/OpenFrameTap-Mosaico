#pragma once
#include "oft_head_target.h"
#include "oft_control.h"
typedef struct {
    float deadband_deg,offset_per_degree;
    int maximum_offset;
} oft_head_servo_config_t;
oft_head_servo_config_t oft_head_servo_default(void);
/* Angle-error -> the EXISTING single-writer normalized input. No direct writes,
   no time integration/windup, no invented camera angular command. Gain is an
   engineering controller parameter, not a protocol degrees/second conversion. */
bool oft_head_servo_input(oft_head_result_t target,oft_head_servo_config_t config,
    uint32_t gesture,int64_t now_us,oft_input_t *input);
