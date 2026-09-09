#pragma once
#include "oft_orientation.h"
#include <stdint.h>

/* Degree-domain target generation only. This component cannot send commands.
   The camera adapter must supply measured/validated units and unwrapped yaw.
   Do not set bounds_valid from an uncalibrated raw encoder or a stall. */
typedef struct {
    float yaw,pitch;
    int64_t timestamp_us;
    bool valid;
} oft_head_camera_t;
typedef struct {bool pending;int64_t due_us,expires_us,last_sample_us;unsigned count;float low,high;} oft_head_center_anchor_t;
void oft_head_center_begin(oft_head_center_anchor_t *anchor,int64_t ack_us);
bool oft_head_center_sample(oft_head_center_anchor_t *anchor,oft_head_camera_t camera,bool no_motion_since_ack,int64_t now_us,float *center);
typedef struct {bool valid;float raw,continuous;int64_t timestamp_us;} oft_head_yaw_tracker_t;
/* Unwrap consecutive camera samples, not merely the Mosaico target. A gap
   invalidates active pursuit; the next sample seeds a new tracking segment. */
bool oft_head_yaw_update(oft_head_yaw_tracker_t *tracker,float raw,int64_t timestamp_us,float *continuous);
typedef struct {
    bool bounds_valid;
    float yaw_min,yaw_max,pitch_min,pitch_max;
} oft_head_bounds_t;
/* Position-based, non-latching measured-SOFT-limit indication:bit0 yaw,bit1 pitch.
   Does not depend on a held input; leaving HEAD or stale feedback clears it. */
unsigned oft_head_limit_status(oft_head_camera_t camera,oft_head_bounds_t bounds,int64_t now_us,bool enabled);
typedef struct {
    oft_quat_t reference;
    oft_direction_tracker_t direction;
    float camera_yaw,camera_pitch,previous_yaw,relative_yaw;
    bool active,release_required;
} oft_head_target_t;
typedef struct {
    bool active,release_required,yaw_limit,pitch_limit;
    float requested_yaw,requested_pitch,target_yaw,target_pitch;
    float yaw_error,pitch_error;
} oft_head_result_t;
void oft_head_target_init(oft_head_target_t *state);
/* Cancel on action/mode change/connection loss; an old held touch cannot rearm. */
void oft_head_target_cancel(oft_head_target_t *state);
/* Temporary trial TARGET window, not a physical limit or a cancellation.
   Returns whether clipping occurred; invalid arguments fail closed in result. */
bool oft_head_target_window(oft_head_result_t *result,const oft_head_target_t *state,float degrees);
oft_head_result_t oft_head_target_step(oft_head_target_t *state,bool held,
    int64_t input_us,oft_quat_t orientation,int64_t sensor_us,
    oft_head_camera_t camera,oft_head_bounds_t bounds,int64_t now_us);
