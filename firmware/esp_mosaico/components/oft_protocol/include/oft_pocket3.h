#pragma once
#include "oft_duml.h"
#include "oft_head_target.h"
/* Four directions and stop confirmed by owner, 2026-09-06; see firmware README. */
#define OFT_P3_TOUCH_CONTROL_VALIDATED 1

typedef enum {
    OFT_P3_OPEN, OFT_P3_BLE_HEARTBEAT, OFT_P3_PAIR_STATUS, OFT_P3_PAIR_ACK,
    OFT_P3_PAIR_FINISH, OFT_P3_SSID, OFT_P3_PASSWORD, OFT_P3_PRESENCE,
    OFT_P3_ENABLE, OFT_P3_CONTROL_HEARTBEAT, OFT_P3_STICK, OFT_P3_RECENTER,
    OFT_P3_FLIP, OFT_P3_PHOTO, OFT_P3_RECORD_START, OFT_P3_RECORD_STOP, OFT_P3_COMMAND_COUNT
} oft_p3_command_t;
/* No cmdSet/cmdId or arbitrary payload argument. Motion additionally needs a
   session-level validated control gate; the wire builder alone never arms it. */
size_t oft_p3_build(oft_p3_command_t command,uint16_t sequence,int yaw_offset,
    int pitch_offset,uint8_t *out,size_t capacity);
bool oft_p3_allowed(const uint8_t *raw,size_t size,bool ble,bool control_validated);
bool oft_p3_ble_reply_matches(oft_p3_command_t command,uint16_t sequence,const oft_duml_frame_t *frame);
bool oft_p3_wifi_string(const uint8_t *payload,size_t size,bool password,char *out,size_t capacity);
bool oft_p3_battery(const oft_duml_frame_t *frame,uint8_t *percent);
bool oft_p3_advertisement(const uint8_t *manufacturer,size_t size);
bool oft_p3_gimbal_candidates(const oft_duml_frame_t *frame,int16_t values[3]);
/* Experimental angle feedback; yaw16 /100 and pitch20 /-10. Cross-checked
   against captured flip/tilt and quaternion deltas; not physical gain acceptance. */
bool oft_p3_head_feedback(const oft_duml_frame_t *frame,float *yaw,float *pitch);
/* Bench-calibrated INNER travel envelope from both owner-observed facings.
   Not DJI mechanical limits or a universal calibration for other cameras. */
oft_head_bounds_t oft_p3_head_bounds(void);
float oft_p3_head_yaw_seed(float wrapped_yaw);
bool oft_p3_head_center_shift(float centered_yaw,float *shift);
size_t oft_p3_registration_reply(const oft_duml_frame_t *request,uint8_t *out,size_t capacity);
