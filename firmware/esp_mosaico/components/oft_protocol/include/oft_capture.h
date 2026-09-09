#pragma once
#include "oft_duml.h"
#include <stdbool.h>
typedef enum {OFT_CAPTURE_UNKNOWN,OFT_CAPTURE_PHOTO,OFT_CAPTURE_VIDEO} oft_capture_mode_t;
typedef struct {
    oft_capture_mode_t mode;bool valid,recording,transition,playback;
    uint8_t raw_mode;uint32_t flags;int64_t updated_us;
} oft_camera_state_t;
typedef enum {OFT_SHUTTER_NONE,OFT_SHUTTER_PHOTO,OFT_SHUTTER_START,OFT_SHUTTER_STOP} oft_shutter_action_t;
bool oft_capture_status(const oft_duml_frame_t *frame,int64_t now,oft_camera_state_t *out);
oft_shutter_action_t oft_capture_select(oft_camera_state_t state,int64_t now);
typedef struct {bool down,blocked;int64_t changed_us;} oft_button_filter_t;
bool oft_button_step(oft_button_filter_t *filter,bool down,int64_t now);
typedef struct {oft_shutter_action_t action;uint16_t sequence;int64_t sent_us;bool pending,acknowledged;int result;} oft_shutter_tx_t;
void oft_shutter_begin(oft_shutter_tx_t *tx,oft_shutter_action_t action,uint16_t seq,int64_t now);
bool oft_shutter_reply(oft_shutter_tx_t *tx,const oft_duml_frame_t *frame);
void oft_shutter_observe(oft_shutter_tx_t *tx,oft_camera_state_t state,int64_t now);
