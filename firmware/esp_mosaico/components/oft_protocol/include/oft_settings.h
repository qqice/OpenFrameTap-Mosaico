#pragma once
#include "oft_duml.h"
#include "oft_capture.h"
#define OFT_SETTINGS_PAIRS 64
typedef enum {
    OFT_SUB_VIDEO_CAP,OFT_SUB_VIDEO,OFT_SUB_LENS,OFT_SUB_FOV,
    OFT_SUB_PHOTO,OFT_SUB_PHOTO_SIZE_CAP,OFT_SUB_PHOTO_FILE_CAP,OFT_SUB_ZOOM_CAP,
    OFT_SETTINGS_SUB_COUNT,
    OFT_SET_VIDEO=20,OFT_SET_ZOOM,OFT_SET_MODE,OFT_SET_PHOTO_SIZE,OFT_SET_PHOTO_FILE,
    OFT_GET_PHOTO_SIZE,OFT_GET_PHOTO_FILE
} oft_setting_action_t;
typedef struct {uint8_t resolution,fps,flags;} oft_video_pair_t;
typedef struct {
    oft_video_pair_t pairs[OFT_SETTINGS_PAIRS];uint8_t pair_count,resolution,fps;
    bool video_valid,lens_valid,photo_valid,photo_file_valid,video_cap_valid,reference_pairs;
    uint16_t lens;uint32_t fov;uint8_t photo_size,photo_aspect,photo_file;
    uint8_t photo_sizes[8],photo_files[8],photo_size_count,photo_file_count;
    unsigned updates,unknown,malformed;
    int64_t video_us,lens_us,photo_us,photo_file_us;
} oft_settings_state_t;
typedef struct {const char *name;const uint8_t *value;size_t size;unsigned key;} oft_setting_item_t;
const char *oft_settings_key(unsigned key);
bool oft_settings_item(const oft_duml_frame_t *frame,oft_setting_item_t *item);
bool oft_settings_observe(oft_settings_state_t *state,const oft_duml_frame_t *frame,int64_t now);
bool oft_settings_video_pair(const oft_settings_state_t *state,unsigned resolution,unsigned fps);
bool oft_settings_reference_pairs(oft_settings_state_t *state,unsigned camera_mode);
unsigned oft_settings_fps(unsigned code);
const char *oft_settings_resolution(unsigned code);
unsigned oft_settings_zoom_max(unsigned mode,unsigned resolution);
size_t oft_settings_build(oft_setting_action_t action,unsigned a,unsigned b,uint16_t sequence,uint8_t *out,size_t capacity);
bool oft_settings_allowed(const uint8_t *raw,size_t size);
bool oft_settings_can_set(const oft_settings_state_t *state,oft_camera_state_t camera,oft_setting_action_t action,unsigned a,unsigned b,int64_t now);
