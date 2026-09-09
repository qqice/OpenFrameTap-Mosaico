#pragma once
#include "esp_err.h"
#include "oft_control.h"
#include "oft_capture.h"
#include "oft_settings.h"
typedef struct {oft_settings_state_t values;bool busy;unsigned sent,confirmed,errors;char detail[80];} oft_settings_snapshot_t;
void oft_udp_settings_snapshot(oft_settings_snapshot_t *out);
bool oft_udp_setting(oft_setting_action_t action,unsigned a,unsigned b);
void oft_udp_settings_refresh(void);
/* Bounded readback/restore:0 fps,1 zoom,2 mode,3 file,4 aspect,5 HD,6 2.7K. */
bool oft_udp_settings_test(unsigned kind);
typedef enum {OFT_UDP_OFF,OFT_UDP_HANDSHAKE,OFT_UDP_REGISTER,OFT_UDP_ENABLE,OFT_UDP_READY,OFT_UDP_FAULT,OFT_UDP_ARMING} oft_udp_state_t;
typedef struct {
    oft_udp_state_t state;char detail[64];
    unsigned packets,media_packets,bytes,acks,frames,invalid_packets,ambiguous_status;
    unsigned registrations,enable_count,heartbeat_sent,heartbeat_replies;
    unsigned keyframe_requests,keyframe_replies,keyframe_failures;
    int64_t keyframe_sent_us;bool keyframe_pending;
    unsigned commands,nonzero_commands,zero_commands,control_failures;
    uint32_t max_ack_gap_us,stack_free;
    uint16_t local_port;
    int64_t started_us,last_packet_us,last_ack_us;
    bool battery_valid;uint8_t battery;int64_t battery_updated_us;
    int16_t gimbal_raw[3];bool have_gimbal;
    uint8_t gimbal_frame[62];int64_t gimbal_updated_us;
    /* Bounded passive status snapshots:04/27 and04/38. Raw semantics unknown. */
    uint8_t gimbal_status_frame[2][96],gimbal_status_size[2];
    int64_t gimbal_status_us[2];
    bool probe_running,probe_done;
    bool zero_receipt_confirmed;uint32_t first_zero_delay_us;
    bool live_enabled;unsigned mock_commands,mock_zeros,action_count;
    bool action_busy;
    bool settings_busy;
    oft_camera_state_t camera;bool shutter_busy;unsigned shutter_sent,shutter_errors;
    char shutter_detail[64];
    bool head_mode,head_held,head_sensor_ready,head_release_required;
    bool head_sensor_calibrated;
    bool head_yaw_held;
    float head_yaw_deg,head_pitch_deg;
    float head_camera_yaw,head_camera_pitch,head_target_yaw,head_target_pitch;
    bool head_tracking,head_trial_guard;
    unsigned head_limit_mask;
    bool head_yaw_aligned;float head_yaw_shift;
    oft_control_state_t control_state;
} oft_udp_snapshot_t;
esp_err_t oft_udp_start(void);
void oft_udp_snapshot(oft_udp_snapshot_t *out);
void oft_udp_input(const oft_input_t *input);
void oft_udp_stop(void);
void oft_udp_disconnect(void);
/* Bring-up only (live UI must be disabled). One explicit low-output 200ms
   probe, direction0..3=yaw+/yaw-/pitch+/pitch-.
   No public raw-write or unrestricted live-arm interface in bring-up image. */
bool oft_udp_probe(unsigned direction);
bool oft_udp_action(bool flip);
bool oft_udp_shutter(int64_t physical_press_us);
void oft_udp_head_mode(bool head_mode);
void oft_udp_head_hold(bool held,bool new_press);
/* Only queues a typed request; the existing UDP owner remains the only writer. */
bool oft_udp_keyframe(void);
void oft_udp_cancel_keyframe(void);
