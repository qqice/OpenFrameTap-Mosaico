#pragma once
#include "esp_err.h"
#include "oft_orientation.h"
#include <stdint.h>
typedef struct {
    bool imu_ok,mag_ok[2],bias_ready,motor_on;
    bool orientation_ready;unsigned orientation_epoch,recovery_count,event_drops;
    uint8_t imu_id,mag_id[2];
    esp_err_t imu_init_error;unsigned imu_init_stage;
    float accel[3],gyro[3],bias[3],mag_ut[2][3];
    int16_t mag_raw[2][3];
    uint16_t mag_rhall[2];
    uint8_t mag_valid_axes[2]; /* bit0=X, bit1=Y, bit2=Z; never infer from 0uT */
    int64_t mag_sample_us[2];
    oft_quat_t orientation;
    int64_t sample_us;
    unsigned samples,errors,stationary_samples;
} oft_motion_snapshot_t;
typedef struct {int64_t timestamp_us,interval_us;unsigned epoch;bool gap,on;} oft_motion_event_t;
bool oft_motion_event_pop(oft_motion_event_t *event);
esp_err_t oft_motion_start(void);
void oft_motion_snapshot(oft_motion_snapshot_t *out);
/* Leased output: refresh while a validated live limit exists; false stops now. */
void oft_motion_haptic(bool on);
void oft_motion_haptic_test(void);
/* Two50ms pulses with50ms gap; release cancels immediately. */
void oft_motion_head_ack(bool start);
