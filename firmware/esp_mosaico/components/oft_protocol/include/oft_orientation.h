#pragma once
#include <stdbool.h>
typedef struct {float w,x,y,z;} oft_quat_t;
typedef struct {float yaw,pitch;bool valid,near_vertical;} oft_direction_tracker_t;
oft_quat_t oft_quat_identity(void);
bool oft_quat_normalize(oft_quat_t *q);
bool oft_orientation_from_gravity(const float accel_g[3],oft_quat_t *orientation);
oft_quat_t oft_quat_multiply(oft_quat_t a,oft_quat_t b);
oft_quat_t oft_quat_inverse(oft_quat_t q);
void oft_quat_rotate(oft_quat_t q,const float v[3],float out[3]);
bool oft_orientation_step(oft_quat_t *q,const float gyro_dps[3],const float accel_g[3],float dt);
bool oft_orientation_relative(oft_quat_t reference,oft_quat_t current,float *yaw_deg,float *pitch_deg);
/* Continuous direction past90deg; hold yaw within10deg of vertical, release
   outside15deg. Azimuth is ill-conditioned near the vertical pole. */
bool oft_orientation_direction(oft_quat_t reference,oft_quat_t current,oft_direction_tracker_t *state);
