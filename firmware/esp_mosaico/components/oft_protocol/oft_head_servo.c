#include "oft_head_servo.h"
#include <math.h>
oft_head_servo_config_t oft_head_servo_default(void)
{
    /* Same188 envelope already accepted for joystick. Faster large-error
       pursuit, proportional slowdown near target; no frequency increase. */
    return(oft_head_servo_config_t){.75f,16,188};
}
bool oft_head_servo_input(oft_head_result_t target,oft_head_servo_config_t c,
    uint32_t gesture,int64_t now,oft_input_t *out)
{
    if(!out)return false;
    *out=(oft_input_t){.timestamp_us=now,.gesture=gesture};
    if(!isfinite(c.deadband_deg)||c.deadband_deg<=0||!isfinite(c.offset_per_degree)||
       c.offset_per_degree<=0||c.maximum_offset<32||c.maximum_offset>188)return false;
    if(!target.active||target.release_required)return true;
    float y=target.yaw_error,p=target.pitch_error;
    if(!isfinite(y)||!isfinite(p))return false;
    if(fabsf(y)<=c.deadband_deg)y=0;
    if(fabsf(p)<=c.deadband_deg)p=0;
    float magnitude=hypotf(y,p);if(!isfinite(magnitude))return false;
    if(magnitude==0)return true;
    float offset=fminf(c.maximum_offset,fmaxf(32,magnitude*c.offset_per_degree));
    /* Inverse of oft_control_map's radial32..188 envelope. Tiny epsilon keeps
       the minimum request above the inclusive .12 input deadzone. */
    float radius=fmaxf(.1201f,.12f+.88f*(offset-32)/156);
    out->yaw=y/magnitude*radius;out->pitch=p/magnitude*radius;out->active=true;
    return true;
}
