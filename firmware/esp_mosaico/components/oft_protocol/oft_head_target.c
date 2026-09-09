#include "oft_head_target.h"
#include <math.h>
#include <string.h>
void oft_head_target_init(oft_head_target_t *s){memset(s,0,sizeof(*s));}
void oft_head_target_cancel(oft_head_target_t *s){s->active=false;s->release_required=true;}
static bool fresh(int64_t at,int64_t now,int64_t max_age)
{return at>0&&now>=at&&now-at<max_age;}
static float clamp(float x,float lo,float hi){return fminf(hi,fmaxf(lo,x));}
void oft_head_center_begin(oft_head_center_anchor_t *s,int64_t at)
{*s=(oft_head_center_anchor_t){.pending=true,.due_us=at+1000000,.expires_us=at+3500000};}
bool oft_head_center_sample(oft_head_center_anchor_t *s,oft_head_camera_t c,bool quiet,int64_t now,float *center)
{
    if(!s||!center||!s->pending)return false;
    if(!quiet||now>=s->expires_us){s->pending=false;return false;}
    if(now<s->due_us||!c.valid||!fresh(c.timestamp_us,now,300000)||!isfinite(c.yaw)||!isfinite(c.pitch)||fabsf(c.pitch)>1)return false;
    if(c.timestamp_us<=s->last_sample_us)return false;
    s->last_sample_us=c.timestamp_us;
    if(!s->count){s->low=s->high=c.yaw;s->count=1;}
    else{
        s->low=fminf(s->low,c.yaw);s->high=fmaxf(s->high,c.yaw);
        if(s->high-s->low>.15f){s->low=s->high=c.yaw;s->count=1;}else s->count++;
    }
    if(s->count<5)return false;
    *center=(s->low+s->high)*.5f;s->pending=false;return true;
}
unsigned oft_head_limit_status(oft_head_camera_t c,oft_head_bounds_t b,int64_t now,bool enabled)
{
    if(!enabled||!c.valid||!b.bounds_valid||!fresh(c.timestamp_us,now,300000)||
       !isfinite(c.yaw)||!isfinite(c.pitch)||!isfinite(b.yaw_min)||!isfinite(b.yaw_max)||
       !isfinite(b.pitch_min)||!isfinite(b.pitch_max)||b.yaw_min>=b.yaw_max||b.pitch_min>=b.pitch_max)return 0;
    unsigned mask=0;
    if(c.yaw<=b.yaw_min+.75f||c.yaw>=b.yaw_max-.75f)mask|=1;
    if(c.pitch<=b.pitch_min+.75f||c.pitch>=b.pitch_max-.75f)mask|=2;
    return mask;
}
bool oft_head_yaw_update(oft_head_yaw_tracker_t *s,float raw,int64_t at,float *out)
{
    if(!s||!out)return false;
    if(!isfinite(raw)||fabsf(raw)>180.5f||at<=0){s->valid=false;return false;}
    if(s->valid&&(at<s->timestamp_us||at-s->timestamp_us>=300000)){
        s->valid=false;return false;
    }
    if(!s->valid){s->continuous=raw;s->valid=true;}
    else{
        float d=raw-s->raw;if(d>180)d-=360;else if(d< -180)d+=360;
        s->continuous+=d;
    }
    s->raw=raw;s->timestamp_us=at;*out=s->continuous;return true;
}
bool oft_head_target_window(oft_head_result_t *r,const oft_head_target_t *s,float degrees)
{
    if(!r)return false;
    if(!s||!isfinite(degrees)||degrees<=0||!isfinite(s->camera_yaw)||!isfinite(s->camera_pitch)||
       !isfinite(r->target_yaw)||!isfinite(r->target_pitch)||!isfinite(r->yaw_error)||!isfinite(r->pitch_error)){
        r->active=false;r->release_required=true;return false;
    }
    if(!r->active)return false;
    float y=clamp(r->target_yaw,s->camera_yaw-degrees,s->camera_yaw+degrees);
    float p=clamp(r->target_pitch,s->camera_pitch-degrees,s->camera_pitch+degrees);
    bool clipped=y!=r->target_yaw||p!=r->target_pitch;
    r->yaw_error+=y-r->target_yaw;r->pitch_error+=p-r->target_pitch;
    r->target_yaw=y;r->target_pitch=p;
    return clipped;
}
oft_head_result_t oft_head_target_step(oft_head_target_t *s,bool held,
    int64_t input_us,oft_quat_t orientation,int64_t sensor_us,
    oft_head_camera_t camera,oft_head_bounds_t b,int64_t now)
{
    oft_head_result_t out={0};
    if(!held){s->active=false;s->release_required=false;return out;}
    bool bounds_ok=!b.bounds_valid||(isfinite(b.yaw_min)&&isfinite(b.yaw_max)&&
        isfinite(b.pitch_min)&&isfinite(b.pitch_max)&&b.yaw_min<b.yaw_max&&b.pitch_min<b.pitch_max);
    if(!bounds_ok||!camera.valid||!isfinite(camera.yaw)||!isfinite(camera.pitch)||
       !fresh(input_us,now,250000)||!fresh(sensor_us,now,100000)||
       !fresh(camera.timestamp_us,now,300000)||!oft_quat_normalize(&orientation))
        oft_head_target_cancel(s);
    if(s->release_required){out.release_required=true;return out;}
    if(!s->active){
        s->reference=orientation;s->camera_yaw=camera.yaw;s->camera_pitch=camera.pitch;
        s->direction=(oft_direction_tracker_t){0};
        s->previous_yaw=0;s->relative_yaw=0;s->active=true;
    }
    if(!oft_orientation_direction(s->reference,orientation,&s->direction)){
        oft_head_target_cancel(s);out.release_required=true;return out;
    }
    s->relative_yaw=s->direction.yaw;s->previous_yaw=s->direction.yaw;
    out.requested_yaw=s->camera_yaw+s->relative_yaw;
    out.requested_pitch=s->camera_pitch+s->direction.pitch;
    out.target_yaw=out.requested_yaw;out.target_pitch=out.requested_pitch;
    if(b.bounds_valid){
        out.target_yaw=clamp(out.target_yaw,b.yaw_min,b.yaw_max);
        out.target_pitch=clamp(out.target_pitch,b.pitch_min,b.pitch_max);
        /* At an actual endpoint slightly outside our conservative soft bound,
           pressing with zero relative input must NOT create an inward jump.
           Block further outward demand; inward demand is still permitted. */
        if((camera.yaw>b.yaw_max&&out.requested_yaw>=camera.yaw)||
           (camera.yaw<b.yaw_min&&out.requested_yaw<=camera.yaw))out.target_yaw=camera.yaw;
        if((camera.pitch>b.pitch_max&&out.requested_pitch>=camera.pitch)||
           (camera.pitch<b.pitch_min&&out.requested_pitch<=camera.pitch))out.target_pitch=camera.pitch;
        /* Warn only at a measured, calibrated controllable boundary while the
           requested target asks farther outward. Clear on the very next sample
           away from it. These are NOT mechanical contact/status-bit claims. */
        out.yaw_limit=(camera.yaw>=b.yaw_max-.75f&&out.requested_yaw>b.yaw_max)||
            (camera.yaw<=b.yaw_min+.75f&&out.requested_yaw<b.yaw_min);
        out.pitch_limit=(camera.pitch>=b.pitch_max-.75f&&out.requested_pitch>b.pitch_max)||
            (camera.pitch<=b.pitch_min+.75f&&out.requested_pitch<b.pitch_min);
    }
    out.yaw_error=out.target_yaw-camera.yaw;out.pitch_error=out.target_pitch-camera.pitch;
    out.active=true;return out;
}
