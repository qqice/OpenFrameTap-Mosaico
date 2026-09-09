#include "oft_orientation.h"
#include <math.h>
#define RAD (0.017453292519943295f)
oft_quat_t oft_quat_identity(void){return (oft_quat_t){1,0,0,0};}
bool oft_orientation_from_gravity(const float a[3],oft_quat_t *q)
{
    if(!a||!q||!isfinite(a[0])||!isfinite(a[1])||!isfinite(a[2]))return false;
    float n=hypotf(hypotf(a[0],a[1]),a[2]);if(n<.9f||n>1.1f)return false;
    *q=a[2]/n>-.999f?(oft_quat_t){1+a[2]/n,a[1]/n,-a[0]/n,0}:(oft_quat_t){0,1,0,0};
    return oft_quat_normalize(q);
}
bool oft_quat_normalize(oft_quat_t *q)
{
    float n=sqrtf(q->w*q->w+q->x*q->x+q->y*q->y+q->z*q->z);
    if(!isfinite(n)||n<1e-8f)return false;
    q->w/=n;q->x/=n;q->y/=n;q->z/=n;return true;
}
oft_quat_t oft_quat_multiply(oft_quat_t a,oft_quat_t b)
{
    return (oft_quat_t){a.w*b.w-a.x*b.x-a.y*b.y-a.z*b.z,
      a.w*b.x+a.x*b.w+a.y*b.z-a.z*b.y,
      a.w*b.y-a.x*b.z+a.y*b.w+a.z*b.x,
      a.w*b.z+a.x*b.y-a.y*b.x+a.z*b.w};
}
oft_quat_t oft_quat_inverse(oft_quat_t q){return (oft_quat_t){q.w,-q.x,-q.y,-q.z};}
void oft_quat_rotate(oft_quat_t q,const float v[3],float out[3])
{
    oft_quat_t r=oft_quat_multiply(oft_quat_multiply(q,(oft_quat_t){0,v[0],v[1],v[2]}),oft_quat_inverse(q));
    out[0]=r.x;out[1]=r.y;out[2]=r.z;
}
bool oft_orientation_step(oft_quat_t *q,const float gyro[3],const float accel[3],float dt)
{
    if(!q||!gyro||!accel||!isfinite(dt)||dt<=0||dt>.05f)return false;
    float rate[3];for(unsigned i=0;i<3;i++){if(!isfinite(gyro[i])||!isfinite(accel[i]))return false;rate[i]=gyro[i]*RAD;}
    /* Gravity feedback only when acceleration is near1g. No claim that an
       uncalibrated magnetic field provides an absolute heading. */
    float an=hypotf(hypotf(accel[0],accel[1]),accel[2]);
    if(an>.9f&&an<1.1f){
        float expected[3];const float up[]={0,0,1};oft_quat_rotate(oft_quat_inverse(*q),up,expected);
        float a[3]={accel[0]/an,accel[1]/an,accel[2]/an};
        const float gain=1.5f;
        rate[0]+=gain*(a[1]*expected[2]-a[2]*expected[1]);
        rate[1]+=gain*(a[2]*expected[0]-a[0]*expected[2]);
        rate[2]+=gain*(a[0]*expected[1]-a[1]*expected[0]);
    }
    oft_quat_t d=oft_quat_multiply(*q,(oft_quat_t){0,rate[0],rate[1],rate[2]});
    q->w+=d.w*dt*.5f;q->x+=d.x*dt*.5f;q->y+=d.y*dt*.5f;q->z+=d.z*dt*.5f;
    return oft_quat_normalize(q);
}
bool oft_orientation_relative(oft_quat_t ref,oft_quat_t now,float *yaw,float *pitch)
{
    if(!yaw||!pitch||!oft_quat_normalize(&ref)||!oft_quat_normalize(&now))return false;
    /* Sensor-frame convention pending physical mounting check: +X right,
       +Y top, +Z toward screen viewer; look away through the device (-Z).
       Quaternion nose direction avoids Euler yaw/pitch subtraction coupling. */
    const float forward[]={0,0,-1};float v[3];
    oft_quat_rotate(oft_quat_multiply(oft_quat_inverse(ref),now),forward,v);
    *yaw=atan2f(v[0],-v[2])/RAD;*pitch=atan2f(v[1],hypotf(v[0],v[2]))/RAD;
    return isfinite(*yaw)&&isfinite(*pitch);
}
static float nearest_angle(float angle,float reference)
{return reference+remainderf(angle-reference,360.0f);}
bool oft_orientation_direction(oft_quat_t ref,oft_quat_t now,oft_direction_tracker_t *s)
{
    if(!s||!oft_quat_normalize(&ref)||!oft_quat_normalize(&now))return false;
    if(!s->valid)*s=(oft_direction_tracker_t){.valid=true};
    if(!isfinite(s->yaw)||!isfinite(s->pitch))return false;
    const float forward[]={0,0,-1};float v[3];
    oft_quat_rotate(oft_quat_multiply(oft_quat_inverse(ref),now),forward,v);
    float horizontal=hypotf(v[0],v[2]);
    bool vertical=horizontal<(s->near_vertical?.25881905f:.17364818f);
    if(vertical){
        /* Signed projection onto retained heading allows pitch through90deg
           without unwrapping the noisy azimuth into a full yaw turn. */
        float along=v[0]*sinf(s->yaw*RAD)-v[2]*cosf(s->yaw*RAD);
        s->pitch=nearest_angle(atan2f(v[1],along)/RAD,s->pitch);
    }else{
        float y=atan2f(v[0],-v[2])/RAD,p=atan2f(v[1],horizontal)/RAD;
        float y0=nearest_angle(y,s->yaw),p0=nearest_angle(p,s->pitch);
        float y1=nearest_angle(y+180,s->yaw),p1=nearest_angle(p>=0?180-p:-180-p,s->pitch);
        float d0=(y0-s->yaw)*(y0-s->yaw)+(p0-s->pitch)*(p0-s->pitch);
        float d1=(y1-s->yaw)*(y1-s->yaw)+(p1-s->pitch)*(p1-s->pitch);
        s->yaw=d1<d0?y1:y0;s->pitch=d1<d0?p1:p0;
    }
    s->near_vertical=vertical;return isfinite(s->yaw)&&isfinite(s->pitch);
}
