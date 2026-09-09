#include "oft_control.h"
#include <math.h>
#include <string.h>

void oft_control_init(oft_control_t *c){memset(c,0,sizeof(*c));}
bool oft_control_map(float yaw,float pitch,int *y,int *p)
{
    if(!y||!p||!isfinite(yaw)||!isfinite(pitch)||fabsf(yaw)>1||fabsf(pitch)>1)return false;
    float r=hypotf(yaw,pitch);*y=*p=0;if(r<=.12f)return true;
    /* Deadzone -> minimum32; edge ->188, radial normalization avoids a faster
       diagonal. These are protocol offsets, not claimed degrees per second. */
    float radius=fminf(r,1),offset=32+(radius-.12f)/.88f*(188-32);
    *y=(int)lroundf(yaw/r*offset);*p=(int)lroundf(pitch/r*offset);return true;
}
static bool emit(oft_control_t *c,int y,int p,int64_t now,oft_control_send_t send,void *ctx)
{
    c->sends++;
    if(y||p)c->stop_required=true; /* failed send may still have left the host */
    if(!send||!send(y,p,ctx)){
        c->failures++;c->state=OFT_CONTROL_FAULT;
        c->blocked_gesture=c->last_gesture;c->have_blocked_gesture=true;
        /* Output fields retain the last successful nonzero send. Never claim
           physical stop or delivery when the write failed. */
        return false;
    }
    c->yaw_offset=y;c->pitch_offset=p;c->last_send_us=now;c->have_sent=true;
    if(!y&&!p){c->zeros++;c->last_zero_us=now;c->stop_required=false;}
    return true;
}
void oft_control_stop(oft_control_t *c,bool fault,int64_t now,oft_control_send_t send,void *ctx)
{
    c->blocked_gesture=c->last_gesture;c->have_blocked_gesture=true;
    if(c->stop_required){
        /* One attempt plus one zero-only retry; no retry loop or new cmdId. */
        if(!emit(c,0,0,now,send,ctx)&&!emit(c,0,0,now,send,ctx))return;
        if(c->state==OFT_CONTROL_FAULT)fault=true;
    }
    c->state=fault?OFT_CONTROL_FAULT:OFT_CONTROL_DISABLED;
}
void oft_control_reset(oft_control_t *c)
{
    /* Only an explicit session reset may clear the fault; retain old gesture
       block and do not erase uncertainty about an undelivered zero. */
    if(c->stop_required)return;
    c->state=OFT_CONTROL_DISABLED;
}
void oft_control_tick(oft_control_t *c,const oft_input_t *in,bool gate,int64_t now,
    oft_control_send_t send,void *ctx)
{
    if(c->state==OFT_CONTROL_FAULT)return;
    if(!gate){oft_control_stop(c,c->state==OFT_CONTROL_ACTIVE,now,send,ctx);return;}
    if(!in||in->timestamp_us>now||now-in->timestamp_us>=250000){
        if(c->state==OFT_CONTROL_ACTIVE)oft_control_stop(c,true,now,send,ctx);
        return;
    }
    if(c->have_blocked_gesture&&in->gesture==c->blocked_gesture&&in->active)return;
    c->last_gesture=in->gesture;
    int y=0,p=0;
    if(in->active&&!oft_control_map(in->yaw,in->pitch,&y,&p)){
        oft_control_stop(c,true,now,send,ctx);return;
    }
    if(!y&&!p){
        /* Release/cancel/neutral bypass the 10 Hz nonzero rate limit. */
        if(c->stop_required&&!emit(c,0,0,now,send,ctx)){
            emit(c,0,0,now,send,ctx);c->state=OFT_CONTROL_FAULT;return;
        }
        c->state=OFT_CONTROL_ARMED;return;
    }
    if(c->have_sent&&now-c->last_send_us<100000)return;
    if(emit(c,y,p,now,send,ctx))c->state=OFT_CONTROL_ACTIVE;
    else oft_control_stop(c,true,now,send,ctx);
}
