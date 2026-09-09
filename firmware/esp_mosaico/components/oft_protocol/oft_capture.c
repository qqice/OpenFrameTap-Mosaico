#include "oft_capture.h"
#include <string.h>
/* OPC CameraStatus.swift: 02/80 flags@0, recording bit7, playback bit30,
   shooting mode@57. Only ordinary Pocket photo(05) and video(01) enabled.
   Unknown/special modes are never interpreted as video by default. */
bool oft_capture_status(const oft_duml_frame_t *f,int64_t now,oft_camera_state_t *out)
{
    if(!f||!out||f->sender!=1||f->receiver!=2||f->cmd_set!=2||f->cmd_id!=0x80||f->flags!=0||f->payload_size<58)return false;
    const uint8_t *p=f->payload;uint32_t flags=(uint32_t)p[0]|((uint32_t)p[1]<<8)|((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24);
    *out=(oft_camera_state_t){.raw_mode=p[57],.flags=flags,.updated_us=now,
        .recording=(flags&0x80)!=0,.transition=(flags&0xc0)==0x40,.playback=(flags&0x40000000)!=0};
    out->mode=p[57]==5?OFT_CAPTURE_PHOTO:p[57]==1?OFT_CAPTURE_VIDEO:OFT_CAPTURE_UNKNOWN;
    out->valid=out->mode!=OFT_CAPTURE_UNKNOWN&&!out->playback;
    return true;
}
oft_shutter_action_t oft_capture_select(oft_camera_state_t s,int64_t now)
{
    if(!s.valid||s.playback||s.transition||now<s.updated_us||now-s.updated_us>2000000)return OFT_SHUTTER_NONE;
    if(s.mode==OFT_CAPTURE_VIDEO)return s.recording?OFT_SHUTTER_STOP:OFT_SHUTTER_START;
    if(s.mode==OFT_CAPTURE_PHOTO&&!s.recording)return OFT_SHUTTER_PHOTO;
    return OFT_SHUTTER_NONE;
}
bool oft_button_step(oft_button_filter_t *s,bool down,int64_t now)
{
    if(down!=s->down){s->down=down;s->changed_us=now;return false;}
    if(now<s->changed_us||now-s->changed_us<40000)return false;
    if(!down){s->blocked=false;return false;}
    if(s->blocked)return false;
    s->blocked=true;return true;
}
void oft_shutter_begin(oft_shutter_tx_t *t,oft_shutter_action_t action,uint16_t seq,int64_t now)
{*t=(oft_shutter_tx_t){.action=action,.sequence=seq,.sent_us=now,.pending=action!=OFT_SHUTTER_NONE};}
bool oft_shutter_reply(oft_shutter_tx_t *t,const oft_duml_frame_t *f)
{
    if(!t->pending||!f||f->sender!=1||f->receiver!=2||f->cmd_set!=2||
       f->cmd_id!=(t->action==OFT_SHUTTER_PHOTO?1:2)||f->sequence!=t->sequence||(f->flags!=0x80&&f->flags!=0xc0))return false;
    if(f->payload_size!=1||f->payload[0]){t->result=-2;t->pending=false;return true;}
    t->acknowledged=true;
    if(t->action==OFT_SHUTTER_PHOTO){t->pending=false;t->result=1;} // Accepted, not proof of an SD file.
    return true;
}
void oft_shutter_observe(oft_shutter_tx_t *t,oft_camera_state_t s,int64_t now)
{
    if(!t->pending)return;
    if(t->acknowledged&&s.valid&&!s.transition&&s.updated_us>t->sent_us&&
        s.mode==OFT_CAPTURE_VIDEO&&s.recording==(t->action==OFT_SHUTTER_START)){
        t->pending=false;t->result=1;return;
    }
    if(now<t->sent_us||now-t->sent_us>=4000000){t->pending=false;t->result=-1;}
}
