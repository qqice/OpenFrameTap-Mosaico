#include "oft_pocket3.h"
#include <math.h>
#include <string.h>
#include <ctype.h>

/* Existing reviewed Pocket 3 profile; not a generic DJI command catalogue.
   Sources: devices/pocket3{,_normal}.py, control/gimbal_profile.py and
   protocol/camera_actions.py. Shutter: reviewed OPC Commands.swift at9c4e7334;
   physical-button requests only. No RTMP, calibration or raw PWM API. */
typedef struct {uint8_t target,set,id,flags,size;uint8_t payload[24];} definition_t;
static const definition_t defs[OFT_P3_COMMAND_COUNT]={
    [OFT_P3_OPEN]={0xf0,0,0x2b,0x40,2,{4,0}},
    [OFT_P3_BLE_HEARTBEAT]={0xf0,0,0x2b,0x40,2,{1,1}},
    [OFT_P3_PAIR_STATUS]={7,7,0x45,0x40,21,{15,'0','0','1','7','4','9','3','1','9','2','8','6','1','0','2',4,'5','1','6','0'}},
    [OFT_P3_PAIR_ACK]={7,7,0x46,0xc0,1,{0}},
    [OFT_P3_PAIR_FINISH]={0x88,0,0x32,0x40,5,{0x31,0x31,0,0,0}},
    [OFT_P3_SSID]={7,7,7,0x40,0,{0}},
    [OFT_P3_PASSWORD]={7,7,0x0e,0x40,0,{0}},
    [OFT_P3_PRESENCE]={0x28,0,0x88,0x40,14,{0x17,0,0,0x23,0,'A','P','P',0,0,0,0,0,2}},
    [OFT_P3_ENABLE]={0x41,9,0xa8,0x40,10,{0,4,2,0,0,0,0,0,0,0}},
    [OFT_P3_CONTROL_HEARTBEAT]={4,4,0x50,0x40,3,{1,4,5}},
    [OFT_P3_STICK]={4,4,1,0,10,{0,4,0,0,0,4,0,0x80,0x42,0}},
    [OFT_P3_RECENTER]={4,4,0x4c,0x40,2,{0xfe,8}},
    [OFT_P3_FLIP]={4,4,0x4c,0x40,2,{0xfe,9}},
    [OFT_P3_PHOTO]={1,2,1,0x40,1,{1}},
    [OFT_P3_RECORD_START]={1,2,2,0x40,1,{1}},
    [OFT_P3_RECORD_STOP]={1,2,2,0x40,1,{0}},
};
size_t oft_p3_build(oft_p3_command_t c,uint16_t seq,int yaw,int pitch,uint8_t *out,size_t cap)
{
    if((unsigned)c>=OFT_P3_COMMAND_COUNT)return 0;
    definition_t d=defs[c];
    if(c==OFT_P3_STICK){
        if(yaw< -188||yaw>188||pitch< -188||pitch>188)return 0;
        unsigned p=1024+pitch,y=1024+yaw;
        d.payload[0]=p;d.payload[1]=p>>8;d.payload[4]=y;d.payload[5]=y>>8;
    } else if(yaw||pitch)return 0;
    return oft_duml_encode(out,cap,2,d.target,seq,d.flags,d.set,d.id,d.payload,d.size);
}
bool oft_p3_allowed(const uint8_t *raw,size_t size,bool ble,bool control)
{
    oft_duml_frame_t f;if(!oft_duml_decode(raw,size,&f)||f.sender!=2||f.version!=1)return false;
    for(unsigned c=0;c<OFT_P3_COMMAND_COUNT;c++){
        /* Session-open is captured on both BLE and the normal UDP socket. */
        if(ble?(c>OFT_P3_PASSWORD):(c<=OFT_P3_PASSWORD&&c!=OFT_P3_OPEN))continue;
        if(c>=OFT_P3_CONTROL_HEARTBEAT&&!control)continue;
        const definition_t *d=&defs[c];
        if(f.receiver!=d->target||f.cmd_set!=d->set||f.cmd_id!=d->id||f.flags!=d->flags||f.payload_size!=d->size)continue;
        if(c!=OFT_P3_STICK){if(memcmp(f.payload,d->payload,d->size)==0)return true;continue;}
        uint8_t expected[32];
        int pitch=(f.payload[0]|f.payload[1]<<8)-1024,yaw=(f.payload[4]|f.payload[5]<<8)-1024;
        size_t n=oft_p3_build(c,f.sequence,yaw,pitch,expected,sizeof(expected));
        return n==size&&memcmp(raw,expected,n)==0;
    }
    return false;
}
bool oft_p3_ble_reply_matches(oft_p3_command_t cmd,uint16_t seq,const oft_duml_frame_t *f)
{
    if(!f||(cmd!=OFT_P3_PAIR_STATUS&&cmd!=OFT_P3_SSID&&cmd!=OFT_P3_PASSWORD))return false;
    const definition_t *d=&defs[cmd];
    /* Bit7 identifies a reply; captured Pocket BLE replies use C0, while
       80 is also a response class. Keep lower reserved/encryption bits zero.
       Call only with a successfully decoded (CRC-verified) frame. */
    return f->version==1&&f->sequence==seq&&f->sender==d->target&&f->receiver==2&&
        f->cmd_set==d->set&&f->cmd_id==d->id&&(f->flags==0xc0||f->flags==0x80);
}
bool oft_p3_wifi_string(const uint8_t *p,size_t n,bool pass,char *out,size_t cap)
{
    if(!p||!out||n<2||p[0]||n!=p[1]+2u||cap<=p[1])return false;
    size_t len=p[1];
    for(size_t i=2;i<n;i++)if(p[i]<32||p[i]==127)return false;
    if(pass){
        if(len<8||len>64)return false;
        if(len==64)for(size_t i=2;i<n;i++)if(!isxdigit(p[i]))return false;
    }else{
        static const char prefix[]="osmopocket3";
        if(len<sizeof(prefix)-1||len>32)return false;
        for(size_t i=0;i<sizeof(prefix)-1;i++)if(tolower(p[i+2])!=prefix[i])return false;
    }
    memcpy(out,p+2,len);out[len]=0;return true;
}
bool oft_p3_battery(const oft_duml_frame_t *f,uint8_t *percent)
{
    if(!f||!percent||f->cmd_set!=0x0d||f->cmd_id!=2||f->payload_size<=20||f->payload[20]>100)return false;
    *percent=f->payload[20];return true;
}
bool oft_p3_advertisement(const uint8_t *p,size_t n)
{
    /* Same owner's Pocket3: warm 20 00 80 and cold 20 00 C0. The fifth
       manufacturer byte is not an immutable model ID. Do not admit unseen
       model prefixes or arbitrary flag values based on DJI company ID alone. */
    return p&&n>=11&&p[0]==0xaa&&p[1]==8&&p[2]==0x20&&p[3]==0&&(p[4]==0x80||p[4]==0xc0);
}
size_t oft_p3_registration_reply(const oft_duml_frame_t *r,uint8_t *out,size_t cap)
{
    if(!r||r->sender!=0x48||r->receiver!=2||r->cmd_set||
        (r->cmd_id!=0x81&&r->cmd_id!=0x82)||r->flags!=0x40||r->payload_size!=64)return 0;
    uint8_t payload[64]={0};size_t n=1;
    if(r->cmd_id==0x81){n=64;memcpy(payload+1,"APP",3);payload[34]=2;payload[41]=2;payload[42]=8;}
    return oft_duml_encode(out,cap,2,0x48,r->sequence,0x80,0,r->cmd_id,payload,n);
}
bool oft_p3_head_feedback(const oft_duml_frame_t *f,float *yaw,float *pitch)
{
    int16_t raw[3];
    if(!yaw||!pitch||!f||f->sender!=4||f->receiver!=2||f->flags!=0||
       !oft_p3_gimbal_candidates(f,raw))return false;
    *yaw=raw[0]*.01f;*pitch=raw[1]*-.1f;return true;
}
oft_head_bounds_t oft_p3_head_bounds(void)
{
    /* docs/esp-mosaico-head-tracking.md:repeat yaw endpoints -224.46/-224.25,
       48.31/48.53; original pitch -54.9..110.1, selfie -53.8..111.5.
       Stay inside their common envelope; no interpolation of unknown modes. */
    return(oft_head_bounds_t){true,-224.0f,48.0f,-53.5f,110.0f};
}
float oft_p3_head_yaw_seed(float raw)
{
    /* Choose the continuous branch containing the measured travel interval.
       In particular raw+135.54 is yaw-224.46, NOT an out-of-range+135.54. */
    return raw>90?raw-360:raw;
}
bool oft_p3_head_center_shift(float yaw,float *shift)
{
    if(!shift||!isfinite(yaw))return false;
    float original=yaw-(-175.55f),selfie=yaw-4.0f;
    float d=fabsf(original)<fabsf(selfie)?original:selfie;
    if(fabsf(d)>8)return false;
    *shift=d;return true;
}
bool oft_p3_gimbal_candidates(const oft_duml_frame_t *f,int16_t values[3])
{
    /* docs/telemetry-findings.md:49-byte layout, yaw16/pitch20/roll22.
       Values stay raw candidates. The former 0/2/4 semantic layout was rejected. */
    if(!f||!values||f->cmd_set!=4||f->cmd_id!=5||f->payload_size!=49)return false;
    const unsigned offsets[]={16,20,22};
    for(unsigned i=0;i<3;i++)values[i]=(int16_t)(f->payload[offsets[i]]|f->payload[offsets[i]+1]<<8);
    return true;
}
