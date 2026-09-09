#include "oft_settings.h"
#include <string.h>
/* Narrow normal-mode settings. OPC9c4e7334 Commands/CameraStatus/CamCap;
 * photo12/13/16/17: o-gs camera command map, validated by GET/capability before
 * any SET. No recording trigger, exposure, encoder/live-view or arbitrary PID.
 */
static const char *keys[]={"camcap_video_format","cam_video_param_v2","cam_lens_state","cam_fov",
    "cam_photo_param_new","camcap_photo_size","camcap_photo_storage_format","camcap_zoom"};
static unsigned le16(const uint8_t *p){return p[0]|((unsigned)p[1]<<8);}
const char *oft_settings_key(unsigned k){return k<OFT_SETTINGS_SUB_COUNT?keys[k]:NULL;}
unsigned oft_settings_fps(unsigned c){static const unsigned rates[]={0,24,25,30,48,50,60,120,240,0,100};return c<sizeof(rates)/sizeof(rates[0])?rates[c]:0;}
const char *oft_settings_resolution(unsigned c)
{
    switch(c){case 0x0a:return "1080p";case 0x10:return "4K";case 0x2d:return "2.7K";
    case 0x42:return "1080p V";case 0x43:return "2.7K V";case 0x6c:return "3K V";
    case 0x69:return "1080 SQ";case 0x6a:return "2160 SQ";case 0x6b:return "3K SQ";default:return NULL;}
}
unsigned oft_settings_zoom_max(unsigned mode,unsigned resolution)
{
    // Official Pocket3 Video bounds. No other-body12x or implicit color/mode hop.
    if(mode==5)return resolution==0x10?200:100;
    if(mode!=1)return 100;
    switch(resolution){case 0x0a:return 400;case 0x2d:return 300;case 0x10:return 200;default:return 100;}
}
bool oft_settings_item(const oft_duml_frame_t *f,oft_setting_item_t *item)
{
    if(!f||!item||f->receiver!=2||(f->sender!=0x28&&f->sender!=1)||f->cmd_set||f->cmd_id!=0x99||f->payload_size<24)return false;
    const uint8_t *p=f->payload;size_t n=f->payload_size;
    if(p[0]!=2||p[1]!=6||p[2]||p[3])return false;
    unsigned name=le16(p+13);if(!name||name>64||23u+name>n)return false;
    unsigned size=le16(p+21+name);if(size>n-23-name||le16(p+11)>n-13)return false;
    for(unsigned k=0;k<OFT_SETTINGS_SUB_COUNT;k++)if(strlen(keys[k])==name&&!memcmp(p+15,keys[k],name)){
        *item=(oft_setting_item_t){keys[k],p+23+name,size,k};return true;
    }
    return false;
}
static bool enum_list(const uint8_t *v,size_t n,uint8_t out[8],uint8_t *count)
{
    // Only explicit compact lists; do not scan arbitrary offsets for plausible IDs.
    if(n<5||v[0]!=1||le16(v+1)+3!=n||!v[3]||v[3]>8)return false;
    unsigned c=v[3],width=(unsigned)(n-4)/c;if((width!=1&&width!=2)||n!=4+c*width)return false;
    for(unsigned i=0;i<c;i++){if(width==2&&v[5+2*i])return false;out[i]=v[4+width*i];}
    *count=c;return true;
}
bool oft_settings_observe(oft_settings_state_t *s,const oft_duml_frame_t *f,int64_t now)
{
    oft_setting_item_t item;if(!s||!oft_settings_item(f,&item))return false;
    const uint8_t *p=item.value;size_t n=item.size;bool ok=false;
    switch(item.key){
    case OFT_SUB_VIDEO_CAP:
        s->pair_count=0;s->video_cap_valid=false;s->reference_pairs=false;
        if(n>=7&&p[0]==1&&le16(p+1)+3==n&&p[3]&&p[3]<=OFT_SETTINGS_PAIRS&&n==4u+3u*p[3]){
            for(unsigned i=0;i<p[3];i++)s->pairs[i]=(oft_video_pair_t){p[4+i*3],p[5+i*3],p[6+i*3]};
            s->pair_count=p[3];s->video_cap_valid=true;ok=true;
        }break;
    case OFT_SUB_VIDEO:if(n>=2){s->resolution=p[0];s->fps=p[1];s->video_valid=true;s->video_us=now;ok=true;}break;
    case OFT_SUB_LENS:if(n>=16){s->lens=le16(p+14);s->lens_valid=s->lens>=217&&s->lens<=868;s->lens_us=now;ok=true;}break;
    case OFT_SUB_FOV:if(n>=4){s->fov=(uint32_t)p[0]|((uint32_t)p[1]<<8)|((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24);ok=true;}break;
    case OFT_SUB_PHOTO:if(n>=5){s->photo_size=p[3];s->photo_aspect=p[4];s->photo_valid=true;s->photo_us=now;ok=true;}break;
    case OFT_SUB_PHOTO_SIZE_CAP:s->photo_size_count=0;ok=enum_list(p,n,s->photo_sizes,&s->photo_size_count);break;
    case OFT_SUB_PHOTO_FILE_CAP:s->photo_file_count=0;ok=enum_list(p,n,s->photo_files,&s->photo_file_count);break;
    default:s->unknown++;return true;
    }
    s->updates++;if(!ok)s->malformed++;return true;
}
bool oft_settings_video_pair(const oft_settings_state_t *s,unsigned res,unsigned fps)
{
    if(!s||!s->video_cap_valid||!oft_settings_resolution(res)||fps<1||fps>6)return false;
    for(unsigned i=0;i<s->pair_count;i++)if(s->pairs[i].resolution==res&&s->pairs[i].fps==fps&&!s->pairs[i].flags)return true;
    return false;
}
bool oft_settings_reference_pairs(oft_settings_state_t *s,unsigned mode)
{
    /* P3 ordinary16:9 Video: explicit DJI-spec combinations, OPC wire codes.
       Used only after this body declined capability publication. Distinct
       provenance; never mix partial advertised rows into a cartesian product. */
    static const oft_video_pair_t documented[]={
        {0x0a,1,0},{0x0a,2,0},{0x0a,3,0},{0x0a,4,0},{0x0a,5,0},{0x0a,6,0},
        {0x2d,1,0},{0x2d,2,0},{0x2d,3,0},{0x2d,4,0},{0x2d,5,0},{0x2d,6,0},
        {0x10,1,0},{0x10,2,0},{0x10,3,0},{0x10,4,0},{0x10,5,0},{0x10,6,0}};
    if(!s||mode!=1||!s->video_valid||s->video_cap_valid)return false;
    bool known=false;for(unsigned i=0;i<sizeof(documented)/sizeof(documented[0]);i++)if(documented[i].resolution==s->resolution&&documented[i].fps==s->fps)known=true;
    if(!known)return false;
    memcpy(s->pairs,documented,sizeof(documented));s->pair_count=sizeof(documented)/sizeof(documented[0]);s->video_cap_valid=true;s->reference_pairs=true;return true;
}
size_t oft_settings_build(oft_setting_action_t c,unsigned a,unsigned b,uint16_t seq,uint8_t *out,size_t cap)
{
    uint8_t p[64]={0};size_t n=0;unsigned receiver=1,set=2,id=0;
    if((unsigned)c<OFT_SETTINGS_SUB_COUNT){
        if(a||b)return 0;
        size_t len=strlen(keys[c]);p[0]=2;p[1]=2;unsigned sub=0x6b00+(unsigned)c;
        /* Owner Pocket3 PCAP confirms the compact layout for current-value
           subscriptions. The longer public cam_status example is NOT a P3
           compatibility fix. No speculative alternate subscription sweep. */
        p[4]=sub;p[5]=sub>>8;p[11]=len+6;p[13]=len;memcpy(p+15,keys[c],len);n=19+len;receiver=0x28;set=0;id=0x99;
    }else switch(c){
    case OFT_SET_VIDEO:if(!oft_settings_resolution(a)||b<1||b>6)return 0;id=0x18;n=5;p[0]=a;p[1]=b;break;
    case OFT_SET_ZOOM:if(a<217||a>868||b)return 0;id=0xb8;n=4;p[0]=0x0a;p[1]=0x4e;p[2]=a;p[3]=a>>8;break;
    case OFT_SET_MODE:if((a!=1&&a!=5)||b)return 0;id=0xe1;n=1;p[0]=a;break;
    case OFT_SET_PHOTO_SIZE:if(a>5||(b!=1&&b!=3))return 0;id=0x12;n=2;p[0]=a;p[1]=b;break;
    case OFT_SET_PHOTO_FILE:if((a!=1&&a!=2)||b)return 0;id=0x16;n=1;p[0]=a;break;
    case OFT_GET_PHOTO_SIZE:if(a||b)return 0;id=0x13;break;
    case OFT_GET_PHOTO_FILE:if(a||b)return 0;id=0x17;break;
    default:return 0;
    }
    return oft_duml_encode(out,cap,2,receiver,seq,0x40,set,id,p,n);
}
bool oft_settings_allowed(const uint8_t *raw,size_t n)
{
    oft_duml_frame_t f;if(!oft_duml_decode(raw,n,&f)||f.sender!=2||f.flags!=0x40)return false;
    uint8_t expected[100];unsigned a=0,b=0;
    for(unsigned k=0;k<OFT_SETTINGS_SUB_COUNT;k++){size_t sz=oft_settings_build(k,0,0,f.sequence,expected,sizeof(expected));if(sz==n&&!memcmp(raw,expected,n))return true;}
    if(f.cmd_set!=2||f.receiver!=1)return false;
    oft_setting_action_t c;
    switch(f.cmd_id){
    case 0x18:if(f.payload_size!=5)return false;c=OFT_SET_VIDEO;a=f.payload[0];b=f.payload[1];break;
    case 0xb8:if(f.payload_size!=4)return false;c=OFT_SET_ZOOM;a=le16(f.payload+2);break;
    case 0xe1:if(f.payload_size!=1)return false;c=OFT_SET_MODE;a=f.payload[0];break;
    case 0x12:if(f.payload_size!=2)return false;c=OFT_SET_PHOTO_SIZE;a=f.payload[0];b=f.payload[1];break;
    case 0x16:if(f.payload_size!=1)return false;c=OFT_SET_PHOTO_FILE;a=f.payload[0];break;
    case 0x13:c=OFT_GET_PHOTO_SIZE;break;case 0x17:c=OFT_GET_PHOTO_FILE;break;
    default:return false;
    }
    size_t sz=oft_settings_build(c,a,b,f.sequence,expected,sizeof(expected));return sz==n&&!memcmp(raw,expected,n);
}
bool oft_settings_can_set(const oft_settings_state_t *s,oft_camera_state_t cam,oft_setting_action_t c,unsigned a,unsigned b,int64_t now)
{
    if(!s||!cam.valid||(cam.recording&&c!=OFT_SET_ZOOM)||cam.transition||cam.playback||now<cam.updated_us||now-cam.updated_us>2000000)return false;
    if(c==OFT_SET_MODE)return (a==1||a==5)&&!b;
    if(c==OFT_SET_VIDEO)return cam.mode==OFT_CAPTURE_VIDEO&&s->video_valid&&oft_settings_video_pair(s,a,b);
    if(c==OFT_SET_ZOOM){unsigned resolution=cam.mode==OFT_CAPTURE_PHOTO?(s->photo_valid&&s->photo_aspect==1?0x10:0):s->resolution;
        return (s->video_valid||cam.mode==OFT_CAPTURE_PHOTO)&&s->lens_valid&&a>=217&&a<=217*oft_settings_zoom_max(cam.raw_mode,resolution)/100&&!b;}
    // P3 GET13 returned size0/default and aspect3. Keep size unchanged; only
    // its supported still aspects1(16:9) /3(square), not arbitrary size buckets.
    if(c==OFT_SET_PHOTO_SIZE&&cam.mode==OFT_CAPTURE_PHOTO&&s->photo_valid&&a==s->photo_size&&(b==1||b==3))return true;
    // Official P3 JPEG/JPEG+DNG profile, gated by successful legacy GET17.
    if(c==OFT_SET_PHOTO_FILE&&cam.mode==OFT_CAPTURE_PHOTO&&s->photo_file_valid&&!b&&(a==1||a==2)&&(s->photo_file==1||s->photo_file==2))return true;
    return false;
}
