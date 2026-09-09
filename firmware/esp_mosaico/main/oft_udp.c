#include "oft_udp.h"
#include "oft_wifi_wire.h"
#include "oft_pocket3.h"
#include "oft_ble.h"
#include "oft_network.h"
#include "oft_motion.h"
#include "oft_video.h"
#include "oft_video_policy.h"
#include "esp_heap_caps.h"
#include "oft_head_target.h"
#include "oft_head_servo.h"
#include "esp_timer.h"
#include "esp_random.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include <errno.h>
#include <fcntl.h>
#include <stdatomic.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

static portMUX_TYPE mux=portMUX_INITIALIZER_UNLOCKED;
static oft_udp_snapshot_t view;
static oft_input_t latest_input;
static atomic_bool stop_requested;
static atomic_bool disconnect_requested;
static atomic_int probe_request=-1;
static atomic_int action_request=-1;
static atomic_int mode_request=-1;
static atomic_bool keyframe_requested;
static atomic_llong shutter_request;
static oft_shutter_action_t shutter_request_action;
static oft_shutter_tx_t shutter;
static int64_t shutter_cooldown;
static bool shutter_write_authorized;
static oft_settings_snapshot_t settings;
static struct {oft_setting_action_t action;unsigned a,b;int64_t at;} setting_request;
static struct {bool active,acknowledged,queried;oft_setting_action_t action;unsigned a,b;uint16_t sequence,verify_sequence;int64_t at;} setting_tx;
static atomic_bool settings_refresh;
static unsigned subscription_cursor;
static int64_t subscription_due;
static int64_t setting_send_due;
static uint16_t subscription_sequences[OFT_SETTINGS_SUB_COUNT];static unsigned rejected_subscriptions;
static unsigned setting_results;static bool setting_result_ok;
static atomic_int setting_test_request=-1;
static struct {bool active,restore_attempted,changed_ok;unsigned kind,a,b,success_base,error_base;oft_setting_action_t action;int64_t deadline,settle;} setting_test;
static bool keyframe_pending;
static uint16_t keyframe_sequence;
static int64_t keyframe_deadline;
static bool requested_head_hold,head_reference_valid,head_hold_blocked;
static int64_t head_input_us;
static unsigned head_orientation_epoch;
static oft_head_target_t head_target;
static atomic_bool head_new_press;
static uint32_t head_generation;
static oft_head_camera_t head_camera;
static oft_head_yaw_tracker_t head_camera_yaw;
static oft_head_result_t head_result;
static unsigned head_log_status=~0u;
static bool action_pending;
static bool action_is_center;
static oft_head_center_anchor_t yaw_anchor;
static unsigned anchor_nonzero_count;
static float yaw_limit_shift;
static uint16_t action_sequence;
static int64_t action_deadline,action_cooldown;
static int64_t disconnect_ble_due;
static int sock=-1;
static oft_wifi_session_t wire;
static oft_duml_stream_t parser;
static uint8_t rx[OFT_WIFI_DATAGRAM_MAX+1];
static uint16_t duml_counter,enable_sequence,heartbeat_sequence;
static uint16_t last_zero_transport_sequence;
static int64_t deadline,next_ack,next_presence,next_heartbeat,last_heartbeat_reply;
static oft_wifi_pending_t pending_heartbeats;
static bool registered81,enabled;
static uint16_t registration_seq[2];static bool seen_registration[2];
static bool probe_running,probe_zeros_pending;
static bool probe_priming;static unsigned prime_count,probe_direction;
static int64_t prime_due,prime_settle;
static bool arm_pending;static unsigned arm_zero_count;static int64_t arm_due,arm_settle;
static int64_t probe_start_us;
typedef struct {int64_t at_us;uint8_t raw[62];} gimbal_sample_t;
static gimbal_sample_t history[8];static unsigned history_count,history_next;
static bool attempted; /* exactly one UDP session per Wi-Fi association */
static oft_control_t control;
static oft_control_t mock_control;
static const char *TAG="oft_udp";
void oft_udp_settings_snapshot(oft_settings_snapshot_t *out)
{portENTER_CRITICAL(&mux);*out=settings;if(view.state!=OFT_UDP_READY)memset(&out->values,0,sizeof(out->values));portEXIT_CRITICAL(&mux);}
void oft_udp_settings_refresh(void){atomic_store(&settings_refresh,true);}
bool oft_udp_settings_test(unsigned kind)
{
    if(kind>6)return false;
    oft_udp_snapshot_t u;oft_udp_snapshot(&u);oft_settings_snapshot_t s;oft_udp_settings_snapshot(&s);
    if(u.state!=OFT_UDP_READY||u.head_held||u.action_busy||u.probe_running||u.control_state==OFT_CONTROL_ACTIVE||s.busy||u.camera.recording)return false;
    int expected=-1;return atomic_compare_exchange_strong(&setting_test_request,&expected,(int)kind);
}
bool oft_udp_setting(oft_setting_action_t c,unsigned a,unsigned b)
{
    int64_t now=esp_timer_get_time();portENTER_CRITICAL(&mux);
    bool ok=view.state==OFT_UDP_READY&&!view.action_busy&&!view.head_held&&!view.probe_running&&!view.shutter_busy&&
        (!settings.busy||c==OFT_SET_ZOOM)&&oft_settings_can_set(&settings.values,view.camera,c,a,b,now);
    if(ok)setting_request=(typeof(setting_request)){c,a,b,now};
    else strlcpy(settings.detail,"Unavailable: mode, busy, or missing camera capability",sizeof(settings.detail));
    portEXIT_CRITICAL(&mux);return ok;
}
static void log_duml(const char *kind,const uint8_t *raw,size_t size)
{
    /* Only the UDP owner calls this; bounded static storage avoids task-stack
       growth and preserves complete action responses including unknown bytes. */
    static char hex[2*OFT_DUML_MAX+1];
    if(size>OFT_DUML_MAX)return;
    for(size_t i=0;i<size;i++)snprintf(hex+i*2,3,"%02x",raw[i]);
    ESP_LOGI(TAG,"%s us=%lld frame=%s",kind,(long long)esp_timer_get_time(),hex);
}

void oft_udp_snapshot(oft_udp_snapshot_t *out){portENTER_CRITICAL(&mux);*out=view;portEXIT_CRITICAL(&mux);}
bool oft_udp_shutter(int64_t at)
{
    portENTER_CRITICAL(&mux);
    oft_shutter_action_t action=oft_capture_select(view.camera,at);
    bool accepted=view.state==OFT_UDP_READY&&!view.shutter_busy&&(!view.settings_busy||action==OFT_SHUTTER_STOP)&&action!=OFT_SHUTTER_NONE&&!atomic_load(&shutter_request);
    if(accepted){shutter_request_action=action;atomic_store(&shutter_request,(long long)at);}
    else strlcpy(view.shutter_detail,"Shutter unavailable: busy or unknown mode",sizeof(view.shutter_detail));
    portEXIT_CRITICAL(&mux);return accepted;
}
bool oft_udp_keyframe(void)
{
    oft_udp_snapshot_t s;oft_udp_snapshot(&s);int64_t now=esp_timer_get_time();
    if(s.state!=OFT_UDP_READY||s.action_busy||s.keyframe_pending||s.keyframe_failures||
       now-s.started_us<5000000||(s.keyframe_sent_us&&now-s.keyframe_sent_us<(int64_t)OFT_VIDEO_MIN_INTERVAL_MS*1000))return false;
    bool expected=false;return atomic_compare_exchange_strong(&keyframe_requested,&expected,true);
}
void oft_udp_cancel_keyframe(void){atomic_store(&keyframe_requested,false);}
void oft_udp_input(const oft_input_t *in){portENTER_CRITICAL(&mux);latest_input=*in;portEXIT_CRITICAL(&mux);}
void oft_udp_stop(void){atomic_store(&stop_requested,true);}
void oft_udp_disconnect(void){atomic_store(&disconnect_requested,true);}
void oft_udp_head_mode(bool head)
{
    portENTER_CRITICAL(&mux);memset(&latest_input,0,sizeof(latest_input));requested_head_hold=false;head_input_us=0;portEXIT_CRITICAL(&mux);
    oft_motion_haptic(false);atomic_store(&mode_request,head?1:0);
}
void oft_udp_head_hold(bool held,bool new_press)
{
    portENTER_CRITICAL(&mux);requested_head_hold=held;head_input_us=esp_timer_get_time();portEXIT_CRITICAL(&mux);
    if(new_press)atomic_store(&head_new_press,true);
    /* Release stops camera motion. A fresh measured soft-limit indication may
       remain while HEAD is selected; mode exit/disconnect always stops haptics. */
}
bool oft_udp_probe(unsigned direction)
{
    if(direction>3)return false;
    oft_udp_snapshot_t s;oft_udp_snapshot(&s);
    if(s.state!=OFT_UDP_READY||s.probe_running||s.live_enabled||!s.have_gimbal)return false;
    int expected=-1;return atomic_compare_exchange_strong(&probe_request,&expected,(int)direction);
}
bool oft_udp_action(bool flip)
{
    oft_udp_snapshot_t s;oft_udp_snapshot(&s);
    if(!s.live_enabled||s.state!=OFT_UDP_READY||s.probe_running||s.action_busy)return false;
    int expected=-1;return atomic_compare_exchange_strong(&action_request,&expected,flip?1:0);
}
static void state(oft_udp_state_t s,const char *detail)
{
    portENTER_CRITICAL(&mux);view.state=s;strlcpy(view.detail,detail,sizeof(view.detail));portEXIT_CRITICAL(&mux);
    ESP_LOGI(TAG,"state=%u %s",s,detail);
}
static bool transmit(const uint8_t *data,size_t n)
{
    if(sock<0||!n)return false;
    ssize_t sent=send(sock,data,n,MSG_DONTWAIT);
    if(sent!=(ssize_t)n)ESP_LOGE(TAG,"send_failed bytes=%u sent=%d errno=%d internal_free=%u largest=%u",
        (unsigned)n,(int)sent,errno,(unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),(unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
    return sent==(ssize_t)n;
}
static bool profile(oft_p3_command_t cmd,int yaw,int pitch,uint16_t *seq_out)
{
    if(cmd>=OFT_P3_PHOTO&&!shutter_write_authorized)return false;
    uint8_t raw[100],packet[128];
    uint16_t seq=oft_wifi_duml_sequence(duml_counter);
    /* Even a caller inside this task cannot emit motion outside the bounded
       probe. Heartbeat only becomes available after registration and enable. */
    bool control_gate=enabled&&(cmd==OFT_P3_CONTROL_HEARTBEAT||probe_priming||probe_running||probe_zeros_pending||OFT_P3_TOUCH_CONTROL_VALIDATED);
    if((cmd==OFT_P3_RECENTER||cmd==OFT_P3_FLIP)&&!OFT_P3_TOUCH_CONTROL_VALIDATED)return false;
    size_t n=oft_p3_build(cmd,seq,yaw,pitch,raw,sizeof(raw));
    if(!n||!oft_p3_allowed(raw,n,false,control_gate))return false;
    if(cmd==OFT_P3_ENABLE)log_duml("video_09_a8_tx",raw,n);
    if(cmd>=OFT_P3_PHOTO)log_duml("physical_shutter_tx",raw,n);
    n=oft_wifi_wrap(&wire,raw,n,packet,sizeof(packet));
    if(!transmit(packet,n))return false;
    duml_counter++;if(seq_out)*seq_out=seq;
    portENTER_CRITICAL(&mux);view.commands++;
    if(cmd==OFT_P3_STICK){
        if(yaw||pitch)view.nonzero_commands++;
        else {view.zero_commands++;view.zero_receipt_confirmed=false;last_zero_transport_sequence=wire.last_sent;}
    }
    portEXIT_CRITICAL(&mux);
    if(cmd==OFT_P3_STICK){
        char hex[201];for(size_t i=0;i<n-20;i++)snprintf(hex+2*i,3,"%02x",raw[i]);
        ESP_LOGI(TAG,"stick us=%lld yaw_offset=%d pitch_offset=%d frame=%s",(long long)esp_timer_get_time(),yaw,pitch,hex);
    }
    if(cmd==OFT_P3_RECENTER||cmd==OFT_P3_FLIP)log_duml(cmd==OFT_P3_RECENTER?"action_recenter":"action_flip",raw,n-20);
    return true;
}
static bool setting_send(oft_setting_action_t c,unsigned a,unsigned b,uint16_t *seq)
{
    uint8_t raw[100],packet[128];uint16_t id=oft_wifi_duml_sequence(duml_counter);
    size_t n=oft_settings_build(c,a,b,id,raw,sizeof(raw));
    if(!n||!oft_settings_allowed(raw,n)||!enabled||view.state!=OFT_UDP_READY)return false;
    log_duml((unsigned)c<OFT_SETTINGS_SUB_COUNT?"settings_subscribe":"settings_tx",raw,n);
    size_t count=oft_wifi_wrap(&wire,raw,n,packet,sizeof(packet));
    if(!transmit(packet,count))return false;
    duml_counter++;if(seq)*seq=id;
    if((unsigned)c<OFT_SETTINGS_SUB_COUNT)subscription_sequences[c]=id;
    portENTER_CRITICAL(&mux);view.commands++;settings.sent++;portEXIT_CRITICAL(&mux);return true;
}
static void setting_complete(bool success,const char *detail)
{
    if(setting_tx.action>=OFT_SET_VIDEO&&setting_tx.action<=OFT_SET_PHOTO_FILE){setting_results++;setting_result_ok=success;}
    setting_tx.active=false;
    portENTER_CRITICAL(&mux);settings.busy=false;view.settings_busy=false;
    if(success)settings.confirmed++;else {settings.errors++;setting_request.at=0;}
    strlcpy(settings.detail,detail,sizeof(settings.detail));portEXIT_CRITICAL(&mux);
    ESP_LOGI(TAG,"settings result=%d action=%u %s",success,setting_tx.action,detail);
}
static void setting_observed(int64_t now)
{
    if(!setting_tx.active||!setting_tx.acknowledged)return;
    const oft_settings_state_t *s=&settings.values;bool same=false;
    switch(setting_tx.action){
    case OFT_SET_VIDEO:same=s->video_valid&&s->video_us>=setting_tx.at&&s->resolution==setting_tx.a&&s->fps==setting_tx.b;break;
    case OFT_SET_ZOOM:same=s->lens_valid&&s->lens_us>=setting_tx.at&&s->lens==setting_tx.a;break;
    case OFT_SET_MODE:same=view.camera.valid&&view.camera.updated_us>=setting_tx.at&&view.camera.raw_mode==setting_tx.a;break;
    case OFT_SET_PHOTO_SIZE:same=s->photo_valid&&s->photo_us>=setting_tx.at&&s->photo_size==setting_tx.a&&s->photo_aspect==setting_tx.b;break;
    case OFT_SET_PHOTO_FILE:same=s->photo_file_valid&&s->photo_file_us>=setting_tx.at&&s->photo_file==setting_tx.a;break;
    default:break;
    }
    if(same)setting_complete(true,"Pocket state confirmed");
    else if(now-setting_tx.at>5000000)setting_complete(false,"No state confirmation; SET not retried");
}
static void settings_received(const oft_duml_frame_t *f,int64_t now)
{
    if(!f->cmd_set&&f->cmd_id==0x99){
        oft_setting_item_t item;bool parsed=oft_settings_item(f,&item),log=true;
        if(parsed&&item.key==OFT_SUB_VIDEO&&item.size>=2)log=!settings.values.video_valid||settings.values.resolution!=item.value[0]||settings.values.fps!=item.value[1];
        if(parsed&&item.key==OFT_SUB_LENS&&item.size>=16)log=!settings.values.lens_valid||settings.values.lens!=(item.value[14]|((unsigned)item.value[15]<<8));
        if(parsed&&item.key==OFT_SUB_FOV&&item.size>=4){uint32_t v=(uint32_t)item.value[0]|((uint32_t)item.value[1]<<8)|((uint32_t)item.value[2]<<16)|((uint32_t)item.value[3]<<24);log=settings.values.fov!=v;}
        // Keep every reply/capability and semantic transition, not a40Hz hex
        // dump of unchanged lens AF/temperature bytes during normal operation.
        if(log)log_duml("settings_push",f->raw,f->raw_size);
        if(f->sender==0x28&&f->receiver==2&&(f->flags==0x80||f->flags==0xc0)&&f->payload_size){
            for(unsigned i=0;i<OFT_SETTINGS_SUB_COUNT;i++)if(f->sequence==subscription_sequences[i]&&f->payload[0]){
                rejected_subscriptions|=1u<<i;ESP_LOGI(TAG,"settings subscription=%s status=%u",oft_settings_key(i),f->payload[0]);
            }
        }
        portENTER_CRITICAL(&mux);oft_settings_observe(&settings.values,f,now);portEXIT_CRITICAL(&mux);
    }
    if(setting_tx.active&&f->sender==1&&f->receiver==2&&f->cmd_set==2&&(f->flags==0x80||f->flags==0xc0)){
        uint8_t raw[100];oft_duml_frame_t wanted;size_t n=oft_settings_build(setting_tx.action,setting_tx.a,setting_tx.b,setting_tx.sequence,raw,sizeof(raw));
        bool expected=n&&oft_duml_decode(raw,n,&wanted)&&f->cmd_id==wanted.cmd_id&&f->sequence==setting_tx.sequence;
        bool verify=setting_tx.queried&&f->sequence==setting_tx.verify_sequence&&
            f->cmd_id==(setting_tx.action==OFT_SET_PHOTO_SIZE?0x13:0x17);
        if(expected||verify){
            log_duml("settings_reply",f->raw,f->raw_size);
            if(!f->payload_size||f->payload[0]){setting_complete(false,"Pocket rejected setting/query");return;}
            bool size_reply=f->cmd_id==0x13,file_reply=f->cmd_id==0x17;
            if(size_reply||file_reply){
                if(f->payload_size!=(size_reply?3u:2u)){setting_complete(false,"Unexpected photo GET layout");return;}
                portENTER_CRITICAL(&mux);
                if(size_reply){settings.values.photo_size=f->payload[1];settings.values.photo_aspect=f->payload[2];settings.values.photo_valid=true;settings.values.photo_us=now;}
                else {settings.values.photo_file=f->payload[1];settings.values.photo_file_valid=true;settings.values.photo_file_us=now;}
                portEXIT_CRITICAL(&mux);
                if(setting_tx.action==OFT_GET_PHOTO_SIZE||setting_tx.action==OFT_GET_PHOTO_FILE){setting_complete(true,"Photo state read");return;}
            }else setting_tx.acknowledged=true;
        }
    }
    setting_observed(now);
}
static void settings_tick(int64_t now)
{
    if(view.state!=OFT_UDP_READY){setting_tx.active=false;portENTER_CRITICAL(&mux);setting_request.at=0;settings.busy=false;view.settings_busy=false;portEXIT_CRITICAL(&mux);return;}
    if(atomic_exchange(&settings_refresh,false)){subscription_cursor=0;subscription_due=now;}
    if((rejected_subscriptions&(1u<<OFT_SUB_VIDEO_CAP))&&view.camera.raw_mode==1){
        portENTER_CRITICAL(&mux);bool added=oft_settings_reference_pairs(&settings.values,view.camera.raw_mode);
        if(added){strlcpy(settings.detail,"P3 documented profile (camera table unavailable)",sizeof(settings.detail));}portEXIT_CRITICAL(&mux);
    }
    if(setting_tx.active){
        if(now-setting_tx.at>5000000){setting_complete(false,"Setting/query timeout; not retried");return;}
        if(setting_tx.acknowledged&&!setting_tx.queried&&(setting_tx.action==OFT_SET_PHOTO_SIZE||setting_tx.action==OFT_SET_PHOTO_FILE)){
            setting_tx.queried=true;
            if(!setting_send(setting_tx.action==OFT_SET_PHOTO_SIZE?OFT_GET_PHOTO_SIZE:OFT_GET_PHOTO_FILE,0,0,&setting_tx.verify_sequence))setting_complete(false,"Photo verification send failed");
        }else if(setting_tx.acknowledged&&!setting_tx.queried&&(setting_tx.action==OFT_SET_VIDEO||setting_tx.action==OFT_SET_ZOOM)){
            setting_tx.queried=true;
            if(!setting_send(setting_tx.action==OFT_SET_VIDEO?OFT_SUB_VIDEO:OFT_SUB_LENS,0,0,NULL))setting_complete(false,"State resubscription failed");
        }
        setting_observed(now);return;
    }
    if(now<setting_send_due)return;
    if(shutter.pending||now<shutter_cooldown||atomic_load(&shutter_request))return;
    portENTER_CRITICAL(&mux);typeof(setting_request) request=setting_request;setting_request.at=0;portEXIT_CRITICAL(&mux);
    if(request.at){
        if(now<request.at||now-request.at>2000000||view.action_busy||view.head_held||view.shutter_busy||
            !oft_settings_can_set(&settings.values,view.camera,request.action,request.a,request.b,now))return;
        uint16_t seq;
        setting_send_due=now+100000; // <=10Hz, one pending SET, latest slider input only
        if(!setting_send(request.action,request.a,request.b,&seq)){setting_complete(false,"Setting send failed");return;}
        setting_tx=(typeof(setting_tx)){.active=true,.action=request.action,.a=request.a,.b=request.b,.sequence=seq,.at=now};
        portENTER_CRITICAL(&mux);settings.busy=true;view.settings_busy=true;strlcpy(settings.detail,"Waiting for Pocket confirmation",sizeof(settings.detail));portEXIT_CRITICAL(&mux);return;
    }
    if(subscription_cursor<OFT_SETTINGS_SUB_COUNT+2&&now>=subscription_due){
        if(subscription_cursor>=OFT_SETTINGS_SUB_COUNT&&view.camera.mode!=OFT_CAPTURE_PHOTO){subscription_cursor=OFT_SETTINGS_SUB_COUNT+2;return;}
        unsigned index=subscription_cursor++;subscription_due=now+250000;
        oft_setting_action_t c=index<OFT_SETTINGS_SUB_COUNT?index:index==OFT_SETTINGS_SUB_COUNT?OFT_GET_PHOTO_SIZE:OFT_GET_PHOTO_FILE;
        uint16_t seq;if(!setting_send(c,0,0,&seq))return;
        if(index>=OFT_SETTINGS_SUB_COUNT){setting_tx=(typeof(setting_tx)){.active=true,.action=c,.sequence=seq,.at=now};portENTER_CRITICAL(&mux);settings.busy=true;view.settings_busy=true;portEXIT_CRITICAL(&mux);}
    }
}
static void settings_test_tick(int64_t now)
{
    int requested=atomic_exchange(&setting_test_request,-1);
    if(requested>=0&&!setting_test.active){
        oft_setting_action_t action;unsigned before_a=0,before_b=0,next_a=0,next_b=0;
        if(requested==0||requested>=5){
            action=OFT_SET_VIDEO;before_a=next_a=settings.values.resolution;before_b=settings.values.fps;
            if(requested>=5){next_a=requested==5?0x0a:0x2d;next_b=before_b;}
            else for(unsigned i=0;i<settings.values.pair_count;i++){oft_video_pair_t p=settings.values.pairs[i];if(p.resolution==before_a&&p.fps!=before_b&&oft_settings_video_pair(&settings.values,p.resolution,p.fps)){next_b=p.fps;break;}}
        }else if(requested==1){
            action=OFT_SET_ZOOM;before_a=settings.values.lens;unsigned max=217*oft_settings_zoom_max(view.camera.raw_mode,settings.values.resolution)/100;
            next_a=before_a+22<=max?before_a+22:before_a>=239?before_a-22:0;
        }else if(requested==2){action=OFT_SET_MODE;before_a=view.camera.raw_mode;next_a=before_a==1?5:1;}
        else if(requested==3){
            action=OFT_SET_PHOTO_FILE;before_a=settings.values.photo_file;
            next_a=before_a==1?2:before_a==2?1:0;
        }else {
            action=OFT_SET_PHOTO_SIZE;before_a=settings.values.photo_size;before_b=next_b=settings.values.photo_aspect;
            next_a=before_a;next_b=before_b==1?3:before_b==3?1:0;
        }
        if(!oft_settings_can_set(&settings.values,view.camera,action,before_a,before_b,now)||
           !oft_udp_setting(action,next_a,next_b)){ESP_LOGI(TAG,"OFT_SETTINGS_TEST kind=%d refused=missing_state_or_capability",requested);return;}
        setting_test=(typeof(setting_test)){.active=true,.kind=(unsigned)requested,.action=action,.a=before_a,.b=before_b,.success_base=setting_results,.deadline=now+25000000};
        ESP_LOGI(TAG,"OFT_SETTINGS_TEST start kind=%u original=%u,%u target=%u,%u",setting_test.kind,before_a,before_b,next_a,next_b);
    }
    if(!setting_test.active)return;
    if(view.state!=OFT_UDP_READY||now>setting_test.deadline){ESP_LOGE(TAG,"OFT_SETTINGS_TEST failed=timeout_or_disconnect restored=0");setting_test.active=false;return;}
    if(settings.busy||setting_request.at)return;
    if(!setting_test.restore_attempted){
        if(setting_results==setting_test.success_base)return;
        if(!setting_test.settle){setting_test.changed_ok=setting_result_ok;setting_test.settle=now+2000000;return;}
        if(now<setting_test.settle)return;
        setting_test.restore_attempted=true;setting_test.success_base=setting_results;
        if(!oft_udp_setting(setting_test.action,setting_test.a,setting_test.b)){ESP_LOGE(TAG,"OFT_SETTINGS_TEST restore_refused=1");setting_test.active=false;}
    }else if(setting_results>setting_test.success_base){
        bool restored=setting_result_ok;
        ESP_LOGI(TAG,"OFT_SETTINGS_TEST complete kind=%u changed=%d restored=%d success=%d",setting_test.kind,setting_test.changed_ok,restored,restored&&setting_test.changed_ok);setting_test.active=false;
    }
}
static bool control_send(int y,int p,void *unused)
{
    (void)unused;bool sent=profile(OFT_P3_STICK,y,p,NULL);
    if(!y&&!p){
        /* Retain the latest status at release too, rather than depending on
           whether a short boundary flag survived until the1Hz main snapshot.
           This runs in the sole writer, never in the receive callback. */
        for(unsigned i=0;i<2;i++)if(view.gimbal_status_size[i]){
            ESP_LOGI(TAG,"status_at_stop source_us=%lld",(long long)view.gimbal_status_us[i]);
            log_duml("gimbal_status_stop",view.gimbal_status_frame[i],view.gimbal_status_size[i]);
        }
    }
    return sent;
}
static bool mock_send(int y,int p,void *unused)
{
    (void)unused;uint8_t raw[32];size_t n=oft_p3_build(OFT_P3_STICK,(uint16_t)view.mock_commands,y,p,raw,sizeof(raw));
    if(!n||!oft_p3_allowed(raw,n,false,true))return false;
    portENTER_CRITICAL(&mux);view.mock_commands++;if(!y&&!p)view.mock_zeros++;portEXIT_CRITICAL(&mux);
    return true; /* codec/policy exercised, never reaches socket */
}
static void close_session(bool fault,const char *detail)
{
    if(probe_priming||probe_running||probe_zeros_pending||control.stop_required){
        probe_zeros_pending=true;
        oft_control_stop(&control,true,esp_timer_get_time(),control_send,NULL);
        /* A probe bypasses normalized mapping, so its physical send is guarded
           by this extra zero path as well as the independent deadline. */
        if(!profile(OFT_P3_STICK,0,0,NULL)){portENTER_CRITICAL(&mux);view.control_failures++;portEXIT_CRITICAL(&mux);fault=true;}
    }
    probe_priming=probe_running=probe_zeros_pending=false;enabled=false;memset(&pending_heartbeats,0,sizeof(pending_heartbeats));
    keyframe_pending=false;atomic_store(&keyframe_requested,false);
    portENTER_CRITICAL(&mux);view.keyframe_pending=false;portEXIT_CRITICAL(&mux);
    oft_control_stop(&mock_control,false,esp_timer_get_time(),mock_send,NULL);
    action_pending=arm_pending=false;yaw_anchor.pending=false;atomic_store(&action_request,-1);
    head_reference_valid=false;head_hold_blocked=true;oft_motion_haptic(false);
    oft_head_target_cancel(&head_target);head_camera.valid=false;head_camera_yaw.valid=false;head_result=(oft_head_result_t){0};
    portENTER_CRITICAL(&mux);view.head_limit_mask=0;portEXIT_CRITICAL(&mux);
    if(sock>=0){close(sock);sock=-1;}
    portENTER_CRITICAL(&mux);memset(&latest_input,0,sizeof(latest_input));requested_head_hold=false;view.head_held=false;view.live_enabled=false;view.action_busy=false;view.probe_running=false;view.control_state=fault?OFT_CONTROL_FAULT:OFT_CONTROL_DISABLED;portEXIT_CRITICAL(&mux);
    atomic_store(&probe_request,-1);state(fault?OFT_UDP_FAULT:OFT_UDP_OFF,detail);
}
static bool reply(const oft_duml_frame_t *f,uint8_t sender,uint8_t set,uint8_t id,uint16_t seq)
{return f->sender==sender&&f->receiver==2&&f->cmd_set==set&&f->cmd_id==id&&f->sequence==seq&&(f->flags==0x80||f->flags==0xc0);}
static void log_gimbal(const gimbal_sample_t *sample,const char *phase)
{
    char hex[125];for(unsigned i=0;i<62;i++)snprintf(hex+i*2,3,"%02x",sample->raw[i]);
    ESP_LOGI(TAG,"gimbal phase=%s us=%lld duml=%s",phase,(long long)sample->at_us,hex);
}
static void frame_received(const oft_duml_frame_t *f,void *unused)
{
    (void)unused;if(view.state==OFT_UDP_FAULT||view.state==OFT_UDP_OFF)return;
    oft_camera_state_t camera;
    if(oft_capture_status(f,esp_timer_get_time(),&camera)){
        bool mode_changed=camera.raw_mode!=view.camera.raw_mode||!view.camera.updated_us;
        bool changed=camera.raw_mode!=view.camera.raw_mode||camera.recording!=view.camera.recording||!view.camera.updated_us;
        portENTER_CRITICAL(&mux);view.camera=camera;portEXIT_CRITICAL(&mux);
        if(mode_changed){portENTER_CRITICAL(&mux);settings.values.video_cap_valid=false;settings.values.video_valid=false;settings.values.lens_valid=false;settings.values.photo_valid=false;settings.values.photo_file_valid=false;settings.values.photo_size_count=0;settings.values.photo_file_count=0;portEXIT_CRITICAL(&mux);atomic_store(&settings_refresh,true);}
        if(changed){ESP_LOGI(TAG,"camera mode=%u known=%d rec=%d flags=%08lx bytes=%u",camera.raw_mode,camera.valid,camera.recording,(unsigned long)camera.flags,(unsigned)f->payload_size);log_duml("camera_status",f->raw,f->raw_size);}
    }
    if(oft_shutter_reply(&shutter,f))log_duml("physical_shutter_reply",f->raw,f->raw_size);
    settings_received(f,esp_timer_get_time());
    uint8_t battery;int16_t candidates[3];bool have_candidates=oft_p3_gimbal_candidates(f,candidates);
    float camera_yaw,camera_pitch;
    if(oft_p3_head_feedback(f,&camera_yaw,&camera_pitch)){
        int64_t at=esp_timer_get_time();float unwrapped;bool seed=!head_camera_yaw.valid;
        if(oft_head_yaw_update(&head_camera_yaw,camera_yaw,at,&unwrapped)){
            if(seed){unwrapped=oft_p3_head_yaw_seed(camera_yaw);head_camera_yaw.continuous=unwrapped;}
            head_camera=(oft_head_camera_t){unwrapped,camera_pitch,at,true};
        }
        else{
            head_camera.valid=false;oft_head_target_cancel(&head_target);head_hold_blocked=true;
        }
    }
    portENTER_CRITICAL(&mux);view.frames++;
    if(f->sender==4&&f->receiver==2&&f->flags==0&&f->cmd_set==4&&
       (f->cmd_id==0x27||f->cmd_id==0x38)&&f->raw_size<=sizeof(view.gimbal_status_frame[0])){
        unsigned i=f->cmd_id==0x27?0:1;
        memcpy(view.gimbal_status_frame[i],f->raw,f->raw_size);
        view.gimbal_status_size[i]=(uint8_t)f->raw_size;view.gimbal_status_us[i]=esp_timer_get_time();
    }
    if(oft_p3_battery(f,&battery)){view.battery_valid=true;view.battery=battery;view.battery_updated_us=esp_timer_get_time();}
    if(have_candidates){
        memcpy(view.gimbal_raw,candidates,sizeof(candidates));view.have_gimbal=true;
        if(f->raw_size==sizeof(view.gimbal_frame)){
            memcpy(view.gimbal_frame,f->raw,sizeof(view.gimbal_frame));view.gimbal_updated_us=esp_timer_get_time();
        }
    }
    portEXIT_CRITICAL(&mux);
    if(have_candidates&&f->raw_size==62){
        gimbal_sample_t *sample=&history[history_next++%8];sample->at_us=esp_timer_get_time();memcpy(sample->raw,f->raw,62);
        if(history_count<8)history_count++;
        if(probe_start_us&&sample->at_us-probe_start_us<2500000)log_gimbal(sample,"pulse_or_after");
    }
    if(f->sender==0x48&&f->receiver==2&&f->cmd_set==0&&(f->cmd_id==0x81||f->cmd_id==0x82)&&f->flags==0x40){
        unsigned index=f->cmd_id-0x81;
        if(seen_registration[index]&&registration_seq[index]==f->sequence)return;
        if(index&&!registered81){close_session(true,"Registration82 before81");return;}
        uint8_t raw[100],packet[128];size_t n=oft_p3_registration_reply(f,raw,sizeof(raw));
        if(!n){close_session(true,"Unexpected registration payload");return;}
        n=oft_wifi_wrap(&wire,raw,n,packet,sizeof(packet));
        if(!transmit(packet,n)){close_session(true,"Registration send failed");return;}
        duml_counter++;seen_registration[index]=true;registration_seq[index]=f->sequence;
        portENTER_CRITICAL(&mux);view.registrations++;portEXIT_CRITICAL(&mux);
        if(!index)registered81=true;
        else if(!view.enable_count){
            portENTER_CRITICAL(&mux);view.enable_count++;portEXIT_CRITICAL(&mux);
            if(!profile(OFT_P3_ENABLE,0,0,&enable_sequence)){close_session(true,"Enable-once send failed");return;}
            state(OFT_UDP_ENABLE,"Waiting for normal-view enable ACK");deadline=esp_timer_get_time()+5000000;
        }
    }
    if(keyframe_pending&&reply(f,0x41,9,0xa8,keyframe_sequence)){
        log_duml("video_09_a8_rx",f->raw,f->raw_size);keyframe_pending=false;
        portENTER_CRITICAL(&mux);view.keyframe_pending=false;
        if(f->payload_size==1&&!f->payload[0])view.keyframe_replies++;else view.keyframe_failures++;
        portEXIT_CRITICAL(&mux);
    }
    if(view.state==OFT_UDP_ENABLE&&reply(f,0x41,9,0xa8,enable_sequence)){
        if(f->payload_size!=1||f->payload[0]){close_session(true,"Normal-view enable rejected");return;}
        enabled=true;deadline=esp_timer_get_time()+5000000;next_heartbeat=0;
        state(OFT_UDP_ENABLE,"Enabled; checking control keepalive");
    }
    if(f->sender==4&&f->cmd_set==4&&f->cmd_id==0x50)
        ESP_LOGI(TAG,"keepalive_rx seq=%u flags=%02x payload_bytes=%u",f->sequence,f->flags,(unsigned)f->payload_size);
    if(reply(f,4,4,0x50,f->sequence)&&oft_wifi_pending_take(&pending_heartbeats,f->sequence)){
        static const uint8_t expected[]={0,1,4,1,0,5,1,1};
        if(f->payload_size!=sizeof(expected)||memcmp(f->payload,expected,sizeof(expected))){close_session(true,"Unexpected control keepalive response");return;}
        last_heartbeat_reply=esp_timer_get_time();
        portENTER_CRITICAL(&mux);view.heartbeat_replies++;portEXIT_CRITICAL(&mux);
        if(view.state==OFT_UDP_ENABLE){
            if(OFT_P3_TOUCH_CONTROL_VALIDATED){
                arm_pending=true;arm_zero_count=0;arm_due=esp_timer_get_time();arm_settle=0;deadline=arm_due+2500000;
                state(OFT_UDP_ARMING,"Preparing controls; release the joystick");
            }else{deadline=0;state(OFT_UDP_READY,"Session ready; motion requires physical test");}
        }
    }
    if(action_pending&&reply(f,4,4,0x4c,action_sequence)){
        log_duml("action_response",f->raw,f->raw_size);
        /* Match the validated Python action policy: status byte0 is success;
           preserve/log additional response bytes rather than assuming length1. */
        if(!f->payload_size||f->payload[0]){close_session(true,"Gimbal action rejected");return;}
        action_pending=false;action_cooldown=esp_timer_get_time()+2000000;
        if(action_is_center){oft_head_center_begin(&yaw_anchor,esp_timer_get_time());anchor_nonzero_count=view.nonzero_commands;}
        ESP_LOGI(TAG,"action ACK matched seq=%u payload_bytes=%u",f->sequence,(unsigned)f->payload_size);
    }
}
static void open_session(const char *ip)
{
    portENTER_CRITICAL(&mux);memset(&settings,0,sizeof(settings));setting_request.at=0;portEXIT_CRITICAL(&mux);
    memset(&setting_tx,0,sizeof(setting_tx));subscription_cursor=0;subscription_due=0;setting_send_due=0;rejected_subscriptions=0;memset(subscription_sequences,0,sizeof(subscription_sequences));atomic_store(&settings_refresh,false);
    shutter=(oft_shutter_tx_t){0};shutter_cooldown=0;atomic_store(&shutter_request,0);
    portENTER_CRITICAL(&mux);view.camera=(oft_camera_state_t){0};view.shutter_busy=false;view.shutter_detail[0]=0;portEXIT_CRITICAL(&mux);
    memset(&wire,0,sizeof(wire));memset(&parser,0,sizeof(parser));oft_control_init(&control);oft_control_init(&mock_control);
    portENTER_CRITICAL(&mux);
    control.blocked_gesture=mock_control.blocked_gesture=latest_input.gesture;
    control.have_blocked_gesture=mock_control.have_blocked_gesture=true;
    portEXIT_CRITICAL(&mux);
    action_pending=arm_pending=false;action_cooldown=0;
    yaw_anchor.pending=false;yaw_limit_shift=0;
    head_camera.valid=false;head_camera_yaw.valid=false;head_result=(oft_head_result_t){0};
    head_reference_valid=false;head_hold_blocked=true;oft_head_target_cancel(&head_target);
    atomic_store(&head_new_press,false);
    duml_counter=0;registered81=enabled=false;memset(seen_registration,0,sizeof(seen_registration));
    memset(&pending_heartbeats,0,sizeof(pending_heartbeats));last_heartbeat_reply=0;probe_priming=probe_running=probe_zeros_pending=false;
    history_count=history_next=0;probe_start_us=0;
    portENTER_CRITICAL(&mux);bool head_mode=view.head_mode;memset(&view,0,sizeof(view));view.head_mode=head_mode;view.started_us=esp_timer_get_time();memset(&latest_input,0,sizeof(latest_input));portEXIT_CRITICAL(&mux);
    uint16_t id=(uint16_t)(esp_random()%65535+1),seed=(uint16_t)(esp_random()&0xfff8);
    oft_wifi_init(&wire,id,seed);
    sock=socket(AF_INET,SOCK_DGRAM,IPPROTO_UDP);
    if(sock<0){close_session(true,"UDP socket allocation failed");return;}
    struct sockaddr_in local={.sin_family=AF_INET,.sin_port=0},peer={.sin_family=AF_INET,.sin_port=htons(9004)};
    if(inet_pton(AF_INET,ip,&local.sin_addr)!=1||inet_pton(AF_INET,"192.168.2.1",&peer.sin_addr)!=1||
        bind(sock,(struct sockaddr*)&local,sizeof(local))||connect(sock,(struct sockaddr*)&peer,sizeof(peer))||fcntl(sock,F_SETFL,O_NONBLOCK)<0){
        close_session(true,"UDP ephemeral bind/connect failed");return;
    }
    socklen_t size=sizeof(local);
    if(getsockname(sock,(struct sockaddr*)&local,&size)){close_session(true,"UDP port read failed");return;}
    portENTER_CRITICAL(&mux);view.local_port=ntohs(local.sin_port);portEXIT_CRITICAL(&mux);
    uint8_t packet[64];size_t n=oft_wifi_handshake(&wire,packet,sizeof(packet));
    if(!transmit(packet,n)){close_session(true,"Handshake send failed");return;}
    state(OFT_UDP_HANDSHAKE,"UDP handshake");deadline=esp_timer_get_time()+3000000;
}
static void receive_packet(const uint8_t *p,size_t n)
{
    oft_wifi_packet_t v;
    if(!oft_wifi_parse(p,n,&v)||v.session!=wire.session){portENTER_CRITICAL(&mux);view.invalid_packets++;portEXIT_CRITICAL(&mux);return;}
    int64_t now=esp_timer_get_time();
    portENTER_CRITICAL(&mux);view.packets++;view.bytes+=n;view.last_packet_us=now;portEXIT_CRITICAL(&mux);
    if(view.state==OFT_UDP_HANDSHAKE){
        if(!oft_wifi_handshake_accepted(&wire,p,n))return;
        state(OFT_UDP_REGISTER,"Registering application");deadline=now+5000000;next_ack=now;next_presence=now+1000000;
        if(!profile(OFT_P3_OPEN,0,0,NULL)||!profile(OFT_P3_PRESENCE,0,0,NULL))close_session(true,"Application open failed");
        return;
    }
    if(v.type==0)return; /* delayed duplicate handshake response */
    if(!oft_wifi_observe(&wire,p,n)){
        portENTER_CRITICAL(&mux);view.ambiguous_status++;portEXIT_CRITICAL(&mux);return;
    }
    if(v.type==1&&view.zero_commands&&(uint16_t)(wire.peer-last_zero_transport_sequence)<0x8000){
        portENTER_CRITICAL(&mux);view.zero_receipt_confirmed=true;portEXIT_CRITICAL(&mux);
    }
    oft_duml_frame_t direct;
    if(oft_wifi_direct_duml(&v,&direct)){
        if(direct.cmd_set==4&&direct.cmd_id==0x50)
            ESP_LOGI(TAG,"direct keepalive wh_type=%u seq=%u",v.type,direct.sequence);
        frame_received(&direct,NULL);return;
    }
    if(v.type==2){portENTER_CRITICAL(&mux);view.media_packets++;portEXIT_CRITICAL(&mux);oft_video_feed(p,n,now);return;}
    if(v.type==1||v.type==3){
        /* WhType03 is a device response, not an operator WhType05 envelope.
           The Python reference resynchronizes over the packet, not offset20. */
        size_t offset=v.type==1?34:8;
        /* Packet-local resync, no media AU allocation; status trailers may
           contain padding and are deliberately not interpreted as commands. */
        memset(&parser,0,sizeof(parser));oft_duml_feed(&parser,p+offset,n-offset,frame_received,NULL);
    }
}
static void timers(int64_t now)
{
    if(view.state<OFT_UDP_REGISTER||view.state==OFT_UDP_FAULT)return;
    if(wire.have_status&&now>=next_ack){
        uint8_t packet[40];size_t n=oft_wifi_ack(&wire,packet,sizeof(packet));
        if(!transmit(packet,n)){close_session(true,"Flow ACK failed");return;}
        portENTER_CRITICAL(&mux);
        if(view.last_ack_us&&now-view.last_ack_us>view.max_ack_gap_us)view.max_ack_gap_us=(uint32_t)(now-view.last_ack_us);
        view.last_ack_us=now;view.acks++;portEXIT_CRITICAL(&mux);
        next_ack+=25000;if(next_ack<=now)next_ack=now+25000;
    }
    if(now>=next_presence){
        next_presence=now+1000000;if(!profile(OFT_P3_PRESENCE,0,0,NULL)){close_session(true,"APP presence failed");return;}
    }
    if(enabled&&now>=next_heartbeat){
        if(!profile(OFT_P3_CONTROL_HEARTBEAT,0,0,&heartbeat_sequence)){close_session(true,"Control keepalive send failed");return;}
        oft_wifi_pending_add(&pending_heartbeats,heartbeat_sequence);next_heartbeat=now+1000000;
        ESP_LOGI(TAG,"keepalive_tx seq=%u transport=%u peer=%u",heartbeat_sequence,wire.last_sent,wire.peer);
        portENTER_CRITICAL(&mux);view.heartbeat_sent++;portEXIT_CRITICAL(&mux);
        if(!last_heartbeat_reply)last_heartbeat_reply=now;
    }
    if(enabled&&now-last_heartbeat_reply>3000000){close_session(true,"Control keepalive timeout");return;}
    if(view.last_packet_us&&now-view.last_packet_us>3000000){close_session(true,"UDP receive timeout");return;}
    if(action_pending&&now>=action_deadline){close_session(true,"Gimbal action ACK timeout");return;}
    if(keyframe_pending&&now>=keyframe_deadline){
        keyframe_pending=false;
        portENTER_CRITICAL(&mux);view.keyframe_pending=false;view.keyframe_failures++;portEXIT_CRITICAL(&mux);
        ESP_LOGW(TAG,"Keyframe ACK timeout; video retries disabled, controls retained");
    }
    if(atomic_exchange(&keyframe_requested,false)&&view.state==OFT_UDP_READY&&!action_pending&&!keyframe_pending&&
       !view.keyframe_failures&&now-view.started_us>=5000000&&(!view.keyframe_sent_us||now-view.keyframe_sent_us>=(int64_t)OFT_VIDEO_MIN_INTERVAL_MS*1000)){
        bool sent=profile(OFT_P3_ENABLE,0,0,&keyframe_sequence);
        portENTER_CRITICAL(&mux);view.keyframe_requests++;view.keyframe_sent_us=now;view.keyframe_pending=sent;
        if(!sent)view.keyframe_failures++;
        portEXIT_CRITICAL(&mux);keyframe_pending=sent;keyframe_deadline=now+3000000;
    }
    if(arm_pending){
        if(arm_zero_count<3&&now>=arm_due){
            if(!profile(OFT_P3_STICK,0,0,NULL)){close_session(true,"Initial center send failed");return;}
            arm_zero_count++;arm_due=now+50000;if(arm_zero_count==3)arm_settle=now+500000;
        }else if(arm_zero_count==3&&now>=arm_settle&&view.zero_receipt_confirmed&&view.have_gimbal){
            arm_pending=false;deadline=0;
            portENTER_CRITICAL(&mux);
            control.blocked_gesture=latest_input.gesture;control.have_blocked_gesture=true;
            memset(&latest_input,0,sizeof(latest_input));control.state=OFT_CONTROL_ARMED;
            view.live_enabled=true;view.control_state=OFT_CONTROL_ARMED;
            portEXIT_CRITICAL(&mux);
            state(OFT_UDP_READY,"LIVE ready / release to stop");
        }
    }
    if(probe_priming){
        if(prime_count<3&&now>=prime_due){
            if(!profile(OFT_P3_STICK,0,0,NULL)){close_session(true,"Probe priming center failed");return;}
            prime_count++;prime_due=now+50000;
            if(prime_count==3)prime_settle=now+500000;
        }else if(prime_count==3&&now>=prime_settle){
            if(!view.zero_receipt_confirmed||history_count<4){close_session(true,"Priming receipt/telemetry missing");return;}
            for(unsigned i=0;i<history_count;i++)log_gimbal(&history[(history_next-history_count+i)%8],"before");
            probe_priming=false;probe_running=true;probe_start_us=esp_timer_get_time();
            int y=probe_direction<2?(probe_direction==0?32:-32):0;
            int p=probe_direction>=2?(probe_direction==2?32:-32):0;
            if(!profile(OFT_P3_STICK,y,p,NULL)){close_session(true,"Probe nonzero send failed");return;}
        }
    }
    if(probe_running&&now-probe_start_us>=200000){
        if(!profile(OFT_P3_STICK,0,0,NULL)){close_session(true,"Probe zero send failed");return;}
        portENTER_CRITICAL(&mux);view.first_zero_delay_us=(uint32_t)(now-probe_start_us);portEXIT_CRITICAL(&mux);
        probe_running=false;probe_zeros_pending=true;
    }
    if(probe_zeros_pending&&now-probe_start_us>=500000){
        if(!profile(OFT_P3_STICK,0,0,NULL)){close_session(true,"Probe redundant zero failed");return;}
        probe_zeros_pending=false;
        portENTER_CRITICAL(&mux);view.probe_running=false;view.probe_done=true;view.control_state=OFT_CONTROL_DISABLED;portEXIT_CRITICAL(&mux);
        ESP_LOGI(TAG,"probe complete; three priming zeros, one nonzero, two stop zeros; requires visual acceptance");
    }
}
static void manager(void *unused)
{
    (void)unused;
    for(;;){
        oft_network_snapshot_t wifi;oft_network_snapshot(&wifi);oft_ble_snapshot_t ble;oft_ble_snapshot(&ble);
        if(view.state!=OFT_UDP_READY){atomic_store(&shutter_request,0);shutter.pending=false;}
        bool gate=wifi.state==OFT_NETWORK_CONNECTED&&ble.state==OFT_BLE_READY&&ble.pairing_status==1;
        if(!gate){
            if(sock>=0)close_session(view.state==OFT_UDP_READY,"Network/BLE unavailable");
            attempted=false;
        }else if(!attempted){attempted=true;open_session(wifi.ip);}
        if(atomic_exchange(&stop_requested,false)){close_session(false,"Control/session stopped");attempted=true;}
        if(atomic_exchange(&disconnect_requested,false)){
            close_session(false,"Stopping before disconnect");attempted=true;
            /* Leave the STA up briefly so queued center datagrams are not
               immediately flushed by esp_wifi_stop. This is not a physical
               stop guarantee if the radio itself has already failed. */
            disconnect_ble_due=esp_timer_get_time()+300000;
        }
        if(disconnect_ble_due&&esp_timer_get_time()>=disconnect_ble_due){disconnect_ble_due=0;oft_ble_disconnect();}
        int mode=atomic_exchange(&mode_request,-1);
        if(mode>=0){
            oft_control_stop(&control,false,esp_timer_get_time(),control_send,NULL);
            head_reference_valid=false;head_hold_blocked=false;oft_motion_haptic(false);
            oft_head_target_cancel(&head_target);head_result=(oft_head_result_t){0};
            portENTER_CRITICAL(&mux);view.head_mode=mode==1;view.head_held=false;view.head_limit_mask=0;memset(&latest_input,0,sizeof(latest_input));portEXIT_CRITICAL(&mux);
            if(control.state==OFT_CONTROL_FAULT)close_session(true,"Cannot stop before mode change");
            else oft_control_init(&control); /* cleared only AFTER successful stop */
        }
        if(yaw_anchor.pending){
            float center,shift;
            if(oft_head_center_sample(&yaw_anchor,head_camera,view.nonzero_commands==anchor_nonzero_count,esp_timer_get_time(),&center)){
                bool aligned=oft_p3_head_center_shift(center,&shift);
                if(aligned)yaw_limit_shift=shift;
                portENTER_CRITICAL(&mux);view.head_yaw_aligned=aligned;view.head_yaw_shift=yaw_limit_shift;portEXIT_CRITICAL(&mux);
                ESP_LOGI(TAG,"yaw_center_anchor us=%lld center=%.3f shift=%.3f accepted=%d",(long long)esp_timer_get_time(),(double)center,(double)yaw_limit_shift,aligned);
            }
        }
        if(view.head_mode){
            oft_motion_snapshot_t motion;oft_motion_snapshot(&motion);
            bool wanted;int64_t at;
            portENTER_CRITICAL(&mux);wanted=requested_head_hold;at=head_input_us;portEXIT_CRITICAL(&mux);
            int64_t head_now=esp_timer_get_time();
            oft_head_bounds_t bounds=oft_p3_head_bounds();
            bounds.yaw_min+=yaw_limit_shift;bounds.yaw_max+=yaw_limit_shift;
            bool in_profile=head_camera.valid&&head_camera.yaw>=bounds.yaw_min-3&&head_camera.yaw<=bounds.yaw_max+3&&
                head_camera.pitch>=bounds.pitch_min-3&&head_camera.pitch<=bounds.pitch_max+3;
            bool ready=motion.imu_ok&&motion.bias_ready&&motion.orientation_ready&&head_now>=motion.sample_us&&head_now-motion.sample_us<100000;
            bool new_press=atomic_exchange(&head_new_press,false);
            if(new_press){
                head_hold_blocked=false;head_reference_valid=false;oft_head_target_init(&head_target);
                head_generation++;
            }
            if(wanted&&(!ready||head_now-at>=250000||yaw_anchor.pending))head_hold_blocked=true;
            if(head_reference_valid&&head_orientation_epoch!=motion.orientation_epoch)head_hold_blocked=true;
            bool held=wanted&&!head_hold_blocked&&ready&&!action_pending;
            if(held&&!head_reference_valid){head_reference_valid=true;head_orientation_epoch=motion.orientation_epoch;}
            if(!held)head_reference_valid=false;
            head_result=oft_head_target_step(&head_target,held,at,motion.orientation,motion.sample_us,
                (view.state==OFT_UDP_READY&&in_profile?head_camera:(oft_head_camera_t){0}),bounds,head_now);
            float yaw=head_result.active?head_target.direction.yaw:0,pitch=head_result.active?head_target.direction.pitch:0;
            bool guard=false; /* No arbitrary per-press15-degree window. */
            if(head_result.release_required){
                oft_head_target_cancel(&head_target);head_hold_blocked=true;head_result.active=false;
            }
            unsigned limits=oft_head_limit_status(head_camera,bounds,head_now,view.state==OFT_UDP_READY&&in_profile&&!action_pending);
            oft_motion_haptic(limits!=0);
            unsigned status=wanted|(held<<1)|(ready<<2)|(head_result.active<<3)|(guard<<4)|(head_hold_blocked<<5)|(limits<<6);
            if(status!=head_log_status||new_press){
                const char *reason=!wanted?"released":!ready?"imu_not_ready":head_now-at>=250000?"input_timeout":
                    action_pending||yaw_anchor.pending?"action_or_center_alignment":head_hold_blocked?"feedback_or_repress_required":limits?"soft_limit":"tracking";
                ESP_LOGI(TAG,"head_event us=%lld generation=%u wanted=%d tracking=%d reason=%s relative=%.2f,%.2f target=%.2f,%.2f camera=%.2f,%.2f input_age_us=%lld imu_age_us=%lld camera_age_us=%lld",
                    (long long)head_now,(unsigned)head_generation,wanted,head_result.active,reason,(double)yaw,(double)pitch,
                    (double)head_result.target_yaw,(double)head_result.target_pitch,(double)head_camera.yaw,(double)head_camera.pitch,
                    (long long)(head_now-at),(long long)(head_now-motion.sample_us),(long long)(head_now-head_camera.timestamp_us));
                head_log_status=status;
            }
            portENTER_CRITICAL(&mux);view.head_held=held;view.head_sensor_ready=ready;view.head_release_required=head_hold_blocked;
            view.head_sensor_calibrated=motion.bias_ready;
            view.head_yaw_held=head_result.active&&head_target.direction.near_vertical;
            view.head_yaw_deg=yaw;view.head_pitch_deg=pitch;view.head_tracking=head_result.active;view.head_trial_guard=guard;
            view.head_limit_mask=limits;
            view.head_camera_yaw=head_camera.yaw;view.head_camera_pitch=head_camera.pitch;
            view.head_target_yaw=head_result.target_yaw;view.head_target_pitch=head_result.target_pitch;portEXIT_CRITICAL(&mux);
        }
        int request=atomic_exchange(&probe_request,-1);
        if(request>=0&&view.state==OFT_UDP_READY&&!probe_priming&&!probe_running&&!probe_zeros_pending){
            probe_priming=true;probe_direction=(unsigned)request;prime_count=0;prime_due=esp_timer_get_time();prime_settle=0;probe_start_us=0;
            portENTER_CRITICAL(&mux);view.probe_running=true;view.probe_done=false;view.control_state=OFT_CONTROL_ACTIVE;portEXIT_CRITICAL(&mux);
        }
        for(unsigned batch=0;batch<64&&sock>=0;batch++){
            ssize_t n=recv(sock,rx,sizeof(rx),MSG_DONTWAIT);
            if(n<0){if(errno!=EAGAIN&&errno!=EWOULDBLOCK)close_session(true,"UDP receive failed");break;}
            receive_packet(rx,(size_t)n);timers(esp_timer_get_time());
        }
        if(sock>=0){
            int64_t now=esp_timer_get_time();timers(now);
            portENTER_CRITICAL(&mux);long long press=atomic_exchange(&shutter_request,0);oft_shutter_action_t pressed_action=shutter_request_action;portEXIT_CRITICAL(&mux);
            if(press&&view.state==OFT_UDP_READY&&!shutter.pending&&now>=shutter_cooldown&&now>=press&&now-press<500000){
                oft_shutter_action_t action=oft_capture_select(view.camera,now);
                if(action!=OFT_SHUTTER_NONE&&action==pressed_action&&(!settings.busy||action==OFT_SHUTTER_STOP)){
                    oft_p3_command_t cmd=action==OFT_SHUTTER_PHOTO?OFT_P3_PHOTO:action==OFT_SHUTTER_START?OFT_P3_RECORD_START:OFT_P3_RECORD_STOP;
                    uint16_t seq;shutter_write_authorized=true;bool sent=profile(cmd,0,0,&seq);shutter_write_authorized=false;
                    if(sent){oft_shutter_begin(&shutter,action,seq,now);shutter_cooldown=now+750000;}
                    portENTER_CRITICAL(&mux);view.shutter_sent+=sent;view.shutter_errors+=!sent;
                    strlcpy(view.shutter_detail,sent?"Shutter sent; waiting for Pocket":"Shutter send failed; not retried",sizeof(view.shutter_detail));portEXIT_CRITICAL(&mux);
                }
            }
            oft_shutter_observe(&shutter,view.camera,now);
            if(shutter.result){
                int result=shutter.result;shutter.result=0;
                const char *detail=result==-1?"Shutter timeout; check Pocket":result<0?"Pocket rejected shutter":shutter.action==OFT_SHUTTER_PHOTO?"Photo command accepted":shutter.action==OFT_SHUTTER_START?"Recording confirmed":"Recording stopped";
                portENTER_CRITICAL(&mux);view.shutter_errors+=result<0;strlcpy(view.shutter_detail,detail,sizeof(view.shutter_detail));portEXIT_CRITICAL(&mux);
                ESP_LOGI(TAG,"shutter result=%d action=%u %s",result,shutter.action,detail);
            }
            if(deadline&&now>=deadline)close_session(true,"UDP startup response timeout");
            if(view.state==OFT_UDP_READY&&!probe_priming&&!probe_running&&!probe_zeros_pending){
                oft_input_t input;portENTER_CRITICAL(&mux);input=latest_input;portEXIT_CRITICAL(&mux);
                if(view.head_mode){
                    if(!oft_head_servo_input(head_result,oft_head_servo_default(),
                        0x80000000u|head_generation,now,&input)){
                        head_hold_blocked=true;oft_head_target_cancel(&head_target);
                        input=(oft_input_t){.timestamp_us=now};
                    }
                }
                oft_control_t *c=OFT_P3_TOUCH_CONTROL_VALIDATED?&control:&mock_control;
                /* Sample time after the cross-core snapshot: a newer GUI
                   event must not look like a future timestamp and false-fault. */
                int64_t input_now=esp_timer_get_time();
                bool link_fresh=view.last_packet_us&&input_now-view.last_packet_us<=300000;
                oft_control_tick(c,&input,!action_pending&&link_fresh,input_now,OFT_P3_TOUCH_CONTROL_VALIDATED?control_send:mock_send,NULL);
                portENTER_CRITICAL(&mux);view.control_state=c->state;portEXIT_CRITICAL(&mux);
                if(OFT_P3_TOUCH_CONTROL_VALIDATED&&c->state==OFT_CONTROL_FAULT)close_session(true,"Input/link watchdog or send fault");
                int action=atomic_exchange(&action_request,-1);
                if(action>=0&&OFT_P3_TOUCH_CONTROL_VALIDATED&&!action_pending&&now>=action_cooldown&&view.state==OFT_UDP_READY){
                    head_reference_valid=false;head_hold_blocked=true;
                    oft_motion_haptic(false);
                    oft_head_target_cancel(&head_target);head_result.active=false;
                    portENTER_CRITICAL(&mux);view.head_held=false;portEXIT_CRITICAL(&mux);
                    oft_control_stop(&control,false,now,control_send,NULL);
                    if(control.state==OFT_CONTROL_FAULT){close_session(true,"Cannot stop before action");continue;}
                    if(!profile(action?OFT_P3_FLIP:OFT_P3_RECENTER,0,0,&action_sequence)){close_session(true,"Action send failed");continue;}
                    action_is_center=action==0;yaw_anchor.pending=false;
                    action_pending=true;action_deadline=now+1500000;action_cooldown=now+2000000;
                    portENTER_CRITICAL(&mux);view.action_count++;portEXIT_CRITICAL(&mux);
                }
            }
        }
        settings_tick(esp_timer_get_time());
        settings_test_tick(esp_timer_get_time());
        portENTER_CRITICAL(&mux);view.stack_free=uxTaskGetStackHighWaterMark(NULL);view.shutter_busy=shutter.pending||esp_timer_get_time()<shutter_cooldown;portEXIT_CRITICAL(&mux);
        portENTER_CRITICAL(&mux);view.action_busy=action_pending||yaw_anchor.pending||esp_timer_get_time()<action_cooldown;portEXIT_CRITICAL(&mux);
        if(sock>=0){
            /* Wake for packets immediately; avoid a fixed 10ms accumulation
               window in the default six-datagram lwIP inbox. */
            int64_t wait_us=10000,now=esp_timer_get_time();
            if(wire.have_status&&next_ack>now&&next_ack-now<wait_us)wait_us=next_ack-now;
            fd_set reads;FD_ZERO(&reads);FD_SET(sock,&reads);
            struct timeval timeout={.tv_sec=0,.tv_usec=(long)wait_us};
            if(select(sock+1,&reads,NULL,NULL,&timeout)<0&&errno!=EINTR)close_session(true,"UDP wait failed");
        }else vTaskDelay(1);
    }
}
esp_err_t oft_udp_start(void)
{return xTaskCreate(manager,"oft_udp",6144,NULL,5,NULL)==pdPASS?ESP_OK:ESP_ERR_NO_MEM;}
