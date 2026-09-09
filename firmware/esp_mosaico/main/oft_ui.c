#include "oft_ui.h"
#include "oft_ble.h"
#include "oft_network.h"
#include "oft_udp.h"
#include "oft_video.h"
#include "oft_board.h"
#include "oft_motion.h"
#include "oft_touch.h"
#include "oft_ui_geometry.h"
#include "oft_peripherals.h"
#include <stdatomic.h>
#include "esp_heap_caps.h"
#include "lvgl.h"
#include "src/misc/cache/instance/lv_image_cache.h"
#include "esp_timer.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

static lv_obj_t *knob,*values,*status_label,*connection_label,*stick_area,*choices_panel,*choice_buttons[OFT_BLE_CHOICES],*choice_labels[OFT_BLE_CHOICES];
static lv_obj_t *action_buttons[2],*action_labels[2];static bool live_control;
static lv_obj_t *mode_buttons[2],*head_button,*wheels[5];
static oft_menu_side_t menu_side;
static oft_swipe_t edge_swipe;
static bool pointer_down,consume_contact;
typedef struct {lv_obj_t *group,*body,*fill,*tip,*text;} battery_icon_t;
static battery_icon_t local_battery,pocket_battery;
static void battery_update(battery_icon_t *icon,const char *name,bool valid,unsigned percent,unsigned mv);
static atomic_bool menu_test_requested;
static unsigned menu_test_step;static int64_t menu_test_due;
static lv_obj_t *resolution_arc,*rate_arc,*format_label,*format_note,*camera_modes[2],*zoom_slider,*zoom_label,*zoom_note;
static uint8_t resolution_codes[16],rate_codes[12];static unsigned resolution_count,rate_count;
static bool setting_ui_sync;
static oft_settings_snapshot_t setting_view;
static lv_obj_t *limit_banner,*limit_text;
static lv_obj_t *preview_image,*preview_note;static lv_image_dsc_t preview_descriptor;
static bool preview_assigned,preview_presented;
static uint16_t *preview_pixels;static uint32_t preview_generation;
static float yaw,pitch;
static bool active;
static uint32_t gesture;
static int64_t last_log;
static lv_indev_t *shadow_indev;
static atomic_bool shadow_requested,shadow_running;
static int64_t shadow_end,shadow_last;
static unsigned shadow_press,shadow_release,shadow_cancel,shadow_gap,shadow_render;
static unsigned shadow_epoch,shadow_imu_gaps,shadow_imu_stale,shadow_nonzero,shadow_frames;
static bool shadow_aborted;
static void publish_head_region(void)
{
    if(!head_button)return;
    oft_udp_snapshot_t u;oft_udp_snapshot(&u);lv_area_t a;lv_obj_get_coords(head_button,&a);
    oft_touch_head_region(a.x1,a.y1,a.x2,a.y2,menu_side==OFT_MENU_NONE&&u.head_mode&&
        u.state==OFT_UDP_READY&&!u.action_busy&&lv_obj_is_visible(head_button)&&!lv_obj_has_state(head_button,LV_STATE_DISABLED));
}
static void stop_inputs(void)
{
    yaw=pitch=0;active=false;
    oft_input_t in={.gesture=gesture,.timestamp_us=esp_timer_get_time()};oft_udp_input(&in);
    oft_udp_head_hold(false,false);oft_touch_head_region(0,0,0,0,false);
    oft_motion_head_ack(false);
}
static void menu_set(oft_menu_side_t side)
{
    stop_inputs();menu_side=side;
    if(!wheels[1])return;
    for(unsigned i=1;i<=4;i++){
        if(i==(unsigned)side){lv_obj_remove_flag(wheels[i],LV_OBJ_FLAG_HIDDEN);lv_obj_move_foreground(wheels[i]);}
        else lv_obj_add_flag(wheels[i],LV_OBJ_FLAG_HIDDEN);
    }
    oft_udp_snapshot_t u;oft_udp_snapshot(&u);
    if(u.head_mode&&side==OFT_MENU_NONE)lv_obj_remove_flag(head_button,LV_OBJ_FLAG_HIDDEN);else lv_obj_add_flag(head_button,LV_OBJ_FLAG_HIDDEN);
    if(!u.head_mode&&side==OFT_MENU_TOP)lv_obj_remove_flag(stick_area,LV_OBJ_FLAG_HIDDEN);else lv_obj_add_flag(stick_area,LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(local_battery.group);lv_obj_move_foreground(pocket_battery.group);lv_obj_move_foreground(limit_banner);
    lv_obj_update_layout(head_button);publish_head_region();
    printf("OFT_MENU side=%u radius=240 opacity=153 inputs_cancelled=1\n",side);
}
bool oft_ui_pointer(int x,int y,bool down,bool valid)
{
    if(!wheels[1])return false;
    oft_menu_side_t open;bool edge=oft_swipe_step(&edge_swipe,x,y,down,valid,&open);
    bool new_press=down&&!pointer_down;
    if(new_press&&menu_test_step){menu_test_step=0;printf("OFT_MENU_TEST aborted=physical_input\n");}
    if(new_press&&menu_side!=OFT_MENU_NONE&&!oft_menu_inside(menu_side,x,y)){menu_set(OFT_MENU_NONE);consume_contact=true;}
    bool consume=edge||consume_contact;
    if(open!=OFT_MENU_NONE)menu_set(open);
    if(!down||!valid){consume_contact=false;pointer_down=false;}else pointer_down=true;
    return consume;
}
bool oft_ui_menu_test(void)
{
    oft_udp_snapshot_t u;oft_udp_snapshot(&u);
    if(u.head_held||active||atomic_load(&shadow_running))return false;
    bool expected=false;return atomic_compare_exchange_strong(&menu_test_requested,&expected,true);
}
static void menu_test_poll(void)
{
    int64_t now=esp_timer_get_time();
    if(atomic_exchange(&menu_test_requested,false)){menu_test_step=1;menu_test_due=now;}
    if(!menu_test_step||now<menu_test_due)return;
    if(menu_test_step<=4){
        unsigned s=menu_test_step;menu_set((oft_menu_side_t)s);int cx,cy;oft_menu_center((oft_menu_side_t)s,&cx,&cy);
        lv_obj_update_layout(wheels[s]);
        bool pass=lv_obj_get_width(wheels[s])==480&&lv_obj_get_height(wheels[s])==480&&lv_obj_get_x(wheels[s])==cx-240&&lv_obj_get_y(wheels[s])==cy-240&&lv_obj_get_style_bg_opa(wheels[s],0)==153;
        printf("OFT_MENU_TEST side=%u geometry_pass=%d placeholder=0\n",s,pass);
        menu_test_step++;menu_test_due=now+1000000;
    }else{menu_set(OFT_MENU_NONE);menu_test_step=0;printf("OFT_MENU_TEST complete=1 camera_actions=0\n");}
}
static bool shadow_event(lv_event_t *e)
{return shadow_indev&&(lv_indev_active()==shadow_indev||lv_event_get_param(e)==shadow_indev);}
static void setting_mode_event(lv_event_t *e)
{
    if(shadow_event(e)||setting_ui_sync)return;
    stop_inputs();oft_udp_setting(OFT_SET_MODE,(unsigned)(uintptr_t)lv_event_get_user_data(e),0);
}
static void format_event(lv_event_t *e)
{
    if(shadow_event(e)||setting_ui_sync)return;
    if(lv_event_get_code(e)!=LV_EVENT_RELEASED)return;
    unsigned ri=(unsigned)lv_arc_get_value(resolution_arc),fi=(unsigned)lv_arc_get_value(rate_arc);
    oft_udp_snapshot_t u;oft_udp_snapshot(&u);oft_settings_snapshot_t v;oft_udp_settings_snapshot(&v);
    if(u.camera.mode==OFT_CAPTURE_VIDEO){
        if(ri>=resolution_count||fi>=rate_count)return;
        unsigned res=resolution_codes[ri],fps=rate_codes[fi];
        if(!oft_settings_video_pair(&v.values,res,fps)){
            fps=0;for(unsigned i=0;i<v.values.pair_count;i++)if(oft_settings_video_pair(&v.values,res,v.values.pairs[i].fps)){fps=v.values.pairs[i].fps;break;}
        }
        if(fps)oft_udp_setting(OFT_SET_VIDEO,res,fps);
    }else if(u.camera.mode==OFT_CAPTURE_PHOTO){
        if(lv_event_get_target_obj(e)==resolution_arc&&ri<resolution_count)oft_udp_setting(OFT_SET_PHOTO_SIZE,v.values.photo_size,resolution_codes[ri]);
        else if(lv_event_get_target_obj(e)==rate_arc&&fi<rate_count)oft_udp_setting(OFT_SET_PHOTO_FILE,rate_codes[fi],0);
    }
}
static void zoom_event(lv_event_t *e)
{
    if(shadow_event(e)||setting_ui_sync)return;
    int value=lv_slider_get_value(zoom_slider);
    if(lv_event_get_code(e)==LV_EVENT_VALUE_CHANGED||lv_event_get_code(e)==LV_EVENT_RELEASED)oft_udp_setting(OFT_SET_ZOOM,(unsigned)value,0);
}
static void setting_widget_enable(lv_obj_t *o,bool enabled)
{if(enabled)lv_obj_remove_state(o,LV_STATE_DISABLED);else lv_obj_add_state(o,LV_STATE_DISABLED);}
static void settings_ui_update(oft_udp_snapshot_t u)
{
    if(!resolution_arc)return;
    oft_udp_settings_snapshot(&setting_view);const oft_settings_state_t *s=&setting_view.values;
    bool dragging=lv_obj_has_state(resolution_arc,LV_STATE_PRESSED)||lv_obj_has_state(rate_arc,LV_STATE_PRESSED);
    setting_ui_sync=true;
    if(!dragging&&!setting_view.busy){
        resolution_count=rate_count=0;unsigned ri=0,fi=0;
        if(u.camera.mode==OFT_CAPTURE_VIDEO){
            for(unsigned i=0;i<s->pair_count;i++)if(oft_settings_video_pair(s,s->pairs[i].resolution,s->pairs[i].fps)){
                unsigned j;for(j=0;j<resolution_count;j++)if(resolution_codes[j]==s->pairs[i].resolution)break;
                if(j==resolution_count&&resolution_count<16)resolution_codes[resolution_count++]=s->pairs[i].resolution;
            }
            for(unsigned i=0;i<resolution_count;i++)if(resolution_codes[i]==s->resolution)ri=i;
            if(resolution_count)for(unsigned code=1;code<=6;code++)if(oft_settings_video_pair(s,resolution_codes[ri],code)){if(code==s->fps)fi=rate_count;rate_codes[rate_count++]=code;}
        }else if(u.camera.mode==OFT_CAPTURE_PHOTO){
            if(s->photo_valid&&(s->photo_aspect==1||s->photo_aspect==3)){resolution_codes[0]=1;resolution_codes[1]=3;resolution_count=2;ri=s->photo_aspect==3;}
            if(s->photo_file_valid&&(s->photo_file==1||s->photo_file==2)){rate_codes[0]=1;rate_codes[1]=2;rate_count=2;fi=s->photo_file==2;}
        }
        lv_arc_set_range(resolution_arc,0,resolution_count>1?(int)resolution_count-1:1);lv_arc_set_value(resolution_arc,(int)ri);
        lv_arc_set_range(rate_arc,0,rate_count>1?(int)rate_count-1:1);lv_arc_set_value(rate_arc,(int)fi);
    }
    bool can=u.state==OFT_UDP_READY&&u.camera.valid&&!u.camera.recording&&!setting_view.busy&&!u.shutter_busy;
    setting_widget_enable(resolution_arc,can&&resolution_count>1);
    setting_widget_enable(rate_arc,can&&rate_count>1);
    for(unsigned i=0;i<2;i++){
        setting_widget_enable(camera_modes[i],can);
        lv_obj_set_style_bg_color(camera_modes[i],lv_color_hex(u.camera.raw_mode==(i?5:1)?0x287e9d:0x203347),0);
    }
    if(u.camera.mode==OFT_CAPTURE_VIDEO){const char *r=oft_settings_resolution(s->resolution);lv_label_set_text_fmt(format_label,"%s\n%u fps",s->video_valid&&r?r:"--",oft_settings_fps(s->fps));}
    else if(u.camera.mode==OFT_CAPTURE_PHOTO)lv_label_set_text_fmt(format_label,"%s\n%s",s->photo_valid?(s->photo_aspect==1?"16:9 8.3M":s->photo_aspect==3?"1:1 9.4M":"--"):"--",s->photo_file_valid?(s->photo_file==1?"JPEG":s->photo_file==2?"RAW+JPG":"?"):"--");
    else lv_label_set_text(format_label,"Mode ?");
    lv_label_set_text(format_note,setting_view.busy?"Waiting for Pocket":!resolution_count||!rate_count?"Capability unavailable":u.camera.recording?"Locked while recording":"Outer: size\nInner: fps / file type");
    unsigned zoom_res=u.camera.mode==OFT_CAPTURE_PHOTO?(s->photo_valid&&s->photo_aspect==1?0x10:0):s->resolution;
    unsigned max=217*oft_settings_zoom_max(u.camera.raw_mode,zoom_res)/100;
    bool zoom=u.state==OFT_UDP_READY&&u.camera.valid&&(s->video_valid||u.camera.mode==OFT_CAPTURE_PHOTO)&&s->lens_valid&&max>217;
    setting_widget_enable(zoom_slider,zoom);
    if(!lv_obj_has_state(zoom_slider,LV_STATE_PRESSED)){lv_slider_set_range(zoom_slider,217,max>217?(int)max:218);if(s->lens_valid)lv_slider_set_value(zoom_slider,s->lens,LV_ANIM_OFF);}
    if(s->lens_valid){unsigned factor=(s->lens*100u+108u)/217u;lv_label_set_text_fmt(zoom_label,"%u.%02ux",factor/100,factor%100);}else lv_label_set_text(zoom_label,"Zoom --");
    lv_label_set_text(zoom_note,setting_view.busy?"Applying...":zoom?"Digital zoom":"Unavailable / waiting for lens state");
    setting_ui_sync=false;
}
bool oft_ui_hold_test(void)
{
    oft_udp_snapshot_t u;oft_udp_snapshot(&u);
    if(u.state!=OFT_UDP_READY||!u.head_mode||u.head_held||u.action_busy||atomic_load(&shadow_running))return false;
    bool expected=false;return atomic_compare_exchange_strong(&shadow_requested,&expected,true);
}
static void shadow_read(lv_indev_t *indev,lv_indev_data_t *data)
{
    (void)indev;int64_t now=esp_timer_get_time();
    if(atomic_load(&shadow_running)){
        oft_udp_snapshot_t u;oft_udp_snapshot(&u);
        if(!u.head_mode||u.state!=OFT_UDP_READY||u.action_busy)shadow_aborted=true;
    }
    lv_area_t a;lv_obj_get_coords(head_button,&a);data->point=(lv_point_t){(a.x1+a.x2)/2,(a.y1+a.y2)/2};
    data->state=atomic_load(&shadow_running)&&!shadow_aborted&&now<shadow_end?LV_INDEV_STATE_PRESSED:LV_INDEV_STATE_RELEASED;
    if(data->state==LV_INDEV_STATE_PRESSED){
        oft_motion_snapshot_t m;oft_motion_snapshot(&m);
        if(m.orientation_epoch!=shadow_epoch){shadow_imu_gaps++;shadow_epoch=m.orientation_epoch;}
        if(!m.orientation_ready||now-m.sample_us>=100000)shadow_imu_stale++;
        oft_display_stats_t d=oft_board_display_stats();if(d.render_last_us>shadow_render)shadow_render=d.render_last_us;
    }
}
static void shadow_poll(void)
{
    int64_t now=esp_timer_get_time();
      if(atomic_exchange(&shadow_requested,false)){
          menu_set(OFT_MENU_NONE);
        if(lv_obj_has_state(head_button,LV_STATE_PRESSED)||lv_obj_has_state(head_button,LV_STATE_DISABLED)||lv_obj_has_flag(head_button,LV_OBJ_FLAG_HIDDEN)){
            printf("OFT_UI_HOLD_TEST refused=busy\n");return;
        }
        oft_udp_snapshot_t u;oft_udp_snapshot(&u);oft_video_snapshot_t v;oft_video_snapshot(&v);
        oft_motion_snapshot_t m;oft_motion_snapshot(&m);
        shadow_press=shadow_release=shadow_cancel=shadow_gap=shadow_render=0;
        shadow_imu_gaps=shadow_imu_stale=0;shadow_last=0;shadow_aborted=false;
        shadow_epoch=m.orientation_epoch;shadow_nonzero=u.nonzero_commands;shadow_frames=v.presented_frames;
        shadow_end=now+30000000;atomic_store(&shadow_running,true);oft_touch_shadow_start(shadow_end);
        lv_indev_set_mode(shadow_indev,LV_INDEV_MODE_TIMER);
        lv_timer_set_period(lv_indev_get_read_timer(shadow_indev),20);
        lv_timer_resume(lv_indev_get_read_timer(shadow_indev));
        printf("OFT_UI_HOLD_TEST start=1 shadow_routes_to_camera=0 duration_ms=30000\n");
    }
    if(atomic_load(&shadow_running)&&(now>shadow_end+1000000||shadow_aborted)){
        if(lv_indev_get_state(shadow_indev)!=LV_INDEV_STATE_RELEASED)return;
        oft_udp_snapshot_t u;oft_udp_snapshot(&u);oft_video_snapshot_t v;oft_video_snapshot(&v);
        unsigned images=v.presented_frames-shadow_frames,writes=u.nonzero_commands-shadow_nonzero;
        oft_touch_stats_t touch;oft_touch_stats(&touch);
        bool passed=!shadow_aborted&&shadow_press==1&&shadow_release==1&&!shadow_cancel&&shadow_gap<250000&&images>=5&&!writes&&!shadow_imu_gaps&&!shadow_imu_stale&&
            touch.shadow_presses==1&&touch.shadow_releases==1&&!touch.shadow_early_stops&&touch.shadow_max_gap_us<100000;
        printf("OFT_UI_HOLD_TEST passed=%d press=%u release=%u cancel=%u max_gap_us=%u render_max_us=%u imu_gaps=%u imu_stale_reads=%u images=%u nonzero_delta=%u aborted=%d\n",passed,shadow_press,shadow_release,shadow_cancel,shadow_gap,shadow_render,shadow_imu_gaps,shadow_imu_stale,images,writes,shadow_aborted);
        printf("OFT_TOUCH_SHADOW press=%u release=%u early_stops=%u max_gap_us=%u\n",(unsigned)touch.shadow_presses,(unsigned)touch.shadow_releases,(unsigned)touch.shadow_early_stops,(unsigned)touch.shadow_max_gap_us);
        atomic_store(&shadow_running,false);lv_indev_set_mode(shadow_indev,LV_INDEV_MODE_EVENT);
    }
}
static void stick_event(lv_event_t *event)
{
    if(shadow_event(event))return;
    const lv_event_code_t code=lv_event_get_code(event);
    if(code==LV_EVENT_RELEASED||code==LV_EVENT_PRESS_LOST){yaw=pitch=0;active=false;}
    else if(code==LV_EVENT_PRESSED||code==LV_EVENT_PRESSING){
        if(code==LV_EVENT_PRESSED)gesture++;
        lv_point_t p;lv_indev_get_point(lv_indev_active(),&p);
        lv_area_t area;lv_obj_get_coords(lv_event_get_target_obj(event),&area);
        float radius=(area.x2-area.x1+1)/2.0f-10.0f;
        yaw=(p.x-(area.x1+area.x2)/2.0f)/radius;
        pitch=((area.y1+area.y2)/2.0f-p.y)/radius;
        float r=sqrtf(yaw*yaw+pitch*pitch);
        if(r>1){yaw/=r;pitch/=r;}
        if(r<.12f)yaw=pitch=0;
        active=true;
    }else return;
    float travel=(lv_obj_get_width(stick_area)-54)/2.0f;
    lv_obj_align(knob,LV_ALIGN_CENTER,(int)(yaw*travel),(int)(-pitch*travel));
    lv_label_set_text_fmt(values,"%s  yaw %+d%%  pitch %+d%%",live_control?"LIVE":"MOCK",(int)(yaw*100),(int)(pitch*100));
    int64_t now=esp_timer_get_time();
    oft_input_t input={.yaw=yaw,.pitch=pitch,.active=active,.gesture=gesture,.timestamp_us=now};oft_udp_input(&input);
    if(!active||now-last_log>100000){
        printf("OFT_TOUCH active=%d yaw=%.3f pitch=%.3f us=%lld\n",active,(double)yaw,(double)pitch,(long long)now);
        last_log=now;
    }
}
static void button_event(lv_event_t *e)
{
    if(shadow_event(e))return;
    if(atomic_load(&shadow_running))shadow_aborted=true;
    const char *name=lv_event_get_user_data(e);
    if(strcmp(name,"Disconnect")==0)oft_udp_disconnect();
    else if(strcmp(name,"Scan")==0){if(live_control)oft_udp_action(false);else oft_ble_rescan();}
    else if(strcmp(name,"180 deg")==0)oft_udp_action(true);
}
static void select_event(lv_event_t *e){if(shadow_event(e))return;oft_ble_select((unsigned)(uintptr_t)lv_event_get_user_data(e));}
static void mode_event(lv_event_t *e){if(shadow_event(e))return;if(atomic_load(&shadow_running))shadow_aborted=true;stop_inputs();oft_udp_head_mode((uintptr_t)lv_event_get_user_data(e)==1);if((uintptr_t)lv_event_get_user_data(e)==1)menu_set(OFT_MENU_NONE);}
static void head_event(lv_event_t *e)
{
    lv_event_code_t code=lv_event_get_code(e);
    if(shadow_event(e)){
        int64_t now=esp_timer_get_time();
        if(code==LV_EVENT_PRESSED){shadow_press++;shadow_last=now;}
        if(code==LV_EVENT_PRESSING){if(shadow_last&&(unsigned)(now-shadow_last)>shadow_gap)shadow_gap=(unsigned)(now-shadow_last);shadow_last=now;}
        if(code==LV_EVENT_RELEASED)shadow_release++;
        if(code==LV_EVENT_PRESS_LOST)shadow_cancel++;
        return; // Hard boundary: no simulated input reaches oft_udp_head_hold.
    }
    if(atomic_load(&shadow_running)&&code==LV_EVENT_PRESSED)shadow_aborted=true;
    if(code==LV_EVENT_PRESSED||code==LV_EVENT_RELEASED||code==LV_EVENT_PRESS_LOST)
        printf("OFT_HEAD_TOUCH us=%lld event=%s\n",(long long)esp_timer_get_time(),
            code==LV_EVENT_PRESSED?"press":code==LV_EVENT_RELEASED?"release":"cancel");
    /* Physical sampling owns the head lease. GUI callbacks are visual/logging
       only, so repaint or pointer cancellation cannot renew/cancel stale input. */
}
static void refresh(lv_timer_t *timer)
{
    (void)timer;oft_ble_snapshot_t v;oft_ble_snapshot(&v);
    shadow_poll();
    menu_test_poll();
    if(v.battery_valid&&esp_timer_get_time()-v.battery_updated_us>=5000000)v.battery_valid=false;
    lv_label_set_text(connection_label,v.detail[0]?v.detail:"Testing protocol before BLE startup");
    oft_network_snapshot_t wifi;oft_network_snapshot(&wifi);
    if(v.state==OFT_BLE_READY){
        if(wifi.state==OFT_NETWORK_CONNECTED)lv_label_set_text(connection_label,"BLE + Pocket hotspot connected / MOCK");
        else if(wifi.state==OFT_NETWORK_FAULT)lv_label_set_text_fmt(connection_label,"Hotspot failed (reason %d); Disconnect / Scan",wifi.last_reason);
    }
    oft_udp_snapshot_t udp;oft_udp_snapshot(&udp);
    settings_ui_update(udp);
    oft_video_visible(udp.head_mode);
    oft_video_snapshot_t picture;oft_video_snapshot(&picture);
    if(picture.ready&&udp.head_mode&&!preview_pixels)
        preview_pixels=heap_caps_malloc(352*198*2,MALLOC_CAP_SPIRAM);
    int64_t picture_source_us=0;
    uint32_t frame=udp.head_mode?oft_video_copy_timed(preview_pixels,352*198*2,preview_generation,&picture_source_us):0;
    if(frame){
        preview_descriptor.header.magic=LV_IMAGE_HEADER_MAGIC;preview_descriptor.header.cf=LV_COLOR_FORMAT_RGB565;
        preview_descriptor.header.w=picture.display_width;preview_descriptor.header.h=picture.display_height;
        preview_descriptor.header.stride=picture.display_width*2;preview_descriptor.data_size=picture.display_width*picture.display_height*2;
        preview_descriptor.data=(const uint8_t*)preview_pixels;
        lv_image_cache_drop(&preview_descriptor);
        if(!preview_assigned)lv_image_set_src(preview_image,&preview_descriptor);
        lv_obj_invalidate(preview_image);preview_assigned=true;preview_generation=frame;oft_video_presented();
        printf("OFT_VIDEO_PRESENT generation=%lu source_us=%lld ui_us=%lld\n",(unsigned long)frame,(long long)picture_source_us,(long long)esp_timer_get_time());
    }
    if(udp.head_mode&&preview_assigned){
        lv_obj_remove_flag(preview_image,LV_OBJ_FLAG_HIDDEN);lv_obj_add_flag(preview_note,LV_OBJ_FLAG_HIDDEN);
        if(esp_timer_get_time()-picture.last_frame_us>5000000){
            lv_label_set_text(preview_note,picture.refresh_failures?"IMAGE STALE - VIDEO FAULT":
                picture.sparse_live?"IMAGE STALE - RECOVERING":"IMAGE STALE - PAUSED");
            lv_obj_set_style_bg_color(preview_note,lv_color_hex(0x202020),0);
            lv_obj_set_style_bg_opa(preview_note,LV_OPA_90,0);
            lv_obj_set_style_text_color(preview_note,lv_color_hex(0xffbb55),0);
            lv_obj_remove_flag(preview_note,LV_OBJ_FLAG_HIDDEN);
        }
        if(!preview_presented){printf("OFT_PREVIEW_PRESENTED width=%u height=%u frozen=0\n",picture.display_width,picture.display_height);preview_presented=true;}
    }else{
        lv_obj_add_flag(preview_image,LV_OBJ_FLAG_HIDDEN);
        if(udp.head_mode){lv_label_set_text(preview_note,picture.detail);lv_obj_remove_flag(preview_note,LV_OBJ_FLAG_HIDDEN);}
        else lv_obj_add_flag(preview_note,LV_OBJ_FLAG_HIDDEN);
    }
    live_control=udp.live_enabled;
    if(udp.action_busy||udp.settings_busy||!udp.head_sensor_ready)lv_obj_add_state(head_button,LV_STATE_DISABLED);else lv_obj_remove_state(head_button,LV_STATE_DISABLED);
    for(unsigned i=0;i<2;i++)lv_obj_set_style_bg_color(mode_buttons[i],lv_color_hex(udp.head_mode==(i==1)?0x287e9d:0x203347),0);
    const char *mode=udp.state==OFT_UDP_FAULT?"FAULT":live_control?(udp.control_state==OFT_CONTROL_ACTIVE?"LIVE ACTIVE":"LIVE READY"):"CONNECTING";
    lv_label_set_text_fmt(values,"%s  Y %+d%%  P %+d%%",mode,(int)(yaw*100),(int)(pitch*100));
    if(udp.head_mode){
        lv_label_set_text_fmt(values,"HEAD Y %s%d.%d P %s%d.%d",
            udp.head_yaw_deg<0?"-":"+",(int)fabsf(udp.head_yaw_deg),abs((int)(udp.head_yaw_deg*10))%10,
            udp.head_pitch_deg<0?"-":"+",(int)fabsf(udp.head_pitch_deg),abs((int)(udp.head_pitch_deg*10))%10);
    }
    if(udp.head_mode&&udp.head_limit_mask){
        lv_label_set_text(limit_text,udp.head_limit_mask==3?"SOFT LIMIT: YAW + PITCH":
            udp.head_limit_mask==1?"SOFT LIMIT: YAW":"SOFT LIMIT: PITCH");
        lv_obj_remove_flag(limit_banner,LV_OBJ_FLAG_HIDDEN);
    }else lv_obj_add_flag(limit_banner,LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_style_text_color(values,lv_color_hex(udp.state==OFT_UDP_FAULT?0xff6a60:live_control?0x35e0b1:0xa9b7c8),0);
    lv_label_set_text(action_labels[0],live_control?"Center":"Scan");
    if(live_control&&!udp.action_busy)lv_obj_remove_state(action_buttons[1],LV_STATE_DISABLED);else lv_obj_add_state(action_buttons[1],LV_STATE_DISABLED);
    if(live_control&&udp.action_busy)lv_obj_add_state(action_buttons[0],LV_STATE_DISABLED);else lv_obj_remove_state(action_buttons[0],LV_STATE_DISABLED);
    if(udp.state!=OFT_UDP_OFF)lv_label_set_text(connection_label,udp.detail);
    if(udp.head_mode&&udp.state==OFT_UDP_READY)lv_label_set_text(connection_label,
        !udp.head_sensor_ready?(udp.head_sensor_calibrated?"IMU recovering: release, then hold again":"Place Mosaico still for 2s to initialize IMU"):
        udp.action_busy?"Center/action settling; please wait":
        udp.head_release_required?"PAUSED: release, then hold again":
        udp.head_limit_mask?"Turn inward to leave soft limit; JOYSTICK mutes":
        !udp.head_yaw_aligned?"Center once to align yaw-limit coordinates":
        udp.head_yaw_held?"Near vertical: yaw held; pitch still tracks":
        udp.head_tracking?(udp.control_state==OFT_CONTROL_ACTIVE?"FOLLOWING target":"ON TARGET (not a limit)"):
        "Gyro+accel tracking / measured soft limits");
    if(udp.battery_valid&&esp_timer_get_time()-udp.battery_updated_us<5000000){v.battery_valid=true;v.battery=udp.battery;}
    char video_status[32];snprintf(video_status,sizeof(video_status),"%s %u.%u fps",picture.low_latency?"FRESH":"VIDEO",picture.fps_milli/1000,(picture.fps_milli%1000)/100);
    if(picture.sparse_live&&!picture.burst_frames)snprintf(video_status,sizeof(video_status),"IDR %u.%u fps",picture.fps_milli/1000,(picture.fps_milli%1000)/100);
    const char *image_state=picture.benchmark?"BENCH (NO LIVE)":picture.refresh_failures?"VIDEO REFRESH FAULT":picture.refresh_recovering?"VIDEO RECOVERING":picture.sparse_live?video_status:
        picture.ready?(esp_timer_get_time()-picture.last_frame_us>3000000?"VIDEO PAUSED":video_status):picture.busy?"DECODING":"VIDEO WAIT";
    bool camera_fresh=udp.camera.valid&&esp_timer_get_time()-udp.camera.updated_us<2000000;
    const char *capture=camera_fresh?(udp.camera.recording?"REC":udp.camera.mode==OFT_CAPTURE_PHOTO?"PHOTO":"VIDEO"):"MODE ?";
    lv_label_set_text_fmt(status_label,"%s | %s | %s",image_state,udp.head_mode?"HEAD":"JOYSTICK",capture);
    lv_obj_set_style_text_color(status_label,lv_color_hex(camera_fresh&&udp.camera.recording?0xff6068:0xe8f2fa),0);
    oft_peripherals_snapshot_t local;oft_peripherals_snapshot(&local);
    bool local_fresh=local.battery_valid&&esp_timer_get_time()-local.battery_us<15000000;
    battery_update(&local_battery,local.gauge_configured?"M~":"M",local_fresh&&local.gauge_configured,local.percent,local_fresh?local.millivolts:0);
    battery_update(&pocket_battery,"P3",v.battery_valid,v.battery,0);
    const char *head_hint=udp.head_release_required?"PAUSED: release, then hold center 1s":
        udp.action_busy||udp.settings_busy?"Camera busy: release before trying again":
        !udp.head_sensor_ready?"IMU recovering: release before trying again":
        udp.head_tracking?"HEAD TRACKING - release to stop":"Release, then hold center 1s; swipe edges for menus";
    if(v.state==OFT_BLE_READY)lv_label_set_text_fmt(connection_label,"%s\n%s",udp.shutter_detail[0]?udp.shutter_detail:udp.detail,
        udp.head_mode&&menu_side==OFT_MENU_NONE?head_hint:"Swipe an edge for controls");
    if(menu_side==OFT_MENU_LEFT||menu_side==OFT_MENU_RIGHT)lv_label_set_text(connection_label,
        setting_view.detail[0]?setting_view.detail:menu_side==OFT_MENU_LEFT?"Outer: size | Inner: fps / file type":"Drag for digital zoom");
    if(active||udp.head_held)lv_obj_remove_flag(values,LV_OBJ_FLAG_HIDDEN);else lv_obj_add_flag(values,LV_OBJ_FLAG_HIDDEN);
    bool choosing=v.state==OFT_BLE_SCANNING&&menu_side==OFT_MENU_NONE;
    if(choosing){lv_obj_add_flag(stick_area,LV_OBJ_FLAG_HIDDEN);lv_obj_add_flag(head_button,LV_OBJ_FLAG_HIDDEN);lv_obj_remove_flag(choices_panel,LV_OBJ_FLAG_HIDDEN);}
    else{
        lv_obj_add_flag(choices_panel,LV_OBJ_FLAG_HIDDEN);
        if(udp.head_mode&&menu_side==OFT_MENU_NONE){lv_obj_add_flag(stick_area,LV_OBJ_FLAG_HIDDEN);lv_obj_remove_flag(head_button,LV_OBJ_FLAG_HIDDEN);}
        else if(!udp.head_mode&&menu_side==OFT_MENU_TOP){lv_obj_remove_flag(stick_area,LV_OBJ_FLAG_HIDDEN);lv_obj_add_flag(head_button,LV_OBJ_FLAG_HIDDEN);}
        else{lv_obj_add_flag(stick_area,LV_OBJ_FLAG_HIDDEN);lv_obj_add_flag(head_button,LV_OBJ_FLAG_HIDDEN);}
    }
    for(unsigned i=0;i<OFT_BLE_CHOICES;i++){
        if(i<v.choice_count){
            const oft_ble_choice_t *c=&v.choices[i];
            lv_label_set_text_fmt(choice_labels[i],"%s  [%02X:%02X]  %d dBm",c->name,c->address[1],c->address[0],c->rssi);
            lv_obj_remove_flag(choice_buttons[i],LV_OBJ_FLAG_HIDDEN);
        }else lv_obj_add_flag(choice_buttons[i],LV_OBJ_FLAG_HIDDEN);
    }
    publish_head_region();
}
static lv_obj_t *label(lv_obj_t *parent,const char *text,int y)
{
    lv_obj_t *o=lv_label_create(parent);lv_label_set_text(o,text);lv_obj_align(o,LV_ALIGN_TOP_MID,0,y);return o;
}
static battery_icon_t battery_create(lv_obj_t *screen,int x)
{
    battery_icon_t b={0};b.group=lv_obj_create(screen);lv_obj_remove_style_all(b.group);
    lv_obj_set_pos(b.group,x,26);lv_obj_set_size(b.group,116,38);lv_obj_remove_flag(b.group,LV_OBJ_FLAG_CLICKABLE|LV_OBJ_FLAG_SCROLLABLE);
    b.body=lv_obj_create(b.group);lv_obj_remove_style_all(b.body);lv_obj_set_pos(b.body,0,5);lv_obj_set_size(b.body,30,17);
    lv_obj_set_style_border_width(b.body,2,0);lv_obj_set_style_radius(b.body,3,0);lv_obj_remove_flag(b.body,LV_OBJ_FLAG_CLICKABLE|LV_OBJ_FLAG_SCROLLABLE);
    b.fill=lv_obj_create(b.body);lv_obj_remove_style_all(b.fill);lv_obj_set_pos(b.fill,2,2);lv_obj_set_size(b.fill,22,9);lv_obj_remove_flag(b.fill,LV_OBJ_FLAG_CLICKABLE|LV_OBJ_FLAG_SCROLLABLE);
    b.tip=lv_obj_create(b.group);lv_obj_remove_style_all(b.tip);lv_obj_set_pos(b.tip,30,10);lv_obj_set_size(b.tip,3,7);lv_obj_remove_flag(b.tip,LV_OBJ_FLAG_CLICKABLE|LV_OBJ_FLAG_SCROLLABLE);
    b.text=lv_label_create(b.group);lv_obj_set_pos(b.text,38,0);lv_obj_remove_flag(b.text,LV_OBJ_FLAG_CLICKABLE);return b;
}
static void battery_update(battery_icon_t *b,const char *name,bool valid,unsigned percent,unsigned mv)
{
    unsigned band=oft_battery_band(valid,percent);const uint32_t colors[]={0x8896a5,0xff525c,0xf4ba54,0x36d18a};lv_color_t c=lv_color_hex(colors[band]);
    lv_obj_set_style_border_color(b->body,c,0);lv_obj_set_style_bg_color(b->tip,c,0);lv_obj_set_style_bg_opa(b->tip,LV_OPA_COVER,0);
    lv_obj_set_style_bg_color(b->fill,c,0);lv_obj_set_width(b->fill,valid&&percent>=25?(int)(22*percent/100):1);lv_obj_set_style_bg_opa(b->fill,valid&&percent>=25?LV_OPA_COVER:LV_OPA_TRANSP,0);
    lv_obj_set_style_text_color(b->text,c,0);
    if(band)lv_label_set_text_fmt(b->text,"%s %u%%",name,percent);
    else if(mv)lv_label_set_text_fmt(b->text,"%s --\n%u.%02uV",name,mv/1000,(mv%1000)/10);
    else lv_label_set_text_fmt(b->text,"%s --",name);
}
static lv_obj_t *wheel_button(oft_menu_side_t side,int x,int y,int w,int h,const char *name)
{
    int cx,cy;oft_menu_center(side,&cx,&cy);lv_obj_t *b=lv_button_create(wheels[side]);
    lv_obj_set_pos(b,x-cx+240,y-cy+240);lv_obj_set_size(b,w,h);lv_obj_set_style_radius(b,22,0);
    lv_obj_set_style_bg_color(b,lv_color_hex(0x203347),0);lv_obj_set_style_bg_opa(b,220,0);lv_obj_set_style_text_color(b,lv_color_white(),0);lv_obj_set_style_shadow_width(b,0,0);
    lv_obj_t *text=lv_label_create(b);lv_label_set_text(text,name);lv_obj_center(text);return b;
}
static lv_obj_t *setting_arc(int radius,uint32_t color)
{
    lv_obj_t *a=lv_arc_create(wheels[OFT_MENU_LEFT]);lv_obj_set_size(a,radius*2,radius*2);
    lv_obj_set_pos(a,240-radius,240-radius);lv_arc_set_rotation(a,radius==200?310:295);lv_arc_set_bg_angles(a,0,radius==200?100:130);
    lv_arc_set_range(a,0,1);lv_arc_set_value(a,0);lv_obj_set_ext_click_area(a,12);
    lv_obj_set_style_arc_width(a,24,LV_PART_MAIN);lv_obj_set_style_arc_width(a,24,LV_PART_INDICATOR);
    lv_obj_set_style_arc_color(a,lv_color_hex(0x344e62),LV_PART_MAIN);lv_obj_set_style_arc_color(a,lv_color_hex(color),LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(a,lv_color_hex(color),LV_PART_KNOB);lv_obj_set_style_pad_all(a,9,LV_PART_KNOB);
    lv_obj_add_flag(a,LV_OBJ_FLAG_PRESS_LOCK);lv_obj_add_event_cb(a,format_event,LV_EVENT_ALL,NULL);return a;
}
static lv_obj_t *wheel_text(oft_menu_side_t side,int x,int y,int width,const char *text)
{
    int cx,cy;oft_menu_center(side,&cx,&cy);lv_obj_t *l=lv_label_create(wheels[side]);
    lv_obj_set_pos(l,x-cx+240,y-cy+240);lv_obj_set_width(l,width);lv_label_set_text(l,text);
    lv_obj_set_style_text_align(l,LV_TEXT_ALIGN_CENTER,0);return l;
}
void oft_ui_create(void)
{
    lv_obj_t *screen=lv_screen_active();
    lv_obj_set_style_bg_color(screen,lv_color_hex(0x081018),0);
    lv_obj_set_style_text_color(screen,lv_color_white(),0);
    lv_obj_set_style_border_width(screen,0,0);
    lv_obj_remove_flag(screen,LV_OBJ_FLAG_SCROLLABLE);
    /* Mid-edge markers stay visible on the rounded physical panel. Missing
       rectangular corners must not be mistaken for an incorrect resolution. */
    const int guides[][4]={{230,4,20,2},{230,474,20,2},{4,230,2,20},{474,230,2,20}};
    for(unsigned i=0;i<4;i++) {
        lv_obj_t *guide=lv_obj_create(screen);
        lv_obj_set_pos(guide,guides[i][0],guides[i][1]);
        lv_obj_set_size(guide,guides[i][2],guides[i][3]);
        lv_obj_set_style_bg_color(guide,lv_color_hex(0x00d0c0),0);
        lv_obj_set_style_border_width(guide,0,0);
        lv_obj_set_style_radius(guide,0,0);
        lv_obj_remove_flag(guide,LV_OBJ_FLAG_CLICKABLE|LV_OBJ_FLAG_SCROLLABLE);
    }
    label(screen,"OpenFrameTap",5);
    status_label=label(screen,"BLE starting",66);lv_obj_set_width(status_label,440);lv_obj_set_style_text_align(status_label,LV_TEXT_ALIGN_CENTER,0);
    values=label(screen,"Connecting",392);
    preview_image=lv_image_create(screen);lv_obj_align(preview_image,LV_ALIGN_TOP_MID,0,141);
    lv_obj_remove_flag(preview_image,LV_OBJ_FLAG_CLICKABLE);lv_obj_add_flag(preview_image,LV_OBJ_FLAG_HIDDEN);
    preview_note=label(screen,"Waiting for real preview image",210);lv_obj_set_width(preview_note,410);
    lv_obj_set_style_text_align(preview_note,LV_TEXT_ALIGN_CENTER,0);lv_obj_add_flag(preview_note,LV_OBJ_FLAG_HIDDEN);
    limit_banner=lv_obj_create(screen);lv_obj_set_size(limit_banner,420,44);lv_obj_align(limit_banner,LV_ALIGN_TOP_MID,0,86);
    lv_obj_set_style_bg_color(limit_banner,lv_color_hex(0xffaa32),0);lv_obj_set_style_text_color(limit_banner,lv_color_hex(0x161b20),0);
    lv_obj_set_style_border_width(limit_banner,0,0);lv_obj_set_style_radius(limit_banner,12,0);
    lv_obj_remove_flag(limit_banner,LV_OBJ_FLAG_CLICKABLE|LV_OBJ_FLAG_SCROLLABLE);lv_obj_add_flag(limit_banner,LV_OBJ_FLAG_HIDDEN);
    limit_text=lv_label_create(limit_banner);lv_obj_center(limit_text);
    for(unsigned i=1;i<=4;i++){
        int cx,cy;oft_menu_center((oft_menu_side_t)i,&cx,&cy);wheels[i]=lv_obj_create(screen);
        lv_obj_set_pos(wheels[i],cx-240,cy-240);lv_obj_set_size(wheels[i],480,480);
        lv_obj_set_style_pad_all(wheels[i],0,0);lv_obj_set_style_radius(wheels[i],LV_RADIUS_CIRCLE,0);
        lv_obj_set_style_bg_color(wheels[i],lv_color_hex(0x172c40),0);lv_obj_set_style_bg_opa(wheels[i],153,0);
        lv_obj_set_style_border_color(wheels[i],lv_color_hex(0x5aa5c5),0);lv_obj_set_style_border_width(wheels[i],2,0);
        lv_obj_remove_flag(wheels[i],LV_OBJ_FLAG_SCROLLABLE);lv_obj_add_flag(wheels[i],LV_OBJ_FLAG_HIDDEN);
    }
    for(unsigned i=0;i<2;i++){
        mode_buttons[i]=wheel_button(OFT_MENU_TOP,i?348:48,68,84,52,i?"HEAD":"JOYSTICK");
        lv_obj_add_event_cb(mode_buttons[i],mode_event,LV_EVENT_CLICKED,(void*)(uintptr_t)i);
    }
    resolution_arc=setting_arc(200,0x38c8e8);rate_arc=setting_arc(128,0xeba84f);
    format_label=wheel_text(OFT_MENU_LEFT,14,220,90,"Loading");
    format_note=wheel_text(OFT_MENU_LEFT,32,370,166,"Reading camera capabilities");
    lv_obj_add_flag(format_note,LV_OBJ_FLAG_HIDDEN); // Shared footer reports state without covering either dial.
    for(unsigned i=0;i<2;i++){
        camera_modes[i]=wheel_button(OFT_MENU_LEFT,22,i?267:173,70,40,i?"PHOTO":"VIDEO");
        lv_obj_add_event_cb(camera_modes[i],setting_mode_event,LV_EVENT_CLICKED,(void*)(uintptr_t)(i?5:1));
    }
    zoom_slider=lv_slider_create(wheels[OFT_MENU_RIGHT]);lv_obj_set_pos(zoom_slider,124,144);lv_obj_set_size(zoom_slider,28,192);
    lv_slider_set_range(zoom_slider,217,434);lv_slider_set_value(zoom_slider,217,LV_ANIM_OFF);lv_obj_set_ext_click_area(zoom_slider,22);
    lv_obj_set_style_bg_color(zoom_slider,lv_color_hex(0x344e62),LV_PART_MAIN);lv_obj_set_style_bg_color(zoom_slider,lv_color_hex(0x38c8e8),LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(zoom_slider,lv_color_hex(0x38c8e8),LV_PART_KNOB);lv_obj_set_style_pad_all(zoom_slider,12,LV_PART_KNOB);
    lv_obj_add_flag(zoom_slider,LV_OBJ_FLAG_PRESS_LOCK);lv_obj_add_event_cb(zoom_slider,zoom_event,LV_EVENT_ALL,NULL);
    zoom_label=wheel_text(OFT_MENU_RIGHT,292,95,150,"Zoom --");zoom_note=wheel_text(OFT_MENU_RIGHT,286,367,160,"Waiting for camera");
    lv_obj_t *stick=lv_obj_create(wheels[OFT_MENU_TOP]);lv_obj_set_size(stick,192,192);lv_obj_set_pos(stick,144,256);
    lv_obj_set_style_radius(stick,LV_RADIUS_CIRCLE,0);
    lv_obj_set_style_bg_color(stick,lv_color_hex(0x152f43),0);lv_obj_set_style_bg_opa(stick,100,0);
    lv_obj_set_style_border_color(stick,lv_color_hex(0x33bad4),0);
    lv_obj_remove_flag(stick,LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(stick,LV_OBJ_FLAG_PRESS_LOCK);
    lv_obj_add_event_cb(stick,stick_event,LV_EVENT_ALL,NULL);
    stick_area=stick;
    knob=lv_obj_create(stick);lv_obj_set_size(knob,54,54);lv_obj_center(knob);
    lv_obj_set_style_radius(knob,LV_RADIUS_CIRCLE,0);lv_obj_set_style_bg_color(knob,lv_color_hex(0x22bbe4),0);
    lv_obj_remove_flag(knob,LV_OBJ_FLAG_CLICKABLE|LV_OBJ_FLAG_SCROLLABLE);
    choices_panel=lv_obj_create(screen);lv_obj_set_size(choices_panel,440,285);lv_obj_align(choices_panel,LV_ALIGN_TOP_MID,0,93);
    lv_obj_set_style_bg_color(choices_panel,lv_color_hex(0x081018),0);lv_obj_set_style_border_width(choices_panel,0,0);
    lv_obj_set_style_pad_all(choices_panel,0,0);lv_obj_remove_flag(choices_panel,LV_OBJ_FLAG_SCROLLABLE);
    for(unsigned i=0;i<OFT_BLE_CHOICES;i++){
        lv_obj_t *b=lv_button_create(choices_panel);choice_buttons[i]=b;lv_obj_set_size(b,436,62);lv_obj_set_pos(b,0,i*69);
        lv_obj_set_style_bg_color(b,lv_color_hex(0x23465b),0);lv_obj_set_style_text_color(b,lv_color_white(),0);
        choice_labels[i]=lv_label_create(b);lv_obj_center(choice_labels[i]);
        lv_obj_add_event_cb(b,select_event,LV_EVENT_CLICKED,(void*)(uintptr_t)i);lv_obj_add_flag(b,LV_OBJ_FLAG_HIDDEN);
    }
    lv_obj_add_flag(choices_panel,LV_OBJ_FLAG_HIDDEN);
    /* Transparent central gesture surface, not a rendered button.48px side
       margins preserve edge swipes; header/battery and footer are excluded. */
    head_button=lv_obj_create(screen);lv_obj_remove_style_all(head_button);
    lv_obj_set_pos(head_button,48,104);lv_obj_set_size(head_button,384,288);
    lv_obj_remove_flag(head_button,LV_OBJ_FLAG_SCROLLABLE);lv_obj_add_flag(head_button,LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(head_button,LV_OBJ_FLAG_PRESS_LOCK);
    lv_obj_add_event_cb(head_button,head_event,LV_EVENT_ALL,NULL);lv_obj_add_flag(head_button,LV_OBJ_FLAG_HIDDEN);
    const char *names[]={"Scan","180 deg","Disconnect"};
    for(int i=0;i<3;i++){
        lv_obj_t *b=wheel_button(OFT_MENU_BOTTOM,i==0?88:i==1?264:174,i==2?282:358,i==2?132:128,58,names[i]);
        lv_obj_set_style_radius(b,18,0);lv_obj_set_style_bg_color(b,lv_color_hex(i==2?0x853b46:0x23465b),0);
        lv_obj_t *l=lv_obj_get_child(b,0);
        if(i<2){action_buttons[i]=b;action_labels[i]=l;}
        lv_obj_add_event_cb(b,button_event,LV_EVENT_CLICKED,(void*)names[i]);
        if(i==1)lv_obj_add_state(b,LV_STATE_DISABLED);
    }
    connection_label=label(screen,"Swipe an edge for controls",438);
    lv_obj_set_width(connection_label,430);lv_obj_set_style_text_align(connection_label,LV_TEXT_ALIGN_CENTER,0);
    lv_timer_create(refresh,100,NULL);
    shadow_indev=lv_indev_create();lv_indev_set_type(shadow_indev,LV_INDEV_TYPE_POINTER);
    lv_indev_set_disp(shadow_indev,lv_display_get_default());lv_indev_set_read_cb(shadow_indev,shadow_read);
    lv_indev_set_mode(shadow_indev,LV_INDEV_MODE_EVENT);
    local_battery=battery_create(screen,26);pocket_battery=battery_create(screen,338);
    menu_set(OFT_MENU_NONE);
}
