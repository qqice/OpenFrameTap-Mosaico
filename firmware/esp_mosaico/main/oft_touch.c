#include "oft_touch.h"
#include "oft_udp.h"
#include "oft_ui.h"
#include "oft_motion.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "unity.h"
#include <string.h>
#include <stdio.h>

typedef struct {bool valid;unsigned count;int x,y;int64_t at;} sample_t;
typedef struct {bool enabled;int x1,y1,x2,y2;int64_t at;} region_t;
typedef struct {bool previous_down,held,blocked,pending;int start_x,start_y;int64_t started,last_step;} lease_t;
enum {NONE,PRESS,HOLD,RELEASE};
static portMUX_TYPE mux=portMUX_INITIALIZER_UNLOCKED;
static sample_t latest;
static region_t region;
static oft_touch_stats_t stats;
static esp_lcd_touch_handle_t device;
static TaskHandle_t sampler;
static bool shadow_reset;
static int64_t shadow_until;

static void IRAM_ATTR touch_interrupt(esp_lcd_touch_handle_t tp)
{
    (void)tp;BaseType_t wake=pdFALSE;
    if(sampler)vTaskNotifyGiveFromISR(sampler,&wake);
    if(wake)portYIELD_FROM_ISR();
}

/* Only fresh hardware samples renew the lease. A read/UI fault requires a real
   release before another press. PRESS_LOCK allows motion outside the rectangle
   after an initial hit; it never turns a stale cached coordinate into input. */
static unsigned step(lease_t *l,sample_t s,region_t r,int64_t now)
{
    bool gap=l->last_step&&(now<l->last_step||now-l->last_step>=100000);
    l->last_step=now;
    bool fresh=s.valid&&now>=s.at&&now-s.at<100000;
    if(fresh&&s.count==0){
        bool held=l->held;*l=(lease_t){0};return held?RELEASE:NONE;
    }
    bool ui=r.enabled&&now>=r.at&&now-r.at<250000;
    if(!fresh||s.count!=1||!ui||gap){
        bool held=l->held;
        if(held||l->previous_down||(fresh&&s.count)){l->blocked=true;l->previous_down=true;}
        l->held=false;l->pending=false;
        return held?RELEASE:NONE;
    }
    bool down=!l->previous_down;l->previous_down=true;
    if(!l->held&&down&&!l->blocked&&s.x>=r.x1&&s.x<=r.x2&&s.y>=r.y1&&s.y<=r.y2){
        l->pending=true;l->started=now;l->start_x=s.x;l->start_y=s.y;
    }
    if(l->pending){
        int dx=s.x-l->start_x,dy=s.y-l->start_y;
        if(s.x<r.x1||s.x>r.x2||s.y<r.y1||s.y>r.y2||dx*dx+dy*dy>24*24){l->pending=false;l->blocked=true;}
        else if(s.at>=l->started+1000000){l->pending=false;l->held=true;return PRESS;}
    }
    return l->held?HOLD:NONE;
}
void oft_touch_head_region(int x1,int y1,int x2,int y2,bool enabled)
{
    portENTER_CRITICAL(&mux);region=(region_t){enabled,x1,y1,x2,y2,esp_timer_get_time()};portEXIT_CRITICAL(&mux);
}
void oft_touch_shadow_start(int64_t end_us)
{portENTER_CRITICAL(&mux);shadow_until=end_us;shadow_reset=true;portEXIT_CRITICAL(&mux);}
void oft_touch_stats(oft_touch_stats_t *out)
{portENTER_CRITICAL(&mux);*out=stats;portEXIT_CRITICAL(&mux);}
static void read_gui(lv_indev_t *indev,lv_indev_data_t *out)
{
    static bool blocked;sample_t s;portENTER_CRITICAL(&mux);s=latest;portEXIT_CRITICAL(&mux);
    int64_t now=esp_timer_get_time();bool fresh=s.valid&&now>=s.at&&now-s.at<100000;
    if(!fresh||s.count>1){
        bool pressed=lv_indev_get_state(indev)==LV_INDEV_STATE_PRESSED;
        if(pressed||(fresh&&s.count))blocked=true;
        if(pressed)lv_indev_wait_release(indev); // Cancel, do not synthesize a button click on an IO fault.
    }
    if(fresh&&!s.count)blocked=false;
    bool down=fresh&&s.count==1&&!blocked;
    if(oft_ui_pointer(s.x,s.y,down,fresh&&s.count<=1)){
        if(lv_indev_get_state(indev)==LV_INDEV_STATE_PRESSED)lv_indev_wait_release(indev);
        down=false;
    }
    out->state=down?LV_INDEV_STATE_PRESSED:LV_INDEV_STATE_RELEASED;
    out->point=(lv_point_t){s.x,s.y};
}
static void poll_touch(void *arg)
{
    (void)arg;lease_t physical={0},shadow={0};int64_t last=0,shadow_last=0;
    TickType_t next=xTaskGetTickCount();
    for(;;){
        /* CST reports are interrupt-driven, as in the original LVGL bridge.
           Polling its report register without an IRQ returns an invalid ACK.
           Coalesce IRQs at 50Hz; no IRQ must NOT refresh a cached contact. */
        bool received=ulTaskNotifyTake(pdTRUE,0)>0;
        esp_lcd_touch_point_data_t points[2]={0};uint8_t count=0;
        esp_err_t rc=ESP_OK;
        if(received){
            rc=esp_lcd_touch_read_data(device);
            if(rc==ESP_OK)rc=esp_lcd_touch_get_data(device,points,&count,2);
        }
        int64_t now=esp_timer_get_time();
        sample_t s={.valid=rc==ESP_OK,.count=count,.at=now};
        if(count){s.x=points[0].x;s.y=points[0].y;if(s.x>=480||s.y>=480)s.valid=false;}
        region_t r;int64_t until;
        portENTER_CRITICAL(&mux);
        if(!received)s=latest;
        else{
            if(!count){s.x=latest.x;s.y=latest.y;} // Release preserves click coordinates.
            latest=s;stats.samples++;if(!s.valid)stats.errors++;
        }
        r=region;until=shadow_until;
        if(last&&(uint32_t)(now-last)>stats.max_gap_us)stats.max_gap_us=(uint32_t)(now-last);
        if(shadow_reset){
            shadow_reset=false;shadow=(lease_t){0};shadow_last=0;
            stats.shadow_presses=stats.shadow_releases=stats.shadow_early_stops=stats.shadow_max_gap_us=0;
        }
        portEXIT_CRITICAL(&mux);last=now;
        oft_udp_snapshot_t u;oft_udp_snapshot(&u);
        if(!u.head_mode||u.state!=OFT_UDP_READY||u.action_busy||u.settings_busy||!u.head_sensor_ready)r.enabled=false;
        unsigned action=step(&physical,s,r,now);
        if(received&&(action==PRESS||action==HOLD)){
            oft_udp_head_hold(true,action==PRESS);
            if(action==PRESS)oft_motion_head_ack(true);
        }else if(action==RELEASE){oft_udp_head_hold(false,false);oft_motion_head_ack(false);}
        if(action==PRESS||action==RELEASE){
            portENTER_CRITICAL(&mux);if(action==PRESS)stats.head_presses++;else stats.head_releases++;portEXIT_CRITICAL(&mux);
            printf("OFT_TOUCH_LEASE event=%s us=%lld valid=%d count=%u ui_age_us=%lld\n",action==PRESS?"press":"release",(long long)now,s.valid,s.count,(long long)(now-r.at));
        }
        /* Shadow simulates fresh reports at the real producer cadence. Idle
           hardware sends no IRQ, so this is scheduling/lease evidence, not a
           physical-finger test. Actual read errors still invalidate the trial.
           Its output is NEVER passed to the network/controller publisher. */
        if(until){
            sample_t fake={.valid=!received||s.valid,.at=now};fake.count=now<until?1:0;fake.x=(r.x1+r.x2)/2;fake.y=(r.y1+r.y2)/2;
            unsigned simulated=step(&shadow,fake,r,now);
            portENTER_CRITICAL(&mux);
            if(simulated==PRESS)stats.shadow_presses++;
            if(simulated==PRESS||simulated==HOLD){
                if(shadow_last&&(uint32_t)(now-shadow_last)>stats.shadow_max_gap_us)stats.shadow_max_gap_us=(uint32_t)(now-shadow_last);
                shadow_last=now;
            }
            if(simulated==RELEASE){stats.shadow_releases++;if(now<until)stats.shadow_early_stops++;}
            portEXIT_CRITICAL(&mux);
        }
        vTaskDelayUntil(&next,pdMS_TO_TICKS(20));
    }
}
esp_err_t oft_touch_start(esp_lcd_touch_handle_t touch,lv_display_t *display)
{
    if(!touch||!display)return ESP_ERR_INVALID_ARG;
    device=touch;
    lv_indev_t *input=lv_indev_create();if(!input)return ESP_ERR_NO_MEM;
    lv_indev_set_type(input,LV_INDEV_TYPE_POINTER);lv_indev_set_disp(input,display);
    lv_indev_set_read_cb(input,read_gui);lv_timer_set_period(lv_indev_get_read_timer(input),20);
    if(xTaskCreatePinnedToCore(poll_touch,"oft_touch",4096,NULL,5,&sampler,0)!=pdPASS){lv_indev_delete(input);return ESP_ERR_NO_MEM;}
    esp_err_t rc=esp_lcd_touch_register_interrupt_callback(touch,touch_interrupt);
    if(rc!=ESP_OK){vTaskDelete(sampler);sampler=NULL;lv_indev_delete(input);return rc;}
    return ESP_OK;
}
static void arm_test_lease(lease_t *l,sample_t *s,region_t *r)
{
    TEST_ASSERT_EQUAL_UINT(NONE,step(l,*s,*r,s->at));
    for(unsigned i=1;i<=50;i++){
        s->at+=20000;r->at=s->at;
        TEST_ASSERT_EQUAL_UINT(i==50?PRESS:NONE,step(l,*s,*r,s->at));
    }
}
static void test_touch_press_release_and_lock(void)
{
    lease_t l={0};region_t r={true,90,318,390,374,1000};sample_t s={true,1,240,346,1000};
    arm_test_lease(&l,&s,&r);
    s.x=10;s.y=10;s.at+=20000;TEST_ASSERT_EQUAL_UINT(HOLD,step(&l,s,r,s.at));
    s.count=0;s.at+=20000;TEST_ASSERT_EQUAL_UINT(RELEASE,step(&l,s,r,s.at));
    TEST_ASSERT_EQUAL_UINT(NONE,step(&l,s,r,s.at));
    s.count=1;s.at+=20000;TEST_ASSERT_EQUAL_UINT(NONE,step(&l,s,r,s.at));
    s.x=240;s.y=346;TEST_ASSERT_EQUAL_UINT(NONE,step(&l,s,r,s.at));
}
static void test_touch_stale_and_error_fail_closed(void)
{
    lease_t l={0};region_t r={true,90,318,390,374,1000};sample_t s={true,1,240,346,1000};
    sample_t idle_error={0};TEST_ASSERT_EQUAL_UINT(NONE,step(&l,idle_error,r,1000));
    arm_test_lease(&l,&s,&r);
    TEST_ASSERT_EQUAL_UINT(RELEASE,step(&l,s,r,s.at+100000));
    s.at+=120000;r.at=s.at;TEST_ASSERT_EQUAL_UINT(NONE,step(&l,s,r,s.at));
    s.count=0;step(&l,s,r,s.at);s.count=1;arm_test_lease(&l,&s,&r);
    s.valid=false;TEST_ASSERT_EQUAL_UINT(RELEASE,step(&l,s,r,s.at));
}
static void test_touch_ui_lease_and_multiple_contacts(void)
{
    lease_t l={0};region_t r={true,90,318,390,374,1000};sample_t s={true,1,240,346,1000};
    arm_test_lease(&l,&s,&r);
    for(unsigned i=0;i<12;i++){s.at+=20000;TEST_ASSERT_EQUAL_UINT(HOLD,step(&l,s,r,s.at));}
    s.at+=10000;TEST_ASSERT_EQUAL_UINT(RELEASE,step(&l,s,r,s.at));
    r.at=s.at;TEST_ASSERT_EQUAL_UINT(NONE,step(&l,s,r,s.at));
    s.count=0;step(&l,s,r,s.at);s.count=1;arm_test_lease(&l,&s,&r);
    s.count=2;TEST_ASSERT_EQUAL_UINT(RELEASE,step(&l,s,r,s.at));
}
static void test_touch_longpress_cancel(void)
{
    lease_t l={0};region_t r={true,48,104,432,392,1000};sample_t s={true,1,240,240,1000};
    TEST_ASSERT_EQUAL_UINT(NONE,step(&l,s,r,s.at));
    s.at+=20000;s.x+=25;r.at=s.at;TEST_ASSERT_EQUAL_UINT(NONE,step(&l,s,r,s.at));
    TEST_ASSERT_TRUE(l.blocked);TEST_ASSERT_FALSE(l.held);
    s.count=0;step(&l,s,r,s.at);s.count=1;s.x=240;
    TEST_ASSERT_EQUAL_UINT(NONE,step(&l,s,r,s.at));s.count=0;s.at+=20000;
    TEST_ASSERT_EQUAL_UINT(NONE,step(&l,s,r,s.at));TEST_ASSERT_FALSE(l.pending);
}
void oft_touch_selftests(void)
{RUN_TEST(test_touch_press_release_and_lock);RUN_TEST(test_touch_stale_and_error_fail_closed);RUN_TEST(test_touch_ui_lease_and_multiple_contacts);RUN_TEST(test_touch_longpress_cancel);}
