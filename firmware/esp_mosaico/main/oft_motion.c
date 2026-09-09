#include "oft_motion.h"
#include "oft_board.h"
#include "oft_haptic.h"
#include "oft_bmi_delay.h"
#include "unity.h"
#include "bmi270.h"
#include "bmm150.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <math.h>
#include <string.h>

static portMUX_TYPE mux=portMUX_INITIALIZER_UNLOCKED;
static oft_motion_snapshot_t view;
static oft_haptic_pattern_t haptic;
static int64_t diagnostic_until;
static int64_t head_ack_started;
static bool head_ack_at(int64_t started,int64_t now)
{return started&&now>=started&&now-started<150000&&(now-started)%100000<50000;}
static bool head_ack_on(int64_t now){return head_ack_at(head_ack_started,now);}
static bmi270_handle_t *imu;
static struct bmm150_dev mag[2];
static i2c_master_dev_handle_t mag_io[2];
static uint8_t mag_raw_bytes[2][8];
static const char *TAG="oft_motion";
static void test_imu_delay_preserves_minimum_time(void)
{
    for(unsigned i=0;i<3;i++){
        int64_t begin=esp_timer_get_time();oft_bmi_delay(1);
        TEST_ASSERT_TRUE(esp_timer_get_time()-begin>=1000000/configTICK_RATE_HZ);
    }
}
static void test_head_ack_two_pulses(void)
{
    TEST_ASSERT_TRUE(head_ack_at(1000,1000));TEST_ASSERT_TRUE(head_ack_at(1000,50999));
    TEST_ASSERT_FALSE(head_ack_at(1000,51000));TEST_ASSERT_FALSE(head_ack_at(1000,100999));
    TEST_ASSERT_TRUE(head_ack_at(1000,101000));TEST_ASSERT_FALSE(head_ack_at(1000,151000));
    TEST_ASSERT_FALSE(head_ack_at(0,1000));TEST_ASSERT_FALSE(head_ack_at(1000,999));
}
void oft_motion_selftests(void){RUN_TEST(test_imu_delay_preserves_minimum_time);RUN_TEST(test_head_ack_two_pulses);}
static QueueHandle_t events;
static esp_timer_handle_t haptic_timer;
bool oft_motion_event_pop(oft_motion_event_t *e){return events&&e&&xQueueReceive(events,e,0)==pdTRUE;}
static void record_event(oft_motion_event_t e)
{
    if(events&&xQueueSend(events,&e,0)!=pdTRUE){portENTER_CRITICAL(&mux);view.event_drops++;portEXIT_CRITICAL(&mux);}
}
static void haptic_tick(void *unused)
{
    (void)unused;int64_t now=esp_timer_get_time();
    portENTER_CRITICAL(&mux);
    bool motor=oft_haptic_output(&haptic,now)||now<diagnostic_until||head_ack_on(now);
    bool changed=motor!=view.motor_on;gpio_set_level(8,motor);view.motor_on=motor;
    portEXIT_CRITICAL(&mux);
    if(changed)record_event((oft_motion_event_t){.timestamp_us=now,.on=motor});
}
void oft_motion_snapshot(oft_motion_snapshot_t *out){portENTER_CRITICAL(&mux);*out=view;portEXIT_CRITICAL(&mux);}
void oft_motion_haptic(bool on)
{
    int64_t now=esp_timer_get_time();bool changed=false,output=false;
    portENTER_CRITICAL(&mux);oft_haptic_request(&haptic,on,now);
    if(!on){diagnostic_until=0;output=head_ack_on(now);changed=view.motor_on!=output;gpio_set_level(8,output);view.motor_on=output;}portEXIT_CRITICAL(&mux);
    if(changed)record_event((oft_motion_event_t){.timestamp_us=now,.on=output});
}
void oft_motion_haptic_test(void)
{portENTER_CRITICAL(&mux);diagnostic_until=esp_timer_get_time()+50000;portEXIT_CRITICAL(&mux);}
void oft_motion_head_ack(bool start)
{
    int64_t now=esp_timer_get_time();bool changed=false;
    portENTER_CRITICAL(&mux);head_ack_started=start?now:0;
    if(!start){changed=view.motor_on;gpio_set_level(8,0);view.motor_on=false;}portEXIT_CRITICAL(&mux);
    if(changed)record_event((oft_motion_event_t){.timestamp_us=now,.on=false});
}
static BMM150_INTF_RET_TYPE mag_read(uint8_t reg,uint8_t *data,uint32_t size,void *ctx)
{
    if(i2c_master_transmit_receive(*(i2c_master_dev_handle_t*)ctx,&reg,1,data,size,25)!=ESP_OK)return -1;
    /* Preserve the SAME burst that Bosch compensates. A second read could
       describe a different sample and disguise overflow as a real zero. */
    if(reg==BMM150_REG_DATA_X_LSB&&size==8){
        for(unsigned i=0;i<2;i++)if(ctx==&mag_io[i])memcpy(mag_raw_bytes[i],data,8);
    }
    return 0;
}
static BMM150_INTF_RET_TYPE mag_write(uint8_t reg,const uint8_t *data,uint32_t size,void *ctx)
{
    uint8_t buf[32];if(size>sizeof(buf)-1)return -1;buf[0]=reg;memcpy(buf+1,data,size);
    return i2c_master_transmit(*(i2c_master_dev_handle_t*)ctx,buf,size+1,25)==ESP_OK?0:-1;
}
static void delay_us(uint32_t us,void *ctx)
{(void)ctx;if(us>=10000)vTaskDelay(pdMS_TO_TICKS((us+999)/1000));else esp_rom_delay_us(us);}
static bool init_mag(unsigned index)
{
    i2c_device_config_t cfg={.dev_addr_length=I2C_ADDR_BIT_LEN_7,.device_address=0x11+index,.scl_speed_hz=400000};
    if(i2c_master_bus_add_device(oft_board_i2c(),&cfg,&mag_io[index])!=ESP_OK)return false;
    mag[index]=(struct bmm150_dev){.intf=BMM150_I2C_INTF,.intf_ptr=&mag_io[index],.read=mag_read,.write=mag_write,.delay_us=delay_us};
    if(bmm150_init(&mag[index])!=BMM150_OK)return false;
    struct bmm150_settings settings={.preset_mode=BMM150_PRESETMODE_REGULAR,.pwr_mode=BMM150_POWERMODE_NORMAL};
    if(bmm150_set_presetmode(&settings,&mag[index])!=BMM150_OK)return false;
    return bmm150_set_op_mode(&settings,&mag[index])==BMM150_OK;
}
static void task(void *unused)
{
    (void)unused;
    bmi270_driver_config_t driver={.addr=0x69,.interface=BMI270_USE_I2C,.i2c_bus=oft_board_i2c()};
    unsigned init_stage=1;esp_err_t init_error=bmi270_create(&driver,&imu);
    bool imu_ok=init_error==ESP_OK;
    uint8_t id=0;
    if(imu_ok){init_stage=2;init_error=bmi270_get_chip_id(imu,&id);
        if(init_error==ESP_OK&&id!=BMI270_CHIP_ID)init_error=ESP_ERR_NOT_FOUND;
        imu_ok=init_error==ESP_OK;}
    bmi270_config_t cfg={.acce_odr=BMI270_ACC_ODR_100_HZ,.acce_range=BMI270_ACC_RANGE_4_G,.gyro_odr=BMI270_GYR_ODR_100_HZ,.gyro_range=BMI270_GYR_RANGE_500_DPS};
    if(imu_ok){init_stage=3;init_error=bmi270_start(imu,&cfg);imu_ok=init_error==ESP_OK;}
    bool mag_ok[2]={init_mag(0),init_mag(1)};
    portENTER_CRITICAL(&mux);view.imu_ok=imu_ok;view.imu_id=id;
    view.imu_init_error=init_error;view.imu_init_stage=init_stage;
    for(unsigned i=0;i<2;i++){view.mag_ok[i]=mag_ok[i];view.mag_id[i]=mag[i].chip_id;}
    portEXIT_CRITICAL(&mux);
    ESP_LOGI(TAG,"IMU=%d id=%02x BMM[11]=%d/%02x BMM[12]=%d/%02x",imu_ok,id,mag_ok[0],mag[0].chip_id,mag_ok[1],mag[1].chip_id);
    oft_quat_t q=oft_quat_identity();float sum[3]={0},bias[3]={0};unsigned still=0;bool calibrated=false,orientation_ready=false;
    unsigned epoch=0,recoveries=0;
    int64_t last=0,next_mag=0;TickType_t tick=xTaskGetTickCount();
    for(;;){
        int64_t now=esp_timer_get_time();
        /* Independent timer owns pulse timing; I2C stalls cannot lengthen it. */
        portENTER_CRITICAL(&mux);bool motor=view.motor_on;portEXIT_CRITICAL(&mux);
        float a[3],g[3];
        if(imu_ok&&bmi270_get_acce_data(imu,&a[0],&a[1],&a[2])==ESP_OK&&
           bmi270_get_gyro_data(imu,&g[0],&g[1],&g[2])==ESP_OK){
            now=esp_timer_get_time();float an=hypotf(hypotf(a[0],a[1]),a[2]),gn=hypotf(hypotf(g[0],g[1]),g[2]);
            if(!calibrated){
                if(!motor&&fabsf(an-1)<.06f&&gn<1){for(unsigned i=0;i<3;i++)sum[i]+=g[i];still++;}
                else{still=0;memset(sum,0,sizeof(sum));}
                if(still>=200){
                    for(unsigned i=0;i<3;i++)bias[i]=sum[i]/still;
                    calibrated=true;orientation_ready=oft_orientation_from_gravity(a,&q);last=now;
                }
            }else if(!orientation_ready){
                /* Bias is still valid after a scheduler/read gap. Rebase the
                   attitude only; a changed epoch invalidates any held target. */
                orientation_ready=oft_orientation_from_gravity(a,&q);
            }else if(last){float corrected[3];for(unsigned i=0;i<3;i++)corrected[i]=g[i]-bias[i];
                if(!oft_orientation_step(&q,corrected,a,(now-last)/1e6f)){
                    orientation_ready=false;epoch++;recoveries++;
                    record_event((oft_motion_event_t){.timestamp_us=now,.interval_us=now-last,.epoch=epoch,.gap=true});
                }
            }
            last=now;
            portENTER_CRITICAL(&mux);memcpy(view.accel,a,sizeof(a));memcpy(view.gyro,g,sizeof(g));memcpy(view.bias,bias,sizeof(bias));
            view.bias_ready=calibrated;view.orientation_ready=orientation_ready;view.orientation_epoch=epoch;view.recovery_count=recoveries;
            view.stationary_samples=still;view.orientation=q;view.sample_us=now;view.samples++;portEXIT_CRITICAL(&mux);
        }else if(imu_ok){
            if(orientation_ready){epoch++;recoveries++;}orientation_ready=false;
            portENTER_CRITICAL(&mux);view.errors++;view.orientation_ready=false;view.orientation_epoch=epoch;view.recovery_count=recoveries;portEXIT_CRITICAL(&mux);
        }
        if(now>=next_mag){
            next_mag=now+100000;
            for(unsigned i=0;i<2;i++)if(mag_ok[i]){
                struct bmm150_mag_data data;
                int rc=bmm150_read_mag_data(&data,&mag[i]);
                const uint8_t *r=mag_raw_bytes[i];
                int16_t raw[3]={((int8_t)r[1])*32+(r[0]>>3),((int8_t)r[3])*32+(r[2]>>3),((int8_t)r[5])*128+(r[4]>>1)};
                uint16_t rhall=((uint16_t)r[7]<<6)|(r[6]>>2);
                uint8_t valid=0;
                if(rc==BMM150_OK&&rhall&&mag[i].trim_data.dig_xyz1){
                    if(raw[0]!=BMM150_OVERFLOW_ADCVAL_XYAXES_FLIP&&isfinite(data.x))valid|=1;
                    if(raw[1]!=BMM150_OVERFLOW_ADCVAL_XYAXES_FLIP&&isfinite(data.y))valid|=2;
                    if(raw[2]!=BMM150_OVERFLOW_ADCVAL_ZAXIS_HALL&&mag[i].trim_data.dig_z1&&mag[i].trim_data.dig_z2&&isfinite(data.z))valid|=4;
                }
                portENTER_CRITICAL(&mux);
                view.mag_valid_axes[i]=valid;
                if(rc==BMM150_OK){
                    view.mag_ut[i][0]=data.x;view.mag_ut[i][1]=data.y;view.mag_ut[i][2]=data.z;
                    memcpy(view.mag_raw[i],raw,sizeof(raw));view.mag_rhall[i]=rhall;view.mag_sample_us[i]=esp_timer_get_time();
                }
                else view.errors++;
                portEXIT_CRITICAL(&mux);
            }
        }
        vTaskDelayUntil(&tick,pdMS_TO_TICKS(10)>0?pdMS_TO_TICKS(10):1);
    }
}
esp_err_t oft_motion_start(void)
{
    if(!oft_board_i2c())return ESP_ERR_INVALID_STATE;
    events=xQueueCreate(32,sizeof(oft_motion_event_t));if(!events)return ESP_ERR_NO_MEM;
    gpio_set_level(8,0);gpio_config_t cfg={.pin_bit_mask=1ULL<<8,.mode=GPIO_MODE_OUTPUT};
    esp_err_t rc=gpio_config(&cfg);if(rc!=ESP_OK)return rc;
    const esp_timer_create_args_t timer={.callback=haptic_tick,.name="oft_haptic",.skip_unhandled_events=true};
    rc=esp_timer_create(&timer,&haptic_timer);if(rc!=ESP_OK)return rc;
    rc=esp_timer_start_periodic(haptic_timer,2000);
    if(rc!=ESP_OK){esp_timer_delete(haptic_timer);return rc;}
    if(xTaskCreatePinnedToCore(task,"oft_motion",6144,NULL,6,NULL,0)!=pdPASS){
        esp_timer_stop(haptic_timer);esp_timer_delete(haptic_timer);vQueueDelete(events);events=NULL;return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}
