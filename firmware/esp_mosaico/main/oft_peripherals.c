#include "oft_peripherals.h"
#include "oft_board.h"
#include "oft_udp.h"
#include "oft_capture.h"
#include "oft_gauge.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include <stdio.h>
static portMUX_TYPE mux=portMUX_INITIALIZER_UNLOCKED;
static oft_peripherals_snapshot_t view;
void oft_peripherals_snapshot(oft_peripherals_snapshot_t *out)
{portENTER_CRITICAL(&mux);*out=view;portEXIT_CRITICAL(&mux);}
static void button_task(void *arg)
{
    (void)arg;oft_button_filter_t filter={.blocked=true}; // Held at boot never shoots.
    for(;;){
        int64_t now=esp_timer_get_time();
        if(oft_button_step(&filter,gpio_get_level(GPIO_NUM_7)==0,now)){
            bool queued=oft_udp_shutter(now);
            portENTER_CRITICAL(&mux);view.button_presses++;view.button_queued+=queued;portEXIT_CRITICAL(&mux);
            printf("OFT_FUNCTION us=%lld press=%u queued=%d\n",(long long)now,view.button_presses,queued);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
static bool word(i2c_master_dev_handle_t device,uint8_t reg,uint16_t *out)
{
    uint8_t p[2];esp_err_t rc=i2c_master_transmit_receive(device,&reg,1,p,2,20);
    esp_rom_delay_us(100); // TI bus-free timing; battery task only, no configuration write.
    if(rc!=ESP_OK)return false;
    *out=p[0]|((uint16_t)p[1]<<8);return true;
}
static void battery_task(void *arg)
{
    (void)arg;i2c_master_dev_handle_t device=NULL;
    i2c_device_config_t cfg={.dev_addr_length=I2C_ADDR_BIT_LEN_7,.device_address=0x55,.scl_speed_hz=100000};
    if(i2c_master_bus_add_device(oft_board_i2c(),&cfg,&device)!=ESP_OK){vTaskDelete(NULL);return;}
    for(;;){
        uint16_t soc=0,voltage=0,current=0,design=0,status=0,flags=0,remaining=0,fcc=0;
        oft_gauge_poll(device);
        /* These periodic measurements remain read-only. Configuration is only
           through the typed RAM transaction above, never the charger. */
        bool ok=word(device,0x2c,&soc)&&word(device,0x08,&voltage)&&word(device,0x0c,&current)&&word(device,0x3c,&design)&&word(device,0x3a,&status)&&word(device,0x0a,&flags)&&word(device,0x10,&remaining)&&word(device,0x12,&fcc);
        ok=ok&&soc<=100&&voltage>=2000&&voltage<=5000;
        bool configured=ok&&oft_gauge_ready()&&design==130&&(status&0x20)&&!(status&0x400)&&(flags&0x4000)&&!(flags&0x2000);
        oft_gauge_observe(ok,soc,voltage,(int16_t)current,design,status,flags,remaining,fcc);
        printf("OFT_GAUGE_CAPACITY valid=%d raw_rm=%u raw_fcc=%u raw_design=%u scale=%u\n",ok,remaining,fcc,design,configured?2:0);
        portENTER_CRITICAL(&mux);view.battery_valid=ok;view.gauge_configured=configured;
        if(ok){view.percent=soc;view.millivolts=voltage;view.milliamps=(int16_t)current/(configured?2:1);view.design_mah=design/(configured?2:1);view.battery_us=esp_timer_get_time();}
        else view.read_errors++;
        portEXIT_CRITICAL(&mux);
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
esp_err_t oft_peripherals_start(void)
{
    /* Official Mosaico V1.0 Function Button: GPIO7, active-low. */
    gpio_config_t cfg={.pin_bit_mask=1ULL<<7,.mode=GPIO_MODE_INPUT,.pull_up_en=GPIO_PULLUP_ENABLE};
    esp_err_t rc=gpio_config(&cfg);if(rc!=ESP_OK)return rc;
    if(xTaskCreatePinnedToCore(button_task,"oft_button",3072,NULL,5,NULL,0)!=pdPASS)return ESP_ERR_NO_MEM;
    if(xTaskCreatePinnedToCore(battery_task,"oft_battery",6144,NULL,2,NULL,0)!=pdPASS)return ESP_ERR_NO_MEM;
    return ESP_OK;
}
