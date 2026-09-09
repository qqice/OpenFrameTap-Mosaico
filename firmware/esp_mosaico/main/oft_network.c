#include "oft_network.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <string.h>
#include <stdio.h>
#include <stdatomic.h>

typedef struct {char ssid[33],password[65];} join_t;
static QueueHandle_t requests;
static atomic_bool stop_requested,link_down,got_ip;
static atomic_int disconnect_reason;
static portMUX_TYPE mux=portMUX_INITIALIZER_UNLOCKED;
static oft_network_snapshot_t view;
static const char *TAG="oft_wifi";
static bool initialized,ram_storage,started,wanted;
static int64_t deadline,retry_at;

void oft_network_snapshot(oft_network_snapshot_t *out){portENTER_CRITICAL(&mux);*out=view;portEXIT_CRITICAL(&mux);}
static void state(oft_network_state_t s)
{portENTER_CRITICAL(&mux);view.state=s;portEXIT_CRITICAL(&mux);ESP_LOGI(TAG,"state=%u",s);}
static void clear_join(join_t *j){volatile uint8_t *p=(volatile uint8_t*)j;for(size_t i=0;i<sizeof(*j);i++)p[i]=0;}
static void event_handler(void *ctx,esp_event_base_t base,int32_t id,void *event)
{
    (void)ctx;
    if(base==WIFI_EVENT&&id==WIFI_EVENT_STA_DISCONNECTED){
        atomic_store(&disconnect_reason,((wifi_event_sta_disconnected_t*)event)->reason);
        atomic_store(&link_down,true);
    }else if(base==IP_EVENT&&id==IP_EVENT_STA_GOT_IP){
        ip_event_got_ip_t *e=event;
        portENTER_CRITICAL(&mux);snprintf(view.ip,sizeof(view.ip),IPSTR,IP2STR(&e->ip_info.ip));portEXIT_CRITICAL(&mux);
        atomic_store(&got_ip,true);
    }
}
static void disconnect_wifi(bool fault)
{
    wanted=false;deadline=retry_at=0;
    if(started){esp_wifi_disconnect();esp_wifi_stop();started=false;}
    if(ram_storage){
        wifi_config_t empty={0};esp_wifi_set_config(WIFI_IF_STA,&empty); /* RAM storage only */
    }
    portENTER_CRITICAL(&mux);view.ip[0]=0;portEXIT_CRITICAL(&mux);
    atomic_store(&got_ip,false);atomic_store(&link_down,false);
    state(fault?OFT_NETWORK_FAULT:OFT_NETWORK_OFF);
}
static esp_err_t initialize_wifi(void)
{
    esp_err_t rc;
    if(!initialized){
        wifi_init_config_t cfg=WIFI_INIT_CONFIG_DEFAULT();
        /* IDR bursts can exceed the default32 dynamic RX buffers before socket
           delivery. Follow the SDK's static>=BA-window recommendation. Dynamic
           buffers prefer PSRAM through the verified project allocation setting;
           counts stay bounded, and DMA/static storage is still internal. */
        cfg.static_rx_buf_num=16;
        cfg.dynamic_rx_buf_num=64;
        cfg.rx_ba_win=16;
        ESP_LOGI(TAG,"RX budget static=%d dynamic=%d ba_window=%d",cfg.static_rx_buf_num,cfg.dynamic_rx_buf_num,cfg.rx_ba_win);
        rc=esp_wifi_init(&cfg);if(rc!=ESP_OK)return rc;
        initialized=true;
    }
    rc=esp_wifi_set_storage(WIFI_STORAGE_RAM);if(rc!=ESP_OK)return rc;
    ram_storage=true;
    rc=esp_wifi_set_mode(WIFI_MODE_STA);if(rc!=ESP_OK)return rc;
    /* Interactive control/40Hz ACK must not wait for modem-sleep DTIM bursts.
       This is RAM-only on our owned STA, not a persistent network setting. */
    return esp_wifi_set_ps(WIFI_PS_NONE);
}
static void manager(void *unused)
{
    (void)unused;join_t join;
    for(;;){
        if(atomic_exchange(&stop_requested,false)){
            while(xQueueReceive(requests,&join,0)==pdTRUE)clear_join(&join);
            disconnect_wifi(false);
        }
        if(xQueueReceive(requests,&join,pdMS_TO_TICKS(20))==pdTRUE){
            disconnect_wifi(false);
            esp_err_t rc=initialize_wifi();
            if(rc==ESP_OK){
                wifi_config_t cfg={0};
                memcpy(cfg.sta.ssid,join.ssid,strlen(join.ssid));
                memcpy(cfg.sta.password,join.password,strlen(join.password));
                cfg.sta.threshold.authmode=WIFI_AUTH_WPA2_PSK;
                cfg.sta.pmf_cfg.capable=true;
                rc=esp_wifi_set_config(WIFI_IF_STA,&cfg);
                volatile uint8_t *p=(volatile uint8_t*)&cfg;for(size_t i=0;i<sizeof(cfg);i++)p[i]=0;
            }
            clear_join(&join);
            if(rc==ESP_OK){rc=esp_wifi_start();started=rc==ESP_OK;}
            if(rc==ESP_OK){
                wanted=true;
                portENTER_CRITICAL(&mux);view.attempts=1;view.last_reason=0;portEXIT_CRITICAL(&mux);
                state(OFT_NETWORK_JOINING);deadline=esp_timer_get_time()+20000000;
                rc=esp_wifi_connect();
            }
            if(rc!=ESP_OK){ESP_LOGE(TAG,"STA setup failed: %s",esp_err_to_name(rc));disconnect_wifi(true);}
        }
        int64_t now=esp_timer_get_time();
        if(atomic_exchange(&got_ip,false)&&wanted){
            wifi_ap_record_t ap;
            if(esp_wifi_sta_get_ap_info(&ap)==ESP_OK){
                ESP_LOGI(TAG,"STA signal rssi=%d channel=%u",ap.rssi,ap.primary);
                deadline=retry_at=0;state(OFT_NETWORK_CONNECTED);
            }
        }
        if(atomic_exchange(&link_down,false)&&wanted){
            portENTER_CRITICAL(&mux);view.last_reason=atomic_load(&disconnect_reason);view.ip[0]=0;portEXIT_CRITICAL(&mux);
            if(view.state==OFT_NETWORK_CONNECTED||view.attempts>=3){
                /* Established-session loss must not resume motion by reconnect. */
                disconnect_wifi(true);
            }else{state(OFT_NETWORK_JOINING);retry_at=now+1000000;deadline=now+20000000;}
        }
        if(wanted&&retry_at&&now>=retry_at){
            retry_at=0;portENTER_CRITICAL(&mux);view.attempts++;portEXIT_CRITICAL(&mux);
            if(esp_wifi_connect()!=ESP_OK)disconnect_wifi(true);
        }
        if(wanted&&deadline&&now>=deadline)disconnect_wifi(true);
    }
}
esp_err_t oft_network_start(void)
{
    if(requests)return ESP_ERR_INVALID_STATE;
    esp_err_t rc=esp_netif_init();if(rc!=ESP_OK)return rc;
    rc=esp_event_loop_create_default();if(rc!=ESP_OK&&rc!=ESP_ERR_INVALID_STATE)return rc;
    if(!esp_netif_create_default_wifi_sta())return ESP_ERR_NO_MEM;
    rc=esp_event_handler_register(WIFI_EVENT,WIFI_EVENT_STA_DISCONNECTED,event_handler,NULL);if(rc!=ESP_OK)return rc;
    rc=esp_event_handler_register(IP_EVENT,IP_EVENT_STA_GOT_IP,event_handler,NULL);if(rc!=ESP_OK)return rc;
    requests=xQueueCreate(1,sizeof(join_t));if(!requests)return ESP_ERR_NO_MEM;
    return xTaskCreate(manager,"oft_wifi",4096,NULL,4,NULL)==pdPASS?ESP_OK:ESP_ERR_NO_MEM;
}
bool oft_network_join(const char *ssid,const char *password)
{
    if(!requests||!ssid||!password||strlen(ssid)>32||strlen(password)>64||strlen(password)<8)return false;
    /* BLE manager orders stop/join calls; a newer join supersedes an earlier
       queued stop. The worker still disconnects the previous STA first. */
    atomic_store(&stop_requested,false);
    join_t request={0};strlcpy(request.ssid,ssid,sizeof(request.ssid));strlcpy(request.password,password,sizeof(request.password));
    bool ok=xQueueSend(requests,&request,0)==pdTRUE;clear_join(&request);return ok;
}
void oft_network_stop(void){atomic_store(&stop_requested,true);}
