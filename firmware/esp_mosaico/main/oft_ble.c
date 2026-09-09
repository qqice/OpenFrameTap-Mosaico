#include "oft_ble.h"
#include "oft_pocket3.h"
#include "oft_network.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include <string.h>
#include <stdatomic.h>

/* Callbacks only enqueue bounded work; BLE task owns discovery, auth,
   credentials and every FFF5 write. UI reads copied snapshots, never secrets. */
enum {EV_SYNC,EV_ADV,EV_CONNECTED,EV_MTU,EV_SERVICE,EV_CHAR,EV_DSC,EV_SUBSCRIBED,EV_NOTIFY,EV_RESET};
typedef struct {
    uint8_t kind;int status;
    uint16_t connection,a,b,c,size;
    uint8_t flags,address_type,address[6];int8_t rssi;
    uint8_t data[514];
} event_t;
static QueueHandle_t events;
static portMUX_TYPE mux=portMUX_INITIALIZER_UNLOCKED;
static oft_ble_snapshot_t view;
static atomic_int user_request=-1; /* -1 none, -2 disconnect, -3 scan, >=0 choice */
static atomic_bool overflow,disconnected;
static uint16_t connection=BLE_HS_CONN_HANDLE_NONE,svc_start,svc_end,notify_handle,write_handle,notify_end,cccd;
static uint16_t sequence=0x9000,pending_sequence;
static oft_p3_command_t pending_command=OFT_P3_COMMAND_COUNT;
static uint8_t own_address_type;
static bool synced,subscribed,heartbeat_enabled,closing;
static int64_t deadline,next_heartbeat,query_due;
static unsigned ssid_attempts,pair_requests;
static oft_duml_stream_t stream;
static char ssid[33],password[65];
static uint8_t remembered[7];static bool have_remembered,auto_select=true;
static const char *TAG="oft_ble";

void oft_ble_snapshot(oft_ble_snapshot_t *out){portENTER_CRITICAL(&mux);*out=view;portEXIT_CRITICAL(&mux);}
static void state(oft_ble_state_t s,const char *detail)
{
    portENTER_CRITICAL(&mux);view.state=s;strlcpy(view.detail,detail,sizeof(view.detail));portEXIT_CRITICAL(&mux);
    ESP_LOGI(TAG,"state=%u %s",s,detail);
}
static void erase_secrets(void)
{
    volatile char *p=ssid;for(size_t i=0;i<sizeof(ssid);i++)p[i]=0;
    p=password;for(size_t i=0;i<sizeof(password);i++)p[i]=0;
}
static void stop_link(bool fault,const char *detail)
{
    heartbeat_enabled=false;subscribed=false;pending_command=OFT_P3_COMMAND_COUNT;deadline=0;query_due=0;
    erase_secrets();auto_select=false;closing=true;
    oft_network_stop();
    /* Link termination removes the CCCD subscription too, including when a
       previous GATT operation failed. Never remove peer bonding records. */
    if(connection!=BLE_HS_CONN_HANDLE_NONE)ble_gap_terminate(connection,BLE_ERR_REM_USER_CONN_TERM);
    else if(ble_gap_conn_active())ble_gap_conn_cancel();
    if(ble_gap_disc_active())ble_gap_disc_cancel();
    if(fault){portENTER_CRITICAL(&mux);view.errors++;portEXIT_CRITICAL(&mux);}
    state(fault?OFT_BLE_FAULT:OFT_BLE_IDLE,detail);
}
static void enqueue(const event_t *e){if(xQueueSend(events,e,0)!=pdTRUE&&e->kind!=EV_ADV)atomic_store(&overflow,true);}
static uint16_t uuid16(const ble_uuid_any_t *uuid)
{
    if(uuid->u.type==BLE_UUID_TYPE_16)return uuid->u16.value;
    if(uuid->u.type==BLE_UUID_TYPE_128){
        static const uint8_t base[]={0xfb,0x34,0x9b,0x5f,0x80,0,0,0x80,0,0x10,0,0};
        const uint8_t *p=uuid->u128.value;
        if(!memcmp(p,base,12)&&p[14]==0&&p[15]==0)return p[12]|((uint16_t)p[13]<<8);
    }
    return 0;
}
static int gap_event(struct ble_gap_event *e,void *unused)
{
    (void)unused;event_t v={0};
    switch(e->type){
    case BLE_GAP_EVENT_DISC:
        v.kind=EV_ADV;v.address_type=e->disc.addr.type;memcpy(v.address,e->disc.addr.val,6);
        v.rssi=e->disc.rssi;v.size=e->disc.length_data;
        if(v.size<=31){memcpy(v.data,e->disc.data,v.size);enqueue(&v);}break;
    case BLE_GAP_EVENT_CONNECT:
        v.kind=EV_CONNECTED;v.status=e->connect.status;v.connection=e->connect.conn_handle;enqueue(&v);break;
    case BLE_GAP_EVENT_DISCONNECT:
        atomic_store(&disconnected,true);break;
    case BLE_GAP_EVENT_NOTIFY_RX:
        v.kind=EV_NOTIFY;v.connection=e->notify_rx.conn_handle;v.a=e->notify_rx.attr_handle;
        v.size=OS_MBUF_PKTLEN(e->notify_rx.om);
        if(v.size>sizeof(v.data)||os_mbuf_copydata(e->notify_rx.om,0,v.size,v.data))atomic_store(&overflow,true);
        else enqueue(&v);
        break;
    case BLE_GAP_EVENT_PASSKEY_ACTION:
        v.kind=EV_RESET;v.status=BLE_HS_EAUTHEN;enqueue(&v);break;
    default:break;
    }
    return 0;
}
static int mtu_cb(uint16_t c,const struct ble_gatt_error *e,uint16_t mtu,void *u)
{(void)u;event_t v={.kind=EV_MTU,.status=e->status,.connection=c,.a=mtu};enqueue(&v);return 0;}
static int service_cb(uint16_t c,const struct ble_gatt_error *e,const struct ble_gatt_svc *s,void *u)
{(void)u;event_t v={.kind=EV_SERVICE,.status=e->status,.connection=c};if(!e->status){v.a=s->start_handle;v.b=s->end_handle;}enqueue(&v);return 0;}
static int char_cb(uint16_t c,const struct ble_gatt_error *e,const struct ble_gatt_chr *ch,void *u)
{(void)u;event_t v={.kind=EV_CHAR,.status=e->status,.connection=c};if(!e->status){v.a=uuid16(&ch->uuid);v.b=ch->val_handle;v.c=ch->def_handle;v.flags=ch->properties;}enqueue(&v);return 0;}
static int dsc_cb(uint16_t c,const struct ble_gatt_error *e,uint16_t h,const struct ble_gatt_dsc *d,void *u)
{(void)h;(void)u;event_t v={.kind=EV_DSC,.status=e->status,.connection=c};if(!e->status){v.a=uuid16(&d->uuid);v.b=d->handle;}enqueue(&v);return 0;}
static int sub_cb(uint16_t c,const struct ble_gatt_error *e,struct ble_gatt_attr *a,void *u)
{(void)a;(void)u;event_t v={.kind=EV_SUBSCRIBED,.status=e->status,.connection=c};enqueue(&v);return 0;}
static void reset_cb(int reason){event_t e={.kind=EV_RESET,.status=reason};enqueue(&e);}
static void sync_cb(void){event_t e={.kind=EV_SYNC};enqueue(&e);}
static void host_task(void *p){(void)p;nimble_port_run();nimble_port_freertos_deinit();}

static void scan(void)
{
    if(!synced||connection!=BLE_HS_CONN_HANDLE_NONE||ble_gap_conn_active())return;
    if(ble_gap_disc_active())ble_gap_disc_cancel();
    closing=false;portENTER_CRITICAL(&mux);view.choice_count=0;memset(view.choices,0,sizeof(view.choices));portEXIT_CRITICAL(&mux);
    /* Discovery includes standard SCAN_REQ/SCAN_RSP: names/services may be
       absent from the primary advertisement. No connection or DJI command
       occurs until an explicit selection (or an exact remembered identity). */
    struct ble_gap_disc_params p={.passive=0,.itvl=160,.window=80,.filter_duplicates=1};
    int rc=ble_gap_disc(own_address_type,BLE_HS_FOREVER,&p,gap_event,NULL);
    if(rc)stop_link(true,"BLE scan failed");else state(OFT_BLE_SCANNING,"Tap your Pocket 3 to connect");
}
static void advertisement(const event_t *e)
{
    if(view.state!=OFT_BLE_SCANNING)return;
    portENTER_CRITICAL(&mux);view.advertisements++;portEXIT_CRITICAL(&mux);
    struct ble_hs_adv_fields f;
    if(ble_hs_adv_parse_fields(&f,e->data,e->size))return;
    bool fff0=false;for(unsigned i=0;i<f.num_uuids16;i++)if(f.uuids16[i].value==0xfff0)fff0=true;
    bool dji=f.mfg_data_len>=2&&f.mfg_data[0]==0xaa&&f.mfg_data[1]==8;
    if(dji||fff0){
        char raw[63];for(unsigned j=0;j<e->size;j++)snprintf(raw+j*2,3,"%02x",e->data[j]);raw[e->size*2]=0;
        bool known=have_remembered&&remembered[0]==e->address_type&&!memcmp(remembered+1,e->address,6);
        ESP_LOGI(TAG,"advertisement known=%d type=%u name=%.*s raw=%s",known,e->address_type,(int)f.name_len,f.name? (const char*)f.name:"",raw);
    }
    portENTER_CRITICAL(&mux);view.dji_advertisements+=dji;view.fff0_advertisements+=fff0;portEXIT_CRITICAL(&mux);
    /* Model signature from the already verified Pocket 3 advertisement;
       FFF0 alone is insufficient to operate an unrelated DJI device. */
    bool model=oft_p3_advertisement(f.mfg_data,f.mfg_data_len);
    bool named=f.name_len>=11&&memcmp(f.name,"OsmoPocket3",11)==0;
    if(!model&&!(fff0&&named))return;
    unsigned i;
    portENTER_CRITICAL(&mux);
    for(i=0;i<view.choice_count;i++)if(view.choices[i].address_type==e->address_type&&memcmp(view.choices[i].address,e->address,6)==0)break;
    if(i<OFT_BLE_CHOICES){
        if(i==view.choice_count)view.choice_count++;
        oft_ble_choice_t *c=&view.choices[i];c->address_type=e->address_type;memcpy(c->address,e->address,6);c->rssi=e->rssi;
        if(f.name_len){unsigned n=f.name_len<31?f.name_len:31;memcpy(c->name,f.name,n);c->name[n]=0;}
        else if(!c->name[0])strlcpy(c->name,"Pocket 3 (advertisement)",sizeof(c->name));
    }
    portEXIT_CRITICAL(&mux);
    if(i<OFT_BLE_CHOICES&&have_remembered&&auto_select&&remembered[0]==e->address_type&&memcmp(remembered+1,e->address,6)==0){
        auto_select=false;atomic_store(&user_request,(int)i);
    }
}
static void remember(const oft_ble_choice_t *c)
{
    nvs_handle_t n;
    if(nvs_open("oft_mosaico",NVS_READWRITE,&n)!=ESP_OK)return;
    uint8_t identity[7];identity[0]=c->address_type;memcpy(identity+1,c->address,6);
    if(nvs_set_blob(n,"peer",identity,sizeof(identity))==ESP_OK&&nvs_commit(n)==ESP_OK){memcpy(remembered,identity,7);have_remembered=true;}
    nvs_close(n); /* only identity; credentials never enter NVS */
}
static bool send_command(oft_p3_command_t cmd,bool reply,uint16_t seq_override)
{
    if(!subscribed||connection==BLE_HS_CONN_HANDLE_NONE)return false;
    if(cmd==OFT_P3_PAIR_STATUS&&pair_requests++>=2){stop_link(true,"Pairing attempt budget exhausted");return false;}
    uint8_t raw[64];uint16_t seq=cmd==OFT_P3_PAIR_ACK?seq_override:sequence++;
    size_t n=oft_p3_build(cmd,seq,0,0,raw,sizeof(raw));
    if(!n||!oft_p3_allowed(raw,n,true,false)){stop_link(true,"BLE command policy rejected");return false;}
    size_t fragment=ble_att_mtu(connection)-3;
    if(fragment<20){stop_link(true,"Invalid ATT MTU");return false;}
    for(size_t offset=0;offset<n;offset+=fragment){
        size_t part=n-offset<fragment?n-offset:fragment;
        if(ble_gattc_write_no_rsp_flat(connection,write_handle,raw+offset,part)){
            stop_link(true,"BLE write failed; stopped");return false;
        }
        portENTER_CRITICAL(&mux);view.writes++;portEXIT_CRITICAL(&mux);
    }
    /* No raw payload/credentials or identifiers in diagnostic output. */
    ESP_LOGI(TAG,"send profile=%u seq=%u bytes=%u",cmd,seq,(unsigned)n);
    if(reply){pending_command=cmd;pending_sequence=seq;deadline=esp_timer_get_time()+15000000;}
    return true;
}
static void authenticated(void)
{
    state(OFT_BLE_CREDENTIALS,"Paired; waking Pocket hotspot");heartbeat_enabled=true;
    next_heartbeat=0;query_due=esp_timer_get_time()+600000;ssid_attempts=0;deadline=0;
}
static void notification_frame(const oft_duml_frame_t *f,void *unused)
{
    (void)unused;if(closing)return;uint8_t battery;
    portENTER_CRITICAL(&mux);view.frames++;
    if(oft_p3_battery(f,&battery)){view.battery_valid=true;view.battery=battery;view.battery_updated_us=esp_timer_get_time();}
    portEXIT_CRITICAL(&mux);
    if(view.state==OFT_BLE_CONFIRM&&f->sender==7&&f->receiver==2&&f->cmd_set==7&&f->cmd_id==0x46&&f->flags==0x40){
        if(f->payload_size!=1||f->payload[0]!=1){stop_link(true,"Unexpected Pocket confirmation");return;}
        if(!send_command(OFT_P3_PAIR_ACK,false,f->sequence))return;
        if(!send_command(OFT_P3_PAIR_FINISH,false,0))return;
        state(OFT_BLE_AUTHENTICATING,"Verifying application pairing");
        /* Re-query known status; don't infer success from a successful write. */
        send_command(OFT_P3_PAIR_STATUS,true,0);return;
    }
    if(!oft_p3_ble_reply_matches(pending_command,pending_sequence,f))return;
    portENTER_CRITICAL(&mux);view.matched_replies++;view.last_reply_flags=f->flags;portEXIT_CRITICAL(&mux);
    ESP_LOGI(TAG,"reply profile=%u seq=%u flags=%02x payload_bytes=%u",pending_command,f->sequence,f->flags,(unsigned)f->payload_size);
    oft_p3_command_t cmd=pending_command;pending_command=OFT_P3_COMMAND_COUNT;deadline=0;
    if(cmd==OFT_P3_PAIR_STATUS){
        if(f->payload_size==2&&f->payload[0]==0){portENTER_CRITICAL(&mux);view.pairing_status=f->payload[1];portEXIT_CRITICAL(&mux);}
        if(f->payload_size==2&&f->payload[0]==0&&f->payload[1]==1)authenticated();
        else if(f->payload_size==2&&f->payload[0]==0&&f->payload[1]==2){state(OFT_BLE_CONFIRM,"Confirm pairing on Pocket screen");deadline=esp_timer_get_time()+60000000;}
        else stop_link(true,"Unexpected pairing status; stopped");
    }else if(cmd==OFT_P3_SSID){
        if(!oft_p3_wifi_string(f->payload,f->payload_size,false,ssid,sizeof(ssid))){stop_link(true,"Invalid Pocket SSID response");return;}
        send_command(OFT_P3_PASSWORD,true,0);
    }else if(cmd==OFT_P3_PASSWORD){
        if(!oft_p3_wifi_string(f->payload,f->payload_size,true,password,sizeof(password))){stop_link(true,"Invalid Pocket password response");return;}
        if(!oft_network_join(ssid,password)){stop_link(true,"Hotspot join queue unavailable");return;}
        state(OFT_BLE_READY,"Paired; joining hotspot; controls MOCK");
        ESP_LOGI(TAG,"credentials validated in RAM; SSID/password omitted");
        erase_secrets(); /* network owner now has the only credential copy */
    }
}
static void handle(const event_t *e)
{
    int rc=0;
    if(e->kind==EV_SYNC){
        rc=ble_hs_util_ensure_addr(0);if(!rc)rc=ble_hs_id_infer_auto(0,&own_address_type);
        if(rc){stop_link(true,"BLE identity unavailable");return;}synced=true;scan();return;
    }
    if(e->kind==EV_ADV){advertisement(e);return;}
    if(e->kind==EV_RESET){stop_link(true,"BLE security/host error; stopped");return;}
    if(e->kind==EV_CONNECTED){
        if(e->status){stop_link(true,"Connection failed; tap Scan to retry");return;}
        struct ble_gap_conn_desc desc;
        if(ble_gap_conn_find(e->connection,&desc)){stop_link(true,"Link closed before discovery");return;}
        connection=e->connection;
        if(closing){ble_gap_terminate(connection,BLE_ERR_REM_USER_CONN_TERM);return;}
        state(OFT_BLE_DISCOVERING,"Discovering FFF4 / FFF5");deadline=esp_timer_get_time()+15000000;
        rc=ble_gattc_exchange_mtu(connection,mtu_cb,NULL);
    }else{
        if(e->connection!=connection||closing)return;
        switch(e->kind){
        case EV_MTU:
            if(e->status){stop_link(true,"ATT MTU exchange failed");return;}
            portENTER_CRITICAL(&mux);view.mtu=e->a;portEXIT_CRITICAL(&mux);
            rc=ble_gattc_disc_svc_by_uuid(connection,BLE_UUID16_DECLARE(0xfff0),service_cb,NULL);break;
        case EV_SERVICE:
            if(!e->status){if(svc_start){stop_link(true,"Ambiguous FFF0 service");return;}svc_start=e->a;svc_end=e->b;}
            else if(e->status==BLE_HS_EDONE&&svc_start)rc=ble_gattc_disc_all_chrs(connection,svc_start,svc_end,char_cb,NULL);
            else rc=e->status?e->status:BLE_HS_ENOENT;
            break;
        case EV_CHAR:
            if(!e->status){
                if(notify_handle&&e->c>notify_handle&&e->c-1<notify_end)notify_end=e->c-1;
                if(e->a==0xfff4&&(e->flags&BLE_GATT_CHR_PROP_NOTIFY)){notify_handle=e->b;notify_end=svc_end;}
                if(e->a==0xfff5&&(e->flags&BLE_GATT_CHR_PROP_WRITE_NO_RSP))write_handle=e->b;
            }else if(e->status==BLE_HS_EDONE&&notify_handle&&write_handle&&notify_end>notify_handle)
                rc=ble_gattc_disc_all_dscs(connection,notify_handle,notify_end,dsc_cb,NULL);
            else rc=e->status?e->status:BLE_HS_ENOENT;
            break;
        case EV_DSC:
            if(!e->status){if(e->a==0x2902){if(cccd){stop_link(true,"Ambiguous FFF4 CCCD");return;}cccd=e->b;}}
            else if(e->status==BLE_HS_EDONE&&cccd){const uint8_t enabled[]={1,0};rc=ble_gattc_write_flat(connection,cccd,enabled,2,sub_cb,NULL);}
            else rc=e->status?e->status:BLE_HS_ENOENT;
            break;
        case EV_SUBSCRIBED:
            if(e->status){rc=e->status;break;}
            subscribed=true;state(OFT_BLE_AUTHENTICATING,"FFF4 subscribed; application pairing");
            if(send_command(OFT_P3_OPEN,false,0))send_command(OFT_P3_PAIR_STATUS,true,0);
            break;
        case EV_NOTIFY:
            if(e->a==notify_handle&&subscribed){
                portENTER_CRITICAL(&mux);view.notifications++;portEXIT_CRITICAL(&mux);
                oft_duml_feed(&stream,e->data,e->size,notification_frame,NULL);
                /* A callback can fault the session. Clear sensitive residuals
                   only after feed returns; never mutate its buffer mid-frame. */
                if(closing)memset(&stream,0,sizeof(stream));
            }break;
        default:break;
        }
    }
    if(rc){ESP_LOGW(TAG,"GATT operation failed status=%d",rc);stop_link(true,"GATT discovery/subscription failed");}
}
static void manager(void *unused)
{
    (void)unused;event_t e;
    for(;;){
        if(atomic_exchange(&overflow,false))stop_link(true,"BLE event queue overflow; stopped");
        if(atomic_exchange(&disconnected,false)){
            connection=BLE_HS_CONN_HANDLE_NONE;portENTER_CRITICAL(&mux);view.disconnects++;view.battery_valid=false;portEXIT_CRITICAL(&mux);
            if(!closing)stop_link(true,"Pocket disconnected; tap Scan");
        }
        int choice=atomic_exchange(&user_request,-1);
        if(choice==-2)stop_link(false,"Disconnected; tap Scan to reconnect");
        else if(choice==-3)scan();
        else if(choice>=0&&view.state==OFT_BLE_SCANNING&&(unsigned)choice<view.choice_count){
            oft_ble_choice_t c=view.choices[choice];remember(&c);auto_select=false;
            ble_gap_disc_cancel();closing=false;erase_secrets();memset(&stream,0,sizeof(stream));pair_requests=0;
            svc_start=svc_end=notify_handle=write_handle=notify_end=cccd=0;
            ble_addr_t address={.type=c.address_type};memcpy(address.val,c.address,6);
            state(OFT_BLE_CONNECTING,"Connecting selected Pocket 3");deadline=esp_timer_get_time()+15000000;
            int rc=ble_gap_connect(own_address_type,&address,10000,NULL,gap_event,NULL);
            if(rc)stop_link(true,"BLE connect request failed");
        }
        if(xQueueReceive(events,&e,pdMS_TO_TICKS(20))==pdTRUE)handle(&e);
        int64_t now=esp_timer_get_time();
        if(!closing&&heartbeat_enabled&&now>=next_heartbeat){next_heartbeat=now+1000000;send_command(OFT_P3_BLE_HEARTBEAT,false,0);}
        if(!closing&&query_due&&now>=query_due){query_due=0;ssid_attempts++;send_command(OFT_P3_SSID,true,0);}
        if(!closing&&deadline&&now>=deadline){
            if(pending_command==OFT_P3_SSID&&ssid_attempts<2){pending_command=OFT_P3_COMMAND_COUNT;deadline=0;query_due=now+1000000;}
            else if(pending_command==OFT_P3_PAIR_STATUS)stop_link(true,"Pairing status response timeout");
            else if(pending_command==OFT_P3_SSID)stop_link(true,"SSID response timeout");
            else if(pending_command==OFT_P3_PASSWORD)stop_link(true,"Password response timeout");
            else stop_link(true,"BLE discovery/confirmation timeout");
        }
    }
}
bool oft_ble_select(unsigned index)
{
    if(index>=OFT_BLE_CHOICES)return false;
    int expected=-1;return atomic_compare_exchange_strong(&user_request,&expected,(int)index);
}
void oft_ble_disconnect(void){atomic_store(&user_request,-2);}
void oft_ble_rescan(void){atomic_store(&user_request,-3);}
esp_err_t oft_ble_start(void)
{
    if(events)return ESP_ERR_INVALID_STATE;
    esp_err_t rc=nvs_flash_init();
    if(rc!=ESP_OK){state(OFT_BLE_FAULT,"NVS unavailable; not erased");return rc;}
    nvs_handle_t n;
    if(nvs_open("oft_mosaico",NVS_READONLY,&n)==ESP_OK){size_t len=sizeof(remembered);have_remembered=nvs_get_blob(n,"peer",remembered,&len)==ESP_OK&&len==7;nvs_close(n);}
    events=xQueueCreate(16,sizeof(event_t));if(!events)return ESP_ERR_NO_MEM;
    rc=nimble_port_init();if(rc!=ESP_OK){state(OFT_BLE_FAULT,"NimBLE initialization failed");return rc;}
    ble_hs_cfg.sync_cb=sync_cb;ble_hs_cfg.reset_cb=reset_cb;
    ble_hs_cfg.sm_bonding=0;ble_hs_cfg.sm_mitm=0;
    if(xTaskCreate(manager,"oft_ble",6144,NULL,5,NULL)!=pdPASS)return ESP_ERR_NO_MEM;
    nimble_port_freertos_init(host_task);return ESP_OK;
}
