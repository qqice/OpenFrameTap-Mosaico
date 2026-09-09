#include "oft_gauge.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "unity.h"
#include "nvs.h"
#include "psa/crypto.h"
#include "oft_udp.h"
#include <math.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
static atomic_int requested;
static atomic_bool configured;
static bool startup_checked;
static bool deferred_restore;
typedef struct {int64_t us;uint16_t soc,mv,design,status,flags,remaining,fcc;int16_t raw_ma;bool valid;} sample_t;
#define HISTORY_CAPACITY 128u
static sample_t history[HISTORY_CAPACITY];static unsigned history_next,history_count;
_Static_assert(sizeof(history)<=4096,"Keep battery evidence storage bounded");
static int current_direction(int16_t raw){return raw>3?1:raw< -3?-1:0;}
static bool history_due(const sample_t *last,const sample_t *next)
{
    if(next->us<=last->us)return false;
    return next->us-last->us>=30000000||next->valid!=last->valid||
        (next->valid&&(current_direction(next->raw_ma)!=current_direction(last->raw_ma)||
         next->fcc!=last->fcc||next->design!=last->design||(next->soc==0)!=(last->soc==0)));
}
void oft_gauge_observe(bool valid,uint16_t soc,uint16_t mv,int16_t raw_ma,uint16_t design,uint16_t status,uint16_t flags,uint16_t remaining,uint16_t fcc)
{
    /* Battery task is the only caller/reader. Bounded RAM history survives USB
       unplugging while running on battery, but deliberately not a power-off. */
    sample_t sample={.us=esp_timer_get_time(),.soc=soc,.mv=mv,.design=design,.status=status,.flags=flags,.remaining=remaining,.fcc=fcc,.raw_ma=raw_ma,.valid=valid};
    if(history_count&&!history_due(&history[(history_next+HISTORY_CAPACITY-1)%HISTORY_CAPACITY],&sample))return;
    history[history_next]=sample;
    history_next=(history_next+1)%HISTORY_CAPACITY;if(history_count<HISTORY_CAPACITY)history_count++;
}
static void dump_history(void)
{
    printf("OFT_GAUGE history_begin version=2 count=%u capacity=128 period_s=30 edge_poll_s=5\n",history_count);
    for(unsigned i=0;i<history_count;i++){
        const sample_t *s=&history[(history_next+HISTORY_CAPACITY-history_count+i)%HISTORY_CAPACITY];
        printf("OFT_GAUGE_SAMPLE us=%lld valid=%d soc=%u mv=%u raw_ma=%d raw_design=%u status=%04x flags=%04x raw_rm=%u raw_fcc=%u\n",(long long)s->us,s->valid,s->soc,s->mv,s->raw_ma,s->design,s->status,s->flags,s->remaining,s->fcc);
    }
    printf("OFT_GAUGE history_end\n");
}
static TickType_t delay_ticks(unsigned ms)
{return pdMS_TO_TICKS(ms+portTICK_PERIOD_MS-1)+1;}
static void delay_ms(unsigned ms){vTaskDelay(delay_ticks(ms));}
static bool read_reg(i2c_master_dev_handle_t d,uint8_t r,uint8_t *p,size_t n)
{esp_err_t rc=i2c_master_transmit_receive(d,&r,1,p,n,25);esp_rom_delay_us(100);return rc==ESP_OK;}
static bool write_reg(i2c_master_dev_handle_t d,uint8_t reg,const uint8_t *p,size_t n)
{
    uint8_t b[35];if(n>34)return false;b[0]=reg;memcpy(b+1,p,n);
    esp_err_t rc=i2c_master_transmit(d,b,n+1,25);esp_rom_delay_us(100);return rc==ESP_OK;
}
static uint8_t checksum(const uint8_t *p,size_t n)
{uint8_t s=0;for(size_t i=0;i<n;i++)s+=p[i];return (uint8_t)(255-s);}
static bool block_valid(uint16_t address,const uint8_t b[36],bool memory)
{
    unsigned n=b[35];return n>=4&&n<=36&&(!memory||n==36)&&
        b[0]==(uint8_t)address&&b[1]==(uint8_t)(address>>8)&&checksum(b,n-2)==b[34];
}
static bool mac_read(i2c_master_dev_handle_t d,uint16_t address,uint8_t b[36],bool memory)
{
    uint8_t a[]={(uint8_t)address,(uint8_t)(address>>8)};
    if(!write_reg(d,0x3e,a,2))return false;
    delay_ms(15);
    /* 3E is a command selector, not a readable echo on this device. Validate
       the returned data checksum against the address we selected. */
    b[0]=a[0];b[1]=a[1];
    for(unsigned i=0;i<34;i+=2)if(!read_reg(d,(uint8_t)(0x40+i),b+2+i,2))return false;
    if(!block_valid(address,b,memory)){
        printf("OFT_GAUGE invalid_block address=%04x raw=",address);for(unsigned i=0;i<36;i++)printf("%02x",b[i]);printf("\n");return false;
    }
    return true;
}
static bool word(i2c_master_dev_handle_t d,uint8_t reg,uint16_t *out)
{uint8_t b[2];if(!read_reg(d,reg,b,2))return false;*out=b[0]|((uint16_t)b[1]<<8);return true;}
static bool control(i2c_master_dev_handle_t d,uint16_t cmd)
{
    uint8_t b[]={(uint8_t)cmd,(uint8_t)(cmd>>8)};
    bool ok=write_reg(d,0,b,1)&&write_reg(d,1,b+1,1);delay_ms(5);return ok;
}
static bool wait_security(i2c_master_dev_handle_t d,unsigned expected)
{
    for(unsigned i=0;i<20;i++){uint16_t s;if(word(d,0x3a,&s)&&((s>>1)&3)==expected)return true;delay_ms(100);}
    return false;
}
static bool restore_security(i2c_master_dev_handle_t d,unsigned prior)
{
    uint16_t s;if(!word(d,0x3a,&s))return false;if(((s>>1)&3)==prior)return true;
    if(prior==1)return false;
    if(((s>>1)&3)!=3&&(!control(d,0x0030)||!wait_security(d,3)))return false;
    if(prior==3)return true;
    delay_ms(5000); // TI: allow the unseal state machine's four-second timer to expire.
    /* Use the documented single-byte transfer form for the backed-up keys.
       No key enumeration and no modification of the key data memory. */
    const uint8_t bytes[]={0x14,0x04,0x72,0x36};
    bool ok=write_reg(d,0,&bytes[0],1)&&write_reg(d,1,&bytes[1],1);
    delay_ms(100);
    ok=ok&&write_reg(d,0,&bytes[2],1)&&write_reg(d,1,&bytes[3],1);
    delay_ms(100);
    bool restored=ok&&wait_security(d,2);word(d,0x3a,&s);
    printf("OFT_GAUGE restore_security single_byte=1 io_ok=%d status=%04x\n",ok,s);
    return restored;
}
bool oft_gauge_request(oft_gauge_action_t a)
{int expected=0;if(a<OFT_GAUGE_AUDIT||a>OFT_GAUGE_MODEL)return false;return atomic_compare_exchange_strong(&requested,&expected,a);}
bool oft_gauge_ready(void){return atomic_load(&configured);}
/* The fixed whitelist is the entire write surface. No OTP, security-key,
 * voltage/temperature calibration, ROM, charger or arbitrary-address API.
 * Virtual current/capacity = physical x2 (TI SLUA792); original analog offsets
 * and hardware shunt are retained. This is an estimate, not cell characterization.
 */
enum {SET_VALUE,DOUBLE_VALUE,HALF_VALUE,XEMICS_DOUBLE};
typedef struct {uint16_t address;uint8_t width,rule;int value,max;} parameter_t;
static const parameter_t parameters[]={
    {0x9184,4,XEMICS_DOUBLE,0,0}, // CC Gain
    {0x9188,4,XEMICS_DOUBLE,0,0}, // CC Delta; same scaling as current
    {0x91de,1,SET_VALUE,2,255}, // reported-current deadband = 1 physical mA
    {0x91fb,2,SET_VALUE,132,1000}, // TP4057/R47: about 66 physical mA
    {0x9201,2,SET_VALUE,20,1000}, // taper 10 physical mA, above charger C/10
    {0x920e,2,DOUBLE_VALUE,0,32767}, // unused BTP: preserve physical threshold
    {0x9210,2,DOUBLE_VALUE,0,32767},
    {0x9217,2,SET_VALUE,4,100}, // sleep 2 physical mA
    {0x9228,2,SET_VALUE,10,2000}, // discharge detect 5 physical mA
    {0x922a,2,SET_VALUE,10,2000}, // charge detect 5 physical mA
    {0x922c,2,SET_VALUE,4,1000}, // relax 2 physical mA
    {0x923c,1,DOUBLE_VALUE,0,127}, // signed initial standby current
    {0x9264,2,DOUBLE_VALUE,0,32767}, // overload detection, not charger control
    {0x9269,2,DOUBLE_VALUE,0,255}, // electronics load; currently zero
    {0x926b,2,SET_VALUE,8,32767}, // near-full 4 physical mAh
    {0x926d,2,DOUBLE_VALUE,0,32767}, // reserve capacity; currently zero
    {0x9276,2,DOUBLE_VALUE,0,32767}, // smoothing current (TRM ROM table: mA)
    {0x929d,2,SET_VALUE,130,32767}, // initial FCC = actual 65 mAh x2
    {0x929f,2,SET_VALUE,130,32767}, // design capacity
    {0x92ab,2,HALF_VALUE,0,65535}, // EDV equation ILOAD*R0: preserve product
};
#define FIELD_COUNT (sizeof(parameters)/sizeof(parameters[0]))
typedef struct {uint8_t bytes[FIELD_COUNT][4];} profile_t;
static double xemics(const uint8_t p[4])
{
    uint32_t m=0x800000|((uint32_t)(p[1]&127)<<16)|((uint32_t)p[2]<<8)|p[3];
    return (p[1]&128?-1.:1.)*ldexp((double)m,(int)p[0]-128-24);
}
static bool make_profile(const profile_t *before,profile_t *after)
{
    memset(after,0,sizeof(*after));
    double gain=xemics(before->bytes[0]),delta=xemics(before->bytes[1]);
    if(gain<.1||gain>2||delta<30000||delta>1500000||fabs(delta/gain-1193046.5)>120)return false;
    for(unsigned i=0;i<FIELD_COUNT;i++){
        const parameter_t *p=&parameters[i];const uint8_t *b=before->bytes[i];uint8_t *v=after->bytes[i];
        if(p->rule==XEMICS_DOUBLE){if(b[0]>=254)return false;memcpy(v,b,4);v[0]++;continue;}
        int n=p->width==1?b[0]:(b[0]<<8)|b[1];if(p->address==0x923c)n=(int8_t)b[0];
        n=p->rule==SET_VALUE?p->value:p->rule==HALF_VALUE?(n+1)/2:n*2;
        if(n>p->max||n<(p->address==0x923c?-127:0))return false;
        if(p->width==1)v[0]=(uint8_t)n;else {v[0]=(uint8_t)(n>>8);v[1]=(uint8_t)n;}
    }
    return true;
}
static bool read_profile(i2c_master_dev_handle_t d,profile_t *out)
{
    memset(out,0,sizeof(*out));uint8_t b[36];
    for(unsigned i=0;i<FIELD_COUNT;i++){
        if(!mac_read(d,parameters[i].address,b,true))return false;
        memcpy(out->bytes[i],b+2,parameters[i].width);
    }
    return true;
}
static bool digest(const profile_t *p,uint8_t out[32])
{size_t n=0;return psa_crypto_init()==PSA_SUCCESS&&psa_hash_compute(PSA_ALG_SHA_256,(const uint8_t*)p,sizeof(*p),out,32,&n)==PSA_SUCCESS&&n==32;}
static bool load_backup(nvs_handle_t n,profile_t *p)
{
    size_t len=sizeof(*p),hlen=32;uint8_t stored[32],actual[32];
    return nvs_get_blob(n,"backup_v1",p,&len)==ESP_OK&&len==sizeof(*p)&&
        nvs_get_blob(n,"hash_v1",stored,&hlen)==ESP_OK&&hlen==32&&digest(p,actual)&&!memcmp(actual,stored,32);
}
static bool save_backup(nvs_handle_t n,const profile_t *p)
{
    uint8_t h[32];profile_t check;
    if(!digest(p,h)||nvs_set_blob(n,"backup_v1",p,sizeof(*p))!=ESP_OK||nvs_set_blob(n,"hash_v1",h,32)!=ESP_OK||nvs_commit(n)!=ESP_OK||!load_backup(n,&check)||memcmp(&check,p,sizeof(check)))return false;
    printf("OFT_GAUGE backup_sha256=");for(unsigned i=0;i<32;i++)printf("%02x",h[i]);printf("\n");return true;
}
static bool journal(nvs_handle_t n,uint8_t state)
{return nvs_set_u8(n,"state_v1",state)==ESP_OK&&nvs_commit(n)==ESP_OK;}
static bool cfg_mode(i2c_master_dev_handle_t d,bool enter)
{
    if(!control(d,enter?0x0090:0x0091))return false;
    delay_ms(2000);
    for(unsigned i=0;i<20;i++){uint16_t s;if(word(d,0x3a,&s)&&!!(s&0x0400)==enter)return true;delay_ms(100);}
    return false;
}
static bool write_memory_parameter(i2c_master_dev_handle_t d,const parameter_t *p,const uint8_t *value)
{
    uint8_t data[6]={(uint8_t)p->address,(uint8_t)(p->address>>8)};
    memcpy(data+2,value,p->width);
    /* TI's BQ27220-specific corrected DM example: address+data in one MAC
       transaction, then checksum+length together. The unseal Control command
       still uses the independently verified single-byte path. */
    if(!write_reg(d,0x3e,data,p->width+2))return false;
    delay_ms(1); // >=250us MAC processing time; not just I2C bus-free time
    uint8_t commit[]={checksum(data,p->width+2),p->width+4};
    if(!write_reg(d,0x60,commit,2))return false;
    delay_ms(15);uint8_t b[36];
    for(unsigned j=0;j<2;j++){
        if(!mac_read(d,p->address,b,true))return false;
        if(memcmp(b+2,value,p->width)){
            printf("OFT_GAUGE mismatch address=%04x expected=",p->address);
            for(unsigned k=0;k<p->width;k++){printf("%02x",value[k]);}printf(" actual=");
            for(unsigned k=0;k<p->width;k++){printf("%02x",b[k+2]);}printf("\n");return false;
        }
    }
    return true;
}
static bool write_parameter(i2c_master_dev_handle_t d,unsigned i,const uint8_t *value)
{return i<FIELD_COUNT&&write_memory_parameter(d,&parameters[i],value);}
static uint16_t model_config(uint16_t original)
{
    // TI TRM4.9.14: independent TP4057 charger (SC=1), existing fixed EDV
    // voltages (EDV_CMP=0), not uncharacterized18650 impedance compensation.
    return original==0x102a||original==0x1032?0x1032:0;
}
static bool write_model_config(i2c_master_dev_handle_t d,uint16_t value)
{
    if(value!=0x102a&&value!=0x1032)return false;
    const parameter_t p={0x929b,2,SET_VALUE,0,0};uint8_t bytes[]={(uint8_t)(value>>8),(uint8_t)value};
    return write_memory_parameter(d,&p,bytes);
}
static bool read_model_config(i2c_master_dev_handle_t d,uint16_t *value)
{uint8_t b[36];if(!mac_read(d,0x929b,b,true))return false;*value=((uint16_t)b[2]<<8)|b[3];return true;}
static bool write_profile(i2c_master_dev_handle_t d,const profile_t *p,bool rollback)
{
    bool ok=true;
    for(unsigned i=0;i<FIELD_COUNT;i++){
        bool field_ok=write_parameter(d,i,p->bytes[i]);
        printf("OFT_GAUGE parameter=%04x verified=%d\n",parameters[i].address,field_ok);
        if(!field_ok){ok=false;if(!rollback)break;}
    }
    return ok;
}
static bool full_access(i2c_master_dev_handle_t d)
{return control(d,0xffff)&&control(d,0xffff)&&wait_security(d,1);}
static void configure(i2c_master_dev_handle_t d,bool restore,bool upgrade_model)
{
    atomic_store(&configured,false);
    oft_udp_snapshot_t camera;oft_udp_snapshot(&camera);
    if(camera.head_held||camera.action_busy||camera.shutter_busy||camera.settings_busy||camera.probe_running||camera.control_state==OFT_CONTROL_ACTIVE){printf("OFT_GAUGE configure_blocked=active_camera_input\n");return;}
    uint16_t s,v,ma,dc;profile_t old,before,target,verify;nvs_handle_t n;bool opened=false,entered=false,changed=false,ok=false,rollback=false;
    uint16_t config_before=0,config_original=0,config_target=0,config_verify=0;uint8_t model_active=0;bool model_saved=false,touch_model=false,target_model=false;
    if(!word(d,0x3a,&s)||((s>>1)&3)!=2||(s&0x400)||!word(d,8,&v)||!word(d,12,&ma)||!word(d,0x3c,&dc)||v<4100||v>4250||(int16_t)ma>3||(int16_t)ma< -3){printf("OFT_GAUGE configure_blocked=need_unsealed_quiet_charged_cell\n");return;}
    if(nvs_open("oft_gauge",NVS_READWRITE,&n)!=ESP_OK)return;
    opened=true;
    if(!full_access(d))goto done;
    if(!read_profile(d,&before))goto done;
    if(!read_model_config(d,&config_before))goto done;
    esp_err_t model_rc=nvs_get_u16(n,"model_orig",&config_original);
    if(model_rc==ESP_OK)model_saved=true;else if(model_rc!=ESP_ERR_NVS_NOT_FOUND)goto done;
    model_rc=nvs_get_u8(n,"model_active",&model_active);
    if(model_rc!=ESP_OK&&model_rc!=ESP_ERR_NVS_NOT_FOUND)goto done;
    if(upgrade_model&&!model_saved){
        if(config_before!=0x102a||nvs_set_u16(n,"model_orig",config_before)!=ESP_OK||nvs_commit(n)!=ESP_OK||nvs_get_u16(n,"model_orig",&config_original)!=ESP_OK||config_original!=config_before)goto done;
        model_saved=true;
    }
    touch_model=model_saved;target_model=!restore&&(upgrade_model||model_active);
    config_target=target_model?model_config(config_original):model_saved?config_original:config_before;
    if(touch_model&&(!model_config(config_before)||!model_config(config_original)||!config_target))goto done;
    size_t saved_len=0;esp_err_t saved=nvs_get_blob(n,"backup_v1",NULL,&saved_len);
    if(saved==ESP_ERR_NVS_NOT_FOUND){
        if(restore||dc!=3000||!make_profile(&before,&target)||!save_backup(n,&before))goto done;
        old=before;
    }else if(saved!=ESP_OK||!load_backup(n,&old)){printf("OFT_GAUGE configure_blocked=invalid_backup\n");goto done;}
    if(restore)target=old;else if(!make_profile(&old,&target))goto done;
    if(!journal(n,2))goto done; // durable pending flag precedes any parameter write
    entered=true; // attempt exit even if the entry status read failed
    if(!cfg_mode(d,true))goto done;
    if(!word(d,0x3a,&s)||((s>>1)&3)!=1)goto done;
    printf("OFT_GAUGE cfg_status=%04x\n",s);
    changed=true;
    ok=write_profile(d,&target,false)&&read_profile(d,&verify)&&!memcmp(&target,&verify,sizeof(target));
    if(ok&&touch_model)ok=write_model_config(d,config_target)&&read_model_config(d,&config_verify)&&config_verify==config_target;
    if(!ok){
        bool parameters_ok=write_profile(d,&before,true);
        bool config_ok=!touch_model||write_model_config(d,config_before);
        rollback=parameters_ok&&config_ok&&read_profile(d,&verify)&&!memcmp(&before,&verify,sizeof(before))&&read_model_config(d,&config_verify)&&config_verify==config_before;
    }
done:
    if(entered){bool exited=cfg_mode(d,false);ok=ok&&exited;rollback=rollback&&exited;}
    bool security_ok=restore_security(d,2);ok=ok&&security_ok;
    if(ok&&word(d,0x3c,&dc)&&dc==(restore?3000:130)){
        if(touch_model)ok=nvs_set_u8(n,"model_active",target_model?1:0)==ESP_OK&&nvs_commit(n)==ESP_OK;
        ok=ok&&journal(n,restore?0:1);
    }else ok=false;
    if(rollback&&security_ok)journal(n,0);
    if(opened)nvs_close(n);
    atomic_store(&configured,ok&&!restore);
    printf("OFT_GAUGE configure_complete=%d restore=%d rollback=%d ram_attempted=%d security_restored=%d profile_scale=%d\n",ok,restore,rollback,changed,security_ok,oft_gauge_ready()?2:0);
    printf("OFT_GAUGE model active=%d original=%04x before=%04x desired=%04x verified=%04x\n",ok&&target_model,config_original,config_before,config_target,config_verify);
}
static bool check_existing(i2c_master_dev_handle_t d)
{
    /* MCU restart only verifies. A gauge power loss does NOT silently reapply
       calibration under load, nor overwrite learned FCC on every boot. */
    nvs_handle_t n;uint8_t state=0;profile_t original,target,current;uint16_t s;
    esp_err_t rc=nvs_open("oft_gauge",NVS_READONLY,&n);
    // Peripherals start before the BLE task initializes shared NVS. Retry only
    // that dependency, not a failed gauge transaction or missing profile.
    if(rc==ESP_ERR_NVS_NOT_INITIALIZED)return false;
    if(rc!=ESP_OK)return true;
    bool ok=nvs_get_u8(n,"state_v1",&state)==ESP_OK&&state==1&&load_backup(n,&original)&&make_profile(&original,&target);
    uint8_t model_active=0;uint16_t original_config=0;bool model_saved=nvs_get_u16(n,"model_orig",&original_config)==ESP_OK;
    if(model_saved&&nvs_get_u8(n,"model_active",&model_active)!=ESP_OK)ok=false;
    nvs_close(n);
    if(!ok){printf("OFT_GAUGE saved_profile_unavailable state=%u\n",state);return true;}
    uint16_t design;
    if(!word(d,0x3a,&s)||!word(d,0x3c,&design)||(s&0x400))return true;
    if(model_active&&design==3000){deferred_restore=true;printf("OFT_GAUGE restore_deferred=profile_lost_wait_for_quiet_USB\n");return true;}
    unsigned prior=(s>>1)&3;
    if(prior==3){if(!restore_security(d,2))return true;prior=2;}
    if(prior!=2&&prior!=1)return true;
    // This unit returns a stale MAC window for calibration addresses in
    // UNSEALED. Full Access is required for CRC-verified gain validation.
    if(prior==2&&!full_access(d)){restore_security(d,2);return true;}
    ok=read_profile(d,&current);
    if(ok&&model_saved){uint16_t actual;ok=read_model_config(d,&actual)&&actual==(model_active?model_config(original_config):original_config);}
    if(ok){ // FCC is learned; verify its plausible range without overwriting it.
        unsigned fcc=(current.bytes[17][0]<<8)|current.bytes[17][1];
        memcpy(current.bytes[17],target.bytes[17],4);ok=fcc>=65&&fcc<=195&&!memcmp(&current,&target,sizeof(current));
    }
    bool security_ok=prior==1||restore_security(d,2);ok=ok&&security_ok;atomic_store(&configured,ok);
    printf("OFT_GAUGE existing_profile_valid=%d scale=2 security_restored=%d\n",ok,security_ok);
    return true;
}
static void audit(i2c_master_dev_handle_t d,int action)
{
    uint16_t status,flags,voltage,current,dc,fcc;
    if(!word(d,0x3a,&status)||!word(d,0x0a,&flags)||!word(d,8,&voltage)||!word(d,12,&current)||!word(d,0x3c,&dc)||!word(d,0x12,&fcc)){printf("OFT_GAUGE audit_failed=standard_read\n");return;}
    printf("OFT_GAUGE audit_begin us=%lld status=%04x security=%u flags=%04x mv=%u ma=%d design=%u fcc=%u writes_to_ram=0\n",(long long)esp_timer_get_time(),status,(status>>1)&3,flags,voltage,(int16_t)current,dc,fcc);
    uint8_t b[36],command[]={1,0};uint16_t identity;
    if(!write_reg(d,0,command,2)){printf("OFT_GAUGE audit_failed=identity_request\n");return;}
    delay_ms(20);
    if(!word(d,0x40,&identity)){printf("OFT_GAUGE audit_failed=identity_read\n");return;}
    printf("OFT_GAUGE device=%04x\n",identity);
    if(identity!=0x0220){printf("OFT_GAUGE audit_failed=wrong_device\n");return;}
    if(action==OFT_GAUGE_ACCESS_RESTORE){printf("OFT_GAUGE access_restore=%d target_security=2 writes_to_ram=0\n",restore_security(d,2));return;}
    unsigned prior=(status>>1)&3;bool complete=false;
    if(prior!=2&&prior!=1){printf("OFT_GAUGE audit_failed=unexpected_security\n");return;}
    if(prior==2&&!full_access(d)){restore_security(d,2);printf("OFT_GAUGE audit_failed=full_access\n");return;}
    printf("OFT_GAUGE full_access=1 config_update=0\n");
    if(!mac_read(d,0x9180,b,true)){printf("OFT_GAUGE audit_failed=gain_read\n");goto done;}
    printf("OFT_GAUGE block=9180 data=");for(unsigned j=0;j<32;j++)printf("%02x",b[j+2]);printf(" checksum=%02x length=%u\n",b[34],b[35]);
    for(unsigned i=0;i<9;i++){
        uint16_t a=0x91b4+i*32;
        if(!mac_read(d,a,b,true)){printf("OFT_GAUGE audit_failed=memory_read address=%04x\n",a);goto done;}
        printf("OFT_GAUGE block=%04x data=",a);for(unsigned j=0;j<32;j++)printf("%02x",b[j+2]);printf(" checksum=%02x length=%u\n",b[34],b[35]);
    }
    complete=true;
done:
    printf("OFT_GAUGE audit_complete=%d security_restored=%d writes_to_ram=0\n",complete,prior==1||restore_security(d,2));
}
void oft_gauge_poll(i2c_master_dev_handle_t d)
{
    if(!startup_checked)startup_checked=check_existing(d);
    if(deferred_restore&&!atomic_load(&requested)){
        uint16_t v,ma,design,s;
        if(word(d,8,&v)&&word(d,12,&ma)&&word(d,0x3c,&design)&&word(d,0x3a,&s)&&design==3000&&
           v>=4100&&v<=4250&&(int16_t)ma>=-3&&(int16_t)ma<=3&&!(s&0x400)){
            deferred_restore=false; // One attempt per boot, never an unlock/write retry loop.
            if(restore_security(d,2)){printf("OFT_GAUGE automatic_restore=known_model_quiet_USB\n");configure(d,false,true);}
            else printf("OFT_GAUGE automatic_restore_failed=access\n");
        }
    }
    int action=atomic_load(&requested);if(!action)return;
    if(action==OFT_GAUGE_HISTORY)dump_history();
    else if(action==OFT_GAUGE_APPLY||action==OFT_GAUGE_RESTORE||action==OFT_GAUGE_MODEL)configure(d,action==OFT_GAUGE_RESTORE,action==OFT_GAUGE_MODEL);
    else audit(d,action);
    atomic_store(&requested,0);
}
static void test_gauge_block_checks(void)
{
    TEST_ASSERT_TRUE((delay_ticks(5)-1)*portTICK_PERIOD_MS>=5);
    TEST_ASSERT_TRUE((delay_ticks(15)-1)*portTICK_PERIOD_MS>=15);
    uint8_t b[36]={0x9f,0x92,0x0b,0xb8};b[35]=36;b[34]=checksum(b,34);
    TEST_ASSERT_TRUE(block_valid(0x929f,b,true));b[3]^=1;TEST_ASSERT_FALSE(block_valid(0x929f,b,true));b[3]^=1;
    TEST_ASSERT_FALSE(block_valid(0x929d,b,true));b[35]=37;TEST_ASSERT_FALSE(block_valid(0x929f,b,true));
    TEST_ASSERT_FALSE(oft_gauge_request(99));
}
static void test_gauge_profile(void)
{
    TEST_ASSERT_EQUAL_HEX16(0x1032,model_config(0x102a));TEST_ASSERT_EQUAL_HEX16(0x1032,model_config(0x1032));
    TEST_ASSERT_EQUAL_HEX16(0,model_config(0xffff));
    profile_t a={0},b;const uint8_t gain[]={0x7e,0x73,0xb6,0x45},delta[]={0x93,0x0a,0xa5,0x22};
    memcpy(a.bytes[0],gain,4);memcpy(a.bytes[1],delta,4);a.bytes[11][0]=(uint8_t)-10;
    a.bytes[19][0]=3;a.bytes[19][1]=0x63;
    TEST_ASSERT_TRUE(make_profile(&a,&b));TEST_ASSERT_EQUAL_UINT8(0x7f,b.bytes[0][0]);
    TEST_ASSERT_FLOAT_WITHIN(.00001,.476,xemics(b.bytes[0]));
    TEST_ASSERT_EQUAL_UINT8(130,b.bytes[17][1]);TEST_ASSERT_EQUAL_UINT8(130,b.bytes[18][1]);
    TEST_ASSERT_EQUAL_INT8(-20,(int8_t)b.bytes[11][0]);TEST_ASSERT_EQUAL_UINT8(0xb2,b.bytes[19][1]);
    a.bytes[5][0]=0x7f;TEST_ASSERT_FALSE(make_profile(&a,&b));
    a.bytes[0][0]=255;TEST_ASSERT_FALSE(make_profile(&a,&b));
    TEST_ASSERT_FALSE(write_parameter(NULL,FIELD_COUNT,gain));
}
static void test_gauge_history_cadence(void)
{
    sample_t a={.us=1000000,.valid=true,.soc=50,.raw_ma=-540,.fcc=100,.design=130},b=a;
    TEST_ASSERT_FALSE(history_due(&a,&b));b.us+=5000000;TEST_ASSERT_FALSE(history_due(&a,&b));
    b.raw_ma=136;TEST_ASSERT_TRUE(history_due(&a,&b));b.raw_ma=a.raw_ma;
    b.fcc=98;TEST_ASSERT_TRUE(history_due(&a,&b));b.fcc=a.fcc;
    b.soc=0;TEST_ASSERT_TRUE(history_due(&a,&b));b.soc=a.soc;
    b.valid=false;TEST_ASSERT_TRUE(history_due(&a,&b));b.valid=true;
    b.us=a.us+30000000;TEST_ASSERT_TRUE(history_due(&a,&b));
    TEST_ASSERT_EQUAL_INT(0,current_direction(2));TEST_ASSERT_EQUAL_INT(0,current_direction(-2));
    TEST_ASSERT_EQUAL_UINT(0,(HISTORY_CAPACITY-1+1)%HISTORY_CAPACITY);
}
void oft_gauge_selftests(void){RUN_TEST(test_gauge_block_checks);RUN_TEST(test_gauge_profile);RUN_TEST(test_gauge_history_cadence);}
