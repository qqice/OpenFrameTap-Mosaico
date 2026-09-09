#include "oft_video.h"
#include "oft_video_policy.h"
extern "C" {
#include "oft_media.h"
#include "oft_udp.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "mbedtls/base64.h"
#include "psa/crypto.h"
#include "unity.h"
#include "lwip/mem.h"
#include "esp_memory_utils.h"
}
#include "wels/codec_api.h"
#include "oft_profile.h"
#include "cabac32.h"
#include <atomic>
#include <cstdio>
#include <cstring>
#ifndef OFT_CABAC_IRAM
#define OFT_CABAC_IRAM 0
#endif
#ifndef OFT_RESIDUAL_IRAM
#define OFT_RESIDUAL_IRAM 0
#endif

static constexpr size_t AU_CAP=262144,QUEUED_CAP=256*1024;
static constexpr unsigned DW=352,DH=198,RGB_BYTES=DW*DH*2;
struct Packet {int64_t at;uint32_t hash;uint16_t size;uint8_t bytes[1472];};
struct Unit {uint8_t *data;size_t size;int64_t at;unsigned epoch;bool idr;};
static QueueHandle_t packets,units;
static StaticQueue_t packet_control,unit_control;
static StaticSemaphore_t image_control;
static SemaphoreHandle_t image_lock;
static uint8_t *packet_storage;
static uint8_t unit_storage[128*sizeof(Unit)];
static std::atomic<bool> accepting{false};
static std::atomic<unsigned> epoch{1},queued_bytes{0};
static portMUX_TYPE mux=portMUX_INITIALIZER_UNLOCKED;
static oft_video_snapshot_t view;
static uint16_t *published;
static uint32_t image_generation;
static int64_t image_source_us;
static uint8_t *bench_sample;static size_t bench_size;static int64_t bench_source_us;
#ifdef OFT_BENCH_EMBEDDED
extern const uint8_t fixture_begin[] asm("_binary_oft_bench_fixture_start");
extern const uint8_t fixture_end[] asm("_binary_oft_bench_fixture_end");
#endif
static std::atomic<bool> bench_ready{false},bench_requested{false};
extern "C" bool oft_video_benchmark(void)
{
    oft_udp_snapshot_t s;oft_udp_snapshot(&s);
    if(!bench_ready.load()||s.control_state==OFT_CONTROL_ACTIVE||s.head_held||s.action_busy)return false;
    oft_video_stop_refresh();bench_requested=true;return true;
}
static void detail(const char *s);
static std::atomic<bool> sparse{true};
static std::atomic<bool> reassembly_reset{false};
struct Trace {int64_t at;uint16_t size;uint8_t prefix[64];uint64_t bitmap;uint32_t declared,used,complete,dropped;uint8_t group,count,protected_idr;};
static Trace *traces;static unsigned trace_count;static int64_t trace_until;
static std::atomic<bool> trace_armed{false},trace_active{false},trace_ready{false};
extern "C" bool oft_video_trace_arm(void)
{
    if(trace_active.load()||trace_armed.load())return false;
    if(!traces)traces=(Trace*)heap_caps_malloc(256*sizeof(Trace),MALLOC_CAP_SPIRAM);
    if(!traces)return false;
    trace_ready=false;trace_armed=true;return true;
}
extern "C" void oft_video_trace_dump(void)
{
    if(!trace_ready.load()||trace_active.load()){printf("OFT_VIDEO_TRACE not_ready=1\n");return;}
    printf("OFT_VIDEO_TRACE count=%u\n",trace_count);
    for(unsigned i=0;i<trace_count;i++){
        const Trace &t=traces[i];char h[129];for(unsigned j=0;j<64;j++)snprintf(h+2*j,3,"%02x",t.prefix[j]);
        printf("OFT_VIDEO_PACKET us=%lld size=%u head=%s group=%u count=%u bitmap=%016llx declared=%lu used=%lu complete=%lu dropped=%lu protected=%u\n",
            (long long)t.at,t.size,h,t.group,t.count,(unsigned long long)t.bitmap,(unsigned long)t.declared,
            (unsigned long)t.used,(unsigned long)t.complete,(unsigned long)t.dropped,t.protected_idr);
    }
}
static unsigned refresh_remaining,refresh_interval=5000;
static int64_t refresh_due,refresh_wait;
static uint32_t refresh_generation,refresh_request_count;
static std::atomic<int> refresh_command{-1};
static std::atomic<TaskHandle_t> refresh_task{nullptr};
static void wake_refresh()
{TaskHandle_t task=refresh_task.load();if(task)xTaskNotifyGive(task);}
static std::atomic<bool> visible{false};
static std::atomic<bool> fresh_mode{true};
static unsigned work_budget_ms(){return fresh_mode.load()?OFT_VIDEO_FRESH_BUDGET_MS:1600u;}
static unsigned gui_interval_ms(){return fresh_mode.load()?OFT_VIDEO_FRESH_INTERVAL_MS:OFT_VIDEO_DEFAULT_INTERVAL_MS;}
extern "C" void oft_video_latency_mode(bool fresh){fresh_mode=fresh;wake_refresh();}
static bool gui_refresh,previous_visible;
static unsigned missed_picture;
static unsigned recovery_level;
static std::atomic<unsigned> simulated_losses{0};
static std::atomic<int64_t> burst_until{0};
static std::atomic<unsigned> trial_burst_maximum{3};
static std::atomic<bool> burst_disabled{false};
static unsigned burst_budget(){return burst_disabled.load()?0:esp_timer_get_time()<burst_until.load()?trial_burst_maximum.load():OFT_VIDEO_DEFAULT_BURST_FRAMES;}
static unsigned predicted_prefix(unsigned maximum,unsigned idr_ms,unsigned p_ms,unsigned convert_ms,unsigned budget=1600)
{
    uint64_t base=(uint64_t)(idr_ms?idr_ms:800)+(convert_ms?convert_ms:20);
    uint64_t cost=(uint64_t)(p_ms?p_ms:300)+(convert_ms?convert_ms:20);
    if(base>=budget)return 0;
    unsigned fit=(budget-base)/cost;return fit<maximum?fit:maximum;
}
static bool prediction_timely(int64_t at,int64_t now,unsigned cost,unsigned budget)
{return now>=at&&now-at+(int64_t)cost*1000<=(int64_t)budget*1000;}
extern "C" void oft_video_idr_only(void){burst_disabled=true;burst_until=0;}
extern "C" bool oft_video_burst_trial(unsigned maximum)
{
    if(maximum!=3&&maximum!=6)return false;
    oft_udp_snapshot_t u;oft_udp_snapshot(&u);
    if(u.state!=OFT_UDP_READY||u.head_held||u.control_state==OFT_CONTROL_ACTIVE||u.action_busy||simulated_losses.load())return false;
    trial_burst_maximum=maximum;burst_disabled=false;burst_until=esp_timer_get_time()+120000000;return true;
}
static std::atomic<int64_t> simulated_until{0};
extern "C" bool oft_video_loss_test(void)
{
    oft_udp_snapshot_t u;oft_udp_snapshot(&u);oft_video_snapshot_t s;oft_video_snapshot(&s);
    if(u.state!=OFT_UDP_READY||u.head_held||u.control_state==OFT_CONTROL_ACTIVE||u.action_busy||
       !s.sparse_live||!visible.load()||simulated_losses.load()||burst_budget())return false;
    simulated_until=esp_timer_get_time()+30000000;simulated_losses=2;return true;
}
static unsigned image_backoff_ms(unsigned level)
{return level>=3?30000u:(4000u<<level);}
static bool retry_image_only(const oft_udp_snapshot_t &u,unsigned requests,bool gui,bool shown)
{
    return gui&&shown&&u.state==OFT_UDP_READY&&!u.keyframe_failures&&!u.keyframe_pending&&
        u.keyframe_requests>requests&&u.keyframe_replies==u.keyframe_requests;
}
static bool refresh_success(const oft_udp_snapshot_t &u,const oft_video_snapshot_t &s,unsigned requests,uint32_t generation)
{return u.state==OFT_UDP_READY&&!u.keyframe_failures&&u.keyframe_requests>requests&&
    u.keyframe_replies==u.keyframe_requests&&s.last_idr_generation!=generation&&s.last_idr_source_us>u.keyframe_sent_us;}
static bool recover_missing(const oft_udp_snapshot_t &u,const oft_video_snapshot_t &s,
    unsigned requests,uint32_t generation,bool gui,unsigned missed,int64_t age)
{
    bool known_loss=s.lost_idr_source_us>u.keyframe_sent_us&&age>=(int64_t)OFT_VIDEO_MIN_INTERVAL_MS*1000;
    return gui&&!missed&&(age>OFT_VIDEO_MISSING_TIMEOUT_US||known_loss)&&!s.busy&&u.state==OFT_UDP_READY&&!u.keyframe_failures&&
        u.keyframe_requests>requests&&u.keyframe_replies==u.keyframe_requests&&
        !refresh_success(u,s,requests,generation);
}
static bool begin_refresh(unsigned interval_ms)
{
    if(interval_ms&&interval_ms<OFT_VIDEO_MIN_INTERVAL_MS)return false;
    oft_udp_snapshot_t u;oft_udp_snapshot(&u);oft_video_snapshot_t s;oft_video_snapshot(&s);
    if(refresh_wait||!oft_udp_keyframe())return false;
    reassembly_reset.store(true);
    refresh_remaining=interval_ms?11:0;refresh_interval=interval_ms?interval_ms:5000;
    refresh_generation=s.last_idr_generation;refresh_request_count=u.keyframe_requests;
    refresh_wait=esp_timer_get_time();sparse=true;
    missed_picture=0;recovery_level=0;
    portENTER_CRITICAL(&mux);view.sparse_live=interval_ms!=0;view.refresh_interval_ms=refresh_interval;view.refresh_failures=0;view.refresh_recovering=false;view.retry_after_ms=0;portEXIT_CRITICAL(&mux);
    return true;
}
static void stop_refresh()
{refresh_remaining=0;refresh_wait=0;recovery_level=0;simulated_losses=0;oft_udp_cancel_keyframe();portENTER_CRITICAL(&mux);view.sparse_live=false;view.refresh_recovering=false;view.retry_after_ms=0;portEXIT_CRITICAL(&mux);}
extern "C" bool oft_video_refresh(unsigned interval_ms)
{
    if((interval_ms&&interval_ms<OFT_VIDEO_MIN_INTERVAL_MS)||interval_ms>60000)return false;
    oft_udp_snapshot_t u;oft_udp_snapshot(&u);
    if(u.state!=OFT_UDP_READY||u.keyframe_failures)return false;
    int expected=-1;return refresh_command.compare_exchange_strong(expected,(int)interval_ms);
}
extern "C" void oft_video_stop_refresh(void){refresh_command.store(-2);}
extern "C" void oft_video_visible(bool v){visible.store(v);}
extern "C" void oft_video_poll(void)
{
    int64_t now=esp_timer_get_time();oft_udp_snapshot_t u;oft_udp_snapshot(&u);
    oft_video_snapshot_t s;oft_video_snapshot(&s);
    int command=refresh_command.exchange(-1);bool shown=visible.load();
    if(gui_refresh&&refresh_interval!=gui_interval_ms()){
        refresh_interval=gui_interval_ms();
        refresh_due=u.keyframe_sent_us+(int64_t)refresh_interval*1000;
        portENTER_CRITICAL(&mux);view.refresh_interval_ms=refresh_interval;portEXIT_CRITICAL(&mux);
    }
    if(shown!=previous_visible){
        printf("OFT_VIDEO_VISIBLE us=%lld shown=%d\n",(long long)now,shown);
        if(!shown&&gui_refresh){stop_refresh();gui_refresh=false;}
        if(shown&&!refresh_wait&&!refresh_remaining&&command==-1)gui_refresh=true;
        previous_visible=shown;
    }
    if(command==-2){stop_refresh();gui_refresh=false;}
    if(command>=0){
        gui_refresh=false;
        if(!begin_refresh((unsigned)command))printf("OFT_VIDEO_REQUEST rejected_by_scheduler=1\n");
    }
    if((refresh_wait||refresh_remaining)&&(u.state!=OFT_UDP_READY||u.keyframe_failures)){
        stop_refresh();gui_refresh=false;
        portENTER_CRITICAL(&mux);view.refresh_failures++;portEXIT_CRITICAL(&mux);
        detail("VIDEO session/protocol fault; automatic requests stopped");
    }
    if(gui_refresh&&!refresh_wait&&!refresh_remaining&&u.state==OFT_UDP_READY&&!u.keyframe_failures&&!s.busy){
        if(begin_refresh(gui_interval_ms()))refresh_remaining=UINT32_MAX; // Explicit visible-page lifetime.
    }
    if(refresh_wait){
        bool received=refresh_success(u,s,refresh_request_count,refresh_generation);
        if(recover_missing(u,s,refresh_request_count,refresh_generation,gui_refresh,missed_picture,now-refresh_wait)&&oft_udp_keyframe()){
            reassembly_reset.store(true);missed_picture=1;refresh_wait=now;
            refresh_generation=s.last_idr_generation;refresh_request_count=u.keyframe_requests;
            portENTER_CRITICAL(&mux);view.refresh_recoveries++;portEXIT_CRITICAL(&mux);
            printf("OFT_VIDEO_REFRESH recovery=1 reason=missing_picture_after_matched_ack known_loss=%d age_ms=%lld\n",s.lost_idr_source_us>u.keyframe_sent_us,(long long)(now-u.keyframe_sent_us)/1000);
        }else if(u.state!=OFT_UDP_READY||u.keyframe_failures){
            stop_refresh();gui_refresh=false;portENTER_CRITICAL(&mux);view.refresh_failures++;portEXIT_CRITICAL(&mux);
            detail("VIDEO refresh failed; automatic requests stopped");
        }else if(received){
            missed_picture=0;recovery_level=0;
            refresh_wait=0;refresh_due=u.keyframe_sent_us+(int64_t)refresh_interval*1000;
            portENTER_CRITICAL(&mux);view.refresh_completed++;view.refresh_recovering=false;view.retry_after_ms=0;portEXIT_CRITICAL(&mux);
            printf("OFT_VIDEO_REFRESH success=%u generation=%lu source_delay_ms=%lld hash=%08lx\n",s.refresh_completed+1,(unsigned long)s.last_idr_generation,(long long)((s.last_idr_frame_us-u.keyframe_sent_us)/1000),(unsigned long)s.last_idr_hash);
        }else if(now-refresh_wait>OFT_VIDEO_MISSING_TIMEOUT_US){
            if(retry_image_only(u,refresh_request_count,gui_refresh,shown)){
                unsigned delay=image_backoff_ms(recovery_level);if(recovery_level<3)recovery_level++;
                refresh_wait=0;refresh_due=now+(int64_t)delay*1000;refresh_remaining=UINT32_MAX;
                missed_picture=1; // No new fast-retry loop until a real image arrives.
                portENTER_CRITICAL(&mux);view.refresh_recovering=true;view.retry_after_ms=delay;view.refresh_recoveries++;portEXIT_CRITICAL(&mux);
                detail("Image missing; backing off (control remains independent)");
                printf("OFT_VIDEO_REFRESH backoff_ms=%u matched_ack=1\n",delay);
            }else{
                stop_refresh();gui_refresh=false;portENTER_CRITICAL(&mux);view.refresh_failures++;portEXIT_CRITICAL(&mux);
                detail("VIDEO request failed; automatic requests stopped");
            }
        }
    }else if(refresh_remaining&&now>=refresh_due&&!u.action_busy&&!s.busy&&!s.queued_units){
        if(oft_udp_keyframe()){
            reassembly_reset.store(true);
            if(refresh_remaining!=UINT32_MAX)refresh_remaining--;
            refresh_generation=s.last_idr_generation;refresh_request_count=u.keyframe_requests;refresh_wait=now;
        }
    }
    if(!refresh_remaining&&!refresh_wait){portENTER_CRITICAL(&mux);view.sparse_live=false;portEXIT_CRITICAL(&mux);}
    portENTER_CRITICAL(&mux);
    view.retry_after_ms=view.refresh_recovering&&!refresh_wait&&refresh_due>now?(unsigned)((refresh_due-now+999)/1000):0;
    portEXIT_CRITICAL(&mux);
}
static void refresh_worker(void *)
{
    refresh_task=xTaskGetCurrentTaskHandle();
    for(;;){
        oft_video_poll();
        /* Decode completion wakes this owner, not the UDP writer directly.
           The100ms fallback still handles cadence deadlines, ACKs and loss.
           Notifications coalesce; an early wake never bypasses due/busy gates. */
        ulTaskNotifyTake(pdTRUE,pdMS_TO_TICKS(100));
    }
}
static bool timely(int64_t at,int64_t now,bool first)
{return now>=at&&now-at<=(first?4000000:1500000);}
static uint32_t checksum(const uint8_t *p,size_t n){uint32_t h=2166136261u;for(size_t i=0;i<n;i++)h=(h^p[i])*16777619u;return h;}
static void detail(const char *s){portENTER_CRITICAL(&mux);snprintf(view.detail,sizeof(view.detail),"%s",s);portEXIT_CRITICAL(&mux);}
extern "C" void oft_video_snapshot(oft_video_snapshot_t *out)
{
    portENTER_CRITICAL(&mux);*out=view;portEXIT_CRITICAL(&mux);
    out->queued_bytes=queued_bytes.load();out->queued_units=units?uxQueueMessagesWaiting(units):0;
    out->burst_frames=burst_budget();
    out->low_latency=fresh_mode.load();out->latency_budget_ms=work_budget_ms();
    if(out->last_source_us)out->source_age_ms=(esp_timer_get_time()-out->last_source_us)/1000;
    if(!out->last_frame_us||esp_timer_get_time()-out->last_frame_us>3000000)out->fps_milli=0;
}
extern "C" void oft_video_feed(const uint8_t *data,size_t size,int64_t at)
{
    if(!accepting.load()||!packets||size>1472)return;
    Packet p;p.at=at;p.size=(uint16_t)size;memcpy(p.bytes,data,size);p.hash=checksum(p.bytes,size);
    bool ok=xQueueSend(packets,&p,0)==pdTRUE;
    portENTER_CRITICAL(&mux);view.packets++;if(!ok)view.queue_drops++;portEXIT_CRITICAL(&mux);
    if(!ok&&!sparse.load())epoch.fetch_add(1); // A lost later P packet cannot invalidate an already complete sparse IDR.
}
extern "C" uint32_t oft_video_copy(uint16_t *dst,size_t capacity,uint32_t after)
{return oft_video_copy_timed(dst,capacity,after,nullptr);}
extern "C" uint32_t oft_video_copy_timed(uint16_t *dst,size_t capacity,uint32_t after,int64_t *source)
{
    if(!image_lock||!dst||capacity<RGB_BYTES)return 0;
    xSemaphoreTake(image_lock,portMAX_DELAY);uint32_t result=0;
    if(image_generation&&image_generation!=after){memcpy(dst,published,RGB_BYTES);result=image_generation;if(source)*source=image_source_us;}
    xSemaphoreGive(image_lock);return result;
}
extern "C" void oft_video_presented(void){portENTER_CRITICAL(&mux);view.presented_frames++;portEXIT_CRITICAL(&mux);}
static void release(Unit &u)
{
    queued_bytes.fetch_sub(u.size+64);free(u.data);
    if(units&&uxQueueMessagesWaiting(units)==0)wake_refresh();
}
static void codec_trace(void *,int,const char *message)
{portENTER_CRITICAL(&mux);snprintf(view.codec_error,sizeof(view.codec_error),"%s",message?message:"");portEXIT_CRITICAL(&mux);}
static ISVCDecoder *new_decoder()
{
    ISVCDecoder *d=nullptr;if(WelsCreateDecoder(&d)||!d)return nullptr;
    int trace=WELS_LOG_ERROR;d->SetOption(DECODER_OPTION_TRACE_LEVEL,&trace);
    WelsTraceCallback cb=codec_trace;d->SetOption(DECODER_OPTION_TRACE_CALLBACK,&cb);
    int threads=0;d->SetOption(DECODER_OPTION_NUM_OF_THREADS,&threads);
    SDecodingParam p{};p.eEcActiveIdc=ERROR_CON_DISABLE;p.uiTargetDqLayer=255;
    p.sVideoProperty.size=sizeof(p.sVideoProperty);p.sVideoProperty.eVideoBsType=VIDEO_BITSTREAM_AVC;
    if(d->Initialize(&p)){WelsDestroyDecoder(d);return nullptr;}return d;
}
static uint8_t clamp(int x){return(uint8_t)(x<0?0:x>255?255:x);}
template<bool Mapped> static void convert_pixels(unsigned char **planes,const SBufferInfo &info,uint16_t *rgb)
{
    int w=info.UsrData.sSystemBuffer.iWidth,h=info.UsrData.sSystemBuffer.iHeight;
    int ys=info.UsrData.sSystemBuffer.iStride[0],cs=info.UsrData.sSystemBuffer.iStride[1];
    uint16_t xmap[Mapped?DW:1];
    if(Mapped)for(unsigned x=0;x<DW;x++)xmap[x]=(uint16_t)(x*w/DW);
    for(unsigned y=0;y<DH;y++){
        unsigned sy=y*h/DH;
        for(unsigned x=0;x<DW;x++){
            unsigned sx=Mapped?xmap[x]:x*w/DW;int Y=planes[0][sy*ys+sx]-16,U=planes[1][(sy/2)*cs+sx/2]-128,V=planes[2][(sy/2)*cs+sx/2]-128;
            // Existing physically accepted BT.709 limited-range conversion.
            uint8_t r=clamp((298*Y+459*V+128)>>8),g=clamp((298*Y-55*U-136*V+128)>>8),b=clamp((298*Y+541*U+128)>>8);
            rgb[y*DW+x]=(uint16_t)(((r>>3)<<11)|((g>>2)<<5)|(b>>3));
        }
    }
}
static void convert(unsigned char **planes,const SBufferInfo &info,uint16_t *rgb)
{convert_pixels<false>(planes,info,rgb);} // Coordinate-map candidate showed no measured gain.
static int decode_frame(ISVCDecoder *decoder,const uint8_t *data,size_t size,unsigned char **planes,SBufferInfo &info,int *frame_number=nullptr,uint64_t stamp=0,bool *direct=nullptr)
{
    if(frame_number)*frame_number=-1;
    if(direct)*direct=false;
    info.uiInBsTimeStamp=stamp;
    int result=(int)decoder->DecodeFrame2(data,(int)size,planes,&info);
    int number=-1;decoder->GetOption(DECODER_OPTION_FRAME_NUM,&number);
    if(frame_number&&number>=0)*frame_number=number;
    if(!result&&info.iBufferStatus==1&&info.uiOutYuvTimeStamp==stamp){if(direct)*direct=true;return result;}
    unsigned char *tail_planes[3]={};SBufferInfo tail{};
    tail.uiInBsTimeStamp=stamp;
    result|=(int)decoder->DecodeFrame2(nullptr,0,tail_planes,&tail);
    number=-1;decoder->GetOption(DECODER_OPTION_FRAME_NUM,&number);
    if(frame_number&&number>=0)*frame_number=number;
    if(tail.iBufferStatus&&(!info.iBufferStatus||tail.uiOutYuvTimeStamp>=info.uiOutYuvTimeStamp)){info=tail;memcpy(planes,tail_planes,sizeof(tail_planes));}
    if(!result&&!info.iBufferStatus)result=(int)decoder->FlushFrame(planes,&info);
    return result;
}
static bool digest(const void *data,size_t size,uint8_t *out)
{
    size_t n=0;return psa_crypto_init()==PSA_SUCCESS&&psa_hash_compute(PSA_ALG_SHA_256,
        (const uint8_t*)data,size,out,32,&n)==PSA_SUCCESS&&n==32;
}
static void hex_digest(const uint8_t *hash,char *hex){for(unsigned i=0;i<32;i++)snprintf(hex+i*2,3,"%02x",hash[i]);}
static void benchmark(ISVCDecoder *decoder,uint16_t *back)
{
    if(!decoder||!bench_ready.load())return;
    portENTER_CRITICAL(&mux);view.benchmark=true;portEXIT_CRITICAL(&mux);
    uint8_t sh[32],expected[32]={};char hex[65];
    if(!digest(bench_sample,bench_size,sh))goto done;
    hex_digest(sh,hex);
#ifdef OFT_CABAC32
    printf("OFT_CABAC_VARIANT version=%u\n",(unsigned)OFT_CABAC32);
#endif
    printf("OFT_BENCH_BEGIN bytes=%u source_us=%lld sample_sha256=%s residual_iram=%d\n",(unsigned)bench_size,(long long)bench_source_us,hex,OFT_RESIDUAL_IRAM);
#ifdef OFT_CABAC32
    for(unsigned trial=0;trial<16;trial++){
#else
    for(unsigned trial=0;trial<8;trial++){
#endif
        oft_udp_snapshot_t s;oft_udp_snapshot(&s);
        if(s.state!=OFT_UDP_READY||s.control_state==OFT_CONTROL_ACTIVE||s.head_held||s.action_busy){printf("OFT_BENCH_ABORT reason=control_or_link\n");break;}
        unsigned char *planes[3]={};SBufferInfo info{};bool profile=(trial&1)!=0;
        bool cabac32=false;
#ifdef OFT_CABAC32
        cabac32=(trial&1)!=0;profile=trial>=8;
#endif
        oft_cabac32_select(cabac32);
        oft_h264_profile_begin(profile);int64_t begin=esp_timer_get_time();
        uint64_t expected_stamp=(uint64_t)begin;
        int rc=decode_frame(decoder,bench_sample,bench_size,planes,info,nullptr,expected_stamp);
        int64_t elapsed=esp_timer_get_time()-begin;OftH264Profile p=oft_h264_profile_end();
        if(rc||info.iBufferStatus!=1||info.uiOutYuvTimeStamp!=expected_stamp||!planes[0]||!planes[1]||!planes[2]||
           info.UsrData.sSystemBuffer.iWidth<=0||info.UsrData.sSystemBuffer.iWidth>1280||
           info.UsrData.sSystemBuffer.iHeight<=0||info.UsrData.sSystemBuffer.iHeight>720){printf("OFT_BENCH_ABORT result=%d status=%d\n",rc,info.iBufferStatus);break;}
        convert(planes,info,back);uint8_t pixels[32];
        if(!digest(back,RGB_BYTES,pixels))break;
        if(!trial)memcpy(expected,pixels,32);
        bool equal=!memcmp(expected,pixels,32);hex_digest(pixels,hex);
        printf("OFT_BENCH trial=%u profile=%d iram_cabac=%d cabac32=%d decode_us=%lld cabac_us=%llu cavlc_us=%llu intra_us=%llu deblock_us=%llu calls=%u,%u,%u,%u equal=%d pixel_sha256=%s\n",
            trial,profile,OFT_CABAC_IRAM,cabac32,(long long)elapsed,(unsigned long long)p.us[0],(unsigned long long)p.us[1],
            (unsigned long long)p.us[2],(unsigned long long)p.us[3],p.calls[0],p.calls[1],p.calls[2],p.calls[3],equal,hex);
        if(!equal)break;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    printf("OFT_BENCH_END replay_not_presented=1\n");
done:
    oft_cabac32_select(false);
    portENTER_CRITICAL(&mux);view.benchmark=false;portEXIT_CRITICAL(&mux);
}
extern "C" void oft_video_sample_dump(void)
{
    if(!bench_ready.load()){printf("OFT_SAMPLE_NOT_READY\n");return;}
    uint8_t hash[32];char hex[65];if(!digest(bench_sample,bench_size,hash))return;hex_digest(hash,hex);
    printf("OFT_SAMPLE_BEGIN bytes=%u sha256=%s source_us=%lld\n",(unsigned)bench_size,hex,(long long)bench_source_us);
    for(size_t at=0;at<bench_size;at+=288){unsigned char b[385];size_t used=0,n=bench_size-at<288?bench_size-at:288;
        mbedtls_base64_encode(b,sizeof(b),&used,bench_sample+at,n);b[used]=0;printf("OFT_SAMPLE_DATA %u %s\n",(unsigned)(at/288),b);}
    printf("OFT_SAMPLE_END\n");
}
static void decode_worker(void *)
{
    auto *back=(uint16_t*)heap_caps_malloc(RGB_BYTES,MALLOC_CAP_SPIRAM);
    if(!back){detail("Video unavailable: RGB allocation");vTaskDelete(nullptr);return;}
    ISVCDecoder *decoder=nullptr;unsigned current_epoch=0;int expected_frame_num=1;Unit u;
    int64_t fps_start=esp_timer_get_time();unsigned fps_count=0;
    for(;;){
        if(bench_requested.exchange(false))benchmark(decoder,back);
        if(xQueueReceive(units,&u,pdMS_TO_TICKS(100))!=pdTRUE){
            oft_udp_snapshot_t s;oft_udp_snapshot(&s);
            if(decoder&&(s.state==OFT_UDP_OFF||s.state==OFT_UDP_FAULT)){
                decoder->Uninitialize();WelsDestroyDecoder(decoder);decoder=nullptr;
            }
            continue;
        }
        if(u.epoch!=epoch.load()){release(u);continue;}
        bool first=!decoder||current_epoch!=u.epoch;
        unsigned predicted_ms;portENTER_CRITICAL(&mux);predicted_ms=view.p_decode_ms+view.convert_ms;portEXIT_CRITICAL(&mux);
        bool predicted_late=!u.idr&&!prediction_timely(u.at,esp_timer_get_time(),predicted_ms,work_budget_ms());
        if(!timely(u.at,esp_timer_get_time(),first)||predicted_late){
            portENTER_CRITICAL(&mux);view.deadline_drops++;portEXIT_CRITICAL(&mux);
            epoch.fetch_add(1);release(u);detail(fresh_mode.load()?"FRESH: old tail dropped; waiting for IDR":"VIDEO PAUSED: decode cannot keep up");continue;
        }
        bool ready=false;
        while(u.epoch==epoch.load()){
            oft_udp_snapshot_t s;oft_udp_snapshot(&s);
            if(s.state==OFT_UDP_READY){ready=true;break;}
            if(s.state==OFT_UDP_OFF||s.state==OFT_UDP_FAULT)break;
            vTaskDelay(pdMS_TO_TICKS(20));
        }
        if(!ready||u.epoch!=epoch.load()||!timely(u.at,esp_timer_get_time(),first)){release(u);continue;}
        if(esp_timer_get_time()>=simulated_until.load())simulated_losses=0;
        unsigned skips=simulated_losses.load();
        if(skips&&simulated_losses.compare_exchange_strong(skips,skips-1)){
            printf("OFT_VIDEO_SIMULATED_LOSS remaining=%u source_us=%lld\n",skips-1,(long long)u.at);
            release(u);continue;
        }
        if(!decoder||current_epoch!=u.epoch){
            // A verified complete IDR resets references itself. Reuse allocated
            // picture storage instead of zeroing ~15MB on every sparse refresh.
            if(!decoder)decoder=new_decoder();
            current_epoch=u.epoch;
            if(!decoder){detail("Video decoder allocation failed");epoch.fetch_add(1);release(u);continue;}
        }
        portENTER_CRITICAL(&mux);view.busy=true;view.attempts++;view.decode_input_bytes=u.size;
        view.queue_age_ms=(esp_timer_get_time()-u.at)/1000;portEXIT_CRITICAL(&mux);
        unsigned char *planes[3]={};SBufferInfo info{};
        int64_t begin=esp_timer_get_time();
        // Some AUs already produce output on the data call. NoDelay's unconditional
        // null call can overwrite that SBufferInfo; preserve both call results.
        int frame_num=-1;bool direct=false;int result=decode_frame(decoder,u.data,u.size,planes,info,&frame_num,(uint64_t)u.at,&direct);
        unsigned decode_ms=(esp_timer_get_time()-begin)/1000;
        if(!result&&info.iBufferStatus!=1)result=(int)dsFramePending; // Complete AU + drain must yield its picture.
        // No per-picture flush/reset: keep the real inter-frame reference chain.
        bool good=!result&&info.iBufferStatus==1&&planes[0]&&planes[1]&&planes[2];
        if(good&&info.uiOutYuvTimeStamp!=(uint64_t)u.at){
            portENTER_CRITICAL(&mux);view.timestamp_mismatches++;portEXIT_CRITICAL(&mux);
            printf("OFT_VIDEO_TIMESTAMP expected=%lld output=%llu\n",(long long)u.at,info.uiOutYuvTimeStamp);
            good=false;result=(int)dsBitstreamError;
        }
        if(good){
            if(u.idr)expected_frame_num=frame_num+1;
            else if(frame_num<0||frame_num!=expected_frame_num){
                printf("OFT_VIDEO_BURST discontinuity expected=%d observed=%d\n",expected_frame_num,frame_num);
                good=false;result=(int)dsBitstreamError;
            }else ++expected_frame_num;
        }
        int w=info.UsrData.sSystemBuffer.iWidth,h=info.UsrData.sSystemBuffer.iHeight;
        good=good&&w>0&&w<=1280&&h>0&&h<=720;unsigned convert_ms=0;
        if(good&&u.epoch==epoch.load()){
            oft_udp_snapshot_t src;oft_udp_snapshot(&src);
            if(!bench_ready.load()&&u.size<=200*1024&&src.keyframe_replies&&u.at>src.keyframe_sent_us){
                const uint8_t *sample=u.data;size_t size=u.size;int64_t source=u.at;
#ifdef OFT_BENCH_EMBEDDED
                sample=fixture_begin;size=fixture_end-fixture_begin;source=0;
#endif
                bench_sample=(uint8_t*)heap_caps_calloc(1,size+64,MALLOC_CAP_SPIRAM);
                if(bench_sample){memcpy(bench_sample,sample,size);bench_size=size;bench_source_us=source;bench_ready=true;}
            }
            int64_t convert_start=esp_timer_get_time();convert(planes,info,back);
            convert_ms=(esp_timer_get_time()-convert_start)/1000;
            uint32_t hash=checksum((const uint8_t*)back,RGB_BYTES);
            xSemaphoreTake(image_lock,portMAX_DELAY);
            uint16_t *old=published;published=back;back=old;image_generation++;image_source_us=u.at;
            uint32_t gen=image_generation;xSemaphoreGive(image_lock);
            portENTER_CRITICAL(&mux);view.ready=true;view.width=w;view.height=h;
            view.display_width=DW;view.display_height=DH;view.generation=gen;
            view.decoded_frames++;view.last_frame_us=esp_timer_get_time();view.last_source_us=u.at;
            if(direct)view.direct_outputs++;else view.drained_outputs++;
            view.output_age_ms=(unsigned)((view.last_frame_us-u.at)/1000);
            if(u.idr){view.last_idr_generation=gen;view.last_idr_hash=hash;view.last_idr_source_us=u.at;view.last_idr_frame_us=view.last_frame_us;view.last_idr_decode_ms=decode_ms;}
            else{view.decoded_predicted_frames++;view.p_decode_ms=view.p_decode_ms?(view.p_decode_ms*3+decode_ms)/4:decode_ms;}
            portEXIT_CRITICAL(&mux);
            portENTER_CRITICAL(&mux);view.pixel_hash=hash;portEXIT_CRITICAL(&mux);
            printf("OFT_VIDEO_PICTURE generation=%lu source_us=%lld decode_ms=%u hash=%08lx idr=%d completed_us=%lld\n",(unsigned long)gen,(long long)u.at,decode_ms,(unsigned long)hash,u.idr,(long long)esp_timer_get_time());
            fps_count++;detail("Video decoded on Mosaico");
        }else if(result){
            oft_video_idr_only(); // Dependency failure returns to IDR-only, never corrupt sampling.
            portENTER_CRITICAL(&mux);view.decode_errors++;portEXIT_CRITICAL(&mux);
            epoch.fetch_add(1);detail("Video reference lost; waiting for IDR");
        }
        int64_t now=esp_timer_get_time();unsigned stack_free=uxTaskGetStackHighWaterMark(nullptr);
        portENTER_CRITICAL(&mux);view.busy=false;view.decoder_result=result;view.decode_ms=decode_ms;view.convert_ms=convert_ms;
        view.stack_free=stack_free;
        if(now-fps_start>=5000000){view.fps_milli=(uint64_t)fps_count*1000000000/(now-fps_start);fps_start=now;fps_count=0;}
        portEXIT_CRITICAL(&mux);release(u);vTaskDelay(1);
    }
}
static void test_video_latency_budget(void)
{
    TEST_ASSERT_TRUE(timely(1000000,2500000,false));
    TEST_ASSERT_FALSE(timely(1000000,2500001,false));
    TEST_ASSERT_TRUE(timely(1000000,5000000,true));
    TEST_ASSERT_FALSE(timely(1000000,5000001,true));
    TEST_ASSERT_FALSE(timely(1000000,999999,true));
}
static void test_video_copy_bounds(void)
{
    uint16_t pixel=0x1234;
    TEST_ASSERT_EQUAL_UINT(0,oft_video_copy(nullptr,352*198*2,0));
    TEST_ASSERT_EQUAL_UINT(0,oft_video_copy(&pixel,sizeof(pixel),0));
    TEST_ASSERT_EQUAL_HEX16(0x1234,pixel);
}
static void test_conversion_mapping_is_bit_exact(void)
{
    auto *source=(uint8_t*)heap_caps_malloc(1280*720*3/2,MALLOC_CAP_SPIRAM);
    auto *expected=(uint16_t*)heap_caps_malloc(RGB_BYTES,MALLOC_CAP_SPIRAM);
    auto *actual=(uint16_t*)heap_caps_malloc(RGB_BYTES,MALLOC_CAP_SPIRAM);
    if(!source||!expected||!actual){free(source);free(expected);free(actual);TEST_FAIL_MESSAGE("Conversion test allocation");return;}
    for(unsigned i=0;i<1280*720*3/2;i++)source[i]=(uint8_t)((i*37)^(i>>8));
    unsigned char *planes[]={source,source+1280*720,source+1280*720+640*360};
    SBufferInfo info{};info.UsrData.sSystemBuffer.iStride[0]=1280;info.UsrData.sSystemBuffer.iStride[1]=640;
    bool good=true;unsigned ref_us=0,map_us=0;
    const unsigned sizes[][2]={{1280,720},{640,360},{352,198},{17,17}};
    for(const auto &size:sizes){
        info.UsrData.sSystemBuffer.iWidth=size[0];info.UsrData.sSystemBuffer.iHeight=size[1];
        int64_t before=esp_timer_get_time();convert_pixels<false>(planes,info,expected);
        ref_us+=(unsigned)(esp_timer_get_time()-before);
        before=esp_timer_get_time();convert_pixels<true>(planes,info,actual);
        map_us+=(unsigned)(esp_timer_get_time()-before);
        if(memcmp(expected,actual,RGB_BYTES))good=false;
    }
    free(source);free(expected);free(actual);
    portENTER_CRITICAL(&mux);view.convert_reference_us=ref_us;view.convert_mapped_us=map_us;portEXIT_CRITICAL(&mux);
    TEST_ASSERT_TRUE(good);
}
static void test_video_refresh_correlates_reply_and_picture(void)
{
    oft_udp_snapshot_t u{};oft_video_snapshot_t s{};u.state=OFT_UDP_READY;
    u.keyframe_requests=u.keyframe_replies=1;u.keyframe_sent_us=100;s.last_idr_generation=2;s.last_idr_source_us=101;
    TEST_ASSERT_TRUE(refresh_success(u,s,0,1));
    s.last_idr_source_us=99;s.generation=3;s.last_source_us=105;TEST_ASSERT_FALSE(refresh_success(u,s,0,1));s.last_idr_source_us=101;
    u.keyframe_replies=0;TEST_ASSERT_FALSE(refresh_success(u,s,0,1));u.keyframe_replies=1;
    u.keyframe_failures=1;TEST_ASSERT_FALSE(refresh_success(u,s,0,1));u.keyframe_failures=0;
    u.state=OFT_UDP_FAULT;TEST_ASSERT_FALSE(refresh_success(u,s,0,1));
}
static void test_video_refresh_rejects_fast_rate(void)
{TEST_ASSERT_FALSE(oft_video_refresh(1));TEST_ASSERT_FALSE(oft_video_refresh(OFT_VIDEO_MIN_INTERVAL_MS-1));}
static void test_video_lwip_prefers_psram(void)
{
    void *p=mem_malloc(2048);TEST_ASSERT_NOT_NULL(p);
    if(heap_caps_get_total_size(MALLOC_CAP_SPIRAM))TEST_ASSERT_TRUE(esp_ptr_external_ram(p));
    mem_free(p);
}
static void test_psram_pattern_integrity(void)
{
    const size_t count=512*1024/sizeof(uint32_t);
    volatile uint32_t *p=(volatile uint32_t*)heap_caps_malloc(count*sizeof(uint32_t),MALLOC_CAP_SPIRAM);
    TEST_ASSERT_NOT_NULL((void*)p);bool good=true;
    const uint32_t seeds[]={0,0xffffffffu,0x55555555u,0xaaaaaaaau};
    for(uint32_t seed:seeds){
        for(size_t i=0;i<count;i++)p[i]=(uint32_t)(i*2654435761u)^seed;
        for(size_t i=0;i<count;i++)if(p[i]!=((uint32_t)(i*2654435761u)^seed))good=false;
    }
    free((void*)p);TEST_ASSERT_TRUE(good);
}
static void test_video_missing_picture_recovery_is_bounded(void)
{
    oft_udp_snapshot_t u{};oft_video_snapshot_t s{};u.state=OFT_UDP_READY;
    u.keyframe_requests=u.keyframe_replies=1;u.keyframe_sent_us=100;s.last_idr_generation=1;
    TEST_ASSERT_TRUE(recover_missing(u,s,0,1,true,0,8000001));
    TEST_ASSERT_FALSE(recover_missing(u,s,0,1,true,1,8000001));
    TEST_ASSERT_FALSE(recover_missing(u,s,0,1,false,0,8000001));
    TEST_ASSERT_FALSE(recover_missing(u,s,0,1,true,0,8000000));
    TEST_ASSERT_FALSE(recover_missing(u,s,1,1,true,0,8000001));
    u.keyframe_replies=0;TEST_ASSERT_FALSE(recover_missing(u,s,0,1,true,0,8000001));
    u.keyframe_replies=1;s.last_idr_generation=2;s.last_idr_source_us=101;
    TEST_ASSERT_FALSE(recover_missing(u,s,0,1,true,0,8000001));
    s.last_idr_generation=1;s.lost_idr_source_us=101;
    TEST_ASSERT_TRUE(recover_missing(u,s,0,1,true,0,(int64_t)OFT_VIDEO_MIN_INTERVAL_MS*1000));
    TEST_ASSERT_FALSE(recover_missing(u,s,0,1,true,0,(int64_t)OFT_VIDEO_MIN_INTERVAL_MS*1000-1));
    TEST_ASSERT_FALSE(recover_missing(u,s,0,1,true,1,(int64_t)OFT_VIDEO_MIN_INTERVAL_MS*1000));
    s.busy=true;TEST_ASSERT_FALSE(recover_missing(u,s,0,1,true,0,3000000));s.busy=false;
    s.lost_idr_source_us=100;TEST_ASSERT_FALSE(recover_missing(u,s,0,1,true,0,3000000));
}
static void test_image_retry_requires_healthy_reply_and_page(void)
{
    oft_udp_snapshot_t u{};u.state=OFT_UDP_READY;u.keyframe_requests=u.keyframe_replies=4;
    u.head_held=true;u.control_state=OFT_CONTROL_ACTIVE;
    TEST_ASSERT_TRUE(retry_image_only(u,3,true,true)); // Holding must not stop video.
    TEST_ASSERT_FALSE(retry_image_only(u,3,false,true));
    TEST_ASSERT_FALSE(retry_image_only(u,3,true,false));
    TEST_ASSERT_FALSE(retry_image_only(u,4,true,true));
    u.keyframe_pending=true;TEST_ASSERT_FALSE(retry_image_only(u,3,true,true));u.keyframe_pending=false;
    u.keyframe_replies=3;TEST_ASSERT_FALSE(retry_image_only(u,3,true,true));u.keyframe_replies=4;
    u.keyframe_failures=1;TEST_ASSERT_FALSE(retry_image_only(u,3,true,true));u.keyframe_failures=0;
    u.state=OFT_UDP_FAULT;TEST_ASSERT_FALSE(retry_image_only(u,3,true,true));
}
static void test_image_backoff_is_bounded(void)
{
    TEST_ASSERT_EQUAL_UINT(4000,image_backoff_ms(0));TEST_ASSERT_EQUAL_UINT(8000,image_backoff_ms(1));
    TEST_ASSERT_EQUAL_UINT(16000,image_backoff_ms(2));TEST_ASSERT_EQUAL_UINT(30000,image_backoff_ms(3));
    TEST_ASSERT_EQUAL_UINT(30000,image_backoff_ms(UINT32_MAX));
}
static bool admit_sparse_picture(bool idr,bool vcl,unsigned kind,unsigned budget,bool &sync,unsigned &remaining)
{
    if(idr)return true;
    if(!budget||!remaining||!sync||!vcl)return false;
    if(kind!=0){sync=false;remaining=0;return false;} // No B/unknown-frame sampling.
    --remaining;return true;
}
static void test_burst_requires_contiguous_prefix(void)
{
    bool sync=true;unsigned remaining=3;
    TEST_ASSERT_FALSE(admit_sparse_picture(false,false,9,3,sync,remaining));TEST_ASSERT_EQUAL_UINT(3,remaining);
    for(unsigned i=0;i<3;i++)TEST_ASSERT_TRUE(admit_sparse_picture(false,true,0,3,sync,remaining));
    TEST_ASSERT_FALSE(admit_sparse_picture(false,true,0,3,sync,remaining));
    remaining=3;sync=false;TEST_ASSERT_FALSE(admit_sparse_picture(false,true,0,3,sync,remaining));
    sync=true;TEST_ASSERT_FALSE(admit_sparse_picture(false,true,1,3,sync,remaining));TEST_ASSERT_FALSE(sync);
    TEST_ASSERT_TRUE(admit_sparse_picture(true,true,2,3,sync,remaining));
    TEST_ASSERT_EQUAL_UINT(3,predicted_prefix(3,400,200,20));
    TEST_ASSERT_EQUAL_UINT(2,predicted_prefix(3,900,300,20));
    TEST_ASSERT_EQUAL_UINT(1,predicted_prefix(3,1100,400,20));
    TEST_ASSERT_EQUAL_UINT(0,predicted_prefix(3,1550,400,20));
    TEST_ASSERT_EQUAL_UINT(0,predicted_prefix(0,400,200,20));
    TEST_ASSERT_EQUAL_UINT(2,predicted_prefix(3,0,0,0));
    TEST_ASSERT_EQUAL_UINT(5,predicted_prefix(6,600,160,20));
    TEST_ASSERT_EQUAL_UINT(6,predicted_prefix(6,400,100,20));
    TEST_ASSERT_EQUAL_UINT(2,predicted_prefix(6,900,300,20));
    TEST_ASSERT_FALSE(oft_video_burst_trial(0));
    TEST_ASSERT_FALSE(oft_video_burst_trial(7));
    TEST_ASSERT_EQUAL_UINT(0,predicted_prefix(3,UINT32_MAX,UINT32_MAX,20));
}
static void test_fresh_picture_budget(void)
{
    TEST_ASSERT_EQUAL_UINT(0,predicted_prefix(1,757,253,20,900));
    TEST_ASSERT_EQUAL_UINT(1,predicted_prefix(1,600,160,20,900));
    TEST_ASSERT_EQUAL_UINT(0,predicted_prefix(1,1000,100,20,900));
    TEST_ASSERT_TRUE(prediction_timely(1000000,1650000,250,900));
    TEST_ASSERT_FALSE(prediction_timely(1000000,1650001,250,900));
    TEST_ASSERT_FALSE(prediction_timely(1000000,999999,0,900));
    TEST_ASSERT_FALSE(prediction_timely(1000000,1000000,UINT32_MAX,900));
    TEST_ASSERT_EQUAL_UINT(3,predicted_prefix(3,600,160,20,1600));
}
#ifdef OFT_CABAC32
static void test_cabac32_differential(){TEST_ASSERT_TRUE(oft_cabac32_selftest());}
#endif
extern "C" void oft_video_selftests(void)
{RUN_TEST(test_video_latency_budget);RUN_TEST(test_video_copy_bounds);RUN_TEST(test_video_refresh_correlates_reply_and_picture);RUN_TEST(test_video_refresh_rejects_fast_rate);RUN_TEST(test_video_lwip_prefers_psram);RUN_TEST(test_psram_pattern_integrity);RUN_TEST(test_video_missing_picture_recovery_is_bounded);RUN_TEST(test_image_retry_requires_healthy_reply_and_page);RUN_TEST(test_image_backoff_is_bounded);
RUN_TEST(test_burst_requires_contiguous_prefix);
RUN_TEST(test_fresh_picture_budget);
RUN_TEST(test_conversion_mapping_is_bit_exact);
#ifdef OFT_CABAC32
RUN_TEST(test_cabac32_differential);
#endif
}
static void assembly_worker(void *)
{
    auto *g=(uint8_t*)heap_caps_malloc(OFT_MEDIA_GROUP_BYTES,MALLOC_CAP_SPIRAM);
    auto *au=(uint8_t*)heap_caps_malloc(AU_CAP,MALLOC_CAP_SPIRAM);
    if(!g||!au){detail("Video assembly allocation failed");free(g);free(au);vTaskDelete(nullptr);return;}
    oft_media_t assembler;oft_media_init(&assembler,g,au,AU_CAP);
    uint8_t config[4096];size_t config_size=0;uint32_t config_mask=0;
    bool synchronized=false;unsigned seen_epoch=epoch.load(),remaining_predicted=0;uint16_t session=0;
    int64_t last_packet=0;Packet p;detail("Waiting for SPS/PPS and IDR");accepting=true;
    for(;;){
        if(xQueueReceive(packets,&p,pdMS_TO_TICKS(100))!=pdTRUE)continue;
        if(reassembly_reset.exchange(false)){
            remaining_predicted=0;
            if(trace_armed.exchange(false)){trace_count=0;trace_until=p.at+1000000;trace_active=true;}
            // The camera can restart group numbering on an IDR request. Do not
            // mistake the new independent picture for a recently ACKed old one.
            assembler.group_active=assembler.continuation=assembler.protect_idr=false;assembler.used=0;assembler.received=0;
            memset(assembler.finished_at,0,sizeof(assembler.finished_at));
        }
        if(p.size<20||p.size>sizeof(p.bytes)||checksum(p.bytes,p.size)!=p.hash){
            portENTER_CRITICAL(&mux);view.copy_errors++;portEXIT_CRITICAL(&mux);epoch.fetch_add(1);continue;
        }
        uint16_t next_session=p.bytes[2]|((uint16_t)p.bytes[3]<<8);
        if(next_session!=session||(last_packet&&p.at-last_packet>2000000)){
            session=next_session;config_size=0;config_mask=0;epoch.fetch_add(1);oft_media_init(&assembler,g,au,AU_CAP);
        }
        last_packet=p.at;
        if(seen_epoch!=epoch.load()){seen_epoch=epoch.load();synchronized=false;}
        unsigned lost=assembler.dropped;size_t n=oft_media_feed(&assembler,p.bytes,p.size,p.at);
        if(trace_active.load()){
            if(p.at>trace_until||trace_count==256){trace_active=false;trace_ready=true;}
            else {
                Trace &t=traces[trace_count++];t.at=p.at;t.size=p.size;memset(t.prefix,0,64);
                memcpy(t.prefix,p.bytes,p.size<64?p.size:64);t.bitmap=assembler.received;
                t.declared=assembler.declared;t.used=assembler.used;t.complete=n;t.dropped=assembler.dropped;
                t.group=assembler.group_id;t.count=assembler.count;t.protected_idr=assembler.protect_idr;
            }
        }
        if(assembler.dropped!=lost&&synchronized){synchronized=false;detail("Video fragment loss; waiting for IDR");}
        portENTER_CRITICAL(&mux);view.units=assembler.complete;view.assembly_drops=assembler.dropped;
        view.invalid=assembler.invalid+assembler.conflicts;view.lost_idr_source_us=assembler.lost_idr_source_us;view.lost_idrs=assembler.lost_idrs;portEXIT_CRITICAL(&mux);
        if(!n)continue;
        uint32_t mask=oft_media_nal_mask(au,n);
        unsigned ref=0,kind=9;bool vcl=oft_media_slice(au,n,&ref,&kind);
        portENTER_CRITICAL(&mux);view.slice_type=kind;view.nal_reference=ref;
        if(vcl&&kind==2)view.intra_units++;
        if(mask&(1u<<5))view.idr_units++;
        if(vcl&&!ref)view.nonreference_units++;
        portEXIT_CRITICAL(&mux);
        if(assembler.complete<12||assembler.complete%300==0||(vcl&&kind==2)){
            printf("OFT_VIDEO_AU number=%u bytes=%u mask=%08lx vcl=%d ref=%u slice=%u\n",assembler.complete,(unsigned)n,(unsigned long)mask,vcl,ref,kind);
        }
        if((mask&0x180)&&!(mask&((1u<<5)|(1u<<1)))&&n<=sizeof(config)){
            if(mask&(1u<<7)){config_size=0;config_mask=0;}
            if(config_size+n<=sizeof(config)){memcpy(config+config_size,au,n);config_size+=n;config_mask|=mask;}
        }
        portENTER_CRITICAL(&mux);view.nal_mask=mask;view.au_bytes=n;portEXIT_CRITICAL(&mux);
        bool idr=(mask&(1u<<5))!=0;
        if(sparse.load()&&!admit_sparse_picture(idr,vcl,kind,burst_budget(),synchronized,remaining_predicted))continue;
        size_t prefix=0;
        if(mask&(1u<<5)){
            // A genuine new random-access point supersedes old queued pictures.
            // This also prevents startup/leftover GOPs hiding enable's fresh IDR.
            synchronized=false;
        }
        if(!synchronized){
            if(!(mask&(1u<<5))||(((mask|config_mask)&0x180)!=0x180))continue;
            seen_epoch=epoch.fetch_add(1)+1;
            Unit obsolete;while(xQueueReceive(units,&obsolete,0)==pdTRUE)release(obsolete);
            prefix=config_size;
        }
        // Bound bytes and count. Never skip arbitrary P references or grow delay forever.
        size_t bytes=prefix+n+64;uint8_t *data=nullptr;
        if(queued_bytes.load()+bytes<=QUEUED_CAP)data=(uint8_t*)heap_caps_malloc(bytes,MALLOC_CAP_SPIRAM);
        bool ok=false;
        if(data){
            memcpy(data,config,prefix);memcpy(data+prefix,au,n);memset(data+prefix+n,0,64);
            Unit u{data,prefix+n,p.at,seen_epoch,idr};queued_bytes.fetch_add(bytes);
            ok=xQueueSend(units,&u,0)==pdTRUE;if(!ok)release(u);
        }
        if(!ok){
            portENTER_CRITICAL(&mux);view.au_queue_drops++;portEXIT_CRITICAL(&mux);
            synchronized=false;detail("Decoder overloaded; waiting for IDR");
        }else{
            synchronized=true;
            if(idr){
                portENTER_CRITICAL(&mux);
                unsigned maximum=burst_budget();if(fresh_mode.load()&&maximum>1)maximum=1;
                remaining_predicted=predicted_prefix(maximum,view.last_idr_decode_ms,view.p_decode_ms,view.convert_ms,work_budget_ms());
                view.selected_p_frames=remaining_predicted;
                portEXIT_CRITICAL(&mux);
            }
        }
    }
}
extern "C" void oft_video_start(void)
{
    if(heap_caps_get_free_size(MALLOC_CAP_SPIRAM)<5000000){detail("Video requires PSRAM");return;}
    packet_storage=(uint8_t*)heap_caps_malloc(128*sizeof(Packet),MALLOC_CAP_SPIRAM);
    published=(uint16_t*)heap_caps_malloc(RGB_BYTES,MALLOC_CAP_SPIRAM);
    if(!packet_storage||!published){detail("Video allocation failed");free(packet_storage);free(published);return;}
    image_lock=xSemaphoreCreateMutexStatic(&image_control);
    packets=xQueueCreateStatic(128,sizeof(Packet),packet_storage,&packet_control);
    units=xQueueCreateStatic(128,sizeof(Unit),unit_storage,&unit_control);
    bool ok=xTaskCreatePinnedToCore(decode_worker,"oft_decode",24576,nullptr,1,nullptr,1)==pdPASS;
    // Drain bursts before low-priority decode; keep the control writer above us.
    if(ok)ok=xTaskCreatePinnedToCore(assembly_worker,"oft_assemble",12288,nullptr,4,nullptr,1)==pdPASS;
    if(ok)ok=xTaskCreate(refresh_worker,"oft_refresh",4096,nullptr,1,nullptr)==pdPASS;
    portENTER_CRITICAL(&mux);view.available=ok;portEXIT_CRITICAL(&mux);
    if(!ok)detail("Video worker unavailable");
}
static void le32(uint8_t *p,uint32_t v){for(unsigned i=0;i<4;i++)p[i]=(uint8_t)(v>>(8*i));}
extern "C" void oft_video_dump(void)
{
    auto *rgb=(uint16_t*)heap_caps_malloc(RGB_BYTES,MALLOC_CAP_SPIRAM);
    if(!rgb)return;
    uint32_t generation=oft_video_copy(rgb,RGB_BYTES,0);
    if(!generation){free(rgb);printf("OFT_IMAGE_NOT_READY\n");return;}
    size_t n=54+DW*DH*3;uint8_t *bmp=(uint8_t*)heap_caps_calloc(1,n,MALLOC_CAP_SPIRAM);
    if(!bmp){free(rgb);return;}
    bmp[0]='B';bmp[1]='M';le32(bmp+2,n);le32(bmp+10,54);le32(bmp+14,40);le32(bmp+18,DW);le32(bmp+22,(uint32_t)-(int32_t)DH);bmp[26]=1;bmp[28]=24;
    for(unsigned i=0;i<DW*DH;i++){uint16_t v=rgb[i];bmp[54+i*3]=(v&31)*255/31;bmp[55+i*3]=((v>>5)&63)*255/63;bmp[56+i*3]=(v>>11)*255/31;}
    free(rgb);uint8_t hash[32];size_t hash_size=0;
    if(psa_crypto_init()!=PSA_SUCCESS||psa_hash_compute(PSA_ALG_SHA_256,bmp,n,hash,sizeof(hash),&hash_size)!=PSA_SUCCESS||hash_size!=32){free(bmp);return;}
    char hex[65];for(unsigned i=0;i<32;i++)snprintf(hex+2*i,3,"%02x",hash[i]);
    printf("OFT_IMAGE_BEGIN bytes=%u sha256=%s generation=%lu\n",(unsigned)n,hex,(unsigned long)generation);
    for(size_t at=0;at<n;at+=288){unsigned char encoded[385];size_t used=0,k=n-at<288?n-at:288;mbedtls_base64_encode(encoded,sizeof(encoded),&used,bmp+at,k);encoded[used]=0;printf("OFT_IMAGE_DATA %u %s\n",(unsigned)(at/288),encoded);}
    printf("OFT_IMAGE_END\n");fflush(stdout);free(bmp);
}
