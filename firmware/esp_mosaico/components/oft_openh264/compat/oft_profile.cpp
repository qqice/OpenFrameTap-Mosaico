#include "oft_profile.h"
#include "esp_timer.h"
#include <cstring>
// Single decoder owner, including benchmarks. No logging or allocations in hooks.
static bool active;
static OftH264Profile result;
static unsigned depth[OFT_H264_PHASES];
void oft_h264_profile_begin(bool enabled){active=enabled;result={};memset(depth,0,sizeof(depth));}
OftH264Profile oft_h264_profile_end(){active=false;return result;}
int64_t oft_h264_profile_enter(unsigned p)
{
    if(!active)return 0;
    result.calls[p]++;
    return depth[p]++? -1:esp_timer_get_time();
}
void oft_h264_profile_leave(unsigned p,int64_t start)
{
    if(!start)return;
    if(--depth[p]==0&&start>0)result.us[p]+=esp_timer_get_time()-start;
}
