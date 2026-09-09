#include "oft_haptic.h"
#include <limits.h>
void oft_haptic_request(oft_haptic_pattern_t *s,bool active,int64_t now)
{
    if(!s)return;
    if(!active||now<0||now>INT64_MAX-100000){*s=(oft_haptic_pattern_t){0};return;}
    if(!s->active||now>=s->lease_until_us||now<s->started_us)s->started_us=now;
    s->active=true;s->lease_until_us=now+100000;
}
bool oft_haptic_output(const oft_haptic_pattern_t *s,int64_t now)
{
    return s&&s->active&&now>=s->started_us&&now<s->lease_until_us&&
        (now-s->started_us)%100000<50000;
}
