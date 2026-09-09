#include "oft_media.h"
#include "oft_wifi_wire.h"
#include <string.h>
void oft_media_init(oft_media_t *s,uint8_t *g,uint8_t *a,size_t cap)
{memset(s,0,sizeof(*s));s->group=g;s->au=a;s->capacity=cap;}
static void drop(oft_media_t *s){
    if(s->protect_idr){s->lost_idrs++;s->lost_idr_source_us=s->idr_started_us;}
    s->group_active=false;s->continuation=false;s->protect_idr=false;s->used=0;s->received=0;s->dropped++;
}
size_t oft_media_feed(oft_media_t *s,const uint8_t *p,size_t n,int64_t now)
{
    oft_wifi_packet_t wire;
    if(!s||!s->group||!s->au)return 0;
    if(!oft_wifi_parse(p,n,&wire)||wire.type!=2||n<=20||n>20+OFT_MEDIA_FRAGMENT_MAX){s->invalid++;return 0;}
    unsigned count=p[17]&127,index=2*(p[18]&31)+(p[17]>>7);uint8_t id=p[16];size_t bytes=n-20;
    if(!count||count>64||index>=count){s->invalid++;return 0;}
    if(s->finished_at[id]&&now>=s->finished_at[id]&&now-s->finished_at[id]<2000000){s->duplicates++;return 0;}
    if((s->group_active&&(now<s->group_us||now-s->group_us>500000))||
       (s->continuation&&(now<s->au_us||now-s->au_us>500000)))drop(s);
    if(s->protect_idr&&(s->group_active||s->continuation)&&id!=(s->group_active?s->group_id:s->next_id)){
        s->foreign_packets++;return 0; // Late unrelated groups must not destroy a completeable IDR.
    }
    if(s->group_active&&id!=s->group_id)drop(s);
    if(!s->group_active){s->group_active=true;s->group_id=id;s->count=count;s->received=0;s->group_us=now;}
    if(count!=s->count){s->conflicts++;drop(s);return 0;}
    uint8_t *slot=s->group+index*OFT_MEDIA_FRAGMENT_MAX;uint64_t bit=UINT64_C(1)<<index;
    if(s->received&bit){
        if(s->lengths[index]!=bytes||memcmp(slot,p+20,bytes)){s->conflicts++;drop(s);}
        else s->duplicates++;
        return 0;
    }
    memcpy(slot,p+20,bytes);s->lengths[index]=(uint16_t)bytes;s->received|=bit;
    if(index==0&&bytes>16&&!memcmp(slot,"\0\0\1\xff",4)&&(oft_media_nal_mask(slot+16,bytes-16)&(1u<<5))){
        s->protect_idr=true;s->idr_started_us=now;
    }
    uint64_t all=count==64?UINT64_MAX:((UINT64_C(1)<<count)-1);
    if(s->received!=all)return 0;
    s->group_active=false;s->finished_at[id]=now;bool start=s->lengths[0]>=16&&!memcmp(s->group,"\0\0\1\xff",4);
    if(start){
        const uint8_t *h=s->group;
        s->declared=(uint32_t)h[4]|((uint32_t)h[5]<<8)|((uint32_t)h[6]<<16)|((uint32_t)h[7]<<24);
        if(!s->declared||s->declared>s->capacity){s->invalid++;drop(s);return 0;}
        s->used=0;s->au_us=now;s->continuation=false;
        s->timestamp=(uint32_t)h[12]|((uint32_t)h[13]<<8)|((uint32_t)h[14]<<16)|((uint32_t)h[15]<<24);
    }else if(!s->continuation||id!=s->next_id){drop(s);return 0;}
    bool full=count==63;
    for(unsigned i=0;i<count;i++){
        size_t skip=start&&i==0?16:0,len=s->lengths[i]-skip;
        if(len>s->declared-s->used){s->invalid++;drop(s);return 0;}
        memcpy(s->au+s->used,s->group+i*OFT_MEDIA_FRAGMENT_MAX+skip,len);s->used+=len;
        if(s->lengths[i]!=OFT_MEDIA_FRAGMENT_MAX)full=false;
    }
    if(s->used==s->declared){s->continuation=false;s->protect_idr=false;s->complete++;return s->used;}
    if(!full){drop(s);return 0;}
    s->continuation=true;s->next_id=(uint8_t)(id+1);return 0;
}
uint32_t oft_media_nal_mask(const uint8_t *p,size_t n)
{
    uint32_t mask=0;
    for(size_t i=0;i+3<n;i++)if(!p[i]&&!p[i+1]&&p[i+2]==1){mask|=UINT32_C(1)<<(p[i+3]&31);i+=2;}
    return mask;
}
static bool read_ue(const uint8_t *p,unsigned bits,unsigned *at,unsigned *value)
{
    unsigned zeros=0;
    while(*at<bits&&!(p[*at/8]&(0x80>>(*at%8)))){(*at)++;if(++zeros>20)return false;}
    if(*at>=bits)return false;
    (*at)++;unsigned v=1;
    for(unsigned i=0;i<zeros;i++){if(*at>=bits)return false;v=2*v+!!(p[*at/8]&(0x80>>(*at%8)));(*at)++;}
    *value=v-1;return true;
}
bool oft_media_slice(const uint8_t *p,size_t n,unsigned *reference,unsigned *type)
{
    if(!p||!reference||!type)return false;
    for(size_t i=0;i+4<n;i++)if(!p[i]&&!p[i+1]&&p[i+2]==1){
        unsigned nal=p[i+3]&31;if(nal!=1&&nal!=5)continue;
        uint8_t rbsp[16];unsigned k=0,zeros=0;
        for(size_t j=i+4;j<n&&k<sizeof(rbsp);j++){
            if(zeros>=2&&p[j]==3){zeros=0;continue;}
            rbsp[k++]=p[j];zeros=p[j]?0:zeros+1;
        }
        unsigned at=0,first,kind;
        if(!read_ue(rbsp,k*8,&at,&first)||!read_ue(rbsp,k*8,&at,&kind)||kind>9)return false;
        *reference=(p[i+3]>>5)&3;*type=kind%5;return true;
    }
    return false;
}
