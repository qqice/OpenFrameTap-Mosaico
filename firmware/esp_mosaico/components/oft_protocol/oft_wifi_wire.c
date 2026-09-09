#include "oft_wifi_wire.h"
#include "oft_duml.h"
#include <string.h>

static uint16_t get16(const uint8_t *p){return p[0]|((uint16_t)p[1]<<8);}
static void put16(uint8_t *p,uint16_t n){p[0]=n;p[1]=n>>8;}
static void header(uint8_t *p,size_t n,uint16_t session,uint16_t seq,uint8_t type)
{
    put16(p,(uint16_t)n|0x8000);put16(p+2,session);put16(p+4,seq);p[6]=type;p[7]=0;
    for(unsigned i=0;i<7;i++)p[7]^=p[i];
}
bool oft_wifi_parse(const uint8_t *p,size_t n,oft_wifi_packet_t *v)
{
    if(!p||!v||n<8||n>OFT_WIFI_DATAGRAM_MAX||get16(p)!=(0x8000|n))return false;
    uint8_t check=0;for(unsigned i=0;i<8;i++)check^=p[i];
    if(check)return false;
    *v=(oft_wifi_packet_t){get16(p+2),get16(p+4),p[6],p,n};return true;
}
bool oft_wifi_init(oft_wifi_session_t *s,uint16_t session,uint16_t seed)
{
    if(!s||!session||(seed&7))return false;
    *s=(oft_wifi_session_t){.session=session,.next_sequence=(uint16_t)(seed+8),.last_sent=seed,.peer=seed,.message_counter=1};
    return true;
}
bool oft_wifi_direct_duml(const oft_wifi_packet_t *p,oft_duml_frame_t *f)
{
    /* DjiWifiUdpTransport decodes a direct reply before classifying WhType.
       In particular, normal-mode WhType02 cannot be assumed to be only video. */
    return p&&p->raw&&p->size>=33&&oft_duml_decode(p->raw+20,p->size-20,f);
}
size_t oft_wifi_handshake(const oft_wifi_session_t *s,uint8_t *out,size_t cap)
{
    /* Profile provenance: Python DjiWifiHandshakeProfile, Mimo capture 8e7c7eb6...
       Fixed qualities/MTU/interval/product capabilities; identity is per session. */
    static const uint8_t capabilities[]={1,0xc0,5,0x14,0,0,0x64,0,0x14,0,0x64,0,0xc0,5,0x14,0,0,0x64,0,1,1,4,1,2};
    const size_t n=24+sizeof(capabilities);
    if(!s||!out||cap<n||!s->session)return 0;
    header(out,n,s->session,0,0);put16(out+8,s->last_sent);
    put16(out+10,100);put16(out+12,100);put16(out+14,1472);put16(out+16,20);
    out[18]=0;out[19]=100; /* trailing quality is big endian */
    out[20]=0;out[21]=0;out[22]=1;out[23]=0x90;
    memcpy(out+24,capabilities,sizeof(capabilities));return n;
}
bool oft_wifi_handshake_accepted(const oft_wifi_session_t *s,const uint8_t *p,size_t n)
{
    oft_wifi_packet_t v;return s&&oft_wifi_parse(p,n,&v)&&v.session==s->session&&v.type==0&&n==9&&p[8]==1;
}
static void forward(uint16_t *old,uint16_t value)
{
    uint16_t delta=value-*old;if(delta&&delta<0x8000)*old=value;
}
bool oft_wifi_observe(oft_wifi_session_t *s,const uint8_t *p,size_t n)
{
    oft_wifi_packet_t v;
    if(!s||!oft_wifi_parse(p,n,&v)||v.session!=s->session)return false;
    if(v.type==1){
        if(n<34||n!=34u+get16(p+32)||get16(p+24)!=get16(p+26))return false;
        uint16_t peer=get16(p+26),outstanding=s->last_sent-peer;
        if(outstanding>=0x8000||(outstanding&7))return false;
        /* Stale cumulative statuses cannot roll the writer window backwards. */
        forward(&s->peer,peer);
        if(!s->have_media)s->media_received=get16(p+10);
        if(!s->have_reply)s->reply_received=get16(p+18);
        s->have_status=true;return true;
    }
    if(v.type==2||v.type==3){
        if(n<20)return false;
        uint16_t value=v.sequence&0xfff8;
        uint16_t *received=v.type==2?&s->media_received:&s->reply_received;
        bool *have=v.type==2?&s->have_media:&s->have_reply;
        if(!*have)*received=value;else forward(received,value);
        *have=true;return true;
    }
    return false;
}
size_t oft_wifi_ack(const oft_wifi_session_t *s,uint8_t *out,size_t cap)
{
    if(!s||!out||cap<34||!s->have_status)return 0;
    uint16_t outstanding=s->last_sent-s->peer;
    if(outstanding>=0x8000||(outstanding&7))return 0;
    memset(out,0,34);header(out,34,s->session,0,4);
    put16(out+8,s->media_received);put16(out+10,s->media_received);
    put16(out+16,s->reply_received);put16(out+18,s->reply_received);
    put16(out+24,s->peer);put16(out+26,s->last_sent);return 34;
}
size_t oft_wifi_wrap(oft_wifi_session_t *s,const uint8_t *duml,size_t n,uint8_t *out,size_t cap)
{
    oft_duml_frame_t frame;
    if(!s||!out||!s->session||cap<20+n||!oft_duml_decode(duml,n,&frame))return 0;
    memset(out,0,20);header(out,n+20,s->session,s->next_sequence,5);
    put16(out+8,s->peer);put16(out+10,s->next_sequence);
    put16(out+16,0x100|s->message_counter);memcpy(out+20,duml,n);
    s->last_sent=s->next_sequence;s->next_sequence+=8;s->message_counter++;return n+20;
}
/* The local C DUML codec writes its numeric sequence little endian already.
   Python swaps only because its DUML codec writes numeric sequences big endian.
   Swapping again here changes captured wire 01 00,02 00 into 00 01,00 02. */
uint16_t oft_wifi_duml_sequence(uint16_t c){return c;}
void oft_wifi_pending_add(oft_wifi_pending_t *p,uint16_t seq)
{
    unsigned slot=p->next++%4;p->sequence[slot]=seq;p->used[slot]=true;
}
bool oft_wifi_pending_take(oft_wifi_pending_t *p,uint16_t seq)
{
    for(unsigned i=0;i<4;i++)if(p->used[i]&&p->sequence[i]==seq){p->used[i]=false;return true;}
    return false;
}
