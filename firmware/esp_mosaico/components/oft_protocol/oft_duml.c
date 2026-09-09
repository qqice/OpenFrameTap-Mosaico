/* Direct C port of src/openframetap/protocol/crc.py and DUML wire fields.
 * Codec only: this API grants no permission to transmit commands. */
#include "oft_duml.h"
#include <string.h>

static uint16_t reflect(uint16_t value,unsigned width)
{
    uint16_t result=0;
    for(unsigned i=0;i<width;i++){result=(result<<1)|(value&1);value>>=1;}
    return result;
}
uint8_t oft_crc8(const uint8_t *data,size_t size)
{
    uint8_t v=0xee;
    for(size_t i=0;i<size;i++){
        v^=(uint8_t)reflect(data[i],8);
        for(unsigned b=0;b<8;b++)v=(uint8_t)((v<<1)^((v&0x80)?0x31:0));
    }
    return (uint8_t)reflect(v,8);
}
uint16_t oft_crc16(const uint8_t *data,size_t size)
{
    uint16_t v=0x496c;
    for(size_t i=0;i<size;i++){
        v^=reflect(data[i],8)<<8;
        for(unsigned b=0;b<8;b++)v=(uint16_t)((v<<1)^((v&0x8000)?0x1021:0));
    }
    return reflect(v,16);
}
static uint16_t u16(const uint8_t *p){return (uint16_t)p[0]|((uint16_t)p[1]<<8);}
static void put16(uint8_t *p,uint16_t value){p[0]=value;p[1]=value>>8;}
bool oft_duml_decode(const uint8_t *raw,size_t size,oft_duml_frame_t *frame)
{
    if(!raw||!frame||size<13||size>OFT_DUML_MAX||raw[0]!=0x55)return false;
    if((u16(raw+1)&0x3ff)!=size||oft_crc8(raw,3)!=raw[3]||oft_crc16(raw,size-2)!=u16(raw+size-2))return false;
    *frame=(oft_duml_frame_t){.version=(uint8_t)(u16(raw+1)>>10),.sender=raw[4],.receiver=raw[5],
        .sequence=u16(raw+6),.flags=raw[8],.cmd_set=raw[9],.cmd_id=raw[10],
        .raw=raw,.raw_size=size,.payload=raw+11,.payload_size=size-13};
    return true;
}
size_t oft_duml_encode(uint8_t *out,size_t capacity,uint8_t sender,uint8_t receiver,
    uint16_t sequence,uint8_t flags,uint8_t cmd_set,uint8_t cmd_id,
    const uint8_t *payload,size_t payload_size)
{
    if(!out||payload_size>OFT_DUML_MAX-13||capacity<payload_size+13||(payload_size&&!payload))return 0;
    size_t n=payload_size+13;
    out[0]=0x55;put16(out+1,(uint16_t)n|0x400);out[3]=oft_crc8(out,3);
    out[4]=sender;out[5]=receiver;put16(out+6,sequence);out[8]=flags;out[9]=cmd_set;out[10]=cmd_id;
    if(payload_size)memcpy(out+11,payload,payload_size);
    put16(out+n-2,oft_crc16(out,n-2));return n;
}
static void consume(oft_duml_stream_t *s,size_t n)
{
    memmove(s->bytes,s->bytes+n,s->size-n);s->size-=n;
}
void oft_duml_feed(oft_duml_stream_t *s,const uint8_t *data,size_t size,
    oft_frame_callback_t callback,void *context)
{
    if(!s||(!data&&size))return;
    for(size_t i=0;i<size;i++){
        if(s->size==sizeof(s->bytes)){consume(s,1);s->discarded++;}
        s->bytes[s->size++]=data[i];
        while(s->size){
            if(s->bytes[0]!=0x55){consume(s,1);s->discarded++;continue;}
            if(s->size<4)break;
            if(oft_crc8(s->bytes,3)!=s->bytes[3]){consume(s,1);s->crc8_errors++;continue;}
            size_t n=u16(s->bytes+1)&0x3ff;
            if(n<13||n>OFT_DUML_MAX){consume(s,1);s->length_errors++;continue;}
            if(s->size<n)break;
            oft_duml_frame_t frame;
            if(!oft_duml_decode(s->bytes,n,&frame)){consume(s,1);s->crc16_errors++;continue;}
            s->frames++;
            if(callback)callback(&frame,context);
            consume(s,n);
        }
    }
}
