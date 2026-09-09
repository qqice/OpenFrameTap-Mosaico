#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define OFT_DUML_MAX 1023
typedef struct {
    uint8_t version,sender,receiver,flags,cmd_set,cmd_id;
    uint16_t sequence;
    const uint8_t *raw,*payload; /* views valid only until the callback returns */
    size_t raw_size,payload_size;
} oft_duml_frame_t;
typedef struct {
    uint8_t bytes[OFT_DUML_MAX];
    size_t size;
    uint32_t frames,crc8_errors,crc16_errors,length_errors,discarded;
} oft_duml_stream_t;
typedef void (*oft_frame_callback_t)(const oft_duml_frame_t *,void *);

uint8_t oft_crc8(const uint8_t *data,size_t size);
uint16_t oft_crc16(const uint8_t *data,size_t size);
bool oft_duml_decode(const uint8_t *raw,size_t size,oft_duml_frame_t *frame);
size_t oft_duml_encode(uint8_t *out,size_t capacity,uint8_t sender,uint8_t receiver,
    uint16_t sequence,uint8_t flags,uint8_t cmd_set,uint8_t cmd_id,
    const uint8_t *payload,size_t payload_size);
void oft_duml_feed(oft_duml_stream_t *stream,const uint8_t *data,size_t size,
    oft_frame_callback_t callback,void *context);
