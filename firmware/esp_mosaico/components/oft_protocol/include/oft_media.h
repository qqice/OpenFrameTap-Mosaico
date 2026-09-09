#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#define OFT_MEDIA_FRAGMENT_MAX 1452
#define OFT_MEDIA_GROUP_BYTES (64*OFT_MEDIA_FRAGMENT_MAX)
typedef struct {
    uint8_t *group,*au;size_t capacity,used,declared;
    uint16_t lengths[64];uint64_t received;
    uint8_t group_id,count,next_id;bool group_active,continuation;
    int64_t group_us,au_us;uint32_t timestamp;
    int64_t finished_at[256];
    unsigned complete,dropped,duplicates,conflicts,invalid;
    bool protect_idr;unsigned foreign_packets;
    int64_t idr_started_us,lost_idr_source_us;unsigned lost_idrs;
} oft_media_t;
void oft_media_init(oft_media_t *s,uint8_t *group,uint8_t *au,size_t capacity);
/* Returns complete AU bytes in s->au, valid until next feed. No allocations.
   Single current group; loss/inter-group reordering drops an AU, never conceals. */
size_t oft_media_feed(oft_media_t *s,const uint8_t *packet,size_t size,int64_t now_us);
uint32_t oft_media_nal_mask(const uint8_t *data,size_t size);
/* First VCL header only; diagnostic, never treats non-IDR I as random access. */
bool oft_media_slice(const uint8_t *data,size_t size,unsigned *reference,unsigned *type);
