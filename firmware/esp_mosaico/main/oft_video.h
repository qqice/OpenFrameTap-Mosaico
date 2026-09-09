#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#ifdef __cplusplus
extern "C" {
#endif
typedef struct {
    unsigned packets,queue_drops,units,assembly_drops,invalid,attempts;
    unsigned width,height,decode_ms;int decoder_result;
    bool available,busy,ready;
    uint32_t nal_mask;size_t au_bytes;
    unsigned display_width,display_height;
    char detail[64];
    char codec_error[192];unsigned decode_input_bytes,copy_errors;
    unsigned decoded_frames,presented_frames,decode_errors,au_queue_drops;
    unsigned queued_units,queued_bytes,convert_ms,queue_age_ms,fps_milli;
    uint32_t generation;int64_t last_frame_us;
    uint32_t last_idr_generation,last_idr_hash;int64_t last_idr_source_us,last_idr_frame_us;
    unsigned burst_frames,decoded_predicted_frames;
    unsigned output_age_ms,deadline_drops;
    bool low_latency;unsigned latency_budget_ms;
    unsigned convert_reference_us,convert_mapped_us;
    unsigned last_idr_decode_ms,p_decode_ms,selected_p_frames;
    unsigned direct_outputs,drained_outputs,timestamp_mismatches;
    unsigned slice_type,nal_reference,stack_free;
    unsigned intra_units,idr_units,nonreference_units;
    int64_t last_source_us;unsigned source_age_ms;
    bool sparse_live;unsigned refresh_interval_ms,refresh_completed,refresh_failures;
    unsigned refresh_recoveries;
    bool refresh_recovering;unsigned retry_after_ms;
    uint32_t pixel_hash;
    int64_t lost_idr_source_us;unsigned lost_idrs;
    bool benchmark;
} oft_video_snapshot_t;
void oft_video_start(void);
void oft_video_feed(const uint8_t *data,size_t size,int64_t now_us);
void oft_video_snapshot(oft_video_snapshot_t *out);
void oft_video_dump(void);
void oft_video_selftests(void);
bool oft_video_refresh(unsigned interval_ms); /* 0:single; shared policy minimum; bounded12 requests */
void oft_video_poll(void);
void oft_video_stop_refresh(void);
void oft_video_visible(bool visible);
void oft_video_latency_mode(bool fresh); /* Local scheduling only; no new camera command. */
bool oft_video_trace_arm(void);
void oft_video_trace_dump(void);
bool oft_video_benchmark(void);
bool oft_video_loss_test(void); /* Explicit idle-only diagnostic: suppress at most2 decoded units,30s expiry. */
bool oft_video_burst_trial(unsigned maximum); /* 3 or6 consecutive P pictures; adaptive time budget;120s expiry. */
void oft_video_idr_only(void); /* Diagnostic/manual fallback; no motion command. */
void oft_video_sample_dump(void);
/* Copies a coherent published frame into GUI-owned storage. No LVGL calls in
   decoder/receiver tasks. Returns zero if unchanged/unavailable. */
uint32_t oft_video_copy(uint16_t *destination,size_t bytes,uint32_t after);
uint32_t oft_video_copy_timed(uint16_t *destination,size_t bytes,uint32_t after,int64_t *source_us);
void oft_video_presented(void);
#ifdef __cplusplus
}
#endif
