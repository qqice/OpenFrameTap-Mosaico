#pragma once
#include <stdbool.h>
#include <stdint.h>

typedef struct {float yaw,pitch;int64_t timestamp_us;uint32_t gesture;bool active;} oft_input_t;
typedef enum {OFT_CONTROL_DISABLED,OFT_CONTROL_ARMED,OFT_CONTROL_ACTIVE,OFT_CONTROL_FAULT} oft_control_state_t;
typedef bool (*oft_control_send_t)(int yaw_offset,int pitch_offset,void *context);
typedef struct {
    oft_control_state_t state;
    uint32_t last_gesture,blocked_gesture;
    int yaw_offset,pitch_offset;
    int64_t last_send_us,last_zero_us;
    unsigned sends,zeros,failures;
    bool have_sent,have_blocked_gesture,stop_required;
} oft_control_t;
/* Session owner only; no GUI or network dependencies, no queue backlog.
   gate=true means every external connection/profile guard has already passed.
   A fault needs explicit reset AND a new touch generation before movement. */
void oft_control_init(oft_control_t *control);
bool oft_control_map(float yaw,float pitch,int *yaw_offset,int *pitch_offset);
void oft_control_tick(oft_control_t *control,const oft_input_t *input,bool gate,int64_t now_us,
    oft_control_send_t send,void *context);
void oft_control_stop(oft_control_t *control,bool fault,int64_t now_us,oft_control_send_t send,void *context);
void oft_control_reset(oft_control_t *control);
