#pragma once
#include <stdbool.h>
#include <stdint.h>
typedef struct {bool active;int64_t started_us,lease_until_us;} oft_haptic_pattern_t;
/* Immediate first50ms pulse; then50ms off, repeating while100ms lease renewed.
   Renewal does not restart pulse phase. Cancellation and lease expiry force off. */
void oft_haptic_request(oft_haptic_pattern_t *state,bool active,int64_t now_us);
bool oft_haptic_output(const oft_haptic_pattern_t *state,int64_t now_us);
