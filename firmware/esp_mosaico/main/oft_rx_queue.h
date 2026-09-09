#pragma once
#include <stdint.h>
typedef struct {unsigned posts,drops,high_water,capacity;} oft_rx_queue_stats_t;
oft_rx_queue_stats_t oft_rx_queue_stats(void);
