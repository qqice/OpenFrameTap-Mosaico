#pragma once
#include "driver/i2c_master.h"
#include <stdbool.h>
typedef enum {OFT_GAUGE_AUDIT=1,OFT_GAUGE_APPLY,OFT_GAUGE_RESTORE,OFT_GAUGE_ACCESS_RESTORE,OFT_GAUGE_HISTORY,OFT_GAUGE_MODEL} oft_gauge_action_t;
bool oft_gauge_request(oft_gauge_action_t action);
void oft_gauge_poll(i2c_master_dev_handle_t device);
bool oft_gauge_ready(void);
void oft_gauge_observe(bool valid,uint16_t soc,uint16_t mv,int16_t raw_ma,uint16_t design,uint16_t status,uint16_t flags,uint16_t remaining,uint16_t fcc);
void oft_gauge_selftests(void);
