#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
/* vTaskDelay(1) may return almost immediately at a tick boundary. The BMI270
 * driver's millisecond reset/power waits require at least the requested time.
 * Only that driver's translation unit enables the override below. No SDK edit,
 * global tick-rate change, busy-wait or retry/reset loop is involved. */
static inline void oft_bmi_delay(TickType_t ticks)
{
    configASSERT(ticks < portMAX_DELAY);
    vTaskDelay(ticks + 1);
}
#ifdef OFT_BMI_DELAY_OVERRIDE
#define vTaskDelay oft_bmi_delay
#endif
