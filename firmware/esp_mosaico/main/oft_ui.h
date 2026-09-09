#pragma once
#include <stdbool.h>
/* Called by the physical LVGL indev reader; never from the sensor ISR. */
bool oft_ui_pointer(int x,int y,bool down,bool valid);
bool oft_ui_menu_test(void);
void oft_ui_create(void);
bool oft_ui_hold_test(void); /* Shadow LVGL pointer only; never routes to camera control. */
