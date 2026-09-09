#pragma once
#include <stdbool.h>
typedef enum {OFT_MENU_NONE,OFT_MENU_TOP,OFT_MENU_BOTTOM,OFT_MENU_LEFT,OFT_MENU_RIGHT} oft_menu_side_t;
typedef struct {bool down,consumed;int x,y;oft_menu_side_t edge;} oft_swipe_t;
bool oft_menu_inside(oft_menu_side_t side,int x,int y);
void oft_menu_center(oft_menu_side_t side,int *x,int *y);
bool oft_swipe_step(oft_swipe_t *s,int x,int y,bool down,bool valid,oft_menu_side_t *open);
unsigned oft_battery_band(bool valid,unsigned percent);
void oft_ui_geometry_selftests(void);
