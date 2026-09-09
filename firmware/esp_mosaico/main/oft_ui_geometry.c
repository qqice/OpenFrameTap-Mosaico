#include "oft_ui_geometry.h"
#include "unity.h"
#include <stdlib.h>
void oft_menu_center(oft_menu_side_t side,int *x,int *y)
{*x=side==OFT_MENU_LEFT?0:side==OFT_MENU_RIGHT?480:240;*y=side==OFT_MENU_TOP?0:side==OFT_MENU_BOTTOM?480:240;}
bool oft_menu_inside(oft_menu_side_t side,int x,int y)
{int cx,cy;if(side==OFT_MENU_NONE)return false;oft_menu_center(side,&cx,&cy);return (x-cx)*(x-cx)+(y-cy)*(y-cy)<=240*240;}
static oft_menu_side_t edge_at(int x,int y)
{
    int distances[]={y,480-y,x,480-x},best=25;oft_menu_side_t side=OFT_MENU_NONE;
    for(unsigned i=0;i<4;i++)if(distances[i]>=0&&distances[i]<best){best=distances[i];side=(oft_menu_side_t)(i+1);}
    return side;
}
bool oft_swipe_step(oft_swipe_t *s,int x,int y,bool down,bool valid,oft_menu_side_t *open)
{
    *open=OFT_MENU_NONE;
    if(!valid){bool consumed=s->consumed;*s=(oft_swipe_t){0};return consumed;}
    if(down&&!s->down){s->x=x;s->y=y;s->edge=edge_at(x,y);s->consumed=s->edge!=OFT_MENU_NONE;}
    bool consumed=s->consumed;
    if(!down&&s->down&&s->consumed){
        int dx=x-s->x,dy=y-s->y;
        int inward=s->edge==OFT_MENU_TOP?dy:s->edge==OFT_MENU_BOTTOM?-dy:s->edge==OFT_MENU_LEFT?dx:-dx;
        int lateral=(s->edge==OFT_MENU_TOP||s->edge==OFT_MENU_BOTTOM)?abs(dx):abs(dy);
        if(inward>=48&&inward>=lateral)*open=s->edge;
        *s=(oft_swipe_t){0};
    }
    s->down=down;return consumed;
}
unsigned oft_battery_band(bool valid,unsigned percent)
{return !valid||percent>100?0:percent<25?1:percent<75?2:3;}
static void test_four_edge_swipes(void)
{
    const int xy[][4]={{240,10,240,100},{240,470,240,380},{10,240,100,240},{470,240,380,240}};
    for(unsigned i=0;i<4;i++){
        oft_swipe_t s={0};oft_menu_side_t open;
        TEST_ASSERT_TRUE(oft_swipe_step(&s,xy[i][0],xy[i][1],true,true,&open));TEST_ASSERT_EQUAL(OFT_MENU_NONE,open);
        TEST_ASSERT_TRUE(oft_swipe_step(&s,xy[i][2],xy[i][3],false,true,&open));TEST_ASSERT_EQUAL_UINT(i+1,open);
    }
}
static void test_swipe_cancel_and_geometry(void)
{
    oft_swipe_t s={0};oft_menu_side_t open;
    TEST_ASSERT_FALSE(oft_swipe_step(&s,240,240,true,true,&open));oft_swipe_step(&s,240,350,false,true,&open);TEST_ASSERT_EQUAL(OFT_MENU_NONE,open);
    oft_swipe_step(&s,240,10,true,true,&open);oft_swipe_step(&s,240,100,false,false,&open);TEST_ASSERT_EQUAL(OFT_MENU_NONE,open);
    oft_swipe_step(&s,240,10,true,true,&open);oft_swipe_step(&s,240,30,false,true,&open);TEST_ASSERT_EQUAL(OFT_MENU_NONE,open);
    TEST_ASSERT_TRUE(oft_menu_inside(OFT_MENU_TOP,240,240));TEST_ASSERT_FALSE(oft_menu_inside(OFT_MENU_TOP,240,241));
    TEST_ASSERT_TRUE(oft_menu_inside(OFT_MENU_LEFT,240,240));TEST_ASSERT_FALSE(oft_menu_inside(OFT_MENU_LEFT,241,240));
    TEST_ASSERT_FALSE(oft_menu_inside(OFT_MENU_NONE,240,240));
}
static void test_battery_colors_and_unknown(void)
{TEST_ASSERT_EQUAL(0,oft_battery_band(false,4));TEST_ASSERT_EQUAL(0,oft_battery_band(true,101));TEST_ASSERT_EQUAL(1,oft_battery_band(true,24));TEST_ASSERT_EQUAL(2,oft_battery_band(true,25));TEST_ASSERT_EQUAL(3,oft_battery_band(true,100));}
void oft_ui_geometry_selftests(void)
{RUN_TEST(test_four_edge_swipes);RUN_TEST(test_swipe_cancel_and_geometry);RUN_TEST(test_battery_colors_and_unknown);}
