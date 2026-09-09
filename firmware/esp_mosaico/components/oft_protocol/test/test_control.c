#include "oft_control.h"
#include "oft_pocket3.h"
#include "unity.h"
#include <math.h>
typedef struct {unsigned calls;int y,p;bool fail;} writer_t;
static bool mock_send(int y,int p,void *ctx)
{
    writer_t *w=ctx;w->calls++;
    uint8_t raw[32];size_t n=oft_p3_build(OFT_P3_STICK,(uint16_t)w->calls,y,p,raw,sizeof(raw));
    TEST_ASSERT_TRUE(oft_p3_allowed(raw,n,false,true));
    if(w->fail)return false;
    w->y=y;w->p=p;return true;
}
static void test_radial_deadzone_and_limits(void)
{
    int y,p;TEST_ASSERT_TRUE(oft_control_map(.1f,0,&y,&p));TEST_ASSERT_EQUAL_INT(0,y);
    TEST_ASSERT_TRUE(oft_control_map(.121f,0,&y,&p));TEST_ASSERT_EQUAL_INT(32,y);
    TEST_ASSERT_TRUE(oft_control_map(1,0,&y,&p));TEST_ASSERT_EQUAL_INT(188,y);
    TEST_ASSERT_TRUE(oft_control_map(1,1,&y,&p));TEST_ASSERT_EQUAL_INT(133,y);TEST_ASSERT_EQUAL_INT(133,p);
    TEST_ASSERT_TRUE(oft_control_map(0,-1,&y,&p));TEST_ASSERT_EQUAL_INT(-188,p);
    TEST_ASSERT_FALSE(oft_control_map(NAN,0,&y,&p));TEST_ASSERT_FALSE(oft_control_map(2,0,&y,&p));
}
static void test_release_cancel_and_rate(void)
{
    oft_control_t c;oft_control_init(&c);writer_t w={0};
    oft_input_t in={.yaw=1,.active=true,.gesture=1,.timestamp_us=1000000};
    oft_control_tick(&c,&in,true,1000000,mock_send,&w);TEST_ASSERT_EQUAL_UINT(1,w.calls);
    in.timestamp_us+=20000;oft_control_tick(&c,&in,true,in.timestamp_us,mock_send,&w);TEST_ASSERT_EQUAL_UINT(1,w.calls);
    in.active=false;oft_control_tick(&c,&in,true,in.timestamp_us,mock_send,&w);
    TEST_ASSERT_EQUAL_INT(0,w.y);TEST_ASSERT_EQUAL_UINT(2,w.calls);TEST_ASSERT_EQUAL_INT(OFT_CONTROL_ARMED,c.state);
    in.active=true;in.gesture++;in.timestamp_us+=100000;
    oft_control_tick(&c,&in,true,in.timestamp_us,mock_send,&w);
    oft_control_stop(&c,false,in.timestamp_us+1,mock_send,&w);
    TEST_ASSERT_EQUAL_INT(0,w.y);TEST_ASSERT_EQUAL_INT(OFT_CONTROL_DISABLED,c.state);
    oft_control_tick(&c,&in,true,in.timestamp_us+2,mock_send,&w);TEST_ASSERT_EQUAL_INT(0,w.y); /* held old input cannot resume */
}
static void test_watchdog_fault_and_explicit_reset(void)
{
    oft_control_t c;oft_control_init(&c);writer_t w={0};
    oft_input_t in={.pitch=1,.active=true,.gesture=1,.timestamp_us=1000000};
    oft_control_tick(&c,&in,true,1000000,mock_send,&w);
    oft_control_tick(&c,&in,true,1250000,mock_send,&w);
    TEST_ASSERT_EQUAL_INT(0,w.p);TEST_ASSERT_EQUAL_INT(OFT_CONTROL_FAULT,c.state);
    in.timestamp_us=1500000;oft_control_tick(&c,&in,true,1500000,mock_send,&w);TEST_ASSERT_EQUAL_INT(0,w.p);
    oft_control_reset(&c);oft_control_tick(&c,&in,true,1500000,mock_send,&w);TEST_ASSERT_EQUAL_INT(0,w.p);
    in.gesture++;oft_control_tick(&c,&in,true,1500000,mock_send,&w);TEST_ASSERT_NOT_EQUAL(0,w.p);
    oft_control_tick(&c,&in,false,1500001,mock_send,&w);TEST_ASSERT_EQUAL_INT(0,w.p);TEST_ASSERT_EQUAL_INT(OFT_CONTROL_FAULT,c.state);
}
static void test_failed_zero_is_not_reported_as_stopped(void)
{
    oft_control_t c;oft_control_init(&c);writer_t w={0};
    oft_input_t in={.yaw=1,.active=true,.gesture=1,.timestamp_us=1000000};
    oft_control_tick(&c,&in,true,1000000,mock_send,&w);w.fail=true;
    oft_control_stop(&c,true,1000001,mock_send,&w);
    TEST_ASSERT_EQUAL_UINT(3,w.calls);TEST_ASSERT_EQUAL_UINT(2,c.failures);
    TEST_ASSERT_EQUAL_UINT(0,c.zeros);TEST_ASSERT_EQUAL_INT(188,c.yaw_offset);
    TEST_ASSERT_EQUAL_INT(OFT_CONTROL_FAULT,c.state);
    oft_control_reset(&c);TEST_ASSERT_EQUAL_INT(OFT_CONTROL_FAULT,c.state);
    oft_control_init(&c);w.calls=0;
    oft_control_tick(&c,&in,true,1000000,mock_send,&w);
    TEST_ASSERT_EQUAL_UINT(3,w.calls); /* failed initial send still attempts zero */
    TEST_ASSERT_TRUE(c.stop_required);
}
void oft_control_selftests(void)
{
    RUN_TEST(test_radial_deadzone_and_limits);
    RUN_TEST(test_release_cancel_and_rate);
    RUN_TEST(test_watchdog_fault_and_explicit_reset);
    RUN_TEST(test_failed_zero_is_not_reported_as_stopped);
}
