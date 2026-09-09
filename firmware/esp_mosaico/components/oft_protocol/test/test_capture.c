#include "oft_capture.h"
#include "oft_pocket3.h"
#include "unity.h"
static void test_camera_mode_and_stale_state(void)
{
    uint8_t p[58]={0};p[57]=1;
    oft_duml_frame_t f={.sender=1,.receiver=2,.cmd_set=2,.cmd_id=0x80,.payload=p,.payload_size=58};oft_camera_state_t s;
    TEST_ASSERT_TRUE(oft_capture_status(&f,100,&s));TEST_ASSERT_EQUAL(OFT_SHUTTER_START,oft_capture_select(s,100));
    p[0]=0x80;oft_capture_status(&f,100,&s);TEST_ASSERT_EQUAL(OFT_SHUTTER_STOP,oft_capture_select(s,100));
    p[0]=0;p[57]=5;oft_capture_status(&f,100,&s);TEST_ASSERT_EQUAL(OFT_SHUTTER_PHOTO,oft_capture_select(s,100));
    TEST_ASSERT_EQUAL(OFT_SHUTTER_NONE,oft_capture_select(s,2000101));TEST_ASSERT_EQUAL(OFT_SHUTTER_NONE,oft_capture_select(s,99));
    p[57]=0x17;oft_capture_status(&f,100,&s);TEST_ASSERT_EQUAL(OFT_SHUTTER_NONE,oft_capture_select(s,100));
    f.payload_size=57;TEST_ASSERT_FALSE(oft_capture_status(&f,100,&s));
}
static void test_physical_button_debounce_and_boot_hold(void)
{
    oft_button_filter_t b={.blocked=true};
    TEST_ASSERT_FALSE(oft_button_step(&b,true,0));TEST_ASSERT_FALSE(oft_button_step(&b,true,100000));
    TEST_ASSERT_FALSE(oft_button_step(&b,false,110000));TEST_ASSERT_FALSE(oft_button_step(&b,false,160000));
    TEST_ASSERT_FALSE(oft_button_step(&b,true,200000));TEST_ASSERT_FALSE(oft_button_step(&b,true,239999));
    TEST_ASSERT_TRUE(oft_button_step(&b,true,240000));TEST_ASSERT_FALSE(oft_button_step(&b,true,2000000));
    TEST_ASSERT_FALSE(oft_button_step(&b,false,2100000));TEST_ASSERT_FALSE(oft_button_step(&b,true,2110000));
    TEST_ASSERT_FALSE(oft_button_step(&b,true,2200000)); // Release bounce cannot rearm.
}
static void test_shutter_reply_timeout_and_policy(void)
{
    oft_shutter_tx_t t;oft_shutter_begin(&t,OFT_SHUTTER_START,12,100);
    uint8_t p=0;oft_duml_frame_t f={.sender=1,.receiver=2,.cmd_set=2,.cmd_id=2,.flags=0x80,.sequence=11,.payload=&p,.payload_size=1};
    TEST_ASSERT_FALSE(oft_shutter_reply(&t,&f));f.sequence=12;TEST_ASSERT_TRUE(oft_shutter_reply(&t,&f));TEST_ASSERT_TRUE(t.pending);
    oft_camera_state_t s={.mode=OFT_CAPTURE_VIDEO,.valid=true,.recording=true,.updated_us=200};
    oft_shutter_observe(&t,s,200);TEST_ASSERT_FALSE(t.pending);TEST_ASSERT_EQUAL(1,t.result);
    TEST_ASSERT_FALSE(oft_shutter_reply(&t,&f));
    oft_shutter_begin(&t,OFT_SHUTTER_STOP,13,100);oft_shutter_observe(&t,s,4000100);TEST_ASSERT_EQUAL(-1,t.result);
    oft_shutter_begin(&t,OFT_SHUTTER_PHOTO,14,100);f.sequence=14;f.cmd_id=1;p=0xd9;
    TEST_ASSERT_TRUE(oft_shutter_reply(&t,&f));TEST_ASSERT_EQUAL(-2,t.result);
    uint8_t raw[32];size_t n=oft_p3_build(OFT_P3_PHOTO,14,0,0,raw,sizeof(raw));
    TEST_ASSERT_EQUAL_UINT(14,n);TEST_ASSERT_TRUE(oft_p3_allowed(raw,n,false,true));
    TEST_ASSERT_FALSE(oft_p3_allowed(raw,n,true,true));TEST_ASSERT_FALSE(oft_p3_allowed(raw,n,false,false));
}
void oft_capture_selftests(void)
{RUN_TEST(test_camera_mode_and_stale_state);RUN_TEST(test_physical_button_debounce_and_boot_hold);RUN_TEST(test_shutter_reply_timeout_and_policy);}
