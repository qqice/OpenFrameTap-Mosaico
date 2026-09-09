#include "unity.h"
#include "oft_duml.h"
#include <string.h>

/* Existing project fixture, no private device address or credential. */
static const uint8_t golden[]={0x55,0x0e,0x04,0x66,0x04,0x02,0x6b,0x13,0x00,0x04,0x1c,0x48,0xe5,0xe2};
static void test_crc_and_encoding(void)
{
    TEST_ASSERT_EQUAL_HEX8(0x66,oft_crc8(golden,3));
    TEST_ASSERT_EQUAL_HEX16(0xe2e5,oft_crc16(golden,sizeof(golden)-2));
    uint8_t raw[32],payload=0x48;
    size_t n=oft_duml_encode(raw,sizeof(raw),4,2,0x136b,0,4,0x1c,&payload,1);
    TEST_ASSERT_EQUAL_UINT(sizeof(golden),n);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(golden,raw,n);
    oft_duml_frame_t frame;
    TEST_ASSERT_TRUE(oft_duml_decode(raw,n,&frame));
    TEST_ASSERT_EQUAL_HEX16(0x136b,frame.sequence);
    TEST_ASSERT_EQUAL_UINT(1,frame.payload_size);
}
static void test_corruption_and_unknown_fields(void)
{
    uint8_t raw[32];memcpy(raw,golden,sizeof(golden));
    oft_duml_frame_t f;
    raw[5]^=1;TEST_ASSERT_FALSE(oft_duml_decode(raw,sizeof(golden),&f));
    TEST_ASSERT_FALSE(oft_duml_decode(golden,sizeof(golden)-1,&f));
    TEST_ASSERT_EQUAL_UINT(0,oft_duml_encode(raw,4,2,4,0,0,0,0,NULL,0));
    size_t n=oft_duml_encode(raw,sizeof(raw),2,4,65535,0,0xab,0xcd,(uint8_t[]){0xde,0xad},2);
    TEST_ASSERT_TRUE(oft_duml_decode(raw,n,&f));
    TEST_ASSERT_EQUAL_HEX8(0xab,f.cmd_set);TEST_ASSERT_EQUAL_HEX8(0xcd,f.cmd_id);
    const uint8_t unknown_payload[]={0xde,0xad};
    TEST_ASSERT_EQUAL_HEX8_ARRAY(unknown_payload,f.payload,2);
    uint16_t seq=65535;seq++;
    n=oft_duml_encode(raw,sizeof(raw),2,4,seq,0,4,1,NULL,0);
    TEST_ASSERT_TRUE(oft_duml_decode(raw,n,&f));TEST_ASSERT_EQUAL_UINT(0,f.sequence);
}
static void counted(const oft_duml_frame_t *f,void *ctx)
{
    TEST_ASSERT_EQUAL_HEX8_ARRAY(golden,f->raw,sizeof(golden));
    (*(unsigned *)ctx)++;
}
static void test_maximum_frame(void)
{
    static uint8_t raw[OFT_DUML_MAX+1],payload[OFT_DUML_MAX-13];
    memset(payload,0xa5,sizeof(payload));
    size_t n=oft_duml_encode(raw,sizeof(raw),2,4,1,0,0xfe,0xfd,payload,sizeof(payload));
    TEST_ASSERT_EQUAL_UINT(OFT_DUML_MAX,n);
    oft_duml_frame_t f;
    TEST_ASSERT_TRUE(oft_duml_decode(raw,n,&f));
    TEST_ASSERT_EQUAL_UINT(sizeof(payload),f.payload_size);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(payload,f.payload,sizeof(payload));
    TEST_ASSERT_FALSE(oft_duml_decode(raw,n+1,&f));
    TEST_ASSERT_EQUAL_UINT(0,oft_duml_encode(raw,sizeof(raw),2,4,1,0,0,0,payload,sizeof(payload)+1));
    raw[3]^=1;TEST_ASSERT_FALSE(oft_duml_decode(raw,n,&f));
}
static void test_stream_resynchronization(void)
{
    oft_duml_stream_t stream={0};unsigned count=0;
    const uint8_t garbage[]={0,0x13,0x55,0,0,0};
    oft_duml_feed(&stream,garbage,sizeof(garbage),counted,&count);
    oft_duml_feed(&stream,golden,7,counted,&count);TEST_ASSERT_EQUAL_UINT(0,count);
    oft_duml_feed(&stream,golden+7,sizeof(golden)-7,counted,&count);TEST_ASSERT_EQUAL_UINT(1,count);
    uint8_t joined[sizeof(golden)*2];memcpy(joined,golden,sizeof(golden));memcpy(joined+sizeof(golden),golden,sizeof(golden));
    oft_duml_feed(&stream,joined,sizeof(joined),counted,&count);TEST_ASSERT_EQUAL_UINT(3,count);
    joined[sizeof(golden)-1]^=1;
    oft_duml_feed(&stream,joined,sizeof(joined),counted,&count);TEST_ASSERT_EQUAL_UINT(4,count);
    TEST_ASSERT_GREATER_THAN_UINT(0,stream.crc16_errors);
    TEST_ASSERT_EQUAL_UINT(0,stream.size);
}

/* Explicit entry point keeps the tests linked into the application. Relying
   only on constructor registration can let an entire static-library object
   be discarded, producing a misleading zero-test run. */
void oft_protocol_run_selftests(void)
{
    void oft_wifi_profile_selftests(void);
    void oft_control_selftests(void);
    void oft_orientation_selftests(void);
    void oft_head_target_selftests(void);
    void oft_media_selftests(void);
    void oft_video_selftests(void);
    void oft_motion_selftests(void);
    void oft_touch_selftests(void);
    void oft_capture_selftests(void);
    void oft_ui_geometry_selftests(void);
    void oft_gauge_selftests(void);
    void oft_settings_selftests(void);
    UNITY_BEGIN();
    RUN_TEST(test_crc_and_encoding);
    RUN_TEST(test_corruption_and_unknown_fields);
    RUN_TEST(test_maximum_frame);
    RUN_TEST(test_stream_resynchronization);
    oft_wifi_profile_selftests();
    oft_control_selftests();
    oft_orientation_selftests();
    oft_head_target_selftests();
    oft_media_selftests();
    oft_video_selftests();
    oft_motion_selftests();
    oft_touch_selftests();
    oft_capture_selftests();
    oft_ui_geometry_selftests();
    oft_gauge_selftests();
    oft_settings_selftests();
    UNITY_END();
}
