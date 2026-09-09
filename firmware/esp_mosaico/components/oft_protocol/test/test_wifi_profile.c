#include "unity.h"
#include "oft_wifi_wire.h"
#include "oft_pocket3.h"
#include <string.h>

/* Sanitized fixtures already committed in tests/test_{wifi_gimbal_control,
   dji_wifi_envelope}.py. Session numbers are historical wire state, not a key. */
static const uint8_t handshake[]={
  0x30,0x80,0x55,0x70,0,0,0,0x95,0xa8,0x82,0x64,0,0x64,0,0xc0,5,0x14,0,0,0x64,0,0,1,0x90,
  1,0xc0,5,0x14,0,0,0x64,0,0x14,0,0x64,0,0xc0,5,0x14,0,0,0x64,0,1,1,4,1,2};
static const uint8_t center[]={
  0x2b,0x80,0x55,0x70,0xa8,0x90,5,0xb3,0xa0,0x90,0xa8,0x90,0,0,0,0,0xc0,1,0,0,
  0x55,0x17,4,0x38,2,4,0x42,0xb7,0,4,1,0,4,0,0,0,4,0,0x80,0x42,0,5,0xb8};
static const uint8_t status[]={
  0x22,0x80,0x55,0x70,0,0,1,0x86,0xa8,0x82,0xa8,0x82,0,0,0,0,
  0xa8,0x82,0xc8,0x82,0,0,0,0,0xd8,0x82,0xd8,0x82,0,0,0,0,0,0};
static void wire_checksum(uint8_t *p){p[7]=0;for(unsigned i=0;i<7;i++)p[7]^=p[i];}
static void test_wifi_golden(void)
{
    oft_wifi_session_t s;uint8_t raw[128],duml[64];
    TEST_ASSERT_TRUE(oft_wifi_init(&s,0x7055,0x82a8));
    size_t n=oft_wifi_handshake(&s,raw,sizeof(raw));
    TEST_ASSERT_EQUAL_UINT(sizeof(handshake),n);TEST_ASSERT_EQUAL_HEX8_ARRAY(handshake,raw,n);
    TEST_ASSERT_FALSE(oft_wifi_init(&s,0,0));TEST_ASSERT_FALSE(oft_wifi_init(&s,1,3));
    TEST_ASSERT_TRUE(oft_wifi_init(&s,0x7055,0x90a0));s.message_counter=0xc0;
    n=oft_p3_build(OFT_P3_STICK,oft_wifi_duml_sequence(0xb742),0,0,duml,sizeof(duml));
    n=oft_wifi_wrap(&s,duml,n,raw,sizeof(raw));
    TEST_ASSERT_EQUAL_UINT(sizeof(center),n);TEST_ASSERT_EQUAL_HEX8_ARRAY(center,raw,n);
    oft_wifi_packet_t v;TEST_ASSERT_TRUE(oft_wifi_parse(raw,n,&v));
    raw[7]^=1;TEST_ASSERT_FALSE(oft_wifi_parse(raw,n,&v));
    TEST_ASSERT_FALSE(oft_wifi_parse(center,sizeof(center)-1,&v));
}
static void test_wifi_flow_and_resynchronization(void)
{
    oft_wifi_session_t s;uint8_t raw[80],ack[40];
    oft_wifi_init(&s,0x7055,0x8300);s.peer=0x82d0;
    TEST_ASSERT_TRUE(oft_wifi_observe(&s,status,sizeof(status)));
    TEST_ASSERT_EQUAL_HEX16(0x82d8,s.peer);
    const uint8_t expected[]={0x22,0x80,0x55,0x70,0,0,4,0x83,
      0xa8,0x82,0xa8,0x82,0,0,0,0,0xc8,0x82,0xc8,0x82,0,0,0,0,0xd8,0x82,0,0x83,0,0,0,0,0,0};
    size_t n=oft_wifi_ack(&s,ack,sizeof(ack));TEST_ASSERT_EQUAL_UINT(34,n);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected,ack,n);
    memcpy(raw,status,sizeof(status));raw[26]+=8;
    TEST_ASSERT_FALSE(oft_wifi_observe(&s,raw,sizeof(status))); /* ambiguous range */
    memcpy(raw,status,sizeof(status));raw[32]=1;
    TEST_ASSERT_FALSE(oft_wifi_observe(&s,raw,sizeof(status))); /* bad inner length */
    memcpy(raw,center,sizeof(center));raw[6]=3;wire_checksum(raw);
    TEST_ASSERT_TRUE(oft_wifi_observe(&s,raw,sizeof(center)));
    TEST_ASSERT_EQUAL_HEX16(0x82d8,s.peer); /* regression: reply sequence is not peer ACK */
    TEST_ASSERT_EQUAL_HEX16(0x90a8,s.reply_received);
    raw[2]^=1;wire_checksum(raw);TEST_ASSERT_FALSE(oft_wifi_observe(&s,raw,sizeof(center)));
}
static void test_wifi_sequence_wrap_and_rejected_send(void)
{
    oft_wifi_session_t s;uint8_t raw[100],duml[32];oft_wifi_init(&s,1,0xfff0);s.message_counter=255;
    size_t n=oft_p3_build(OFT_P3_PRESENCE,oft_wifi_duml_sequence(0x1234),0,0,duml,sizeof(duml));
    TEST_ASSERT_EQUAL_HEX16(0x1234,oft_wifi_duml_sequence(0x1234));
    TEST_ASSERT_EQUAL_HEX8(0x34,duml[6]);TEST_ASSERT_EQUAL_HEX8(0x12,duml[7]);
    TEST_ASSERT_EQUAL_UINT(0,oft_wifi_wrap(&s,duml,n,raw,2));
    TEST_ASSERT_EQUAL_HEX16(0xfff8,s.next_sequence);
    TEST_ASSERT_NOT_EQUAL(0,oft_wifi_wrap(&s,duml,n,raw,sizeof(raw)));
    TEST_ASSERT_EQUAL_HEX8(0xff,raw[16]);TEST_ASSERT_EQUAL_UINT(0,s.next_sequence);
    TEST_ASSERT_NOT_EQUAL(0,oft_wifi_wrap(&s,duml,n,raw,sizeof(raw)));
    TEST_ASSERT_EQUAL_HEX8(0,raw[16]);TEST_ASSERT_EQUAL_HEX8(1,raw[17]);
}
static void test_profile_allowlist_and_raw_pwm_rejection(void)
{
    uint8_t raw[100];
    for(unsigned c=0;c<OFT_P3_COMMAND_COUNT;c++){
        size_t n=oft_p3_build(c,1,0,0,raw,sizeof(raw));TEST_ASSERT_GREATER_THAN_UINT(0,n);
        TEST_ASSERT_TRUE(oft_p3_allowed(raw,n,c<=OFT_P3_PASSWORD,true));
        if(c!=OFT_P3_OPEN)TEST_ASSERT_FALSE(oft_p3_allowed(raw,n,c>OFT_P3_PASSWORD,true));
        else TEST_ASSERT_TRUE(oft_p3_allowed(raw,n,false,false));
        if(c>=OFT_P3_CONTROL_HEARTBEAT)TEST_ASSERT_FALSE(oft_p3_allowed(raw,n,false,false));
    }
    TEST_ASSERT_EQUAL_UINT(0,oft_p3_build(999,1,0,0,raw,sizeof(raw)));
    TEST_ASSERT_EQUAL_UINT(0,oft_p3_build(OFT_P3_STICK,1,189,0,raw,sizeof(raw)));
    const uint8_t pwm[]={1,2};
    size_t n=oft_duml_encode(raw,sizeof(raw),2,4,1,0,4,1,pwm,sizeof(pwm));
    TEST_ASSERT_FALSE(oft_p3_allowed(raw,n,false,true));
    n=oft_duml_encode(raw,sizeof(raw),2,4,1,0,0xff,0xfe,pwm,sizeof(pwm));
    TEST_ASSERT_FALSE(oft_p3_allowed(raw,n,false,true));
    n=oft_p3_build(OFT_P3_STICK,1,-188,188,raw,sizeof(raw));
    TEST_ASSERT_TRUE(oft_p3_allowed(raw,n,false,true));
}
static void test_credentials_and_telemetry_bounds(void)
{
    char value[65];
    const uint8_t ssid[]={0,15,'O','s','m','o','P','o','c','k','e','t','3','-','T','E','S'};
    TEST_ASSERT_TRUE(oft_p3_wifi_string(ssid,sizeof(ssid),false,value,sizeof(value)));
    TEST_ASSERT_FALSE(oft_p3_wifi_string(ssid,sizeof(ssid)-1,false,value,sizeof(value)));
    TEST_ASSERT_FALSE(oft_p3_wifi_string(ssid,sizeof(ssid),false,value,5));
    const uint8_t short_pass[]={0,2,'x','x'};
    TEST_ASSERT_FALSE(oft_p3_wifi_string(short_pass,sizeof(short_pass),true,value,sizeof(value)));
    uint8_t payload[21]={0},percent=0;payload[20]=73;
    oft_duml_frame_t f={.cmd_set=13,.cmd_id=2,.payload=payload,.payload_size=21};
    TEST_ASSERT_TRUE(oft_p3_battery(&f,&percent));TEST_ASSERT_EQUAL_UINT8(73,percent);
    f.payload_size=20;TEST_ASSERT_FALSE(oft_p3_battery(&f,&percent));
    f.payload_size=21;payload[20]=101;TEST_ASSERT_FALSE(oft_p3_battery(&f,&percent));
}
static void test_captured_ble_reply_matching(void)
{
    /* paired_session_frames.json: Pocket->FFF4 at 2026-07-18T13:30:06.735927Z,
       source SHA256 8b6fd9c2ebd8e0e9e4eeff376689287478d30a4de6b75edf97b429f7add28b49. */
    const uint8_t raw[]={0x55,0x0f,4,0xa2,7,2,0x72,0xaa,0xc0,7,0x45,0,1,0x9b,0x5b};
    oft_duml_frame_t f;
    TEST_ASSERT_TRUE(oft_duml_decode(raw,sizeof(raw),&f));
    TEST_ASSERT_TRUE(oft_p3_ble_reply_matches(OFT_P3_PAIR_STATUS,0xaa72,&f));
    TEST_ASSERT_FALSE(oft_p3_ble_reply_matches(OFT_P3_PAIR_STATUS,0xaa73,&f));
    TEST_ASSERT_FALSE(oft_p3_ble_reply_matches(OFT_P3_SSID,0xaa72,&f));
    f.flags=0x40;TEST_ASSERT_FALSE(oft_p3_ble_reply_matches(OFT_P3_PAIR_STATUS,0xaa72,&f));
    f.flags=0xc1;TEST_ASSERT_FALSE(oft_p3_ble_reply_matches(OFT_P3_PAIR_STATUS,0xaa72,&f));
    f.flags=0xc0;f.sender=8;TEST_ASSERT_FALSE(oft_p3_ble_reply_matches(OFT_P3_PAIR_STATUS,0xaa72,&f));
    f.sender=7;f.receiver=3;TEST_ASSERT_FALSE(oft_p3_ble_reply_matches(OFT_P3_PAIR_STATUS,0xaa72,&f));
    f.receiver=2;f.cmd_id=7;
    TEST_ASSERT_TRUE(oft_p3_ble_reply_matches(OFT_P3_SSID,0xaa72,&f));
    f.cmd_id=0x0e;TEST_ASSERT_TRUE(oft_p3_ble_reply_matches(OFT_P3_PASSWORD,0xaa72,&f));
    f.flags=0x80;TEST_ASSERT_TRUE(oft_p3_ble_reply_matches(OFT_P3_PASSWORD,0xaa72,&f));
}
static void test_handshake_and_registration_boundaries(void)
{
    oft_wifi_session_t s;oft_wifi_init(&s,0x7055,0x82a8);
    uint8_t ack[]={9,0x80,0x55,0x70,0,0,0,0xac,1};
    TEST_ASSERT_TRUE(oft_wifi_handshake_accepted(&s,ack,sizeof(ack)));
    ack[8]=0;TEST_ASSERT_FALSE(oft_wifi_handshake_accepted(&s,ack,sizeof(ack)));
    ack[8]=1;ack[2]^=1;wire_checksum(ack);TEST_ASSERT_FALSE(oft_wifi_handshake_accepted(&s,ack,sizeof(ack)));
    uint8_t raw[100],out[100],payload[64]={0};oft_duml_frame_t f,r;
    size_t n=oft_duml_encode(raw,sizeof(raw),0x48,2,0x1234,0x40,0,0x81,payload,64);
    TEST_ASSERT_TRUE(oft_duml_decode(raw,n,&f));
    n=oft_p3_registration_reply(&f,out,sizeof(out));TEST_ASSERT_TRUE(oft_duml_decode(out,n,&r));
    TEST_ASSERT_EQUAL_HEX16(0x1234,r.sequence);TEST_ASSERT_EQUAL_HEX8(0x80,r.flags);
    TEST_ASSERT_EQUAL_UINT(64,r.payload_size);TEST_ASSERT_EQUAL_MEMORY("APP",r.payload+1,3);
    TEST_ASSERT_EQUAL_UINT8(2,r.payload[34]);TEST_ASSERT_EQUAL_UINT8(8,r.payload[42]);
    f.payload_size=63;TEST_ASSERT_EQUAL_UINT(0,oft_p3_registration_reply(&f,out,sizeof(out)));
    f.payload_size=64;f.cmd_id=0x82;n=oft_p3_registration_reply(&f,out,sizeof(out));
    TEST_ASSERT_TRUE(oft_duml_decode(out,n,&r));TEST_ASSERT_EQUAL_UINT(1,r.payload_size);TEST_ASSERT_EQUAL_UINT8(0,r.payload[0]);
    f.sender=4;TEST_ASSERT_EQUAL_UINT(0,oft_p3_registration_reply(&f,out,sizeof(out)));
}
static void test_media_receipt_does_not_advance_operator(void)
{
    oft_wifi_session_t s;oft_wifi_init(&s,0x7055,0x8300);s.peer=0x82d0;
    TEST_ASSERT_TRUE(oft_wifi_observe(&s,status,sizeof(status)));
    uint8_t packet[sizeof(center)],ack[40];memcpy(packet,center,sizeof(packet));packet[6]=2;wire_checksum(packet);
    TEST_ASSERT_TRUE(oft_wifi_observe(&s,packet,sizeof(packet)));TEST_ASSERT_EQUAL_HEX16(0x82d8,s.peer);
    TEST_ASSERT_EQUAL_HEX16(0x90a8,s.media_received);
    TEST_ASSERT_EQUAL_UINT(34,oft_wifi_ack(&s,ack,sizeof(ack)));TEST_ASSERT_EQUAL_HEX8(0xa8,ack[8]);TEST_ASSERT_EQUAL_HEX8(0x90,ack[9]);
    packet[4]=0xa0;wire_checksum(packet);TEST_ASSERT_TRUE(oft_wifi_observe(&s,packet,sizeof(packet)));
    TEST_ASSERT_EQUAL_HEX16(0x90a8,s.media_received); /* out-of-order receipt */
}
static void test_pending_heartbeat_loss_does_not_block_next(void)
{
    oft_wifi_pending_t pending={0};oft_wifi_session_t s;oft_wifi_init(&s,1,8);
    TEST_ASSERT_EQUAL_UINT8(1,s.message_counter);
    oft_wifi_pending_add(&pending,0xffff); /* this response is lost */
    oft_wifi_pending_add(&pending,0);
    TEST_ASSERT_TRUE(oft_wifi_pending_take(&pending,0));
    TEST_ASSERT_FALSE(oft_wifi_pending_take(&pending,0));
    TEST_ASSERT_TRUE(oft_wifi_pending_take(&pending,0xffff));
    for(unsigned i=1;i<=5;i++)oft_wifi_pending_add(&pending,i);
    TEST_ASSERT_FALSE(oft_wifi_pending_take(&pending,1)); /* fixed four-slot bound */
    TEST_ASSERT_TRUE(oft_wifi_pending_take(&pending,5));
}
static void test_direct_control_reply_before_media_classification(void)
{
    /* Synthetic multiplexing regression matching the existing Python receive
       order: direct DUML is decoded independently of the envelope WhType. */
    oft_wifi_session_t s;oft_wifi_init(&s,0x7055,8);
    uint8_t raw[32],packet[64];const uint8_t payload[]={0,1,4,1,0,5,1,1};
    size_t n=oft_duml_encode(raw,sizeof(raw),4,2,0x1200,0x80,4,0x50,payload,sizeof(payload));
    n=oft_wifi_wrap(&s,raw,n,packet,sizeof(packet));packet[6]=2;wire_checksum(packet);
    oft_wifi_packet_t v;oft_duml_frame_t f;
    TEST_ASSERT_TRUE(oft_wifi_parse(packet,n,&v));
    TEST_ASSERT_TRUE(oft_wifi_direct_duml(&v,&f));TEST_ASSERT_EQUAL_HEX8(0x50,f.cmd_id);
    TEST_ASSERT_EQUAL_HEX16(0x1200,f.sequence);
    packet[n-1]^=1;TEST_ASSERT_FALSE(oft_wifi_direct_duml(&v,&f));
}
static void test_gimbal_candidate_layout_provenance(void)
{
    uint8_t p[49]={0};p[0]=99;p[16]=0x34;p[17]=0x12;p[20]=0xfe;p[21]=0xff;p[22]=7;
    oft_duml_frame_t f={.cmd_set=4,.cmd_id=5,.payload=p,.payload_size=49};int16_t values[3];
    TEST_ASSERT_TRUE(oft_p3_gimbal_candidates(&f,values));
    TEST_ASSERT_EQUAL_INT(0x1234,values[0]);TEST_ASSERT_EQUAL_INT(-2,values[1]);TEST_ASSERT_EQUAL_INT(7,values[2]);
    f.payload_size=48;TEST_ASSERT_FALSE(oft_p3_gimbal_candidates(&f,values));
}
static void test_cold_and_warm_advertisement(void)
{
    uint8_t p[11]={0xaa,8,0x20,0,0x80,1,2,3,4,5,6};
    TEST_ASSERT_TRUE(oft_p3_advertisement(p,sizeof(p)));
    p[4]=0xc0;TEST_ASSERT_TRUE(oft_p3_advertisement(p,sizeof(p)));
    for(size_t n=0;n<11;n++)TEST_ASSERT_FALSE(oft_p3_advertisement(p,n));
    p[4]=0x40;TEST_ASSERT_FALSE(oft_p3_advertisement(p,sizeof(p)));
    p[4]=0xc0;p[2]=0x21;TEST_ASSERT_FALSE(oft_p3_advertisement(p,sizeof(p)));
    TEST_ASSERT_FALSE(oft_p3_advertisement(NULL,11));
}
void oft_wifi_profile_selftests(void)
{
    RUN_TEST(test_wifi_golden);
    RUN_TEST(test_wifi_flow_and_resynchronization);
    RUN_TEST(test_wifi_sequence_wrap_and_rejected_send);
    RUN_TEST(test_profile_allowlist_and_raw_pwm_rejection);
    RUN_TEST(test_credentials_and_telemetry_bounds);
    RUN_TEST(test_captured_ble_reply_matching);
    RUN_TEST(test_handshake_and_registration_boundaries);
    RUN_TEST(test_media_receipt_does_not_advance_operator);
    RUN_TEST(test_pending_heartbeat_loss_does_not_block_next);
    RUN_TEST(test_direct_control_reply_before_media_classification);
    RUN_TEST(test_gimbal_candidate_layout_provenance);
    RUN_TEST(test_cold_and_warm_advertisement);
}
