#include "oft_settings.h"
#include "unity.h"
#include <string.h>
#include "fixtures/pocket3_settings.h"
static oft_duml_frame_t named_frame(unsigned key,const uint8_t *value,size_t n,uint8_t *p)
{
    size_t len=strlen(oft_settings_key(key));memset(p,0,300);p[0]=2;p[1]=6;
    unsigned total=(unsigned)(10+len+n);p[11]=total;p[12]=total>>8;p[13]=len;
    memcpy(p+15,oft_settings_key(key),len);p[21+len]=n;p[22+len]=n>>8;memcpy(p+23+len,value,n);
    return(oft_duml_frame_t){.sender=0x28,.receiver=2,.cmd_id=0x99,.payload=p,.payload_size=23+len+n};
}
static void test_settings_wire_allowlist(void)
{
    uint8_t raw[100];oft_duml_frame_t f;size_t n=oft_settings_build(OFT_SET_VIDEO,0x10,3,7,raw,sizeof(raw));
    TEST_ASSERT_TRUE(oft_settings_allowed(raw,n));TEST_ASSERT_TRUE(oft_duml_decode(raw,n,&f));
    const uint8_t payload[]={0x10,3,0,0,0};TEST_ASSERT_EQUAL_MEMORY(payload,f.payload,5);
    TEST_ASSERT_EQUAL_UINT8(1,f.receiver);TEST_ASSERT_EQUAL_UINT8(0x18,f.cmd_id);
    TEST_ASSERT_EQUAL_UINT(0,oft_settings_build(OFT_SET_VIDEO,0xff,3,1,raw,sizeof(raw)));
    TEST_ASSERT_EQUAL_UINT(0,oft_settings_build(OFT_SET_VIDEO,0x10,8,1,raw,sizeof(raw)));
    TEST_ASSERT_EQUAL_UINT(0,oft_settings_build(OFT_SET_MODE,0x1a,0,1,raw,sizeof(raw)));
    TEST_ASSERT_EQUAL_UINT(0,oft_settings_build(OFT_SET_ZOOM,2604,0,1,raw,sizeof(raw)));
    TEST_ASSERT_EQUAL_UINT(0,oft_settings_build(999,0,0,1,raw,sizeof(raw)));
    n=oft_settings_build(OFT_SET_ZOOM,434,0,1,raw,sizeof(raw));TEST_ASSERT_TRUE(oft_settings_allowed(raw,n));
    raw[n-1]^=1;TEST_ASSERT_FALSE(oft_settings_allowed(raw,n));
}
static void test_settings_capabilities_not_cartesian(void)
{
    uint8_t p[300],v[]={1,10,0,3,0x10,3,0,0x0a,6,0,0xff,7,0};oft_settings_state_t s={0};
    oft_duml_frame_t f=named_frame(OFT_SUB_VIDEO_CAP,v,sizeof(v),p);
    TEST_ASSERT_TRUE(oft_settings_observe(&s,&f,100));TEST_ASSERT_EQUAL_UINT(3,s.pair_count);
    TEST_ASSERT_EQUAL_UINT(0xff,s.pairs[2].resolution);
    TEST_ASSERT_TRUE(oft_settings_video_pair(&s,0x10,3));TEST_ASSERT_FALSE(oft_settings_video_pair(&s,0x10,6));
    TEST_ASSERT_FALSE(oft_settings_video_pair(&s,0xff,7));
    f.payload_size--;TEST_ASSERT_FALSE(oft_settings_observe(&s,&f,200));
}
static void test_settings_named_bounds(void)
{
    uint8_t p[300],v[]={0x10,3};oft_setting_item_t item;
    oft_duml_frame_t f=named_frame(OFT_SUB_VIDEO,v,sizeof(v),p);
    TEST_ASSERT_TRUE(oft_settings_item(&f,&item));TEST_ASSERT_EQUAL_UINT(2,item.size);
    p[13]=255;TEST_ASSERT_FALSE(oft_settings_item(&f,&item));
    f=named_frame(OFT_SUB_VIDEO,v,sizeof(v),p);f.sender=7;TEST_ASSERT_FALSE(oft_settings_item(&f,&item));
    f.sender=0x28;f.payload_size=22;TEST_ASSERT_FALSE(oft_settings_item(&f,&item));
}
static void test_settings_mode_record_and_zoom_gates(void)
{
    oft_camera_state_t cam={.mode=OFT_CAPTURE_VIDEO,.raw_mode=1,.valid=true,.updated_us=100};
    oft_settings_state_t s={.video_cap_valid=true,.pair_count=1,.pairs={{0x10,3,0}},.video_valid=true,.resolution=0x10,.lens_valid=true,.lens=217};
    TEST_ASSERT_TRUE(oft_settings_can_set(&s,cam,OFT_SET_VIDEO,0x10,3,100));
    cam.recording=true;TEST_ASSERT_FALSE(oft_settings_can_set(&s,cam,OFT_SET_VIDEO,0x10,3,100));
    TEST_ASSERT_TRUE(oft_settings_can_set(&s,cam,OFT_SET_ZOOM,434,0,100));
    TEST_ASSERT_FALSE(oft_settings_can_set(&s,cam,OFT_SET_ZOOM,435,0,100));
    cam.recording=false;TEST_ASSERT_FALSE(oft_settings_can_set(&s,cam,OFT_SET_VIDEO,0x10,3,2000101));
    s.video_cap_valid=false;TEST_ASSERT_FALSE(oft_settings_can_set(&s,cam,OFT_SET_VIDEO,0x10,3,100));
    s.fps=3;TEST_ASSERT_FALSE(oft_settings_reference_pairs(&s,5));TEST_ASSERT_TRUE(oft_settings_reference_pairs(&s,1));
    TEST_ASSERT_TRUE(s.reference_pairs);TEST_ASSERT_EQUAL_UINT(18,s.pair_count);TEST_ASSERT_FALSE(oft_settings_video_pair(&s,0x10,8));
    TEST_ASSERT_EQUAL_UINT(300,oft_settings_zoom_max(1,0x2d));TEST_ASSERT_EQUAL_UINT(100,oft_settings_zoom_max(0,0x10));
}
static void test_settings_photo_and_unknown_values(void)
{
    uint8_t p[300],v[]={1,5,0,2,1,0,2,0};oft_settings_state_t s={0};
    oft_duml_frame_t f=named_frame(OFT_SUB_PHOTO_FILE_CAP,v,sizeof(v),p);
    TEST_ASSERT_TRUE(oft_settings_observe(&s,&f,100));TEST_ASSERT_EQUAL_UINT(2,s.photo_file_count);
    oft_camera_state_t cam={.mode=OFT_CAPTURE_PHOTO,.raw_mode=5,.valid=true,.updated_us=100};
    TEST_ASSERT_FALSE(oft_settings_can_set(&s,cam,OFT_SET_PHOTO_FILE,2,0,100));s.photo_file_valid=true;s.photo_file=1;
    TEST_ASSERT_TRUE(oft_settings_can_set(&s,cam,OFT_SET_PHOTO_FILE,2,0,100));
    TEST_ASSERT_FALSE(oft_settings_can_set(&s,cam,OFT_SET_PHOTO_FILE,0,0,100));
    s.photo_valid=true;s.photo_size=0;s.photo_aspect=3;
    TEST_ASSERT_TRUE(oft_settings_can_set(&s,cam,OFT_SET_PHOTO_SIZE,0,1,100));
    TEST_ASSERT_FALSE(oft_settings_can_set(&s,cam,OFT_SET_PHOTO_SIZE,4,1,100));
    TEST_ASSERT_FALSE(oft_settings_can_set(&s,cam,OFT_SET_PHOTO_SIZE,0,2,100));
    uint8_t lens[16]={0};lens[14]=0xff;lens[15]=0xff;f=named_frame(OFT_SUB_LENS,lens,sizeof(lens),p);
    oft_settings_observe(&s,&f,100);TEST_ASSERT_FALSE(s.lens_valid);TEST_ASSERT_EQUAL_UINT(65535,s.lens);
}
static unsigned nibble(char c){return c<='9'?(unsigned)(c-'0'):(unsigned)(c-'a'+10);}
static void test_settings_owner_capture(void)
{
    uint8_t raw[100];oft_settings_state_t s={0};oft_duml_frame_t f;
    const char *frames[]={pocket3_lens_frame,pocket3_video_frame};
    for(unsigned k=0;k<2;k++){
        size_t n=strlen(frames[k])/2;TEST_ASSERT_TRUE(n<=sizeof(raw));
        for(size_t i=0;i<n;i++)raw[i]=(uint8_t)((nibble(frames[k][2*i])<<4)|nibble(frames[k][2*i+1]));
        TEST_ASSERT_TRUE(oft_duml_decode(raw,n,&f));TEST_ASSERT_TRUE(oft_settings_observe(&s,&f,100+k));
    }
    TEST_ASSERT_TRUE(s.lens_valid);TEST_ASSERT_EQUAL_UINT(217,s.lens);
    TEST_ASSERT_TRUE(s.video_valid);TEST_ASSERT_EQUAL_HEX8(0x10,s.resolution);TEST_ASSERT_EQUAL_UINT(6,s.fps);
}
void oft_settings_selftests(void)
{RUN_TEST(test_settings_wire_allowlist);RUN_TEST(test_settings_capabilities_not_cartesian);RUN_TEST(test_settings_named_bounds);RUN_TEST(test_settings_mode_record_and_zoom_gates);RUN_TEST(test_settings_photo_and_unknown_values);RUN_TEST(test_settings_owner_capture);}
