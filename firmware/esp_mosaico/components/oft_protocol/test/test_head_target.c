#include "oft_head_target.h"
#include "oft_head_servo.h"
#include "oft_pocket3.h"
#include "oft_haptic.h"
#include "unity.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
/* Captured immutable evidence: 20260906T175759Z-cdc.txt, us234484610 and
   us245672132; SHA2560627025a597aacb25926fee5275db9aa31ed5a2c7ec5b83a3299995a596e77a3.
   Receive direction, no device credentials. See the head-tracking report. */
static const char *pose_hex[]={
 "553e044b040224ad000405f9f80000000080000000000127670206ffba0000ffff04008207993719e47fbf16f8eebc53a524ba0000000001000000006d70",
 "553e044b040224b600040520fb000009008000f7ff0001e792020632bb000026020000f5407f3c0db362bfe6eaf3bc8e3aedbe00000000010000000033fe"};
static void test_head_captured_feedback_and_quaternion_delta(void)
{
    float q[2][4],pitch[2],y;
    for(unsigned n=0;n<2;n++){
        uint8_t raw[62];for(unsigned i=0;i<62;i++){char h[]={pose_hex[n][i*2],pose_hex[n][i*2+1],0};raw[i]=(uint8_t)strtoul(h,NULL,16);}
        oft_duml_frame_t f;TEST_ASSERT_TRUE(oft_duml_decode(raw,sizeof(raw),&f));
        TEST_ASSERT_TRUE(oft_p3_head_feedback(&f,&y,&pitch[n]));
        float norm=0;for(unsigned i=0;i<4;i++){memcpy(&q[n][i],f.payload+24+4*i,4);norm+=q[n][i]*q[n][i];}
        TEST_ASSERT_FLOAT_WITHIN(.001f,1,norm);
        f.sender=3;TEST_ASSERT_FALSE(oft_p3_head_feedback(&f,&y,&pitch[n]));
        f.sender=4;f.payload_size=48;TEST_ASSERT_FALSE(oft_p3_head_feedback(&f,&y,&pitch[n]));
    }
    float dot=0;for(unsigned i=0;i<4;i++)dot+=q[0][i]*q[1][i];
    float delta=2*acosf(fminf(1,fabsf(dot)))*57.2957795f;
    TEST_ASSERT_FLOAT_WITHIN(.5f,55.1f,delta);
    TEST_ASSERT_FLOAT_WITHIN(.01f,-55.1f,pitch[1]-pitch[0]);
}
static oft_quat_t yaw(float degrees)
{float a=-degrees*.017453292519943295f*.5f;return(oft_quat_t){cosf(a),0,sinf(a),0};}
static oft_head_result_t step(oft_head_target_t *s,bool held,float degrees,float camera_yaw)
{return oft_head_target_step(s,held,1000000,yaw(degrees),1000000,
    (oft_head_camera_t){camera_yaw,3,1000000,true},(oft_head_bounds_t){0},1000000);}
static void test_head_gain_and_reference(void)
{
    oft_head_target_t s;oft_head_target_init(&s);
    TEST_ASSERT_TRUE(step(&s,true,20,10).active);
    oft_head_result_t r=step(&s,true,25,11);
    TEST_ASSERT_FLOAT_WITHIN(.001f,15,r.target_yaw);TEST_ASSERT_FLOAT_WITHIN(.001f,4,r.yaw_error);
    TEST_ASSERT_FALSE(step(&s,false,25,11).active);
    r=step(&s,true,25,11);TEST_ASSERT_FLOAT_WITHIN(.001f,11,r.target_yaw);
    oft_head_target_init(&s);step(&s,true,0,0);
    float a=5*.017453292519943295f*.5f;
    r=oft_head_target_step(&s,true,1000000,(oft_quat_t){cosf(a),sinf(a),0,0},1000000,
        (oft_head_camera_t){0,3,1000000,true},(oft_head_bounds_t){0},1000000);
    TEST_ASSERT_FLOAT_WITHIN(.001f,8,r.target_pitch);
}
static void test_head_fault_requires_release(void)
{
    oft_head_target_t s;oft_head_target_init(&s);step(&s,true,0,0);
    oft_head_result_t r=oft_head_target_step(&s,true,1000000,yaw(5),1000000,
        (oft_head_camera_t){0,0,700000,true},(oft_head_bounds_t){0},1000000);
    TEST_ASSERT_FALSE(r.active);TEST_ASSERT_TRUE(r.release_required);
    TEST_ASSERT_FALSE(step(&s,true,5,0).active);
    step(&s,false,5,0);TEST_ASSERT_TRUE(step(&s,true,5,0).active);
    oft_head_target_cancel(&s);TEST_ASSERT_FALSE(step(&s,true,5,0).active);
}
static void test_head_limits_clear_not_latch(void)
{
    oft_head_target_t s;oft_head_target_init(&s);step(&s,true,0,8);
    oft_head_bounds_t b={true,-10,10,-5,5};
    oft_head_result_t r=oft_head_target_step(&s,true,1000000,yaw(5),1000000,
        (oft_head_camera_t){10,3,1000000,true},b,1000000);
    TEST_ASSERT_TRUE(r.yaw_limit);TEST_ASSERT_FLOAT_WITHIN(.001f,10,r.target_yaw);
    r=oft_head_target_step(&s,true,1000000,yaw(-2),1000000,
        (oft_head_camera_t){10,3,1000000,true},b,1000000);
    TEST_ASSERT_FALSE(r.yaw_limit);TEST_ASSERT_LESS_THAN_FLOAT(0,r.yaw_error);
    TEST_ASSERT_FALSE(step(&s,false,0,10).yaw_limit);
}
static void test_head_wrap_and_invalid_data(void)
{
    oft_head_target_t s;oft_head_target_init(&s);step(&s,true,0,0);
    step(&s,true,90,0);step(&s,true,179,0);
    TEST_ASSERT_FLOAT_WITHIN(.001f,181,step(&s,true,181,0).target_yaw);
    TEST_ASSERT_FALSE(step(&s,true,181,NAN).active);
    step(&s,false,0,0);
    oft_head_result_t r=oft_head_target_step(&s,true,1,yaw(0),1000000,
        (oft_head_camera_t){0,0,1000000,true},(oft_head_bounds_t){0},1000000);
    TEST_ASSERT_FALSE(r.active);TEST_ASSERT_TRUE(r.release_required);
}
static void test_head_servo_existing_writer_mapping(void)
{
    oft_head_servo_config_t c={.75f,8,64};oft_input_t input;int y,p;
    oft_head_result_t target={.active=true,.yaw_error=5};
    TEST_ASSERT_TRUE(oft_head_servo_input(target,c,1,1000,&input));
    TEST_ASSERT_TRUE(oft_control_map(input.yaw,input.pitch,&y,&p));
    TEST_ASSERT_EQUAL_INT(40,y);TEST_ASSERT_EQUAL_INT(0,p);
    target.yaw_error=100;target.pitch_error=100;
    TEST_ASSERT_TRUE(oft_head_servo_input(target,c,1,1000,&input));
    TEST_ASSERT_TRUE(oft_control_map(input.yaw,input.pitch,&y,&p));
    TEST_ASSERT_LESS_OR_EQUAL_FLOAT(65,hypotf(y,p));
    target.yaw_error=0;target.pitch_error=-1;
    TEST_ASSERT_TRUE(oft_head_servo_input(target,c,1,1000,&input));
    TEST_ASSERT_TRUE(oft_control_map(input.yaw,input.pitch,&y,&p));
    TEST_ASSERT_EQUAL_INT(0,y);TEST_ASSERT_EQUAL_INT(-32,p);
    target.pitch_error=.5f;
    TEST_ASSERT_TRUE(oft_head_servo_input(target,c,1,1000,&input));TEST_ASSERT_FALSE(input.active);
    target.release_required=true;target.pitch_error=20;
    TEST_ASSERT_TRUE(oft_head_servo_input(target,c,1,1000,&input));TEST_ASSERT_FALSE(input.active);
    c.maximum_offset=189;TEST_ASSERT_FALSE(oft_head_servo_input(target,c,1,1000,&input));
    TEST_ASSERT_FALSE(input.active);
}
static void test_trial_window_tracks_instead_of_cancelling(void)
{
    oft_head_target_t s;oft_head_target_init(&s);step(&s,true,0,0);
    oft_head_result_t r;oft_input_t input;
    for(unsigned camera=0;camera<=15;camera++){
        r=step(&s,true,20,(float)camera);
        TEST_ASSERT_TRUE(oft_head_target_window(&r,&s,15));
        TEST_ASSERT_TRUE(r.active);TEST_ASSERT_FALSE(r.release_required);
        TEST_ASSERT_FALSE(r.yaw_limit); /* trial window is not a camera limit */
        TEST_ASSERT_FLOAT_WITHIN(.001f,15,r.target_yaw);
        TEST_ASSERT_FLOAT_WITHIN(.001f,15-camera,r.yaw_error);
        TEST_ASSERT_TRUE(oft_head_servo_input(r,(oft_head_servo_config_t){.75f,8,64},1,1000000,&input));
        TEST_ASSERT_EQUAL(camera<15,input.active);
    }
    r=step(&s,true,10,15);TEST_ASSERT_FALSE(oft_head_target_window(&r,&s,15));
    TEST_ASSERT_FLOAT_WITHIN(.001f,10,r.target_yaw);TEST_ASSERT_FLOAT_WITHIN(.001f,-5,r.yaw_error);
    r=step(&s,false,20,15);oft_head_target_window(&r,&s,15);TEST_ASSERT_FALSE(r.active);
    r=step(&s,true,20,15);TEST_ASSERT_FALSE(oft_head_target_window(&r,&s,15));
    TEST_ASSERT_FLOAT_WITHIN(.001f,15,r.target_yaw);
    oft_head_target_window(&r,&s,NAN);TEST_ASSERT_FALSE(r.active);TEST_ASSERT_TRUE(r.release_required);
}
static void test_camera_yaw_wrap_and_gap(void)
{
    oft_head_yaw_tracker_t s={0};float out;
    TEST_ASSERT_TRUE(oft_head_yaw_update(&s,-179,1000000,&out));
    TEST_ASSERT_TRUE(oft_head_yaw_update(&s,179,1100000,&out));TEST_ASSERT_FLOAT_WITHIN(.001f,-181,out);
    TEST_ASSERT_TRUE(oft_head_yaw_update(&s,135.54f,1200000,&out));TEST_ASSERT_FLOAT_WITHIN(.001f,-224.46f,out);
    TEST_ASSERT_TRUE(oft_head_yaw_update(&s,179,1300000,&out));
    TEST_ASSERT_TRUE(oft_head_yaw_update(&s,-179,1400000,&out));TEST_ASSERT_FLOAT_WITHIN(.001f,-179,out);
    s=(oft_head_yaw_tracker_t){0};
    TEST_ASSERT_TRUE(oft_head_yaw_update(&s,179,2000000,&out));
    TEST_ASSERT_TRUE(oft_head_yaw_update(&s,-179,2100000,&out));TEST_ASSERT_FLOAT_WITHIN(.001f,181,out);
    TEST_ASSERT_FALSE(oft_head_yaw_update(&s,-178,2400000,&out));TEST_ASSERT_FALSE(s.valid);
    TEST_ASSERT_TRUE(oft_head_yaw_update(&s,-178,2500000,&out));TEST_ASSERT_FLOAT_WITHIN(.001f,-178,out);
    TEST_ASSERT_FALSE(oft_head_yaw_update(&s,NAN,2600000,&out));
}
static void test_measured_soft_limit_position_and_clear(void)
{
    oft_head_bounds_t b=oft_p3_head_bounds();
    TEST_ASSERT_FLOAT_WITHIN(.001f,-224.46f,oft_p3_head_yaw_seed(135.54f));
    TEST_ASSERT_FLOAT_WITHIN(.001f,-175,oft_p3_head_yaw_seed(-175));
    oft_head_camera_t camera={b.yaw_min,0,1000000,true};
    TEST_ASSERT_EQUAL_UINT(1,oft_head_limit_status(camera,b,1000000,true));
    /* No held-input parameter: release alone does not falsely report that a
       camera remaining at the soft boundary has physically moved away. */
    camera.yaw=b.yaw_min+1;TEST_ASSERT_EQUAL_UINT(0,oft_head_limit_status(camera,b,1000000,true));
    camera.pitch=b.pitch_max;TEST_ASSERT_EQUAL_UINT(2,oft_head_limit_status(camera,b,1000000,true));
    camera.yaw=b.yaw_max;TEST_ASSERT_EQUAL_UINT(3,oft_head_limit_status(camera,b,1000000,true));
    TEST_ASSERT_EQUAL_UINT(0,oft_head_limit_status(camera,b,1000000,false));
    TEST_ASSERT_EQUAL_UINT(0,oft_head_limit_status(camera,b,1300000,true));
    camera.pitch=NAN;TEST_ASSERT_EQUAL_UINT(0,oft_head_limit_status(camera,b,1000000,true));
}
static void test_press_at_real_endpoint_has_no_inward_jump(void)
{
    oft_head_target_t s;oft_head_target_init(&s);oft_head_bounds_t b=oft_p3_head_bounds();
    oft_head_camera_t camera={48.31f,0,1000000,true};
    oft_head_result_t r=oft_head_target_step(&s,true,1000000,yaw(0),1000000,camera,b,1000000);
    TEST_ASSERT_FLOAT_WITHIN(.001f,0,r.yaw_error);
    r=oft_head_target_step(&s,true,1000000,yaw(5),1000000,camera,b,1000000);
    TEST_ASSERT_FLOAT_WITHIN(.001f,0,r.yaw_error);
    r=oft_head_target_step(&s,true,1000000,yaw(-5),1000000,camera,b,1000000);
    TEST_ASSERT_FLOAT_WITHIN(.001f,-5,r.yaw_error);
}
static void test_fast_head_defaults_and_slowdown(void)
{
    oft_head_servo_config_t c=oft_head_servo_default();oft_input_t in;int y,p;
    const float error[]={50,12,6,3,1,.75f};const int expected[]={188,188,96,48,32,0};
    for(unsigned i=0;i<6;i++){
        TEST_ASSERT_TRUE(oft_head_servo_input((oft_head_result_t){.active=true,.yaw_error=error[i]},c,1,1000000,&in));
        TEST_ASSERT_TRUE(oft_control_map(in.yaw,in.pitch,&y,&p));
        TEST_ASSERT_EQUAL_INT(expected[i],y);TEST_ASSERT_EQUAL_INT(0,p);
    }
    TEST_ASSERT_TRUE(oft_head_servo_input((oft_head_result_t){.active=true,.yaw_error=50,.pitch_error=-50},c,1,1000000,&in));
    TEST_ASSERT_TRUE(oft_control_map(in.yaw,in.pitch,&y,&p));
    TEST_ASSERT_GREATER_THAN_INT(0,y);TEST_ASSERT_LESS_THAN_INT(0,p);
    TEST_ASSERT_LESS_OR_EQUAL_FLOAT(189,hypotf(y,p));
    TEST_ASSERT_TRUE(oft_head_servo_input((oft_head_result_t){0},c,1,1000000,&in));TEST_ASSERT_FALSE(in.active);
}
static void test_short_haptic_pulses_and_cancel(void)
{
    oft_haptic_pattern_t s={0};
    TEST_ASSERT_FALSE(oft_haptic_output(&s,1000000));
    for(int64_t elapsed=0;elapsed<=1200000;elapsed+=10000){
        oft_haptic_request(&s,true,1000000+elapsed);
        TEST_ASSERT_EQUAL(elapsed%100000<50000,oft_haptic_output(&s,1000000+elapsed));
    }
    oft_haptic_request(&s,false,2210000);TEST_ASSERT_FALSE(oft_haptic_output(&s,2210000));
    oft_haptic_request(&s,true,2220000);TEST_ASSERT_TRUE(oft_haptic_output(&s,2220000));
    oft_haptic_request(&s,true,2280000);TEST_ASSERT_FALSE(oft_haptic_output(&s,2300000)); /* renew keeps phase */
    TEST_ASSERT_FALSE(oft_haptic_output(&s,2380000)); /* expired lease */
    oft_haptic_request(&s,true,2400000);TEST_ASSERT_TRUE(oft_haptic_output(&s,2400000));
    oft_haptic_request(&s,false,2410000);TEST_ASSERT_FALSE(oft_haptic_output(&s,2410000));
    oft_haptic_request(&s,true,INT64_MAX);TEST_ASSERT_FALSE(oft_haptic_output(&s,INT64_MAX));
}
static void test_center_anchor_replays_yaw_limit_shift(void)
{
    oft_head_center_anchor_t s;oft_head_center_begin(&s,1000000);float center,shift;
    oft_head_camera_t c={-173.96f,.1f,1500000,true};
    TEST_ASSERT_FALSE(oft_head_center_sample(&s,c,true,1500000,&center));
    for(unsigned i=0;i<5;i++){
        c.timestamp_us=2000000+i*100000;
        TEST_ASSERT_EQUAL(i==4,oft_head_center_sample(&s,c,true,c.timestamp_us,&center));
    }
    TEST_ASSERT_TRUE(oft_p3_head_center_shift(center,&shift));TEST_ASSERT_FLOAT_WITHIN(.01f,1.59f,shift);
    oft_head_bounds_t b=oft_p3_head_bounds();c.yaw=-222.58f;
    TEST_ASSERT_EQUAL_UINT(0,oft_head_limit_status(c,b,c.timestamp_us,true));
    b.yaw_min+=shift;b.yaw_max+=shift;
    TEST_ASSERT_EQUAL_UINT(1,oft_head_limit_status(c,b,c.timestamp_us,true));
    c.yaw=-221;TEST_ASSERT_EQUAL_UINT(0,oft_head_limit_status(c,b,c.timestamp_us,true));
    TEST_ASSERT_FALSE(oft_p3_head_center_shift(-150,&shift));
    oft_head_center_begin(&s,1000000);
    TEST_ASSERT_FALSE(oft_head_center_sample(&s,c,false,2000000,&center));TEST_ASSERT_FALSE(s.pending);
    oft_head_center_begin(&s,1000000);
    TEST_ASSERT_FALSE(oft_head_center_sample(&s,c,true,4500000,&center));TEST_ASSERT_FALSE(s.pending);
}
static void test_captured_vertical_path_does_not_latch_left_limit(void)
{
    /* Directions from20260906T213236Z-cdc.txt (not claimed raw IMU quaternions).
       Reconstruct equivalent nose directions to reproduce azimuth pole flips. */
    const float samples[][2]={{-1.076f,62.546f},{-41.916f,86.408f},{-129.956f,87.890f},
        {-157.738f,85.952f},{-177.077f,85.774f},{173.224f,84.909f},{12.571f,35.164f},{19.641f,3.999f}};
    oft_head_target_t s;oft_head_target_init(&s);
    oft_head_camera_t camera={-173.87f,.1f,1000000,true};oft_head_bounds_t bounds=oft_p3_head_bounds();
    oft_head_result_t r=oft_head_target_step(&s,true,1000000,oft_quat_identity(),1000000,camera,bounds,1000000);
    for(unsigned i=0;i<sizeof(samples)/sizeof(samples[0]);i++){
        float a=samples[i][1]*.017453292519943295f*.5f;
        oft_quat_t q=oft_quat_multiply(yaw(samples[i][0]),(oft_quat_t){cosf(a),sinf(a),0,0});
        r=oft_head_target_step(&s,true,1000000,q,1000000,camera,bounds,1000000);
        TEST_ASSERT_TRUE(r.active);
        TEST_ASSERT_GREATER_THAN_FLOAT(-200,r.target_yaw);
    }
    TEST_ASSERT_FLOAT_WITHIN(.01f,-154.229f,r.target_yaw);
    TEST_ASSERT_FLOAT_WITHIN(.01f,4.099f,r.target_pitch);
    TEST_ASSERT_FALSE(s.direction.near_vertical);
}
void oft_head_target_selftests(void)
{
    RUN_TEST(test_head_gain_and_reference);
    RUN_TEST(test_head_fault_requires_release);
    RUN_TEST(test_head_limits_clear_not_latch);
    RUN_TEST(test_head_wrap_and_invalid_data);
    RUN_TEST(test_head_servo_existing_writer_mapping);
    RUN_TEST(test_head_captured_feedback_and_quaternion_delta);
    RUN_TEST(test_trial_window_tracks_instead_of_cancelling);
    RUN_TEST(test_camera_yaw_wrap_and_gap);
    RUN_TEST(test_measured_soft_limit_position_and_clear);
    RUN_TEST(test_press_at_real_endpoint_has_no_inward_jump);
    RUN_TEST(test_fast_head_defaults_and_slowdown);
    RUN_TEST(test_short_haptic_pulses_and_cancel);
    RUN_TEST(test_center_anchor_replays_yaw_limit_shift);
    RUN_TEST(test_captured_vertical_path_does_not_latch_left_limit);
}
