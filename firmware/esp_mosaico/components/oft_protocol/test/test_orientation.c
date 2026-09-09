#include "oft_orientation.h"
#include "unity.h"
#include <math.h>
static oft_quat_t axis(float x,float y,float z,float degrees)
{float a=degrees*.017453292519943295f*.5f;return (oft_quat_t){cosf(a),x*sinf(a),y*sinf(a),z*sinf(a)};}
static void test_relative_five_degree_axes(void)
{
    float y,p;oft_quat_t identity=oft_quat_identity();
    TEST_ASSERT_TRUE(oft_orientation_relative(identity,axis(0,1,0,-5),&y,&p));
    TEST_ASSERT_FLOAT_WITHIN(.001f,5,y);TEST_ASSERT_FLOAT_WITHIN(.001f,0,p);
    TEST_ASSERT_TRUE(oft_orientation_relative(identity,axis(1,0,0,5),&y,&p));
    TEST_ASSERT_FLOAT_WITHIN(.001f,0,y);TEST_ASSERT_FLOAT_WITHIN(.001f,5,p);
}
static void test_nod_at_yawed_heading_and_roll(void)
{
    float y,p;oft_quat_t q=oft_quat_multiply(axis(0,1,0,-45),axis(1,0,0,20));
    TEST_ASSERT_TRUE(oft_orientation_relative(oft_quat_identity(),q,&y,&p));
    TEST_ASSERT_FLOAT_WITHIN(.001f,45,y);TEST_ASSERT_FLOAT_WITHIN(.001f,20,p);
    q=oft_quat_multiply(q,axis(0,0,1,30));
    TEST_ASSERT_TRUE(oft_orientation_relative(oft_quat_identity(),q,&y,&p));
    TEST_ASSERT_FLOAT_WITHIN(.001f,45,y);TEST_ASSERT_FLOAT_WITHIN(.001f,20,p);
}
static void test_reference_capture_and_quaternion_sign(void)
{
    float y,p;oft_quat_t ref=oft_quat_multiply(axis(1,0,0,43),axis(0,0,1,80));
    oft_quat_t q=oft_quat_multiply(ref,axis(0,1,0,-5));
    TEST_ASSERT_TRUE(oft_orientation_relative(ref,q,&y,&p));TEST_ASSERT_FLOAT_WITHIN(.001f,5,y);
    q=(oft_quat_t){-q.w,-q.x,-q.y,-q.z};
    TEST_ASSERT_TRUE(oft_orientation_relative(ref,q,&y,&p));TEST_ASSERT_FLOAT_WITHIN(.001f,5,y);
    TEST_ASSERT_TRUE(oft_orientation_relative(q,q,&y,&p));TEST_ASSERT_FLOAT_WITHIN(.001f,0,y);
}
static void test_gyro_integration_and_invalid_samples(void)
{
    oft_quat_t q=oft_quat_identity();float gyro[]={0,-90,0},a[]={0,0,0},y,p;
    for(unsigned i=0;i<100;i++)TEST_ASSERT_TRUE(oft_orientation_step(&q,gyro,a,.01f));
    TEST_ASSERT_TRUE(oft_orientation_relative(oft_quat_identity(),q,&y,&p));TEST_ASSERT_FLOAT_WITHIN(.01f,90,y);
    TEST_ASSERT_FALSE(oft_orientation_step(&q,gyro,a,.1f));
    gyro[0]=NAN;TEST_ASSERT_FALSE(oft_orientation_step(&q,gyro,a,.01f));
}
static void test_gravity_rebase_after_timing_gap(void)
{
    oft_quat_t q=oft_quat_identity();float a[]={0,1,0},g[]={0,0,0},expected[3],up[]={0,0,1};
    TEST_ASSERT_FALSE(oft_orientation_step(&q,g,a,.2f));
    TEST_ASSERT_TRUE(oft_orientation_from_gravity(a,&q));
    oft_quat_rotate(oft_quat_inverse(q),up,expected);
    TEST_ASSERT_FLOAT_WITHIN(.001f,1,expected[1]);
    TEST_ASSERT_TRUE(oft_orientation_step(&q,g,a,.01f));
    a[1]=2;TEST_ASSERT_FALSE(oft_orientation_from_gravity(a,&q));
    a[1]=0;a[2]=-1;TEST_ASSERT_TRUE(oft_orientation_from_gravity(a,&q));
    a[0]=NAN;TEST_ASSERT_FALSE(oft_orientation_from_gravity(a,&q));
}
static void test_vertical_crossing_preserves_yaw_and_ignores_roll(void)
{
    for(int sign=-1;sign<=1;sign+=2){
        oft_direction_tracker_t s={0};
        for(int p=0;p<=120;p+=2){
            oft_quat_t q=oft_quat_multiply(axis(0,1,0,-25),axis(1,0,0,sign*p));
            q=oft_quat_multiply(q,axis(0,0,1,p/3.0f));
            TEST_ASSERT_TRUE(oft_orientation_direction(oft_quat_identity(),q,&s));
            TEST_ASSERT_FLOAT_WITHIN(.01f,25,s.yaw);TEST_ASSERT_FLOAT_WITHIN(.01f,sign*p,s.pitch);
            if(p==90)TEST_ASSERT_TRUE(s.near_vertical);
        }
        for(int p=120;p>=0;p-=2){
            oft_quat_t q=oft_quat_multiply(axis(0,1,0,-25),axis(1,0,0,sign*p));
            TEST_ASSERT_TRUE(oft_orientation_direction(oft_quat_identity(),q,&s));
            TEST_ASSERT_FLOAT_WITHIN(.01f,25,s.yaw);TEST_ASSERT_FLOAT_WITHIN(.01f,sign*p,s.pitch);
        }
        TEST_ASSERT_FALSE(s.near_vertical);
    }
}
void oft_orientation_selftests(void)
{
    RUN_TEST(test_relative_five_degree_axes);
    RUN_TEST(test_nod_at_yawed_heading_and_roll);
    RUN_TEST(test_reference_capture_and_quaternion_sign);
    RUN_TEST(test_gyro_integration_and_invalid_samples);
    RUN_TEST(test_gravity_rebase_after_timing_gap);
    RUN_TEST(test_vertical_crossing_preserves_yaw_and_ignores_roll);
}
