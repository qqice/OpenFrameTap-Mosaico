#include "oft_media.h"
#include "unity.h"
#include <stdlib.h>
#include <string.h>
static size_t packet(uint8_t *p,unsigned id,unsigned count,unsigned index,const uint8_t *data,size_t n)
{
    memset(p,0,20);size_t size=n+20;p[0]=size;p[1]=0x80|(size>>8);p[2]=1;p[6]=2;
    for(unsigned i=0;i<7;i++)p[7]^=p[i];
    p[16]=id;p[17]=count|((index&1)<<7);p[18]=index/2;
    memcpy(p+20,data,n);return size;
}
static void test_media_reorder_duplicates_and_nals(void)
{
    uint8_t *g=malloc(OFT_MEDIA_GROUP_BYTES),au[100],p[1472];TEST_ASSERT_NOT_NULL(g);
    oft_media_t *s=malloc(sizeof(*s));TEST_ASSERT_NOT_NULL(s);oft_media_init(s,g,au,sizeof(au));
    uint8_t a[]={0,0,1,255,8,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},b[]={0x65,1,2,3};
    size_t n=packet(p,1,2,1,b,sizeof(b));TEST_ASSERT_EQUAL_UINT(0,oft_media_feed(s,p,n,100000));
    TEST_ASSERT_EQUAL_UINT(0,oft_media_feed(s,p,n,100010));TEST_ASSERT_EQUAL_UINT(1,s->duplicates);
    n=packet(p,1,2,0,a,sizeof(a));TEST_ASSERT_EQUAL_UINT(8,oft_media_feed(s,p,n,100020));
    TEST_ASSERT_EQUAL_UINT(1u<<5,oft_media_nal_mask(au,8));
    TEST_ASSERT_EQUAL_UINT(0,oft_media_feed(s,p,n,100030));
    free(s);free(g);
}
static void test_media_large_idr_continuation_and_bounds(void)
{
    uint8_t *g=malloc(OFT_MEDIA_GROUP_BYTES),*au=malloc(100000),data[1452]={0},p[1472];TEST_ASSERT_NOT_NULL(g);TEST_ASSERT_NOT_NULL(au);
    oft_media_t *s=malloc(sizeof(*s));TEST_ASSERT_NOT_NULL(s);oft_media_init(s,g,au,100000);size_t expected=63*1452-16+4;
    for(unsigned i=0;i<63;i++){
        memset(data,0,sizeof(data));
        if(i==0){data[2]=1;data[3]=255;for(unsigned k=0;k<4;k++)data[4+k]=expected>>(8*k);data[18]=1;data[19]=0x65;}
        size_t n=packet(p,255,63,i,data,sizeof(data));TEST_ASSERT_EQUAL_UINT(0,oft_media_feed(s,p,n,100000+i));
    }
    TEST_ASSERT_TRUE(s->continuation);
    size_t n=packet(p,0,1,0,(uint8_t[]){4,3,2,1},4);
    TEST_ASSERT_EQUAL_UINT(expected,oft_media_feed(s,p,n,100100));
    TEST_ASSERT_EQUAL_UINT(1u<<5,oft_media_nal_mask(au,expected));
    p[7]^=1;TEST_ASSERT_EQUAL_UINT(0,oft_media_feed(s,p,n,100200));TEST_ASSERT_GREATER_THAN_UINT(0,s->invalid);
    free(s);free(g);free(au);
}
static void test_media_conflict_expiry_and_length_rejection(void)
{
    uint8_t *g=malloc(OFT_MEDIA_GROUP_BYTES),au[32],p[1472],h[20]={0,0,1,255,8};TEST_ASSERT_NOT_NULL(g);
    oft_media_t *s=malloc(sizeof(*s));TEST_ASSERT_NOT_NULL(s);oft_media_init(s,g,au,sizeof(au));size_t n=packet(p,2,2,0,h,sizeof(h));
    oft_media_feed(s,p,n,100000);p[20+19]^=1;oft_media_feed(s,p,n,100100);TEST_ASSERT_EQUAL_UINT(1,s->conflicts);
    n=packet(p,3,2,0,h,sizeof(h));oft_media_feed(s,p,n,200000);
    n=packet(p,3,2,1,(uint8_t[]){1,2,3,4},4);TEST_ASSERT_EQUAL_UINT(0,oft_media_feed(s,p,n,800001));
    n=packet(p,4,1,0,h,sizeof(h));TEST_ASSERT_EQUAL_UINT(0,oft_media_feed(s,p,n,900000));
    h[4]=100;n=packet(p,5,1,0,h,sizeof(h));TEST_ASSERT_EQUAL_UINT(0,oft_media_feed(s,p,n,1000000));TEST_ASSERT_GREATER_THAN_UINT(0,s->invalid);
    free(s);free(g);
}
static void test_media_slice_headers(void)
{
    unsigned ref,kind;
    TEST_ASSERT_TRUE(oft_media_slice((uint8_t[]){0,0,0,1,0x61,0xe0},6,&ref,&kind));
    TEST_ASSERT_EQUAL_UINT(3,ref);TEST_ASSERT_EQUAL_UINT(0,kind);
    TEST_ASSERT_TRUE(oft_media_slice((uint8_t[]){0,0,1,0x01,0xa8},5,&ref,&kind));
    TEST_ASSERT_EQUAL_UINT(0,ref);TEST_ASSERT_EQUAL_UINT(1,kind);
    TEST_ASSERT_FALSE(oft_media_slice((uint8_t[]){0,0,1,0x65,0},5,&ref,&kind));
    TEST_ASSERT_FALSE(oft_media_slice((uint8_t[]){0,0,1,0x67,0xe0},5,&ref,&kind));
}
static void test_media_late_foreign_group_preserves_idr(void)
{
    uint8_t *g=malloc(OFT_MEDIA_GROUP_BYTES),au[32],p[1472];TEST_ASSERT_NOT_NULL(g);
    oft_media_t *s=malloc(sizeof(*s));TEST_ASSERT_NOT_NULL(s);oft_media_init(s,g,au,sizeof(au));
    uint8_t a[]={0,0,1,255,8,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0x65};
    size_t n=packet(p,3,2,0,a,sizeof(a));oft_media_feed(s,p,n,100000);
    TEST_ASSERT_TRUE(s->protect_idr);
    n=packet(p,9,1,0,(uint8_t[]){1,2,3,4},4);oft_media_feed(s,p,n,100010);
    TEST_ASSERT_EQUAL_UINT(1,s->foreign_packets);TEST_ASSERT_EQUAL_UINT(0,s->dropped);
    n=packet(p,3,2,1,(uint8_t[]){1,2,3,4},4);
    TEST_ASSERT_EQUAL_UINT(8,oft_media_feed(s,p,n,100020));TEST_ASSERT_FALSE(s->protect_idr);
    TEST_ASSERT_EQUAL_UINT(0,s->lost_idrs);
    n=packet(p,5,2,0,a,sizeof(a));oft_media_feed(s,p,n,200000);
    n=packet(p,9,1,0,(uint8_t[]){1,2,3,4},4);oft_media_feed(s,p,n,800001);
    TEST_ASSERT_EQUAL_UINT(1,s->lost_idrs);TEST_ASSERT_TRUE(s->lost_idr_source_us==200000);
    oft_media_feed(s,p,n,900000);TEST_ASSERT_EQUAL_UINT(1,s->lost_idrs);
    free(s);free(g);
}
void oft_media_selftests(void)
{
    RUN_TEST(test_media_reorder_duplicates_and_nals);
    RUN_TEST(test_media_large_idr_continuation_and_bounds);
    RUN_TEST(test_media_conflict_expiry_and_length_rejection);
    RUN_TEST(test_media_slice_headers);
    RUN_TEST(test_media_late_foreign_group_preserves_idr);
}
