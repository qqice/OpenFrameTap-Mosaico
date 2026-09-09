/* SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2013, Cisco Systems
 * Copyright (c) 2026, OpenFrameTap contributors
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Experimental software CABAC engine: a normalized small arithmetic value and
 * a separate32-bit bit reservoir replace the reference40-bit lazy offset.
 * uiOffset packs value(low32)/reservoir(high32); iBitsLeft counts cached bits.
 * Init/restore and all arithmetic entry points must be selected together.
 * Syntax parsing, probability tables and reconstruction remain OpenH264.
 */
#include "cabac32.h"
#include "cabac_decoder.h"
#include <cstdio>

static bool enabled;
void oft_cabac32_select(bool on){enabled=on;}
namespace WelsDec {
int32_t ReferenceDecodeBinCabac(PWelsCabacDecEngine,PWelsCabacCtx,uint32_t&);
int32_t ReferenceDecodeBypassCabac(PWelsCabacDecEngine,uint32_t&);
int32_t ReferenceDecodeTerminateCabac(PWelsCabacDecEngine,uint32_t&);
int32_t ReferenceInitCabacDecEngineFromBS(PWelsCabacDecEngine,PBitStringAux);
void ReferenceRestoreCabacDecEngineToBS(PWelsCabacDecEngine,PBitStringAux);
static int32_t invalid(){return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_CABAC_UNEXPECTED_VALUE);}
static uint32_t value(PWelsCabacDecEngine e){return (uint32_t)e->uiOffset;}
static void set_value(PWelsCabacDecEngine e,uint32_t v){e->uiOffset=(e->uiOffset&0xffffffff00000000ULL)|v;}
static int32_t refill_bits(PWelsCabacDecEngine e,unsigned need,uint32_t &out)
{
    if(need>16||e->iBitsLeft<0||e->iBitsLeft>32)return invalid();
    uint32_t reservoir=(uint32_t)(e->uiOffset>>32),v=value(e);int count=e->iBitsLeft;out=0;
    while(need){
        if(!count){
            auto left=e->pBuffEnd-e->pBuffCurr;
            if(left<=0){e->uiOffset=v;e->iBitsLeft=0;return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_CABAC_NO_BS_TO_READ);}
            unsigned n=left>=4?4:(unsigned)left;reservoir=0;
            for(unsigned i=0;i<n;i++)reservoir|=(uint32_t)e->pBuffCurr[i]<<(24-8*i);
            e->pBuffCurr+=n;count=(int)n*8;
        }
        unsigned take=need<(unsigned)count?need:(unsigned)count;
        out=(out<<take)|(reservoir>>(32-take));reservoir<<=take;
        count-=(int)take;need-=take;
    }
    e->uiOffset=((uint64_t)reservoir<<32)|v;e->iBitsLeft=count;return ERR_NONE;
}
static inline __attribute__((always_inline)) int32_t get_bits(PWelsCabacDecEngine e,unsigned need,uint32_t &out)
{
    /* Most renormalizations consume 1..7 already-buffered bits. Keep refill
       and its loop out of the hot path; retain all bounds checks on the slow
       path. No bitstream read or speculative overread in this branch. */
    unsigned count=(unsigned)e->iBitsLeft;
    if(__builtin_expect(need>0&&need<=16&&count>=need&&count<=32,1)){
        uint32_t reservoir=(uint32_t)(e->uiOffset>>32);
        out=reservoir>>(32-need);
        e->uiOffset=((uint64_t)(reservoir<<need)<<32)|value(e);
        e->iBitsLeft=(int)(count-need);return ERR_NONE;
    }
    return refill_bits(e,need,out);
}
static int32_t initialize(PWelsCabacDecEngine e,PBitStringAux bs)
{
    uint8_t *start=bs->pCurBuf-((-bs->iLeftBits>>3)+2);
    if(start>bs->pEndBuf||bs->pEndBuf-start<2)return ERR_INFO_INVALID_ACCESS;
    e->uiRange=WELS_CABAC_HALF;e->uiOffset=0;e->iBitsLeft=0;
    e->pBuffStart=bs->pStartBuf;e->pBuffCurr=start;e->pBuffEnd=bs->pEndBuf;
    uint32_t v;int32_t rc=get_bits(e,9,v);if(rc)return rc;
    if(v>=WELS_CABAC_HALF)return invalid();
    set_value(e,v);bs->iLeftBits=0;return ERR_NONE;
}
static int32_t bin(PWelsCabacDecEngine e,PWelsCabacCtx c,uint32_t &out)
{
    if(e->uiRange<2||e->uiRange>510||c->uiState>63||c->uiMPS>1)return invalid();
    uint32_t range=(uint32_t)e->uiRange,v=value(e),state=c->uiState;
    uint32_t lps=g_kuiCabacRangeLps[state][(range>>6)&3];range-=lps;out=c->uiMPS;
    unsigned shift=1;
    if(v>=range){
        v-=range;out^=1;if(!state)c->uiMPS^=1;
        c->uiState=g_kuiStateTransTable[state][0];shift=g_kRenormTable256[lps];range=lps<<shift;
    }else{
        c->uiState=g_kuiStateTransTable[state][1];
        if(range>=WELS_CABAC_QUARTER){e->uiRange=range;return ERR_NONE;}
        range<<=1;
    }
    uint32_t incoming;int32_t rc=get_bits(e,shift,incoming);if(rc)return rc;
    e->uiRange=range;set_value(e,(v<<shift)|incoming);return ERR_NONE;
}
static int32_t bypass(PWelsCabacDecEngine e,uint32_t &out)
{
    if(e->uiRange>510)return invalid();
    uint32_t range=(uint32_t)e->uiRange;
    uint32_t next;int32_t rc=get_bits(e,1,next);if(rc)return rc;
    uint32_t v=(value(e)<<1)|next;out=v>=range;if(out)v-=range;
    set_value(e,v);return ERR_NONE;
}
static int32_t terminate(PWelsCabacDecEngine e,uint32_t &out)
{
    if(e->uiRange<2||e->uiRange>510)return invalid();
    uint32_t range=(uint32_t)e->uiRange-2,v=value(e);out=v>=range;
    if(out)return ERR_NONE;
    if(range<WELS_CABAC_QUARTER){
        unsigned shift=g_kRenormTable256[range];uint32_t next;
        int32_t rc=get_bits(e,shift,next);if(rc)return rc;
        range<<=shift;set_value(e,(v<<shift)|next);
    }
    e->uiRange=range;return ERR_NONE;
}
int32_t InitCabacDecEngineFromBS(PWelsCabacDecEngine e,PBitStringAux bs)
{return enabled?initialize(e,bs):ReferenceInitCabacDecEngineFromBS(e,bs);}
void RestoreCabacDecEngineToBS(PWelsCabacDecEngine e,PBitStringAux bs)
{ReferenceRestoreCabacDecEngineToBS(e,bs);} // Pointer minus cached whole bytes yields the same consumed-byte position.
int32_t DecodeBinCabac(PWelsCabacDecEngine e,PWelsCabacCtx c,uint32_t &v)
{return enabled?bin(e,c,v):ReferenceDecodeBinCabac(e,c,v);}
int32_t DecodeBypassCabac(PWelsCabacDecEngine e,uint32_t &v)
{return enabled?bypass(e,v):ReferenceDecodeBypassCabac(e,v);}
int32_t DecodeTerminateCabac(PWelsCabacDecEngine e,uint32_t &v)
{return enabled?terminate(e,v):ReferenceDecodeTerminateCabac(e,v);}

static uint32_t random_word(uint32_t &s){s^=s<<13;s^=s>>17;s^=s<<5;return s;}
bool differential()
{
    uint8_t bytes[264]={};uint32_t seed=0x201cabac;unsigned decisions=0;
    for(unsigned run=0;run<1024;run++){
        for(auto &x:bytes)x=(uint8_t)random_word(seed);
        bytes[0]&=0x7f;
        unsigned length=32+run%224;
        SBitStringAux sa{},sb{};sa.pStartBuf=bytes;sa.pCurBuf=bytes+2;sa.pEndBuf=bytes+length;sb=sa;
        SWelsCabacDecEngine a{},b{};
        if(ReferenceInitCabacDecEngineFromBS(&a,&sa)||initialize(&b,&sb))return false;
        SWelsCabacCtx c{},d{};c.uiState=random_word(seed)%64;c.uiMPS=random_word(seed)&1;d=c;
        for(unsigned step=0;step<128;step++){
            if((a.pBuffCurr-bytes)*8-a.iBitsLeft+48>(int)length*8)break;
            uint32_t x=0xdeadbeef,y=x;int32_t ra,rb;unsigned op=random_word(seed)%3;
            if(op==0){ra=ReferenceDecodeBinCabac(&a,&c,x);rb=bin(&b,&d,y);}
            else if(op==1){ra=ReferenceDecodeBypassCabac(&a,x);rb=bypass(&b,y);}
            else{ra=ReferenceDecodeTerminateCabac(&a,x);rb=terminate(&b,y);}
            decisions++;
            if(ra||rb||x!=y||a.uiRange!=b.uiRange||c.uiState!=d.uiState||c.uiMPS!=d.uiMPS||
                a.iBitsLeft<0||(uint32_t)(a.uiOffset>>a.iBitsLeft)!=value(&b)||
                (a.pBuffCurr-bytes)*8-a.iBitsLeft!=(b.pBuffCurr-bytes)*8-b.iBitsLeft){
                printf("OFT_CABAC_COMPACT_DIFF failed=1 run=%u step=%u op=%u decisions=%u\n",run,step,op,decisions);return false;
            }
            if(op==2&&x)break;
        }
        ReferenceRestoreCabacDecEngineToBS(&a,&sa);ReferenceRestoreCabacDecEngineToBS(&b,&sb);
        if(sa.pCurBuf!=sb.pCurBuf)return false;
    }
    for(unsigned length=0;length<5;length++){
        SBitStringAux bs{};bs.pStartBuf=bytes;bs.pCurBuf=bytes+2;bs.pEndBuf=bytes+length;
        SWelsCabacDecEngine e{};int32_t rc=initialize(&e,&bs);
        if(length<2){if(!rc)return false;continue;}
        if(rc)return false;
        uint32_t bit=0;
        for(unsigned i=0;i<40&&!rc;i++)rc=bypass(&e,bit);
        if(!rc||e.pBuffCurr>e.pBuffEnd)return false;
    }
    printf("OFT_CABAC_COMPACT_DIFF failed=0 runs=1024 decisions=%u short_inputs=5\n",decisions);return true;
}
}
bool oft_cabac32_selftest(){return WelsDec::differential();}
