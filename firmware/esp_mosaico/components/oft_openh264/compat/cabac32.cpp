/*
 * Copyright (c) 2013, Cisco Systems
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
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
 * Experimental RV32-oriented adaptation of OpenH264's CABAC arithmetic core.
 * Keep the nine-bit range in32bits; specialize the common32-bit refill, avoiding
 * a variable64-bit shift and separate helper call. Offset/reservoir ABI remains
 * unchanged, so the existing syntax/reconstruction implementation is the oracle.
 * Not a standalone H.264 decoder and not Espressif's closed TinyH264 backend.
 */
#include "cabac32.h"
#include "cabac_decoder.h"
#include <cstring>
#include <cstdio>

static bool enabled;
void oft_cabac32_select(bool on){enabled=on;} // Single decoder owner only.

namespace WelsDec {
int32_t ReferenceDecodeBinCabac(PWelsCabacDecEngine,PWelsCabacCtx,uint32_t&);
int32_t ReferenceDecodeBypassCabac(PWelsCabacDecEngine,uint32_t&);

static inline int32_t refill(PWelsCabacDecEngine e,uint64_t &offset,int32_t &bits)
{
    if(e->pBuffEnd-e->pBuffCurr>=4){
        const uint8_t *p=e->pBuffCurr;
        uint32_t next=((uint32_t)p[0]<<24)|((uint32_t)p[1]<<16)|((uint32_t)p[2]<<8)|p[3];
        e->pBuffCurr+=4;bits=32;
        offset=((uint64_t)(uint32_t)offset<<32)|next;
        return ERR_NONE;
    }
    uint32_t value=0;int32_t rc=Read32BitsCabac(e,value,bits);
    offset=(offset<<bits)|value;return rc;
}

static int32_t bin32(PWelsCabacDecEngine e,PWelsCabacCtx ctx,uint32_t &out)
{
    // Invalid/unusual engine states retain the upstream error behavior.
    if(e->uiRange>510||e->uiRange<2||e->iBitsLeft<0||e->iBitsLeft>32)
        return ReferenceDecodeBinCabac(e,ctx,out);
    uint32_t state=ctx->uiState,range=(uint32_t)e->uiRange;
    out=ctx->uiMPS;uint64_t offset=e->uiOffset;
    uint32_t lps=g_kuiCabacRangeLps[state][(range>>6)&3];range-=lps;
    uint64_t boundary=(uint64_t)range<<e->iBitsLeft;
    int32_t renorm=1;
    if(offset>=boundary){
        offset-=boundary;out^=1;
        if(!state)ctx->uiMPS^=1;
        ctx->uiState=g_kuiStateTransTable[state][0];
        renorm=g_kRenormTable256[lps];range=lps<<renorm;
    }else{
        ctx->uiState=g_kuiStateTransTable[state][1];
        if(range>=WELS_CABAC_QUARTER){e->uiRange=range;return ERR_NONE;}
        range<<=1;
    }
    e->uiRange=range;e->iBitsLeft-=renorm;
    if(e->iBitsLeft>0){e->uiOffset=offset;return ERR_NONE;}
    int32_t count=0,rc=refill(e,offset,count);
    e->uiOffset=offset;e->iBitsLeft+=count;
    return rc&&e->iBitsLeft<0?rc:(int32_t)ERR_NONE;
}
static int32_t bypass32(PWelsCabacDecEngine e,uint32_t &out)
{
    if(e->uiRange>510||e->iBitsLeft>32)return ReferenceDecodeBypassCabac(e,out);
    int32_t bits=e->iBitsLeft;uint64_t offset=e->uiOffset;
    if(bits<=0){
        int32_t rc=refill(e,offset,bits);
        if(rc&&!bits)return rc;
    }
    --bits;uint64_t boundary=(uint64_t)(uint32_t)e->uiRange<<bits;
    out=offset>=boundary;if(out)offset-=boundary;
    e->iBitsLeft=bits;e->uiOffset=offset;return ERR_NONE;
}
int32_t DecodeBinCabac(PWelsCabacDecEngine e,PWelsCabacCtx c,uint32_t &b)
{return enabled?bin32(e,c,b):ReferenceDecodeBinCabac(e,c,b);}
int32_t DecodeBypassCabac(PWelsCabacDecEngine e,uint32_t &b)
{return enabled?bypass32(e,b):ReferenceDecodeBypassCabac(e,b);}

static uint32_t random_word(uint32_t &s){s^=s<<13;s^=s>>17;s^=s<<5;return s;}
bool differential()
{
    uint8_t bytes[256];uint32_t seed=0x512039ab;unsigned decisions=0;
    for(unsigned run=0;run<1024;run++){
        for(auto &b:bytes)b=(uint8_t)random_word(seed);
        SWelsCabacDecEngine a{},b{};
        a.uiRange=256+random_word(seed)%255;a.iBitsLeft=1+random_word(seed)%31;
        a.uiOffset=((uint64_t)(random_word(seed)%(uint32_t)a.uiRange)<<a.iBitsLeft)|
            (random_word(seed)&((1u<<a.iBitsLeft)-1));
        a.pBuffStart=a.pBuffCurr=bytes;a.pBuffEnd=bytes+(run%sizeof(bytes));b=a;
        SWels_Cabac_Element c{},d{};c.uiState=random_word(seed)%64;c.uiMPS=random_word(seed)&1;d=c;
        for(unsigned step=0;step<128;step++){
            uint32_t x=0xdeadbeef,y=x;int32_t ra,rb;
            if(random_word(seed)&1){ra=ReferenceDecodeBinCabac(&a,&c,x);rb=bin32(&b,&d,y);}
            else{ra=ReferenceDecodeBypassCabac(&a,x);rb=bypass32(&b,y);}
            decisions++;
            if(ra!=rb||x!=y||c.uiState!=d.uiState||c.uiMPS!=d.uiMPS||
                a.uiRange!=b.uiRange||a.uiOffset!=b.uiOffset||a.iBitsLeft!=b.iBitsLeft||
                a.pBuffCurr!=b.pBuffCurr){printf("OFT_CABAC32_DIFF failed=1 run=%u step=%u decisions=%u\n",run,step,decisions);return false;}
            if(ra)break;
        }
    }
    printf("OFT_CABAC32_DIFF failed=0 runs=1024 decisions=%u\n",decisions);return true;
}
}
bool oft_cabac32_selftest(){return WelsDec::differential();}
