#pragma once
#include <stdint.h>
enum { OFT_H264_CABAC, OFT_H264_CAVLC, OFT_H264_INTRA, OFT_H264_DEBLOCK, OFT_H264_PHASES };
struct OftH264Profile { uint64_t us[OFT_H264_PHASES];unsigned calls[OFT_H264_PHASES]; };
void oft_h264_profile_begin(bool enabled);
OftH264Profile oft_h264_profile_end();
int64_t oft_h264_profile_enter(unsigned phase);
void oft_h264_profile_leave(unsigned phase,int64_t start);
class OftH264Scope {
    unsigned phase;int64_t start;
public:
    explicit OftH264Scope(unsigned p):phase(p),start(oft_h264_profile_enter(p)){}
    ~OftH264Scope(){oft_h264_profile_leave(phase,start);}
    OftH264Scope(const OftH264Scope&)=delete;
};
