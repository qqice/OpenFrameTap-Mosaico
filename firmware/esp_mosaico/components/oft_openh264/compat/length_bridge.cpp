#include "decoder_core.h"
/* Upstream declares these lengths as int32_t but defines them as int. On the
   ESP32-S31 newlib ABI int32_t is long, both32-bit; bridge the C++ overloads
   without changing the pinned source or truncating a value. */
static_assert(sizeof(int32_t)==sizeof(int),"Expected32-bit length ABI");
namespace WelsDec {
int32_t ExpandBsBuffer(PWelsDecoderContext,const int);
int32_t ExpandBsLenBuffer(PWelsDecoderContext,const int);
int32_t ExpandBsBuffer(PWelsDecoderContext p,const int32_t n){return ExpandBsBuffer(p,static_cast<int>(n));}
int32_t ExpandBsLenBuffer(PWelsDecoderContext p,const int32_t n){return ExpandBsLenBuffer(p,static_cast<int>(n));}
}
