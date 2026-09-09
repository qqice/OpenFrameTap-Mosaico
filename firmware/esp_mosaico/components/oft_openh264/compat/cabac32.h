#pragma once
#ifdef OFT_CABAC32
void oft_cabac32_select(bool enable);
bool oft_cabac32_selftest();
#else
inline void oft_cabac32_select(bool) {}
#endif
