#pragma once
#include <stdlib.h>
extern "C" void *oft_openh264_malloc(size_t size);
/* Applied ONLY to upstream memory_align.cpp, not the app/network allocator. */
#define malloc oft_openh264_malloc
