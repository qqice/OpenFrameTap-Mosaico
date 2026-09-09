#include "esp_heap_caps.h"
extern "C" void *oft_openh264_malloc(size_t size)
{return heap_caps_malloc(size,MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT);}
