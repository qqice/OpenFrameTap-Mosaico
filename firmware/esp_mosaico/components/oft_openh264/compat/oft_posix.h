#pragma once
#include <pthread.h>
#include <errno.h>
/* The wrapper selects zero decoder threads. These POSIX attribute APIs are
   absent from ESP-IDF; reject them rather than claiming SCHED_FIFO support. */
static inline int oft_setscope(pthread_attr_t *,int){return ENOTSUP;}
static inline int oft_setschedpolicy(pthread_attr_t *,int){return ENOTSUP;}
#define pthread_attr_setscope oft_setscope
#define pthread_attr_setschedpolicy oft_setschedpolicy
