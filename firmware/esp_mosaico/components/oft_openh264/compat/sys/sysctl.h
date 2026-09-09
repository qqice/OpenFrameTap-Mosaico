#pragma once
#include <stddef.h>
#include <string.h>
/* The only sysctl used by upstream's generic POSIX path asks CPU count.
   Decoder thread count is explicitly0: execute synchronously in our worker. */
static inline int sysctlbyname(const char *name,void *value,size_t *size,void *new_value,size_t new_size)
{
    (void)new_value;(void)new_size;
    if(!name||strcmp(name,"hw.ncpu")||!value||!size||*size<sizeof(int))return -1;
    *(int*)value=1;*size=sizeof(int);return 0;
}
