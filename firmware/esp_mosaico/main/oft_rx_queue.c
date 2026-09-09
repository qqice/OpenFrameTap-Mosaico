/* Read-only queue diagnostics through project linker interposition. No IDF
   source modification: every original operation and return value is preserved.
   This app has exactly one UDP socket; DHCP uses lwIP's raw API. */
#include "oft_rx_queue.h"
#include "lwip/sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <stdatomic.h>
#include <stdint.h>

static atomic_uintptr_t observed;
static atomic_uint posts,drops,high_water;
err_t __real_sys_mbox_new(sys_mbox_t *mbox,int size);
err_t __real_sys_mbox_trypost(sys_mbox_t *mbox,void *message);
void __real_sys_mbox_free(sys_mbox_t *mbox);
err_t __wrap_sys_mbox_new(sys_mbox_t *mbox,int size)
{
    err_t rc=__real_sys_mbox_new(mbox,size);
    if(rc==ERR_OK&&size==CONFIG_LWIP_UDP_RECVMBOX_SIZE){
        atomic_store(&posts,0);atomic_store(&drops,0);atomic_store(&high_water,0);
        atomic_store(&observed,(uintptr_t)mbox);
    }
    return rc;
}
err_t __wrap_sys_mbox_trypost(sys_mbox_t *mbox,void *message)
{
    err_t rc=__real_sys_mbox_trypost(mbox,message);
    if((uintptr_t)mbox==atomic_load(&observed)){
        atomic_fetch_add(&posts,1);if(rc!=ERR_OK)atomic_fetch_add(&drops,1);
        unsigned used=(unsigned)uxQueueMessagesWaiting((QueueHandle_t)&(*mbox)->os_mbox);
        unsigned previous=atomic_load(&high_water);
        while(used>previous&&!atomic_compare_exchange_weak(&high_water,&previous,used)){}
    }
    return rc;
}
void __wrap_sys_mbox_free(sys_mbox_t *mbox)
{
    uintptr_t expected=(uintptr_t)mbox;
    atomic_compare_exchange_strong(&observed,&expected,0);
    __real_sys_mbox_free(mbox);
}
oft_rx_queue_stats_t oft_rx_queue_stats(void)
{
    return (oft_rx_queue_stats_t){atomic_load(&posts),atomic_load(&drops),atomic_load(&high_water),CONFIG_LWIP_UDP_RECVMBOX_SIZE};
}
