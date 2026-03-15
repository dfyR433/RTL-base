#ifndef MONITOR_H
#define MONITOR_H

#include <stdint.h>
#include <stddef.h>

#ifndef WLAN_IDX
#define WLAN_IDX                STA_WLAN_INDEX
#endif

static const uint8_t CHANNEL_LIST[] = {
    1,2,3,4,5,6,7,8,9,10,11,12,13,
    36,40,44,48,52,56,60,64,100,104,108,112,116,120,124,128,132,136,140,144,149,153,157,161,165
};
#define CHANNEL_LIST_LEN        (sizeof(CHANNEL_LIST)/sizeof(CHANNEL_LIST[0]))

#define HOP_INTERVAL_MS         100

#define MONITOR_TASK_STACK      4096
#define HOP_TASK_STACK          1024
#define MONITOR_TASK_PRIO       (tskIDLE_PRIORITY + 3)
#define HOP_TASK_PRIO           (tskIDLE_PRIORITY + 1)

#define RING_SIZE               1024
#define PACKET_POOL_SIZE        64
#define PACKET_BUFFER_SIZE      2346

#define TX_PIN                   _PB_5
#define RX_PIN                   _PB_4
#define BAUD_RATE                2000000U

#ifndef USE_DWT_CYCCNT
#define USE_DWT_CYCCNT
#endif

#define IDB_FCS_LEN              4
#define IDB_TSRESOL              9

void monitor_start(void);

#endif
