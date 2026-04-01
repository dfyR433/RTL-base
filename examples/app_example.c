#include "inject.h"
#include "monitor.h"

static injectorManager *g_mgr = NULL;

void app_main(void) {
    // --- Setup injector ---
    injector_set_timer_freq_hz(80000000UL);          // 80 MHz hardware timer
    g_mgr = injectorManager_create();
    injectorManager_setWlanIndex(g_mgr, 0);

    uint8_t beacon[] = { /* 802.11 beacon frame */ };
    injectorManager_setInjector(g_mgr, "beacon", beacon, sizeof(beacon),
                                6,       // channel
                                102400,  // interval (µs) → auto‑converted to ns
                                0,       // unlimited packets
                                3);      // hw retries

    // --- Setup monitor ---
    monitor_set_filter(FILTER_BEACON);
    monitor_set_rssi_threshold(-70);
    monitor_set_fixed_channel(6);           // match injector channel

    // --- Start both ---
    injectorManager_startSchedulerTask(g_mgr, 5);
    injectorManager_activateInjector(g_mgr, "beacon");

    monitor_start();                        // begins capturing and streaming

    // --- Application loop ---
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
        // Print stats (injector & monitor) via your own debug channel
        InjectorInfo info;
        injectorManager_getInfo(g_mgr, "beacon", &info);
        printf("injector: sent=%u errors=%u retries=%u\n",
               info.packets_sent, info.tx_errors, info.tx_retries);

        uint32_t cap, dr, dp, pk;
        monitor_get_stats(&cap, &dr, &dp, &pk);
        printf("monitor: captured=%u ring_drops=%u pool_drops=%u peak_pool=%u\n",
               cap, dr, dp, pk);
    }

    // --- Shutdown (not reached in this example) ---
    monitor_stop();
    injectorManager_stopSchedulerTask(g_mgr);
    injectorManager_destroy(g_mgr);
}
