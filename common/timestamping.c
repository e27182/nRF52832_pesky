#include "timestamping.h"
#include "nrf_drv_clock.h"
#include "app_timer.h"

void lfclk_config(void)
{
    uint32_t err_code;

    err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_clock_lfclk_request(NULL);
}

uint32_t ticks_from = 0;
uint32_t timestamp_ms = 0;

uint32_t timestamp_func(void) {
    uint32_t ticks_diff = 0;
    uint32_t ms_diff = 0;
    uint32_t ticks_to = NRF_RTC0->COUNTER; //app_timer_cnt_get();

    APP_ERROR_CHECK(app_timer_cnt_diff_compute(ticks_to, ticks_from, &ticks_diff));
    ticks_from = ticks_to;

    ms_diff = APP_TIMER_MS(ticks_diff, APP_TIMER_PRESCALER);

    timestamp_ms += ms_diff;

    return timestamp_ms; 
}