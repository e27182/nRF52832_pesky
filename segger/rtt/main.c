#include "nrf_log_ctrl.h"
#include "nrf_log.h"
#include "nrf_delay.h"

#include "app_timer.h"
#include "nrf_drv_clock.h"

#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE         4                                           /**< Size of timer operation queues. */
#define APP_TIMER_MS(TICKS, PRESCALER)\
            ((uint32_t)ROUNDED_DIV((TICKS) * ((PRESCALER) + 1) * 1000, (uint64_t)APP_TIMER_CLOCK_FREQ))

char p_data[] = "data";
int length = 4;
float val = 0.214;
char string_on_stack[] = "Testing string on stack\r\n";

uint32_t ticks_from = 0;
uint32_t timestamp_ms = 0;

uint32_t timestamp_func(void) {
    uint32_t ticks_diff = 0;
    uint32_t ms_diff = 0;
    uint32_t ticks_to = app_timer_cnt_get();

    APP_ERROR_CHECK(app_timer_cnt_diff_compute(ticks_to, ticks_from, &ticks_diff));
    ticks_from = ticks_to;

    ms_diff = APP_TIMER_MS(ticks_diff, APP_TIMER_PRESCALER);

    timestamp_ms += ms_diff;

    return timestamp_ms; 
}

// Function starting the internal LFCLK oscillator.
// This is needed by RTC1 which is used by the application timer
// (When SoftDevice is enabled the LFCLK is always running and this is not needed).
static void lfclk_request(void)
{
    uint32_t err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);
    nrf_drv_clock_lfclk_request(NULL);
}

static void timer_handler(void * p_context)
{
    NRF_LOG_INFO("Tetsing timer...\r\n");
}

int main(void)
{
    lfclk_request();
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);
    APP_TIMER_DEF(m_timer_id);
    APP_ERROR_CHECK(app_timer_create(&m_timer_id, APP_TIMER_MODE_REPEATED, timer_handler));
    APP_ERROR_CHECK(app_timer_start(m_timer_id, APP_TIMER_TICKS(10000, APP_TIMER_PRESCALER), NULL));

    APP_ERROR_CHECK(NRF_LOG_INIT(timestamp_func));

    while (true)
    {
        // do
        NRF_LOG_DEBUG("Tetsing debug...\r\n");
        NRF_LOG_INFO("Tetsing info...\r\n");
        NRF_LOG_WARNING("Tetsing warning...\r\n");
        NRF_LOG_ERROR("Testing error...\r\n");

        NRF_LOG_INFO("Testing %d bytes, data:\r\n", length);
        NRF_LOG_HEXDUMP_INFO(p_data, length);

        NRF_LOG_INFO("Testing float value:" NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(val));

        // nrf_log_push() copies the string into the logger buffer and returns address from the logger buffer
        NRF_LOG_INFO("%s", nrf_log_push(string_on_stack));

        while (NRF_LOG_PROCESS()) ;

        nrf_delay_ms(1000);
    }
}
