#include "nrf_log_ctrl.h"
#include "nrf_log.h"
#include "nrf_delay.h"

char p_data[] = "data";
int length = 4;
float val = 0.214;
char string_on_stack[] = "Testing string on stack\r\n";

uint32_t timestamp_func(void)
{
    return 0;
}

int main(void)
{
    uint32_t err_code;
    err_code = NRF_LOG_INIT(timestamp_func);

    APP_ERROR_CHECK(err_code);

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
