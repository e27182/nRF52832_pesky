#ifndef _INV_PESKY_
#define _INV_PESKY_

#include "timestamping.h"
#include "nrf_delay.h"
#include "app_twi.h"
#include "app_error.h"
#include "nrf_drv_gpiote.h"

#ifdef MPU_LOG_RTTT
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#define log_i NRF_LOG_INFO
#define log_e NRF_LOG_ERROR
#else
#include "log.h"

#define log_i MPL_LOGI
#define log_e MPL_LOGE
#endif

extern app_twi_t m_app_twi;

inline void get_ms(long unsigned int *timestamp)
{
    *timestamp = timestamp_func();
}

#define delay_ms nrf_delay_ms

#define min(a,b) ((a<b)?a:b)

static inline int i2c_write(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char const *data) {
    unsigned char new_data[length + 1];
    memcpy(new_data + 1, data, length);
    new_data[0] = reg_addr;

    app_twi_transfer_t const transfers[] = 
    {
        APP_TWI_WRITE(slave_addr, new_data, length + 1, 0)
    };

    ret_code_t err_code = app_twi_perform(&m_app_twi, transfers, sizeof(transfers) / sizeof(transfers[0]), NULL);
    APP_ERROR_CHECK(err_code);
    
    return 0;
}

static inline int i2c_read(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data) {
    if (length == 0)
        return NRF_SUCCESS;

    app_twi_transfer_t const transfers[] =
    {
        APP_TWI_WRITE(slave_addr, &reg_addr, 1, APP_TWI_NO_STOP),
        APP_TWI_READ (slave_addr, data, length, 0)
    };

    /*ret_code_t err_code = */app_twi_perform(&m_app_twi, transfers, sizeof(transfers) / sizeof(transfers[0]), NULL);
    //APP_ERROR_CHECK(err_code);

    return 0;
}

static inline int reg_int_cb(struct int_param_s *int_param)
{
    nrf_drv_gpiote_in_config_t config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(true); // true - high accurracy
    //config.pull = NRF_GPIO_PIN_PULLUP;

    APP_ERROR_CHECK(nrf_drv_gpiote_in_init(int_param->pin, &config, int_param->cb));
    nrf_drv_gpiote_in_event_enable(int_param->pin, true);
    
    return 0;
}

#define __no_operation __NOP

#endif // _INV_PESKY_