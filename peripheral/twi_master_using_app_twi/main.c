/* Copyright (c) 2015 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 * @defgroup nrf_twi_master_example main.c
 * @{
 * @ingroup nrf_twi_example
 * @brief TWI Example Application main file.
 *
 * This file contains the source code for a sample application using TWI.
 */

#include "boards.h"
#include "nrf_drv_rtc.h"
#include "nrf_drv_clock.h"
#include "bsp.h"
#include "app_twi.h"
#include "nrf_delay.h"
#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"


#define MAX_PENDING_TRANSACTIONS    5

#define MPU_PWR_MGMT_1_REG          0x6B
#define MPU_WHO_AM_I_REG            0x75
#define MPU_I2C_ADDRESS             0x68
#define BIT_RESET                   0x80

// Pin number for indicating communication with sensors.
#ifdef BSP_LED_2
    #define READ_ALL_INDICATOR  BSP_LED_2
#else
    #error "Please choose an output pin"
#endif


static app_twi_t m_app_twi = APP_TWI_INSTANCE(0);

static nrf_drv_rtc_t const m_rtc = NRF_DRV_RTC_INSTANCE(0);


uint8_t const mpu_who_am_i_reg_addr = MPU_WHO_AM_I_REG;

#define BUFFER_SIZE  5
static uint8_t m_buffer[BUFFER_SIZE] = { MPU_PWR_MGMT_1_REG, BIT_RESET, MPU_PWR_MGMT_1_REG, 0x00, 0x00 };

static app_twi_transfer_t const mpu_init_transfers[] = 
{
    APP_TWI_WRITE(MPU_I2C_ADDRESS, &m_buffer[0], 2, 0),
    APP_TWI_WRITE(MPU_I2C_ADDRESS, &m_buffer[2], 2, 0),
};

void read_cb(ret_code_t result, void * p_user_data)
{
    if (result != NRF_SUCCESS)
    {
        NRF_LOG_INFO("read_cb - error: %d\r\n", (int)result);
        return;
    }

    uint8_t whoami = m_buffer[4];
    NRF_LOG_INFO("WHOAMI: %d\r\n", (int)whoami);
}

static void read(void)
{

    // Signal on LED that something is going on.
    nrf_gpio_pin_toggle(READ_ALL_INDICATOR);

    // [these structures have to be "static" - they cannot be placed on stack
    //  since the transaction is scheduled and these structures most likely
    //  will be referred after this function returns]
    static app_twi_transfer_t const transfers[] =
    {
        APP_TWI_WRITE(MPU_I2C_ADDRESS, &mpu_who_am_i_reg_addr, 1, APP_TWI_NO_STOP)
        ,
        APP_TWI_READ (MPU_I2C_ADDRESS, &m_buffer[4], 1, 0)
    };
    static app_twi_transaction_t const transaction =
    {
        .callback            = read_cb,
        .p_user_data         = NULL,
        .p_transfers         = transfers,
        .number_of_transfers = sizeof(transfers) / sizeof(transfers[0])
    };

    APP_ERROR_CHECK(app_twi_schedule(&m_app_twi, &transaction));
}

// TWI (with transaction manager) initialization.
static void twi_config(void)
{
    uint32_t err_code;

    nrf_drv_twi_config_t const config = {
       .scl                = ARDUINO_SCL_PIN,
       .sda                = ARDUINO_SDA_PIN,
       .frequency          = NRF_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_LOW,
       .clear_bus_init     = false
    };

    APP_TWI_INIT(&m_app_twi, &config, MAX_PENDING_TRANSACTIONS, err_code);
    APP_ERROR_CHECK(err_code);
}


// RTC tick events generation.
static void rtc_handler(nrf_drv_rtc_int_type_t int_type)
{
    if (int_type == NRF_DRV_RTC_INT_TICK)
    {
        // On each RTC tick (their frequency is set in "nrf_drv_config.h")
        // we read data from our sensor.
        read();
    }
}
static void rtc_config(void)
{
    uint32_t err_code;

    // Initialize RTC instance with default configuration.
    nrf_drv_rtc_config_t config = NRF_DRV_RTC_DEFAULT_CONFIG;
    config.prescaler = RTC_FREQ_TO_PRESCALER(32); //Set RTC frequency to 32Hz
    err_code = nrf_drv_rtc_init(&m_rtc, &config, rtc_handler);
    APP_ERROR_CHECK(err_code);

    // Enable tick event and interrupt.
    nrf_drv_rtc_tick_enable(&m_rtc, true);

    // Power on RTC instance.
    nrf_drv_rtc_enable(&m_rtc);
}


static void lfclk_config(void)
{
    uint32_t err_code;

    err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_clock_lfclk_request(NULL);
}


int main(void)
{
    LEDS_CONFIGURE(1U << READ_ALL_INDICATOR);
    LEDS_OFF(1U << READ_ALL_INDICATOR);

    // Start internal LFCLK XTAL oscillator - it is needed
    // for "read" ticks generation
    // (by RTC).
    lfclk_config();

    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));

    NRF_LOG_INFO("TWI master example\r\n");
    NRF_LOG_FLUSH();
    twi_config();

    // Initialize sensors.
    APP_ERROR_CHECK(app_twi_perform(&m_app_twi, &mpu_init_transfers[0], 1, NULL));
    nrf_delay_ms(100);
    APP_ERROR_CHECK(app_twi_perform(&m_app_twi, &mpu_init_transfers[1], 1, NULL));

    rtc_config();

    while (true)
    {
        __WFI();
        NRF_LOG_FLUSH();
    }
}


/** @} */
