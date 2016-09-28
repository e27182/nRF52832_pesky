//#define NRF_LOG_MODULE_NAME "Pesky"
#define MPL_LOG_NDEBUG 1 

#include <math.h>
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_rtc.h"
#include "nrf_drv_gpiote.h"
#include "app_twi.h"
#include "app_error.h"
#include "timestamping.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "data_builder.h"
#include "results_holder.h"
#include "invensense.h"

#define QUAT_W 0
#define QUAT_X 1
#define QUAT_Y 2
#define QUAT_Z 3

#define VEC_X 0
#define VEC_Y 1
#define VEC_Z 2

#define SDA_PIN 6
#define SCL_PIN 7
#define MPU_INT_PIN 10

#define MPU_HZ 100
#define COMPASS_HZ 100
#define USE_DMP 1
#define GYRO_FSR 250
#define ACCEL_FSR 2
//#define PRINT_TIMESTAMP_SENSORS
#define INT_ENABLE 1

#define MAX_PENDING_TRANSACTIONS 5

app_twi_t m_app_twi = APP_TWI_INSTANCE(0);
static nrf_drv_rtc_t const m_rtc = NRF_DRV_RTC_INSTANCE(0);

volatile bool mpu_data_ready = false;

const signed char _orientation[9] = {
    1,  0,  0,
    0,  1,  0,
    0,  0,  1
}; 

// TWI (with transaction manager) initialization.
static void twi_config(void)
{
    uint32_t err_code;

    nrf_drv_twi_config_t const config = {
       .scl                = SCL_PIN,
       .sda                = SDA_PIN,
       .frequency          = NRF_TWI_FREQ_400K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH
    };

    APP_TWI_INIT(&m_app_twi, &config, MAX_PENDING_TRANSACTIONS, err_code);
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief Function that configures GPIOTE to give an interrupt on pin change.
 */
static void gpio_config(void)
{
    if ((NRF_UICR->NFCPINS & UICR_NFCPINS_PROTECT_Msk) == (UICR_NFCPINS_PROTECT_NFC << UICR_NFCPINS_PROTECT_Pos)){
        NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos;
        while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}
        NRF_UICR->NFCPINS &= ~UICR_NFCPINS_PROTECT_Msk;
        while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}
        NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos;
        while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}
        NVIC_SystemReset();
    }

    APP_ERROR_CHECK(nrf_drv_gpiote_init());
}

void dataReadyCallback(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    mpu_data_ready = true;
}

static int mpu_config(void)
{
    int ret;

    struct int_param_s int_param;
    int_param.cb = &dataReadyCallback;
    int_param.pin = MPU_INT_PIN;

    NRF_LOG_INFO("Init MPU\r\n");

    if (0 != (ret = mpu_init(&int_param)))
    {
        NRF_LOG_ERROR("Failed to init mpu: %d\r\n", ret);
        return ret;
    }

    if (0 != (ret = mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS)))
    {
        NRF_LOG_ERROR("Failed to set sensors: %d\r\n", ret);
        return ret;
    }

    if (0 != (ret = mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL)))
    {
        NRF_LOG_ERROR("Failed to set fifo: %d\r\n", ret);
        return ret;
    }

    if (0 != (ret = mpu_set_sample_rate(MPU_HZ))) // Sampling rate must be between 4Hz and 1kHz. LPF will be set to 1/2 of smapling rate.
    {
        NRF_LOG_ERROR("Failed to set sample rate: %d\r\n", ret);
        return ret;
    }
    if (0 != (ret = mpu_set_compass_sample_rate(COMPASS_HZ))) // Sampling rate must be between xxHz and 100Hz.
    {
        NRF_LOG_ERROR("Failed to set compass sample rate: %d\r\n", ret);
        return ret;
    }
    if (0 != (ret = mpu_set_lpf(10))) // The following Low-Pass Filter settings are supported: 188, 98, 42, 20, 10, 5.
    {
        NRF_LOG_ERROR("Failed to set LPF: %d\r\n", ret);
        return ret;
    }

    if (0 != (ret = mpu_set_gyro_fsr(GYRO_FSR))) // The following Gyro Full-Scale Ranges are supported: 250, 500, 1000, 2000.
    {
        NRF_LOG_ERROR("Failed to set Gyro FSR: %d\r\n", ret);
        return ret;
    }
    if (0 != (ret = mpu_set_accel_fsr(ACCEL_FSR))) // The following Accel Full-Scale Ranges are supported: 2, 4, 8, 16.
    {
        NRF_LOG_ERROR("Failed to set Accel FSR: %d\r\n", ret);
        return ret;
    }
    if (0 != (ret = dmp_load_motion_driver_firmware()))
    {
        NRF_LOG_ERROR("Failed to load dmp firmware: %d\r\n", ret);
        return ret;
    }

    if (0 != (ret = dmp_set_orientation(inv_orientation_matrix_to_scalar(_orientation))))
    {
        NRF_LOG_ERROR("Failed to set orientation: %d\r\n", ret);
        return ret;
    }

    if (0 != (ret = dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_GYRO_CAL)))
    {
        NRF_LOG_ERROR("Failed to enable feature: %d\r\n", ret);
        return ret;
    }

    if (0 != (ret = dmp_set_fifo_rate(MPU_HZ)))
    {
        NRF_LOG_ERROR("Failed to set fifo rate: %d\r\n", ret);
        return ret;
    }

    if (0 != (ret = dmp_set_interrupt_mode(DMP_INT_CONTINUOUS)))
    {
        NRF_LOG_ERROR("Failed to set DMP interrupt: %d\r\n", ret);
        return ret;
    }

    if (0 != (ret = mpu_set_dmp_state(USE_DMP)))
    {
        NRF_LOG_ERROR("Failed to set DMP state: %d\r\n", ret);
        return ret;
    }

    return 0;
}

static inline void run_self_test(void)
{
    int result;
    long gyro[3], accel[3];

    NRF_LOG_INFO("Starting self-test\n");

#if defined (MPU6500) || defined (MPU9250)
    result = mpu_run_6500_self_test(gyro, accel, 1);
#elif defined (MPU6050) || defined (MPU9150)
    result = mpu_run_self_test(gyro, accel);
#endif
    if (result == 0x7) {
	    NRF_LOG_INFO("Passed: \r\n");
        NRF_LOG_RAW_INFO("\taccel: "NRF_LOG_FLOAT_MARKER" "NRF_LOG_FLOAT_MARKER" "NRF_LOG_FLOAT_MARKER"\r\n",
                    NRF_LOG_FLOAT(accel[0]/65536.f),
                    NRF_LOG_FLOAT(accel[1]/65536.f),
                    NRF_LOG_FLOAT(accel[2]/65536.f));
        NRF_LOG_RAW_INFO("\tgyro: "NRF_LOG_FLOAT_MARKER" "NRF_LOG_FLOAT_MARKER" "NRF_LOG_FLOAT_MARKER"\r\n",
                    NRF_LOG_FLOAT(gyro[0]/65536.f),
                    NRF_LOG_FLOAT(gyro[1]/65536.f),
                    NRF_LOG_FLOAT(gyro[2]/65536.f));
        /* Test passed. We can trust the gyro data here, so now we need to update calibrated data*/

#ifdef USE_CAL_HW_REGISTERS
        /*
         * This portion of the code uses the HW offset registers that are in the MPUxxxx devices
         * instead of pushing the cal data to the MPL software library
         */
        unsigned char i = 0;

        for(i = 0; i<3; i++) {
        	gyro[i] = (long)(gyro[i] * 32.8f); //convert to +-1000dps
        	accel[i] *= 2048.f; //convert to +-16G
        	accel[i] = accel[i] >> 16;
        	gyro[i] = (long)(gyro[i] >> 16);
        }

        mpu_set_gyro_bias_reg(gyro);

#if defined (MPU6500) || defined (MPU9250)
        mpu_set_accel_bias_6500_reg(accel);
#elif defined (MPU6050) || defined (MPU9150)
        mpu_set_accel_bias_6050_reg(accel);
#endif
#endif
    } else {
        if (!(result & 0x1))
            NRF_LOG_ERROR("Gyro failed.\r\n");
        if (!(result & 0x2))
            NRF_LOG_ERROR("Accel failed.\r\n");
        if (!(result & 0x4))
            NRF_LOG_ERROR("Compass failed.\r\n");
    }
}

// RTC tick events generation.
static void rtc_handler(nrf_drv_rtc_int_type_t int_type)
{
}
static void rtc_config(void)
{
    uint32_t err_code;

    // Initialize RTC instance with default configuration.
    nrf_drv_rtc_config_t config = NRF_DRV_RTC_DEFAULT_CONFIG;
    err_code = nrf_drv_rtc_init(&m_rtc, &config, rtc_handler);
    APP_ERROR_CHECK(err_code);

    // Enable tick event and interrupt.
    //nrf_drv_rtc_tick_enable(&m_rtc, true);

    // Power on RTC instance.
    nrf_drv_rtc_enable(&m_rtc);
}

int main(void)
{
    // Start internal LFCLK XTAL oscillator - it is needed for "read" ticks generation (by RTC).
    lfclk_config();
    rtc_config();
    
    APP_ERROR_CHECK(NRF_LOG_INIT(timestamp_func));

    NRF_LOG_INFO("MPU9250 example\r\n");

    gpio_config();
    twi_config();
    APP_ERROR_CHECK(mpu_config());

    run_self_test();

    nrf_gpio_range_cfg_output(22, 24);
    while (true)
    {
        while (NRF_LOG_PROCESS()) ;

        nrf_gpio_pin_clear(22); // Pin low when CPU is sleeping
        while(mpu_data_ready != true)
        {
            // Make sure any pending events are cleared
            __SEV();
            __WFE();
            // Enter System ON sleep mode
            __WFE();
        }
        nrf_gpio_pin_set(22); // Pin high when CPU is working

        short gyro[3];
        short accel[3];
        unsigned long timestamp;
        unsigned char more;
        int ret;

        if (USE_DMP == 1)
        {
            long quat[4];
            float quaternion[4];
            short sensors;

            do
            {
                if (0 == (ret = dmp_read_fifo(gyro, accel, quat, &timestamp, &sensors, &more)))
                {
                    if (INV_WXYZ_QUAT == (sensors & INV_WXYZ_QUAT))
                    {
                        quaternion[QUAT_W] = (float)quat[QUAT_W];
                        quaternion[QUAT_X] = (float)quat[QUAT_X];
                        quaternion[QUAT_Y] = (float)quat[QUAT_Y];
                        quaternion[QUAT_Z] = (float)quat[QUAT_Z];
                        inv_q_norm4(quaternion);

                        NRF_LOG_INFO("Q: " NRF_LOG_FLOAT_MARKER " " NRF_LOG_FLOAT_MARKER " " NRF_LOG_FLOAT_MARKER " ",
                            NRF_LOG_FLOAT(quaternion[QUAT_W]),
                            NRF_LOG_FLOAT(quaternion[QUAT_X]),
                            NRF_LOG_FLOAT(quaternion[QUAT_Y]));
                        
                        NRF_LOG_RAW_INFO(NRF_LOG_FLOAT_MARKER "\r\n",
                            NRF_LOG_FLOAT(quaternion[QUAT_Z]));
                    }

                    if (INV_XYZ_GYRO == (sensors & INV_XYZ_GYRO))
                        NRF_LOG_INFO("G: %d %d %d\r\n", gyro[VEC_X], gyro[VEC_Y], gyro[VEC_Z]);

                    if (INV_XYZ_ACCEL == (sensors & INV_XYZ_ACCEL))
                        NRF_LOG_INFO("A: %d %d %d\r\n", accel[VEC_X], accel[VEC_Y], accel[VEC_Z]);

#ifdef PRINT_TIMESTAMP_SENSORS
                    NRF_LOG_INFO("T: %d\tS: %d\r\n", timestamp, sensors);
#endif
                }
                // else
                //     NRF_LOG_ERROR("%d\r\n", ret); // NOTE: frequently called when there is not enough data in fifo yet
            }
            while (more > 0);
        }
        else
        {
            unsigned char sensors;

            do
            {
                if (0 == (ret = mpu_read_fifo(gyro, accel, &timestamp, &sensors, &more)))
                {
                    NRF_LOG_INFO("G: %d %d %d\r\n", gyro[VEC_X], gyro[VEC_Y], gyro[VEC_Z]);
                    NRF_LOG_INFO("A: %d %d %d\r\n", accel[VEC_X], accel[VEC_Y], accel[VEC_Z]);

#ifdef PRINT_TIMESTAMP_SENSORS
                    NRF_LOG_INFO("T: %d\tS: %d\r\n", timestamp, sensors);
#endif
                }
                // else
                //     NRF_LOG_ERROR("%d\r\n", ret);
            }
            while (more > 0);
        }

        mpu_data_ready = false;
    }
}


/** @} */
