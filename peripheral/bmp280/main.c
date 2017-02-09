#include "boards.h"
#include "nrf_drv_rtc.h"
#include "nrf_drv_clock.h"
#include "bsp.h"
#include "app_twi.h"
#include "nrf_delay.h"
#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "bmp280.h"

#define MAX_PENDING_TRANSACTIONS    8
#define	I2C_BUFFER_LEN 24
#define BMP280_DATA_INDEX	1

// Pin number for indicating communication with sensors.
#ifdef BSP_LED_2
    #define READ_ALL_INDICATOR  BSP_LED_2
#else
    #error "Please choose an output pin"
#endif

static app_twi_t m_app_twi = APP_TWI_INSTANCE(0);

static nrf_drv_rtc_t const m_rtc = NRF_DRV_RTC_INSTANCE(0);

struct bmp280_t bmp280;

static void read(void)
{
    // Signal on LED that something is going on.
    nrf_gpio_pin_toggle(READ_ALL_INDICATOR);
    
    s32 com_rslt = ERROR;
    u32 v_actual_press_combined_u32 = BMP280_INIT_VALUE;
    s32 v_actual_temp_combined_s32 = BMP280_INIT_VALUE;

    com_rslt = bmp280_read_pressure_temperature(&v_actual_press_combined_u32, &v_actual_temp_combined_s32);
    APP_ERROR_CHECK(com_rslt);

    NRF_LOG_RAW_INFO("P, %d, T, %d\r\n", v_actual_press_combined_u32, v_actual_temp_combined_s32);
}

// TWI (with transaction manager) initialization.
static void twi_config(void)
{
    uint32_t err_code;

    nrf_drv_twi_config_t const config = {
       .scl                = ARDUINO_SCL_PIN,
       .sda                = ARDUINO_SDA_PIN,
       .frequency          = NRF_TWI_FREQ_400K,
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
    config.prescaler = RTC_FREQ_TO_PRESCALER(27);
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

s8 BMP280_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	s32 iError = BMP280_INIT_VALUE;
	u8 array[I2C_BUFFER_LEN];
	u8 stringpos = BMP280_INIT_VALUE;
	array[BMP280_INIT_VALUE] = reg_addr;
	for (stringpos = BMP280_INIT_VALUE; stringpos < cnt; stringpos++) {
		array[stringpos + BMP280_DATA_INDEX] = *(reg_data + stringpos);
	}

    app_twi_transfer_t const transfers[] = 
    {
        APP_TWI_WRITE(dev_addr, array, cnt + 1, 0)
    };

    iError = app_twi_perform(&m_app_twi, transfers, sizeof(transfers) / sizeof(transfers[0]), NULL);
    //APP_ERROR_CHECK(err_code);
    // if (reg_addr < 0xF7) {
    //     NRF_LOG_INFO("W %X %X %X %X\r\n", dev_addr, reg_addr, cnt, iError);
    //     NRF_LOG_HEXDUMP_INFO(reg_data, cnt);
    //     //NRF_LOG_FLUSH();
    // }

	return (s8)iError;
}

s8 BMP280_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	s32 iError = BMP280_INIT_VALUE;
	u8 array[I2C_BUFFER_LEN] = {BMP280_INIT_VALUE};
	u8 stringpos = BMP280_INIT_VALUE;
	array[BMP280_INIT_VALUE] = reg_addr;
	
    app_twi_transfer_t const transfers[] =
    {
        APP_TWI_WRITE(dev_addr, &reg_addr, 1, APP_TWI_NO_STOP),
        APP_TWI_READ (dev_addr, array, cnt, 0)
    };

    iError = app_twi_perform(&m_app_twi, transfers, sizeof(transfers) / sizeof(transfers[0]), NULL);
    //APP_ERROR_CHECK(err_code);
    // if (reg_addr < 0xF7) {
    //     NRF_LOG_INFO("R %X %X %X %X\r\n", dev_addr, reg_addr, cnt, iError);
    //     NRF_LOG_HEXDUMP_INFO(array, cnt);
    //     //NRF_LOG_FLUSH();
    // }

	for (stringpos = BMP280_INIT_VALUE; stringpos < cnt; stringpos++) {
		*(reg_data + stringpos) = array[stringpos];
	}

	return (s8)iError;
}

void BMP280_delay_msek(u32 msek)
{
	nrf_delay_ms(msek);
}

static void bmp280_config(void)
{
	bmp280.bus_write = BMP280_I2C_bus_write;
	bmp280.bus_read = BMP280_I2C_bus_read;
	bmp280.dev_addr = BMP280_I2C_ADDRESS2;
	bmp280.delay_msec = BMP280_delay_msek;

    s32 com_rslt = ERROR;
    com_rslt = bmp280_init(&bmp280);
    com_rslt += bmp280_set_power_mode(BMP280_SLEEP_MODE);
    com_rslt += bmp280_set_work_mode(BMP280_ULTRA_HIGH_RESOLUTION_MODE);
    com_rslt += bmp280_set_filter(BMP280_FILTER_COEFF_16);
    com_rslt += bmp280_set_standby_durn(BMP280_STANDBY_TIME_1_MS);
    com_rslt += bmp280_set_power_mode(BMP280_NORMAL_MODE);
    APP_ERROR_CHECK(com_rslt);

    // u8 data[4];
    // bmp280.bus_read(bmp280.dev_addr, 0xF2, data, 4);
    // NRF_LOG_INFO("Configuration:\r\n");
    // NRF_LOG_HEXDUMP_INFO(data, 4);
    // NRF_LOG_FLUSH();
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

    NRF_LOG_INFO("BMP280 example\r\n");
    twi_config();
    bmp280_config();
    rtc_config();

    while (true)
    {
        __WFI();
        NRF_LOG_FLUSH();
    }
}


/** @} */
