#ifndef NRF52_PESKY_H
#define NRF52_PESKY_H

#ifdef __cplusplus
extern "C" {
#endif

// LEDs definitions
#define LEDS_NUMBER    3

#define LED_1          22
#define LED_2          23
#define LED_3          24

#define LEDS_LIST { LED_1, LED_2, LED_3 }

#define BSP_LED_0      LED_1
#define BSP_LED_1      LED_2
#define BSP_LED_2      LED_3

#define BSP_LED_0_MASK (1<<BSP_LED_0)
#define BSP_LED_1_MASK (1<<BSP_LED_1)
#define BSP_LED_2_MASK (1<<BSP_LED_2)

#define LEDS_MASK      (BSP_LED_0_MASK | BSP_LED_1_MASK | BSP_LED_2_MASK)
/* determines which LEDs are lit when GPIO is low */
#define LEDS_INV_MASK  LEDS_MASK

#define BUTTONS_NUMBER 4

#define BUTTON_START   8
#define BUTTON_1       8
#define BUTTON_2       9 // by default protected and used as NFC, so protection should be removed before use. See "pin_change_int" example for details.
#define BUTTON_3       12 // ARDUINO_0_PIN
#define BUTTON_4       11 // ARDUINO_1_PIN
#define BUTTON_STOP    11
#define BUTTON_PULL    NRF_GPIO_PIN_PULLUP

#define BUTTONS_LIST { BUTTON_1, BUTTON_2, BUTTON_3, BUTTON_4 }

#define BSP_BUTTON_0   BUTTON_1
#define BSP_BUTTON_1   BUTTON_2
#define BSP_BUTTON_2   BUTTON_3
#define BSP_BUTTON_3   BUTTON_4

#define BSP_BUTTON_0_MASK (1<<BSP_BUTTON_0)
#define BSP_BUTTON_1_MASK (1<<BSP_BUTTON_1)
#define BSP_BUTTON_2_MASK (1<<BSP_BUTTON_2)
#define BSP_BUTTON_3_MASK (1<<BSP_BUTTON_3)

#define BUTTONS_MASK   (BSP_BUTTON_0_MASK | BSP_BUTTON_1_MASK | BSP_BUTTON_2_MASK | BSP_BUTTON_3_MASK)

#define RX_PIN_NUMBER  25
#define TX_PIN_NUMBER  26
#define CTS_PIN_NUMBER 27
#define RTS_PIN_NUMBER 28
#define HWFC           true

// Arduino board mappings
#define ARDUINO_SCL_PIN             7    // SCL signal pin
#define ARDUINO_SDA_PIN             6    // SDA signal pin
#define ARDUINO_AREF_PIN            2     // Aref pin
#define ARDUINO_13_PIN              28    // Digital pin 13, also used for RTS
#define ARDUINO_12_PIN              24    // Digital pin 12
#define ARDUINO_11_PIN              23    // Digital pin 11
#define ARDUINO_10_PIN              22    // Digital pin 10
#define ARDUINO_9_PIN               20    // Digital pin 9
#define ARDUINO_8_PIN               19    // Digital pin 8

#define ARDUINO_7_PIN               18    // Digital pin 7
#define ARDUINO_6_PIN               17    // Digital pin 6
#define ARDUINO_5_PIN               16    // Digital pin 5
#define ARDUINO_4_PIN               15    // Digital pin 4
#define ARDUINO_3_PIN               14    // Digital pin 3
#define ARDUINO_2_PIN               13    // Digital pin 2
#define ARDUINO_1_PIN               12    // Digital pin 1 / BUTTON_4
#define ARDUINO_0_PIN               11    // Digital pin 0 / BUTTON_3

#define ARDUINO_A0_PIN              3     // Analog channel 0
#define ARDUINO_A1_PIN              4     // Analog channel 1
#define ARDUINO_A2_PIN              5     // Analog channel 2
#define ARDUINO_A3_PIN              29    // Analog channel 3
#define ARDUINO_A4_PIN              30    // Analog channel 4
#define ARDUINO_A5_PIN              31    // Analog channel 5

// Low frequency clock source to be used by the SoftDevice
#define NRF_CLOCK_LFCLKSRC      {.source        = NRF_CLOCK_LF_SRC_XTAL,            \
                                 .rc_ctiv       = 0,                                \
                                 .rc_temp_ctiv  = 0,                                \
                                 .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM}

#ifdef __cplusplus
}
#endif

#endif // NRF52_PESKY_H
