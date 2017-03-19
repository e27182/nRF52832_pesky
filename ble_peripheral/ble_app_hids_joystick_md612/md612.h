#ifndef __MD612__
#define __MD612__

#include "nrf_drv_gpiote.h"

/* Platform-specific information. Kinda like a boardfile. */
typedef struct {
    void (*cb) (unsigned char type, long *data, int8_t accuracy, unsigned long timestamp, unsigned short tap);
    signed char gyro_orientation[9];
    signed char compass_orientation[9];
    nrf_drv_gpiote_pin_t pin;
} platform_data_t;

void md612_configure(platform_data_t const * platform_data);
void md612_selftest();
void md612_beforesleep();
void md612_aftersleep();
unsigned char md612_hasnewdata();

#endif