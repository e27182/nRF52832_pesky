#ifndef _TIMESTAMPING_
#define _TIMESTAMPING_

#include <stdint.h>

#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE         4                                           /**< Size of timer operation queues. */
#define APP_TIMER_MS(TICKS, PRESCALER)\
            ((uint32_t)ROUNDED_DIV((TICKS) * ((PRESCALER) + 1) * 1000, (uint64_t)APP_TIMER_CLOCK_FREQ))

void lfclk_config(void);
uint32_t timestamp_func(void);

#endif // _TIMESTAMPING_