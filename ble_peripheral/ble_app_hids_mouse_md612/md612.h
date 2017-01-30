#ifndef __MD612__
#define __MD612__

void md612_configure(void (*cb) (unsigned char type, long *data, int8_t *accuracy, unsigned long *timestamp));
void md612_selftest();
void md612_beforesleep();
void md612_aftersleep();
unsigned char md612_hasnewdata();

#endif