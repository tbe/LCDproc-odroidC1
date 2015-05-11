#ifndef HD_ODROID_H
#define HD_ODROID_H

#include "lcd.h"		/* for Driver */

/* initialize this particular driver */
int hd_init_odroid(Driver *drvthis);

/**
 * odroid_gpio_map is addressed through the hd44780_private_data struct. Data
 * stored here is used for mapping physical GPIO pins to BCM2835 gpio. */
struct odroid_gpio_map {
	int en;
	int rs;
	int d7;
	int d6;
	int d5;
	int d4;
	int led1;
	int led2;
	int led3;
	int led4;
	int led5;
	int led6;
	int led7;
	int btn1;
	int btn2;
};

/** Peripheral base address of the BCM2835 */
#define BCM2835_PERI_BASE       0xC1100000
/** GPIO register start address */
#define GPIO_BASE               BCM2835_PERI_BASE + 0x8000
/** Length of register space */
#define GPIO_BLOCK_SIZE         180
/** The ODRoid has 32 GPIO pins */
#define GPIO_PINS               32

#endif // HD_LCDRPI_H
