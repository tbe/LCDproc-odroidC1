/** \file server/drivers/hd44780-odroid.c
 * \c odroid connection type of \c hd44780 driver for Hitachi HD44780 based LCD
 * displays connected to the GPIO pins on the odroid-c1
 *
 * The LCD is operated in its 4 bit-mode. R/W (5) on the LCD MUST be hard wired
 * low to prevent 5V logic appearing on the GPIO pins.
 *
 *
 * Mappings can be set in the config file using the key-words:
 * pin_EN, pin_RS, pin_D7, pin_D6, pin_D5, pin_D4
 * in the [HD44780] section.
 */

/*-
 * Copyright (c) 2012-2013 Paul Corner <paul_c@users.sourceforge.net>
 *                    2013 warhog <warhog@gmx.de> (Conditional ARM test)
 *                    2013 Serac <Raspberry Pi forum>
 *                               (Backlight & Rev2 board support)
 *                    2015 Loki <loki@lokis-chaos.de>
 *                               (odroid-c1 version)
 *
 * This driver is based in the HD44780-rpi version
 *
 * This file is released under the GNU General Public License. Refer to the
 * COPYING file distributed with this package.
 */

/* Default GPIO pin assignment */
#define ODROID_DEF_D7  4
#define ODROID_DEF_D6  1
#define ODROID_DEF_D5  3
#define ODROID_DEF_D4  2
#define ODROID_DEF_RS  7
#define ODROID_DEF_EN  0


#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <errno.h>

#include "hd44780-odroid.h"
#include "hd44780-low.h"
#include "report.h"

void odroid_HD44780_senddata(PrivateData *p, unsigned char displayID, unsigned char flags, unsigned char ch);
void odroid_HD44780_backlight(PrivateData *p, unsigned char state);
void odroid_HD44780_close(PrivateData *p);

/**
 * Pointer to the memory mapped GPIO registers. Note the pointer type is
 * (unsigned int) not (char). This is important when calculating offset
 * adresses!
 *
 * This pointer is outside PrivataData as it needs to be volatile!
 */
static volatile unsigned int *gpio_map = NULL;


#define GPIOY_PIN_START         80
#define GPIOY_PIN_END           96
#define GPIOX_PIN_START         97
#define GPIOX_PIN_END           118

#define GPIOX_FSEL_REG_OFFSET   0x0C
#define GPIOX_PUEN_REG_OFFSET   0x4C
#define GPIOX_OUTP_REG_OFFSET   0x0D

#define GPIOY_FSEL_REG_OFFSET   0x0F
#define GPIOY_PUEN_REG_OFFSET   0x4B
#define GPIOY_OUTP_REG_OFFSET   0x10

//
// offset to the GPIO Set regsiter
//
static int  gpioToGPSETReg (int pin)
{
    if(pin >= GPIOX_PIN_START && pin <= GPIOX_PIN_END)  return  GPIOX_OUTP_REG_OFFSET;
    if(pin >= GPIOY_PIN_START && pin <= GPIOY_PIN_END)  return  GPIOY_OUTP_REG_OFFSET;
    
    return  -1;
}
   
//
// offset to the GPIO Pull up/down enable regsiter
//
static int  gpioToPUENReg (int pin)
{
    if(pin >= GPIOX_PIN_START && pin <= GPIOX_PIN_END)  return  GPIOX_PUEN_REG_OFFSET;
    if(pin >= GPIOY_PIN_START && pin <= GPIOY_PIN_END)  return  GPIOY_PUEN_REG_OFFSET;
    
    return  -1;
}

//
// offset to the GPIO bit
//
static int  gpioToShiftReg (int pin)
{
    if(pin >= GPIOX_PIN_START && pin <= GPIOX_PIN_END)  return  pin - GPIOX_PIN_START;
    if(pin >= GPIOY_PIN_START && pin <= GPIOY_PIN_END)  return  pin - GPIOY_PIN_START;
    
    return  -1;
}

//
// offset to the GPIO Function register
//
static int  gpioToGPFSELReg (int pin)
{
    if(pin >= GPIOX_PIN_START && pin <= GPIOX_PIN_END)  return  GPIOX_FSEL_REG_OFFSET;
    if(pin >= GPIOY_PIN_START && pin <= GPIOY_PIN_END)  return  GPIOY_FSEL_REG_OFFSET;
    
    return  -1;
}

//
// pinToGpio:
//  Take a Wiring pin (0 through X) and re-map it to the ODROID_GPIO pin
//
static int pinToGpio [64] = {
    88,  87, 116, 115, 104, 102, 103,  83, // 0..7
    -1,  -1, 117, 118, 107, 106, 105,  -1, // 8..16
    -1,  -1,  -1,  -1,  -1, 101, 100, 108, // 16..23
    97,  -1,  99,  98,  -1,  -1,  -1,  -1, // 24..31
    // Padding:
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,   // ... 47
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,   // ... 63
};


/** Setup a GPIO pin as input */
#define INP_GPIO(g) *(gpio_map + gpioToGPFSELReg(pinToGpio[g])) = (*(gpio_map + gpioToGPFSELReg(pinToGpio[g])) |  (1 << gpioToShiftReg(pinToGpio[g])));

/** Sets or clears a GPIO pin */
#define SET_GPIO(g,a) if (a == 0) *(gpio_map + gpioToGPSETReg(pinToGpio[g])) &= ~(1 << gpioToShiftReg(pinToGpio[g])); else *(gpio_map + gpioToGPSETReg(pinToGpio[g])) |=  (1 << gpioToShiftReg(pinToGpio[g]));

/**
 * Maps a memory region to the address space of the BM2835 GPIO controller.
 * \param drvthis  Pointer to driver structure.
 * \return         0 on success, -1 on error.
 */
static int
setup_io(Driver *drvthis)
{
	int mem_fd = 0;

	if (gpio_map != NULL) {
		report(RPT_ERR, "setup_io: IO already in use");
		return -1;
	}

	if ((mem_fd = open("/dev/mem", O_RDWR | O_SYNC)) < 0) {
		report(RPT_ERR, "setup_io: can not open /dev/mem");
		return -1;
	}

	gpio_map = (unsigned int *) mmap(NULL,
					 GPIO_BLOCK_SIZE,
					 PROT_READ | PROT_WRITE,
					 MAP_SHARED,
					 mem_fd,
					 GPIO_BASE);

	if (gpio_map == MAP_FAILED) {
		report(RPT_ERR, "setup_io: mmap failed: %s", strerror(errno));
		close(mem_fd);
		return -1;
	}
	close(mem_fd);

	debug(RPT_INFO, "setup_io: finished");
	return 0;
}


/**
 * Check user provided GPIO pin configuration.
 * \param drvthis       Pointer to driver structure.
 * \param pin           Number of the GPIO pin to use.
 * \param used_pins     Pointer to array of pin usage status.
 * \return  0 on success, -1 on error.
 */
static int
check_pin(Driver *drvthis, int pin, int *used_pins)
{
	/* Check for errors */
	if (pin >= GPIO_PINS || pin < 0) {
		report(RPT_ERR, "check_pin: GPIO pin %i out of range", pin);
		return -1;
	}
	if (used_pins[pin] == 1) {
		report(RPT_ERR, "check_pin: GPIO pin %i already used", pin);
		return -1;
	}

	/* Mark used */
	used_pins[pin] = 1;

	return 0;
}


/**
 * Configures a GPIO pin: Disable pull-up/down and set it up as output.
 * \param drvthis  Pointer to driver structure.
 * \param gpio     Number of the GPIO pin to use.
 */
static void
setup_gpio(Driver *drvthis, int gpio)
{
	volatile int i;

    // map pin
    gpio = pinToGpio[gpio];

	/* Disable pull-up/down */
    *(gpio_map + gpioToPUENReg(gpio)) = (*(gpio_map + gpioToPUENReg(gpio)) & ~(1 << gpioToShiftReg(gpio)));

	/*
	 * After writing to the GPPUD register, need to wait 150 cycles as per
	 * p101 BCM2835.pdf. The following while loop uses approx five
	 * instructions plus another two to load the counter. Note: the int
	 * must be volatile or gcc will optimize loop out.
	 */
	i = 30;
	while (--i);

	/*
	 * Now configure the pin as output
	 */
    *(gpio_map + gpioToGPFSELReg(gpio)) = (*(gpio_map + gpioToGPFSELReg(gpio)) & ~(1 << gpioToShiftReg(gpio)));
}


/**
 * Free resources used by this connection type.
 * \param p  Pointer to driver's PrivateData structure.
 */
void
odroid_HD44780_close(PrivateData *p)
{
	/* Configure all pins as input */
    INP_GPIO(p->odroid_gpio->en);
    INP_GPIO(p->odroid_gpio->rs);
    INP_GPIO(p->odroid_gpio->d7);
    INP_GPIO(p->odroid_gpio->d6);
    INP_GPIO(p->odroid_gpio->d5);
    INP_GPIO(p->odroid_gpio->d4);

	/* Unmap and free memory */
	if (gpio_map != NULL)
		munmap((caddr_t) gpio_map, GPIO_BLOCK_SIZE);
    if (p->odroid_gpio != NULL)
        free(p->odroid_gpio);
    p->odroid_gpio = NULL;
}

/**
 * Initialize the driver.
 * \param drvthis  Pointer to driver structure.
 * \retval 0       Success.
 * \retval -1      Error.
 */
int
hd_init_odroid(Driver *drvthis)
{
	PrivateData *p = (PrivateData *) drvthis->private_data;
	int used_pins[GPIO_PINS] = {};

	/* Get GPIO configuration */
	p->odroid_gpio = malloc(sizeof(struct odroid_gpio_map));
	if (p->odroid_gpio == NULL) {
		report(RPT_ERR, "hd_init_odroid: unable to allocate memory");
		return -1;
	}

	p->odroid_gpio->en = drvthis->config_get_int(drvthis->name, "pin_EN", 0, ODROID_DEF_EN);
    p->odroid_gpio->rs = drvthis->config_get_int(drvthis->name, "pin_RS", 0, ODROID_DEF_RS);
    p->odroid_gpio->d7 = drvthis->config_get_int(drvthis->name, "pin_D7", 0, ODROID_DEF_D7);
    p->odroid_gpio->d6 = drvthis->config_get_int(drvthis->name, "pin_D6", 0, ODROID_DEF_D6);
    p->odroid_gpio->d5 = drvthis->config_get_int(drvthis->name, "pin_D5", 0, ODROID_DEF_D5);
    p->odroid_gpio->d4 = drvthis->config_get_int(drvthis->name, "pin_D4", 0, ODROID_DEF_D4);

    debug(RPT_INFO, "hd_init_odroid: Pin EN mapped to GPIO%d", p->odroid_gpio->en);
    debug(RPT_INFO, "hd_init_odroid: Pin RS mapped to GPIO%d", p->odroid_gpio->rs);
    debug(RPT_INFO, "hd_init_odroid: Pin D4 mapped to GPIO%d", p->odroid_gpio->d4);
    debug(RPT_INFO, "hd_init_odroid: Pin D5 mapped to GPIO%d", p->odroid_gpio->d5);
    debug(RPT_INFO, "hd_init_odroid: Pin D6 mapped to GPIO%d", p->odroid_gpio->d6);
    debug(RPT_INFO, "hd_init_odroid: Pin D7 mapped to GPIO%d", p->odroid_gpio->d7);

    if (check_pin(drvthis, p->odroid_gpio->en, used_pins) ||
        check_pin(drvthis, p->odroid_gpio->rs, used_pins) ||
	    check_pin(drvthis, p->odroid_gpio->d7, used_pins) ||
	    check_pin(drvthis, p->odroid_gpio->d6, used_pins) ||
	    check_pin(drvthis, p->odroid_gpio->d5, used_pins) ||
	    check_pin(drvthis, p->odroid_gpio->d4, used_pins)) {
		free(p->odroid_gpio);
		return -1;
	}


	/* Now that configuration should be correct, set up the GPIO pins */
	if (setup_io(drvthis) < 0) {
		report(RPT_ERR, "hd_init_odroid: Failed to set up GPIO");
		free(p->odroid_gpio);
		return -1;
	}

	setup_gpio(drvthis, p->odroid_gpio->en);
	setup_gpio(drvthis, p->odroid_gpio->rs);
	setup_gpio(drvthis, p->odroid_gpio->d7);
	setup_gpio(drvthis, p->odroid_gpio->d6);
	setup_gpio(drvthis, p->odroid_gpio->d5);
	setup_gpio(drvthis, p->odroid_gpio->d4);

	p->hd44780_functions->senddata = odroid_HD44780_senddata;
	p->hd44780_functions->close = odroid_HD44780_close;

	/* Setup the lcd in 4 bit mode: Send (FUNCSET | IF_8BIT) three times
	 * followed by (FUNCSET | IF_4BIT) using four nibbles. Timing is not
	 * exactly what is required by HD44780. */
	p->hd44780_functions->senddata(p, 0, RS_INSTR, 0x33);
	p->hd44780_functions->uPause(p, 4100);
	p->hd44780_functions->senddata(p, 0, RS_INSTR, 0x32 );
	p->hd44780_functions->uPause(p, 150);

	common_init(p, IF_4BIT);

	return 0;
}


/**
 * Send data or commands to the display.
 * \param p          Pointer to driver's private data structure.
 * \param displayID  ID of the display (or 0 for all) to send data to.
 * \param flags      Defines whether to end a command or data.
 * \param ch         The value to send.
 */
void
odroid_HD44780_senddata(PrivateData *p, unsigned char displayID, unsigned char flags, unsigned char ch)
{
	/* Safeguard: This should never happen */
	if (gpio_map == NULL) {
		return;
	}

	/* we don't support more then one desplay at this time */
    if ( displayID > 1 )
        return;

	if (flags == RS_INSTR) {
		SET_GPIO(p->odroid_gpio->rs, 0);
	}
	else {			/* flags == RS_DATA */
		SET_GPIO(p->odroid_gpio->rs, 1);
	}
	/* Clear data lines ready for nibbles */
	SET_GPIO(p->odroid_gpio->d7, 0);
	SET_GPIO(p->odroid_gpio->d6, 0);
	SET_GPIO(p->odroid_gpio->d5, 0);
	SET_GPIO(p->odroid_gpio->d4, 0);
	p->hd44780_functions->uPause(p, 50);

	/* Output upper nibble first */
	SET_GPIO(p->odroid_gpio->d7, (ch & 0x80));
	SET_GPIO(p->odroid_gpio->d6, (ch & 0x40));
	SET_GPIO(p->odroid_gpio->d5, (ch & 0x20));
	SET_GPIO(p->odroid_gpio->d4, (ch & 0x10));
	p->hd44780_functions->uPause(p, 50);

	/* Data is clocked on the falling edge of EN */
    SET_GPIO(p->odroid_gpio->en, 1);
	p->hd44780_functions->uPause(p, 50);
	SET_GPIO(p->odroid_gpio->en, 0);
	p->hd44780_functions->uPause(p, 50);

	/* Do same for lower nibble */
	SET_GPIO(p->odroid_gpio->d7, 0);
	SET_GPIO(p->odroid_gpio->d6, 0);
	SET_GPIO(p->odroid_gpio->d5, 0);
	SET_GPIO(p->odroid_gpio->d4, 0);
	p->hd44780_functions->uPause(p, 50);

	SET_GPIO(p->odroid_gpio->d7, (ch & 0x08));
	SET_GPIO(p->odroid_gpio->d6, (ch & 0x04));
	SET_GPIO(p->odroid_gpio->d5, (ch & 0x02));
	SET_GPIO(p->odroid_gpio->d4, (ch & 0x01));
	p->hd44780_functions->uPause(p, 50);

	SET_GPIO(p->odroid_gpio->en, 1);
	p->hd44780_functions->uPause(p, 50);

	SET_GPIO(p->odroid_gpio->en, 0);
	p->hd44780_functions->uPause(p, 50);
}


/**
 * Turn display backlight on or off.
 * \param p      Pointer to driver's private data structure.
 * \param state  New backlight status.
 */
void
odroid_HD44780_backlight(PrivateData *p, unsigned char state)
{
    return;
}
