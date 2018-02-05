#ifndef __MARSBOARD__H_
#define __MARSBOARD__H_

#include <stdlib.h>
#include <stdio.h>
#include <inttypes.h>

#include "gpio_lib.h"
#include "spi_lib.h"

#define LED_GREEN    SUNXI_GPH(6)
#define LED_RED      SUNXI_GPH(5)
#define START_KEY    SUNXI_GPH(4)
#define START        SUNXI_GPH(0)
#define JOY_KEY      SUNXI_GPH(1)

/*
#define SET_CHANNEL0 (0x8300)
#define SET_CHANNEL1 (0X8700)
#define SET_CHANNEL2 (0x8f00)
#define JUSTREAD (0x0000)
*/
int8_t analogRead(uint8_t channel);
void digitalWrite(uint16_t, uint8_t);
int digitalRead(uint16_t);

#endif // __MARSBOARD__H_
