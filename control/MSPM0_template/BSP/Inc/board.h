#ifndef BOARD_H
#define BOARD_H

#include "ti_msp_dl_config.h"
#include <stdint.h>

void board_init(void);

void delay_us(uint32_t us);
void delay_ms(uint32_t ms);

#endif /* BOARD_H */
