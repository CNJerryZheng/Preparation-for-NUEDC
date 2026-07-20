#pragma once

#include <stdint.h>
#include "ti_msp_dl_config.h"

void board_init(void);

void delay_us(uint32_t us);
void delay_ms(uint32_t ms);
