#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void tmc_delay_ms(uint32_t delay_ms);
uint32_t tmc_millis(void);

#ifdef __cplusplus
}
#endif
