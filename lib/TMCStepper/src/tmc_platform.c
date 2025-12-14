#include "tmc_platform.h"

#include "stm32g4xx_hal.h"

void tmc_delay_ms(uint32_t delay_ms) {
  HAL_Delay(delay_ms);
}

uint32_t tmc_millis(void) {
  return HAL_GetTick();
}
