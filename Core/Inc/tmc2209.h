#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void tmc2209_init(void);

void tmc2209_set_run_current(uint16_t milliamps);
void tmc2209_set_hold_current(uint16_t milliamps);

void tmc2209_set_microsteps(uint16_t microsteps);

void tmc2209_enable_stealthchop(bool enable);
void tmc2209_enable_spreadcycle(bool enable);

uint32_t tmc2209_read_status(void);
bool tmc2209_is_stall(void);

#ifdef __cplusplus
}
#endif
