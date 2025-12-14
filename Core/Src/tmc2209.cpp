#include "tmc2209.h"

#include <new>

extern "C" {
#include "usart.h"
}

#include "tmc2209_config.h"

#include "TMC2209Stepper.h"
#include "tmc_hal_uart_stream.h"

namespace {

alignas(TMC2209Stepper) static uint8_t driver_storage[sizeof(TMC2209Stepper)];
static TMC2209Stepper *driver_instance = nullptr;
static TmcHalUartStream *uart_stream = nullptr;
alignas(TmcHalUartStream) static uint8_t uart_stream_storage[sizeof(TmcHalUartStream)];

uint8_t current_to_cs(uint16_t milliamps, float rsense_ohms) {
  const float current_amps = static_cast<float>(milliamps) / 1000.0f;
  const float numerator = 32.0f * 1.41421f * current_amps * (rsense_ohms + 0.02f);

  float cs = (numerator / 0.325f) - 1.0f;
  if (cs < 16.0f) {
    cs = (numerator / 0.180f) - 1.0f;
  }

  if (cs < 0.0f) {
    cs = 0.0f;
  }
  if (cs > 31.0f) {
    cs = 31.0f;
  }

  return static_cast<uint8_t>(cs);
}

uint8_t hold_current_to_cs(uint16_t milliamps) {
  return current_to_cs(milliamps, TMC2209_R_SENSE_OHMS);
}

uint8_t run_current_to_cs(uint16_t milliamps) {
  return current_to_cs(milliamps, TMC2209_R_SENSE_OHMS);
}

} // namespace

void tmc2209_init(void) {
  if (uart_stream == nullptr) {
    uart_stream = new (uart_stream_storage) TmcHalUartStream(&huart1);
  }

  if (driver_instance == nullptr) {
    driver_instance = new (driver_storage) TMC2209Stepper(uart_stream, TMC2209_R_SENSE_OHMS, TMC2209_UART_ADDRESS);
  }

  driver_instance->begin();
  driver_instance->toff(TMC2209_TOFF);
  driver_instance->blank_time(TMC2209_BLANK_TIME);

  driver_instance->intpol(TMC2209_ENABLE_INTERPOLATE != 0u);

  driver_instance->microsteps(TMC2209_MICROSTEPS);
  driver_instance->en_spreadCycle(TMC2209_ENABLE_SPREADCYCLE != 0u);
  driver_instance->pwm_autoscale(TMC2209_ENABLE_PWM_AUTOSCALE != 0u);
  driver_instance->pdn_disable(TMC2209_ENABLE_PDN_DISABLE != 0u);

  driver_instance->ihold(hold_current_to_cs(TMC2209_HOLD_CURRENT_MA));
  driver_instance->irun(run_current_to_cs(TMC2209_RUN_CURRENT_MA));
  driver_instance->iholddelay(TMC2209_HOLD_DELAY);
  driver_instance->I_scale_analog(TMC2209_ENABLE_I_SCALE_ANALOG != 0u);

  // Swap to stealthchop at theashold
  driver_instance->TPWMTHRS(TMC2209_SILENT_THRESHOLD_TPWMTHRS);

  // Setup coolstep
  // driver_instance->TCOOLTHRS(200); // Enable CoolStep above this speed
  // driver_instance->semin(5);
  // driver_instance->semax(2);
  // driver_instance->sedn(0b01);
  // driver_instance->seup(0b01);

  // Setup stallguard
  // driver_instance->SGTHRS(8);  // Tune experimentally
}

void tmc2209_set_run_current(uint16_t milliamps) {
  if (driver_instance == nullptr) {
    return;
  }

  driver_instance->irun(milliamps);
}

void tmc2209_set_hold_current(uint16_t milliamps) {
  if (driver_instance == nullptr) {
    return;
  }

  driver_instance->ihold(milliamps);
}

void tmc2209_set_microsteps(uint16_t microsteps) {
  if (driver_instance == nullptr) {
    return;
  }

  driver_instance->microsteps(microsteps);
}

void tmc2209_enable_stealthchop(bool enable) {
  if (driver_instance == nullptr) {
    return;
  }

  driver_instance->en_spreadCycle(!enable);
  driver_instance->pwm_autoscale(enable);
}

void tmc2209_enable_spreadcycle(bool enable) {
  if (driver_instance == nullptr) {
    return;
  }

  driver_instance->en_spreadCycle(enable);
}

uint32_t tmc2209_read_status(void) {
  if (driver_instance == nullptr) {
    return 0;
  }

  return driver_instance->DRV_STATUS();
}

bool tmc2209_is_stall(void) {
  if (driver_instance == nullptr) {
    return false;
  }

  return driver_instance->SG_RESULT() <= TMC2209_STALL_SG_RESULT_THRESHOLD;
}
