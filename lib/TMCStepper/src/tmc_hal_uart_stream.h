#pragma once

#include <stddef.h>
#include <stdint.h>

#include "tmc_stream.h"

extern "C" {
#include "stm32g4xx_hal.h"
}

class TmcHalUartStream final : public Stream {
public:
  explicit TmcHalUartStream(UART_HandleTypeDef *uart_handle);

  int available() override;
  int read() override;
  size_t write(uint8_t value) override;

private:
  UART_HandleTypeDef *uart_handle_;
};
