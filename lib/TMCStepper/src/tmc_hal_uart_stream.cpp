#include "tmc_hal_uart_stream.h"

TmcHalUartStream::TmcHalUartStream(UART_HandleTypeDef *uart_handle) : uart_handle_(uart_handle) {}

int TmcHalUartStream::available() {
  if (uart_handle_ == nullptr || uart_handle_->Instance == nullptr) {
    return 0;
  }

  return (__HAL_UART_GET_FLAG(uart_handle_, UART_FLAG_RXNE) != 0U) ? 1 : 0;
}

int TmcHalUartStream::read() {
  if (uart_handle_ == nullptr || uart_handle_->Instance == nullptr) {
    return -1;
  }

  if (__HAL_UART_GET_FLAG(uart_handle_, UART_FLAG_RXNE) == 0U) {
    return -1;
  }

  return static_cast<int>(uart_handle_->Instance->RDR & 0xFFU);
}

size_t TmcHalUartStream::write(uint8_t value) {
  if (uart_handle_ == nullptr) {
    return 0;
  }

  if (HAL_UART_Transmit(uart_handle_, &value, 1, HAL_MAX_DELAY) != HAL_OK) {
    return 0;
  }

  return 1;
}
