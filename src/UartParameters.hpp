// ----------------------------------------------------------------------------
// UartParameters.hpp
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------

#ifndef TMC51X0_UART_PARAMETERS_HPP
#define TMC51X0_UART_PARAMETERS_HPP

#include <Arduino.h>
#include <stdint.h>

#include "Constants.hpp"

namespace tmc51x0 {
struct UartParameters {
  Stream *uart_ptr;
  uint8_t node_address;
  size_t enable_txrx_pin;

  // Optional configuration. These parameters are primarily used by the
  // poll-driven UART engine (non-blocking path), but the blocking wrappers may
  // also use them for consistency.
  uint32_t baud_rate;
  uint32_t reply_timeout_us;
  uint32_t enable_delay_us;
  uint8_t max_retries;

  // If baud_rate is not known, an optional conservative guard time (in
  // microseconds) to keep TX enabled before switching to RX in half-duplex
  // mode.
  //
  // If both baud_rate and tx_complete_delay_us are zero, the library may fall
  // back to Stream::flush() (which can block on some platforms).
  uint32_t tx_complete_delay_us;

  // Maximum number of bytes to drain from RX before starting a transaction.
  // If the limit is exceeded and stale bytes are still present, the transaction
  // fails with UartError::RxGarbage (or retries if configured).
  uint16_t drain_limit;

  constexpr UartParameters(Stream *uart_ptr = nullptr, uint8_t node_address = 0,
                           size_t enable_txrx_pin = NO_PIN,
                           uint32_t baud_rate = 0,
                           uint32_t reply_timeout_us = 10000,
                           uint32_t enable_delay_us = 10,
                           uint8_t max_retries = 0,
                           uint32_t tx_complete_delay_us = 0,
                           uint16_t drain_limit = 256)
      : uart_ptr(uart_ptr), node_address(node_address),
        enable_txrx_pin(enable_txrx_pin), baud_rate(baud_rate),
        reply_timeout_us(reply_timeout_us), enable_delay_us(enable_delay_us),
        max_retries(max_retries), tx_complete_delay_us(tx_complete_delay_us),
        drain_limit(drain_limit) {}

  // "Named parameter" style helpers

  constexpr UartParameters withUart(Stream *uart) const {
    return UartParameters(uart, node_address, enable_txrx_pin, baud_rate,
                          reply_timeout_us, enable_delay_us, max_retries,
                          tx_complete_delay_us, drain_limit);
  }

  constexpr UartParameters withNodeAddress(uint8_t na) const {
    return UartParameters(uart_ptr, na, enable_txrx_pin, baud_rate,
                          reply_timeout_us, enable_delay_us, max_retries,
                          tx_complete_delay_us, drain_limit);
  }

  constexpr UartParameters withEnableTxRxPin(size_t en) const {
    return UartParameters(uart_ptr, node_address, en, baud_rate,
                          reply_timeout_us, enable_delay_us, max_retries,
                          tx_complete_delay_us, drain_limit);
  }

  constexpr UartParameters withBaudRate(uint32_t br) const {
    return UartParameters(uart_ptr, node_address, enable_txrx_pin, br,
                          reply_timeout_us, enable_delay_us, max_retries,
                          tx_complete_delay_us, drain_limit);
  }

  constexpr UartParameters withReplyTimeoutUs(uint32_t timeout_us) const {
    return UartParameters(uart_ptr, node_address, enable_txrx_pin, baud_rate,
                          timeout_us, enable_delay_us, max_retries,
                          tx_complete_delay_us, drain_limit);
  }

  constexpr UartParameters withEnableDelayUs(uint32_t delay_us) const {
    return UartParameters(uart_ptr, node_address, enable_txrx_pin, baud_rate,
                          reply_timeout_us, delay_us, max_retries,
                          tx_complete_delay_us, drain_limit);
  }

  constexpr UartParameters withMaxRetries(uint8_t retries) const {
    return UartParameters(uart_ptr, node_address, enable_txrx_pin, baud_rate,
                          reply_timeout_us, enable_delay_us, retries,
                          tx_complete_delay_us, drain_limit);
  }

  constexpr UartParameters withTxCompleteDelayUs(uint32_t delay_us) const {
    return UartParameters(uart_ptr, node_address, enable_txrx_pin, baud_rate,
                          reply_timeout_us, enable_delay_us, max_retries,
                          delay_us, drain_limit);
  }

  constexpr UartParameters withDrainLimit(uint16_t limit) const {
    return UartParameters(uart_ptr, node_address, enable_txrx_pin, baud_rate,
                          reply_timeout_us, enable_delay_us, max_retries,
                          tx_complete_delay_us, limit);
  }
};
} // namespace tmc51x0

#endif // TMC51X0_UART_PARAMETERS_HPP
