// ----------------------------------------------------------------------------
// UartParameters.hpp
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------

#ifndef TMC51X0_UART_PARAMETERS_HPP
#define TMC51X0_UART_PARAMETERS_HPP

#include <Arduino.h>

namespace tmc51x0
{
struct UartParameters
{
  Stream * uart_ptr;
  uint8_t  node_address;
  size_t   enable_txrx_pin;

  constexpr UartParameters(Stream *uart_ptr        = nullptr,
                           uint8_t node_address    = 0,
                           size_t  enable_txrx_pin = 0)
  : uart_ptr(uart_ptr),
    node_address(node_address),
    enable_txrx_pin(enable_txrx_pin)
  {}

  // "Named parameter" style helpers

  constexpr UartParameters withUart(Stream *uart) const
  {
    return UartParameters(
      uart,
      node_address,
      enable_txrx_pin);
  }

  constexpr UartParameters withNodeAddress(uint8_t na) const
  {
    return UartParameters(
      uart_ptr,
      na,
      enable_txrx_pin);
  }

  constexpr UartParameters withEnableTxRxPin(size_t en) const
  {
    return UartParameters(
      uart_ptr,
      node_address,
      en);
  }
};
} // namespace tmc51x0

#endif // TMC51X0_UART_PARAMETERS_HPP
