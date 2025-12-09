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
  uint8_t node_address = 0;
  size_t enable_txrx_pin;

  constexpr UartParameters(Stream *s = nullptr,
                           uint8_t na = 0,
                           size_t en = false)
  : uart_ptr(s), node_address(na), enable_txrx_pin(en) {}

  constexpr UartParameters withUart(Stream *s) const
  {
    return UartParameters{s, node_address, enable_txrx_pin};
  }

  constexpr UartParameters withNodeAddress(uint8_t na) const
  {
    return UartParameters{uart_ptr, na, enable_txrx_pin};
  }

  constexpr UartParameters withEnableTxRxPin(size_t en) const
  {
    return UartParameters{uart_ptr, node_address, en};
  }
};
}
#endif
