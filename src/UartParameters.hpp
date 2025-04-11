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
};
}
#endif
