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
  Stream * stream_ptr;
  uint8_t node_address;
  size_t enable_txrx_pin;

  UartParameters(Stream & uart_,
    uint8_t node_address_,
    size_t enable_txrx_pin_=PIN_DEFAULT)
  {
    stream_ptr = &uart_;
    node_address = node_address_;
    enable_txrx_pin = enable_txrx_pin_;
  };

  UartParameters()
  {
    stream_ptr = nullptr;
    node_address = NODE_ADDRESS_DEFAULT;
    enable_txrx_pin = PIN_DEFAULT;
  };

  bool operator==(const UartParameters & rhs) const
  {
    if ((this->stream_ptr == rhs.stream_ptr) &&
      (this->node_address == rhs.node_address) &&
      (this->enable_txrx_pin == rhs.enable_txrx_pin))
    {
      return true;
    }
    return false;
  }
  bool operator!=(const UartParameters & rhs) const
  {
    return !(*this == rhs);
  }

private:
  const static uint8_t NODE_ADDRESS_DEFAULT = 0;
  const static size_t PIN_DEFAULT = 255;
};
}
#endif
