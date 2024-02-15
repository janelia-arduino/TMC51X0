// ----------------------------------------------------------------------------
// Driver.hpp
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------

#ifndef TMC51X0_DRIVER_HPP
#define TMC51X0_DRIVER_HPP
#include <Arduino.h>

#include "Constants.hpp"


class Driver
{
public:
  Driver();

  // driver must be enabled before use it is disabled by default
  void setHardwareEnablePin(size_t hardware_enable_pin);
  void enable();
  void disable();

private:
  int16_t hardware_enable_pin_;

  void hardwareEnable();
  void hardwareDisable();
};

#endif
