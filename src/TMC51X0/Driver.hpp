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
#include "Registers.hpp"


class Driver
{
public:
  Driver();

  // driver must be enabled before use it is disabled by default
  void setHardwareEnablePin(size_t hardware_enable_pin);
  void enable();
  void disable();

  // valid values = 1,2,4,8,...128,256, other values get rounded down
  // void setMicrostepsPerStep(uint16_t microsteps_per_step);

  // valid values = 0-8, microsteps = 2^exponent, 0=1,1=2,2=4,...8=256
  // https://en.wikipedia.org/wiki/Power_of_two
  // void setMicrostepsPerStepPowerOfTwo(uint8_t exponent);

  // uint16_t getMicrostepsPerStep();
private:
  int16_t hardware_enable_pin_;

  void hardwareEnable();
  void hardwareDisable();

  uint32_t constrain_(uint32_t value, uint32_t low, uint32_t high);
};

#endif
