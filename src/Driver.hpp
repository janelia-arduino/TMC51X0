// ----------------------------------------------------------------------------
// Driver.hpp
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------

#ifndef TMC51X0_DRIVER_HPP
#define TMC51X0_DRIVER_HPP
#include <Arduino.h>

#include "TMC51X0/Constants.hpp"
#include "TMC51X0/Registers.hpp"


class TMC51X0;

namespace tmc51x0
{
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
  Registers * registers_ptr_;
  int16_t hardware_enable_pin_;

  void setup(Registers & registers);

  void hardwareEnable();
  void hardwareDisable();
  void softwareEnable();
  void softwareDisable();

  uint32_t constrain_(uint32_t value, uint32_t low, uint32_t high);

  friend class ::TMC51X0;
};
}
#endif
