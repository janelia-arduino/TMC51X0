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
#include "Registers.hpp"


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

  void hardwareEnable();
  void hardwareDisable();
  void softwareEnable();
  void softwareDisable();
private:
  Registers * registers_ptr_;
  int16_t hardware_enable_pin_;
  uint8_t toff_;

  void setup(Registers & registers);

  uint32_t constrain_(uint32_t value, uint32_t low, uint32_t high);

  friend class ::TMC51X0;
};
}
#endif
