// ----------------------------------------------------------------------------
// TMC51X0.hpp
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------

#ifndef TMC51X0_HPP
#define TMC51X0_HPP
#include <Arduino.h>
#include <SPI.h>

#include "TMC51X0/Interface.hpp"
#include "TMC51X0/Register.hpp"


class TMC51X0
{
public:
  TMC51X0();

  void setup(SPIClass & spi,
    size_t chip_select_pin);

  // driver must be enabled before use it is disabled by default
  void setHardwareEnablePin(size_t hardware_enable_pin);
  void enable();
  void disable();

  uint8_t getVersion();
private:
  Interface interface_;
  Register register_;

  int16_t hardware_enable_pin_;

  void hardwareEnable();
  void hardwareDisable();

};

#endif
