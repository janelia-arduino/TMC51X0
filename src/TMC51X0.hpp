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

#include "TMC51X0/Driver.hpp"
#include "TMC51X0/Controller.hpp"
#include "TMC51X0/Interface.hpp"
#include "TMC51X0/Register.hpp"


class TMC51X0
{
public:
  void setup(SPIClass & spi,
    size_t chip_select_pin);

  uint8_t getVersion();

  Driver driver;
  Controller controller;

private:
  Interface interface_;
  Register register_;
};

#endif
