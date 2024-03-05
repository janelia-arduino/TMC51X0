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

#include "Driver.hpp"
#include "Controller.hpp"
#include "Registers.hpp"


struct TMC51X0
{
  void setup(SPIClass & spi,
    size_t chip_select_pin);

  tmc51x0::Driver driver;
  tmc51x0::Controller controller;
  tmc51x0::Registers registers;
};

#endif
