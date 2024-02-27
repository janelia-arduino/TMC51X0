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
#include "TMC51X0/Registers.hpp"


class TMC51X0
{
public:
  void setup(SPIClass & spi,
    size_t chip_select_pin);

  uint8_t readVersion();

  Driver driver;
  Controller controller;

private:
  Registers registers_;
};

#endif
