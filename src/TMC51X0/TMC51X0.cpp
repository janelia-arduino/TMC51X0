// ----------------------------------------------------------------------------
// TMC51X0.cpp
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------
#include "TMC51X0.hpp"


using namespace tmc51x0;

void TMC51X0::setup(SPIClass & spi,
  size_t chip_select_pin)
{
  registers_.setup(spi, chip_select_pin);
  driver.setup(registers_);
  controller.setup(registers_);
}

uint8_t TMC51X0::getVersion()
{
  auto input = registers_.readInput();

  return input.version;
}
