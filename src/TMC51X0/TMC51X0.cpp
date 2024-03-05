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
  registers.setup(spi, chip_select_pin);
  driver.setup(registers);
  controller.setup(registers);
}

