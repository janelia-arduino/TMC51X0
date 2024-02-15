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
  interface_.setup(spi, chip_select_pin);
}

uint8_t TMC51X0::getVersion()
{
  registers::Input input;
  input.bytes = interface_.readRegister(registers::ADDRESS_IOIN);

  return input.version;
}
