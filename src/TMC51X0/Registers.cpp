// ----------------------------------------------------------------------------
// Registers.cpp
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------
#include "Registers.hpp"


using namespace tmc51x0;

void Registers::setup(SPIClass & spi,
  size_t chip_select_pin)
{
  interface_.setup(spi, chip_select_pin);
}

Registers::Input Registers::readInput()
{
  Input input;
  input.bytes = interface_.readRegister(ADDRESS_INPUT);

  return input;
}
