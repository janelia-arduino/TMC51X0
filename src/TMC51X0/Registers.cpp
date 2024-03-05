// ----------------------------------------------------------------------------
// Registers.cpp
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------
#include "Registers.hpp"


using namespace tmc51x0;

void Registers::write(RegisterAddress register_address,
  uint32_t data)
{
  interface_.writeRegister(register_address, data);
  stored_[register_address] = data;
}

uint32_t Registers::read(RegisterAddress register_address)
{
  uint32_t data = interface_.readRegister(register_address);
  stored_[register_address] = data;
  return data;
}

uint32_t Registers::getStored(RegisterAddress register_address)
{
  return 0;
}

// private
void Registers::setup(SPIClass & spi,
  size_t chip_select_pin)
{
  stored_[GLOBAL_CONFIG] = DEFAULT_GLOBAL_CONFIG;
  stored_[INPUTS] = DEFAULT_INPUTS;
  stored_[CHOPPER_CONFIG] = DEFAULT_CHOPPER_CONFIG;

  interface_.setup(spi, chip_select_pin);
}
