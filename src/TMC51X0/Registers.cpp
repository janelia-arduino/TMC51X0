// ----------------------------------------------------------------------------
// Registers.cpp
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------
#include "Registers.hpp"


using namespace tmc51x0;

Registers::GlobalConfig Registers::readGlobalConfig()
{
  global_config_.bytes = interface_.readRegister(ADDRESS_GLOBAL_CONFIG);

  return global_config_;
}

void Registers::writeGlobalConfig(GlobalConfig global_config)
{
  global_config_ = global_config;

  interface_.writeRegister(ADDRESS_GLOBAL_CONFIG, global_config_.bytes);
}

Registers::GlobalConfig Registers::getStoredGlobalConfig()
{
  return global_config_;
}

Registers::Inputs Registers::readInputs()
{
  Inputs inputs;
  inputs.bytes = interface_.readRegister(ADDRESS_INPUTS);

  return inputs;
}

Registers::ChopperConfig Registers::readChopperConfig()
{
  chopper_config_.bytes = interface_.readRegister(ADDRESS_CHOPPER_CONFIG);

  return chopper_config_;
}

void Registers::writeChopperConfig(ChopperConfig chopper_config)
{
  chopper_config_ = chopper_config;

  interface_.writeRegister(ADDRESS_CHOPPER_CONFIG, chopper_config_.bytes);
}

Registers::ChopperConfig Registers::getStoredChopperConfig()
{
  return chopper_config_;
}

// private
void Registers::setup(SPIClass & spi,
  size_t chip_select_pin)
{
  interface_.setup(spi, chip_select_pin);
}
