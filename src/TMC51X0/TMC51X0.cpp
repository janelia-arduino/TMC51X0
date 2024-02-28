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

uint8_t TMC51X0::readVersion()
{
  auto inputs = registers_.readInputs();

  return inputs.version;
}

Registers::ChopperConfig TMC51X0::readChopperConfig()
{
  auto chopper_config = registers_.readChopperConfig();

  return chopper_config;
}
