// ----------------------------------------------------------------------------
// TMC51X0.cpp
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------
#include "TMC51X0.hpp"


TMC51X0::TMC51X0()
{
  hardware_enable_pin_ = -1;
}

void TMC51X0::setup(SPIClass & spi,
  size_t chip_select_pin)
{
  interface_.setup(spi, chip_select_pin);
}

void TMC51X0::setHardwareEnablePin(size_t hardware_enable_pin)
{
  hardware_enable_pin_ = hardware_enable_pin;
  pinMode(hardware_enable_pin_, OUTPUT);
  hardwareDisable();
}

void TMC51X0::enable()
{
  hardwareEnable();
  // chopper_config_.toff = toff_;
  // writeStoredChopperConfig();
}

void TMC51X0::disable()
{
  hardwareDisable();
  // chopper_config_.toff = TOFF_DISABLE;
  // writeStoredChopperConfig();
}

uint8_t TMC51X0::getVersion()
{
  Register::Input input;
  input.bytes = interface_.readRegister(register_.ADDRESS_IOIN);

  return input.version;
}

// private

void TMC51X0::hardwareEnable()
{
  if (hardware_enable_pin_ >= 0)
  {
    digitalWrite(hardware_enable_pin_, LOW);
  }
}

void TMC51X0::hardwareDisable()
{
  if (hardware_enable_pin_ >= 0)
  {
    digitalWrite(hardware_enable_pin_, HIGH);
  }
}

