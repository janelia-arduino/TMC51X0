// ----------------------------------------------------------------------------
// Driver.cpp
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------
#include "Driver.hpp"


using namespace tmc51x0;

Driver::Driver()
{
  hardware_enable_pin_ = -1;
}

void Driver::setHardwareEnablePin(size_t hardware_enable_pin)
{
  hardware_enable_pin_ = hardware_enable_pin;
  pinMode(hardware_enable_pin_, OUTPUT);
  hardwareDisable();
}

void Driver::enable()
{
  hardwareEnable();
  softwareEnable();
}

void Driver::disable()
{
  hardwareDisable();
  softwareDisable();
}

void Driver::hardwareEnable()
{
  if (hardware_enable_pin_ >= 0)
  {
    digitalWrite(hardware_enable_pin_, LOW);
  }
}

void Driver::hardwareDisable()
{
  if (hardware_enable_pin_ >= 0)
  {
    digitalWrite(hardware_enable_pin_, HIGH);
  }
}

void Driver::softwareEnable()
{
  Registers::Chopconf chopconf;
  chopconf.bytes = registers_ptr_->getStored(Registers::CHOPCONF);
  chopconf.toff = toff_;
  registers_ptr_->write(Registers::CHOPCONF, chopconf.bytes);
}

void Driver::softwareDisable()
{
  Registers::Chopconf chopconf;
  chopconf.bytes = registers_ptr_->getStored(Registers::CHOPCONF);
  chopconf.toff = Registers::DISABLE_TOFF;
  registers_ptr_->write(Registers::CHOPCONF, chopconf.bytes);
}

// private

void Driver::setup(Registers & registers)
{
  registers_ptr_ = &registers;
  toff_ = Registers::TOFF_ENABLE_DEFAULT;
}

uint32_t Driver::constrain_(uint32_t value, uint32_t low, uint32_t high)
{
  return ((value)<(low)?(low):((value)>(high)?(high):(value)));
}
