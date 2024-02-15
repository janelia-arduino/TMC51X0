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
  // chopper_config_.toff = toff_;
  // writeStoredChopperConfig();
}

void Driver::disable()
{
  hardwareDisable();
  // chopper_config_.toff = TOFF_DISABLE;
  // writeStoredChopperConfig();
}

// private

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
