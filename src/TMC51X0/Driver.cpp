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

// void Driver::setMicrostepsPerStep(uint16_t microsteps_per_step)
// {
//   uint16_t microsteps_per_step_shifted = constrain_(microsteps_per_step,
//     MICROSTEPS_PER_STEP_MIN,
//     MICROSTEPS_PER_STEP_MAX);
//   microsteps_per_step_shifted = microsteps_per_step >> 1;
//   uint16_t exponent = 0;
//   while (microsteps_per_step_shifted > 0)
//   {
//     microsteps_per_step_shifted = microsteps_per_step_shifted >> 1;
//     ++exponent;
//   }
//   setMicrostepsPerStepPowerOfTwo(exponent);
// }

// void Driver::setMicrostepsPerStepPowerOfTwo(uint8_t exponent)
// {
//   switch (exponent)
//   {
//     case 0:
//     {
//       chopconf_.mres = MRES_001;
//       break;
//     }
//     case 1:
//     {
//       chopconf_.mres = MRES_002;
//       break;
//     }
//     case 2:
//     {
//       chopconf_.mres = MRES_004;
//       break;
//     }
//     case 3:
//     {
//       chopconf_.mres = MRES_008;
//       break;
//     }
//     case 4:
//     {
//       chopconf_.mres = MRES_016;
//       break;
//     }
//     case 5:
//     {
//       chopconf_.mres = MRES_032;
//       break;
//     }
//     case 6:
//     {
//       chopconf_.mres = MRES_064;
//       break;
//     }
//     case 7:
//     {
//       chopconf_.mres = MRES_128;
//       break;
//     }
//     case 8:
//     default:
//     {
//       chopconf_.mres = MRES_256;
//       break;
//     }
//   }
//   writeStoredChopconf();
// }

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
