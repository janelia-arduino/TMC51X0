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
//       chopper_config_.mres = MRES_001;
//       break;
//     }
//     case 1:
//     {
//       chopper_config_.mres = MRES_002;
//       break;
//     }
//     case 2:
//     {
//       chopper_config_.mres = MRES_004;
//       break;
//     }
//     case 3:
//     {
//       chopper_config_.mres = MRES_008;
//       break;
//     }
//     case 4:
//     {
//       chopper_config_.mres = MRES_016;
//       break;
//     }
//     case 5:
//     {
//       chopper_config_.mres = MRES_032;
//       break;
//     }
//     case 6:
//     {
//       chopper_config_.mres = MRES_064;
//       break;
//     }
//     case 7:
//     {
//       chopper_config_.mres = MRES_128;
//       break;
//     }
//     case 8:
//     default:
//     {
//       chopper_config_.mres = MRES_256;
//       break;
//     }
//   }
//   writeStoredChopperConfig();
// }

// private

void Driver::setup(Registers & registers)
{
  registers_ptr_ = &registers;
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

uint32_t Driver::constrain_(uint32_t value, uint32_t low, uint32_t high)
{
  return ((value)<(low)?(low):((value)>(high)?(high):(value)));
}
