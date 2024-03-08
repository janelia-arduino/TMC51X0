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

void Driver::enableAutomaticCurrentControl()
{
  Registers::Pwmconf pwmconf;
  pwmconf.bytes = registers_ptr_->getStored(Registers::PWMCONF);
  pwmconf.pwm_autoscale = 1;
  pwmconf.pwm_autograd = 1;
  registers_ptr_->write(Registers::PWMCONF, pwmconf.bytes);
}

void Driver::disableAutomaticCurrentControl()
{
  Registers::Pwmconf pwmconf;
  pwmconf.bytes = registers_ptr_->getStored(Registers::PWMCONF);
  pwmconf.pwm_autoscale = 0;
  pwmconf.pwm_autograd = 0;
  registers_ptr_->write(Registers::PWMCONF, pwmconf.bytes);
}

// private

void Driver::setup(Registers & registers)
{
  registers_ptr_ = &registers;
  toff_ = TOFF_ENABLE_DEFAULT;

  disable();
  minimizeMotorCurrent();
  disableAutomaticCurrentControl();
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
  chopconf.toff = DISABLE_TOFF;
  registers_ptr_->write(Registers::CHOPCONF, chopconf.bytes);
}

void Driver::minimizeMotorCurrent()
{
  uint32_t global_scaler = GLOBAL_SCALER_MIN;
  registers_ptr_->write(Registers::GLOBAL_SCALER, global_scaler);

  Registers::IholdIrun ihold_irun;
  ihold_irun.ihold = CURRENT_SETTING_MIN;
  ihold_irun.irun = CURRENT_SETTING_MIN;
  registers_ptr_->write(Registers::IHOLD_IRUN, ihold_irun.bytes);
}

uint32_t Driver::constrain_(uint32_t value, uint32_t low, uint32_t high)
{
  return ((value)<(low)?(low):((value)>(high)?(high):(value)));
}
