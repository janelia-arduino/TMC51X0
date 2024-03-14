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

void Driver::setGlobalCurrentScaler(uint8_t scaler)
{
  registers_ptr_->write(Registers::GLOBAL_SCALER, scaler);
}

void Driver::setRunCurrent(uint8_t run_current)
{
  Registers::IholdIrun ihold_irun;
  ihold_irun.bytes = registers_ptr_->getStored(Registers::IHOLD_IRUN);
  ihold_irun.irun = run_current;
  registers_ptr_->write(Registers::IHOLD_IRUN, ihold_irun.bytes);
}

void Driver::setHoldCurrent(uint8_t hold_current)
{
  Registers::IholdIrun ihold_irun;
  ihold_irun.bytes = registers_ptr_->getStored(Registers::IHOLD_IRUN);
  ihold_irun.ihold = hold_current;
  registers_ptr_->write(Registers::IHOLD_IRUN, ihold_irun.bytes);
}

void Driver::setHoldDelay(uint8_t hold_delay)
{
  Registers::IholdIrun ihold_irun;
  ihold_irun.bytes = registers_ptr_->getStored(Registers::IHOLD_IRUN);
  ihold_irun.iholddelay = hold_delay;
  registers_ptr_->write(Registers::IHOLD_IRUN, ihold_irun.bytes);
}

void Driver::enableAutomaticCurrentScaling()
{
  Registers::Pwmconf pwmconf;
  pwmconf.bytes = registers_ptr_->getStored(Registers::PWMCONF);
  pwmconf.pwm_autoscale = 1;
  registers_ptr_->write(Registers::PWMCONF, pwmconf.bytes);
}

void Driver::disableAutomaticCurrentScaling()
{
  Registers::Pwmconf pwmconf;
  pwmconf.bytes = registers_ptr_->getStored(Registers::PWMCONF);
  pwmconf.pwm_autoscale = 0;
  registers_ptr_->write(Registers::PWMCONF, pwmconf.bytes);
}

void Driver::enableAutomaticGradientAdaptation()
{
  Registers::Pwmconf pwmconf;
  pwmconf.bytes = registers_ptr_->getStored(Registers::PWMCONF);
  pwmconf.pwm_autograd = 1;
  registers_ptr_->write(Registers::PWMCONF, pwmconf.bytes);
}

void Driver::disableAutomaticGradientAdaptation()
{
  Registers::Pwmconf pwmconf;
  pwmconf.bytes = registers_ptr_->getStored(Registers::PWMCONF);
  pwmconf.pwm_autograd = 0;
  registers_ptr_->write(Registers::PWMCONF, pwmconf.bytes);
}

void Driver::setPwmOffset(uint8_t pwm_amplitude)
{
  Registers::Pwmconf pwmconf;
  pwmconf.bytes = registers_ptr_->getStored(Registers::PWMCONF);
  pwmconf.pwm_ofs = pwm_amplitude;
  registers_ptr_->write(Registers::PWMCONF, pwmconf.bytes);
}

void Driver::setPwmGradient(uint8_t pwm_amplitude)
{
  Registers::Pwmconf pwmconf;
  pwmconf.bytes = registers_ptr_->getStored(Registers::PWMCONF);
  pwmconf.pwm_grad = pwm_amplitude;
  registers_ptr_->write(Registers::PWMCONF, pwmconf.bytes);
}

void Driver::enableStealthChop()
{
  Registers::Gconf gconf;
  gconf.bytes = registers_ptr_->getStored(Registers::GCONF);
  gconf.en_pwm_mode = 1;
  registers_ptr_->write(Registers::GCONF, gconf.bytes);
}

void Driver::disableStealthChop()
{
  Registers::Gconf gconf;
  gconf.bytes = registers_ptr_->getStored(Registers::GCONF);
  gconf.en_pwm_mode = 0;
  registers_ptr_->write(Registers::GCONF, gconf.bytes);
}

void Driver::setStandstillMode(Driver::StandstillMode mode)
{
  Registers::Pwmconf pwmconf;
  pwmconf.bytes = registers_ptr_->getStored(Registers::PWMCONF);
  pwmconf.freewheel = mode;
  registers_ptr_->write(Registers::PWMCONF, pwmconf.bytes);
}

void Driver::setStallGuardThreshold(int8_t threshold)
{
  Registers::Coolconf coolconf;
  coolconf.bytes = registers_ptr_->getStored(Registers::COOLCONF);
  coolconf.sgt = threshold;
  registers_ptr_->write(Registers::COOLCONF, coolconf.bytes);
}

void Driver::enableCoolStep(uint8_t minimum,
    uint8_t maximum)
{
  Registers::Coolconf coolconf;
  coolconf.bytes = registers_ptr_->getStored(Registers::COOLCONF);
  coolconf.semin = minimum;
  coolconf.semax = maximum;
  registers_ptr_->write(Registers::COOLCONF, coolconf.bytes);
}

void Driver::disableCoolStep()
{
  Registers::Coolconf coolconf;
  coolconf.bytes = registers_ptr_->getStored(Registers::COOLCONF);
  coolconf.semin = SEMIN_OFF;
  registers_ptr_->write(Registers::COOLCONF, coolconf.bytes);
}

// private

void Driver::setup(Registers & registers,
  Converter & converter)
{
  registers_ptr_ = &registers;
  converter_ptr_ = &converter;
  toff_ = TOFF_ENABLE_DEFAULT;

  disable();
  minimizeMotorCurrent();
  enableStealthChop();
  disableAutomaticCurrentScaling();
  disableAutomaticGradientAdaptation();
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
  uint32_t global_scaler = GLOBAL_SCALER_DEFAULT;
  registers_ptr_->write(Registers::GLOBAL_SCALER, global_scaler);

  Registers::IholdIrun ihold_irun;
  ihold_irun.ihold = CURRENT_SETTING_DEFAULT;
  ihold_irun.irun = CURRENT_SETTING_DEFAULT;
  registers_ptr_->write(Registers::IHOLD_IRUN, ihold_irun.bytes);
}
