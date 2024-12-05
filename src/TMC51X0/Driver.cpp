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
  setup_driver_parameters_ = DriverParameters{};
  hardware_enable_pin_ = NO_PIN;
}

void Driver::setup()
{
  writeDriverParameters(setup_driver_parameters_);
}

void Driver::setup(tmc51x0::DriverParameters driver_parameters)
{
  setup_driver_parameters_ = driver_parameters;
  setup();
}

void Driver::setEnableHardwarePin(size_t hardware_enable_pin)
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

void Driver::writeGlobalCurrentScaler(uint8_t scaler)
{
  registers_ptr_->write(Registers::GLOBAL_SCALER, scaler);
}

void Driver::writeRunCurrent(uint8_t run_current)
{
  Registers::IholdIrun ihold_irun;
  ihold_irun.bytes = registers_ptr_->getStored(Registers::IHOLD_IRUN);
  ihold_irun.irun = run_current;
  registers_ptr_->write(Registers::IHOLD_IRUN, ihold_irun.bytes);
}

void Driver::writeHoldCurrent(uint8_t hold_current)
{
  Registers::IholdIrun ihold_irun;
  ihold_irun.bytes = registers_ptr_->getStored(Registers::IHOLD_IRUN);
  ihold_irun.ihold = hold_current;
  registers_ptr_->write(Registers::IHOLD_IRUN, ihold_irun.bytes);
}

void Driver::writeHoldDelay(uint8_t hold_delay)
{
  Registers::IholdIrun ihold_irun;
  ihold_irun.bytes = registers_ptr_->getStored(Registers::IHOLD_IRUN);
  ihold_irun.iholddelay = hold_delay;
  registers_ptr_->write(Registers::IHOLD_IRUN, ihold_irun.bytes);
}

void Driver::writePwmOffset(uint8_t pwm_amplitude)
{
  Registers::Pwmconf pwmconf;
  pwmconf.bytes = registers_ptr_->getStored(Registers::PWMCONF);
  pwmconf.pwm_ofs = pwm_amplitude;
  registers_ptr_->write(Registers::PWMCONF, pwmconf.bytes);
}

void Driver::writePwmGradient(uint8_t pwm_amplitude)
{
  Registers::Pwmconf pwmconf;
  pwmconf.bytes = registers_ptr_->getStored(Registers::PWMCONF);
  pwmconf.pwm_grad = pwm_amplitude;
  registers_ptr_->write(Registers::PWMCONF, pwmconf.bytes);
}

void Driver::enableAutomaticCurrentControl(bool autograd,
    uint8_t pwm_reg)
{
  Registers::Pwmconf pwmconf;
  pwmconf.bytes = registers_ptr_->getStored(Registers::PWMCONF);
  pwmconf.pwm_autoscale = 1;
  pwmconf.pwm_autograd = autograd;
  pwmconf.pwm_reg = pwm_reg;
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

void Driver::writeMotorDirection(MotorDirection motor_direction)
{
  Registers::Gconf gconf;
  gconf.bytes = registers_ptr_->getStored(Registers::GCONF);
  gconf.shaft = motor_direction;
  registers_ptr_->write(Registers::GCONF, gconf.bytes);
}

void Driver::writeStandstillMode(StandstillMode mode)
{
  Registers::Pwmconf pwmconf;
  pwmconf.bytes = registers_ptr_->getStored(Registers::PWMCONF);
  pwmconf.freewheel = mode;
  registers_ptr_->write(Registers::PWMCONF, pwmconf.bytes);
}

void Driver::writeChopperMode(ChopperMode chopper_mode)
{
  Registers::Chopconf chopconf;
  chopconf.bytes = registers_ptr_->getStored(Registers::CHOPCONF);
  chopconf.chm = chopper_mode;
  registers_ptr_->write(Registers::CHOPCONF, chopconf.bytes);
}

void Driver::writeStealthChopThreshold(uint32_t tstep)
{
  registers_ptr_->write(Registers::TPWMTHRS, tstep);
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

void Driver::writeCoolStepThreshold(uint32_t tstep)
{
  registers_ptr_->write(Registers::TCOOLTHRS, tstep);
}

void Driver::enableCoolStep(uint8_t min,
    uint8_t max)
{
  Registers::Coolconf coolconf;
  coolconf.bytes = registers_ptr_->getStored(Registers::COOLCONF);
  coolconf.semin = min;
  coolconf.semax = max;
  registers_ptr_->write(Registers::COOLCONF, coolconf.bytes);
}

void Driver::disableCoolStep()
{
  Registers::Coolconf coolconf;
  coolconf.bytes = registers_ptr_->getStored(Registers::COOLCONF);
  coolconf.semin = SEMIN_OFF;
  registers_ptr_->write(Registers::COOLCONF, coolconf.bytes);
}

void Driver::writeHighVelocityThreshold(uint32_t tstep)
{
  registers_ptr_->write(Registers::THIGH, tstep);
}

void Driver::enableHighVelocityFullstep()
{
  Registers::Chopconf chopconf;
  chopconf.bytes = registers_ptr_->getStored(Registers::CHOPCONF);
  chopconf.vhighfs = 1;
  registers_ptr_->write(Registers::CHOPCONF, chopconf.bytes);
}

void Driver::disableHighVelocityFullstep()
{
  Registers::Chopconf chopconf;
  chopconf.bytes = registers_ptr_->getStored(Registers::CHOPCONF);
  chopconf.vhighfs = 0;
  registers_ptr_->write(Registers::CHOPCONF, chopconf.bytes);
}

void Driver::enableHighVelocityChopperSwitch()
{
  Registers::Chopconf chopconf;
  chopconf.bytes = registers_ptr_->getStored(Registers::CHOPCONF);
  chopconf.vhighchm = 1;
  registers_ptr_->write(Registers::CHOPCONF, chopconf.bytes);
}

void Driver::disableHighVelocityChopperSwitch()
{
  Registers::Chopconf chopconf;
  chopconf.bytes = registers_ptr_->getStored(Registers::CHOPCONF);
  chopconf.vhighchm = 0;
  registers_ptr_->write(Registers::CHOPCONF, chopconf.bytes);
}

void Driver::writeStallGuardThreshold(int8_t threshold)
{
  Registers::Coolconf coolconf;
  coolconf.bytes = registers_ptr_->getStored(Registers::COOLCONF);
  coolconf.sgt = threshold;
  registers_ptr_->write(Registers::COOLCONF, coolconf.bytes);
}

void Driver::enableStallGuardFilter()
{
  Registers::Coolconf coolconf;
  coolconf.bytes = registers_ptr_->getStored(Registers::COOLCONF);
  coolconf.sfilt = STALL_GUARD_FILTER_ENABLE;
  registers_ptr_->write(Registers::COOLCONF, coolconf.bytes);
}

void Driver::disableStallGuardFilter()
{
  Registers::Coolconf coolconf;
  coolconf.bytes = registers_ptr_->getStored(Registers::COOLCONF);
  coolconf.sfilt = STALL_GUARD_FILTER_DISABLE;
  registers_ptr_->write(Registers::COOLCONF, coolconf.bytes);
}

bool Driver::stalled()
{
  Registers::RampStat ramp_stat;
  ramp_stat.bytes = registers_ptr_->read(Registers::RAMP_STAT);
  return ramp_stat.status_sg;
}

uint16_t Driver::readStallGuardResult()
{
  Registers::DrvStatus drv_status;
  drv_status.bytes = registers_ptr_->read(Registers::DRV_STATUS);
  return drv_status.sg_result;
}

uint8_t Driver::readActualCurrentScaling()
{
  Registers::DrvStatus drv_status;
  drv_status.bytes = registers_ptr_->read(Registers::DRV_STATUS);
  return drv_status.cs_actual;
}

void Driver::enableShortToGroundProtection()
{
  Registers::Chopconf chopconf;
  chopconf.bytes = registers_ptr_->getStored(Registers::CHOPCONF);
  chopconf.diss2g = 0;
  registers_ptr_->write(Registers::CHOPCONF, chopconf.bytes);
}

void Driver::disableShortToGroundProtection()
{
  Registers::Chopconf chopconf;
  chopconf.bytes = registers_ptr_->getStored(Registers::CHOPCONF);
  chopconf.diss2g = 1;
  registers_ptr_->write(Registers::CHOPCONF, chopconf.bytes);
}

// private

void Driver::initialize(Registers & registers)
{
  registers_ptr_ = &registers;
  toff_ = TOFF_ENABLE_DEFAULT;

  disable();
}

void Driver::writeDriverParameters(DriverParameters driver_parameters)
{
  writeGlobalCurrentScaler(driver_parameters.global_current_scaler);
  writeRunCurrent(driver_parameters.run_current);
  writeHoldCurrent(driver_parameters.hold_current);
  writeHoldDelay(driver_parameters.hold_delay);
  writePwmOffset(driver_parameters.pwm_offset);
  writePwmGradient(driver_parameters.pwm_gradient);
  if (driver_parameters.automatic_current_control_enabled)
  {
    enableAutomaticCurrentControl();
  }
  else
  {
    disableAutomaticCurrentControl();
  }
  writeMotorDirection(driver_parameters.motor_direction);
  writeStandstillMode(driver_parameters.standstill_mode);
  writeChopperMode(driver_parameters.chopper_mode);
  writeStealthChopThreshold(driver_parameters.stealth_chop_threshold);
  if (driver_parameters.stealth_chop_enabled)
  {
    enableStealthChop();
  }
  else
  {
    disableStealthChop();
  }
  writeCoolStepThreshold(driver_parameters.cool_step_threshold);
  if (driver_parameters.cool_step_enabled)
  {
    enableCoolStep(driver_parameters.cool_step_min, driver_parameters.cool_step_max);
  }
  else
  {
    disableCoolStep();
  }
  writeHighVelocityThreshold(driver_parameters.high_velocity_threshold);
  if (driver_parameters.high_velocity_fullstep_enabled)
  {
    enableHighVelocityFullstep();
  }
  else
  {
    disableHighVelocityFullstep();
  }
  if (driver_parameters.high_velocity_chopper_switch_enabled)
  {
    enableHighVelocityChopperSwitch();
  }
  else
  {
    disableHighVelocityChopperSwitch();
  }
  writeStallGuardThreshold(driver_parameters.stall_guard_threshold);
  if (driver_parameters.stall_guard_filter_enabled)
  {
    enableStallGuardFilter();
  }
  else
  {
    disableStallGuardFilter();
  }
  if (driver_parameters.short_to_ground_protection_enabled)
  {
    enableShortToGroundProtection();
  }
  else
  {
    disableShortToGroundProtection();
  }
}

void Driver::cacheDriverSettings()
{
  cached_driver_settings_.global_current_scaler = registers_ptr_->getStored(Registers::GLOBAL_SCALER);
  Registers::IholdIrun ihold_irun;
  ihold_irun.bytes = registers_ptr_->getStored(Registers::IHOLD_IRUN);
  cached_driver_settings_.run_current = ihold_irun.irun;
  cached_driver_settings_.hold_current = ihold_irun.ihold;
  cached_driver_settings_.hold_delay = ihold_irun.iholddelay;
  Registers::Pwmconf pwmconf;
  pwmconf.bytes = registers_ptr_->getStored(Registers::PWMCONF);
  cached_driver_settings_.pwm_offset = pwmconf.pwm_ofs;
  cached_driver_settings_.pwm_gradient = pwmconf.pwm_grad;
  cached_driver_settings_.automatic_current_control_enabled = pwmconf.pwm_autoscale;
  Registers::Gconf gconf;
  gconf.bytes = registers_ptr_->getStored(Registers::GCONF);
  cached_driver_settings_.motor_direction = gconf.shaft;
  cached_driver_settings_.standstill_mode = pwmconf.freewheel;
  Registers::Chopconf chopconf;
  chopconf.bytes = registers_ptr_->getStored(Registers::CHOPCONF);
  cached_driver_settings_.chopper_mode = chopconf.chm;
  cached_driver_settings_.stealth_chop_threshold = registers_ptr_->getStored(Registers::TPWMTHRS);
  cached_driver_settings_.stealth_chop_enabled = gconf.en_pwm_mode;
  cached_driver_settings_.cool_step_threshold = registers_ptr_->getStored(Registers::TCOOLTHRS);
  Registers::Coolconf coolconf;
  coolconf.bytes = registers_ptr_->getStored(Registers::COOLCONF);
  cached_driver_settings_.cool_step_min = coolconf.semin;
  cached_driver_settings_.cool_step_max = coolconf.semax;
  cached_driver_settings_.cool_step_enabled = not (coolconf.semin == SEMIN_OFF);
  cached_driver_settings_.high_velocity_threshold = registers_ptr_->getStored(Registers::THIGH);
  cached_driver_settings_.high_velocity_fullstep_enabled = chopconf.vhighfs;
  cached_driver_settings_.high_velocity_chopper_switch_enabled = chopconf.vhighchm;
  cached_driver_settings_.stall_guard_threshold = coolconf.sgt;
  cached_driver_settings_.stall_guard_filter_enabled = coolconf.sfilt;
  cached_driver_settings_.short_to_ground_protection_enabled = chopconf.diss2g;
}

void Driver::restoreDriverSettings()
{
  writeDriverParameters(cached_driver_settings_);
}

void Driver::hardwareEnable()
{
  if (hardware_enable_pin_ != NO_PIN)
  {
    digitalWrite(hardware_enable_pin_, LOW);
  }
}

void Driver::hardwareDisable()
{
  if (hardware_enable_pin_ != NO_PIN)
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
