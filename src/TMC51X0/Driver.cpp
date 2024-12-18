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
  registers_ptr_ = nullptr;
  setup_driver_parameters_ = DriverParameters{};
  cached_driver_settings_ = DriverParameters{};
  hardware_enable_pin_ = NO_PIN;
}

void Driver::setup()
{
  writeDriverParameters(setup_driver_parameters_);
}

void Driver::setup(tmc51x0::DriverParameters parameters)
{
  setup_driver_parameters_ = parameters;
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

void Driver::writeComparatorBlankTime(ComparatorBlankTime tbl)
{
  Registers::Chopconf chopconf;
  chopconf.bytes = registers_ptr_->getStored(Registers::CHOPCONF);
  chopconf.tbl = tbl;
  registers_ptr_->write(Registers::CHOPCONF, chopconf.bytes);
}

void Driver::writeEnabledToff(uint8_t toff)
{
  if (toff < ENABLED_TOFF_MIN)
  {
    toff = ENABLED_TOFF_MIN;
  }
  if (toff > ENABLED_TOFF_MAX)
  {
    toff = ENABLED_TOFF_MAX;
  }
  enabled_toff_ = toff;
  Registers::Chopconf chopconf;
  chopconf.bytes = registers_ptr_->getStored(Registers::CHOPCONF);
  chopconf.toff = enabled_toff_;
  registers_ptr_->write(Registers::CHOPCONF, chopconf.bytes);
}

void Driver::writeDcTime(ComparatorBlankTime tbl)
{
  uint16_t dc_time;
  switch (tbl)
  {
    case CLOCK_CYCLES_16:
      dc_time = 17;
      break;
    case CLOCK_CYCLES_24:
      dc_time = 25;
      break;
    case CLOCK_CYCLES_36:
      dc_time = 37;
      break;
    case CLOCK_CYCLES_54:
      dc_time = 55;
      break;
  }
  writeDcTime(dc_time);
}

void Driver::writeDcTime(uint16_t dc_time)
{
  Registers::DcCtrl dc_ctrl;
  dc_ctrl.bytes = registers_ptr_->getStored(Registers::DCCTRL);
  dc_ctrl.dc_time = dc_time;
  registers_ptr_->write(Registers::DCCTRL, dc_ctrl.bytes);
}

// private

void Driver::initialize(Registers & registers)
{
  registers_ptr_ = &registers;

  reinitialize();
}

void Driver::reinitialize()
{
  enabled_toff_ = DriverParameters::ENABLED_TOFF_DEFAULT;
  disable();
  setup();
}

void Driver::writeDriverParameters(DriverParameters parameters)
{
  writeGlobalCurrentScaler(parameters.global_current_scaler);
  writeRunCurrent(parameters.run_current);
  writeHoldCurrent(parameters.hold_current);
  writeHoldDelay(parameters.hold_delay);
  writePwmOffset(parameters.pwm_offset);
  writePwmGradient(parameters.pwm_gradient);
  if (parameters.automatic_current_control_enabled)
  {
    enableAutomaticCurrentControl();
  }
  else
  {
    disableAutomaticCurrentControl();
  }
  writeMotorDirection(parameters.motor_direction);
  writeStandstillMode(parameters.standstill_mode);
  writeChopperMode(parameters.chopper_mode);
  writeStealthChopThreshold(parameters.stealth_chop_threshold);
  if (parameters.stealth_chop_enabled)
  {
    enableStealthChop();
  }
  else
  {
    disableStealthChop();
  }
  writeCoolStepThreshold(parameters.cool_step_threshold);
  if (parameters.cool_step_enabled)
  {
    enableCoolStep(parameters.cool_step_min, parameters.cool_step_max);
  }
  else
  {
    disableCoolStep();
  }
  writeHighVelocityThreshold(parameters.high_velocity_threshold);
  if (parameters.high_velocity_fullstep_enabled)
  {
    enableHighVelocityFullstep();
  }
  else
  {
    disableHighVelocityFullstep();
  }
  if (parameters.high_velocity_chopper_switch_enabled)
  {
    enableHighVelocityChopperSwitch();
  }
  else
  {
    disableHighVelocityChopperSwitch();
  }
  writeStallGuardThreshold(parameters.stall_guard_threshold);
  if (parameters.stall_guard_filter_enabled)
  {
    enableStallGuardFilter();
  }
  else
  {
    disableStallGuardFilter();
  }
  if (parameters.short_to_ground_protection_enabled)
  {
    enableShortToGroundProtection();
  }
  else
  {
    disableShortToGroundProtection();
  }
  writeComparatorBlankTime(parameters.comparator_blank_time);
  writeEnabledToff(parameters.enabled_toff);
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
  cached_driver_settings_.motor_direction = (MotorDirection)gconf.shaft;
  cached_driver_settings_.standstill_mode = (StandstillMode)pwmconf.freewheel;
  Registers::Chopconf chopconf;
  chopconf.bytes = registers_ptr_->getStored(Registers::CHOPCONF);
  cached_driver_settings_.chopper_mode = (ChopperMode)chopconf.chm;
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
  cached_driver_settings_.comparator_blank_time = (ComparatorBlankTime)chopconf.tbl;
  cached_driver_settings_.enabled_toff = enabled_toff_;
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
  chopconf.toff = enabled_toff_;
  registers_ptr_->write(Registers::CHOPCONF, chopconf.bytes);
}

void Driver::softwareDisable()
{
  Registers::Chopconf chopconf;
  chopconf.bytes = registers_ptr_->getStored(Registers::CHOPCONF);
  chopconf.toff = DISABLE_TOFF;
  registers_ptr_->write(Registers::CHOPCONF, chopconf.bytes);
}
