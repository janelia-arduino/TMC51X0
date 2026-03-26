// ----------------------------------------------------------------------------
// Driver.cpp
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------
#include "Driver.hpp"

using namespace tmc51x0;

Driver::Driver() {
  registers_ptr_ = nullptr;
  setup_driver_parameters_ = DriverParameters{};
  cached_driver_settings_ = DriverParameters{};
  hardware_enable_pin_ = NO_PIN;
  setup_pwm_autograd_ = true;
  setup_pwm_reg_ = 4;
  cached_pwm_autograd_ = true;
  cached_pwm_reg_ = 4;
}

void Driver::setup() {
  writeDriverParameters(setup_driver_parameters_);
}

void Driver::setup(tmc51x0::DriverParameters parameters) {
  setup_driver_parameters_ = parameters;
  setup();
}

void Driver::setEnableHardwarePin(size_t hardware_enable_pin) {
  hardware_enable_pin_ = hardware_enable_pin;
  pinMode(hardware_enable_pin_, OUTPUT);
  hardwareDisable();
}

void Driver::enable() {
  hardwareEnable();
  softwareEnable();
}

void Driver::disable() {
  hardwareDisable();
  softwareDisable();
}

void Driver::writeGlobalCurrentScaler(uint8_t scaler) {
  setup_driver_parameters_.global_current_scaler = scaler;
  registers_ptr_->write(Registers::GlobalScalerAddress, scaler);
}

void Driver::writeRunCurrent(uint8_t run_current) {
  setup_driver_parameters_.run_current = run_current;
  Registers::IholdIrun ihold_irun;
  ihold_irun.raw = registers_ptr_->getStored(Registers::IholdIrunAddress);
  ihold_irun.irun(run_current);
  registers_ptr_->write(Registers::IholdIrunAddress, ihold_irun.raw);
}

void Driver::writeHoldCurrent(uint8_t hold_current) {
  setup_driver_parameters_.hold_current = hold_current;
  Registers::IholdIrun ihold_irun;
  ihold_irun.raw = registers_ptr_->getStored(Registers::IholdIrunAddress);
  ihold_irun.ihold(hold_current);
  registers_ptr_->write(Registers::IholdIrunAddress, ihold_irun.raw);
}

void Driver::writeHoldDelay(uint8_t hold_delay) {
  setup_driver_parameters_.hold_delay = hold_delay;
  Registers::IholdIrun ihold_irun;
  ihold_irun.raw = registers_ptr_->getStored(Registers::IholdIrunAddress);
  ihold_irun.iholddelay(hold_delay);
  registers_ptr_->write(Registers::IholdIrunAddress, ihold_irun.raw);
}

void Driver::writePwmOffset(uint8_t pwm_amplitude) {
  setup_driver_parameters_.pwm_offset = pwm_amplitude;
  Registers::Pwmconf pwmconf;
  pwmconf.raw = registers_ptr_->getStored(Registers::PwmconfAddress);
  pwmconf.pwm_ofs(pwm_amplitude);
  registers_ptr_->write(Registers::PwmconfAddress, pwmconf.raw);
}

void Driver::writePwmGradient(uint8_t pwm_amplitude) {
  setup_driver_parameters_.pwm_gradient = pwm_amplitude;
  Registers::Pwmconf pwmconf;
  pwmconf.raw = registers_ptr_->getStored(Registers::PwmconfAddress);
  pwmconf.pwm_grad(pwm_amplitude);
  registers_ptr_->write(Registers::PwmconfAddress, pwmconf.raw);
}

void Driver::enableAutomaticCurrentControl(bool autograd, uint8_t pwm_reg) {
  setup_driver_parameters_.automatic_current_control_enabled = true;
  setup_pwm_autograd_ = autograd;
  setup_pwm_reg_ = pwm_reg;
  Registers::Pwmconf pwmconf;
  pwmconf.raw = registers_ptr_->getStored(Registers::PwmconfAddress);
  pwmconf.pwm_autoscale(true);
  pwmconf.pwm_autograd(autograd);
  pwmconf.pwm_reg(pwm_reg);
  registers_ptr_->write(Registers::PwmconfAddress, pwmconf.raw);
}

void Driver::disableAutomaticCurrentControl() {
  setup_driver_parameters_.automatic_current_control_enabled = false;
  Registers::Pwmconf pwmconf;
  pwmconf.raw = registers_ptr_->getStored(Registers::PwmconfAddress);
  pwmconf.pwm_autoscale(false);
  pwmconf.pwm_autograd(false);
  registers_ptr_->write(Registers::PwmconfAddress, pwmconf.raw);
}

void Driver::writeMotorDirection(MotorDirection motor_direction) {
  setup_driver_parameters_.motor_direction = motor_direction;
  Registers::Gconf gconf;
  gconf.raw = registers_ptr_->getStored(Registers::GconfAddress);
  gconf.shaft(motor_direction);
  registers_ptr_->write(Registers::GconfAddress, gconf.raw);
}

void Driver::writeStandstillMode(StandstillMode mode) {
  setup_driver_parameters_.standstill_mode = mode;
  Registers::Pwmconf pwmconf;
  pwmconf.raw = registers_ptr_->getStored(Registers::PwmconfAddress);
  pwmconf.freewheel(mode);
  registers_ptr_->write(Registers::PwmconfAddress, pwmconf.raw);
}

void Driver::writeChopperMode(ChopperMode chopper_mode) {
  setup_driver_parameters_.chopper_mode = chopper_mode;
  Registers::Chopconf chopconf;
  chopconf.raw = registers_ptr_->getStored(Registers::ChopconfAddress);
  chopconf.chm(chopper_mode);
  registers_ptr_->write(Registers::ChopconfAddress, chopconf.raw);
}

void Driver::writeStealthChopThreshold(uint32_t tstep) {
  setup_driver_parameters_.stealth_chop_threshold = tstep;
  registers_ptr_->write(Registers::TpwmthrsAddress, tstep);
}

void Driver::enableStealthChop() {
  setup_driver_parameters_.stealth_chop_enabled = true;
  Registers::Gconf gconf;
  gconf.raw = registers_ptr_->getStored(Registers::GconfAddress);
  gconf.en_pwm_mode(true);
  registers_ptr_->write(Registers::GconfAddress, gconf.raw);
}

void Driver::disableStealthChop() {
  setup_driver_parameters_.stealth_chop_enabled = false;
  Registers::Gconf gconf;
  gconf.raw = registers_ptr_->getStored(Registers::GconfAddress);
  gconf.en_pwm_mode(false);
  registers_ptr_->write(Registers::GconfAddress, gconf.raw);
}

void Driver::writeCoolStepThreshold(uint32_t tstep) {
  setup_driver_parameters_.cool_step_threshold = tstep;
  registers_ptr_->write(Registers::TcoolthrsAddress, tstep);
}

void Driver::enableCoolStep(uint8_t min, uint8_t max) {
  setup_driver_parameters_.cool_step_enabled = true;
  setup_driver_parameters_.cool_step_min = min;
  setup_driver_parameters_.cool_step_max = max;
  Registers::Coolconf coolconf;
  coolconf.raw = registers_ptr_->getStored(Registers::CoolconfAddress);
  coolconf.semin(min);
  coolconf.semax(max);
  registers_ptr_->write(Registers::CoolconfAddress, coolconf.raw);
}

void Driver::disableCoolStep() {
  setup_driver_parameters_.cool_step_enabled = false;
  Registers::Coolconf coolconf;
  coolconf.raw = registers_ptr_->getStored(Registers::CoolconfAddress);
  coolconf.semin(SEMIN_OFF);
  registers_ptr_->write(Registers::CoolconfAddress, coolconf.raw);
}

void Driver::writeHighVelocityThreshold(uint32_t tstep) {
  setup_driver_parameters_.high_velocity_threshold = tstep;
  registers_ptr_->write(Registers::ThighAddress, tstep);
}

void Driver::enableHighVelocityFullstep() {
  setup_driver_parameters_.high_velocity_fullstep_enabled = true;
  Registers::Chopconf chopconf;
  chopconf.raw = registers_ptr_->getStored(Registers::ChopconfAddress);
  chopconf.vhighfs(true);
  registers_ptr_->write(Registers::ChopconfAddress, chopconf.raw);
}

void Driver::disableHighVelocityFullstep() {
  setup_driver_parameters_.high_velocity_fullstep_enabled = false;
  Registers::Chopconf chopconf;
  chopconf.raw = registers_ptr_->getStored(Registers::ChopconfAddress);
  chopconf.vhighfs(false);
  registers_ptr_->write(Registers::ChopconfAddress, chopconf.raw);
}

void Driver::enableHighVelocityChopperSwitch() {
  setup_driver_parameters_.high_velocity_chopper_switch_enabled = true;
  Registers::Chopconf chopconf;
  chopconf.raw = registers_ptr_->getStored(Registers::ChopconfAddress);
  chopconf.vhighchm(true);
  registers_ptr_->write(Registers::ChopconfAddress, chopconf.raw);
}

void Driver::disableHighVelocityChopperSwitch() {
  setup_driver_parameters_.high_velocity_chopper_switch_enabled = false;
  Registers::Chopconf chopconf;
  chopconf.raw = registers_ptr_->getStored(Registers::ChopconfAddress);
  chopconf.vhighchm(false);
  registers_ptr_->write(Registers::ChopconfAddress, chopconf.raw);
}

void Driver::writeStallGuardThreshold(int8_t threshold) {
  setup_driver_parameters_.stall_guard_threshold = threshold;
  Registers::Coolconf coolconf;
  coolconf.raw = registers_ptr_->getStored(Registers::CoolconfAddress);
  coolconf.sgt(threshold);
  registers_ptr_->write(Registers::CoolconfAddress, coolconf.raw);
}

void Driver::enableStallGuardFilter() {
  setup_driver_parameters_.stall_guard_filter_enabled = true;
  Registers::Coolconf coolconf;
  coolconf.raw = registers_ptr_->getStored(Registers::CoolconfAddress);
  coolconf.sfilt(STALL_GUARD_FILTER_ENABLE);
  registers_ptr_->write(Registers::CoolconfAddress, coolconf.raw);
}

void Driver::disableStallGuardFilter() {
  setup_driver_parameters_.stall_guard_filter_enabled = false;
  Registers::Coolconf coolconf;
  coolconf.raw = registers_ptr_->getStored(Registers::CoolconfAddress);
  coolconf.sfilt(STALL_GUARD_FILTER_DISABLE);
  registers_ptr_->write(Registers::CoolconfAddress, coolconf.raw);
}

bool Driver::stalled() {
  Registers::RampStat ramp_stat;
  ramp_stat.raw = registers_ptr_->read(Registers::RampStatAddress);
  return ramp_stat.status_sg();
}

uint16_t Driver::readStallGuardResult() {
  Registers::DrvStatus drv_status;
  drv_status.raw = registers_ptr_->read(Registers::DrvStatusAddress);
  return drv_status.sg_result();
}

uint8_t Driver::readActualCurrentScaling() {
  Registers::DrvStatus drv_status;
  drv_status.raw = registers_ptr_->read(Registers::DrvStatusAddress);
  return drv_status.cs_actual();
}

void Driver::enableShortToGroundProtection() {
  setup_driver_parameters_.short_to_ground_protection_enabled = true;
  Registers::Chopconf chopconf;
  chopconf.raw = registers_ptr_->getStored(Registers::ChopconfAddress);
  chopconf.diss2g(false);
  registers_ptr_->write(Registers::ChopconfAddress, chopconf.raw);
}

void Driver::disableShortToGroundProtection() {
  setup_driver_parameters_.short_to_ground_protection_enabled = false;
  Registers::Chopconf chopconf;
  chopconf.raw = registers_ptr_->getStored(Registers::ChopconfAddress);
  chopconf.diss2g(true);
  registers_ptr_->write(Registers::ChopconfAddress, chopconf.raw);
}

void Driver::writeComparatorBlankTime(ComparatorBlankTime tbl) {
  setup_driver_parameters_.comparator_blank_time = tbl;
  Registers::Chopconf chopconf;
  chopconf.raw = registers_ptr_->getStored(Registers::ChopconfAddress);
  chopconf.tbl(tbl);
  registers_ptr_->write(Registers::ChopconfAddress, chopconf.raw);
}

void Driver::writeEnabledToff(uint8_t toff) {
  if (toff < ENABLED_TOFF_MIN) {
    toff = ENABLED_TOFF_MIN;
  }
  if (toff > ENABLED_TOFF_MAX) {
    toff = ENABLED_TOFF_MAX;
  }
  setup_driver_parameters_.enabled_toff = toff;
  enabled_toff_ = toff;
  Registers::Chopconf chopconf;
  chopconf.raw = registers_ptr_->getStored(Registers::ChopconfAddress);
  chopconf.toff(enabled_toff_);
  registers_ptr_->write(Registers::ChopconfAddress, chopconf.raw);
}

void Driver::writeDcTime(uint16_t dc_time) {
  setup_driver_parameters_.dc_time = dc_time;
  Registers::Dcctrl dcctrl;
  dcctrl.raw = registers_ptr_->getStored(Registers::DcctrlAddress);
  dcctrl.dc_time(dc_time);
  registers_ptr_->write(Registers::DcctrlAddress, dcctrl.raw);
}

void Driver::writeDcStallGuardThreshold(uint8_t dc_stall_guard_threshold) {
  setup_driver_parameters_.dc_stall_guard_threshold = dc_stall_guard_threshold;
  Registers::Dcctrl dcctrl;
  dcctrl.raw = registers_ptr_->getStored(Registers::DcctrlAddress);
  dcctrl.dc_sg(dc_stall_guard_threshold);
  registers_ptr_->write(Registers::DcctrlAddress, dcctrl.raw);
}

// private

void Driver::initialize(Registers& registers) {
  registers_ptr_ = &registers;

  reinitialize();
}

void Driver::reinitialize() {
  enabled_toff_ = ENABLED_TOFF_DEFAULT;
  disable();
  setup();
}

void Driver::writeDriverParameters(DriverParameters parameters) {
  setup_driver_parameters_ = parameters;
  writeGlobalCurrentScaler(parameters.global_current_scaler);
  writeRunCurrent(parameters.run_current);
  writeHoldCurrent(parameters.hold_current);
  writeHoldDelay(parameters.hold_delay);
  writePwmOffset(parameters.pwm_offset);
  writePwmGradient(parameters.pwm_gradient);
  if (parameters.automatic_current_control_enabled) {
    enableAutomaticCurrentControl(setup_pwm_autograd_, setup_pwm_reg_);
  } else {
    disableAutomaticCurrentControl();
  }
  writeMotorDirection(parameters.motor_direction);
  writeStandstillMode(parameters.standstill_mode);
  writeChopperMode(parameters.chopper_mode);
  writeStealthChopThreshold(parameters.stealth_chop_threshold);
  if (parameters.stealth_chop_enabled) {
    enableStealthChop();
  } else {
    disableStealthChop();
  }
  writeCoolStepThreshold(parameters.cool_step_threshold);
  if (parameters.cool_step_enabled) {
    enableCoolStep(parameters.cool_step_min, parameters.cool_step_max);
  } else {
    disableCoolStep();
  }
  writeHighVelocityThreshold(parameters.high_velocity_threshold);
  if (parameters.high_velocity_fullstep_enabled) {
    enableHighVelocityFullstep();
  } else {
    disableHighVelocityFullstep();
  }
  if (parameters.high_velocity_chopper_switch_enabled) {
    enableHighVelocityChopperSwitch();
  } else {
    disableHighVelocityChopperSwitch();
  }
  writeStallGuardThreshold(parameters.stall_guard_threshold);
  if (parameters.stall_guard_filter_enabled) {
    enableStallGuardFilter();
  } else {
    disableStallGuardFilter();
  }
  if (parameters.short_to_ground_protection_enabled) {
    enableShortToGroundProtection();
  } else {
    disableShortToGroundProtection();
  }
  writeEnabledToff(parameters.enabled_toff);
  writeComparatorBlankTime(parameters.comparator_blank_time);
  writeDcTime(parameters.dc_time);
  writeDcStallGuardThreshold(parameters.dc_stall_guard_threshold);
}

void Driver::cacheDriverSettings() {
  // Refresh the readable pieces of the mirror first so cache snapshots are as
  // close to the live chip state as the transport allows.
  (void)registers_ptr_->read(Registers::GconfAddress);
  (void)registers_ptr_->read(Registers::ChopconfAddress);

  cached_driver_settings_.global_current_scaler =
    registers_ptr_->getStored(Registers::GlobalScalerAddress);
  Registers::IholdIrun ihold_irun;
  ihold_irun.raw = registers_ptr_->getStored(Registers::IholdIrunAddress);
  cached_driver_settings_.run_current = ihold_irun.irun();
  cached_driver_settings_.hold_current = ihold_irun.ihold();
  cached_driver_settings_.hold_delay = ihold_irun.iholddelay();
  Registers::Pwmconf pwmconf;
  pwmconf.raw = registers_ptr_->getStored(Registers::PwmconfAddress);
  cached_driver_settings_.pwm_offset = pwmconf.pwm_ofs();
  cached_driver_settings_.pwm_gradient = pwmconf.pwm_grad();
  cached_driver_settings_.automatic_current_control_enabled =
    pwmconf.pwm_autoscale();
  cached_pwm_autograd_ = pwmconf.pwm_autograd();
  cached_pwm_reg_ = pwmconf.pwm_reg();
  Registers::Gconf gconf;
  gconf.raw = registers_ptr_->getStored(Registers::GconfAddress);
  cached_driver_settings_.motor_direction = (MotorDirection)gconf.shaft();
  cached_driver_settings_.standstill_mode = (StandstillMode)pwmconf.freewheel();
  Registers::Chopconf chopconf;
  chopconf.raw = registers_ptr_->getStored(Registers::ChopconfAddress);
  cached_driver_settings_.chopper_mode = (ChopperMode)chopconf.chm();
  cached_driver_settings_.stealth_chop_threshold =
    registers_ptr_->getStored(Registers::TpwmthrsAddress);
  cached_driver_settings_.stealth_chop_enabled = gconf.en_pwm_mode();
  cached_driver_settings_.cool_step_threshold =
    registers_ptr_->getStored(Registers::TcoolthrsAddress);
  Registers::Coolconf coolconf;
  coolconf.raw = registers_ptr_->getStored(Registers::CoolconfAddress);
  cached_driver_settings_.cool_step_min = coolconf.semin();
  cached_driver_settings_.cool_step_max = coolconf.semax();
  cached_driver_settings_.cool_step_enabled =
    not(coolconf.semin() == SEMIN_OFF);
  cached_driver_settings_.high_velocity_threshold =
    registers_ptr_->getStored(Registers::ThighAddress);
  cached_driver_settings_.high_velocity_fullstep_enabled = chopconf.vhighfs();
  cached_driver_settings_.high_velocity_chopper_switch_enabled =
    chopconf.vhighchm();
  cached_driver_settings_.stall_guard_threshold = coolconf.sgt();
  cached_driver_settings_.stall_guard_filter_enabled = coolconf.sfilt();
  cached_driver_settings_.short_to_ground_protection_enabled =
    !chopconf.diss2g();
  cached_driver_settings_.enabled_toff = enabled_toff_;
  cached_driver_settings_.comparator_blank_time =
    (ComparatorBlankTime)chopconf.tbl();
  Registers::Dcctrl dcctrl;
  dcctrl.raw = registers_ptr_->getStored(Registers::DcctrlAddress);
  cached_driver_settings_.dc_time = dcctrl.dc_time();
  cached_driver_settings_.dc_stall_guard_threshold = dcctrl.dc_sg();
}

void Driver::restoreDriverSettings() {
  setup_pwm_autograd_ = cached_pwm_autograd_;
  setup_pwm_reg_ = cached_pwm_reg_;
  writeDriverParameters(cached_driver_settings_);
}

void Driver::hardwareEnable() {
  if (hardware_enable_pin_ != NO_PIN) {
    digitalWrite(hardware_enable_pin_, LOW);
  }
}

void Driver::hardwareDisable() {
  if (hardware_enable_pin_ != NO_PIN) {
    digitalWrite(hardware_enable_pin_, HIGH);
  }
}

void Driver::softwareEnable() {
  Registers::Chopconf chopconf;
  chopconf.raw = registers_ptr_->getStored(Registers::ChopconfAddress);
  chopconf.toff(enabled_toff_);
  registers_ptr_->write(Registers::ChopconfAddress, chopconf.raw);
}

void Driver::softwareDisable() {
  Registers::Chopconf chopconf;
  chopconf.raw = registers_ptr_->getStored(Registers::ChopconfAddress);
  chopconf.toff(DISABLE_TOFF);
  registers_ptr_->write(Registers::ChopconfAddress, chopconf.raw);
}
