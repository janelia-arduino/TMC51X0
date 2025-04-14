// ----------------------------------------------------------------------------
// Controller.cpp
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------
#include "Controller.hpp"


using namespace tmc51x0;

Controller::Controller()
{
  registers_ptr_ = nullptr;
  setup_controller_parameters_ = ControllerParameters{};
  cached_controller_settings_ = ControllerParameters{};
  setup_switch_parameters_ = SwitchParameters{};
  cached_switch_settings_ = SwitchParameters{};
}

void Controller::setup()
{
  writeControllerParameters(setup_controller_parameters_);
}

void Controller::setup(tmc51x0::ControllerParameters parameters)
{
  setup_controller_parameters_ = parameters;
  setup();
}

bool Controller::stepAndDirectionMode()
{
  Registers::Ioin ioin;
  ioin.bytes = registers_ptr_->read(Registers::IoinAddress);
  return ioin.sd_mode;
}

void Controller::writeRampMode(RampMode ramp_mode)
{
  registers_ptr_->write(Registers::RampmodeAddress, ramp_mode);
}

void Controller::writeStopMode(StopMode stop_mode)
{
  Registers::SwMode sw_mode;
  sw_mode.bytes = registers_ptr_->getStored(Registers::SwModeAddress);
  sw_mode.en_softstop = stop_mode;
  registers_ptr_->write(Registers::SwModeAddress, sw_mode.bytes);
}

uint32_t Controller::readTstep()
{
  return registers_ptr_->read(Registers::TstepAddress);
}

int32_t Controller::readActualPosition()
{
  return registers_ptr_->read(Registers::XactualAddress);
}

void Controller::writeActualPosition(int32_t position)
{
  return registers_ptr_->write(Registers::XactualAddress, position);
}

int32_t Controller::readActualVelocity()
{
  int32_t actual_velocity = registers_ptr_->read(Registers::VactualAddress);
  if (actual_velocity > MAX_POSITIVE_VELOCITY)
  {
    actual_velocity -= VELOCITY_SIGN_CONVERSION;
  }
  return actual_velocity;
}

void Controller::zeroActualPosition()
{
  return writeActualPosition(0);
}

bool Controller::velocityReached()
{
  Registers::RampStat ramp_stat;
  ramp_stat.bytes = registers_ptr_->read(Registers::RampStatAddress);
  return ramp_stat.velocity_reached;
}

bool Controller::positionReached()
{
  Registers::RampStat ramp_stat;
  ramp_stat.bytes = registers_ptr_->read(Registers::RampStatAddress);
  return ramp_stat.position_reached;
}

void Controller::beginRampToZeroVelocity()
{
  cacheControllerSettings();
  writeStartVelocity(0);
  writeMaxVelocity(0);
}

void Controller::endRampToZeroVelocity()
{
  restoreControllerSettings();
}

bool Controller::zeroVelocity()
{
  Registers::RampStat ramp_stat;
  ramp_stat.bytes = registers_ptr_->read(Registers::RampStatAddress);
  return ramp_stat.vzero;
}

void Controller::writeMaxVelocity(uint32_t velocity)
{
  registers_ptr_->write(Registers::VmaxAddress, velocity);
}

void Controller::writeMaxAcceleration(uint32_t acceleration)
{
  registers_ptr_->write(Registers::AmaxAddress, acceleration);
}

void Controller::writeStartVelocity(uint32_t velocity)
{
  registers_ptr_->write(Registers::VstartAddress, velocity);
}

void Controller::writeStopVelocity(uint32_t velocity)
{
  registers_ptr_->write(Registers::VstopAddress, velocity);
}

void Controller::writeFirstAcceleration(uint32_t acceleration)
{
  registers_ptr_->write(Registers::Acceleration1Address, acceleration);
}

void Controller::writeFirstVelocity(uint32_t velocity)
{
  registers_ptr_->write(Registers::Velocity1Address, velocity);
}

void Controller::writeMaxDeceleration(uint32_t deceleration)
{
  registers_ptr_->write(Registers::DmaxAddress, deceleration);
}

void Controller::writeFirstDeceleration(uint32_t deceleration)
{
  registers_ptr_->write(Registers::Deceleration1Address, deceleration);
}

void Controller::writeZeroWaitDuration(uint32_t tzerowait)
{
  registers_ptr_->write(Registers::TzerowaitAddress, tzerowait);
}

bool Controller::zeroWaitActive()
{
  Registers::RampStat ramp_stat;
  ramp_stat.bytes = registers_ptr_->read(Registers::RampStatAddress);
  return ramp_stat.t_zerowait_active;
}

int32_t Controller::readTargetPosition()
{
  return registers_ptr_->read(Registers::XtargetAddress);
}

void Controller::writeTargetPosition(int32_t position)
{
  registers_ptr_->write(Registers::XtargetAddress, position);
}

void Controller::zeroTargetPosition()
{
  return writeTargetPosition(0);
}

void Controller::writeComparePosition(int32_t position)
{
  registers_ptr_->write(Registers::XCompareAddress, position);
}

void Controller::enableStallStop()
{
  Registers::SwMode sw_mode;
  sw_mode.bytes = registers_ptr_->getStored(Registers::SwModeAddress);
  sw_mode.sg_stop = 1;
  registers_ptr_->write(Registers::SwModeAddress, sw_mode.bytes);
}

void Controller::disableStallStop()
{
  Registers::SwMode sw_mode;
  sw_mode.bytes = registers_ptr_->getStored(Registers::SwModeAddress);
  sw_mode.sg_stop = 0;
  registers_ptr_->write(Registers::SwModeAddress, sw_mode.bytes);
}

void Controller::writeMinDcStepVelocity(uint32_t velocity)
{
  registers_ptr_->write(Registers::VdcminAddress, velocity);
}


void Controller::setupSwitches()
{
  writeSwitchParameters(setup_switch_parameters_);
}

void Controller::setupSwitches(SwitchParameters parameters)
{
  setup_switch_parameters_ = parameters;
  setupSwitches();
}

bool Controller::leftSwitchActive()
{
  Registers::RampStat ramp_stat;
  ramp_stat.bytes = registers_ptr_->read(Registers::RampStatAddress);
  return ramp_stat.status_stop_l;
}

bool Controller::rightSwitchActive()
{
  Registers::RampStat ramp_stat;
  ramp_stat.bytes = registers_ptr_->read(Registers::RampStatAddress);
  return ramp_stat.status_stop_r;
}

bool Controller::leftLatchActive()
{
  Registers::RampStat ramp_stat;
  ramp_stat.bytes = registers_ptr_->read(Registers::RampStatAddress);
  return ramp_stat.status_latch_l;
}

bool Controller::rightLatchActive()
{
  Registers::RampStat ramp_stat;
  ramp_stat.bytes = registers_ptr_->read(Registers::RampStatAddress);
  return ramp_stat.status_latch_r;
}

bool Controller::leftStopEvent()
{
  Registers::RampStat ramp_stat;
  ramp_stat.bytes = registers_ptr_->read(Registers::RampStatAddress);
  return ramp_stat.event_stop_l;
}

bool Controller::rightStopEvent()
{
  Registers::RampStat ramp_stat;
  ramp_stat.bytes = registers_ptr_->read(Registers::RampStatAddress);
  return ramp_stat.event_stop_r;
}

// private

void Controller::initialize(Registers & registers)
{
  registers_ptr_ = &registers;

  reinitialize();
}

void Controller::reinitialize()
{
  zeroActualPosition();
  zeroTargetPosition();
  disableStallStop();
  setup();
  setupSwitches();
}

void Controller::writeControllerParameters(ControllerParameters parameters)
{
  writeRampMode(parameters.ramp_mode);
  writeStopMode(parameters.stop_mode);
  writeMaxVelocity(parameters.max_velocity);
  writeMaxAcceleration(parameters.max_acceleration);
  writeStartVelocity(parameters.start_velocity);
  writeStopVelocity(parameters.stop_velocity);
  writeFirstVelocity(parameters.first_velocity);
  writeFirstAcceleration(parameters.first_acceleration);
  writeMaxDeceleration(parameters.max_deceleration);
  writeFirstDeceleration(parameters.first_deceleration);
  writeZeroWaitDuration(parameters.zero_wait_duration);
  if (parameters.stall_stop_enabled)
  {
    enableStallStop();
  }
  else
  {
    disableStallStop();
  }
  writeMinDcStepVelocity(parameters.min_dc_step_velocity);
}

void Controller::cacheControllerSettings()
{
  cached_controller_settings_.ramp_mode = (RampMode)registers_ptr_->getStored(Registers::RampmodeAddress);
  Registers::SwMode sw_mode;
  sw_mode.bytes = registers_ptr_->getStored(Registers::SwModeAddress);
  cached_controller_settings_.stop_mode = (StopMode)sw_mode.en_softstop;
  cached_controller_settings_.max_velocity = registers_ptr_->getStored(Registers::VmaxAddress);
  cached_controller_settings_.max_acceleration = registers_ptr_->getStored(Registers::AmaxAddress);
  cached_controller_settings_.start_velocity = registers_ptr_->getStored(Registers::VstartAddress);
  cached_controller_settings_.stop_velocity = registers_ptr_->getStored(Registers::VstopAddress);
  cached_controller_settings_.first_velocity = registers_ptr_->getStored(Registers::Velocity1Address);
  cached_controller_settings_.first_acceleration = registers_ptr_->getStored(Registers::Acceleration1Address);
  cached_controller_settings_.max_deceleration = registers_ptr_->getStored(Registers::DmaxAddress);
  cached_controller_settings_.first_deceleration = registers_ptr_->getStored(Registers::Deceleration1Address);
  cached_controller_settings_.zero_wait_duration = registers_ptr_->getStored(Registers::TzerowaitAddress);
  cached_controller_settings_.stall_stop_enabled = sw_mode.sg_stop;
  cached_controller_settings_.min_dc_step_velocity = registers_ptr_->getStored(Registers::VdcminAddress);
}

void Controller::restoreControllerSettings()
{
  writeControllerParameters(cached_controller_settings_);
}

void Controller::writeSwitchParameters(SwitchParameters parameters)
{
  Registers::SwMode sw_mode;
  sw_mode.bytes = registers_ptr_->getStored(Registers::SwModeAddress);
  sw_mode.stop_l_enable = parameters.left_stop_enabled;
  sw_mode.stop_r_enable = parameters.right_stop_enabled;
  sw_mode.pol_stop_l = parameters.invert_left_polarity;
  sw_mode.pol_stop_r = parameters.invert_right_polarity;
  sw_mode.swap_lr = parameters.swap_left_right;
  sw_mode.latch_l_active = parameters.latch_left_active;
  sw_mode.latch_l_inactive = parameters.latch_left_inactive;
  sw_mode.latch_r_active = parameters.latch_right_active;
  sw_mode.latch_r_inactive = parameters.latch_right_inactive;
  sw_mode.en_latch_encoder = parameters.latch_encoder_enabled;
  registers_ptr_->write(Registers::SwModeAddress, sw_mode.bytes);
}

void Controller::cacheSwitchSettings()
{
  Registers::SwMode sw_mode;
  sw_mode.bytes = registers_ptr_->getStored(Registers::SwModeAddress);
  cached_switch_settings_.left_stop_enabled = sw_mode.stop_l_enable;
  cached_switch_settings_.right_stop_enabled = sw_mode.stop_r_enable;
  cached_switch_settings_.invert_left_polarity = sw_mode.pol_stop_l;
  cached_switch_settings_.invert_right_polarity = sw_mode.pol_stop_r;
  cached_switch_settings_.swap_left_right = sw_mode.swap_lr;
  cached_switch_settings_.latch_left_active = sw_mode.latch_l_active;
  cached_switch_settings_.latch_left_inactive = sw_mode.latch_l_inactive;
  cached_switch_settings_.latch_right_active = sw_mode.latch_r_active;
  cached_switch_settings_.latch_right_inactive = sw_mode.latch_r_inactive;
  cached_switch_settings_.latch_encoder_enabled = sw_mode.en_latch_encoder;
}

void Controller::restoreSwitchSettings()
{
  writeSwitchParameters(cached_switch_settings_);
}

