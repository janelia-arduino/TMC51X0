// ----------------------------------------------------------------------------
// Controller.cpp
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------
#include "Controller.hpp"


using namespace tmc51x0;

void Controller::setRampMode(RampMode ramp_mode)
{
  registers_ptr_->write(Registers::RAMPMODE, ramp_mode);
}

void Controller::setStopMode(StopMode stop_mode)
{
  Registers::SwMode sw_mode;
  sw_mode.bytes = registers_ptr_->getStored(Registers::SW_MODE);
  sw_mode.en_softstop = stop_mode;
  registers_ptr_->write(Registers::SW_MODE, sw_mode.bytes);
}

uint32_t Controller::getTstep()
{
  return registers_ptr_->read(Registers::TSTEP);
}

int32_t Controller::getActualPosition()
{
  return registers_ptr_->read(Registers::XACTUAL);
}

void Controller::setActualPosition(int32_t position)
{
  return registers_ptr_->write(Registers::XACTUAL, position);
}

int32_t Controller::getActualVelocity()
{
  return registers_ptr_->read(Registers::VACTUAL);
}

void Controller::setVelocityMax(uint32_t velocity)
{
  registers_ptr_->write(Registers::VMAX, velocity);
}

void Controller::setAccelerationMax(uint32_t acceleration)
{
  registers_ptr_->write(Registers::AMAX, acceleration);
}

// private

void Controller::setup(Registers & registers,
  Converter & converter)
{
  registers_ptr_ = &registers;
  converter_ptr_ = &converter;

  setRampMode(RAMP_MODE_DEFAULT);
  setStopMode(STOP_MODE_DEFAULT);
  setActualPosition(ACTUAL_POSITION_DEFAULT);
  setVelocityMax(VELOCITY_MAX_DEFAULT);
  setAccelerationMax(ACCELERATION_MAX_DEFAULT);
}

