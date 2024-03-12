// ----------------------------------------------------------------------------
// Controller.cpp
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------
#include "Controller.hpp"


using namespace tmc51x0;

void Controller::setRampMode(RampMode mode)
{
  Serial.print("setting ramp mode to: ");
  Serial.println(mode);
  registers_ptr_->write(Registers::RAMPMODE, mode);
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

void Controller::setup(Registers & registers)
{
  registers_ptr_ = &registers;
}

