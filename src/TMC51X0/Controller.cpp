// ----------------------------------------------------------------------------
// Controller.cpp
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------
#include "Controller.hpp"


using namespace tmc51x0;

// private

void Controller::setup(Registers & registers)
{
  registers_ptr_ = &registers;
}

