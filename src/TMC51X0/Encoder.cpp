// ----------------------------------------------------------------------------
// Encoder.cpp
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------
#include "Encoder.hpp"


using namespace tmc51x0;

int32_t Encoder::readActualPosition()
{
  return registers_ptr_->read(Registers::X_ENC);
}

// private

void Encoder::setup(Registers & registers,
  Converter & converter)
{
  registers_ptr_ = &registers;
  converter_ptr_ = &converter;
}

