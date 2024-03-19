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

Registers::EncStatus Encoder::readAndClearStatus()
{
  Registers::EncStatus enc_status_read, enc_status_write;
  enc_status_read.bytes = registers_ptr_->read(tmc51x0::Registers::ENC_STATUS);
  enc_status_write.n_event = 1;
  enc_status_write.deviation_warn = 1;
  registers_ptr_->write(tmc51x0::Registers::ENC_STATUS, enc_status_write.bytes);
  return enc_status_read;
}

// private

void Encoder::setup(Registers & registers,
  Converter & converter)
{
  registers_ptr_ = &registers;
  converter_ptr_ = &converter;
}

