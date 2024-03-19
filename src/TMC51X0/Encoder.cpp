// ----------------------------------------------------------------------------
// Encoder.cpp
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------
#include "Encoder.hpp"


using namespace tmc51x0;

void Encoder::writeFractionalMode(Encoder::FractionalMode mode)
{
  Registers::Encmode encmode;
  encmode.bytes = registers_ptr_->getStored(Registers::ENCMODE);
  encmode.enc_sel_decimal = mode;
  registers_ptr_->write(Registers::ENCMODE, encmode.bytes);
}

void Encoder::writeMicrostepsPerPulse(int16_t integer,
  uint16_t fractional)
{
  Registers::EncConst enc_const;
  enc_const.integer = integer;
  enc_const.fractional = fractional;
  return registers_ptr_->write(Registers::ENC_CONST, enc_const.bytes);
}

int32_t Encoder::readActualPosition()
{
  return registers_ptr_->read(Registers::X_ENC);
}

void Encoder::writeActualPosition(int32_t position)
{
  return registers_ptr_->write(Registers::X_ENC, position);
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

  writeFractionalMode(FRACTIONAL_MODE_DEFAULT);
  writeMicrostepsPerPulse(MICROSTEPS_PER_PULSE_INTEGER_DEFAULT, MICROSTEPS_PER_PULSE_FRACTIONAL_DEFAULT);
  writeActualPosition(ACTUAL_POSITION_DEFAULT);
}

