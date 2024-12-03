// ----------------------------------------------------------------------------
// Encoder.cpp
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------
#include "Encoder.hpp"


using namespace tmc51x0;

Encoder::Encoder()
{
  encoder_parameters_ = EncoderParameters{};
}

void Encoder::setup()
{
  writeFractionalMode(encoder_parameters_.fractional_mode);
  writeMicrostepsPerPulse(encoder_parameters_.microsteps_per_pulse_integer,
    encoder_parameters_.microsteps_per_pulse_fractional);
}

void Encoder::setup(tmc51x0::EncoderParameters encoder_parameters)
{
  encoder_parameters_ = encoder_parameters;
  setup();
}

void Encoder::writeFractionalMode(FractionalMode mode)
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

void Encoder::zeroActualPosition()
{
  return writeActualPosition(0);
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

void Encoder::initialize(Registers & registers)
{
  registers_ptr_ = &registers;

  zeroActualPosition();
}

