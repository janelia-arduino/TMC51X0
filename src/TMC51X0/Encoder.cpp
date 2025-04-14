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
  registers_ptr_ = nullptr;
  setup_encoder_parameters_ = EncoderParameters{};
}

void Encoder::setup()
{
  writeEncoderParameters(setup_encoder_parameters_);
}

void Encoder::setup(tmc51x0::EncoderParameters parameters)
{
  setup_encoder_parameters_ = parameters;
  setup();
}

void Encoder::writeFractionalMode(FractionalMode mode)
{
  Registers::Encmode encmode;
  encmode.bytes = registers_ptr_->getStored(Registers::EncmodeAddress);
  encmode.enc_sel_decimal = mode;
  registers_ptr_->write(Registers::EncmodeAddress, encmode.bytes);
}

void Encoder::writeMicrostepsPerPulse(int16_t integer,
  uint16_t fractional)
{
  Registers::EncConst enc_const;
  enc_const.integer = integer;
  enc_const.fractional = fractional;
  return registers_ptr_->write(Registers::EncConstAddress, enc_const.bytes);
}

int32_t Encoder::readActualPosition()
{
  return registers_ptr_->read(Registers::XencAddress);
}

void Encoder::writeActualPosition(int32_t position)
{
  return registers_ptr_->write(Registers::XencAddress, position);
}

void Encoder::zeroActualPosition()
{
  return writeActualPosition(0);
}

Registers::EncStatus Encoder::readAndClearStatus()
{
  Registers::EncStatus enc_status_read, enc_status_write;
  enc_status_read.bytes = registers_ptr_->read(tmc51x0::Registers::EncStatusAddress);
  enc_status_write.n_event = 1;
  enc_status_write.deviation_warn = 1;
  registers_ptr_->write(tmc51x0::Registers::EncStatusAddress, enc_status_write.bytes);
  return enc_status_read;
}

// private

void Encoder::initialize(Registers & registers)
{
  registers_ptr_ = &registers;

  reinitialize();
}

void Encoder::reinitialize()
{
  zeroActualPosition();
  setup();
}

void Encoder::writeEncoderParameters(EncoderParameters parameters)
{
  writeFractionalMode(parameters.fractional_mode);
  writeMicrostepsPerPulse(parameters.microsteps_per_pulse_integer,
    parameters.microsteps_per_pulse_fractional);
}

