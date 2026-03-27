// ----------------------------------------------------------------------------
// Encoder.cpp
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------
#include "Encoder.hpp"

using namespace tmc51x0;

Encoder::Encoder() {
  registers_ptr_ = nullptr;
  setup_encoder_parameters_ = EncoderParameters{};
}

void Encoder::setup() { writeEncoderParameters(setup_encoder_parameters_); }

void Encoder::setup(tmc51x0::EncoderParameters parameters) {
  setup_encoder_parameters_ = parameters;
  setup();
}

void Encoder::writeFractionalMode(FractionalMode mode) {
  setup_encoder_parameters_.fractional_mode = mode;
  Registers::Encmode encmode;
  encmode.raw = registers_ptr_->getStored(Registers::EncmodeAddress);
  encmode.enc_sel_decimal(mode);
  registers_ptr_->write(Registers::EncmodeAddress, encmode.raw);
}

void Encoder::writeMicrostepsPerPulse(int16_t integer, uint16_t fractional) {
  setup_encoder_parameters_.microsteps_per_pulse_integer = integer;
  setup_encoder_parameters_.microsteps_per_pulse_fractional = fractional;
  Registers::EncConst enc_const;
  enc_const.integer(integer);
  enc_const.fractional(fractional);
  return registers_ptr_->write(Registers::EncConstAddress, enc_const.raw);
}

int32_t Encoder::readActualPosition() {
  return registers_ptr_->read(Registers::XencAddress);
}

void Encoder::writeActualPosition(int32_t position) {
  return registers_ptr_->write(Registers::XencAddress, position);
}

void Encoder::zeroActualPosition() { return writeActualPosition(0); }

Registers::EncStatus Encoder::readAndClearStatus() {
  Registers::EncStatus enc_status_read, enc_status_write;
  enc_status_read.raw =
      registers_ptr_->read(tmc51x0::Registers::EncStatusAddress);
  enc_status_write.n_event(true);
  enc_status_write.deviation_warn(true);
  registers_ptr_->write(tmc51x0::Registers::EncStatusAddress,
                        enc_status_write.raw);
  return enc_status_read;
}

// private

void Encoder::initialize(Registers &registers) {
  registers_ptr_ = &registers;

  reinitialize();
}

void Encoder::reinitialize() {
  zeroActualPosition();
  setup();
}

void Encoder::writeEncoderParameters(EncoderParameters parameters) {
  setup_encoder_parameters_ = parameters;
  writeFractionalMode(parameters.fractional_mode);
  writeMicrostepsPerPulse(parameters.microsteps_per_pulse_integer,
                          parameters.microsteps_per_pulse_fractional);
}
