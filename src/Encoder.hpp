// ----------------------------------------------------------------------------
// Encoder.hpp
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------

#ifndef TMC51X0_ENCODER_HPP
#define TMC51X0_ENCODER_HPP

#include "Registers.hpp"
#include "TMC51X0/EncoderParameters.hpp"


class TMC51X0;

namespace tmc51x0
{
class Encoder
{
public:
  Encoder();

  void setup();
  void setup(EncoderParameters parameters);

  void writeFractionalMode(FractionalMode mode);

  // integer: -32768..32767
  // use integer < 0 to reverse rotation polarity
  // fractional binary: 0..65536 e.g. 32768=1/2
  // fractional decimal: 0..10000 e.g. 5000=1/2
  // e.g. 1.5: integer=1, fractional=32768 binary fractional mode
  // e.g. 1.5: integer=1, fractional=5000 decimal fractional mode
  void writeMicrostepsPerPulse(int16_t integer=1,
    uint16_t fractional=0);

  // -2^31..(2^31)-1 encoder counts
  int32_t readActualPosition();
  // -2^31..(2^31)-1 encoder counts
  void writeActualPosition(int32_t position);
  void zeroActualPosition();

  Registers::EncStatus readAndClearStatus();

private:
  Registers * registers_ptr_;
  EncoderParameters setup_encoder_parameters_;

  void initialize(Registers & registers);
  void reinitialize();
  void writeEncoderParameters(EncoderParameters parameters);

  friend class ::TMC51X0;
};
}
#endif
