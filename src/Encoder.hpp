// ----------------------------------------------------------------------------
// Encoder.hpp
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------

#ifndef TMC51X0_ENCODER_HPP
#define TMC51X0_ENCODER_HPP

#include "TMC51X0/Constants.hpp"
#include "TMC51X0/Converter.hpp"
#include "Registers.hpp"


class TMC51X0;

namespace tmc51x0
{
class Encoder
{
public:
  int32_t readActualPosition();
  // Registers::EncStatus readStatus();
private:
  Registers * registers_ptr_;
  Converter * converter_ptr_;

  void setup(Registers & registers,
    Converter & converter);

  friend class ::TMC51X0;
};
}
#endif
