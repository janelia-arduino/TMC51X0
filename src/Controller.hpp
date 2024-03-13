// ----------------------------------------------------------------------------
// Controller.hpp
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------

#ifndef TMC51X0_CONTROLLER_HPP
#define TMC51X0_CONTROLLER_HPP

#include "TMC51X0/Constants.hpp"
#include "Registers.hpp"
#include "Converter.hpp"


class TMC51X0;

namespace tmc51x0
{
class Controller
{
public:
  enum RampMode
  {
    POSITION = 0,
    VELOCITY_POSITIVE = 1,
    VELOCITY_NEGATIVE = 2,
    HOLD = 3,
  };
  void setRampMode(RampMode mode);

  int32_t getTstep();

  // -2^31..(2^31)-1 microsteps
  int32_t getActualPosition();
  // -2^31..(2^31)-1 microsteps
  void setActualPosition(int32_t position);
  // -(2^23)-1..(2^23)-1 microsteps/t
  int32_t getActualVelocity();

  // 0..(2^23)-512 microsteps/t
  void setVelocityMax(uint32_t velocity);
  // 0..(2^16)-1 microsteps/ta^2
  void setAccelerationMax(uint32_t acceleration);

private:
  Registers * registers_ptr_;
  Converter * converter_ptr_;

  void setup(Registers & registers,
    Converter & converter);

  friend class ::TMC51X0;
};
}
#endif
