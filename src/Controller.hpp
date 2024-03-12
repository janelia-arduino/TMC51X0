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

  int32_t getActualPosition();
  void setActualPosition(int32_t position);
  int32_t getActualVelocity();

  void setVelocityMax(uint32_t velocity);
  void setAccelerationMax(uint32_t acceleration);

private:
  Registers * registers_ptr_;

  void setup(Registers & registers);

  friend class ::TMC51X0;
};
}
#endif
