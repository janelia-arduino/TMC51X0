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
  void writeRampMode(RampMode ramp_mode);

  enum StopMode
  {
    HARD = 0,
    SOFT = 1,
  };
  void writeStopMode(StopMode stop_mode);

  uint32_t readTstep();

  // -2^31..(2^31)-1 microsteps
  int32_t readActualPosition();
  // -2^31..(2^31)-1 microsteps
  void writeActualPosition(int32_t position);
  // -(2^23)-1..(2^23)-1 microsteps/t
  int32_t readActualVelocity();

  // 0..(2^23)-512 microsteps/t
  void writeVelocityMax(uint32_t velocity);
  // 0..(2^16)-1 microsteps/ta^2
  void writeAccelerationMax(uint32_t acceleration);

private:
  Registers * registers_ptr_;
  Converter * converter_ptr_;

  const static RampMode RAMP_MODE_DEFAULT = VELOCITY_POSITIVE;
  const static StopMode STOP_MODE_DEFAULT = SOFT;
  const static int32_t ACTUAL_POSITION_DEFAULT = 0;
  const static uint32_t VELOCITY_MAX_DEFAULT = 0;
  const static uint32_t ACCELERATION_MAX_DEFAULT = 10000;

  void setup(Registers & registers,
    Converter & converter);

  friend class ::TMC51X0;
};
}
#endif
