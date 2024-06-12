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
  // reset default: VELOCITY_POSITIVE
  void writeRampMode(RampMode ramp_mode);

  enum StopMode
  {
    HARD = 0,
    SOFT = 1,
  };
  // reset default: SOFT
  void writeStopMode(StopMode stop_mode);

  uint32_t readTstep();

  // -2^31..(2^31)-1 microsteps
  int32_t readActualPosition();
  // -2^31..(2^31)-1 microsteps
  void writeActualPosition(int32_t position);
  // -(2^23)-1..(2^23)-1 microsteps/t
  int32_t readActualVelocity();

  bool velocityReached();
  bool positionReached();

  // 0..(2^23)-512 microsteps/t
  // reset default: 0
  void writeMaxVelocity(uint32_t velocity);
  // 0..(2^16)-1 microsteps/ta^2
  // reset default: 10000
  void writeMaxAcceleration(uint32_t acceleration);

  // 0..(2^18)-1 microsteps/t
  // reset default: 0
  void writeStartVelocity(uint32_t velocity);
  // 0..(2^18)-1 microsteps/t
  // reset default: 10
  void writeStopVelocity(uint32_t velocity);

  // 0..(2^16)-1 microsteps/ta^2
  // reset default: 0
  void writeFirstAcceleration(uint32_t acceleration);
  // 0..(2^20)-1 microsteps/t
  // reset default: 0
  void writeFirstVelocity(uint32_t velocity);

  // 0..(2^16)-1 microsteps/ta^2
  // reset default: 0
  void writeMaxDeceleration(uint32_t deceleration);
  // 0..(2^16)-1 microsteps/ta^2
  // reset default: 10000
  void writeFirstDeceleration(uint32_t deceleration);

  // 0..(2^16)-1 *512 tclk
  // ~0..2s
  // reset default: 0
  void writeTzerowait(uint32_t tzerowait);

  // -2^31..(2^31)-1 microsteps
  int32_t readTargetPosition();
  // -2^31..(2^31)-1 microsteps
  void writeTargetPosition(int32_t position);

private:
  Registers * registers_ptr_;
  Converter * converter_ptr_;

  const static RampMode RAMP_MODE_DEFAULT = VELOCITY_POSITIVE;
  const static StopMode STOP_MODE_DEFAULT = SOFT;
  const static int32_t ACTUAL_POSITION_DEFAULT = 0;
  const static uint32_t MAX_VELOCITY_DEFAULT = 0;
  const static uint32_t MAX_ACCELERATION_DEFAULT = 10000;
  const static uint32_t START_VELOCITY_DEFAULT = 0;
  const static uint32_t STOP_VELOCITY_DEFAULT = 10;
  const static uint32_t FIRST_ACCELERATION_DEFAULT = 0;
  const static uint32_t FIRST_VELOCITY_DEFAULT = 0;
  const static uint32_t MAX_DECELERATION_DEFAULT = 0;
  const static uint32_t FIRST_DECELERATION_DEFAULT = 10000;
  const static uint32_t TZEROWAIT_DEFAULT = 0;
  const static int32_t TARGET_POSITION_DEFAULT = 0;

  void setup(Registers & registers,
    Converter & converter);

  friend class ::TMC51X0;
};
}
#endif
