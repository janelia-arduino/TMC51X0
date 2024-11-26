// ----------------------------------------------------------------------------
// Controller.hpp
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------

#ifndef TMC51X0_CONTROLLER_HPP
#define TMC51X0_CONTROLLER_HPP

#include "Registers.hpp"
#include "TMC51X0/ControllerParameters.hpp"


class TMC51X0;

namespace tmc51x0
{
class Controller
{
public:
  void setup(ControllerParameters controller_parameters);

  // reset default: VELOCITY_POSITIVE
  void writeRampMode(RampMode ramp_mode);

  // reset default: HARD
  void writeStopMode(StopMode stop_mode);

  uint32_t readTstep();

  // -2^31..(2^31)-1 microsteps
  int32_t readActualPosition();
  // -2^31..(2^31)-1 microsteps
  void writeActualPosition(int32_t position);
  // -(2^23)-1..(2^23)-1 microsteps/t
  int32_t readActualVelocity();
  void zeroActualPosition();

  bool velocityReached();
  bool positionReached();

  void rampToZeroVelocity();
  bool zeroVelocity();

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

  // -2^31..(2^31)-1 microsteps
  void writeComparePosition(int32_t position);

  // enabling stall stop sets stop mode to hard
  void enableStallStop();
  // set stop mode after disabling stall stop if desired
  void disableStallStop();

private:
  Registers * registers_ptr_;

  const static int32_t ACTUAL_POSITION_DEFAULT = 0;
  const static uint32_t START_VELOCITY_DEFAULT = 0;
  const static uint32_t STOP_VELOCITY_DEFAULT = 10;
  const static uint32_t FIRST_ACCELERATION_DEFAULT = 0;
  const static uint32_t FIRST_VELOCITY_DEFAULT = 10;
  const static uint32_t MAX_DECELERATION_DEFAULT = 0;
  const static uint32_t FIRST_DECELERATION_DEFAULT = 10000;
  const static uint32_t TZEROWAIT_DEFAULT = 0;
  const static int32_t TARGET_POSITION_DEFAULT = 0;
  const static int32_t MAX_POSITIVE_VELOCITY = 8388607;
  const static int32_t VELOCITY_SIGN_CONVERSION = 16777216;

  void initialize(Registers & registers);

  friend class ::TMC51X0;
};
}
#endif
