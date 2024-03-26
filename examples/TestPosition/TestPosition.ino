#include <TMC51X0.hpp>


#if defined(ARDUINO_ARCH_RP2040)
SPIClassRP2040 & spi = SPI;
pin_size_t SCK_PIN = 18;
pin_size_t TX_PIN = 19;
pin_size_t RX_PIN = 20;
#else
SPIClass & spi = SPI;
#endif

const uint8_t CHIP_SELECT_PIN = 10;
const uint8_t HARDWARE_ENABLE_PIN = 4;

const long SERIAL_BAUD_RATE = 115200;
const int DELAY = 4000;

// converter constants
// internal clock is ~12MHz
const uint8_t CLOCK_FREQUENCY_MHZ = 12;
// 200 fullsteps per revolution for many steppers * 256 microsteps per fullstep
// one "real unit" in this example is one rotation of the motor shaft
constexpr uint32_t MICROSTEPS_PER_REAL_UNIT = 51200;

// driver constants
const uint8_t GLOBAL_CURRENT_SCALAR = 50; // percent
const uint8_t RUN_CURRENT = 50; // percent
const uint8_t PWM_OFFSET = 10; // percent
const uint8_t PWM_GRADIENT = 10; // percent
const tmc51x0::Driver::MotorDirection MOTOR_DIRECTION = tmc51x0::Driver::FORWARD;
const uint8_t STEALTH_CHOP_THRESHOLD = 5; // rot/s
const uint8_t COOL_STEP_THRESHOLD = 6; // rot/s
const uint8_t COOL_STEP_MINIMUM = 1;
const uint8_t COOL_STEP_MAXIMUM = 0;
const uint8_t HIGH_VELOCITY_THRESHOLD = 9; // rot/s
// const int8_t STALL_GUARD_THRESHOLD = -20;

// controller constants
const uint32_t VELOCITY_TARGET = 4; // rot/s
const uint32_t POSITION_TARGET_MIN = 1;  // rot
const uint32_t POSITION_TARGET_MAX = 10;  // rot
const uint32_t ACCELERATION_MAX = 2;  // rot/(s^2)
const tmc51x0::Controller::RampMode RAMP_MODE = tmc51x0::Controller::POSITION;
const int32_t INITIAL_POSITION = 0;

// Instantiate TMC51X0
TMC51X0 stepper;
uint32_t position_target;

void printRegisterPortion(const char * str, uint32_t value, bool hex=false)
{
  Serial.print(str);
  if (not hex)
  {
    Serial.print(": 0b");
    Serial.print(value, BIN);
  }
  else
  {
    Serial.print(": 0x");
    Serial.print(value, HEX);
  }
  Serial.println();
}

void printGlobalStatus(tmc51x0::Registers::Gstat gstat)
{
  printRegisterPortion("reset", gstat.reset);
  printRegisterPortion("drv_err", gstat.drv_err);
  printRegisterPortion("uv_cp", gstat.uv_cp);
  Serial.println("--------------------------");
}

void printRegisterRampStat(uint32_t register_data)
{
  tmc51x0::Registers::RampStat ramp_stat;
  ramp_stat.bytes = register_data;
  printRegisterPortion("ramp_stat", ramp_stat.bytes, true);
  printRegisterPortion("status_stop_l", ramp_stat.status_stop_l);
  printRegisterPortion("status_stop_r", ramp_stat.status_stop_r);
  printRegisterPortion("velocity_reached", ramp_stat.velocity_reached);
  printRegisterPortion("vzero", ramp_stat.vzero);
  printRegisterPortion("status_sg", ramp_stat.status_sg);
  Serial.println("--------------------------");
}

void printRegisterDrvStatus(uint32_t register_data)
{
  tmc51x0::Registers::DrvStatus drv_status;
  drv_status.bytes = register_data;
  printRegisterPortion("drv_status", drv_status.bytes, true);
  printRegisterPortion("sg_result", drv_status.sg_result, true);
  printRegisterPortion("stealth", drv_status.stealth);
  printRegisterPortion("cs_actual", drv_status.cs_actual, true);
  printRegisterPortion("stst", drv_status.stst);
  Serial.println("--------------------------");
}

void printRegisterPwmScale(uint32_t register_data)
{
  tmc51x0::Registers::PwmScale pwm_scale;
  pwm_scale.bytes = register_data;
  printRegisterPortion("pwm_scale", pwm_scale.bytes, true);
  printRegisterPortion("pwm_scale_sum", pwm_scale.pwm_scale_sum, true);
  printRegisterPortion("pwm_scale_auto", pwm_scale.pwm_scale_auto, true);
  Serial.println("--------------------------");
}

void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);
#if defined(ARDUINO_ARCH_RP2040)
  spi.setSCK(SCK_PIN);
  spi.setTX(TX_PIN);
  spi.setRX(RX_PIN);
#endif
  stepper.setup(spi, CHIP_SELECT_PIN);
  stepper.driver.setHardwareEnablePin(HARDWARE_ENABLE_PIN);

  stepper.converter.setClockFrequencyMHz(CLOCK_FREQUENCY_MHZ);
  stepper.converter.setMicrostepsPerRealUnit(MICROSTEPS_PER_REAL_UNIT);

  stepper.driver.writeGlobalCurrentScaler(stepper.converter.percentToGlobalCurrentScaler(GLOBAL_CURRENT_SCALAR));
  stepper.driver.writeRunCurrent(stepper.converter.percentToCurrentSetting(RUN_CURRENT));
  stepper.driver.writePwmOffset(stepper.converter.percentToPwmSetting(PWM_OFFSET));
  stepper.driver.writePwmGradient(stepper.converter.percentToPwmSetting(PWM_GRADIENT));
  stepper.driver.writeMotorDirection(MOTOR_DIRECTION);
  // stepper.driver.writeStealthChopThreshold(stepper.converter.velocityRealToTstep(STEALTH_CHOP_THRESHOLD));
  stepper.driver.writeCoolStepThreshold(stepper.converter.velocityRealToTstep(COOL_STEP_THRESHOLD));
  stepper.driver.enableCoolStep(COOL_STEP_MINIMUM, COOL_STEP_MAXIMUM);
  stepper.driver.writeHighVelocityThreshold(stepper.converter.velocityRealToTstep(HIGH_VELOCITY_THRESHOLD));
  // stepper.driver.writeStallGuardThreshold(STALL_GUARD_THRESHOLD);

  stepper.controller.writeMaxVelocity(stepper.converter.velocityRealToChip(VELOCITY_TARGET));
  stepper.controller.writeMaxAcceleration(stepper.converter.accelerationRealToChip(ACCELERATION_MAX));
  stepper.controller.writeRampMode(RAMP_MODE);
  stepper.controller.writeActualPosition(stepper.converter.positionRealToChip(INITIAL_POSITION));

  stepper.driver.enable();

  delay(DELAY);
}

void loop()
{
  printGlobalStatus(stepper.readAndClearGlobalStatus());
  printRegisterRampStat(stepper.registers.read(tmc51x0::Registers::RAMP_STAT));
  printRegisterDrvStatus(stepper.registers.read(tmc51x0::Registers::DRV_STATUS));
  printRegisterPwmScale(stepper.registers.read(tmc51x0::Registers::PWM_SCALE));

  Serial.print("acceleration (rotations per second per second): ");
  Serial.println(ACCELERATION_MAX);
  Serial.print("acceleration (chip units): ");
  Serial.println(stepper.converter.accelerationRealToChip(ACCELERATION_MAX));
  Serial.println("--------------------------");

  Serial.print("velocity_target (rotations per second): ");
  Serial.println(velocity_target);
  uint32_t actual_velocity_chip = stepper.controller.readActualVelocity();
  Serial.print("actual_velocity (chip units): ");
  Serial.println(actual_velocity_chip);
  uint32_t actual_velocity_real = stepper.converter.velocityChipToReal(actual_velocity_chip);
  Serial.print("actual_velocity (rotations per second): ");
  Serial.println(actual_velocity_real);
  uint32_t tstep = stepper.controller.readTstep();
  Serial.print("tstep (chip units): ");
  Serial.println(tstep);
  uint32_t velocity_real = stepper.converter.tstepToVelocityReal(tstep);
  Serial.print("tstepToVelocityReal (rotations per second): ");
  Serial.println(velocity_real);
  tstep = stepper.converter.velocityRealToTstep(velocity_real);
  Serial.print("velocityRealToTstep (chip_units): ");
  Serial.println(tstep);
  Serial.print("STEALTH_CHOP_THRESHOLD (rotations per second): ");
  Serial.println(STEALTH_CHOP_THRESHOLD);
  Serial.print("STEALTH_CHOP_THRESHOLD (chip units): ");
  Serial.println(stepper.converter.velocityRealToTstep(STEALTH_CHOP_THRESHOLD));
  Serial.println("--------------------------");

  int32_t actual_position_chip = stepper.controller.readActualPosition();
  Serial.print("actual position (chip units): ");
  Serial.println(actual_position_chip);
  int32_t actual_position_real = stepper.converter.positionChipToReal(actual_position_chip);
  Serial.print("actual position (rotations): ");
  Serial.println(actual_position_real);
  Serial.println("--------------------------");

  Serial.println("--------------------------");

  delay(DELAY);

  velocity_target += VELOCITY_TARGET_INC;
  if (velocity_target > VELOCITY_TARGET_MAX)
  {
    velocity_target = VELOCITY_TARGET_MIN;
  }
  stepper.controller.writeMaxVelocity(stepper.converter.velocityRealToChip(velocity_target));

  delay(DELAY);
}
