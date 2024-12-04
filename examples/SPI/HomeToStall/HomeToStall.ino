#include <TMC51X0.hpp>


#if defined(ARDUINO_ARCH_RP2040)
SPIClassRP2040 & spi = SPI;
size_t SCK_PIN = 18;
size_t TX_PIN = 19;
size_t RX_PIN = 20;
#else
SPIClass & spi = SPI;
#endif

const tmc51x0::SpiParameters spi_parameters =
{
  spi,
  1000000, // clock_rate
  8 // chip_select_pin
};

const tmc51x0::ConverterParameters converter_parameters =
{
  16, // clock_frequency_mhz
  142 // microsteps_per_real_unit
};
// external clock is 16MHz
// 200 fullsteps per revolution for many steppers * 256 microsteps per fullstep
// 51200 microsteps per revolution / 360 degrees per revolution ~= 142 microsteps per degree
// one "real unit" in this example is one degree of rotation

const tmc51x0::DriverParameters driver_parameters_real =
{
  100, // global_current_scaler (percent)
  25, // run_current (percent)
  0, // hold_current (percent)
  0, // hold_delay (percent)
  15, // pwm_offset (percent)
  5, // pwm_gradient (percent)
  false, // automatic_current_control_enabled
  tmc51x0::FORWARD, // motor_direction
  tmc51x0::NORMAL, // standstill_mode
  tmc51x0::SPREAD_CYCLE, // chopper_mode
  2000, // stealth_chop_threshold (degrees/s)
  true, // stealth_chop_enabled
  3000, // cool_step_threshold (degrees/s)
  1, // cool_step_min
  0, // cool_step_max
  true, // cool_step_enabled
  4000, // high_velocity_threshold (degrees/s)
  true, // high_velocity_fullstep_enabled
  true, // high_velocity_chopper_switch_enabled
  0, // stall_guard_threshold
  false, // stall_guard_filter_enabled
  true // short_to_ground_protection_enabled
};

const tmc51x0::ControllerParameters controller_parameters_real =
{
  tmc51x0::POSITION, // ramp_mode
  tmc51x0::HARD, // stop_mode
  400, // max_velocity (degrees/s)
  40, // max_acceleration ((degrees/s)/s)
  20, // start_velocity (degrees/s)
  100, // stop_velocity (degrees/s)
  200, // first_velocity (degrees/s)
  200, // first_acceleration ((degrees/s)/s)
  400, // max_deceleration ((degrees/s)/s)
  500, // first_deceleration ((degrees/s)/s)
  0 // zero_wait_duration (milliseconds)
};
// home constants
// const uint8_t HOME_GLOBAL_CURRENT_SCALER = 50; // percent
// const uint8_t HOME_COOL_STEP_THRESHOLD = 5; // (degrees/s)
// const uint32_t HOME_START_VELOCITY = 1; // (degrees/s)
// const uint32_t HOME_MAX_VELOCITY = 10; // (degrees/s)
// const int32_t HOME_TARGET_POSITION = -200;  // (degrees)

const int32_t MIN_TARGET_POSITION = 20;  // degrees
const int32_t MAX_TARGET_POSITION = 270;  // degrees

const uint32_t SERIAL_BAUD_RATE = 115200;
const uint16_t LOOP_DELAY = 500;
// const uint16_t STALL_DELAY = 4000;

// global variables
TMC51X0 tmc5130;
uint32_t target_position;
tmc51x0::ControllerParameters controller_parameters_chip;

void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);

#if defined(ARDUINO_ARCH_RP2040)
  spi.setSCK(SCK_PIN);
  spi.setTX(TX_PIN);
  spi.setRX(RX_PIN);
#endif
  spi.begin();
  tmc5130.setupSpi(spi_parameters);

  tmc5130.converter.setup(converter_parameters);

  tmc51x0::DriverParameters driver_parameters_chip = tmc5130.converter.driverParametersRealToChip(driver_parameters_real);
  tmc5130.driver.setup(driver_parameters_chip);

  controller_parameters_chip = tmc5130.converter.controllerParametersRealToChip(controller_parameters_real);
  tmc5130.controller.setup(controller_parameters_chip);

  tmc5130.driver.enable();

  tmc5130.controller.beginRampToZeroVelocity();
  while (!tmc5130.controller.zeroVelocity())
  {
    Serial.println("Waiting for zero velocity.");
    delay(LOOP_DELAY);
  }
  tmc5130.controller.endRampToZeroVelocity();

  randomSeed(analogRead(A0));
  long random_delay = random(5000);
  delay(random_delay);

  target_position = MIN_TARGET_POSITION;
  tmc5130.controller.writeTargetPosition(tmc5130.converter.positionRealToChip(target_position));

}

void loop()
{
  // Serial.println("Waiting to start homing.");
  // delay(4000);

  // homing
  // Serial.println("Setting homing constants.");
  // tmc5130.driver.writeGlobalCurrentScaler(tmc5130.converter.percentToGlobalCurrentScaler(HOME_GLOBAL_CURRENT_SCALER));
  // tmc5130.driver.writeCoolStepThreshold(tmc5130.converter.velocityRealToTstep(HOME_COOL_STEP_THRESHOLD));
  // tmc5130.controller.writeStartVelocity(tmc5130.converter.velocityRealToChip(HOME_START_VELOCITY));
  // tmc5130.controller.writeMaxVelocity(tmc5130.converter.velocityRealToChip(HOME_MAX_VELOCITY));
  // tmc5130.controller.writeTargetPosition(tmc5130.converter.positionRealToChip(HOME_TARGET_POSITION));
  // Serial.println("Homing.");
  // while (!tmc5130.controller.velocityReached())
  // {
  //   Serial.println("Waiting to reach home velocity.");
  //   delay(LOOP_DELAY);
  // }
  // Serial.println("Enabling stall stop.");
  // tmc5130.controller.enableStallStop();
  // uint16_t stall_guard_result;
  // while (!tmc5130.driver.stalled() && !tmc5130.controller.zeroVelocity() && !tmc5130.controller.positionReached())
  // {
  //   Serial.println("Waiting to stall, reach zero velocity, or reach home position.");
  //   stall_guard_result = tmc5130.driver.readStallGuardResult();
  //   Serial.print("stall guard result: ");
  //   Serial.println(stall_guard_result);
  //   delay(LOOP_DELAY);
  // }
  // if (!tmc5130.controller.positionReached())
  // {
  //   Serial.println("Homed successfully!");
  // }
  // else
  // {
  //   Serial.println("Home Failed!! Try adjusting stallguard threshold or global current scaler.");
  // }
  // Serial.println("Setting zero velocity.");
  // tmc5130.controller.writeStartVelocity(0);
  // tmc5130.controller.writeMaxVelocity(0);
  // Serial.println("Disabling stall stop.");
  // tmc5130.controller.disableStallStop();
  // Serial.println("Setting actual and target positions to zero.");
  // tmc5130.controller.writeActualPosition(0);
  // tmc5130.controller.writeTargetPosition(0);
  // delay(4000);

  // // move to posititions
  // tmc5130.driver.writeGlobalCurrentScaler(driver_parameters_chip.global_current_scaler);
  // tmc5130.driver.writeCoolStepThreshold(driver_parameters_chip.cool_step_threshold);
  // tmc5130.controller.writeStartVelocity(tmc5130.converter.velocityRealToChip(START_VELOCITY));
  // tmc5130.controller.writeMaxVelocity(tmc5130.converter.velocityRealToChip(MAX_VELOCITY));

  // Serial.println("Moving to min target position.");
  // tmc5130.controller.writeTargetPosition(tmc5130.converter.positionRealToChip(MIN_TARGET_POSITION));

  // Serial.println("Waiting to reach min target position.");
  // while (!tmc5130.controller.positionReached())
  // {
  //   delay(LOOP_DELAY);
  // }
  // Serial.println("Reached min target position.");

  // Serial.println("Moving to max target position.");
  // tmc5130.controller.writeTargetPosition(tmc5130.converter.positionRealToChip(MAX_TARGET_POSITION));

  // Serial.println("Waiting to reach max target position.");
  // while (!tmc5130.controller.positionReached())
  // {
  //   delay(LOOP_DELAY);
  // }
  // Serial.println("Reached max target position.");

  // Serial.println("--------------------------");
  // delay(LOOP_DELAY);

  tmc5130.printer.readAndPrintRampStat();
  tmc5130.printer.readAndPrintDrvStatus();

  Serial.print("max_velocity (chip): ");
  Serial.println(controller_parameters_chip.max_velocity);
  Serial.print("max_velocity (degrees per second): ");
  Serial.println(controller_parameters_real.max_velocity);

  int32_t actual_velocity_chip = tmc5130.controller.readActualVelocity();
  Serial.print("actual_velocity (chip): ");
  Serial.println(actual_velocity_chip);
  int32_t actual_velocity_real = tmc5130.converter.velocityChipToReal(actual_velocity_chip);
  Serial.print("actual_velocity (degrees per second): ");
  Serial.println(actual_velocity_real);

  int32_t actual_position_chip = tmc5130.controller.readActualPosition();
  int32_t actual_position_real = tmc5130.converter.positionChipToReal(actual_position_chip);
  Serial.print("actual position (degrees): ");
  Serial.println(actual_position_real);

  int32_t target_position_chip = tmc5130.controller.readTargetPosition();
  int32_t target_position_real = tmc5130.converter.positionChipToReal(target_position_chip);
  Serial.print("target position (degrees): ");
  Serial.println(target_position_real);
  Serial.println("--------------------------");

  if (tmc5130.controller.positionReached())
  {
    Serial.println("Reached target position!");
    Serial.println("--------------------------");
    long random_delay = random(3000);
    delay(random_delay);
    if (target_position == MIN_TARGET_POSITION)
    {
      target_position = MAX_TARGET_POSITION;
    }
    else
    {
      target_position = MIN_TARGET_POSITION;
    }
    tmc5130.controller.writeTargetPosition(tmc5130.converter.positionRealToChip(target_position));
  }

  Serial.println("--------------------------");
  delay(LOOP_DELAY);
}
