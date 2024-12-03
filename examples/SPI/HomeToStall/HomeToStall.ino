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
  10 // chip_select_pin
};

const tmc51x0::ConverterParameters converter_parameters =
{
  12, // clock_frequency_mhz
  4881 // microsteps_per_real_unit
};
// internal clock is ~12MHz
// 200 fullsteps per revolution for many steppers * 256 microsteps per fullstep
// 10.49 millimeters per revolution leadscrew -> 51200 / 10.49 ~= 4881
// one "real unit" in this example is one millimeters of linear travel

const tmc51x0::DriverParameters driver_parameters_real =
{
  50, // global_current_scaler (percent)
  20, // run_current (percent)
  0, // hold_current (percent)
  0, // hold_delay (percent)
  20, // pwm_offset (percent)
  5, // pwm_gradient (percent)
  false, // automatic_current_control_enabled
  tmc51x0::REVERSE, // motor_direction
  tmc51x0::NORMAL, // standstill_mode
  tmc51x0::SPREAD_CYCLE, // chopper_mode
  100, // stealth_chop_threshold (millimeters/s)
  true, // stealth_chop_enabled
  150, // cool_step_threshold (millimeters/s)
  1, // cool_step_min
  0, // cool_step_max
  true, // cool_step_enabled
  200, // high_velocity_threshold (rotations/min)
  true, // high_velocity_fullstep_enabled
  true, // high_velocity_chopper_switch_enabled
  1, // stall_guard_threshold
  false, // stall_guard_filter_enabled
  true // short_to_ground_protection_enabled
};
tmc51x0::DriverParameters driver_parameters_chip;

// controller constants
const uint32_t START_VELOCITY = 1; // millimeters/s
const uint32_t FIRST_ACCELERATION = 10;  // millimeters/(s^2)
const uint32_t FIRST_VELOCITY = 10; // millimeters/s
const uint32_t MAX_ACCELERATION = 2;  // millimeters/(s^2)
const uint32_t MAX_DECELERATION = 25;  // millimeters/(s^2)
const uint32_t FIRST_DECELERATION = 20;  // millimeters/(s^2)
const uint32_t MAX_VELOCITY = 20; // millimeters/s
const uint32_t STOP_VELOCITY = 5; // millimeters/s

const int32_t MIN_TARGET_POSITION = 20;  // millimeters
const int32_t MAX_TARGET_POSITION = 180;  // millimeters
const tmc51x0::Controller::RampMode RAMP_MODE = tmc51x0::Controller::POSITION;

// home constants
const uint8_t HOME_GLOBAL_CURRENT_SCALER = 50; // percent
const uint8_t HOME_COOL_STEP_THRESHOLD = 5; // millimeters/s
const uint32_t HOME_START_VELOCITY = 1; // millimeters/s
const uint32_t HOME_MAX_VELOCITY = 10; // millimeters/s
const int32_t HOME_TARGET_POSITION = -200;  // millimeters

const size_t ENABLE_HARDWARE_PIN = 4;

const uint32_t SERIAL_BAUD_RATE = 115200;
const uint16_t LOOP_DELAY = 500;
const uint16_t STALL_DELAY = 4000;

// Instantiate TMC51X0
TMC51X0 tmc5160;

void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);

#if defined(ARDUINO_ARCH_RP2040)
  spi.setSCK(SCK_PIN);
  spi.setTX(TX_PIN);
  spi.setRX(RX_PIN);
#endif
  spi.begin();
  tmc5160.setupSpi(spi_parameters);

  tmc5160.converter.setup(converter_parameters);

  driver_parameters_chip = tmc5160.converter.driverParametersRealToChip(driver_parameters_real);
  tmc5160.driver.setup(driver_parameters_chip);
  tmc5160.driver.setEnableHardwarePin(ENABLE_HARDWARE_PIN);

  tmc5160.controller.writeFirstAcceleration(tmc5160.converter.accelerationRealToChip(FIRST_ACCELERATION));
  tmc5160.controller.writeFirstVelocity(tmc5160.converter.velocityRealToChip(FIRST_VELOCITY));
  tmc5160.controller.writeMaxAcceleration(tmc5160.converter.accelerationRealToChip(MAX_ACCELERATION));
  tmc5160.controller.writeMaxDeceleration(tmc5160.converter.accelerationRealToChip(MAX_DECELERATION));
  tmc5160.controller.writeFirstDeceleration(tmc5160.converter.accelerationRealToChip(FIRST_DECELERATION));
  tmc5160.controller.writeStopVelocity(tmc5160.converter.velocityRealToChip(STOP_VELOCITY));
  tmc5160.controller.writeRampMode(RAMP_MODE);

  tmc5160.driver.enable();

  tmc5160.controller.beginRampToZeroVelocity();
  while (!tmc5160.controller.zeroVelocity())
  {
    Serial.println("Waiting for zero velocity.");
    delay(LOOP_DELAY);
  }
  tmc5160.controller.endRampToZeroVelocity();
}

void loop()
{
  Serial.println("Waiting to start homing.");
  delay(4000);

  // homing
  Serial.println("Setting homing constants.");
  tmc5160.driver.writeGlobalCurrentScaler(tmc5160.converter.percentToGlobalCurrentScaler(HOME_GLOBAL_CURRENT_SCALER));
  tmc5160.driver.writeCoolStepThreshold(tmc5160.converter.velocityRealToTstep(HOME_COOL_STEP_THRESHOLD));
  tmc5160.controller.writeStartVelocity(tmc5160.converter.velocityRealToChip(HOME_START_VELOCITY));
  tmc5160.controller.writeMaxVelocity(tmc5160.converter.velocityRealToChip(HOME_MAX_VELOCITY));
  tmc5160.controller.writeTargetPosition(tmc5160.converter.positionRealToChip(HOME_TARGET_POSITION));
  Serial.println("Homing.");
  while (!tmc5160.controller.velocityReached())
  {
    Serial.println("Waiting to reach home velocity.");
    delay(LOOP_DELAY);
  }
  Serial.println("Enabling stall stop.");
  tmc5160.controller.enableStallStop();
  uint16_t stall_guard_result;
  while (!tmc5160.driver.stalled() && !tmc5160.controller.zeroVelocity() && !tmc5160.controller.positionReached())
  {
    Serial.println("Waiting to stall, reach zero velocity, or reach home position.");
    stall_guard_result = tmc5160.driver.readStallGuardResult();
    Serial.print("stall guard result: ");
    Serial.println(stall_guard_result);
    delay(LOOP_DELAY);
  }
  if (!tmc5160.controller.positionReached())
  {
    Serial.println("Homed successfully!");
  }
  else
  {
    Serial.println("Home Failed!! Try adjusting stallguard threshold or global current scaler.");
  }
  Serial.println("Setting zero velocity.");
  tmc5160.controller.writeStartVelocity(0);
  tmc5160.controller.writeMaxVelocity(0);
  Serial.println("Disabling stall stop.");
  tmc5160.controller.disableStallStop();
  Serial.println("Setting actual and target positions to zero.");
  tmc5160.controller.writeActualPosition(0);
  tmc5160.controller.writeTargetPosition(0);
  delay(4000);

  // move to posititions
  tmc5160.driver.writeGlobalCurrentScaler(driver_parameters_chip.global_current_scaler);
  tmc5160.driver.writeCoolStepThreshold(driver_parameters_chip.cool_step_threshold);
  tmc5160.controller.writeStartVelocity(tmc5160.converter.velocityRealToChip(START_VELOCITY));
  tmc5160.controller.writeMaxVelocity(tmc5160.converter.velocityRealToChip(MAX_VELOCITY));

  Serial.println("Moving to min target position.");
  tmc5160.controller.writeTargetPosition(tmc5160.converter.positionRealToChip(MIN_TARGET_POSITION));

  Serial.println("Waiting to reach min target position.");
  while (!tmc5160.controller.positionReached())
  {
    delay(LOOP_DELAY);
  }
  Serial.println("Reached min target position.");

  Serial.println("Moving to max target position.");
  tmc5160.controller.writeTargetPosition(tmc5160.converter.positionRealToChip(MAX_TARGET_POSITION));

  Serial.println("Waiting to reach max target position.");
  while (!tmc5160.controller.positionReached())
  {
    delay(LOOP_DELAY);
  }
  Serial.println("Reached max target position.");

  Serial.println("--------------------------");
  delay(LOOP_DELAY);
}
