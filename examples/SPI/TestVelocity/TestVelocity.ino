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
  51200 // microsteps_per_real_unit
};
// internal clock is 12MHz
// 200 fullsteps per revolution for many steppers * 256 microsteps per fullstep
// one "real unit" in this example is one rotation of the motor shaft

const tmc51x0::DriverParameters driver_parameters_real =
{
  50, // global_current_scalar (percent)
  100, // run_current (percent)
  0, // hold_current (percent)
  0, // hold_delay (percent)
  20, // pwm_offset (percent)
  5, // pwm_gradient (percent)
  false, // automatic_current_control_enabled
  tmc51x0::FORWARD, // motor_direction
  tmc51x0::NORMAL, // standstill_mode
  tmc51x0::SPREAD_CYCLE, // chopper_mode
  5, // stealth_chop_threshold (rotations/s)
  true, // stealth_chop_enabled
  6, // cool_step_threshold (rotations/s)
  1, // cool_step_min
  0, // cool_step_max
  true, // cool_step_enabled
  9, // high_velocity_threshold (rotations/s)
  true, // high_velocity_fullstep_enabled
  true, // high_velocity_chopper_switch_enabled
  0, // stall_guard_threshold
  true, // stall_guard_filter_enabled
  true // short_to_ground_protection_enabled
};

// controller constants
const uint32_t MIN_TARGET_VELOCITY = 1;  // rotations/s
const uint32_t MAX_TARGET_VELOCITY = 25; // rotations/s
const uint32_t TARGET_VELOCITY_INC = 1;  // rotations/s
const uint32_t MAX_ACCELERATION = 2;  // rotations/(s^2)
const tmc51x0::Controller::RampMode RAMP_MODE = tmc51x0::Controller::VELOCITY_POSITIVE;
const int32_t INITIAL_POSITION = 0;

const size_t ENABLE_HARDWARE_PIN = 4;

const uint32_t SERIAL_BAUD_RATE = 115200;
const uint16_t DELAY = 4000;

// Instantiate TMC51X0
TMC51X0 tmc5160;
uint32_t target_velocity;

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

  tmc5160.driver.setEnableHardwarePin(ENABLE_HARDWARE_PIN);
  tmc51x0::DriverParameters driver_parameters_chip = tmc5160.converter.driverParametersRealToChip(driver_parameters_real);
  tmc5160.driver.setup(driver_parameters_chip);

  tmc5160.controller.writeMaxAcceleration(tmc5160.converter.accelerationRealToChip(MAX_ACCELERATION));
  tmc5160.controller.writeRampMode(RAMP_MODE);
  tmc5160.controller.writeActualPosition(tmc5160.converter.positionRealToChip(INITIAL_POSITION));

  tmc5160.driver.enable();

  tmc5160.controller.rampToZeroVelocity();
  while (!tmc5160.controller.zeroVelocity())
  {
    Serial.println("Waiting for zero velocity.");
    delay(DELAY);
  }

  target_velocity = MIN_TARGET_VELOCITY;
  tmc5160.controller.writeMaxVelocity(tmc5160.converter.velocityRealToChip(target_velocity));

  delay(DELAY);
}

void loop()
{
  tmc5160.printer.readClearAndPrintGstat();
  tmc5160.printer.readAndPrintRampStat();
  tmc5160.printer.readAndPrintDrvStatus();
  tmc5160.printer.readAndPrintPwmScale();

  // Serial.print("acceleration (rotations per second per second): ");
  // Serial.println(MAX_ACCELERATION);
  // Serial.print("acceleration (chip units): ");
  // Serial.println(tmc5160.converter.accelerationRealToChip(MAX_ACCELERATION));
  // Serial.println("--------------------------");

  Serial.print("target_velocity (rotations per second): ");
  Serial.println(target_velocity);
  uint32_t actual_velocity_chip = tmc5160.controller.readActualVelocity();
  // Serial.print("actual_velocity (chip units): ");
  // Serial.println(actual_velocity_chip);
  uint32_t actual_velocity_real = tmc5160.converter.velocityChipToReal(actual_velocity_chip);
  Serial.print("actual_velocity (rotations per second): ");
  Serial.println(actual_velocity_real);
  // uint32_t tstep = tmc5160.controller.readTstep();
  // Serial.print("tstep (chip units): ");
  // Serial.println(tstep);
  // uint32_t velocity_real = tmc5160.converter.tstepToVelocityReal(tstep);
  // Serial.print("tstepToVelocityReal (rotations per second): ");
  // Serial.println(velocity_real);
  // tstep = tmc5160.converter.velocityRealToTstep(velocity_real);
  // Serial.print("velocityRealToTstep (chip_units): ");
  // Serial.println(tstep);
  // Serial.print("STEALTH_CHOP_THRESHOLD (rotations per second): ");
  // Serial.println(STEALTH_CHOP_THRESHOLD);
  // Serial.print("STEALTH_CHOP_THRESHOLD (chip units): ");
  // Serial.println(tmc5160.converter.velocityRealToTstep(STEALTH_CHOP_THRESHOLD));
  Serial.println("--------------------------");

  // int32_t actual_position_chip = tmc5160.controller.readActualPosition();
  // Serial.print("actual position (chip units): ");
  // Serial.println(actual_position_chip);
  // int32_t actual_position_real = tmc5160.converter.positionChipToReal(actual_position_chip);
  // Serial.print("actual position (rotations): ");
  // Serial.println(actual_position_real);
  // Serial.println("--------------------------");

  Serial.println("--------------------------");

  delay(DELAY);

  target_velocity += TARGET_VELOCITY_INC;
  if (target_velocity > MAX_TARGET_VELOCITY)
  {
    target_velocity = MIN_TARGET_VELOCITY;
  }
  tmc5160.controller.writeMaxVelocity(tmc5160.converter.velocityRealToChip(target_velocity));

  delay(DELAY);
}
