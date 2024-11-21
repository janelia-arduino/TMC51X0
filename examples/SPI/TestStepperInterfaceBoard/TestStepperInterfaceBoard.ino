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
  29 // chip_select_pin
};

const tmc51x0::ConverterParameters converter_parameters =
{
  12, // clock_frequency_mhz
  51200, // microsteps_per_real_unit
  60 // seconds_per_real_velocity_unit
};
// internal clock is ~12MHz
// 200 fullsteps per revolution for many steppers * 256 microsteps per fullstep
// one "real position unit" in this example is one rotation of the motor shaft
// rotations/s -> rotations/min
// rotations/(s^2) -> (rotations/min)/s

// driver constants @ 48V VM
//   motor: LE-4118-048-02-02
//     rated current: 1.5A
//     200 steps/rev
//   driver:
//     0.15 current sense resistor
//     1.6 A RMS
const tmc51x0::DriverParameters driver_parameters_real =
{
  100, // global_current_scalar (percent)
  100, // run_current (percent)
  0, // hold_current (percent)
  0, // hold_delay (percent)
  15, // pwm_offset (percent)
  5, // pwm_gradient (percent)
  false, // automatic_current_control_enabled
  tmc51x0::FORWARD, // motor_direction
  tmc51x0::NORMAL, // standstill_mode
  tmc51x0::SPREAD_CYCLE, // chopper_mode
  40, // stealth_chop_threshold (rotations/min)
  true, // stealth_chop_enabled
  50, // cool_step_threshold (rotations/min)
  1, // cool_step_min
  0, // cool_step_max
  true, // cool_step_enabled
  200, // high_velocity_threshold (rotations/min)
  true, // high_velocity_fullstep_enabled
  true, // high_velocity_chopper_switch_enabled
  0, // stall_guard_threshold
  true, // stall_guard_filter_enabled
  true // short_to_ground_protection_enabled
};

// controller constants
const uint32_t MIN_TARGET_VELOCITY = 60;  // rotations/min
const uint32_t MAX_TARGET_VELOCITY = 600; // rotations/min
const uint32_t TARGET_VELOCITY_INC = 60;  // rotations/min
const uint32_t MAX_ACCELERATION = 60;  // (rotations/min)/s
const tmc51x0::Controller::RampMode RAMP_MODE = tmc51x0::Controller::VELOCITY_POSITIVE;

const size_t ENABLE_VIO_PIN = 21;
const size_t ENABLE_FAN_PIN = 28;

const uint32_t SERIAL_BAUD_RATE = 115200;
const uint16_t LOOP_DELAY = 500;
const uint16_t DELAY = 4000;

// Instantiate TMC51X0
TMC51X0 tmc5160;
uint32_t target_velocity;

void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);

  pinMode(ENABLE_VIO_PIN, OUTPUT);
  digitalWrite(ENABLE_VIO_PIN, HIGH);

  pinMode(ENABLE_FAN_PIN, OUTPUT);
  digitalWrite(ENABLE_FAN_PIN, HIGH);

  delay(LOOP_DELAY);

  tmc5160.printer.setup(Serial);

#if defined(ARDUINO_ARCH_RP2040)
  spi.setSCK(SCK_PIN);
  spi.setTX(TX_PIN);
  spi.setRX(RX_PIN);
#endif
  spi.begin();
  tmc5160.setupSpi(spi_parameters);

  tmc5160.converter.setup(converter_parameters);

  tmc51x0::DriverParameters driver_parameters_chip = tmc5160.converter.driverParametersRealToChip(driver_parameters_real);
  tmc5160.driver.setup(driver_parameters_chip);

  tmc5160.controller.writeMaxAcceleration(tmc5160.converter.accelerationRealToChip(MAX_ACCELERATION));
  tmc5160.controller.writeRampMode(RAMP_MODE);

  tmc5160.driver.enable();

  tmc5160.controller.rampToZeroVelocity();
  while (!tmc5160.controller.zeroVelocity())
  {
    Serial.println("Waiting for zero velocity.");
    delay(LOOP_DELAY);
  }
  delay(LOOP_DELAY);

  target_velocity = MIN_TARGET_VELOCITY;
  tmc5160.controller.writeMaxVelocity(tmc5160.converter.velocityRealToChip(target_velocity));
}

void loop()
{
  tmc5160.printer.getStoredAndPrintPwmconf();
  tmc5160.printer.readAndPrintPwmScale();
  tmc5160.printer.readAndPrintPwmAuto();

  uint8_t actual_current_scaling = tmc5160.driver.readActualCurrentScaling();
  Serial.print("actual_current_scaling: ");
  Serial.println(actual_current_scaling);
  Serial.println("--------------------------");

  Serial.print("acceleration (rotations per second per second): ");
  Serial.println(MAX_ACCELERATION);
  Serial.print("target_velocity (rotations per minute): ");
  Serial.println(target_velocity);
  Serial.println("--------------------------");

  if (!tmc5160.controller.velocityReached())
  {
    Serial.println("Waiting to reach target velocity.");
    delay(LOOP_DELAY);
  }
  Serial.println("Target velocity reached!");
  Serial.println("--------------------------");

  Serial.println("--------------------------");

  delay(DELAY);

  target_velocity += TARGET_VELOCITY_INC;
  if (target_velocity > MAX_TARGET_VELOCITY)
  {
    target_velocity = MIN_TARGET_VELOCITY;
  }
  tmc5160.controller.writeMaxVelocity(tmc5160.converter.velocityRealToChip(target_velocity));
}
