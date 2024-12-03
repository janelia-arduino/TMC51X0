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
  51200, // microsteps_per_real_unit
  60 // seconds_per_real_velocity_unit
};
// external clock is 16MHz
// 200 fullsteps per revolution for many steppers * 256 microsteps per fullstep
// one "real unit" in this example is one rotation of the motor shaft
// rotations/s -> rotations/min
// rotations/(s^2) -> (rotations/min)/s

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

const tmc51x0::ControllerParameters controller_parameters_real =
{
  tmc51x0::VELOCITY_POSITIVE, // ramp_mode
  tmc51x0::HARD, // stop_mode
  45, // max_velocity (rotations/min)
  45, // max_acceleration ((rotations/min)/s)
};

const uint32_t SERIAL_BAUD_RATE = 115200;
const uint16_t LOOP_DELAY = 4000;

// Instantiate TMC51X0
TMC51X0 tmc5130;

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

  tmc51x0::ControllerParameters controller_parameters_chip = tmc5130.converter.controllerParametersRealToChip(controller_parameters_real);
  tmc5130.controller.setup(controller_parameters_chip);

  tmc5130.driver.enable();

  tmc5130.controller.beginRampToZeroVelocity();
  while (!tmc5130.controller.zeroVelocity())
  {
    Serial.println("Waiting for zero velocity.");
    delay(LOOP_DELAY);
  }
  tmc5130.controller.endRampToZeroVelocity();
  tmc5130.controller.zeroActualPosition();
}

void loop()
{
  tmc5130.printer.readClearAndPrintGstat();
  tmc5130.printer.readAndPrintRampStat();
  tmc5130.printer.readAndPrintDrvStatus();
  tmc5130.printer.readAndPrintPwmScale();

  Serial.print("target_velocity (rotations per minute): ");
  Serial.println(controller_parameters_real.max_velocity);
  uint32_t chip_velocity = tmc5130.converter.velocityRealToChip(controller_parameters_real.max_velocity);
  Serial.print("chip_velocity (chip units): ");
  Serial.println(chip_velocity);
  Serial.println("--------------------------");

  uint32_t actual_velocity_chip = tmc5130.controller.readActualVelocity();
  Serial.print("actual_velocity (chip units): ");
  Serial.println(actual_velocity_chip);
  uint32_t actual_velocity_real = tmc5130.converter.velocityChipToReal(actual_velocity_chip);
  Serial.print("actual_velocity (rotations per minute): ");
  Serial.println(actual_velocity_real);
  Serial.println("--------------------------");

  int32_t actual_position_chip = tmc5130.controller.readActualPosition();
  Serial.print("actual position (chip units): ");
  Serial.println(actual_position_chip);
  int32_t actual_position_real = tmc5130.converter.positionChipToReal(actual_position_chip);
  Serial.print("actual position (rotations): ");
  Serial.println(actual_position_real);
  Serial.println("--------------------------");

  Serial.println("--------------------------");

  delay(LOOP_DELAY);
}
