#include <TMC51X0.hpp>


#if defined(ARDUINO_ARCH_RP2040)
SPIClassRP2040 & spi = SPI1;
size_t SCK_PIN = 10;
size_t TX_PIN = 11;
size_t RX_PIN = 12;
#else
SPIClass & spi = SPI;
#endif

const uint8_t PRISM_COUNT = 7;

tmc51x0::SpiParameters spi_parameters =
{
  spi,
  1000000 // clock_rate
};
const size_t SPI_CHIP_SELECT_PINS[PRISM_COUNT] = {14, 8, 7, 6, 5, 4, 3};

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
  50, // run_current (percent)
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
  120, // max_velocity (rotations/min)
  45, // max_acceleration ((rotations/min)/s)
};

const size_t ENABLE_POWER_PIN = 15;
const uint8_t ENABLE_POWER_POLARITY = HIGH;
const uint16_t RESET_DELAY = 5000;

const uint32_t SERIAL_BAUD_RATE = 115200;
const uint16_t LOOP_DELAY = 4000;

// Instantiate TMC51X0
TMC51X0 prisms[PRISM_COUNT];
const uint8_t PRISM_INDEX = 0;

void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);

  TMC51X0 & prism = prisms[PRISM_INDEX];

  prism.setEnablePowerPin(ENABLE_POWER_PIN);
  prism.setEnablePowerPolarity(ENABLE_POWER_POLARITY);
  prism.disablePower();
  delay(RESET_DELAY);
  prism.enablePower();
  delay(RESET_DELAY);

#if defined(ARDUINO_ARCH_RP2040)
  spi.setSCK(SCK_PIN);
  spi.setTX(TX_PIN);
  spi.setRX(RX_PIN);
#endif
  spi.begin();

  spi_parameters.chip_select_pin = SPI_CHIP_SELECT_PINS[PRISM_INDEX];
  prism.setupSpi(spi_parameters);

  prism.converter.setup(converter_parameters);

  tmc51x0::DriverParameters driver_parameters_chip = prism.converter.driverParametersRealToChip(driver_parameters_real);
  prism.driver.setup(driver_parameters_chip);

  tmc51x0::ControllerParameters controller_parameters_chip = prism.converter.controllerParametersRealToChip(controller_parameters_real);
  prism.controller.setup(controller_parameters_chip);

  prism.driver.enable();
}

void loop()
{
  TMC51X0 & prism = prisms[PRISM_INDEX];

  prism.printer.readClearAndPrintGstat();
  prism.printer.readAndPrintRampStat();
  prism.printer.readAndPrintDrvStatus();
  prism.printer.readAndPrintPwmScale();

  Serial.print("target_velocity (rotations per minute): ");
  Serial.println(controller_parameters_real.max_velocity);
  uint32_t chip_velocity = prism.converter.velocityRealToChip(controller_parameters_real.max_velocity);
  Serial.print("chip_velocity (chip units): ");
  Serial.println(chip_velocity);
  Serial.println("--------------------------");

  uint32_t actual_velocity_chip = prism.controller.readActualVelocity();
  Serial.print("actual_velocity (chip units): ");
  Serial.println(actual_velocity_chip);
  uint32_t actual_velocity_real = prism.converter.velocityChipToReal(actual_velocity_chip);
  Serial.print("actual_velocity (rotations per minute): ");
  Serial.println(actual_velocity_real);
  Serial.println("--------------------------");

  int32_t actual_position_chip = prism.controller.readActualPosition();
  Serial.print("actual position (chip units): ");
  Serial.println(actual_position_chip);
  int32_t actual_position_real = prism.converter.positionChipToReal(actual_position_chip);
  Serial.print("actual position (rotations): ");
  Serial.println(actual_position_real);
  Serial.println("--------------------------");

  Serial.println("--------------------------");

  delay(LOOP_DELAY);
}
