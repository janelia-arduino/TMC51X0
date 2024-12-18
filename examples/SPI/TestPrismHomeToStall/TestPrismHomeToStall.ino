#include <TMC51X0.hpp>


#if defined(ARDUINO_ARCH_RP2040)
SPIClassRP2040 & spi = SPI1;
size_t SCK_PIN = 10;
size_t TX_PIN = 11;
size_t RX_PIN = 12;
#else
SPIClass & spi = SPI;
#endif

const tmc51x0::SpiParameters spi_parameters =
{
  spi,
  1000000, // clock_rate
  14 // chip_select_pin
};

const tmc51x0::ConverterParameters converter_parameters =
{
  16, // clock_frequency_mhz
  4881 // microsteps_per_real_unit
};
// external clock is 16MHz
// 200 fullsteps per revolution for many steppers * 256 microsteps per fullstep
// 10.49 millimeters per revolution leadscrew -> 51200 / 10.49 ~= 4881
// one "real unit" in this example is one millimeters of linear travel

const tmc51x0::DriverParameters driver_parameters_real =
{
  100, // global_current_scaler (percent)
  50, // run_current (percent)
  20, // hold_current (percent)
  0, // hold_delay (percent)
  15, // pwm_offset (percent)
  5, // pwm_gradient (percent)
  false, // automatic_current_control_enabled
  tmc51x0::REVERSE, // motor_direction
  tmc51x0::NORMAL, // standstill_mode
  tmc51x0::SPREAD_CYCLE, // chopper_mode
  10, // stealth_chop_threshold (millimeters/s)
  true, // stealth_chop_enabled
  50, // cool_step_threshold (millimeters/s)
  1, // cool_step_min
  0, // cool_step_max
  true, // cool_step_enabled
  90, // high_velocity_threshold (millimeters/s)
  false, // high_velocity_fullstep_enabled
  false, // high_velocity_chopper_switch_enabled
  1, // stall_guard_threshold
  false, // stall_guard_filter_enabled
  true, // short_to_ground_protection_enabled
  3, // enabled_toff
  tmc51x0::CLOCK_CYCLES_36, // comparator_blank_time
  37, // dc_time
  3 // dc_stall_guard_threshold
};

const tmc51x0::ControllerParameters controller_parameters_real =
{
  tmc51x0::POSITION, // ramp_mode
  tmc51x0::HARD, // stop_mode
  20, // max_velocity (millimeters/s)
  2, // max_acceleration ((millimeters/s)/s)
  1, // start_velocity (millimeters/s)
  5, // stop_velocity (millimeters/s)
  10, // first_velocity (millimeters/s)
  10, // first_acceleration ((millimeters/s)/s)
  20, // max_deceleration ((millimeters/s)/s)
  25, // first_deceleration ((millimeters/s)/s)
  0, // zero_wait_duration (milliseconds)
  false // stall_stop_enabled
};

const tmc51x0::HomeParameters home_parameters_real =
{
  50, // run_current (percent)
  20, // hold_current (percent)
  -1000, // target_position (millimeters)
  20, // velocity (millimeters/s)
  2, // acceleration ((millimeters/s)/s)
  100 // zero_wait_duration (milliseconds)
};

const tmc51x0::StallParameters stall_parameters_cool_step_real =
{
  tmc51x0::COOL_STEP, // stall_mode
  10, // stall_guard_threshold
  15 // cool_step_threshold (millimeters/s)
};

const tmc51x0::StallParameters stall_parameters_dc_step_real =
{
  tmc51x0::DC_STEP, // stall_mode
  10, // stall_guard_threshold
  5, // cool_step_threshold (millimeters/s)
  4, // min_dc_step_velocity (millimeters/s)
  3 // dc_stall_guard_threshold
};

const int32_t TARGET_POSITION = 100;  // millimeters

const size_t ENABLE_POWER_PIN = 15;
const uint8_t ENABLE_POWER_POLARITY = HIGH;
const uint16_t RESET_DELAY = 5000;

const uint32_t SERIAL_BAUD_RATE = 115200;
const uint16_t LOOP_DELAY = 500;
const uint16_t PAUSE_DELAY = 4000;

// global variables
TMC51X0 prism;
tmc51x0::ControllerParameters controller_parameters_chip;
tmc51x0::HomeParameters home_parameters_chip;
tmc51x0::StallParameters stall_parameters_cool_step_chip;
tmc51x0::StallParameters stall_parameters_dc_step_chip;
bool stall_using_cool_step = true;

void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);

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
  prism.setupSpi(spi_parameters);

  prism.converter.setup(converter_parameters);

  tmc51x0::DriverParameters driver_parameters_chip = prism.converter.driverParametersRealToChip(driver_parameters_real);
  prism.driver.setup(driver_parameters_chip);

  controller_parameters_chip = prism.converter.controllerParametersRealToChip(controller_parameters_real);
  prism.controller.setup(controller_parameters_chip);

  home_parameters_chip = prism.converter.homeParametersRealToChip(home_parameters_real);
  stall_parameters_cool_step_chip = prism.converter.stallParametersRealToChip(stall_parameters_cool_step_real);
  stall_parameters_dc_step_chip = prism.converter.stallParametersRealToChip(stall_parameters_dc_step_real);

  prism.driver.enable();

  prism.controller.beginRampToZeroVelocity();
  while (not prism.controller.zeroVelocity())
  {
    Serial.println("Waiting for zero velocity.");
    delay(LOOP_DELAY);
  }
  prism.controller.endRampToZeroVelocity();
}

void loop()
{
  Serial.println("Waiting...");
  delay(PAUSE_DELAY);

  if (stall_using_cool_step)
  {
    stall_using_cool_step = false;
    Serial.println("Homing to stall using cool step...");
    prism.beginHomeToStall(home_parameters_chip, stall_parameters_cool_step_chip);
  }
  else
  {
    stall_using_cool_step = true;
    Serial.println("Homing to stall using dc step...");
    prism.beginHomeToStall(home_parameters_chip, stall_parameters_dc_step_chip);
  }
  while (not prism.homed())
  {
    prism.printer.readAndPrintDrvStatus();
    int32_t actual_position_chip = prism.controller.readActualPosition();
    int32_t actual_position_real = prism.converter.positionChipToReal(actual_position_chip);
    Serial.print("actual position (millimeters): ");
    Serial.println(actual_position_real);
    if (stall_using_cool_step)
    {
      Serial.println("stall mode: COOL_STEP");
      Serial.print("stall guard result: ");
      Serial.println(prism.driver.readStallGuardResult());
      Serial.print("stall guard threshold: ");
      Serial.println(stall_parameters_cool_step_real.stall_guard_threshold);
    }
    else
    {
      Serial.println("stall mode: DC_STEP");
      Serial.print("dc stall guard threshold: ");
      Serial.println(stall_parameters_dc_step_real.dc_stall_guard_threshold);
    }
    delay(LOOP_DELAY);
  }
  prism.endHome();
  Serial.println("Homed!");

  Serial.println("Waiting...");
  delay(PAUSE_DELAY);

  int32_t target_position_chip = prism.converter.positionRealToChip(TARGET_POSITION);
  prism.controller.writeTargetPosition(target_position_chip);
  Serial.print("Moving to target position (millimeters): ");
  Serial.print(TARGET_POSITION);
  Serial.println("...");

  while (not prism.controller.positionReached())
  {
    // prism.printer.readAndPrintRampStat();
    // prism.printer.readAndPrintDrvStatus();
    int32_t actual_position_chip = prism.controller.readActualPosition();
    int32_t actual_position_real = prism.converter.positionChipToReal(actual_position_chip);
    Serial.print("actual position (millimeters): ");
    Serial.println(actual_position_real);
    Serial.print("stall_guard_result: ");
    Serial.println(prism.driver.readStallGuardResult());
    delay(LOOP_DELAY);
  }
  Serial.println("Target position reached!");
  delay(PAUSE_DELAY);

  Serial.println("--------------------------");
  delay(LOOP_DELAY);
}
