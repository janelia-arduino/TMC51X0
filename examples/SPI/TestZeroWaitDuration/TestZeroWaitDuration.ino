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
  5, // hold_current (percent)
  0, // hold_delay (percent)
  15, // pwm_offset (percent)
  5, // pwm_gradient (percent)
  false, // automatic_current_control_enabled
  tmc51x0::FORWARD, // motor_direction
  tmc51x0::NORMAL, // standstill_mode
  tmc51x0::SPREAD_CYCLE, // chopper_mode
  20, // stealth_chop_threshold (degrees/s)
  true, // stealth_chop_enabled
  25, // cool_step_threshold (degrees/s)
  1, // cool_step_min
  0, // cool_step_max
  true, // cool_step_enabled
  100, // high_velocity_threshold (degrees/s)
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
  30, // max_velocity (degrees/s)
  5, // max_acceleration ((degrees/s)/s)
  1, // start_velocity (degrees/s)
  5, // stop_velocity (degrees/s)
  15, // first_velocity (degrees/s)
  10, // first_acceleration ((degrees/s)/s)
  10, // max_deceleration ((degrees/s)/s)
  15, // first_deceleration ((degrees/s)/s)
  1500 // zero_wait_duration (milliseconds)
};

const int32_t TARGET_POSITION_0 = 10;  // degrees
const int32_t TARGET_POSITION_1 = 60;  // degrees

const uint32_t SERIAL_BAUD_RATE = 115200;
const uint16_t LOOP_DELAY = 500;
const uint16_t PAUSE_DELAY = 4000;

// global variables
TMC51X0 tmc5130;
tmc51x0::ControllerParameters controller_parameters_chip;
int32_t target_position;
int32_t target_position_0_chip;
int32_t target_position_1_chip;
uint32_t time_position_reached;
uint32_t time_zero_wait_finished;

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

  target_position_0_chip = tmc5130.converter.positionRealToChip(TARGET_POSITION_0);
  target_position_1_chip = tmc5130.converter.positionRealToChip(TARGET_POSITION_1);
  target_position = target_position_0_chip;
}

void loop()
{
  Serial.print("setting target_position (chip): ");
  Serial.println(target_position);
  tmc5130.controller.writeTargetPosition(target_position);

  while (not tmc5130.controller.positionReached())
  {
  }
  time_position_reached = millis();

  while (tmc5130.controller.zeroWaitActive())
  {
  }
  time_zero_wait_finished = millis();

  Serial.print("zero_wait_duration (milliseconds): ");
  Serial.println(controller_parameters_real.zero_wait_duration);
  Serial.print("zero_wait_duration (chip): ");
  Serial.println(controller_parameters_chip.zero_wait_duration);
  Serial.print("measured zero_wait_duration (milliseconds): ");
  Serial.println(time_zero_wait_finished - time_position_reached);
  Serial.println("--------------------------");

  delay(PAUSE_DELAY);

  if (target_position == target_position_0_chip)
  {
    target_position = target_position_1_chip;
  }
  else
  {
    target_position = target_position_0_chip;
  }
}
