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
  .spi_ptr = &spi,
  .chip_select_pin = 8
};

const tmc51x0::ConverterParameters converter_parameters =
{
  // .clock_frequency_mhz = 16, // (typical external clock)
  .microsteps_per_real_position_unit = 142
};
// clock_frequency_mhz default is 12 (internal clock)
// set clock_frequency_mhz if using external clock instead
// 200 fullsteps per revolution for many steppers * 256 microsteps per fullstep
// 51200 microsteps per revolution / 360 degrees per revolution ~= 142 microsteps per degree
// one "real unit" in this example is one degree of rotation

const tmc51x0::DriverParameters driver_parameters_real =
{
  .run_current = 25, // (percent)
  .stealth_chop_threshold = 20, // (degrees/s)
};

const tmc51x0::ControllerParameters controller_parameters_real =
{
  .ramp_mode = tmc51x0::POSITION,
  .max_velocity = 30, // (degrees/s)
  .max_acceleration = 5, // ((degrees/s)/s)
  .start_velocity = 1, // (degrees/s)
  .stop_velocity = 5, // (degrees/s)
  .first_velocity = 15, // (degrees/s)
  .first_acceleration = 10, // ((degrees/s)/s)
  .max_deceleration = 10, // ((degrees/s)/s)
  .first_deceleration = 15, // ((degrees/s)/s)
};

const tmc51x0::HomeParameters home_parameters_homing_to_switch_real =
{
  25, // run_current (percent)
  20, // hold_current (percent)
  -360, // target_position (degrees)
  10, // velocity (degrees/s)
  5, // acceleration ((degrees/s)/s)
  100 // zero_wait_duration (milliseconds)
};

const tmc51x0::SwitchParameters switch_parameters_homing_to_switch =
{
  true, // left_stop_enabled
  false, // right_stop_enabled
  false, // invert_left_polarity
  false, // invert_right_polarity
  false, // swap_left_right
  false, // latch_left_active
  false, // latch_left_inactive
  false, // latch_right_active
  false, // latch_right_inactive
  false // latch_encoder_enabled
};

const tmc51x0::HomeParameters home_parameters_homing_off_switch_real =
{
  25, // run_current (percent)
  20, // hold_current (percent)
  60, // target_position (degrees)
  1, // velocity (degrees/s)
  1, // acceleration ((degrees/s)/s)
  100 // zero_wait_duration (milliseconds)
};

const tmc51x0::SwitchParameters switch_parameters_homing_off_switch =
{
  false, // left_stop_enabled
  true, // right_stop_enabled
  false, // invert_left_polarity
  true, // invert_right_polarity
  true, // swap_left_right
  false, // latch_left_active
  false, // latch_left_inactive
  false, // latch_right_active
  false, // latch_right_inactive
  false // latch_encoder_enabled
};

const int32_t TARGET_POSITION = 100;  // degrees

const uint32_t SERIAL_BAUD_RATE = 115200;
const uint16_t LOOP_DELAY = 500;
const uint16_t PAUSE_DELAY = 4000;

// global variables
TMC51X0 tmc5130;
tmc51x0::ControllerParameters controller_parameters_chip;
tmc51x0::HomeParameters home_parameters_homing_to_switch_chip;
tmc51x0::HomeParameters home_parameters_homing_off_switch_chip;

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

  home_parameters_homing_to_switch_chip = tmc5130.converter.homeParametersRealToChip(home_parameters_homing_to_switch_real);
  home_parameters_homing_off_switch_chip = tmc5130.converter.homeParametersRealToChip(home_parameters_homing_off_switch_real);

  while (!tmc5130.communicating())
  {
    Serial.println("No communication detected, check motor power and connections.");
    delay(LOOP_DELAY);
  }

  while (tmc5130.controller.stepAndDirectionMode())
  {
    Serial.println("Step and Direction mode enabled so SPI/UART motion commands will not work!");
    delay(LOOP_DELAY);
  }

  tmc5130.driver.enable();

  tmc5130.controller.beginRampToZeroVelocity();
  while (not tmc5130.controller.zeroVelocity())
  {
    Serial.println("Waiting for zero velocity.");
    delay(LOOP_DELAY);
  }
  tmc5130.controller.endRampToZeroVelocity();
}

void loop()
{
  Serial.println("Waiting...");
  delay(PAUSE_DELAY);

  Serial.println("Homing to switch...");
  tmc5130.beginHomeToSwitch(home_parameters_homing_to_switch_chip, switch_parameters_homing_to_switch);
  while (not tmc5130.homed())
  {
    int32_t actual_position_chip = tmc5130.controller.readActualPosition();
    int32_t actual_position_real = tmc5130.converter.positionChipToReal(actual_position_chip);
    Serial.print("actual position (degrees): ");
    Serial.println(actual_position_real);
    delay(LOOP_DELAY);
  }
  tmc5130.endHome();
  Serial.println("Homed to switch!");

  Serial.println("Waiting...");
  delay(PAUSE_DELAY);

  Serial.println("Homing off switch...");
  tmc5130.beginHomeToSwitch(home_parameters_homing_off_switch_chip, switch_parameters_homing_off_switch);
  while (not tmc5130.homed())
  {
    int32_t actual_position_chip = tmc5130.controller.readActualPosition();
    int32_t actual_position_real = tmc5130.converter.positionChipToReal(actual_position_chip);
    Serial.print("actual position (degrees): ");
    Serial.println(actual_position_real);
    delay(LOOP_DELAY);
  }
  tmc5130.endHome();
  Serial.println("Homed off switch!");

  Serial.println("Waiting...");
  delay(PAUSE_DELAY);

  int32_t target_position_chip = tmc5130.converter.positionRealToChip(TARGET_POSITION);
  tmc5130.controller.writeTargetPosition(target_position_chip);
  Serial.print("Moving to target position (degrees): ");
  Serial.print(TARGET_POSITION);
  Serial.println("...");

  while (not tmc5130.controller.positionReached())
  {
    int32_t actual_position_chip = tmc5130.controller.readActualPosition();
    int32_t actual_position_real = tmc5130.converter.positionChipToReal(actual_position_chip);
    Serial.print("actual position (degrees): ");
    Serial.println(actual_position_real);
    delay(LOOP_DELAY);
  }
  Serial.println("Target position reached!");
  delay(PAUSE_DELAY);

  Serial.println("--------------------------");
  delay(LOOP_DELAY);
}
