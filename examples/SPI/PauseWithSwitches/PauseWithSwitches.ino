#include <TMC51X0.hpp>


#if defined(ARDUINO_ARCH_RP2040)
SPIClassRP2040 & spi = SPI;
size_t SCK_PIN = 18;
size_t TX_PIN = 19;
size_t RX_PIN = 20;
#else
SPIClass & spi = SPI;
#endif

const auto spi_parameters =
  tmc51x0::SpiParameters{}
    .withSpi(&spi)
    .withChipSelectPin(8);

const tmc51x0::ConverterParameters converter_parameters =
{
  // .clock_frequency_mhz = 16, // (typical external clock)
  .microsteps_per_real_position_unit = 51200,
  .seconds_per_real_velocity_unit = 60
};
// clock_frequency_mhz default is 12 (internal clock)
// set clock_frequency_mhz if using external clock instead
// 200 fullsteps per revolution for many steppers * 256 microsteps per fullstep = 51200
// one "real unit" in this example is one rotation of the motor shaft
// rotations/s -> rotations/min
// rotations/(s^2) -> (rotations/min)/s

const tmc51x0::DriverParameters driver_parameters_real =
{
  .run_current = 100, // (percent)
  .pwm_offset = 30, // (percent)
  .pwm_gradient = 10, // (percent)
  .stealth_chop_threshold = 60, // (rotations/min)
};

const tmc51x0::ControllerParameters controller_parameters_real =
{
  .ramp_mode = tmc51x0::VelocityPositiveMode,
  .max_velocity = 45, // (rotations/min)
  .max_acceleration = 45, // ((rotations/min)/s)
};

const tmc51x0::SwitchParameters switch_parameters_running =
{
  .left_stop_enabled = false,
  .right_stop_enabled = false,
  .invert_left_polarity = false, // left switch permanently tied to ground
  .invert_right_polarity = false, // right switch permanently tied to ground
};

const tmc51x0::SwitchParameters switch_parameters_paused =
{
  .left_stop_enabled = true,
  .right_stop_enabled = true,
  .invert_left_polarity = true, // left switch permanently tied to ground
  .invert_right_polarity = true, // right switch permanently tied to ground
};

const uint32_t SERIAL_BAUD_RATE = 115200;
const uint16_t LOOP_DELAY = 4000;
const uint16_t PAUSE_RUN_DELAY = 4000;

// global variables
TMC51X0 stepper;

void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);

#if defined(ARDUINO_ARCH_RP2040)
  spi.setSCK(SCK_PIN);
  spi.setTX(TX_PIN);
  spi.setRX(RX_PIN);
#endif
  spi.begin();
  stepper.setupSpi(spi_parameters);

  stepper.converter.setup(converter_parameters);

  tmc51x0::DriverParameters driver_parameters_chip = stepper.converter.driverParametersRealToChip(driver_parameters_real);
  stepper.driver.setup(driver_parameters_chip);

  tmc51x0::ControllerParameters controller_parameters_chip = stepper.converter.controllerParametersRealToChip(controller_parameters_real);
  stepper.controller.setup(controller_parameters_chip);

  while (!stepper.communicating())
  {
    Serial.println("No communication detected, check motor power and connections.");
    delay(LOOP_DELAY);
  }

  while (stepper.controller.stepAndDirectionMode())
  {
    Serial.println("Step and Direction mode enabled so SPI/UART motion commands will not work!");
    delay(LOOP_DELAY);
  }

  stepper.driver.enable();

  stepper.controller.beginRampToZeroVelocity();
  while (!stepper.controller.zeroVelocity())
  {
    Serial.println("Waiting for zero velocity.");
    delay(LOOP_DELAY);
  }
  stepper.controller.endRampToZeroVelocity();
  stepper.controller.zeroActualPosition();
}

void loop()
{
  stepper.controller.setupSwitches(switch_parameters_running);
  Serial.println("running... ");
  delay(PAUSE_RUN_DELAY);
  stepper.controller.setupSwitches(switch_parameters_paused);
  Serial.println("paused... ");
  delay(PAUSE_RUN_DELAY);
  Serial.println("--------------------------");
}
