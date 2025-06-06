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
  .microsteps_per_real_position_unit = 8149
};
// clock_frequency_mhz default is 12 (internal clock)
// set clock_frequency_mhz if using external clock instead
// 200 fullsteps per revolution for many steppers * 256 microsteps per fullstep
// 51200 microsteps per revolution / 6.283 radians per revolution ~= 8149 microsteps per radian
// one "real unit" in this example is one radian of rotation

const tmc51x0::DriverParameters driver_parameters_real =
{
  .run_current = 50, // (percent)
  .hold_current = 10, // (percent)
  .pwm_offset = 30, // (percent)
  .pwm_gradient = 10, // (percent)
  .stealth_chop_threshold = 100, // (radians/s)
};

const tmc51x0::ControllerParameters controller_parameters_real =
{
  .ramp_mode = tmc51x0::PositionMode,
  .max_velocity = 20, // (radians/s)
  .max_acceleration = 2, // ((radians/s)/s)
};

const tmc51x0::HomeParameters home_parameters_real =
{
  .run_current = 25, // (percent)
  .hold_current = 10, // (percent)
  .target_position = -100, // (radians)
  .velocity = 20, // (radians/s)
  .acceleration = 2, // ((radians/s)/s)
};

const tmc51x0::StallParameters stall_parameters_real =
{
  .stall_guard_threshold = 3, // -64..63, 0 default, higher is less sensitive
  .cool_step_threshold = 10 // (radians/s)
};

const int32_t MOVE_POSITION = 110;  // radians

const uint32_t SERIAL_BAUD_RATE = 115200;
const uint16_t LOOP_DELAY = 500;
const uint16_t PAUSE_DELAY = 4000;

// global variables
TMC51X0 stepper;
tmc51x0::ControllerParameters controller_parameters_chip;
tmc51x0::HomeParameters home_parameters_chip;
tmc51x0::StallParameters stall_parameters_chip;

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

  controller_parameters_chip = stepper.converter.controllerParametersRealToChip(controller_parameters_real);
  stepper.controller.setup(controller_parameters_chip);

  home_parameters_chip = stepper.converter.homeParametersRealToChip(home_parameters_real);
  stall_parameters_chip = stepper.converter.stallParametersRealToChip(stall_parameters_real);

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
  while (not stepper.controller.zeroVelocity())
  {
    Serial.println("Waiting for zero velocity.");
    delay(LOOP_DELAY);
  }
  stepper.controller.endRampToZeroVelocity();
}

void loop()
{
  Serial.println("Waiting...");
  delay(PAUSE_DELAY);

  Serial.println("Homing to stall...");
  stepper.beginHomeToStall(home_parameters_chip, stall_parameters_chip);

  int32_t actual_position_real;
  while (not stepper.homed())
  {
    // stepper.printer.readAndPrintDrvStatus();
    int32_t actual_position_chip = stepper.controller.readActualPosition();
    actual_position_real = stepper.converter.positionChipToReal(actual_position_chip);
    Serial.print("homing...");
    Serial.print("actual position (radians): ");
    Serial.println(actual_position_real);
    Serial.print("stall guard result: ");
    Serial.println(stepper.driver.readStallGuardResult());
    Serial.print("stall guard threshold: ");
    Serial.println(stall_parameters_real.stall_guard_threshold);
    delay(LOOP_DELAY);
  }
  stepper.endHome();
  Serial.println("Homed!");
  Serial.print("actual_position_real: ");
  Serial.println(actual_position_real);
  Serial.print("home target position: ");
  Serial.println(home_parameters_real.target_position);

  Serial.println("Waiting...");
  delay(PAUSE_DELAY);

  int32_t target_position_chip = stepper.converter.positionRealToChip(MOVE_POSITION);
  stepper.controller.writeTargetPosition(target_position_chip);
  Serial.print("Moving to another position (radians): ");
  Serial.print(MOVE_POSITION);
  Serial.println("...");

  while (not stepper.controller.positionReached())
  {
    // stepper.printer.readAndPrintRampStat();
    // stepper.printer.readAndPrintDrvStatus();
    int32_t actual_position_chip = stepper.controller.readActualPosition();
    int32_t actual_position_real = stepper.converter.positionChipToReal(actual_position_chip);
    Serial.print("actual position (radians): ");
    Serial.println(actual_position_real);
    Serial.print("stall_guard_result: ");
    Serial.println(stepper.driver.readStallGuardResult());
    delay(LOOP_DELAY);
  }
  Serial.println("Target position reached!");
  delay(PAUSE_DELAY);

  Serial.println("--------------------------");
  delay(LOOP_DELAY);
}
