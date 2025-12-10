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

const auto converter_parameters =
  tmc51x0::ConverterParameters{}
    //.withClockFrequencyMHz(16) // (typical external clock)
    .withMicrostepsPerRealPositionUnit(8149);
// clock_frequency_mhz default is 12 (internal clock)
// set clock_frequency_mhz if using external clock instead
// 200 fullsteps per revolution for many steppers * 256 microsteps per fullstep
// 51200 microsteps per revolution / 6.283 radians per revolution ~= 8149 microsteps per radian
// one "real unit" in this example is one radian of rotation

const auto driver_parameters_real =
  tmc51x0::DriverParameters{}
    .withRunCurrent(50) // (percent)
    .withHoldCurrent(10) // (percent)
    .withPwmOffset(30) // (percent)
    .withPwmGradient(10) // (percent)
    .withStealthChopThreshold(100); // (radians/s)

const auto controller_parameters_real =
  tmc51x0::ControllerParameters{}
    .withRampMode(tmc51x0::PositionMode)
    .withMaxVelocity(20) // (radians/s)
    .withMaxAcceleration(2); // ((radians/s)/s)

const auto home_parameters_homing_to_switch_real =
  tmc51x0::HomeParameters{}
    .withRunCurrent(25) // (percent)
    .withHoldCurrent(10) // (percent)
    .withTargetPosition(-100) // (radians)
    .withVelocity(20) // (radians/s)
    .withAcceleration(2); // ((radians/s)/s)

const auto switch_parameters_homing_to_switch =
  tmc51x0::SwitchParameters{}
    .withLeftStopEnabled(true);

const auto home_parameters_homing_off_switch_real =
  tmc51x0::HomeParameters{}
    .withRunCurrent(25) // (percent)
    .withHoldCurrent(10) // (percent)
    .withTargetPosition(2) // (radians)
    .withVelocity(1) // (radians/s)
    .withAcceleration(1); // ((radians/s)/s)

const auto switch_parameters_homing_off_switch =
  tmc51x0::SwitchParameters{}
    .withRightStopEnabled(true)
    .withInvertRightPolarity(true)
    .withSwapLeftRight(true);

const int32_t MOVE_POSITION = 100;  // degrees

const uint32_t SERIAL_BAUD_RATE = 115200;
const uint16_t LOOP_DELAY = 500;
const uint16_t PAUSE_DELAY = 4000;

// global variables
TMC51X0 stepper;
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
  stepper.setupSpi(spi_parameters);

  stepper.converter.setup(converter_parameters);

  tmc51x0::DriverParameters driver_parameters_chip = stepper.converter.driverParametersRealToChip(driver_parameters_real);
  stepper.driver.setup(driver_parameters_chip);

  controller_parameters_chip = stepper.converter.controllerParametersRealToChip(controller_parameters_real);
  stepper.controller.setup(controller_parameters_chip);

  home_parameters_homing_to_switch_chip = stepper.converter.homeParametersRealToChip(home_parameters_homing_to_switch_real);
  home_parameters_homing_off_switch_chip = stepper.converter.homeParametersRealToChip(home_parameters_homing_off_switch_real);

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

  Serial.println("Homing to switch...");
  stepper.beginHomeToSwitch(home_parameters_homing_to_switch_chip, switch_parameters_homing_to_switch);
  while (not stepper.homed())
  {
    int32_t actual_position_chip = stepper.controller.readActualPosition();
    int32_t actual_position_real = stepper.converter.positionChipToReal(actual_position_chip);
    Serial.print("actual position (degrees): ");
    Serial.println(actual_position_real);
    delay(LOOP_DELAY);
  }
  stepper.endHome();
  Serial.println("Homed to switch!");

  Serial.println("Waiting...");
  delay(PAUSE_DELAY);

  Serial.println("Homing off switch...");
  stepper.beginHomeToSwitch(home_parameters_homing_off_switch_chip, switch_parameters_homing_off_switch);
  while (not stepper.homed())
  {
    int32_t actual_position_chip = stepper.controller.readActualPosition();
    int32_t actual_position_real = stepper.converter.positionChipToReal(actual_position_chip);
    Serial.print("actual position (degrees): ");
    Serial.println(actual_position_real);
    delay(LOOP_DELAY);
  }
  stepper.endHome();
  Serial.println("Homed off switch!");

  Serial.println("Waiting...");
  delay(PAUSE_DELAY);

  int32_t target_position_chip = stepper.converter.positionRealToChip(MOVE_POSITION);
  stepper.controller.writeTargetPosition(target_position_chip);
  Serial.print("Moving to another position (degrees): ");
  Serial.print(MOVE_POSITION);
  Serial.println("...");

  while (not stepper.controller.positionReached())
  {
    int32_t actual_position_chip = stepper.controller.readActualPosition();
    int32_t actual_position_real = stepper.converter.positionChipToReal(actual_position_chip);
    Serial.print("actual position (degrees): ");
    Serial.println(actual_position_real);
    delay(LOOP_DELAY);
  }
  Serial.println("Target position reached!");
  delay(PAUSE_DELAY);

  Serial.println("--------------------------");
  delay(LOOP_DELAY);
}
