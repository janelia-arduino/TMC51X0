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
    .withMicrostepsPerRealPositionUnit(51200)
    .withSecondsPerRealVelocityUnit(60);
// clock_frequency_mhz default is 12 (internal clock)
// set clock_frequency_mhz if using external clock instead
// 200 fullsteps per revolution for many steppers * 256 microsteps per fullstep = 51200
// one "real unit" in this example is one rotation of the motor shaft
// rotations/s -> rotations/min
// rotations/(s^2) -> (rotations/min)/s

const auto driver_parameters_real =
  tmc51x0::DriverParameters{}
    .withRunCurrent(100) // (percent)
    .withPwmOffset(30) // (percent)
    .withPwmGradient(10) // (percent)
    .withStealthChopThreshold(60); // (rotations/min)

const auto controller_parameters_real =
  tmc51x0::ControllerParameters{}
    .withRampMode(tmc51x0::VelocityPositiveMode)
    .withMaxVelocity(45) // (rotations/min)
    .withMaxAcceleration(45); // ((rotations/min)/s)

const auto switch_parameters_running =
  tmc51x0::SwitchParameters{}
    .withLeftStopEnabled(false)
    .withRightStopEnabled(false)
    .withInvertLeftPolarity(false) // left switch permanently tied to ground
    .withInvertRightPolarity(false); // right switch permanently tied to ground

const auto switch_parameters_paused =
  tmc51x0::SwitchParameters{}
    .withLeftStopEnabled(true)
    .withRightStopEnabled(true)
    .withInvertLeftPolarity(true) // left switch permanently tied to ground
    .withInvertRightPolarity(true); // right switch permanently tied to ground

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
