#include <TMC51X0.hpp>


#if !defined(ARDUINO_ARCH_RP2040)
HardwareSerial & uart = Serial2;
#else
SerialUART & uart = Serial2;
size_t TX_PIN = 4;
size_t RX_PIN = 5;
#endif

const auto uart_parameters =
  tmc51x0::UartParameters{}
    .withUart(&uart)
    .withEnableTxRxPin(14);

const uint32_t UART_BAUD_RATE = 115200;

const auto converter_parameters =
  tmc51x0::ConverterParameters{}
    //.withClockFrequencyMHz(16) // (typical external clock)
    .withMicrostepsPerRealPositionUnit(4881);
// clock_frequency_mhz default is 12 (internal clock)
// set clock_frequency_mhz if using external clock instead
// 200 fullsteps per revolution for many steppers * 256 microsteps per fullstep
// 10.49 millimeters per revolution leadscrew -> 51200 / 10.49 ~= 4881
// one "real unit" in this example is one millimeters of linear travel

const auto driver_parameters_real =
  tmc51x0::DriverParameters{}
    .withRunCurrent(100) // (percent)
    .withPwmOffset(30) // (percent)
    .withPwmGradient(10) // (percent)
    .withMotorDirection(tmc51x0::ReverseDirection)
    .withStealthChopThreshold(100); // (millimeters/s)

const auto controller_parameters_real =
  tmc51x0::ControllerParameters{}
    .withRampMode(tmc51x0::PositionMode)
    .withMaxVelocity(20) // (millimeters/s)
    .withMaxAcceleration(2) // ((millimeters/s)/s)
    .withStartVelocity(1) // (millimeters/s)
    .withStopVelocity(5) // (millimeters/s)
    .withFirstVelocity(10) // (millimeters/s)
    .withFirstAcceleration(10) // ((millimeters/s)/s)
    .withMaxDeceleration(20) // ((millimeters/s)/s)
    .withFirstDeceleration(25); // ((millimeters/s)/s)

const int32_t MIN_TARGET_POSITION = 20;  // millimeters
const int32_t MAX_TARGET_POSITION = 600;  // millimeters

const uint32_t SERIAL_BAUD_RATE = 115200;
const uint16_t LOOP_DELAY = 2000;

// global variables
TMC51X0 stepper;
uint32_t target_position;

void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);

#if defined(ARDUINO_ARCH_RP2040)
  uart.setTX(TX_PIN);
  uart.setRX(RX_PIN);
#endif
  uart.begin(UART_BAUD_RATE);

  stepper.setupUart(uart_parameters);

  stepper.converter.setup(converter_parameters);

  tmc51x0::DriverParameters driver_parameters_chip = stepper.converter.driverParametersRealToChip(driver_parameters_real);
  stepper.driver.setup(driver_parameters_chip);
  // stepper.driver.setEnableHardwarePin(ENABLE_HARDWARE_PIN);

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

  randomSeed(analogRead(A0));
  long random_delay = random(5000);
  delay(random_delay);

  target_position = MIN_TARGET_POSITION;
  stepper.controller.writeTargetPosition(stepper.converter.positionRealToChip(target_position));

}

void loop()
{
  stepper.printer.readAndPrintRampStat();
  stepper.printer.readAndPrintDrvStatus();

  Serial.print("max_velocity (millimeters per second): ");
  Serial.println(controller_parameters_real.max_velocity);

  int32_t actual_velocity_chip = stepper.controller.readActualVelocity();
  int32_t actual_velocity_real = stepper.converter.velocityChipToReal(actual_velocity_chip);
  Serial.print("actual_velocity (millimeters per second): ");
  Serial.println(actual_velocity_real);

  int32_t actual_position_chip = stepper.controller.readActualPosition();
  int32_t actual_position_real = stepper.converter.positionChipToReal(actual_position_chip);
  Serial.print("actual position (millimeters): ");
  Serial.println(actual_position_real);

  int32_t target_position_chip = stepper.controller.readTargetPosition();
  int32_t target_position_real = stepper.converter.positionChipToReal(target_position_chip);
  Serial.print("target position (millimeters): ");
  Serial.println(target_position_real);
  Serial.println("--------------------------");

  if (stepper.controller.positionReached())
  {
    Serial.println("Reached target position!");
    Serial.println("--------------------------");
    long random_delay = random(3000);
    delay(random_delay);
    if (target_position == MIN_TARGET_POSITION)
    {
      target_position = MAX_TARGET_POSITION;
    }
    else
    {
      target_position = MIN_TARGET_POSITION;
    }
    stepper.controller.writeTargetPosition(stepper.converter.positionRealToChip(target_position));
  }

  Serial.println("--------------------------");
  delay(LOOP_DELAY);
}
