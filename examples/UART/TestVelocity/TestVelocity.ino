#include <TMC51X0.hpp>


#if !defined(ARDUINO_ARCH_RP2040)
HardwareSerial & uart = Serial2;
#else
SerialUART & uart = Serial2;
size_t TX_PIN = 4;
size_t RX_PIN = 5;
#endif

const tmc51x0::UartParameters uart_parameters =
{
  .uart_ptr = &uart,
  .enable_txrx_pin = 14
};
const uint32_t UART_BAUD_RATE = 115200;

const uint32_t SERIAL_BAUD_RATE = 115200;
const uint16_t LOOP_DELAY = 1000;

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
  .global_current_scaler = 50, // (percent)
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

// global variables
TMC51X0 stepper;
uint32_t target_velocity;
tmc51x0::RampMode ramp_mode = tmc51x0::VelocityPositiveMode;

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
  stepper.printer.readClearAndPrintGstat();
  stepper.printer.readAndPrintRampStat();
  stepper.printer.readAndPrintDrvStatus();
  stepper.printer.readAndPrintPwmScale();

  Serial.print("target_velocity (rotations per minute): ");
  Serial.println(controller_parameters_real.max_velocity);
  uint32_t chip_velocity = stepper.converter.velocityRealToChip(controller_parameters_real.max_velocity);
  Serial.print("chip_velocity (chip units): ");
  Serial.println(chip_velocity);
  Serial.println("--------------------------");

  uint32_t actual_velocity_chip = stepper.controller.readActualVelocity();
  Serial.print("actual_velocity (chip units): ");
  Serial.println(actual_velocity_chip);
  uint32_t actual_velocity_real = stepper.converter.velocityChipToReal(actual_velocity_chip);
  Serial.print("actual_velocity (rotations per minute): ");
  Serial.println(actual_velocity_real);
  Serial.println("--------------------------");

  int32_t actual_position_chip = stepper.controller.readActualPosition();
  Serial.print("actual position (chip units): ");
  Serial.println(actual_position_chip);
  int32_t actual_position_real = stepper.converter.positionChipToReal(actual_position_chip);
  Serial.print("actual position (rotations): ");
  Serial.println(actual_position_real);
  Serial.println("--------------------------");

  Serial.println("--------------------------");

  delay(LOOP_DELAY);
}
