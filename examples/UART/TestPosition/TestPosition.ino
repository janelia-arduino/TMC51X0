#include <TMC51X0.hpp>


#if !defined(ARDUINO_ARCH_RP2040)
HardwareSerial & uart = Serial2;
#else
SerialUART & uart = Serial2;
uint16_t TX_PIN = 4;
uint16_t RX_PIN = 5;
#endif

// UART Parameters
const tmc51x0::UartParameters uart_parameters(
  uart,
  0, // node_address
  14); // enable_txrx_pin
const uint32_t UART_BAUD_RATE = 115200;

const uint32_t SERIAL_BAUD_RATE = 115200;
const uint16_t LOOP_DELAY = 2000;

// converter constants
// external clock is 16MHz
const uint8_t CLOCK_FREQUENCY_MHZ = 16;
// 200 fullsteps per revolution for many steppers * 256 microsteps per fullstep
// 10.49 millimeters per revolution leadscrew -> 51200 / 10.49
// one "real unit" in this example is one millimeters of linear travel
constexpr uint32_t MICROSTEPS_PER_REAL_UNIT = 4881;

// driver constants
const uint8_t GLOBAL_CURRENT_SCALAR = 100; // percent
const uint8_t RUN_CURRENT = 50; // percent
const uint8_t PWM_OFFSET = 20; // percent
const uint8_t PWM_GRADIENT = 5; // percent
const tmc51x0::Driver::MotorDirection MOTOR_DIRECTION = tmc51x0::Driver::FORWARD;

const uint8_t STEALTH_CHOP_THRESHOLD = 10; // millimeters/s
const uint8_t COOL_STEP_THRESHOLD = 50; // millimeters/s
const uint8_t MIN_COOL_STEP = 1;
const uint8_t MAX_COOL_STEP = 0;
const uint8_t HIGH_VELOCITY_THRESHOLD = 90; // millimeters/s
const int8_t STALL_GUARD_THRESHOLD = 1;

// controller constants
const uint32_t START_VELOCITY = 1; // millimeters/s
const uint32_t FIRST_ACCELERATION = 10;  // millimeters/(s^2)
const uint32_t FIRST_VELOCITY = 10; // millimeters/s
const uint32_t MAX_ACCELERATION = 2; // millimeters/(s^2)
const uint32_t MAX_DECELERATION = 25;  // millimeters/(s^2)
const uint32_t FIRST_DECELERATION = 20;  // millimeters/(s^2)
const uint32_t MAX_VELOCITY = 20; // millimeters/s
const uint32_t STOP_VELOCITY = 5; // millimeters/s

const int32_t MIN_TARGET_POSITION = 20;  // millimeters
// const int32_t MAX_TARGET_POSITION = 180;  // millimeters
const int32_t MAX_TARGET_POSITION = 600;  // millimeters
const tmc51x0::Controller::RampMode RAMP_MODE = tmc51x0::Controller::POSITION;
const int32_t INITIAL_POSITION = 0;

// Instantiate TMC51X0
TMC51X0 tmc5130;
uint32_t target_position;

void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);

#if defined(ARDUINO_ARCH_RP2040)
  uart.setTX(TX_PIN);
  uart.setRX(RX_PIN);
#endif
  uart.begin(UART_BAUD_RATE);

  tmc5130.setupUart(uart_parameters);

  tmc51x0::ConverterParameters converter_parameters =
    {
      CLOCK_FREQUENCY_MHZ,
      MICROSTEPS_PER_REAL_UNIT
    };
  tmc5130.converter.setup(converter_parameters);

  // tmc5130.driver.setEnableHardwarePin(ENABLE_HARDWARE_PIN);
  tmc5130.driver.writeGlobalCurrentScaler(tmc5130.converter.percentToGlobalCurrentScaler(GLOBAL_CURRENT_SCALAR));
  tmc5130.driver.writeRunCurrent(tmc5130.converter.percentToCurrentSetting(RUN_CURRENT));
  tmc5130.driver.writePwmOffset(tmc5130.converter.percentToPwmSetting(PWM_OFFSET));
  tmc5130.driver.writePwmGradient(tmc5130.converter.percentToPwmSetting(PWM_GRADIENT));
  tmc5130.driver.writeMotorDirection(MOTOR_DIRECTION);
  tmc5130.driver.writeStealthChopThreshold(tmc5130.converter.velocityRealToTstep(STEALTH_CHOP_THRESHOLD));
  // tmc5130.driver.writeCoolStepThreshold(tmc5130.converter.velocityRealToTstep(COOL_STEP_THRESHOLD));
  // tmc5130.driver.enableCoolStep(MIN_COOL_STEP, MAX_COOL_STEP);
  // tmc5130.driver.writeHighVelocityThreshold(tmc5130.converter.velocityRealToTstep(HIGH_VELOCITY_THRESHOLD));
  // tmc5130.driver.writeStallGuardThreshold(STALL_GUARD_THRESHOLD);

  tmc5130.controller.writeFirstAcceleration(tmc5130.converter.accelerationRealToChip(FIRST_ACCELERATION));
  tmc5130.controller.writeFirstVelocity(tmc5130.converter.velocityRealToChip(FIRST_VELOCITY));
  tmc5130.controller.writeMaxAcceleration(tmc5130.converter.accelerationRealToChip(MAX_ACCELERATION));
  tmc5130.controller.writeMaxDeceleration(tmc5130.converter.accelerationRealToChip(MAX_DECELERATION));
  tmc5130.controller.writeFirstDeceleration(tmc5130.converter.accelerationRealToChip(FIRST_DECELERATION));
  tmc5130.controller.writeStopVelocity(tmc5130.converter.velocityRealToChip(STOP_VELOCITY));
  tmc5130.controller.writeRampMode(RAMP_MODE);
  tmc5130.controller.writeActualPosition(tmc5130.converter.positionRealToChip(INITIAL_POSITION));

  tmc5130.driver.enable();

  tmc5130.controller.rampToZeroVelocity();
  while (!tmc5130.controller.zeroVelocity())
  {
    Serial.println("Waiting for zero velocity.");
    delay(LOOP_DELAY);
  }

  tmc5130.controller.writeStartVelocity(tmc5130.converter.velocityRealToChip(START_VELOCITY));
  tmc5130.controller.writeMaxVelocity(tmc5130.converter.velocityRealToChip(MAX_VELOCITY));

  randomSeed(analogRead(A0));
  long random_delay = random(5000);
  delay(random_delay);

  target_position = MIN_TARGET_POSITION;
  tmc5130.controller.writeTargetPosition(tmc5130.converter.positionRealToChip(target_position));

}

void loop()
{
  tmc5130.printer.readAndPrintGconf();
  tmc5130.printer.readAndPrintRampStat();
  tmc5130.printer.readAndPrintDrvStatus();

  Serial.print("max_velocity (millimeters per second): ");
  Serial.println(MAX_VELOCITY);

  int32_t actual_velocity_chip = tmc5130.controller.readActualVelocity();
  int32_t actual_velocity_real = tmc5130.converter.velocityChipToReal(actual_velocity_chip);
  Serial.print("actual_velocity (millimeters per second): ");
  Serial.println(actual_velocity_real);

  int32_t actual_position_chip = tmc5130.controller.readActualPosition();
  int32_t actual_position_real = tmc5130.converter.positionChipToReal(actual_position_chip);
  Serial.print("actual position (millimeters): ");
  Serial.println(actual_position_real);

  int32_t target_position_chip = tmc5130.controller.readTargetPosition();
  int32_t target_position_real = tmc5130.converter.positionChipToReal(target_position_chip);
  Serial.print("target position (millimeters): ");
  Serial.println(target_position_real);
  Serial.println("--------------------------");

  if (tmc5130.controller.positionReached())
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
tmc5130.controller.writeTargetPosition(tmc5130.converter.positionRealToChip(target_position));
  }

  Serial.println("--------------------------");
  delay(LOOP_DELAY);
}
