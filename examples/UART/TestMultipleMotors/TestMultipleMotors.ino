#include <TMC51X0.hpp>


#if !defined(ARDUINO_ARCH_RP2040)
HardwareSerial & uart = Serial2;
#else
SerialUART & uart = Serial2;
uint16_t TX_PIN = 4;
uint16_t RX_PIN = 5;
#endif

// Optional power enable
const uint16_t ENABLE_POWER_PIN = 15;
const uint8_t ENABLE_POWER_POLARITY = HIGH;

// UART Parameters
const uint32_t UART_BAUD_RATE = 115200;
const uint8_t NODE_ADDRESS = 0;

const uint8_t MOTOR_COUNT = 7;
const uint8_t ENABLE_TX_PINS[MOTOR_COUNT] = {14, 13, 12, 11, 10, 9, 8};
const uint8_t MUX_ADDRESS_0_PIN = 6;
const uint8_t MUX_ADDRESS_1_PIN = 3;
const uint8_t MUX_ADDRESS_2_PIN = 2;
const uint8_t MUX_ADDRESS_0_VALUES[MOTOR_COUNT] = {0, 1, 0, 1, 0, 1, 0};
const uint8_t MUX_ADDRESS_1_VALUES[MOTOR_COUNT] = {0, 0, 1, 1, 0, 0, 1};
const uint8_t MUX_ADDRESS_2_VALUES[MOTOR_COUNT] = {0, 0, 0, 0, 1, 1, 1};

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
const int32_t MAX_TARGET_POSITION = 600;  // millimeters
const tmc51x0::Controller::RampMode RAMP_MODE = tmc51x0::Controller::POSITION;
const int32_t INITIAL_POSITION = 0;

// Instantiate TMC51X0
TMC51X0 motors[MOTOR_COUNT];
uint32_t target_position;

void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);

  delay(RESET_DELAY);

  Serial.println("Disabling VCC");

  pinMode(ENABLE_POWER_PIN, OUTPUT);
  digitalWrite(ENABLE_POWER_PIN, ENABLE_POWER_VALUE);
  pinMode(MUX_ADDRESS_0_PIN, OUTPUT);
  pinMode(MUX_ADDRESS_1_PIN, OUTPUT);
  pinMode(MUX_ADDRESS_2_PIN, OUTPUT);

#if defined(ARDUINO_ARCH_RP2040)
  uart.setTX(TX_PIN);
  uart.setRX(RX_PIN);
#endif
  uart.begin(UART_BAUD_RATE);

  tmc51x0::ConverterParameters converter_parameters =
    {
      CLOCK_FREQUENCY_MHZ,
      MICROSTEPS_PER_REAL_UNIT
    };

  randomSeed(analogRead(A0));

  // for (size_t i=0; i<MOTOR_COUNT; ++i)
  for (size_t i=1; i<2; ++i)
  {
    TMC51X0 & motor = motors[i];
    tmc51x0::UartParameters uart_parameters(uart,
      NODE_ADDRESS,
      ENABLE_TXRX_PINS[i]);
    motor.setupUart(uart_parameters);

    digitalWrite(MUX_ADDRESS_0_PINS[i], MUX_ADDRESS_0_VALUES[i]);
    digitalWrite(MUX_ADDRESS_1_PINS[i], MUX_ADDRESS_1_VALUES[i]);
    digitalWrite(MUX_ADDRESS_2_PINS[i], MUX_ADDRESS_2_VALUES[i]);

    motor.driver.writeGlobalCurrentScaler(motor.converter.percentToGlobalCurrentScaler(GLOBAL_CURRENT_SCALAR));
    motor.driver.writeRunCurrent(motor.converter.percentToCurrentSetting(RUN_CURRENT));
    motor.driver.writePwmOffset(motor.converter.percentToPwmSetting(PWM_OFFSET));
    motor.driver.writePwmGradient(motor.converter.percentToPwmSetting(PWM_GRADIENT));
    motor.driver.writeMotorDirection(MOTOR_DIRECTION);
    motor.driver.writeStealthChopThreshold(motor.converter.velocityRealToTstep(STEALTH_CHOP_THRESHOLD));
    // motor.driver.writeCoolStepThreshold(motor.converter.velocityRealToTstep(COOL_STEP_THRESHOLD));
    // motor.driver.enableCoolStep(MIN_COOL_STEP, MAX_COOL_STEP);
    // motor.driver.writeHighVelocityThreshold(motor.converter.velocityRealToTstep(HIGH_VELOCITY_THRESHOLD));
    // motor.driver.writeStallGuardThreshold(STALL_GUARD_THRESHOLD);

    motor.controller.writeFirstAcceleration(motor.converter.accelerationRealToChip(FIRST_ACCELERATION));
    motor.controller.writeFirstVelocity(motor.converter.velocityRealToChip(FIRST_VELOCITY));
    motor.controller.writeMaxAcceleration(motor.converter.accelerationRealToChip(MAX_ACCELERATION));
    motor.controller.writeMaxDeceleration(motor.converter.accelerationRealToChip(MAX_DECELERATION));
    motor.controller.writeFirstDeceleration(motor.converter.accelerationRealToChip(FIRST_DECELERATION));
    motor.controller.writeStopVelocity(motor.converter.velocityRealToChip(STOP_VELOCITY));
    motor.controller.writeRampMode(RAMP_MODE);
    motor.controller.writeActualPosition(motor.converter.positionRealToChip(INITIAL_POSITION));

    motor.driver.enable();

    motor.controller.writeStartVelocity(motor.converter.velocityRealToChip(START_VELOCITY));
    motor.controller.writeMaxVelocity(motor.converter.velocityRealToChip(MAX_VELOCITY));

    target_position = random(MIN_TARGET_POSITION, MAX_TARGET_POSITION);
    Serial.print("motor ");
    Serial.print(i);
    Serial.print(" target position: ");
    Serial.println(target_position);
    motor.controller.writeTargetPosition(motor.converter.positionRealToChip(target_position));
  }
}

void loop()
{
  // for (size_t i=0; i<MOTOR_COUNT; ++i)
  for (size_t i=1; i<2; ++i)
  {
    digitalWrite(MUX_ADDRESS_0_PINS[i], MUX_ADDRESS_0_VALUES[i]);
    digitalWrite(MUX_ADDRESS_1_PINS[i], MUX_ADDRESS_1_VALUES[i]);
    digitalWrite(MUX_ADDRESS_2_PINS[i], MUX_ADDRESS_2_VALUES[i]);

    TMC51X0 & motor = motors[i];
    // uint8_t version = motor.readVersion();
    // Serial.print("version: 0x");
    // Serial.println(version, HEX);
    motor.printer.readAndPrintRampStat();
    if (motor.controller.positionReached())
    {
      Serial.print("motor ");
      Serial.print(i);
      Serial.println(":");
      Serial.println("reached target position!");
      target_position = random(MIN_TARGET_POSITION, MAX_TARGET_POSITION);
      Serial.print("new target position: ");
      Serial.println(target_position);
      motor.controller.writeTargetPosition(motor.converter.positionRealToChip(target_position));
      Serial.println("--------------------------");
    }
  }

  Serial.println("--------------------------");
  delay(LOOP_DELAY);
}
