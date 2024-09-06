#include <TMC51X0.hpp>


#if !defined(ARDUINO_ARCH_RP2040)
HardwareSerial & uart = Serial2;
#else
SerialUART & uart = Serial2;
pin_size_t TX_PIN = 4;
pin_size_t RX_PIN = 5;
#endif

const uint8_t ENABLE_VCC_PIN = 22;
const uint8_t ENABLE_VCC_POLARITY = HIGH;

// ENABLE_TX_PIN and ENABLE_RX_PIN may be the same pin
const uint32_t UART_BAUD_RATE = 115200;
const uint8_t NODE_ADDRESS = 1;
const uint8_t ENABLE_TX_POLARITY = HIGH;
const uint8_t ENABLE_RX_POLARITY = LOW;

const uint8_t PRISM_COUNT = 7;
const uint8_t ENABLE_TX_PINS[PRISM_COUNT] = {15, 13, 11, 9, 7, 3, 1};
const uint8_t ENABLE_RX_PINS[PRISM_COUNT] = {14, 12, 10, 8, 6, 2, 0};

const uint32_t SERIAL_BAUD_RATE = 115200;
const int LOOP_DELAY = 2000;
const int RESET_DELAY = 10000;

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
TMC51X0 prisms[PRISM_COUNT];
uint32_t target_position;

void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);

  delay(RESET_DELAY);

  Serial.println("Disabling VCC");

  pinMode(ENABLE_VCC_PIN, OUTPUT);
  digitalWrite(ENABLE_VCC_PIN, !ENABLE_VCC_POLARITY);

  delay(RESET_DELAY);

  Serial.println("Enabling VCC");

  pinMode(ENABLE_VCC_PIN, OUTPUT);
  digitalWrite(ENABLE_VCC_PIN, ENABLE_VCC_POLARITY);

  delay(RESET_DELAY);

#if defined(ARDUINO_ARCH_RP2040)
  uart.setTX(TX_PIN);
  uart.setRX(RX_PIN);
#endif
  uart.begin(UART_BAUD_RATE);

  tmc51x0::Converter::Settings converter_settings =
    {
      CLOCK_FREQUENCY_MHZ,
      MICROSTEPS_PER_REAL_UNIT
    };

  randomSeed(analogRead(A0));

  // for (size_t i=0; i<PRISM_COUNT; ++i)
  for (size_t i=1; i<2; ++i)
  {
    TMC51X0 & prism = prisms[i];
    tmc51x0::UartParameters uart_parameters(uart,
      NODE_ADDRESS,
      ENABLE_TX_PINS[i],
      ENABLE_RX_PINS[i],
      ENABLE_TX_POLARITY,
      ENABLE_RX_POLARITY);
    prism.setupUart(uart_parameters);
    prism.driver.writeGlobalCurrentScaler(prism.converter.percentToGlobalCurrentScaler(GLOBAL_CURRENT_SCALAR));
    prism.driver.writeRunCurrent(prism.converter.percentToCurrentSetting(RUN_CURRENT));
    prism.driver.writePwmOffset(prism.converter.percentToPwmSetting(PWM_OFFSET));
    prism.driver.writePwmGradient(prism.converter.percentToPwmSetting(PWM_GRADIENT));
    prism.driver.writeMotorDirection(MOTOR_DIRECTION);
    prism.driver.writeStealthChopThreshold(prism.converter.velocityRealToTstep(STEALTH_CHOP_THRESHOLD));
    // prism.driver.writeCoolStepThreshold(prism.converter.velocityRealToTstep(COOL_STEP_THRESHOLD));
    // prism.driver.enableCoolStep(MIN_COOL_STEP, MAX_COOL_STEP);
    // prism.driver.writeHighVelocityThreshold(prism.converter.velocityRealToTstep(HIGH_VELOCITY_THRESHOLD));
    // prism.driver.writeStallGuardThreshold(STALL_GUARD_THRESHOLD);

    prism.controller.writeFirstAcceleration(prism.converter.accelerationRealToChip(FIRST_ACCELERATION));
    prism.controller.writeFirstVelocity(prism.converter.velocityRealToChip(FIRST_VELOCITY));
    prism.controller.writeMaxAcceleration(prism.converter.accelerationRealToChip(MAX_ACCELERATION));
    prism.controller.writeMaxDeceleration(prism.converter.accelerationRealToChip(MAX_DECELERATION));
    prism.controller.writeFirstDeceleration(prism.converter.accelerationRealToChip(FIRST_DECELERATION));
    prism.controller.writeStopVelocity(prism.converter.velocityRealToChip(STOP_VELOCITY));
    prism.controller.writeRampMode(RAMP_MODE);
    prism.controller.writeActualPosition(prism.converter.positionRealToChip(INITIAL_POSITION));

    prism.driver.enable();

    prism.controller.writeStartVelocity(prism.converter.velocityRealToChip(START_VELOCITY));
    prism.controller.writeMaxVelocity(prism.converter.velocityRealToChip(MAX_VELOCITY));

    target_position = random(MIN_TARGET_POSITION, MAX_TARGET_POSITION);
    Serial.print("prism ");
    Serial.print(i);
    Serial.print(" target position: ");
    Serial.println(target_position);
    prism.controller.writeTargetPosition(prism.converter.positionRealToChip(target_position));
  }
}

void loop()
{
  // for (size_t i=0; i<PRISM_COUNT; ++i)
  for (size_t i=1; i<2; ++i)
  {
    TMC51X0 & prism = prisms[i];
    // uint8_t version = prism.readVersion();
    // Serial.print("version: 0x");
    // Serial.println(version, HEX);
    prism.printer.readAndPrintRampStat();
    if (prism.controller.positionReached())
    {
      Serial.print("prism ");
      Serial.print(i);
      Serial.println(":");
      Serial.println("reached target position!");
      target_position = random(MIN_TARGET_POSITION, MAX_TARGET_POSITION);
      Serial.print("new target position: ");
      Serial.println(target_position);
      prism.controller.writeTargetPosition(prism.converter.positionRealToChip(target_position));
      Serial.println("--------------------------");
    }
  }

  Serial.println("--------------------------");
  delay(LOOP_DELAY);
}
