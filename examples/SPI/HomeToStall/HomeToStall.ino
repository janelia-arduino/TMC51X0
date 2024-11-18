#include <TMC51X0.hpp>


#if defined(ARDUINO_ARCH_RP2040)
SPIClassRP2040 & spi = SPI;
uint16_t SCK_PIN = 18;
uint16_t TX_PIN = 19;
uint16_t RX_PIN = 20;
#else
SPIClass & spi = SPI;
#endif

// SPI Parameters
const tmc51x0::SpiParameters spi_parameters(
  spi,
  1000000, // clock_rate
  10); // chip_select_pin

const uint16_t ENABLE_HARDWARE_PIN = 4;

const uint32_t SERIAL_BAUD_RATE = 115200;
const uint16_t LOOP_DELAY = 500;
const uint16_t STALL_DELAY = 4000;

// converter constants
// internal clock is ~12MHz
const uint8_t CLOCK_FREQUENCY_MHZ = 12;
// 200 fullsteps per revolution for many steppers * 256 microsteps per fullstep
// 10.49 millimeters per revolution leadscrew -> 51200 / 10.49
// one "real unit" in this example is one millimeters of linear travel
constexpr uint32_t MICROSTEPS_PER_REAL_UNIT = 4881;

// driver constants
const uint8_t GLOBAL_CURRENT_SCALAR = 50; // percent
const uint8_t RUN_CURRENT = 20; // percent
const uint8_t PWM_OFFSET = 20; // percent
const uint8_t PWM_GRADIENT = 5; // percent
const tmc51x0::Driver::MotorDirection MOTOR_DIRECTION = tmc51x0::Driver::REVERSE;

const uint8_t STEALTH_CHOP_THRESHOLD = 100; // millimeters/s
const uint8_t COOL_STEP_THRESHOLD = 100; // millimeters/s
const uint8_t MIN_COOL_STEP = 1;
const uint8_t MAX_COOL_STEP = 0;
const uint8_t HIGH_VELOCITY_THRESHOLD = 90; // millimeters/s
const int8_t STALL_GUARD_THRESHOLD = 1;

// controller constants
const uint32_t START_VELOCITY = 1; // millimeters/s
const uint32_t FIRST_ACCELERATION = 10;  // millimeters/(s^2)
const uint32_t FIRST_VELOCITY = 10; // millimeters/s
const uint32_t MAX_ACCELERATION = 2;  // millimeters/(s^2)
const uint32_t MAX_DECELERATION = 25;  // millimeters/(s^2)
const uint32_t FIRST_DECELERATION = 20;  // millimeters/(s^2)
const uint32_t MAX_VELOCITY = 20; // millimeters/s
const uint32_t STOP_VELOCITY = 5; // millimeters/s

const int32_t MIN_TARGET_POSITION = 20;  // millimeters
const int32_t MAX_TARGET_POSITION = 180;  // millimeters
const tmc51x0::Controller::RampMode RAMP_MODE = tmc51x0::Controller::POSITION;

// home constants
const uint8_t HOME_GLOBAL_CURRENT_SCALAR = 50; // percent
const uint8_t HOME_COOL_STEP_THRESHOLD = 5; // millimeters/s
const uint32_t HOME_START_VELOCITY = 1; // millimeters/s
const uint32_t HOME_MAX_VELOCITY = 10; // millimeters/s
const int32_t HOME_TARGET_POSITION = -200;  // millimeters

// Instantiate TMC51X0
TMC51X0 tmc5160;

void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);

#if defined(ARDUINO_ARCH_RP2040)
  spi.setSCK(SCK_PIN);
  spi.setTX(TX_PIN);
  spi.setRX(RX_PIN);
#endif
  spi.begin();
  tmc5160.setupSpi(spi_parameters);

  tmc51x0::ConverterParameters converter_parameters =
    {
      CLOCK_FREQUENCY_MHZ,
      MICROSTEPS_PER_REAL_UNIT
    };
  tmc5160.converter.setup(converter_parameters);

  tmc5160.driver.setEnableHardwarePin(ENABLE_HARDWARE_PIN);
  tmc5160.driver.writeGlobalCurrentScaler(tmc5160.converter.percentToGlobalCurrentScaler(GLOBAL_CURRENT_SCALAR));
  tmc5160.driver.writeRunCurrent(tmc5160.converter.percentToCurrentSetting(RUN_CURRENT));
  tmc5160.driver.writePwmOffset(tmc5160.converter.percentToPwmSetting(PWM_OFFSET));
  tmc5160.driver.writePwmGradient(tmc5160.converter.percentToPwmSetting(PWM_GRADIENT));
  tmc5160.driver.writeMotorDirection(MOTOR_DIRECTION);
  tmc5160.driver.writeStealthChopThreshold(tmc5160.converter.velocityRealToTstep(STEALTH_CHOP_THRESHOLD));
  tmc5160.driver.writeCoolStepThreshold(tmc5160.converter.velocityRealToTstep(COOL_STEP_THRESHOLD));
  tmc5160.driver.enableCoolStep(MIN_COOL_STEP, MAX_COOL_STEP);
  tmc5160.driver.writeStallGuardThreshold(STALL_GUARD_THRESHOLD);
  tmc5160.driver.disableStallGuardFilter();

  tmc5160.controller.writeFirstAcceleration(tmc5160.converter.accelerationRealToChip(FIRST_ACCELERATION));
  tmc5160.controller.writeFirstVelocity(tmc5160.converter.velocityRealToChip(FIRST_VELOCITY));
  tmc5160.controller.writeMaxAcceleration(tmc5160.converter.accelerationRealToChip(MAX_ACCELERATION));
  tmc5160.controller.writeMaxDeceleration(tmc5160.converter.accelerationRealToChip(MAX_DECELERATION));
  tmc5160.controller.writeFirstDeceleration(tmc5160.converter.accelerationRealToChip(FIRST_DECELERATION));
  tmc5160.controller.writeStopVelocity(tmc5160.converter.velocityRealToChip(STOP_VELOCITY));
  tmc5160.controller.writeRampMode(RAMP_MODE);

  tmc5160.driver.enable();

  tmc5160.controller.rampToZeroVelocity();
  while (!tmc5160.controller.zeroVelocity())
  {
    Serial.println("Waiting for zero velocity.");
    delay(LOOP_DELAY);
  }
}

void loop()
{
  Serial.println("Waiting to start homing.");
  delay(4000);

  // homing
  Serial.println("Setting homing constants.");
  tmc5160.driver.writeGlobalCurrentScaler(tmc5160.converter.percentToGlobalCurrentScaler(HOME_GLOBAL_CURRENT_SCALAR));
  tmc5160.driver.writeCoolStepThreshold(tmc5160.converter.velocityRealToTstep(HOME_COOL_STEP_THRESHOLD));
  tmc5160.controller.writeStartVelocity(tmc5160.converter.velocityRealToChip(HOME_START_VELOCITY));
  tmc5160.controller.writeMaxVelocity(tmc5160.converter.velocityRealToChip(HOME_MAX_VELOCITY));
  tmc5160.controller.writeTargetPosition(tmc5160.converter.positionRealToChip(HOME_TARGET_POSITION));
  Serial.println("Homing.");
  while (!tmc5160.controller.velocityReached())
  {
    Serial.println("Waiting to reach home velocity.");
    delay(LOOP_DELAY);
  }
  Serial.println("Enabling stall stop.");
  tmc5160.controller.enableStallStop();
  uint16_t stall_guard_result;
  while (!tmc5160.driver.stalled() && !tmc5160.controller.zeroVelocity() && !tmc5160.controller.positionReached())
  {
    Serial.println("Waiting to stall, reach zero velocity, or reach home position.");
    stall_guard_result = tmc5160.driver.readStallGuardResult();
    Serial.print("stall guard result: ");
    Serial.println(stall_guard_result);
    delay(LOOP_DELAY);
  }
  if (!tmc5160.controller.positionReached())
  {
    Serial.println("Homed successfully!");
  }
  else
  {
    Serial.println("Home Failed!! Try adjusting stallguard threshold or global current scalar.");
  }
  Serial.println("Setting zero velocity.");
  tmc5160.controller.writeStartVelocity(0);
  tmc5160.controller.writeMaxVelocity(0);
  Serial.println("Disabling stall stop.");
  tmc5160.controller.disableStallStop();
  Serial.println("Setting actual and target positions to zero.");
  tmc5160.controller.writeActualPosition(0);
  tmc5160.controller.writeTargetPosition(0);
  delay(4000);

  // move to posititions
  tmc5160.driver.writeGlobalCurrentScaler(tmc5160.converter.percentToGlobalCurrentScaler(GLOBAL_CURRENT_SCALAR));
  tmc5160.driver.writeCoolStepThreshold(tmc5160.converter.velocityRealToTstep(COOL_STEP_THRESHOLD));
  tmc5160.controller.writeStartVelocity(tmc5160.converter.velocityRealToChip(START_VELOCITY));
  tmc5160.controller.writeMaxVelocity(tmc5160.converter.velocityRealToChip(MAX_VELOCITY));

  Serial.println("Moving to min target position.");
  tmc5160.controller.writeTargetPosition(tmc5160.converter.positionRealToChip(MIN_TARGET_POSITION));

  Serial.println("Waiting to reach min target position.");
  while (!tmc5160.controller.positionReached())
  {
    delay(LOOP_DELAY);
  }
  Serial.println("Reached min target position.");

  Serial.println("Moving to max target position.");
  tmc5160.controller.writeTargetPosition(tmc5160.converter.positionRealToChip(MAX_TARGET_POSITION));

  Serial.println("Waiting to reach max target position.");
  while (!tmc5160.controller.positionReached())
  {
    delay(LOOP_DELAY);
  }
  Serial.println("Reached max target position.");

  Serial.println("--------------------------");
  delay(LOOP_DELAY);
}
