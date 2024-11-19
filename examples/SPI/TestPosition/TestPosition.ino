#include <TMC51X0.hpp>


#if defined(ARDUINO_ARCH_RP2040)
SPIClassRP2040 & spi = SPI;
uint16_t SCK_PIN = 18;
uint16_t TX_PIN = 19;
uint16_t RX_PIN = 20;
#else
SPIClass & spi = SPI;
#endif

const tmc51x0::SpiParameters spi_parameters =
{
  spi,
  1000000, // clock_rate
  10 // chip_select_pin
};

const tmc51x0::ConverterParameters converter_parameters =
{
  12, // clock_frequency_mhz
  4881 // microsteps_per_real_unit
};
// internal clock is ~12MHz
// 200 fullsteps per revolution for many steppers * 256 microsteps per fullstep
// 10.49 millimeters per revolution leadscrew -> 51200 / 10.49 ~= 4881
// one "real unit" in this example is one millimeters of linear travel

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

const uint16_t ENABLE_HARDWARE_PIN = 4;

const uint32_t SERIAL_BAUD_RATE = 115200;
const uint16_t LOOP_DELAY = 500;

// Instantiate TMC51X0
TMC51X0 tmc5160;
uint32_t target_position;

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

  tmc5160.converter.setup(converter_parameters);

  tmc5160.driver.setEnableHardwarePin(ENABLE_HARDWARE_PIN);
  tmc5160.driver.writeGlobalCurrentScaler(tmc5160.converter.percentToGlobalCurrentScaler(GLOBAL_CURRENT_SCALAR));
  tmc5160.driver.writeRunCurrent(tmc5160.converter.percentToCurrentSetting(RUN_CURRENT));
  tmc5160.driver.writePwmOffset(tmc5160.converter.percentToPwmSetting(PWM_OFFSET));
  tmc5160.driver.writePwmGradient(tmc5160.converter.percentToPwmSetting(PWM_GRADIENT));
  tmc5160.driver.writeMotorDirection(MOTOR_DIRECTION);
  tmc5160.driver.writeStealthChopThreshold(tmc5160.converter.velocityRealToTstep(STEALTH_CHOP_THRESHOLD));
  // tmc5160.driver.writeCoolStepThreshold(tmc5160.converter.velocityRealToTstep(COOL_STEP_THRESHOLD));
  // tmc5160.driver.enableCoolStep(MIN_COOL_STEP, MAX_COOL_STEP);
  // tmc5160.driver.writeHighVelocityThreshold(tmc5160.converter.velocityRealToTstep(HIGH_VELOCITY_THRESHOLD));
  // tmc5160.driver.writeStallGuardThreshold(STALL_GUARD_THRESHOLD);

  tmc5160.controller.writeFirstAcceleration(tmc5160.converter.accelerationRealToChip(FIRST_ACCELERATION));
  tmc5160.controller.writeFirstVelocity(tmc5160.converter.velocityRealToChip(FIRST_VELOCITY));
  tmc5160.controller.writeMaxAcceleration(tmc5160.converter.accelerationRealToChip(MAX_ACCELERATION));
  tmc5160.controller.writeMaxDeceleration(tmc5160.converter.accelerationRealToChip(MAX_DECELERATION));
  tmc5160.controller.writeFirstDeceleration(tmc5160.converter.accelerationRealToChip(FIRST_DECELERATION));
  tmc5160.controller.writeStopVelocity(tmc5160.converter.velocityRealToChip(STOP_VELOCITY));
  tmc5160.controller.writeRampMode(RAMP_MODE);
  tmc5160.controller.writeActualPosition(tmc5160.converter.positionRealToChip(INITIAL_POSITION));

  tmc5160.driver.enable();

  tmc5160.controller.rampToZeroVelocity();
  while (!tmc5160.controller.zeroVelocity())
  {
    Serial.println("Waiting for zero velocity.");
    delay(LOOP_DELAY);
  }

  tmc5160.controller.writeStartVelocity(tmc5160.converter.velocityRealToChip(START_VELOCITY));
  tmc5160.controller.writeMaxVelocity(tmc5160.converter.velocityRealToChip(MAX_VELOCITY));

  randomSeed(analogRead(A0));
  long random_delay = random(5000);
  delay(random_delay);

  target_position = MIN_TARGET_POSITION;
  tmc5160.controller.writeTargetPosition(tmc5160.converter.positionRealToChip(target_position));

}

void loop()
{
  tmc5160.printer.readAndPrintRampStat();
  tmc5160.printer.readAndPrintDrvStatus();

  Serial.print("max_velocity (millimeters per second): ");
  Serial.println(MAX_VELOCITY);

  int32_t actual_velocity_chip = tmc5160.controller.readActualVelocity();
  int32_t actual_velocity_real = tmc5160.converter.velocityChipToReal(actual_velocity_chip);
  Serial.print("actual_velocity (millimeters per second): ");
  Serial.println(actual_velocity_real);

  int32_t actual_position_chip = tmc5160.controller.readActualPosition();
  int32_t actual_position_real = tmc5160.converter.positionChipToReal(actual_position_chip);
  Serial.print("actual position (millimeters): ");
  Serial.println(actual_position_real);

  int32_t target_position_chip = tmc5160.controller.readTargetPosition();
  int32_t target_position_real = tmc5160.converter.positionChipToReal(target_position_chip);
  Serial.print("target position (millimeters): ");
  Serial.println(target_position_real);
  Serial.println("--------------------------");

  if (tmc5160.controller.positionReached())
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
tmc5160.controller.writeTargetPosition(tmc5160.converter.positionRealToChip(target_position));
  }

  Serial.println("--------------------------");
  delay(LOOP_DELAY);
}
