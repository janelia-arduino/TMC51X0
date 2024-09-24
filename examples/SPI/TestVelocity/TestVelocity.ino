#include <TMC51X0.hpp>


#if defined(ARDUINO_ARCH_RP2040)
SPIClassRP2040 & spi = SPI;
pin_size_t SCK_PIN = 18;
pin_size_t TX_PIN = 19;
pin_size_t RX_PIN = 20;
#else
SPIClass & spi = SPI;
#endif

const uint8_t CHIP_SELECT_PIN = 10;
const uint8_t ENABLE_HARDWARE_PIN = 4;

const long SERIAL_BAUD_RATE = 115200;
const uint16_t DELAY = 4000;

// converter constants
// internal clock is ~12MHz
const uint8_t CLOCK_FREQUENCY_MHZ = 12;
// 200 fullsteps per revolution for many steppers * 256 microsteps per fullstep
// one "real unit" in this example is one rotation of the motor shaft
constexpr uint32_t MICROSTEPS_PER_REAL_UNIT =  200 * 256; // 51200

// driver constants
const uint8_t GLOBAL_CURRENT_SCALAR = 50; // percent
const uint8_t RUN_CURRENT = 100; // percent
const uint8_t PWM_OFFSET = 20; // percent
const uint8_t PWM_GRADIENT = 5; // percent
const tmc51x0::Driver::MotorDirection MOTOR_DIRECTION = tmc51x0::Driver::FORWARD;
const uint8_t STEALTH_CHOP_THRESHOLD = 5; // rotations/s
const uint8_t COOL_STEP_THRESHOLD = 6; // rotations/s
const uint8_t MIN_COOL_STEP = 1;
const uint8_t MAX_COOL_STEP = 0;
const uint8_t HIGH_VELOCITY_THRESHOLD = 9; // rotations/s
// const int8_t STALL_GUARD_THRESHOLD = -20;

// controller constants
const uint32_t MIN_TARGET_VELOCITY = 1;  // rotations/s
const uint32_t MAX_TARGET_VELOCITY = 25; // rotations/s
const uint32_t TARGET_VELOCITY_INC = 1;  // rotations/s
const uint32_t MAX_ACCELERATION = 2;  // rotations/(s^2)
const tmc51x0::Controller::RampMode RAMP_MODE = tmc51x0::Controller::VELOCITY_POSITIVE;
const int32_t INITIAL_POSITION = 0;

// Instantiate TMC51X0
TMC51X0 tmc5160;
uint32_t target_velocity;

void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);

#if defined(ARDUINO_ARCH_RP2040)
  spi.setSCK(SCK_PIN);
  spi.setTX(TX_PIN);
  spi.setRX(RX_PIN);
#endif
  spi.begin();
  tmc51x0::SpiParameters spi_parameters(spi, CHIP_SELECT_PIN);
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

  // tmc5160.driver.writeCoolStepThreshold(tmc5160.converter.velocityRealToTstep(COOL_STEP_THRESHOLD));
  // tmc5160.driver.enableCoolStep(MIN_COOL_STEP, MAX_COOL_STEP);

  // tmc5160.driver.writeHighVelocityThreshold(tmc5160.converter.velocityRealToTstep(HIGH_VELOCITY_THRESHOLD));
  // tmc5160.driver.writeStallGuardThreshold(STALL_GUARD_THRESHOLD);

  tmc5160.controller.writeMaxAcceleration(tmc5160.converter.accelerationRealToChip(MAX_ACCELERATION));
  tmc5160.controller.writeRampMode(RAMP_MODE);
  tmc5160.controller.writeActualPosition(tmc5160.converter.positionRealToChip(INITIAL_POSITION));

  tmc5160.driver.enable();

  tmc5160.controller.rampToZeroVelocity();
  while (!tmc5160.controller.zeroVelocity())
  {
    Serial.println("Waiting for zero velocity.");
    delay(DELAY);
  }

  target_velocity = MIN_TARGET_VELOCITY;
  tmc5160.controller.writeMaxVelocity(tmc5160.converter.velocityRealToChip(target_velocity));

  delay(DELAY);
}

void loop()
{
  tmc5160.printer.readClearAndPrintGstat();
  tmc5160.printer.readAndPrintRampStat();
  tmc5160.printer.readAndPrintDrvStatus();
  tmc5160.printer.readAndPrintPwmScale();

  // Serial.print("acceleration (rotations per second per second): ");
  // Serial.println(MAX_ACCELERATION);
  // Serial.print("acceleration (chip units): ");
  // Serial.println(tmc5160.converter.accelerationRealToChip(MAX_ACCELERATION));
  // Serial.println("--------------------------");

  Serial.print("target_velocity (rotations per second): ");
  Serial.println(target_velocity);
  uint32_t actual_velocity_chip = tmc5160.controller.readActualVelocity();
  // Serial.print("actual_velocity (chip units): ");
  // Serial.println(actual_velocity_chip);
  uint32_t actual_velocity_real = tmc5160.converter.velocityChipToReal(actual_velocity_chip);
  Serial.print("actual_velocity (rotations per second): ");
  Serial.println(actual_velocity_real);
  // uint32_t tstep = tmc5160.controller.readTstep();
  // Serial.print("tstep (chip units): ");
  // Serial.println(tstep);
  // uint32_t velocity_real = tmc5160.converter.tstepToVelocityReal(tstep);
  // Serial.print("tstepToVelocityReal (rotations per second): ");
  // Serial.println(velocity_real);
  // tstep = tmc5160.converter.velocityRealToTstep(velocity_real);
  // Serial.print("velocityRealToTstep (chip_units): ");
  // Serial.println(tstep);
  // Serial.print("STEALTH_CHOP_THRESHOLD (rotations per second): ");
  // Serial.println(STEALTH_CHOP_THRESHOLD);
  // Serial.print("STEALTH_CHOP_THRESHOLD (chip units): ");
  // Serial.println(tmc5160.converter.velocityRealToTstep(STEALTH_CHOP_THRESHOLD));
  Serial.println("--------------------------");

  // int32_t actual_position_chip = tmc5160.controller.readActualPosition();
  // Serial.print("actual position (chip units): ");
  // Serial.println(actual_position_chip);
  // int32_t actual_position_real = tmc5160.converter.positionChipToReal(actual_position_chip);
  // Serial.print("actual position (rotations): ");
  // Serial.println(actual_position_real);
  // Serial.println("--------------------------");

  Serial.println("--------------------------");

  delay(DELAY);

  target_velocity += TARGET_VELOCITY_INC;
  if (target_velocity > MAX_TARGET_VELOCITY)
  {
    target_velocity = MIN_TARGET_VELOCITY;
  }
  tmc5160.controller.writeMaxVelocity(tmc5160.converter.velocityRealToChip(target_velocity));

  delay(DELAY);
}
