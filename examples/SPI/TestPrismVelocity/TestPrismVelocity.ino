#include <TMC51X0.hpp>


#if defined(ARDUINO_ARCH_RP2040)
SPIClassRP2040 & spi = SPI1;
pin_size_t SCK_PIN = 10;
pin_size_t TX_PIN = 11;
pin_size_t RX_PIN = 12;
#else
SPIClass & spi = SPI;
#endif

// SPI Parameters
const uint32_t SPI_CLOCK_RATE = 1000000;
// const pin_size_t SPI_CHIP_SELECT_PIN = 14;
const pin_size_t SPI_CHIP_SELECT_PIN = 8;

const pin_size_t ENABLE_POWER_PIN = 15;

const uint32_t SERIAL_BAUD_RATE = 115200;
const uint16_t DELAY = 4000;

// converter constants
// external clock is ~12MHz
const uint8_t CLOCK_FREQUENCY_MHZ = 16;
// 200 fullsteps per revolution for many steppers * 256 microsteps per fullstep
// one "real unit" in this example is one rotation of the motor shaft
constexpr uint32_t MICROSTEPS_PER_REAL_UNIT =  200 * 256; // 51200

// driver constants
const uint8_t GLOBAL_CURRENT_SCALAR = 50; // percent
const uint8_t RUN_CURRENT = 100; // percent
const uint8_t PWM_OFFSET = 50; // percent
const uint8_t PWM_GRADIENT = 10; // percent
const tmc51x0::Driver::MotorDirection MOTOR_DIRECTION = tmc51x0::Driver::FORWARD;
const uint8_t STEALTH_CHOP_THRESHOLD = 5; // rotations/s
const uint8_t COOL_STEP_THRESHOLD = 6; // rotations/s
const uint8_t MIN_COOL_STEP = 1;
const uint8_t MAX_COOL_STEP = 0;
const uint8_t HIGH_VELOCITY_THRESHOLD = 9; // rotations/s
// const int8_t STALL_GUARD_THRESHOLD = -20;

// controller constants
const uint32_t TARGET_VELOCITY = 1;  // rotations/s
const uint32_t MAX_ACCELERATION = 2;  // rotations/(s^2)
const tmc51x0::Controller::RampMode RAMP_MODE = tmc51x0::Controller::VELOCITY_POSITIVE;
const int32_t INITIAL_POSITION = 0;

// Instantiate TMC51X0
TMC51X0 tmc5160;
uint32_t target_velocity;

void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);

  pinMode(ENABLE_POWER_PIN, OUTPUT);
  digitalWrite(ENABLE_POWER_PIN, LOW);
  delay(5000);
  digitalWrite(ENABLE_POWER_PIN, HIGH);
  delay(5000);

#if defined(ARDUINO_ARCH_RP2040)
  spi.setSCK(SCK_PIN);
  spi.setTX(TX_PIN);
  spi.setRX(RX_PIN);
#endif
  spi.begin();
  tmc51x0::SpiParameters spi_parameters(spi, SPI_CLOCK_RATE, SPI_CHIP_SELECT_PIN);
  tmc5160.setupSpi(spi_parameters);

  tmc51x0::ConverterParameters converter_parameters =
    {
      CLOCK_FREQUENCY_MHZ,
      MICROSTEPS_PER_REAL_UNIT
    };
  tmc5160.converter.setup(converter_parameters);

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

  target_velocity = TARGET_VELOCITY;

  delay(DELAY);
}

void loop()
{
  // tmc5160.printer.readClearAndPrintGstat();
  // tmc5160.printer.readAndPrintRampStat();
  // tmc5160.printer.readAndPrintDrvStatus();
  // tmc5160.printer.readAndPrintPwmScale();

  // Serial.print("target_velocity (rotations per second): ");
  // Serial.println(target_velocity);
  // uint32_t actual_velocity_chip = tmc5160.controller.readActualVelocity();
  // Serial.print("actual_velocity (chip units): ");
  // Serial.println(actual_velocity_chip);
  // uint32_t actual_velocity_real = tmc5160.converter.velocityChipToReal(actual_velocity_chip);
  // Serial.print("actual_velocity (rotations per second): ");
  // Serial.println(actual_velocity_real);
  // Serial.println("--------------------------");

  // int32_t actual_position_chip = tmc5160.controller.readActualPosition();
  // Serial.print("actual position (chip units): ");
  // Serial.println(actual_position_chip);
  // int32_t actual_position_real = tmc5160.converter.positionChipToReal(actual_position_chip);
  // Serial.print("actual position (rotations): ");
  // Serial.println(actual_position_real);
  // Serial.println("--------------------------");

  Serial.println("--------------------------");
  tmc5160.controller.writeMaxVelocity(tmc5160.converter.velocityRealToChip(target_velocity));

  delay(DELAY);
}
