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
const uint8_t HARDWARE_ENABLE_PIN = 4;

const long SERIAL_BAUD_RATE = 115200;
const int DELAY = 500;

// converter constants
// internal clock is ~12MHz
const uint8_t CLOCK_FREQUENCY_MHZ = 12;
// 200 fullsteps per revolution for many steppers * 256 microsteps per fullstep / 360 degrees per revolution
// one "real unit" in this example is one degree rotation of the motor shaft
constexpr uint32_t MICROSTEPS_PER_REAL_UNIT = 142;

// driver constants
const uint8_t GLOBAL_CURRENT_SCALAR = 20; // percent
const uint8_t RUN_CURRENT = 20; // percent
const uint8_t PWM_OFFSET = 10; // percent
const uint8_t PWM_GRADIENT = 10; // percent
const tmc51x0::Driver::MotorDirection MOTOR_DIRECTION = tmc51x0::Driver::FORWARD;
const uint8_t STEALTH_CHOP_THRESHOLD = 360; // degrees/s
const uint8_t COOL_STEP_THRESHOLD = 360; // degrees/s
const uint8_t MIN_COOL_STEP = 15;
const uint8_t MAX_COOL_STEP = 0;
const int8_t STALL_GUARD_THRESHOLD = 1;

// controller constants
const uint32_t MAX_VELOCITY = 400; // degrees/s
const uint32_t MAX_ACCELERATION = 360;  // degrees/(s^2)
const tmc51x0::Controller::RampMode RAMP_MODE = tmc51x0::Controller::VELOCITY_POSITIVE;
const int32_t INITIAL_POSITION = 0;

// Instantiate TMC51X0
TMC51X0 stepper;
uint32_t target_velocity;

void printRegisterPortion(const char * str, uint32_t value, bool hex=false)
{
  Serial.print(str);
  if (not hex)
  {
    Serial.print(": 0b");
    Serial.print(value, BIN);
  }
  else
  {
    Serial.print(": 0x");
    Serial.print(value, HEX);
  }
  Serial.println();
}

void printRegisterRampStat(uint32_t register_data)
{
  tmc51x0::Registers::RampStat ramp_stat;
  ramp_stat.bytes = register_data;
  printRegisterPortion("ramp_stat", ramp_stat.bytes, true);
  printRegisterPortion("status_stop_l", ramp_stat.status_stop_l);
  printRegisterPortion("status_stop_r", ramp_stat.status_stop_r);
  printRegisterPortion("velocity_reached", ramp_stat.velocity_reached);
  printRegisterPortion("vzero", ramp_stat.vzero);
  printRegisterPortion("status_sg", ramp_stat.status_sg);
  Serial.println("--------------------------");
}

void printRegisterDrvStatus(uint32_t register_data)
{
  tmc51x0::Registers::DrvStatus drv_status;
  drv_status.bytes = register_data;
  printRegisterPortion("drv_status", drv_status.bytes, true);
  printRegisterPortion("sg_result", drv_status.sg_result, true);
  printRegisterPortion("stealth", drv_status.stealth);
  printRegisterPortion("cs_actual", drv_status.cs_actual, true);
  printRegisterPortion("stst", drv_status.stst);
  Serial.println("--------------------------");
}

void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);
#if defined(ARDUINO_ARCH_RP2040)
  spi.setSCK(SCK_PIN);
  spi.setTX(TX_PIN);
  spi.setRX(RX_PIN);
#endif
  stepper.setup(spi, CHIP_SELECT_PIN);
  stepper.driver.setHardwareEnablePin(HARDWARE_ENABLE_PIN);

  stepper.converter.setClockFrequencyMHz(CLOCK_FREQUENCY_MHZ);
  stepper.converter.setMicrostepsPerRealUnit(MICROSTEPS_PER_REAL_UNIT);

  stepper.driver.writeGlobalCurrentScaler(stepper.converter.percentToGlobalCurrentScaler(GLOBAL_CURRENT_SCALAR));
  stepper.driver.writeRunCurrent(stepper.converter.percentToCurrentSetting(RUN_CURRENT));
  stepper.driver.writePwmOffset(stepper.converter.percentToPwmSetting(PWM_OFFSET));
  stepper.driver.writePwmGradient(stepper.converter.percentToPwmSetting(PWM_GRADIENT));
  stepper.driver.writeMotorDirection(MOTOR_DIRECTION);

  stepper.driver.writeStealthChopThreshold(stepper.converter.velocityRealToTstep(STEALTH_CHOP_THRESHOLD));

  stepper.driver.writeCoolStepThreshold(stepper.converter.velocityRealToTstep(COOL_STEP_THRESHOLD));
  stepper.driver.enableCoolStep(MIN_COOL_STEP, MAX_COOL_STEP);
  stepper.driver.writeStallGuardThreshold(STALL_GUARD_THRESHOLD);
  stepper.driver.enableStallGuardFilter();

  stepper.controller.writeMaxAcceleration(stepper.converter.accelerationRealToChip(MAX_ACCELERATION));
  stepper.controller.writeRampMode(RAMP_MODE);
  stepper.controller.writeActualPosition(stepper.converter.positionRealToChip(INITIAL_POSITION));

  stepper.driver.enable();

  stepper.controller.rampToZeroVelocity();
  while (!stepper.controller.zeroVelocity())
  {
    Serial.println("Waiting for zero velocity.");
    delay(DELAY);
  }
  stepper.controller.writeMaxVelocity(stepper.converter.velocityRealToChip(MAX_VELOCITY));

  delay(DELAY);
}

void loop()
{
  printRegisterRampStat(stepper.registers.read(tmc51x0::Registers::RAMP_STAT));
  printRegisterDrvStatus(stepper.registers.read(tmc51x0::Registers::DRV_STATUS));

  Serial.print("acceleration (degrees per second per second): ");
  Serial.println(MAX_ACCELERATION);
  Serial.print("acceleration (chip units): ");
  Serial.println(stepper.converter.accelerationRealToChip(MAX_ACCELERATION));
  Serial.println("--------------------------");

  Serial.print("target_velocity (degrees per second): ");
  Serial.println(target_velocity);
  uint32_t actual_velocity_chip = stepper.controller.readActualVelocity();
  Serial.print("actual_velocity (chip units): ");
  Serial.println(actual_velocity_chip);
  uint32_t actual_velocity_real = stepper.converter.velocityChipToReal(actual_velocity_chip);
  Serial.print("actual_velocity (degrees per second): ");
  Serial.println(actual_velocity_real);
  Serial.println("--------------------------");

  Serial.println("--------------------------");

  delay(DELAY);
}
