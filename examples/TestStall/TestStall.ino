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
const int LOOP_DELAY = 500;
const int STALL_DELAY = 4000;

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
const uint16_t STEALTH_CHOP_THRESHOLD = 360; // degrees/s
const uint16_t COOL_STEP_THRESHOLD = 360; // degrees/s
const uint8_t MIN_COOL_STEP = 15;
const uint8_t MAX_COOL_STEP = 0;
const int16_t STALL_GUARD_THRESHOLD = 1;

// controller constants
const uint32_t MAX_VELOCITY = 600; // degrees/s
const uint32_t MAX_ACCELERATION = 360;  // degrees/(s^2)
const tmc51x0::Controller::RampMode RAMP_MODE = tmc51x0::Controller::VELOCITY_POSITIVE;
const int32_t INITIAL_POSITION = 0;

// Instantiate TMC51X0
TMC51X0 stepper_interface;
bool stall_stop_enabled;

void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);
#if defined(ARDUINO_ARCH_RP2040)
  spi.setSCK(SCK_PIN);
  spi.setTX(TX_PIN);
  spi.setRX(RX_PIN);
#endif

  stepper_interface.setup(spi, CHIP_SELECT_PIN);
  stall_stop_enabled = false;

  tmc51x0::Converter::Settings converter_settings =
    {
      CLOCK_FREQUENCY_MHZ,
      MICROSTEPS_PER_REAL_UNIT
    };
  stepper_interface.converter.setup(converter_settings);

  stepper_interface.driver.setHardwareEnablePin(HARDWARE_ENABLE_PIN);
stepper_interface.driver.writeGlobalCurrentScaler(stepper_interface.converter.percentToGlobalCurrentScaler(GLOBAL_CURRENT_SCALAR));
  stepper_interface.driver.writeRunCurrent(stepper_interface.converter.percentToCurrentSetting(RUN_CURRENT));
  stepper_interface.driver.writePwmOffset(stepper_interface.converter.percentToPwmSetting(PWM_OFFSET));
  stepper_interface.driver.writePwmGradient(stepper_interface.converter.percentToPwmSetting(PWM_GRADIENT));
  stepper_interface.driver.writeMotorDirection(MOTOR_DIRECTION);

  stepper_interface.driver.writeStealthChopThreshold(stepper_interface.converter.velocityRealToTstep(STEALTH_CHOP_THRESHOLD));

  stepper_interface.driver.writeCoolStepThreshold(stepper_interface.converter.velocityRealToTstep(COOL_STEP_THRESHOLD));
  stepper_interface.driver.enableCoolStep(MIN_COOL_STEP, MAX_COOL_STEP);
  stepper_interface.driver.writeStallGuardThreshold(STALL_GUARD_THRESHOLD);
  stepper_interface.driver.disableStallGuardFilter();

  stepper_interface.controller.writeMaxAcceleration(stepper_interface.converter.accelerationRealToChip(MAX_ACCELERATION));
  stepper_interface.controller.writeRampMode(RAMP_MODE);
  stepper_interface.controller.writeActualPosition(stepper_interface.converter.positionRealToChip(INITIAL_POSITION));

  stepper_interface.driver.enable();

  stepper_interface.controller.rampToZeroVelocity();
  while (!stepper_interface.controller.zeroVelocity())
  {
    Serial.println("Waiting for zero velocity.");
    delay(LOOP_DELAY);
  }
  stepper_interface.controller.writeMaxVelocity(stepper_interface.converter.velocityRealToChip(MAX_VELOCITY));
}

void loop()
{
  tmc51x0::Registers::RampStat ramp_stat;
  ramp_stat.bytes = stepper_interface.registers.read(tmc51x0::Registers::RAMP_STAT);
  stepper_interface.printer.printRampStat(ramp_stat);
  tmc51x0::Registers::DrvStatus drv_status;
  drv_status.bytes = stepper_interface.registers.read(tmc51x0::Registers::DRV_STATUS);
  stepper_interface.printer.printDrvStatus(drv_status);
  tmc51x0::Registers::SwMode sw_mode;
  sw_mode.bytes = stepper_interface.registers.read(tmc51x0::Registers::SW_MODE);
  stepper_interface.printer.printSwMode(sw_mode);

  if (stepper_interface.controller.velocityReached() && !stall_stop_enabled)
  {
    stall_stop_enabled = true;
    Serial.println("enabling stall stop");
    stepper_interface.controller.enableStallStop();
  }

  if (stepper_interface.controller.zeroVelocity() && stall_stop_enabled)
  {
    stall_stop_enabled = false;
    Serial.print("disabling stall stop in ");
    Serial.print(STALL_DELAY);
    Serial.println(" milliseconds");
    delay(STALL_DELAY);
    stepper_interface.controller.disableStallStop();
  }

  delay(LOOP_DELAY);
}
