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
const int DELAY = 1000;

const uint8_t GLOBAL_CURRENT_SCALAR = 129;
const uint8_t RUN_CURRENT = 16;
const uint8_t PWM_OFFSET = 25;
const uint8_t PWM_GRADIENT = 25;
// const int8_t STALL_GUARD_THRESHOLD = -20;
// const uint8_t COOL_STEP_MINIMUM = 1;
// const uint8_t COOL_STEP_MAXIMUM = 0;

const uint32_t VELOCITY_MAX = 20000;
const uint32_t ACCELERATION_MAX = 10000;
const tmc51x0::Controller::RampMode RAMP_MODE = tmc51x0::Controller::VELOCITY_POSITIVE;
const int32_t INITIAL_POSITION = 0;

// Instantiate TMC51X0
TMC51X0 stepper;

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

void printGlobalStatus(tmc51x0::Registers::Gstat gstat)
{
  printRegisterPortion("reset", gstat.reset);
  printRegisterPortion("drv_err", gstat.drv_err);
  printRegisterPortion("uv_cp", gstat.uv_cp);
  Serial.println("--------------------------");
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

void printRegisterPwmScale(uint32_t register_data)
{
  tmc51x0::Registers::PwmScale pwm_scale;
  pwm_scale.bytes = register_data;
  printRegisterPortion("pwm_scale", pwm_scale.bytes, true);
  printRegisterPortion("pwm_scale_sum", pwm_scale.pwm_scale_sum, true);
  printRegisterPortion("pwm_scale_auto", pwm_scale.pwm_scale_auto, true);
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

  stepper.driver.setGlobalCurrentScaler(GLOBAL_CURRENT_SCALAR);
  stepper.driver.setRunCurrent(RUN_CURRENT);
  stepper.driver.setPwmOffset(PWM_OFFSET);
  stepper.driver.setPwmGradient(PWM_GRADIENT);
  // stepper.driver.setStallGuardThreshold(STALL_GUARD_THRESHOLD);
  // stepper.driver.enableCoolStep(COOL_STEP_MINIMUM, COOL_STEP_MAXIMUM);

  stepper.controller.setVelocityMax(VELOCITY_MAX);
  stepper.controller.setAccelerationMax(ACCELERATION_MAX);
  stepper.controller.setRampMode(RAMP_MODE);
  stepper.controller.setActualPosition(INITIAL_POSITION);

  stepper.driver.enable();
}

void loop()
{
  printGlobalStatus(stepper.readAndClearGlobalStatus());
  printRegisterRampStat(stepper.registers.read(tmc51x0::Registers::RAMP_STAT));
  printRegisterDrvStatus(stepper.registers.read(tmc51x0::Registers::DRV_STATUS));
  printRegisterPwmScale(stepper.registers.read(tmc51x0::Registers::PWM_SCALE));

  Serial.print("actual position: ");
  Serial.println(stepper.controller.getActualPosition());
  Serial.print("actual velocity: ");
  Serial.println(stepper.controller.getActualVelocity());
  Serial.println("--------------------------");
  Serial.println("--------------------------");
  delay(DELAY);
}
