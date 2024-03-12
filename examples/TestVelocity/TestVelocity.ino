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

const uint8_t GLOBAL_CURRENT_SCALAR_PERCENT = 50;
const uint8_t RUN_CURRENT_PERCENT = 50;
const uint8_t PWM_OFFSET = 100;
const uint8_t PWM_GRADIENT = 100;

const uint32_t VELOCITY_MAX = 10000;
const uint32_t ACCELERATION_MAX = 10000;
const tmc51x0::Controller::RampMode RAMP_MODE = tmc51x0::Controller::VELOCITY_POSITIVE;

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
  printRegisterPortion("status_latch_l", ramp_stat.status_latch_l);
  printRegisterPortion("status_latch_r", ramp_stat.status_latch_r);
  printRegisterPortion("event_stop_l", ramp_stat.event_stop_l);
  printRegisterPortion("event_stop_r", ramp_stat.event_stop_r);
  printRegisterPortion("event_pos_reached", ramp_stat.event_pos_reached);
  printRegisterPortion("velocity_reached", ramp_stat.velocity_reached);
  printRegisterPortion("position_reached", ramp_stat.position_reached);
  printRegisterPortion("vzero", ramp_stat.vzero);
  printRegisterPortion("t_zerowait_active", ramp_stat.t_zerowait_active);
  printRegisterPortion("second_move", ramp_stat.second_move);
  printRegisterPortion("status_sg", ramp_stat.status_sg);
  Serial.println("--------------------------");
}

void printRegisterDrvStatus(uint32_t register_data)
{
  tmc51x0::Registers::DrvStatus drv_status;
  drv_status.bytes = register_data;
  printRegisterPortion("drv_status", drv_status.bytes, true);
  printRegisterPortion("sg_result", drv_status.sg_result, true);
  printRegisterPortion("s2vsa", drv_status.s2vsa);
  printRegisterPortion("s2vsb", drv_status.s2vsb);
  printRegisterPortion("stealth", drv_status.stealth);
  printRegisterPortion("fsactive", drv_status.fsactive);
  printRegisterPortion("cs_actual", drv_status.cs_actual, true);
  printRegisterPortion("ot", drv_status.ot);
  printRegisterPortion("otpw", drv_status.otpw);
  printRegisterPortion("s2ga", drv_status.s2ga);
  printRegisterPortion("s2gb", drv_status.s2gb);
  printRegisterPortion("ola", drv_status.ola);
  printRegisterPortion("olb", drv_status.olb);
  printRegisterPortion("stst", drv_status.stst);
  Serial.println("--------------------------");
}

void printRegisterSwMode(uint32_t register_data)
{
  tmc51x0::Registers::SwMode sw_mode;
  sw_mode.bytes = register_data;
  printRegisterPortion("sw_mode", sw_mode.bytes, true);
  printRegisterPortion("stop_l_enable", sw_mode.stop_l_enable);
  printRegisterPortion("stop_r_enable", sw_mode.stop_r_enable);
  printRegisterPortion("pol_stop_l", sw_mode.pol_stop_l);
  printRegisterPortion("pol_stop_r", sw_mode.pol_stop_r);
  printRegisterPortion("swap_lr", sw_mode.swap_lr);
  printRegisterPortion("latch_l_active", sw_mode.latch_l_active);
  printRegisterPortion("latch_l_inactive", sw_mode.latch_l_inactive);
  printRegisterPortion("latch_r_active", sw_mode.latch_r_active);
  printRegisterPortion("latch_r_inactive", sw_mode.latch_r_inactive);
  printRegisterPortion("en_latch_encoder", sw_mode.en_latch_encoder);
  printRegisterPortion("sg_stop", sw_mode.sg_stop);
  printRegisterPortion("en_softstop", sw_mode.en_softstop);
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

  stepper.driver.setGlobalCurrentScaler(GLOBAL_CURRENT_SCALAR_PERCENT);
  stepper.driver.setRunCurrent(RUN_CURRENT_PERCENT);
  stepper.driver.setPwmOffset(PWM_OFFSET);
  stepper.driver.setPwmGradient(PWM_GRADIENT);

  stepper.controller.setVelocityMax(VELOCITY_MAX);
  stepper.controller.setAccelerationMax(ACCELERATION_MAX);
  stepper.controller.setRampMode(RAMP_MODE);

  stepper.driver.enable();
}

void loop()
{
  printGlobalStatus(stepper.readAndClearGlobalStatus());
  printRegisterRampStat(stepper.registers.read(tmc51x0::Registers::RAMP_STAT));
  printRegisterDrvStatus(stepper.registers.read(tmc51x0::Registers::DRV_STATUS));
  printRegisterSwMode(stepper.registers.read(tmc51x0::Registers::SW_MODE));

  Serial.print("ramp mode: ");
  Serial.println(stepper.registers.read(tmc51x0::Registers::RAMPMODE));
  Serial.print("actual position: ");
  Serial.println(stepper.controller.getActualPosition());
  Serial.print("actual velocity: ");
  Serial.println(stepper.controller.getActualVelocity());
  Serial.println("--------------------------");
  delay(DELAY);
}
