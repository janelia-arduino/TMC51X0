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

const long SERIAL_BAUD_RATE = 115200;
const int DELAY = 1000;

// Instantiate TMC51X0
TMC51X0 stepper_commander;

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

void printRegister(uint32_t register_data)
{
  tmc51x0::Registers::Gconf gconf;
  gconf.bytes = register_data;
  printRegisterPortion("gconf", gconf.bytes, true);
  printRegisterPortion("recalibrate", gconf.recalibrate);
  printRegisterPortion("faststandstill", gconf.faststandstill);
  printRegisterPortion("enable_pwm_mode", gconf.enable_pwm_mode);
  printRegisterPortion("multistep_filt", gconf.multistep_filt);
  printRegisterPortion("shaft", gconf.shaft);
  printRegisterPortion("diag0_error", gconf.diag0_error);
  printRegisterPortion("diag0_otpw", gconf.diag0_otpw);
  printRegisterPortion("diag0_stall_int_step", gconf.diag0_stall_int_step);
  printRegisterPortion("diag1_stall_poscomp_dir", gconf.diag1_stall_poscomp_dir);
  printRegisterPortion("diag1_index", gconf.diag1_index);
  printRegisterPortion("diag1_onstate", gconf.diag1_onstate);
  printRegisterPortion("diag1_steps_skipped", gconf.diag1_steps_skipped);
  printRegisterPortion("diag0_int_pushpull", gconf.diag0_int_pushpull);
  printRegisterPortion("diag1_poscomp_pushpull", gconf.diag1_poscomp_pushpull);
  printRegisterPortion("small_hysteresis", gconf.small_hysteresis);
  printRegisterPortion("stop_enable", gconf.stop_enable);
  printRegisterPortion("direct_mode", gconf.direct_mode);

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
  stepper_commander.setup(spi, CHIP_SELECT_PIN);
}

void loop()
{
  printRegister(stepper_commander.registers.read(tmc51x0::Registers::GCONF));
  delay(DELAY);
}
