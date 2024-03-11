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

void printRegister(uint32_t register_data)
{
  tmc51x0::Registers::Pwmconf pwmconf;
  pwmconf.bytes = register_data;
  printRegisterPortion("pwmconf", pwmconf.bytes, true);
  printRegisterPortion("pwm_ofs", pwmconf.pwm_ofs, true);
  printRegisterPortion("pwm_grad", pwmconf.pwm_grad, true);
  printRegisterPortion("pwm_freq", pwmconf.pwm_freq);
  printRegisterPortion("pwm_autoscale", pwmconf.pwm_autoscale);
  printRegisterPortion("pwm_autograd", pwmconf.pwm_autograd);
  printRegisterPortion("freewheel", pwmconf.freewheel);
  printRegisterPortion("pwm_reg", pwmconf.pwm_reg, true);
  printRegisterPortion("pwm_lim", pwmconf.pwm_lim, true);

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
}

void loop()
{
  printRegister(stepper.registers.getStored(tmc51x0::Registers::PWMCONF));
  delay(DELAY);
}
