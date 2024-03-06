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
bool enabled;

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
  tmc51x0::Registers::Chopconf chopconf;
  chopconf.bytes = register_data;
  printRegisterPortion("chopconf", chopconf.bytes, true);
  printRegisterPortion("toff", chopconf.toff);
  printRegisterPortion("hstart", chopconf.hstart);
  printRegisterPortion("hend", chopconf.hend);
  printRegisterPortion("fd3", chopconf.fd3);
  printRegisterPortion("disfdcc", chopconf.disfdcc);
  printRegisterPortion("chopper mode", chopconf.chopper_mode);
  printRegisterPortion("tbl", chopconf.tbl);
  printRegisterPortion("vhighfs", chopconf.vhighfs);
  printRegisterPortion("vhighchm", chopconf.vhighchm);
  printRegisterPortion("tpfd", chopconf.tpfd);
  printRegisterPortion("mres", chopconf.mres);
  printRegisterPortion("interpolation", chopconf.interpolation);
  printRegisterPortion("double edge", chopconf.double_edge);
  printRegisterPortion("diss2g", chopconf.diss2g);
  printRegisterPortion("diss2vs", chopconf.diss2vs);

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
  enabled = false;
}

void loop()
{
  if (enabled)
  {
    stepper.driver.softwareDisable();
  }
  else
  {
    stepper.driver.softwareEnable();
  }
  enabled = not enabled;
  printRegister(stepper.registers.read(tmc51x0::Registers::CHOPCONF));
  delay(DELAY);
}
