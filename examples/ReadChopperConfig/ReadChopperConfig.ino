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
  tmc51x0::Registers::ChopperConfig chopper_config;
  chopper_config.bytes = register_data;
  printRegisterPortion("chopper config", chopper_config.bytes, true);
  printRegisterPortion("toff", chopper_config.toff);
  printRegisterPortion("hstart", chopper_config.hstart);
  printRegisterPortion("hend", chopper_config.hend);
  printRegisterPortion("fd3", chopper_config.fd3);
  printRegisterPortion("disfdcc", chopper_config.disfdcc);
  printRegisterPortion("chopper mode", chopper_config.chopper_mode);
  printRegisterPortion("tbl", chopper_config.tbl);
  printRegisterPortion("vhighfs", chopper_config.vhighfs);
  printRegisterPortion("vhighchm", chopper_config.vhighchm);
  printRegisterPortion("tpfd", chopper_config.tpfd);
  printRegisterPortion("mres", chopper_config.mres);
  printRegisterPortion("interpolation", chopper_config.interpolation);
  printRegisterPortion("double edge", chopper_config.double_edge);
  printRegisterPortion("diss2g", chopper_config.diss2g);
  printRegisterPortion("diss2vs", chopper_config.diss2vs);

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
  printRegister(stepper.registers.read(tmc51x0::Registers::CHOPPER_CONFIG));
  delay(DELAY);
}
