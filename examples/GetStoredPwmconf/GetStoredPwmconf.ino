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
TMC51X0 tmc5160;

void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);
#if defined(ARDUINO_ARCH_RP2040)
  spi.setSCK(SCK_PIN);
  spi.setTX(TX_PIN);
  spi.setRX(RX_PIN);
#endif
  tmc5160.setup(spi, CHIP_SELECT_PIN);
  tmc5160.driver.setHardwareEnablePin(HARDWARE_ENABLE_PIN);
}

void loop()
{
  tmc51x0::Registers::Pwmconf pwmconf;
  pwmconf.bytes = tmc5160.registers.getStored(tmc51x0::Registers::PWMCONF);
  tmc5160.printer.printRegister(pwmconf);
  delay(DELAY);
}
