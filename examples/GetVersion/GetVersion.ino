#include <TMC51X0.hpp>


#if defined(ARDUINO_ARCH_RP2040)
SPIClassRP2040 & spi = SPI;
#else
SPIClass & spi = SPI;
#endif
const uint8_t CHIP_SELECT_PIN = 10;
const uint8_t HARDWARE_ENABLE_PIN = 4;

const long SERIAL_BAUD_RATE = 115200;
const int DELAY = 250;

// Instantiate TMC51X0
TMC51X0 stepper_commander;
uint8_t version;


void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);
#if defined(ARDUINO_ARCH_RP2040)
  spi.setRX(20);
  spi.setSCK(18);
  spi.setTX(19);
#endif
  stepper_commander.setup(spi, CHIP_SELECT_PIN);

  stepper_commander.driver.setHardwareEnablePin(HARDWARE_ENABLE_PIN);
}

void loop()
{
  version = stepper_commander.getVersion();
  Serial.print("Stepper controller driver version: ");
  Serial.print(version, HEX);
  Serial.println();
  delay(DELAY);
}
