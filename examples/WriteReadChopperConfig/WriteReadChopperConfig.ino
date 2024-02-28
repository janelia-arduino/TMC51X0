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
const int DELAY = 250;

// Instantiate TMC51X0
TMC51X0 stepper_commander;
tmc51x0::Registers::ChopperConfig chopper_config;


void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);
#if defined(ARDUINO_ARCH_RP2040)
  spi.setSCK(SCK_PIN);
  spi.setTX(TX_PIN);
  spi.setRX(RX_PIN);
#endif
  stepper_commander.setup(spi, CHIP_SELECT_PIN);

  stepper_commander.driver.setHardwareEnablePin(HARDWARE_ENABLE_PIN);
}

void loop()
{
  chopper_config = stepper_commander.readChopperConfig();
  Serial.print("Stepper controller driver chopper_config: ");
  Serial.print(chopper_config.bytes, HEX);
  Serial.println();
  delay(DELAY);
}
