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
TMC51X0 stepper_interface;
bool enabled;

void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);
#if defined(ARDUINO_ARCH_RP2040)
  spi.setSCK(SCK_PIN);
  spi.setTX(TX_PIN);
  spi.setRX(RX_PIN);
#endif
  stepper_interface.setup(spi, CHIP_SELECT_PIN);
  stepper_interface.driver.setHardwareEnablePin(HARDWARE_ENABLE_PIN);
  enabled = false;
}

void loop()
{
  if (enabled)
  {
    stepper_interface.driver.disable();
  }
  else
  {
    stepper_interface.driver.enable();
  }
  enabled = not enabled;
  tmc51x0::Registers::Ioin ioin;
  ioin.bytes = stepper_interface.registers.read(tmc51x0::Registers::IOIN);
  stepper_interface.printer.printIoin(ioin);
  delay(DELAY);
}
