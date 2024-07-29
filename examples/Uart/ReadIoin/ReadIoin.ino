#include <TMC51X0.hpp>


#if defined(ARDUINO_ARCH_RP2040)
pin_size_t TX_PIN = 4;
pin_size_t RX_PIN = 5;
#endif

HardwareSerial & uart = Serial1;

const uint8_t CHIP_SELECT_PIN = 10;

const uint8_t ENABLE_HARDWARE_PIN = 4;

const long SERIAL_BAUD_RATE = 115200;
const int DELAY = 1000;

// Instantiate TMC51X0
TMC51X0 tmc5160;
bool enabled;

void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);
#if defined(ARDUINO_ARCH_RP2040)
  uart.setTX(TX_PIN);
  uart.setRX(RX_PIN);
#endif

  tmc5160.setup(uart, CHIP_SELECT_PIN);
  tmc5160.driver.setEnableHardwarePin(ENABLE_HARDWARE_PIN);

  enabled = false;
}

void loop()
{
  if (enabled)
  {
    tmc5160.driver.disable();
  }
  else
  {
    tmc5160.driver.enable();
  }
  enabled = not enabled;

  tmc5160.printer.readAndPrintIoin();

  delay(DELAY);
}
