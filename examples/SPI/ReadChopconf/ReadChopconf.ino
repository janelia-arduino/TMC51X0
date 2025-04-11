#include <TMC51X0.hpp>


#if defined(ARDUINO_ARCH_RP2040)
SPIClassRP2040 & spi = SPI;
size_t SCK_PIN = 18;
size_t TX_PIN = 19;
size_t RX_PIN = 20;
#else
SPIClass & spi = SPI;
#endif

const tmc51x0::SpiParameters spi_parameters =
{
  .spi_ptr = &spi,
  .chip_select_pin = 8
};

const size_t ENABLE_HARDWARE_PIN = 4;

const uint32_t SERIAL_BAUD_RATE = 115200;
const uint16_t LOOP_DELAY = 1000;

// global variables
TMC51X0 tmc5160;
bool enabled;

void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);

#if defined(ARDUINO_ARCH_RP2040)
  spi.setSCK(SCK_PIN);
  spi.setTX(TX_PIN);
  spi.setRX(RX_PIN);
#endif
  spi.begin();
  tmc5160.setupSpi(spi_parameters);

  tmc5160.driver.setEnableHardwarePin(ENABLE_HARDWARE_PIN);

  while (!tmc5160.communicating())
  {
    Serial.println("No communication detected, check motor power and connections.");
    delay(LOOP_DELAY);
  }

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

  tmc5160.printer.readAndPrintChopconf();

  delay(LOOP_DELAY);
}
