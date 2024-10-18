#include <TMC51X0.hpp>


#if defined(ARDUINO_ARCH_RP2040)
SPIClassRP2040 & spi = SPI1;
pin_size_t SCK_PIN = 10;
pin_size_t TX_PIN = 11;
pin_size_t RX_PIN = 12;
#else
SPIClass & spi = SPI;
#endif

// SPI Parameters
const uint32_t SPI_CLOCK_RATE = 1000000;
const pin_size_t SPI_CHIP_SELECT_PIN = 8;

const uint8_t ENABLE_POWER_PIN = 15;

const uint32_t SERIAL_BAUD_RATE = 115200;
const uint16_t DELAY = 1000;

// Instantiate TMC51X0
TMC51X0 tmc5160;
bool enabled;

void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);

  // Remove after testing
  pinMode(ENABLE_POWER_PIN, OUTPUT);
  digitalWrite(ENABLE_POWER_PIN, LOW);
  delay(5000);
  digitalWrite(ENABLE_POWER_PIN, HIGH);
  delay(5000);

#if defined(ARDUINO_ARCH_RP2040)
  spi.setSCK(SCK_PIN);
  spi.setTX(TX_PIN);
  spi.setRX(RX_PIN);
#endif
  spi.begin();
  tmc51x0::SpiParameters spi_parameters(spi, SPI_CLOCK_RATE, SPI_CHIP_SELECT_PIN);
  tmc5160.setupSpi(spi_parameters);

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
