#include <TMC51X0.hpp>


#if defined(ARDUINO_ARCH_RP2040)
SPIClassRP2040 & spi = SPI1;
size_t SCK_PIN = 10;
size_t TX_PIN = 11;
size_t RX_PIN = 12;
#else
SPIClass & spi = SPI;
#endif

const uint8_t PRISM_COUNT = 7;

tmc51x0::SpiParameters spi_parameters =
{
  spi,
  1000000 // clock_rate
};
const size_t SPI_CHIP_SELECT_PINS[PRISM_COUNT] = {14, 8, 7, 6, 5, 4, 3};

const size_t ENABLE_POWER_PIN = 15;
const uint8_t ENABLE_POWER_POLARITY = HIGH;
const uint16_t RESET_DELAY = 5000;

const uint32_t SERIAL_BAUD_RATE = 115200;
const uint16_t LOOP_DELAY = 1000;

// Instantiate TMC51X0
TMC51X0 prisms[PRISM_COUNT];

void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);

  for (uint8_t prism_index=0; prism_index<PRISM_COUNT; ++prism_index)
  {
    prisms[prism_index].setEnablePowerPin(ENABLE_POWER_PIN);
    prisms[prism_index].setEnablePowerPolarity(ENABLE_POWER_POLARITY);
    prisms[prism_index].disablePower();
  }
  delay(RESET_DELAY);
  for (uint8_t prism_index=0; prism_index<PRISM_COUNT; ++prism_index)
  {
    prisms[prism_index].enablePower();
  }
  delay(RESET_DELAY);

#if defined(ARDUINO_ARCH_RP2040)
  spi.setSCK(SCK_PIN);
  spi.setTX(TX_PIN);
  spi.setRX(RX_PIN);
#endif
  spi.begin();

  for (uint8_t prism_index=0; prism_index<PRISM_COUNT; ++prism_index)
  {
    spi_parameters.chip_select_pin = SPI_CHIP_SELECT_PINS[prism_index];
    prisms[prism_index].setupSpi(spi_parameters);
  }
}

void loop()
{
  for (uint8_t prism_index=0; prism_index<PRISM_COUNT; ++prism_index)
  {
    Serial.print("prism ");
    Serial.print(prism_index);
    Serial.print(" communicating : ");
    bool communicating = prisms[prism_index].communicating();
    Serial.println(communicating);
    delay(LOOP_DELAY);
  }
}
