#include <TMC51X0.hpp>


#if defined(ARDUINO_ARCH_RP2040)
SPIClassRP2040 & spi = SPI;
size_t SCK_PIN = 18;
size_t TX_PIN = 19;
size_t RX_PIN = 20;
#else
SPIClass & spi = SPI;
#endif

const auto spi_parameters =
  tmc51x0::SpiParameters{}
    .withSpi(&spi)
    .withChipSelectPin(8);

const size_t ENABLE_HARDWARE_PIN = 4;

const uint32_t SERIAL_BAUD_RATE = 115200;
const uint16_t LOOP_DELAY = 1000;

// global variables
TMC51X0 stepper;
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
  stepper.setupSpi(spi_parameters);

  stepper.driver.setEnableHardwarePin(ENABLE_HARDWARE_PIN);

  enabled = false;

  while (!stepper.communicating())
  {
    Serial.println("No communication detected, check motor power and connections.");
    delay(LOOP_DELAY);
  }
}

void loop()
{
  if (enabled)
  {
    stepper.driver.disable();
  }
  else
  {
    stepper.driver.enable();
  }
  enabled = not enabled;

  stepper.printer.readAndPrintIoin();

  delay(LOOP_DELAY);
}
