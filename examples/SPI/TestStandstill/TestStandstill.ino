#include <TMC51X0.hpp>


#if defined(ARDUINO_ARCH_RP2040)
SPIClassRP2040 & spi = SPI1;
size_t SCK_PIN = 10;
size_t TX_PIN = 11;
size_t RX_PIN = 12;
#else
SPIClass & spi = SPI;
#endif

const auto spi_parameters =
  tmc51x0::SpiParameters{}
    .withSpi(&spi)
    .withChipSelectPin(8);

const uint32_t SERIAL_BAUD_RATE = 115200;
const uint16_t DELAY = 4000;

// driver constants
const uint8_t HOLD_CURRENT = 0;
const uint8_t HOLD_DELAY = 0;

// global variables
TMC51X0 stepper;

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

  stepper.driver.enableStealthChop();
  stepper.driver.writeHoldCurrent(HOLD_CURRENT);
  stepper.driver.writeHoldDelay(HOLD_DELAY);

  while (!stepper.communicating())
  {
    Serial.println("No communication detected, check motor power and connections.");
    delay(DELAY);
  }

  stepper.driver.enable();

  delay(DELAY);
}

void loop()
{
  Serial.println("standstill mode = NormalMode");
  stepper.driver.writeStandstillMode(tmc51x0::NormalMode);
  delay(DELAY);

  Serial.println("standstill mode = FreewheelingMode");
  stepper.driver.writeStandstillMode(tmc51x0::FreewheelingMode);
  delay(DELAY);

  Serial.println("standstill mode = PassiveBrakingLsMode");
  stepper.driver.writeStandstillMode(tmc51x0::PassiveBrakingLsMode);
  delay(DELAY);

  Serial.println("standstill mode = PassiveBrakingHsMode");
  stepper.driver.writeStandstillMode(tmc51x0::PassiveBrakingHsMode);
  delay(DELAY);

  Serial.println("--------------------------");
}
