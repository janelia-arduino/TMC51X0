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

const uint32_t SERIAL_BAUD_RATE = 115200;
const uint16_t LOOP_DELAY = 1000;

// global variables
TMC51X0 stepper;
uint32_t register_data;

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

  while (!stepper.communicating())
  {
    Serial.println("No communication detected, check motor power and connections.");
    delay(LOOP_DELAY);
  }
}

void loop()
{
  for (uint8_t register_address=0; register_address < tmc51x0::Registers::AddressCount; ++register_address)
  {
    if ((stepper.registers.readable((tmc51x0::Registers::RegisterAddress)register_address)) && (stepper.registers.writeable((tmc51x0::Registers::RegisterAddress)register_address)))
    {
      register_data = stepper.registers.read((tmc51x0::Registers::RegisterAddress)register_address);
      if (register_data != 0)
      {
        Serial.print("register_address: 0x");
        Serial.print(register_address, HEX);
        Serial.print(", register_data: 0x");
        Serial.println(register_data, HEX);
      }
    }
  }
  Serial.println("-------------------------");
  delay(LOOP_DELAY);
}
