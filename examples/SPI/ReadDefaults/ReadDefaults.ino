//#include <TMC51X0.hpp>
#if defined(ARDUINO_ARCH_STM32)
#include "../../../../src/TMC51X0.hpp"
#include "../../../../src/EQ6IO.hpp" 
#elif defined(ESP32)
#include "../../../../src/TMC51X0.hpp"
#include "../../../../src/EQ6IO.hpp" 
#else
#include "TMC51X0.hpp"
#endif

#if defined(ARDUINO_ARCH_RP2040)
SPIClassRP2040 & spi = SPI;
pin_size_t SCK_PIN = 18;
pin_size_t TX_PIN = 19;
pin_size_t RX_PIN = 20;

#elif defined(ARDUINO_ARCH_STM32)
//             MOSI   MISO   SCLK   SSEL
//SPIClass SPI_1(PA_7,  PA_6,  PA_5,  PB_0);    // RA
//SPIClass SPI_2(PB_15, PB_14, PB_13, PB_12);   // DEC
SPIClass RA_SPI(RA_MOSI,  RA_MISO,  RA_SCK,  RA_CSN);    // RA
SPIClass DEC_SPI(DEC_MOSI, DEC_MISO, DEC_SCK, DEC_CSN);   // DEC
#define spi RA_SPI
#define spi2 DEC_SPI

#elif defined(ESP32)
#define RA_MOSI   SPI_MOSI
#define RA_MISO   SPI_MISO
#define RA_SCK    SPI_SCK
#define DEC_MOSI  SPI_MOSI
#define DEC_MISO  SPI_MISO
#define DEC_SCK   SPI_SCK

SPIClass RA_SPI(HSPI);    // RA
SPIClass DEC_SPI(VSPI);   // DEC
#define SPI   RA_SPI
#define spi2  DEC_SPI

#else
SPIClass & spi = SPI1;
#endif

// SPI Parameters
const uint32_t SPI_CLOCK_RATE = 1000000;
const pin_size_t RA_CHIP_SELECT_PIN =   RA_CSN;
const pin_size_t DEC_CHIP_SELECT_PIN =  DEC_CSN;

const pin_size_t ENABLE_HARDWARE_PIN =  RADEC_EN;

#if defined(ARDUINO_ARCH_STM32)
//                       RX    TX
HardwareSerial  Serial1(PB7, PB6);
#endif

const uint32_t SERIAL_BAUD_RATE = 115200;
const uint16_t DELAY = 1000;

// Instantiate TMC51X0
TMC51X0 tmc5160;
uint32_t register_data;

void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);
  Serial.println("Read Defaults V25.1.0 - JMA");

#if defined(ARDUINO_ARCH_RP2040)
  spi.setSCK(SCK_PIN);
  spi.setTX(TX_PIN);
  spi.setRX(RX_PIN);
#endif

  RA_SPI.begin(RA_MOSI,  RA_MISO,  RA_SCK,  RA_CSN);
  DEC_SPI.begin(DEC_MOSI, DEC_MISO, DEC_SCK, DEC_CSN);
  tmc51x0::SpiParameters spi_parameters(spi, SPI_CLOCK_RATE, RA_CHIP_SELECT_PIN);
  tmc5160.setupSpi(spi_parameters);

}

void loop()
{
  Serial.println("----------");
  Serial.println("addr  data");
  Serial.println("----------");
  for (uint8_t register_address=0; register_address < tmc51x0::Registers::ADDRESS_COUNT; ++register_address)
  {
    if ((tmc5160.registers.readable((tmc51x0::Registers::RegisterAddress)register_address))
     && (tmc5160.registers.writeable((tmc51x0::Registers::RegisterAddress)register_address)))
    {
      register_data = 0xAA55AA55;
      register_data = tmc5160.registers.read((tmc51x0::Registers::RegisterAddress)register_address);
      if (register_data != 0)
      {
        Serial.printf("0x%02X, 0x%04X - ", register_address, register_data);
        Serial.print(register_address, HEX);
        Serial.print(", ");
        Serial.println(register_data, HEX);
      }
    }
  }
  delay(DELAY);
}
