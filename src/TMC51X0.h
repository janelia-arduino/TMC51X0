// ----------------------------------------------------------------------------
// TMC51X0.h
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------

#ifndef TMC51X0_H
#define TMC51X0_H
#include <Arduino.h>
#include <SPI.h>


class TMC51X0
{
public:
  TMC51X0();

  void setup(size_t chip_select_pin);

  // driver must be enabled before use it is disabled by default
  void setHardwareEnablePin(uint8_t hardware_enable_pin);
  void enable();
  void disable();

  uint8_t getVersion();
private:
  const static uint32_t SPI_CLOCK = 1000000;
#if defined(ARDUINO_ARCH_SAMD)
  const static BitOrder SPI_BIT_ORDER = MSBFIRST;
#else
  const static uint8_t SPI_BIT_ORDER = MSBFIRST;
#endif
  const static uint8_t SPI_MODE = SPI_MODE3;
  size_t chip_select_pin_;
  int16_t hardware_enable_pin_;

  uint32_t readRegister(uint8_t smda,
    uint8_t address);

  void enableChipSelect();
  void disableChipSelect();
  void beginTransaction();
  void endTransaction();

protected:
  virtual void spiBegin();
  virtual void spiBeginTransaction(SPISettings);
  virtual void spiEndTransaction();
  virtual uint8_t spiTransfer(uint8_t);

};

#endif
