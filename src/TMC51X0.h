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
  size_t chip_select_pin_;
  int16_t hardware_enable_pin_;

  void hardwareEnable();
  void hardwareDisable();

  const static uint8_t REGISTER_ADDRESS_IOIN = 0x04;
  union Input
  {
    struct
    {
      uint32_t refl_step : 1;
      uint32_t refr_dir : 1;
      uint32_t encb_dcen_cfg4 : 1;
      uint32_t enca_dcin_cfg5 : 1;
      uint32_t drv_enn : 1;
      uint32_t enc_n_dco_cfg6 : 1;
      uint32_t sd_mode : 1;
      uint32_t swcomp_in : 1;
      uint32_t version : 8;
      uint32_t reserved : 16;
    };
    uint32_t bytes;
  };
  const static uint8_t VERSION_TMC5130 = 0x11;
  const static uint8_t VERSION_TMC5160 = 0x30;

  // SPI
  const static uint32_t SPI_CLOCK = 1000000;
#if defined(ARDUINO_ARCH_SAMD)
  const static BitOrder SPI_BIT_ORDER = MSBFIRST;
#else
  const static uint8_t SPI_BIT_ORDER = MSBFIRST;
#endif
  const static uint8_t SPI_MODE = SPI_MODE3;
  struct SpiStatus
  {
    uint8_t reset_flag : 1;
    uint8_t driver_error : 1;
    uint8_t sg2 : 1;
    uint8_t standstill : 1;
    uint8_t velocity_reached : 1;
    uint8_t position_reached : 1;
    uint8_t status_stop_l : 1;
    uint8_t status_stop_r : 1;
  };
  SpiStatus spi_status_;

  const static uint8_t SPI_DATAGRAM_SIZE = 5;

  // MOSI Datagrams
  union MosiDatagram
  {
    struct
    {
      uint64_t data : 32;
      uint64_t register_address : 7;
      uint64_t rw : 1;
      uint64_t reserved : 24;
    };
    uint64_t bytes;
  };
  const static uint8_t SPI_RW_READ = 0;
  const static uint8_t SPI_RW_WRITE = 1;

  // MISO Datagrams
  union MisoDatagram
  {
    struct
    {
      uint64_t data : 32;
      SpiStatus spi_status;
      uint64_t reserved : 24;
    };
    uint64_t bytes;
  };

  void writeRegister(uint8_t register_address,
    uint32_t data);
  uint32_t readRegister(uint8_t register_address);
  MisoDatagram writeRead(MosiDatagram mosi_datagram);

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
