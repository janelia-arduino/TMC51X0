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

  const static uint8_t ADDRESS_IOIN = 0x04;
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

  // SPI Datagrams
  const static uint8_t DATAGRAM_SIZE = 4;

  // MOSI Datagrams
  union MosiDatagram
  {
    struct
    {
      uint32_t data : 24;
      uint32_t rw : 1;
      uint32_t address : 4;
      uint32_t smda : 2;
      uint32_t rrs : 1;
    };
    uint32_t bytes;
  };
  const static uint8_t RW_READ = 1;
  const static uint8_t RW_WRITE = 0;

  // MISO Datagrams
  union MisoDatagram
  {
    struct
    {
      uint32_t data : 24;
      Status status;
    };
    uint32_t bytes;
  };

  // UART Datagrams
  // const static uint8_t WRITE_READ_REPLY_DATAGRAM_SIZE = 8;
  // const static uint8_t DATA_SIZE = 4;
  // union WriteReadReplyDatagram
  // {
  //   struct
  //   {
  //     uint64_t sync : 4;
  //     uint64_t reserved : 4;
  //     uint64_t serial_address : 8;
  //     uint64_t register_address : 7;
  //     uint64_t rw : 1;
  //     uint64_t data : 32;
  //     uint64_t crc : 8;
  //   };
  //   uint64_t bytes;
  // };

  // const static uint8_t SYNC = 0b101;
  // const static uint8_t RW_READ = 0;
  // const static uint8_t RW_WRITE = 1;
  // const static uint8_t READ_REPLY_SERIAL_ADDRESS = 0b11111111;

  // const static uint8_t READ_REQUEST_DATAGRAM_SIZE = 4;
  // union ReadRequestDatagram
  // {
  //   struct
  //   {
  //     uint32_t sync : 4;
  //     uint32_t reserved : 4;
  //     uint32_t serial_address : 8;
  //     uint32_t register_address : 7;
  //     uint32_t rw : 1;
  //     uint32_t crc : 8;
  //   };
  //   uint32_t bytes;
  // };

  void write(uint8_t register_address,
    uint32_t data);
  uint32_t read(uint8_t register_address);

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
