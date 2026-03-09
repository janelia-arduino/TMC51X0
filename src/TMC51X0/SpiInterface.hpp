// ----------------------------------------------------------------------------
// SpiInterface.hpp
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------

#ifndef TMC51X0_SPI_INTERFACE_HPP
#define TMC51X0_SPI_INTERFACE_HPP
#include <Arduino.h>
#include <SPI.h>

#include "SpiParameters.hpp"
#include "Interface.hpp"

#include "SpiProtocol.hpp"

namespace tmc51x0
{
class SpiInterface : public Interface
{
public:
  void setup (SpiParameters spi_parameters);

  void writeRegister (uint8_t register_address,
                      uint32_t data) override;
  uint32_t readRegister (uint8_t register_address) override;

private:
  SpiParameters spi_parameters_;
  SPISettings spi_settings_;

  struct SpiStatus
  {
    uint8_t raw{ 0 };

    static bool
    bit (uint8_t r,
         uint8_t pos)
    {
      return ((r >> pos) & 0x01) != 0;
    }

    bool
    reset_flag () const
    {
      return bit (raw, 0);
    }
    bool
    driver_error () const
    {
      return bit (raw, 1);
    }
    bool
    sg2 () const
    {
      return bit (raw, 2);
    }
    bool
    standstill () const
    {
      return bit (raw, 3);
    }
    bool
    velocity_reached () const
    {
      return bit (raw, 4);
    }
    bool
    position_reached () const
    {
      return bit (raw, 5);
    }
    bool
    status_stop_l () const
    {
      return bit (raw, 6);
    }
    bool
    status_stop_r () const
    {
      return bit (raw, 7);
    }
  };
  SpiStatus spi_status_;

  uint8_t tx_buffer_[spi::DATAGRAM_SIZE];
  uint8_t rx_buffer_[spi::DATAGRAM_SIZE];

  void transferDatagram (const uint8_t tx[spi::DATAGRAM_SIZE],
                         uint8_t rx[spi::DATAGRAM_SIZE]);

  void enableChipSelect ();
  void disableChipSelect ();
  void beginTransaction ();
  void endTransaction ();
};
}
#endif
