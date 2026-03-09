// ----------------------------------------------------------------------------
// SpiInterface.cpp
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------
#include "SpiInterface.hpp"

using namespace tmc51x0;

void
SpiInterface::setup (tmc51x0::SpiParameters spi_parameters)
{
  interface_mode = Interface::SpiMode;
  spi_parameters_ = spi_parameters;
  spi_settings_ = SPISettings (spi_parameters.clock_rate,
                               MSBFIRST,
                               SPI_MODE3);
  device_reset_observed_ = false;

  pinMode (spi_parameters_.chip_select_pin, OUTPUT);
  disableChipSelect ();
}

void
SpiInterface::writeRegister (uint8_t register_address,
                             uint32_t data)
{
  spi::packDatagram (register_address,
                     spi::RW_WRITE,
                     data,
                     tx_buffer_);
  transferDatagram (tx_buffer_, rx_buffer_);

  noInterrupts ();
  spi_status_.raw = spi::unpackStatus (rx_buffer_);
  if (spi_status_.reset_flag ())
    {
      device_reset_observed_ = true;
    }
  interrupts ();
}

uint32_t
SpiInterface::readRegister (uint8_t register_address)
{
  spi::packDatagram (register_address,
                     spi::RW_READ,
                     0,
                     tx_buffer_);

  // NOTE: data is returned on the second read.
  transferDatagram (tx_buffer_, rx_buffer_);
  transferDatagram (tx_buffer_, rx_buffer_);

  noInterrupts ();
  spi_status_.raw = spi::unpackStatus (rx_buffer_);
  if (spi_status_.reset_flag ())
    {
      device_reset_observed_ = true;
    }
  interrupts ();

  return spi::unpackData (rx_buffer_);
}

bool
SpiInterface::consumeDeviceResetObserved ()
{
  bool observed = device_reset_observed_;
  device_reset_observed_ = false;
  return observed;
}

// private

void
SpiInterface::transferDatagram (const uint8_t tx[spi::DATAGRAM_SIZE],
                                uint8_t rx[spi::DATAGRAM_SIZE])
{
  beginTransaction ();
  for (size_t i = 0; i < spi::DATAGRAM_SIZE; ++i)
    {
      rx[i] = spi_parameters_.spi_ptr->transfer (tx[i]);
    }
  endTransaction ();
}

void
SpiInterface::enableChipSelect ()
{
  digitalWrite (spi_parameters_.chip_select_pin, LOW);
}

void
SpiInterface::disableChipSelect ()
{
  digitalWrite (spi_parameters_.chip_select_pin, HIGH);
}

void
SpiInterface::beginTransaction ()
{
  spi_parameters_.spi_ptr->beginTransaction (spi_settings_);
  enableChipSelect ();
}

void
SpiInterface::endTransaction ()
{
  disableChipSelect ();
  spi_parameters_.spi_ptr->endTransaction ();
}
