// ----------------------------------------------------------------------------
// SpiInterface.cpp
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------
#include "SpiInterface.hpp"

using namespace tmc51x0;

void SpiInterface::setup(tmc51x0::SpiParameters spi_parameters) {
  interface_mode = Interface::SpiMode;
  spi_parameters_ = spi_parameters;
  spi_settings_ = SPISettings(spi_parameters.clock_rate, MSBFIRST, SPI_MODE3);
  device_reset_observed_ = false;

  pinMode(spi_parameters_.chip_select_pin, OUTPUT);
  disableChipSelect();
}

void SpiInterface::writeRegister(uint8_t register_address, uint32_t data) {
  spi::packDatagram(register_address, spi::RW_WRITE, data, tx_buffer_);
  transferDatagram(tx_buffer_, rx_buffer_);
  latchStatus();
}

uint32_t SpiInterface::readRegister(uint8_t register_address) {
  spi::packDatagram(register_address, spi::RW_READ, 0, tx_buffer_);

  // NOTE: data is returned on the second read.
  transferDatagram(tx_buffer_, rx_buffer_);
  transferDatagram(tx_buffer_, rx_buffer_);
  latchStatus();

  return spi::unpackData(rx_buffer_);
}

void SpiInterface::readRegisters(const uint8_t *register_addresses,
                                 uint32_t *values, size_t count) {
  if (count == 0) {
    return;
  }
  // Each datagram's reply carries the data requested by the previous one, so
  // the addresses go out back to back and one extra datagram collects the
  // last reply. The extra one repeats the last address (see the header).
  spi::packDatagram(register_addresses[0], spi::RW_READ, 0, tx_buffer_);
  transferDatagram(tx_buffer_, rx_buffer_);
  latchStatus();
  for (size_t i = 1; i <= count; ++i) {
    const uint8_t next =
        (i < count) ? register_addresses[i] : register_addresses[count - 1];
    spi::packDatagram(next, spi::RW_READ, 0, tx_buffer_);
    transferDatagram(tx_buffer_, rx_buffer_);
    latchStatus();
    values[i - 1] = spi::unpackData(rx_buffer_);
  }
}

void SpiInterface::latchStatus() {
  noInterrupts();
  spi_status_.raw = spi::unpackStatus(rx_buffer_);
  if (spi_status_.reset_flag()) {
    device_reset_observed_ = true;
  }
  interrupts();
}

bool SpiInterface::consumeDeviceResetObserved() {
  bool observed = device_reset_observed_;
  device_reset_observed_ = false;
  return observed;
}

// private

void SpiInterface::transferDatagram(const uint8_t tx[spi::DATAGRAM_SIZE],
                                    uint8_t rx[spi::DATAGRAM_SIZE]) {
  // One transfer for the whole 40-bit datagram where the core offers a
  // two-buffer form that hands the peripheral the full buffer (the RP2040
  // and Teensy cores do: the clocks then run back to back, ~13 us per
  // datagram at 4 MHz). The in-place `transfer(buf, count)` that every
  // Arduino core provides is the fallback; on the RP2040 core it is itself a
  // byte-by-byte loop, 1.5 us of idle bus between bytes (measured on a logic
  // analyser 2026-10-06), so it is not used where the better form exists.
  beginTransaction();
#if defined(ARDUINO_ARCH_RP2040) || defined(TEENSYDUINO)
  spi_parameters_.spi_ptr->transfer(tx, rx, spi::DATAGRAM_SIZE);
#else
  for (size_t i = 0; i < spi::DATAGRAM_SIZE; ++i) {
    rx[i] = tx[i];
  }
  spi_parameters_.spi_ptr->transfer(rx, spi::DATAGRAM_SIZE);
#endif
  endTransaction();
}

void SpiInterface::enableChipSelect() {
  digitalWrite(spi_parameters_.chip_select_pin, LOW);
}

void SpiInterface::disableChipSelect() {
  digitalWrite(spi_parameters_.chip_select_pin, HIGH);
}

void SpiInterface::beginTransaction() {
  spi_parameters_.spi_ptr->beginTransaction(spi_settings_);
  enableChipSelect();
}

void SpiInterface::endTransaction() {
  disableChipSelect();
  spi_parameters_.spi_ptr->endTransaction();
}
