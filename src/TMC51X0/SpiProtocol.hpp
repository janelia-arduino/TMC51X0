// ----------------------------------------------------------------------------
// SpiProtocol.hpp
//
// Protocol helpers for Trinamic SPI datagrams.
//
// NOTE: This header intentionally avoids Arduino dependencies so it can be
// unit-tested in PlatformIO's native environment.
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------

#ifndef TMC51X0_SPI_PROTOCOL_HPP
#define TMC51X0_SPI_PROTOCOL_HPP

#include <stddef.h>
#include <stdint.h>

namespace tmc51x0 {
namespace spi {
constexpr size_t DATAGRAM_SIZE = 5;

constexpr uint8_t RW_READ = 0;
constexpr uint8_t RW_WRITE = 1;

inline void packDatagram(uint8_t register_address, uint8_t rw, uint32_t data,
                         uint8_t out[DATAGRAM_SIZE]) {
  // Byte order on the wire:
  //   [0] addr|rw
  //   [1] data[31:24]
  //   [2] data[23:16]
  //   [3] data[15:8]
  //   [4] data[7:0]
  out[0] = (register_address & 0x7F) | ((rw & 0x01) << 7);
  out[1] = static_cast<uint8_t>((data >> 24) & 0xFF);
  out[2] = static_cast<uint8_t>((data >> 16) & 0xFF);
  out[3] = static_cast<uint8_t>((data >> 8) & 0xFF);
  out[4] = static_cast<uint8_t>((data >> 0) & 0xFF);
}

inline uint8_t unpackStatus(const uint8_t in[DATAGRAM_SIZE]) {
  // Status is returned in the first byte.
  return in[0];
}

inline uint32_t unpackData(const uint8_t in[DATAGRAM_SIZE]) {
  return (static_cast<uint32_t>(in[1]) << 24) |
         (static_cast<uint32_t>(in[2]) << 16) |
         (static_cast<uint32_t>(in[3]) << 8) |
         (static_cast<uint32_t>(in[4]) << 0);
}

} // namespace spi
} // namespace tmc51x0

#endif // TMC51X0_SPI_PROTOCOL_HPP
