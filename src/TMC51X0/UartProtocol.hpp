// ----------------------------------------------------------------------------
// UartProtocol.hpp
//
// Protocol helpers for Trinamic UART datagrams.
//
// NOTE: This header intentionally avoids Arduino dependencies so it can be
// unit-tested in PlatformIO's native environment.
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------

#ifndef TMC51X0_UART_PROTOCOL_HPP
#define TMC51X0_UART_PROTOCOL_HPP

#include <stddef.h>
#include <stdint.h>

namespace tmc51x0
{
namespace uart
{
constexpr uint8_t SYNC = 0x05; // low nibble is 0b0101, high nibble reserved (0)

constexpr uint8_t RW_READ = 0;
constexpr uint8_t RW_WRITE = 1;

constexpr size_t READ_REQUEST_SIZE = 4;
constexpr size_t WRITE_REQUEST_SIZE = 8;
constexpr size_t REPLY_SIZE = 8;

inline uint8_t
crc8 (const uint8_t *bytes,
      size_t len)
{
  // Trinamic UART CRC-8 (polynomial 0x07), LSB-first per byte.
  uint8_t crc = 0;
  for (size_t i = 0; i < len; ++i)
    {
      uint8_t current_byte = bytes[i];
      for (uint8_t j = 0; j < 8; ++j)
        {
          if ((crc >> 7) ^ (current_byte & 0x01))
            {
              crc = (crc << 1) ^ 0x07;
            }
          else
            {
              crc = (crc << 1);
            }
          current_byte >>= 1;
        }
    }
  return crc;
}

inline bool
checkCrc (const uint8_t *bytes,
          size_t size)
{
  if (size == 0)
    {
      return false;
    }
  return crc8 (bytes, size - 1) == bytes[size - 1];
}

inline void
packReadRequest (uint8_t node_address,
                 uint8_t register_address,
                 uint8_t out[READ_REQUEST_SIZE])
{
  out[0] = SYNC;
  out[1] = node_address;
  out[2] = (register_address & 0x7F) | (RW_READ << 7);
  out[3] = crc8 (out, READ_REQUEST_SIZE - 1);
}

inline void
packWriteRequest (uint8_t node_address,
                  uint8_t register_address,
                  uint32_t data,
                  uint8_t out[WRITE_REQUEST_SIZE])
{
  out[0] = SYNC;
  out[1] = node_address;
  out[2] = (register_address & 0x7F) | (RW_WRITE << 7);

  // Data is transmitted MSB-first on the wire.
  out[3] = static_cast<uint8_t> ((data >> 24) & 0xFF);
  out[4] = static_cast<uint8_t> ((data >> 16) & 0xFF);
  out[5] = static_cast<uint8_t> ((data >> 8) & 0xFF);
  out[6] = static_cast<uint8_t> ((data >> 0) & 0xFF);

  out[7] = crc8 (out, WRITE_REQUEST_SIZE - 1);
}

inline uint8_t
replyNode (const uint8_t in[REPLY_SIZE])
{
  return in[1];
}

inline uint8_t
replyRegister (const uint8_t in[REPLY_SIZE])
{
  return static_cast<uint8_t> (in[2] & 0x7F);
}

inline uint8_t
replyRw (const uint8_t in[REPLY_SIZE])
{
  return static_cast<uint8_t> ((in[2] >> 7) & 0x01);
}

inline uint32_t
replyData (const uint8_t in[REPLY_SIZE])
{
  // Data bytes are MSB-first at indices 3..6.
  return (static_cast<uint32_t> (in[3]) << 24)
         | (static_cast<uint32_t> (in[4]) << 16)
         | (static_cast<uint32_t> (in[5]) << 8)
         | (static_cast<uint32_t> (in[6]) << 0);
}

inline bool
checkSyncByte (uint8_t b)
{
  // Sync is in the low nibble. High nibble is reserved and should be 0.
  return ((b & 0x0F) == SYNC) && ((b & 0xF0) == 0);
}

} // namespace uart
} // namespace tmc51x0

#endif // TMC51X0_UART_PROTOCOL_HPP
