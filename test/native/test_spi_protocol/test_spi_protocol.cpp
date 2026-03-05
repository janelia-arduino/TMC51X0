#include <unity.h>

#include "TMC51X0/SpiProtocol.hpp"

using namespace tmc51x0;

static void
test_spi_pack_datagram ()
{
  uint8_t buf[spi::DATAGRAM_SIZE] = { 0 };
  spi::packDatagram (0x6C, spi::RW_WRITE, 0x11223344UL, buf);

  TEST_ASSERT_EQUAL_UINT8 (0xEC, buf[0]); // 0x6C | 0x80
  TEST_ASSERT_EQUAL_UINT8 (0x11, buf[1]);
  TEST_ASSERT_EQUAL_UINT8 (0x22, buf[2]);
  TEST_ASSERT_EQUAL_UINT8 (0x33, buf[3]);
  TEST_ASSERT_EQUAL_UINT8 (0x44, buf[4]);
}

static void
test_spi_unpack ()
{
  const uint8_t buf[spi::DATAGRAM_SIZE] = {
    0x5A, // status
    0x11,
    0x22,
    0x33,
    0x44,
  };

  TEST_ASSERT_EQUAL_UINT8 (0x5A, spi::unpackStatus (buf));
  TEST_ASSERT_EQUAL_UINT32 (0x11223344UL, spi::unpackData (buf));
}

void
setUp ()
{
}

void
tearDown ()
{
}

int
main (int argc,
      char **argv)
{
  UNITY_BEGIN ();

  RUN_TEST (test_spi_pack_datagram);
  RUN_TEST (test_spi_unpack);

  return UNITY_END ();
}
