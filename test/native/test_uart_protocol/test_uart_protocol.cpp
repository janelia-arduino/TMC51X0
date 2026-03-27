#include <unity.h>

#include "TMC51X0/UartProtocol.hpp"

using namespace tmc51x0;

static void test_uart_pack_read_request() {
  uint8_t req[uart::READ_REQUEST_SIZE] = {0};
  uart::packReadRequest(0x01, 0x12, req);

  TEST_ASSERT_EQUAL_UINT8(0x05, req[0]);
  TEST_ASSERT_EQUAL_UINT8(0x01, req[1]);
  TEST_ASSERT_EQUAL_UINT8(0x12, req[2]);
  TEST_ASSERT_TRUE(uart::checkCrc(req, uart::READ_REQUEST_SIZE));

  // Hard-coded CRC regression guard for this request.
  // If this fails, either the CRC algorithm changed or the packet layout did.
  TEST_ASSERT_EQUAL_UINT8(0x01, req[3]);
}

static void test_uart_pack_write_request() {
  uint8_t req[uart::WRITE_REQUEST_SIZE] = {0};
  uart::packWriteRequest(0x01, 0x12, 0x11223344UL, req);

  TEST_ASSERT_EQUAL_UINT8(0x05, req[0]);
  TEST_ASSERT_EQUAL_UINT8(0x01, req[1]);
  TEST_ASSERT_EQUAL_UINT8(0x92, req[2]);
  TEST_ASSERT_EQUAL_UINT8(0x11, req[3]);
  TEST_ASSERT_EQUAL_UINT8(0x22, req[4]);
  TEST_ASSERT_EQUAL_UINT8(0x33, req[5]);
  TEST_ASSERT_EQUAL_UINT8(0x44, req[6]);

  TEST_ASSERT_TRUE(uart::checkCrc(req, uart::WRITE_REQUEST_SIZE));

  // Hard-coded CRC regression guard.
  TEST_ASSERT_EQUAL_UINT8(0x27, req[7]);
}

static void test_uart_reply_crc_detection() {
  uint8_t reply[uart::REPLY_SIZE] = {
      0x05, // sync
      0x01, // node
      0x12, // reg (read)
      0xAA, 0xBB, 0xCC, 0xDD,
      0x00, // crc (filled below)
  };

  reply[7] = uart::crc8(reply, uart::REPLY_SIZE - 1);
  TEST_ASSERT_TRUE(uart::checkCrc(reply, uart::REPLY_SIZE));

  // Flip one bit in the payload and CRC should fail.
  reply[6] ^= 0x01;
  TEST_ASSERT_FALSE(uart::checkCrc(reply, uart::REPLY_SIZE));
}

void setUp() {}

void tearDown() {}

int main(int argc, char **argv) {
  UNITY_BEGIN();

  RUN_TEST(test_uart_pack_read_request);
  RUN_TEST(test_uart_pack_write_request);
  RUN_TEST(test_uart_reply_crc_detection);

  return UNITY_END();
}
