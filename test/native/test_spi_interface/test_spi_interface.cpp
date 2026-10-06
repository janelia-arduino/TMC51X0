#include <unity.h>

#include <stddef.h>
#include <stdint.h>
#include <vector>

#define private public
#include "TMC51X0/SpiInterface.hpp"
#undef private

#include "../../../src/TMC51X0/SpiInterface.cpp"

using namespace tmc51x0;

namespace {
class FakeSPI : public SPIClass {
public:
  std::vector<uint8_t> rx_bytes;
  std::vector<uint8_t> tx_bytes;
  size_t index{0};
  size_t buffer_transfers{0};

  using SPIClass::transfer;
  uint8_t transfer(uint8_t out) override {
    tx_bytes.push_back(out);
    if (index < rx_bytes.size()) {
      return rx_bytes[index++];
    }
    return 0;
  }
  void transfer(void *buf, size_t count) override {
    ++buffer_transfers;
    SPIClass::transfer(buf, count);
  }
};
} // namespace

void setUp() {}

void tearDown() {}

static void test_spi_read_latches_reset_flag_once(void) {
  FakeSPI spi;
  spi.rx_bytes = {
      0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x11, 0x22, 0x33, 0x44,
  };

  SpiInterface interface;
  interface.setup(SpiParameters{}.withSpi(&spi).withChipSelectPin(8));

  TEST_ASSERT_EQUAL_HEX32(0x11223344UL, interface.readRegister(0x6C));
  TEST_ASSERT_TRUE(interface.consumeDeviceResetObserved());
  TEST_ASSERT_FALSE(interface.consumeDeviceResetObserved());
}

static void test_spi_datagram_is_one_buffer_transfer(void) {
  FakeSPI spi;
  SpiInterface interface;
  interface.setup(SpiParameters{}.withSpi(&spi).withChipSelectPin(8));

  interface.writeRegister(0x6C, 0x11223344UL);
  TEST_ASSERT_EQUAL_size_t(1, spi.buffer_transfers);
  TEST_ASSERT_EQUAL_size_t(5, spi.tx_bytes.size());
  TEST_ASSERT_EQUAL_HEX8(0xEC, spi.tx_bytes[0]); // write bit set
  TEST_ASSERT_EQUAL_HEX8(0x11, spi.tx_bytes[1]);
  TEST_ASSERT_EQUAL_HEX8(0x44, spi.tx_bytes[4]);
}

static void test_spi_read_registers_pipelines_count_plus_one_datagrams(void) {
  FakeSPI spi;
  // Reply k carries the data requested by datagram k-1: datagram 0's reply
  // is stale, datagrams 1..3 return registers 0..2.
  spi.rx_bytes = {
      0x08, 0xDE, 0xAD, 0xBE, 0xEF, // stale
      0x08, 0x00, 0x00, 0x00, 0x01, // register 0
      0x08, 0x00, 0x00, 0x00, 0x02, // register 1
      0x08, 0x00, 0x00, 0x00, 0x03, // register 2
  };
  SpiInterface interface;
  interface.setup(SpiParameters{}.withSpi(&spi).withChipSelectPin(8));

  const uint8_t addresses[3] = {0x35, 0x6F, 0x39};
  uint32_t values[3] = {0, 0, 0};
  interface.readRegisters(addresses, values, 3);

  TEST_ASSERT_EQUAL_HEX32(1UL, values[0]);
  TEST_ASSERT_EQUAL_HEX32(2UL, values[1]);
  TEST_ASSERT_EQUAL_HEX32(3UL, values[2]);
  TEST_ASSERT_EQUAL_size_t(4, spi.buffer_transfers);
  TEST_ASSERT_EQUAL_size_t(20, spi.tx_bytes.size());
  TEST_ASSERT_EQUAL_HEX8(0x35, spi.tx_bytes[0]);
  TEST_ASSERT_EQUAL_HEX8(0x6F, spi.tx_bytes[5]);
  TEST_ASSERT_EQUAL_HEX8(0x39, spi.tx_bytes[10]);
  TEST_ASSERT_EQUAL_HEX8(0x39, spi.tx_bytes[15]); // the last address, again
  TEST_ASSERT_FALSE(interface.consumeDeviceResetObserved());
}

static void test_spi_read_registers_latches_a_reset_flag_in_any_reply(void) {
  FakeSPI spi;
  spi.rx_bytes = {
      0x00, 0, 0, 0, 0, 0x01, 0, 0, 0, 7, 0x00, 0, 0, 0, 8,
  };
  SpiInterface interface;
  interface.setup(SpiParameters{}.withSpi(&spi).withChipSelectPin(8));

  const uint8_t addresses[2] = {0x21, 0x39};
  uint32_t values[2] = {0, 0};
  interface.readRegisters(addresses, values, 2);
  TEST_ASSERT_EQUAL_HEX32(7UL, values[0]);
  TEST_ASSERT_EQUAL_HEX32(8UL, values[1]);
  TEST_ASSERT_TRUE(interface.consumeDeviceResetObserved());
}

int main(int argc, char **argv) {
  UNITY_BEGIN();

  RUN_TEST(test_spi_read_latches_reset_flag_once);
  RUN_TEST(test_spi_datagram_is_one_buffer_transfer);
  RUN_TEST(test_spi_read_registers_pipelines_count_plus_one_datagrams);
  RUN_TEST(test_spi_read_registers_latches_a_reset_flag_in_any_reply);

  return UNITY_END();
}
