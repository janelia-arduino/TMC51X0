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
  size_t index{0};

  uint8_t transfer(uint8_t) override {
    if (index < rx_bytes.size()) {
      return rx_bytes[index++];
    }
    return 0;
  }
};
} // namespace

void setUp() {}

void tearDown() {}

static void test_spi_read_latches_reset_flag_once(void) {
  FakeSPI spi;
  spi.rx_bytes = {
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x01,
    0x11,
    0x22,
    0x33,
    0x44,
  };

  SpiInterface interface;
  interface.setup(SpiParameters{}.withSpi(&spi).withChipSelectPin(8));

  TEST_ASSERT_EQUAL_HEX32(0x11223344UL, interface.readRegister(0x6C));
  TEST_ASSERT_TRUE(interface.consumeDeviceResetObserved());
  TEST_ASSERT_FALSE(interface.consumeDeviceResetObserved());
}

int main(int argc, char** argv) {
  UNITY_BEGIN();

  RUN_TEST(test_spi_read_latches_reset_flag_once);

  return UNITY_END();
}
