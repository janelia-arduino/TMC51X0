#include <unity.h>

#include "tmc_bits.hpp"

static void test_bit_get_set() {
  uint32_t r = 0;

  // Set bit 0
  tmc::bits::Bit<0>::set(r, true);
  TEST_ASSERT_TRUE(tmc::bits::Bit<0>::get(r));
  TEST_ASSERT_EQUAL_UINT32(0x00000001u, r);

  // Set bit 31
  tmc::bits::Bit<31>::set(r, true);
  TEST_ASSERT_TRUE(tmc::bits::Bit<31>::get(r));
  TEST_ASSERT_EQUAL_UINT32(0x80000001u, r);

  // Clear bit 0
  tmc::bits::Bit<0>::set(r, false);
  TEST_ASSERT_FALSE(tmc::bits::Bit<0>::get(r));
  TEST_ASSERT_EQUAL_UINT32(0x80000000u, r);

  // Clear bit 31
  tmc::bits::Bit<31>::set(r, false);
  TEST_ASSERT_FALSE(tmc::bits::Bit<31>::get(r));
  TEST_ASSERT_EQUAL_UINT32(0x00000000u, r);
}

static void test_field_get_set() {
  uint32_t r = 0;

  // 3-bit field at position 4
  using F = tmc::bits::Field<4, 3>;

  F::set(r, 0x5u); // 0b101
  TEST_ASSERT_EQUAL_UINT32(0x00000050u, r);
  TEST_ASSERT_EQUAL_UINT32(0x5u, F::get(r));

  // Over-wide values should be masked.
  F::set(r, 0xFFu);
  TEST_ASSERT_EQUAL_UINT32(0x00000070u, r); // 0b111 at bits [6:4]
  TEST_ASSERT_EQUAL_UINT32(0x7u, F::get(r));

  // Ensure no bleed into neighbors.
  using G = tmc::bits::Field<0, 4>;
  G::set(r, 0xAu);
  TEST_ASSERT_EQUAL_UINT32(0x0000007Au, r);
  TEST_ASSERT_EQUAL_UINT32(0xAu, G::get(r));
  TEST_ASSERT_EQUAL_UINT32(0x7u, F::get(r));
}

int main(int argc, char **argv) {
  (void)argc;
  (void)argv;

  UNITY_BEGIN();
  RUN_TEST(test_bit_get_set);
  RUN_TEST(test_field_get_set);
  return UNITY_END();
}
