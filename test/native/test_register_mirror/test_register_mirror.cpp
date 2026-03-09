#include <unity.h>

#include <stdint.h>

#define private public
#include "Driver.hpp"
#include "Registers.hpp"
#include "TMC51X0/Interface.hpp"
#undef private

// Native tests do not link the Arduino library sources. Pull the small
// implementation units under test directly into this translation unit so the
// regression remains self-contained under `pio test -e native`.
#include "../../../src/TMC51X0/Registers.cpp"
#include "../../../src/TMC51X0/Driver.cpp"

using namespace tmc51x0;

namespace
{
struct FakeInterface : public Interface
{
  bool write_ok{ true };
  bool read_ok{ true };
  bool use_forced_read_value{ false };
  uint8_t last_write_address{ 0 };
  uint8_t last_read_address{ 0 };
  uint32_t last_write_data{ 0 };
  uint32_t forced_read_value{ 0 };
  uint32_t write_calls{ 0 };
  uint32_t read_calls{ 0 };
  uint32_t register_image[Registers::AddressCount] = { 0 };

  FakeInterface ()
  {
    interface_mode = SpiMode;
  }

  Result<void>
  writeRegisterResult (uint8_t register_address,
                       uint32_t data) override
  {
    Result<void> r;
    last_write_address = register_address;
    last_write_data = data;
    ++write_calls;
    if (!write_ok)
      {
        r.error = UartError::ReplyTimeout;
        return r;
      }

    if (register_address < Registers::AddressCount)
      {
        register_image[register_address] = data;
      }
    return r;
  }

  Result<uint32_t>
  readRegisterResult (uint8_t register_address) override
  {
    Result<uint32_t> r;
    last_read_address = register_address;
    ++read_calls;
    if (!read_ok)
      {
        r.error = UartError::CrcMismatch;
        return r;
      }

    if (use_forced_read_value)
      {
        r.value = forced_read_value;
      }
    else if (register_address < Registers::AddressCount)
      {
        r.value = register_image[register_address];
      }
    return r;
  }
};
} // namespace

void
setUp ()
{
}

void
tearDown ()
{
}

static void
initRegisters (Registers &registers,
               FakeInterface &interface)
{
  registers.initialize (interface);
}

static void
test_write_success_updates_the_stored_mirror (void)
{
  FakeInterface interface;
  Registers registers;
  initRegisters (registers, interface);

  const uint32_t value = 0x00061F0AUL;
  registers.write (Registers::IholdIrunAddress, value);

  TEST_ASSERT_EQUAL_UINT32 (1, interface.write_calls);
  TEST_ASSERT_EQUAL_UINT8 (Registers::IholdIrunAddress, interface.last_write_address);
  TEST_ASSERT_EQUAL_HEX32 (value, interface.last_write_data);
  TEST_ASSERT_TRUE (registers.storedValid (Registers::IholdIrunAddress));
  TEST_ASSERT_EQUAL_HEX32 (value, registers.getStored (Registers::IholdIrunAddress));
}

static void
test_failed_write_does_not_overwrite_last_known_good_value (void)
{
  FakeInterface interface;
  Registers registers;
  initRegisters (registers, interface);

  const uint32_t original = registers.getStored (Registers::ChopconfAddress);
  TEST_ASSERT_TRUE (registers.storedValid (Registers::ChopconfAddress));
  TEST_ASSERT_EQUAL_HEX32 (0x10410150UL, original);

  interface.write_ok = false;
  registers.write (Registers::ChopconfAddress, 0xDEADBEEFUL);

  TEST_ASSERT_EQUAL_UINT32 (1, interface.write_calls);
  TEST_ASSERT_TRUE (registers.storedValid (Registers::ChopconfAddress));
  TEST_ASSERT_EQUAL_HEX32 (original, registers.getStored (Registers::ChopconfAddress));
}

static void
test_failed_read_does_not_poison_the_stored_mirror (void)
{
  FakeInterface interface;
  Registers registers;
  initRegisters (registers, interface);

  interface.read_ok = true;
  interface.use_forced_read_value = true;
  interface.forced_read_value = 0xA5A55A5AUL;
  TEST_ASSERT_EQUAL_HEX32 (0xA5A55A5AUL, registers.read (Registers::GconfAddress));
  TEST_ASSERT_TRUE (registers.storedValid (Registers::GconfAddress));
  TEST_ASSERT_EQUAL_HEX32 (0xA5A55A5AUL, registers.getStored (Registers::GconfAddress));

  interface.read_ok = false;
  TEST_ASSERT_EQUAL_HEX32 (0x0UL, registers.read (Registers::GconfAddress));
  TEST_ASSERT_TRUE (registers.storedValid (Registers::GconfAddress));
  TEST_ASSERT_EQUAL_HEX32 (0xA5A55A5AUL, registers.getStored (Registers::GconfAddress));
}

static void
test_assume_device_reset_reseeds_known_defaults_and_invalidates_runtime_only_entries (void)
{
  FakeInterface interface;
  Registers registers;
  initRegisters (registers, interface);

  registers.write (Registers::IholdIrunAddress, 0x00060F0AUL);
  TEST_ASSERT_TRUE (registers.storedValid (Registers::IholdIrunAddress));

  registers.assumeDeviceReset ();

  TEST_ASSERT_FALSE (registers.storedValid (Registers::IholdIrunAddress));
  TEST_ASSERT_EQUAL_HEX32 (0x0UL, registers.getStored (Registers::IholdIrunAddress));

  TEST_ASSERT_TRUE (registers.storedValid (Registers::GconfAddress));
  TEST_ASSERT_EQUAL_HEX32 (0x0UL, registers.getStored (Registers::GconfAddress));

  TEST_ASSERT_TRUE (registers.storedValid (Registers::ChopconfAddress));
  TEST_ASSERT_EQUAL_HEX32 (0x10410150UL, registers.getStored (Registers::ChopconfAddress));

  TEST_ASSERT_TRUE (registers.storedValid (Registers::PwmconfAddress));
  TEST_ASSERT_EQUAL_HEX32 (0xC40C001EUL, registers.getStored (Registers::PwmconfAddress));
}

static void
test_driver_cache_tracks_short_to_ground_protection_enable_state (void)
{
  FakeInterface interface;
  Registers registers;
  initRegisters (registers, interface);

  Driver driver;
  driver.initialize (registers);

  driver.disableShortToGroundProtection ();
  driver.cacheDriverSettings ();
  TEST_ASSERT_FALSE (driver.cached_driver_settings_.short_to_ground_protection_enabled);

  driver.enableShortToGroundProtection ();
  driver.cacheDriverSettings ();
  TEST_ASSERT_TRUE (driver.cached_driver_settings_.short_to_ground_protection_enabled);
}

int
main (int argc,
      char **argv)
{
  UNITY_BEGIN ();

  RUN_TEST (test_write_success_updates_the_stored_mirror);
  RUN_TEST (test_failed_write_does_not_overwrite_last_known_good_value);
  RUN_TEST (test_failed_read_does_not_poison_the_stored_mirror);
  RUN_TEST (test_assume_device_reset_reseeds_known_defaults_and_invalidates_runtime_only_entries);
  RUN_TEST (test_driver_cache_tracks_short_to_ground_protection_enable_state);

  return UNITY_END ();
}
