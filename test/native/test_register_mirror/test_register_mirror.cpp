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
#include "../../../src/TMC51X0/Driver.cpp"
#include "../../../src/TMC51X0/Registers.cpp"

using namespace tmc51x0;

namespace {
struct FakeInterface : public Interface {
  bool write_ok{true};
  bool read_ok{true};
  bool use_forced_read_value{false};
  uint8_t last_write_address{0};
  uint8_t last_read_address{0};
  uint32_t last_write_data{0};
  uint32_t forced_read_value{0};
  uint32_t write_calls{0};
  uint32_t read_calls{0};
  bool device_reset_observed{false};
  uint32_t register_image[Registers::AddressCount] = {0};

  FakeInterface() { interface_mode = SpiMode; }

  Result<void> writeRegisterResult(uint8_t register_address,
                                   uint32_t data) override {
    Result<void> r;
    last_write_address = register_address;
    last_write_data = data;
    ++write_calls;
    if (!write_ok) {
      r.error = UartError::ReplyTimeout;
      return r;
    }

    if (register_address < Registers::AddressCount) {
      register_image[register_address] = data;
    }
    return r;
  }

  Result<uint32_t> readRegisterResult(uint8_t register_address) override {
    Result<uint32_t> r;
    last_read_address = register_address;
    ++read_calls;
    if (!read_ok) {
      r.error = UartError::CrcMismatch;
      return r;
    }

    if (use_forced_read_value) {
      r.value = forced_read_value;
    } else if (register_address < Registers::AddressCount) {
      r.value = register_image[register_address];
    }
    return r;
  }

  bool consumeDeviceResetObserved() override {
    const bool observed = device_reset_observed;
    device_reset_observed = false;
    return observed;
  }
};
} // namespace

void setUp() {}

void tearDown() {}

static void initRegisters(Registers &registers, FakeInterface &interface) {
  registers.initialize(interface);
}

static void test_write_success_updates_the_stored_mirror(void) {
  FakeInterface interface;
  Registers registers;
  initRegisters(registers, interface);

  const uint32_t value = 0x00061F0AUL;
  registers.write(Registers::IholdIrunAddress, value);

  TEST_ASSERT_EQUAL_UINT32(1, interface.write_calls);
  TEST_ASSERT_EQUAL_UINT8(Registers::IholdIrunAddress,
                          interface.last_write_address);
  TEST_ASSERT_EQUAL_HEX32(value, interface.last_write_data);
  TEST_ASSERT_TRUE(registers.storedValid(Registers::IholdIrunAddress));
  TEST_ASSERT_EQUAL_HEX32(value,
                          registers.getStored(Registers::IholdIrunAddress));
}

static void test_failed_write_does_not_overwrite_last_known_good_value(void) {
  FakeInterface interface;
  Registers registers;
  initRegisters(registers, interface);

  const uint32_t original = registers.getStored(Registers::ChopconfAddress);
  TEST_ASSERT_TRUE(registers.storedValid(Registers::ChopconfAddress));
  TEST_ASSERT_EQUAL_HEX32(0x10410150UL, original);

  interface.write_ok = false;
  registers.write(Registers::ChopconfAddress, 0xDEADBEEFUL);

  TEST_ASSERT_EQUAL_UINT32(1, interface.write_calls);
  TEST_ASSERT_TRUE(registers.storedValid(Registers::ChopconfAddress));
  TEST_ASSERT_EQUAL_HEX32(original,
                          registers.getStored(Registers::ChopconfAddress));
}

static void test_failed_read_does_not_poison_the_stored_mirror(void) {
  FakeInterface interface;
  Registers registers;
  initRegisters(registers, interface);

  interface.read_ok = true;
  interface.use_forced_read_value = true;
  interface.forced_read_value = 0xA5A55A5AUL;
  TEST_ASSERT_EQUAL_HEX32(0xA5A55A5AUL,
                          registers.read(Registers::GconfAddress));
  TEST_ASSERT_TRUE(registers.storedValid(Registers::GconfAddress));
  TEST_ASSERT_EQUAL_HEX32(0xA5A55A5AUL,
                          registers.getStored(Registers::GconfAddress));

  interface.read_ok = false;
  TEST_ASSERT_EQUAL_HEX32(0x0UL, registers.read(Registers::GconfAddress));
  TEST_ASSERT_TRUE(registers.storedValid(Registers::GconfAddress));
  TEST_ASSERT_EQUAL_HEX32(0xA5A55A5AUL,
                          registers.getStored(Registers::GconfAddress));
}

static void
test_assume_device_reset_reseeds_known_defaults_and_invalidates_runtime_only_entries(
    void) {
  FakeInterface interface;
  Registers registers;
  initRegisters(registers, interface);

  registers.write(Registers::IholdIrunAddress, 0x00060F0AUL);
  TEST_ASSERT_TRUE(registers.storedValid(Registers::IholdIrunAddress));

  registers.assumeDeviceReset();

  TEST_ASSERT_FALSE(registers.storedValid(Registers::IholdIrunAddress));
  TEST_ASSERT_EQUAL_HEX32(0x0UL,
                          registers.getStored(Registers::IholdIrunAddress));

  TEST_ASSERT_TRUE(registers.storedValid(Registers::GconfAddress));
  TEST_ASSERT_EQUAL_HEX32(0x0UL, registers.getStored(Registers::GconfAddress));

  TEST_ASSERT_TRUE(registers.storedValid(Registers::ChopconfAddress));
  TEST_ASSERT_EQUAL_HEX32(0x10410150UL,
                          registers.getStored(Registers::ChopconfAddress));

  TEST_ASSERT_TRUE(registers.storedValid(Registers::PwmconfAddress));
  TEST_ASSERT_EQUAL_HEX32(0xC40C001EUL,
                          registers.getStored(Registers::PwmconfAddress));
}

static void
test_driver_cache_tracks_short_to_ground_protection_enable_state(void) {
  FakeInterface interface;
  Registers registers;
  initRegisters(registers, interface);

  Driver driver;
  driver.initialize(registers);

  driver.disableShortToGroundProtection();
  driver.cacheDriverSettings();
  TEST_ASSERT_FALSE(
      driver.cached_driver_settings_.short_to_ground_protection_enabled);

  driver.enableShortToGroundProtection();
  driver.cacheDriverSettings();
  TEST_ASSERT_TRUE(
      driver.cached_driver_settings_.short_to_ground_protection_enabled);
}

static void test_chip_specific_reset_defaults_seed_pwmconf(void) {
  FakeInterface interface;
  Registers registers;
  initRegisters(registers, interface);

  registers.setDeviceModel(Registers::DeviceModel::TMC5130A);
  registers.assumeDeviceReset();
  TEST_ASSERT_EQUAL_HEX32(0x00050480UL,
                          registers.getStored(Registers::PwmconfAddress));
  TEST_ASSERT_EQUAL_INT(
      static_cast<int>(Registers::MirrorConfidence::ResetDefault),
      static_cast<int>(registers.storedConfidence(Registers::PwmconfAddress)));

  registers.setDeviceModel(Registers::DeviceModel::TMC5160A);
  registers.assumeDeviceReset();
  TEST_ASSERT_EQUAL_HEX32(0xC40C001EUL,
                          registers.getStored(Registers::PwmconfAddress));
}

static void test_transport_reset_marks_the_mirror_for_recovery(void) {
  FakeInterface interface;
  Registers registers;
  initRegisters(registers, interface);

  interface.device_reset_observed = true;
  const uint32_t value = 0x00060F0AUL;
  registers.write(Registers::IholdIrunAddress, value);

  TEST_ASSERT_TRUE(registers.resyncRequired());
  TEST_ASSERT_TRUE(registers.storedValid(Registers::IholdIrunAddress));
  TEST_ASSERT_EQUAL_HEX32(value,
                          registers.getStored(Registers::IholdIrunAddress));
  TEST_ASSERT_EQUAL_INT(
      static_cast<int>(Registers::MirrorConfidence::AssumedWritten),
      static_cast<int>(
          registers.storedConfidence(Registers::IholdIrunAddress)));
  TEST_ASSERT_TRUE(registers.storedValid(Registers::PwmconfAddress));
  TEST_ASSERT_EQUAL_INT(
      static_cast<int>(Registers::MirrorConfidence::ResetDefault),
      static_cast<int>(registers.storedConfidence(Registers::PwmconfAddress)));
}

static void test_resync_readable_configuration_refreshes_verified_values(void) {
  FakeInterface interface;
  Registers registers;
  initRegisters(registers, interface);

  interface.register_image[Registers::GconfAddress] = 0x00000010UL;
  interface.register_image[Registers::FactoryConfAddress] = 0x0000001FUL;
  interface.register_image[Registers::RampmodeAddress] = 0x00000002UL;
  interface.register_image[Registers::SwModeAddress] = 0x00001234UL;
  interface.register_image[Registers::EncmodeAddress] = 0x00005678UL;
  interface.register_image[Registers::ChopconfAddress] = 0xABCDEF12UL;

  registers.notePossibleDrift();
  TEST_ASSERT_TRUE(registers.resyncReadableConfiguration());
  TEST_ASSERT_TRUE(registers.resyncRequired());

  TEST_ASSERT_EQUAL_HEX32(0x00000010UL,
                          registers.getStored(Registers::GconfAddress));
  TEST_ASSERT_EQUAL_HEX32(0xABCDEF12UL,
                          registers.getStored(Registers::ChopconfAddress));
  TEST_ASSERT_EQUAL_INT(
      static_cast<int>(Registers::MirrorConfidence::ReadVerified),
      static_cast<int>(registers.storedConfidence(Registers::GconfAddress)));
  TEST_ASSERT_EQUAL_INT(
      static_cast<int>(Registers::MirrorConfidence::ReadVerified),
      static_cast<int>(registers.storedConfidence(Registers::ChopconfAddress)));
}

// CHOPCONF's reset default differs between the parts in a way that is not
// cosmetic: bits 23:20 are TPFD on the TMC5160 and SYNC on the TMC5130, where a
// non-zero value enables chopSync. Seeding the TMC5160 value for a TMC5130
// therefore enabled chopSync by accident, because every high-level CHOPCONF
// write is a read-modify-write of this mirror.
static void test_chip_specific_reset_defaults_seed_chopconf(void) {
  FakeInterface interface;

  Registers tmc5130;
  tmc5130.setDeviceModel(Registers::DeviceModel::TMC5130A);
  initRegisters(tmc5130, interface);
  TEST_ASSERT_EQUAL_HEX32(0x10010150UL,
                          tmc5130.getStored(Registers::ChopconfAddress));

  Registers tmc5160;
  tmc5160.setDeviceModel(Registers::DeviceModel::TMC5160A);
  initRegisters(tmc5160, interface);
  TEST_ASSERT_EQUAL_HEX32(0x10410150UL,
                          tmc5160.getStored(Registers::ChopconfAddress));

  // The bits that differ, stated as bits: SYNC/TPFD must be clear for a
  // TMC5130 and 4 for a TMC5160, and the chopper defaults must match.
  Registers::Chopconf as_5130, as_5160;
  as_5130.raw = tmc5130.getStored(Registers::ChopconfAddress);
  as_5160.raw = tmc5160.getStored(Registers::ChopconfAddress);
  TEST_ASSERT_EQUAL_UINT8(0, as_5130.tpfd());
  TEST_ASSERT_EQUAL_UINT8(4, as_5160.tpfd());
  TEST_ASSERT_EQUAL_UINT8(as_5160.hstart(), as_5130.hstart());
  TEST_ASSERT_EQUAL_UINT8(as_5160.hend(), as_5130.hend());
  TEST_ASSERT_EQUAL_UINT8(as_5160.tbl(), as_5130.tbl());
  TEST_ASSERT_FALSE(as_5130.vsense());
}

// setupSpi() seeds the mirror before it identifies the part, so a caller that
// does not declare the model gets the Unknown seed first and the identity read
// only lands afterwards. setDeviceModel() has to correct the model-dependent
// defaults when that happens.
static void
test_learning_the_device_model_reseeds_model_dependent_defaults(void) {
  FakeInterface interface;
  Registers registers;
  initRegisters(registers, interface);

  // Unknown seeds the TMC5160 values, matching pwmconfResetDefault.
  TEST_ASSERT_EQUAL_HEX32(0x10410150UL,
                          registers.getStored(Registers::ChopconfAddress));

  registers.setDeviceModel(Registers::DeviceModel::TMC5130A);

  TEST_ASSERT_EQUAL_HEX32(0x10010150UL,
                          registers.getStored(Registers::ChopconfAddress));
  TEST_ASSERT_EQUAL_HEX32(0x00050480UL,
                          registers.getStored(Registers::PwmconfAddress));
}

// Anything the caller wrote, or anything read back from the chip, is better
// information than a reset default and must survive learning the model.
static void test_reseeding_does_not_clobber_better_information(void) {
  FakeInterface interface;
  Registers registers;
  initRegisters(registers, interface);

  registers.write(Registers::ChopconfAddress, 0x000300C3UL);
  TEST_ASSERT_EQUAL_INT(
      static_cast<int>(Registers::MirrorConfidence::AssumedWritten),
      static_cast<int>(registers.storedConfidence(Registers::ChopconfAddress)));

  registers.setDeviceModel(Registers::DeviceModel::TMC5130A);

  TEST_ASSERT_EQUAL_HEX32(0x000300C3UL,
                          registers.getStored(Registers::ChopconfAddress));
}

// CHOPCONF bit 17 is `vsense` on the TMC5130 and reserved on the TMC5160, and
// on a board with low-value sense resistors it scales every coil current by
// about 1.8x. It was previously unreachable through the API.
static void test_sense_voltage_mode_writes_vsense_on_tmc5130_only(void) {
  FakeInterface interface;
  Registers registers;
  registers.setDeviceModel(Registers::DeviceModel::TMC5130A);
  initRegisters(registers, interface);
  Driver driver;
  driver.initialize(registers);

  driver.writeSenseVoltageMode(LowSenseVoltageMode);
  Registers::Chopconf chopconf;
  chopconf.raw = registers.getStored(Registers::ChopconfAddress);
  TEST_ASSERT_TRUE(chopconf.vsense());

  driver.writeSenseVoltageMode(HighSenseVoltageMode);
  chopconf.raw = registers.getStored(Registers::ChopconfAddress);
  TEST_ASSERT_FALSE(chopconf.vsense());
}

static void test_sense_voltage_mode_is_a_no_op_on_tmc5160(void) {
  FakeInterface interface;
  Registers registers;
  registers.setDeviceModel(Registers::DeviceModel::TMC5160A);
  initRegisters(registers, interface);
  Driver driver;
  driver.initialize(registers);

  const uint32_t before = registers.getStored(Registers::ChopconfAddress);
  driver.writeSenseVoltageMode(LowSenseVoltageMode);

  // Bit 17 is reserved on this part, so the request is recorded but not
  // written: the register must be untouched.
  TEST_ASSERT_EQUAL_HEX32(before,
                          registers.getStored(Registers::ChopconfAddress));
}

// The bit has to survive a full driver.setup(), or it would be silently lost
// on every reconfiguration and on every reset-recovery replay.
static void test_sense_voltage_mode_survives_driver_setup(void) {
  FakeInterface interface;
  Registers registers;
  registers.setDeviceModel(Registers::DeviceModel::TMC5130A);
  initRegisters(registers, interface);
  Driver driver;
  driver.initialize(registers);

  driver.setup(DriverParameters{}.withSenseVoltageMode(LowSenseVoltageMode));

  Registers::Chopconf chopconf;
  chopconf.raw = registers.getStored(Registers::ChopconfAddress);
  TEST_ASSERT_TRUE(chopconf.vsense());

  driver.reinitialize();
  chopconf.raw = registers.getStored(Registers::ChopconfAddress);
  TEST_ASSERT_TRUE(chopconf.vsense());
}

int main(int argc, char **argv) {
  UNITY_BEGIN();

  RUN_TEST(test_write_success_updates_the_stored_mirror);
  RUN_TEST(test_failed_write_does_not_overwrite_last_known_good_value);
  RUN_TEST(test_failed_read_does_not_poison_the_stored_mirror);
  RUN_TEST(
      test_assume_device_reset_reseeds_known_defaults_and_invalidates_runtime_only_entries);
  RUN_TEST(test_driver_cache_tracks_short_to_ground_protection_enable_state);
  RUN_TEST(test_chip_specific_reset_defaults_seed_pwmconf);
  RUN_TEST(test_chip_specific_reset_defaults_seed_chopconf);
  RUN_TEST(test_learning_the_device_model_reseeds_model_dependent_defaults);
  RUN_TEST(test_reseeding_does_not_clobber_better_information);
  RUN_TEST(test_sense_voltage_mode_writes_vsense_on_tmc5130_only);
  RUN_TEST(test_sense_voltage_mode_is_a_no_op_on_tmc5160);
  RUN_TEST(test_sense_voltage_mode_survives_driver_setup);
  RUN_TEST(test_transport_reset_marks_the_mirror_for_recovery);
  RUN_TEST(test_resync_readable_configuration_refreshes_verified_values);

  return UNITY_END();
}
