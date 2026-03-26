#include <unity.h>

#include <stdint.h>

#define private public
#include "Controller.hpp"
#include "Driver.hpp"
#include "Encoder.hpp"
#include "Registers.hpp"
#include "TMC51X0/Interface.hpp"
#undef private

#include "../../../src/TMC51X0/Registers.cpp"
#include "../../../src/TMC51X0/Driver.cpp"
#include "../../../src/TMC51X0/Controller.cpp"
#include "../../../src/TMC51X0/Encoder.cpp"

using namespace tmc51x0;

namespace {
struct FakeInterface : public Interface {
  bool write_ok{true};
  bool read_ok{true};
  bool use_forced_read_value{false};
  uint32_t forced_read_value{0};
  bool device_reset_observed{false};
  uint32_t register_image[Registers::AddressCount] = {0};

  FakeInterface() {
    interface_mode = SpiMode;
  }

  Result<void> writeRegisterResult(uint8_t register_address,
                                   uint32_t data) override {
    Result<void> r;
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

void initRegisters(Registers& registers, FakeInterface& interface) {
  registers.initialize(interface);
}
}

void setUp() {}

void tearDown() {}

static void test_driver_reinitialize_replays_latest_high_level_writes(void) {
  FakeInterface interface;
  Registers registers;
  initRegisters(registers, interface);

  Driver driver;
  driver.initialize(registers);

  driver.writeRunCurrent(23);
  driver.writeHoldCurrent(11);
  driver.writePwmOffset(37);
  driver.enableCoolStep(4, 9);
  driver.writeEnabledToff(5);

  registers.assumeDeviceReset();
  driver.reinitialize();

  Registers::IholdIrun ihold_irun;
  ihold_irun.raw = interface.register_image[Registers::IholdIrunAddress];
  TEST_ASSERT_EQUAL_UINT8(23, ihold_irun.irun());
  TEST_ASSERT_EQUAL_UINT8(11, ihold_irun.ihold());

  Registers::Pwmconf pwmconf;
  pwmconf.raw = interface.register_image[Registers::PwmconfAddress];
  TEST_ASSERT_EQUAL_UINT8(37, pwmconf.pwm_ofs());

  Registers::Coolconf coolconf;
  coolconf.raw = interface.register_image[Registers::CoolconfAddress];
  TEST_ASSERT_EQUAL_UINT8(4, coolconf.semin());
  TEST_ASSERT_EQUAL_UINT8(9, coolconf.semax());

  Registers::Chopconf chopconf;
  chopconf.raw = interface.register_image[Registers::ChopconfAddress];
  TEST_ASSERT_EQUAL_UINT8(5, chopconf.toff());
}

static void
test_driver_reinitialize_preserves_custom_automatic_current_control_shape(
  void) {
  FakeInterface interface;
  Registers registers;
  initRegisters(registers, interface);

  Driver driver;
  driver.initialize(registers);

  driver.enableAutomaticCurrentControl(false, 7);

  registers.assumeDeviceReset();
  driver.reinitialize();

  Registers::Pwmconf pwmconf;
  pwmconf.raw = interface.register_image[Registers::PwmconfAddress];
  TEST_ASSERT_TRUE(pwmconf.pwm_autoscale());
  TEST_ASSERT_FALSE(pwmconf.pwm_autograd());
  TEST_ASSERT_EQUAL_UINT8(7, pwmconf.pwm_reg());
}

static void
test_controller_reinitialize_replays_latest_configuration_writes(void) {
  FakeInterface interface;
  Registers registers;
  initRegisters(registers, interface);

  Controller controller;
  controller.initialize(registers);

  controller.writeRampMode(PositionMode);
  controller.writeStopMode(SoftMode);
  controller.writeMaxVelocity(12345);
  controller.writeZeroWaitDuration(99);
  controller.enableStallStop();
  controller.writeMinDcStepVelocity(456);

  registers.assumeDeviceReset();
  controller.reinitialize();

  TEST_ASSERT_EQUAL_UINT32(
    PositionMode, interface.register_image[Registers::RampmodeAddress]);
  TEST_ASSERT_EQUAL_UINT32(12345UL,
                           interface.register_image[Registers::VmaxAddress]);
  TEST_ASSERT_EQUAL_UINT32(
    99UL, interface.register_image[Registers::TzerowaitAddress]);
  TEST_ASSERT_EQUAL_UINT32(456UL,
                           interface.register_image[Registers::VdcminAddress]);

  Registers::SwMode sw_mode;
  sw_mode.raw = interface.register_image[Registers::SwModeAddress];
  TEST_ASSERT_EQUAL_UINT8(SoftMode, sw_mode.en_softstop());
  TEST_ASSERT_TRUE(sw_mode.sg_stop());
}

static void test_restore_switch_settings_updates_recovery_state(void) {
  FakeInterface interface;
  Registers registers;
  initRegisters(registers, interface);

  Controller controller;
  controller.initialize(registers);

  SwitchParameters original =
    SwitchParameters{}.withLeftStopEnabled(true).withLatchEncoderEnabled(true);
  SwitchParameters temporary =
    SwitchParameters{}.withRightStopEnabled(true).withSwapLeftRight(true);

  controller.setupSwitches(original);
  controller.cacheSwitchSettings();
  controller.setupSwitches(temporary);
  controller.restoreSwitchSettings();

  registers.assumeDeviceReset();
  controller.reinitialize();

  Registers::SwMode sw_mode;
  sw_mode.raw = interface.register_image[Registers::SwModeAddress];
  TEST_ASSERT_TRUE(sw_mode.stop_l_enable());
  TEST_ASSERT_FALSE(sw_mode.stop_r_enable());
  TEST_ASSERT_FALSE(sw_mode.swap_lr());
  TEST_ASSERT_TRUE(sw_mode.en_latch_encoder());
}

static void
test_encoder_reinitialize_replays_latest_configuration_writes(void) {
  FakeInterface interface;
  Registers registers;
  initRegisters(registers, interface);

  Encoder encoder;
  encoder.initialize(registers);

  encoder.writeFractionalMode(DecimalMode);
  encoder.writeMicrostepsPerPulse(-3, 5000);

  registers.assumeDeviceReset();
  encoder.reinitialize();

  Registers::Encmode encmode;
  encmode.raw = interface.register_image[Registers::EncmodeAddress];
  TEST_ASSERT_TRUE(encmode.enc_sel_decimal());

  Registers::EncConst enc_const;
  enc_const.raw = interface.register_image[Registers::EncConstAddress];
  TEST_ASSERT_EQUAL_INT16(-3, enc_const.integer());
  TEST_ASSERT_EQUAL_UINT16(5000, enc_const.fractional());
}

int main(int argc, char** argv) {
  UNITY_BEGIN();

  RUN_TEST(test_driver_reinitialize_replays_latest_high_level_writes);
  RUN_TEST(
    test_driver_reinitialize_preserves_custom_automatic_current_control_shape);
  RUN_TEST(test_controller_reinitialize_replays_latest_configuration_writes);
  RUN_TEST(test_restore_switch_settings_updates_recovery_state);
  RUN_TEST(test_encoder_reinitialize_replays_latest_configuration_writes);

  return UNITY_END();
}
