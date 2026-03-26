#include <unity.h>

#include <stdint.h>

#define private public
#include "TMC51X0.hpp"
#include "TMC51X0/Interface.hpp"
#undef private

#include "../../../src/TMC51X0/Registers.cpp"
#include "../../../src/TMC51X0/Driver.cpp"
#include "../../../src/TMC51X0/Controller.cpp"
#include "../../../src/TMC51X0/Encoder.cpp"
#include "../../../src/TMC51X0/Printer.cpp"
#include "../../../src/TMC51X0/SpiInterface.cpp"
#include "../../../src/TMC51X0/UartInterface.cpp"
#include "../../../src/TMC51X0/TMC51X0.cpp"

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
      r.error = UartError::ReplyTimeout;
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

void initStepper(TMC51X0& stepper, FakeInterface& interface) {
  stepper.registers.initialize(interface);
  stepper.registers.setDeviceModel(Registers::DeviceModel::TMC5130A);
  stepper.initialize();
}
} // namespace

void setUp() {}

void tearDown() {}

static void test_homed_requires_confirmed_stall_for_stall_home(void) {
  FakeInterface interface;
  TMC51X0 stepper;
  initStepper(stepper, interface);

  stepper.beginHomeToStall(HomeParameters{}, StallParameters{});
  interface.register_image[Registers::VactualAddress] = 0;
  interface.register_image[Registers::RampStatAddress] = 0;

  TEST_ASSERT_FALSE(stepper.homed());
  TEST_ASSERT_TRUE(stepper.homeFailed());
}

static void test_homed_accepts_stall_stop_event_for_stall_home(void) {
  FakeInterface interface;
  TMC51X0 stepper;
  initStepper(stepper, interface);

  stepper.beginHomeToStall(HomeParameters{}, StallParameters{});
  interface.register_image[Registers::VactualAddress] = 0;
  Registers::RampStat ramp_stat;
  ramp_stat.event_stop_sg(true);
  interface.register_image[Registers::RampStatAddress] = ramp_stat.raw;

  TEST_ASSERT_TRUE(stepper.homed());
  TEST_ASSERT_FALSE(stepper.homeFailed());
}

static void test_read_health_status_reports_faults_and_resync_need(void) {
  FakeInterface interface;
  TMC51X0 stepper;
  initStepper(stepper, interface);

  Registers::Gstat gstat;
  gstat.reset(true);
  gstat.drv_err(true);
  gstat.uv_cp(true);
  interface.register_image[Registers::GstatAddress] = gstat.raw;

  Registers::Ioin ioin;
  ioin.version(Registers::VERSION_TMC5130);
  interface.register_image[Registers::IoinAddress] = ioin.raw;

  HealthStatus status = stepper.readHealthStatus();

  TEST_ASSERT_TRUE(status.communication_ok);
  TEST_ASSERT_TRUE(status.reset);
  TEST_ASSERT_TRUE(status.driver_error);
  TEST_ASSERT_TRUE(status.charge_pump_undervoltage);
  TEST_ASSERT_TRUE(status.mirror_resync_required);
}

static void test_recover_if_unhealthy_marks_drift_and_recovers(void) {
  FakeInterface interface;
  TMC51X0 stepper;
  initStepper(stepper, interface);

  Registers::Ioin ioin;
  ioin.version(Registers::VERSION_TMC5130);
  interface.register_image[Registers::IoinAddress] = ioin.raw;

  Registers::Gstat gstat;
  gstat.drv_err(true);
  interface.register_image[Registers::GstatAddress] = gstat.raw;

  TEST_ASSERT_TRUE(stepper.recoverIfUnhealthy());
  TEST_ASSERT_FALSE(stepper.mirrorResyncRequired());
}

int main(int argc, char** argv) {
  UNITY_BEGIN();

  RUN_TEST(test_homed_requires_confirmed_stall_for_stall_home);
  RUN_TEST(test_homed_accepts_stall_stop_event_for_stall_home);
  RUN_TEST(test_read_health_status_reports_faults_and_resync_need);
  RUN_TEST(test_recover_if_unhealthy_marks_drift_and_recovers);

  return UNITY_END();
}
