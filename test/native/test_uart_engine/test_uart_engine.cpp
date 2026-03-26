#include <unity.h>

#include <deque>
#include <stdint.h>
#include <vector>

#include "TMC51X0/UartEngine.hpp"
#include "TMC51X0/UartProtocol.hpp"

using namespace tmc51x0;

struct FakeSerial {
  struct Scheduled {
    uint32_t at_us;
    uint8_t byte;
  };

  std::vector<uint8_t> tx;
  std::deque<uint8_t> rx;
  std::deque<Scheduled> scheduled;

  bool flush_called{false};

  void scheduleBytes(uint32_t at_us,
                     const uint8_t* bytes,
                     size_t len,
                     uint32_t spacing_us = 0) {
    for (size_t i = 0; i < len; ++i) {
      Scheduled s;
      s.at_us = at_us + static_cast<uint32_t>(i * spacing_us);
      s.byte = bytes[i];
      scheduled.push_back(s);
    }
  }

  void update(uint32_t now_us) {
    while (!scheduled.empty()) {
      const Scheduled& s = scheduled.front();
      // Wrap-safe compare: now_us >= s.at_us
      if (static_cast<int32_t>(now_us - s.at_us) < 0) {
        break;
      }
      rx.push_back(s.byte);
      scheduled.pop_front();
    }
  }

  int available() const {
    return static_cast<int>(rx.size());
  }

  int read() {
    if (rx.empty()) {
      return -1;
    }
    uint8_t b = rx.front();
    rx.pop_front();
    return static_cast<int>(b);
  }

  size_t write(uint8_t b) {
    tx.push_back(b);
    return 1;
  }

  void flush() {
    flush_called = true;
  }
};

struct PinSpy {
  std::vector<bool> states;
  void set(bool en) {
    states.push_back(en);
  }
};

struct Context {
  FakeSerial serial;
  PinSpy pin;
};

static int cbAvailable(void* ctx) {
  return static_cast<Context*>(ctx)->serial.available();
}

static int cbRead(void* ctx) {
  return static_cast<Context*>(ctx)->serial.read();
}

static size_t cbWrite(void* ctx, uint8_t b) {
  return static_cast<Context*>(ctx)->serial.write(b);
}

static void cbFlush(void* ctx) {
  static_cast<Context*>(ctx)->serial.flush();
}

static void cbSetTxEnable(void* ctx, bool enable) {
  static_cast<Context*>(ctx)->pin.set(enable);
}

static UartEngine::Callbacks
makeCallbacks(Context& ctx, bool with_flush = true, bool with_pin = true) {
  UartEngine::Callbacks cb;
  cb.ctx = &ctx;
  cb.available = &cbAvailable;
  cb.read = &cbRead;
  cb.write = &cbWrite;
  cb.flush = with_flush ? &cbFlush : nullptr;
  cb.set_tx_enable = with_pin ? &cbSetTxEnable : nullptr;
  return cb;
}

static UartEngine::Config makeConfig() {
  UartEngine::Config cfg;
  cfg.baud_rate = 1000000;
  cfg.reply_timeout_us = 5000;
  cfg.enable_delay_us = 0;
  cfg.max_retries = 0;
  cfg.tx_complete_delay_us = 0;
  cfg.drain_limit = 256;
  return cfg;
}

static void assertUartError(UartError expected, UartError actual) {
  TEST_ASSERT_EQUAL_INT(static_cast<int>(expected), static_cast<int>(actual));
}

static void packReply(uint8_t node_address,
                      uint8_t register_address,
                      uint32_t data,
                      uint8_t out[uart::REPLY_SIZE]) {
  // The engine validates sync, CRC, node, register, and payload bytes only.
  uart::packWriteRequest(node_address, register_address, data, out);
}

static void driveUntilDone(UartEngine& engine,
                           Context& ctx,
                           uint32_t t_end_us,
                           uint32_t step_us) {
  for (uint32_t now = 0; now <= t_end_us; now += step_us) {
    ctx.serial.update(now);
    engine.poll(now);
    if (engine.done()) {
      return;
    }
  }
}

static void
test_start_read_without_configuration_returns_not_initialized(void) {
  UartEngine engine;
  Result<void> start = engine.startRead(0, 0x10);
  TEST_ASSERT_FALSE(start.ok());
  assertUartError(UartError::NotInitialized, start.error);
}

static void test_busy_is_reported_while_transaction_is_in_flight(void) {
  Context ctx;

  UartEngine engine;
  engine.configure(makeCallbacks(ctx), makeConfig());

  Result<void> start = engine.startRead(0, 0x10);
  TEST_ASSERT_TRUE(start.ok());
  TEST_ASSERT_TRUE(engine.busy());

  Result<void> second_start = engine.startWrite(0, 0x20, 0x11223344UL);
  TEST_ASSERT_FALSE(second_start.ok());
  assertUartError(UartError::Busy, second_start.error);
}

static void test_read_success_chunked_reply(void) {
  Context ctx;

  UartEngine engine;
  engine.configure(makeCallbacks(ctx), makeConfig());

  // Pre-fill RX with garbage that should be drained before sending.
  ctx.serial.rx.push_back(0xAA);
  ctx.serial.rx.push_back(0xBB);

  Result<void> start = engine.startRead(0, 0x10);
  TEST_ASSERT_TRUE(start.ok());

  // Schedule a valid reply in two chunks.
  uint8_t reply[uart::REPLY_SIZE];
  packReply(0, 0x10, 0x12345678UL, reply);
  ctx.serial.scheduleBytes(200, reply, 4);
  ctx.serial.scheduleBytes(400, reply + 4, 4);

  driveUntilDone(engine, ctx, 2000, 50);

  TEST_ASSERT_TRUE(engine.resultReady());
  TEST_ASSERT_TRUE(engine.done());

  Result<uint32_t> r = engine.takeReadResult();
  TEST_ASSERT_TRUE(r.ok());
  TEST_ASSERT_EQUAL_UINT32(0x12345678UL, r.value);

  // Verify the request bytes.
  uint8_t req[uart::READ_REQUEST_SIZE];
  uart::packReadRequest(0, 0x10, req);
  TEST_ASSERT_EQUAL_UINT8(uart::READ_REQUEST_SIZE, ctx.serial.tx.size());
  TEST_ASSERT_EQUAL_UINT8_ARRAY(
    req, ctx.serial.tx.data(), uart::READ_REQUEST_SIZE);

  // Verify half-duplex direction toggles (TX then RX).
  TEST_ASSERT_EQUAL(2, ctx.pin.states.size());
  TEST_ASSERT_TRUE(ctx.pin.states[0]);
  TEST_ASSERT_FALSE(ctx.pin.states[1]);
}

static void test_read_timeout_then_retry_succeeds(void) {
  Context ctx;

  UartEngine engine;
  UartEngine::Config cfg = makeConfig();
  cfg.reply_timeout_us = 100;
  cfg.max_retries = 1;
  engine.configure(makeCallbacks(ctx), cfg);

  Result<void> start = engine.startRead(0, 0x22);
  TEST_ASSERT_TRUE(start.ok());

  // Only provide a reply late enough that the first attempt times out, but the
  // second attempt succeeds.
  uint8_t reply[uart::REPLY_SIZE];
  packReply(0, 0x22, 0xA5A55A5AUL, reply);
  ctx.serial.scheduleBytes(250, reply, uart::REPLY_SIZE);

  driveUntilDone(engine, ctx, 2000, 50);

  TEST_ASSERT_TRUE(engine.resultReady());
  Result<uint32_t> r = engine.takeReadResult();
  TEST_ASSERT_TRUE(r.ok());
  TEST_ASSERT_EQUAL_UINT32(0xA5A55A5AUL, r.value);

  // Verify that the request was sent twice (due to a retry).
  uint8_t req[uart::READ_REQUEST_SIZE];
  uart::packReadRequest(0, 0x22, req);
  TEST_ASSERT_EQUAL_UINT8(uart::READ_REQUEST_SIZE * 2, ctx.serial.tx.size());
  TEST_ASSERT_EQUAL_UINT8_ARRAY(
    req, ctx.serial.tx.data(), uart::READ_REQUEST_SIZE);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(req,
                                ctx.serial.tx.data() + uart::READ_REQUEST_SIZE,
                                uart::READ_REQUEST_SIZE);
}

static void test_crc_mismatch_reports_explicit_error_without_retry(void) {
  Context ctx;

  UartEngine engine;
  engine.configure(makeCallbacks(ctx), makeConfig());

  Result<void> start = engine.startRead(0, 0x33);
  TEST_ASSERT_TRUE(start.ok());

  uint8_t reply[uart::REPLY_SIZE];
  packReply(0, 0x33, 0x0BADBEEFUL, reply);
  reply[uart::REPLY_SIZE - 1] ^= 0xFF; // corrupt CRC
  ctx.serial.scheduleBytes(200, reply, uart::REPLY_SIZE);

  driveUntilDone(engine, ctx, 2000, 50);

  TEST_ASSERT_TRUE(engine.resultReady());
  Result<uint32_t> r = engine.takeReadResult();
  TEST_ASSERT_FALSE(r.ok());
  assertUartError(UartError::CrcMismatch, r.error);
  TEST_ASSERT_EQUAL_UINT32(0UL, r.value);
}

static void test_unexpected_frame_reports_error(void) {
  Context ctx;

  UartEngine engine;
  engine.configure(makeCallbacks(ctx), makeConfig());

  Result<void> start = engine.startRead(0, 0x44);
  TEST_ASSERT_TRUE(start.ok());

  uint8_t reply[uart::REPLY_SIZE];
  packReply(0, 0x45, 0xCAFEBABEUL, reply); // wrong register
  ctx.serial.scheduleBytes(200, reply, uart::REPLY_SIZE);

  driveUntilDone(engine, ctx, 2000, 50);

  TEST_ASSERT_TRUE(engine.resultReady());
  Result<uint32_t> r = engine.takeReadResult();
  TEST_ASSERT_FALSE(r.ok());
  assertUartError(UartError::UnexpectedFrame, r.error);
}

static void test_rx_garbage_overflow_reports_explicit_error(void) {
  Context ctx;

  UartEngine engine;
  UartEngine::Config cfg = makeConfig();
  cfg.drain_limit = 2;
  engine.configure(makeCallbacks(ctx, true, false), cfg);

  ctx.serial.rx.push_back(0xAA);
  ctx.serial.rx.push_back(0xBB);
  ctx.serial.rx.push_back(0xCC);

  Result<void> start = engine.startRead(0, 0x55);
  TEST_ASSERT_TRUE(start.ok());

  driveUntilDone(engine, ctx, 100, 10);

  TEST_ASSERT_TRUE(engine.resultReady());
  Result<uint32_t> r = engine.takeReadResult();
  TEST_ASSERT_FALSE(r.ok());
  assertUartError(UartError::RxGarbage, r.error);
  TEST_ASSERT_EQUAL_UINT8(0, ctx.serial.tx.size());
}

static void test_write_success_completes_without_reply(void) {
  Context ctx;

  UartEngine engine;
  UartEngine::Config cfg = makeConfig();
  cfg.baud_rate = 0;
  cfg.tx_complete_delay_us = 0;
  engine.configure(makeCallbacks(ctx, true, true), cfg);

  Result<void> start = engine.startWrite(0, 0x12, 0x11223344UL);
  TEST_ASSERT_TRUE(start.ok());

  driveUntilDone(engine, ctx, 100, 10);

  TEST_ASSERT_TRUE(engine.resultReady());
  Result<void> r = engine.takeWriteResult();
  TEST_ASSERT_TRUE(r.ok());
  TEST_ASSERT_TRUE(ctx.serial.flush_called);

  uint8_t req[uart::WRITE_REQUEST_SIZE];
  uart::packWriteRequest(0, 0x12, 0x11223344UL, req);
  TEST_ASSERT_EQUAL_UINT8(uart::WRITE_REQUEST_SIZE, ctx.serial.tx.size());
  TEST_ASSERT_EQUAL_UINT8_ARRAY(
    req, ctx.serial.tx.data(), uart::WRITE_REQUEST_SIZE);

  TEST_ASSERT_EQUAL(2, ctx.pin.states.size());
  TEST_ASSERT_TRUE(ctx.pin.states[0]);
  TEST_ASSERT_FALSE(ctx.pin.states[1]);
}

void setUp(void) {}

void tearDown(void) {}

int main(int argc, char** argv) {
  (void)argc;
  (void)argv;

  UNITY_BEGIN();
  RUN_TEST(test_start_read_without_configuration_returns_not_initialized);
  RUN_TEST(test_busy_is_reported_while_transaction_is_in_flight);
  RUN_TEST(test_read_success_chunked_reply);
  RUN_TEST(test_read_timeout_then_retry_succeeds);
  RUN_TEST(test_crc_mismatch_reports_explicit_error_without_retry);
  RUN_TEST(test_unexpected_frame_reports_error);
  RUN_TEST(test_rx_garbage_overflow_reports_explicit_error);
  RUN_TEST(test_write_success_completes_without_reply);
  return UNITY_END();
}
