// ----------------------------------------------------------------------------
// UartEngine.hpp
//
// Poll-driven UART transaction engine.
//
// This engine is intentionally Arduino-free so it can be unit-tested in
// PlatformIO's native test environment.
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------

#ifndef TMC51X0_UART_ENGINE_HPP
#define TMC51X0_UART_ENGINE_HPP

#include <stddef.h>
#include <stdint.h>

#include "Result.hpp"
#include "UartProtocol.hpp"

namespace tmc51x0
{
class UartEngine
{
public:
  struct Callbacks
  {
    void *ctx{ nullptr };

    // Required byte-stream operations.
    int (*available) (void *ctx){ nullptr };
    int (*read) (void *ctx){ nullptr };
    size_t (*write) (void *ctx,
                     uint8_t b){ nullptr };

    // Optional: may block depending on the platform.
    void (*flush) (void *ctx){ nullptr };

    // Optional: half-duplex direction control.
    // true  -> enable TX / disable RX
    // false -> disable TX / enable RX
    void (*set_tx_enable) (void *ctx,
                           bool enable){ nullptr };
  };

  struct Config
  {
    // If non-zero, used to estimate "TX complete" time in half-duplex mode
    // without calling flush().
    uint32_t baud_rate{ 0 };

    // Maximum time to wait for the reply frame (read transactions).
    uint32_t reply_timeout_us{ 10000 };

    // Optional delay after enabling TX in half-duplex mode.
    uint32_t enable_delay_us{ 10 };

    // Number of retries after the initial attempt (0 => single attempt).
    uint8_t max_retries{ 0 };

    // If baud_rate is 0 and flush is not provided (or should be avoided), an
    // optional conservative guard time (microseconds) to wait after the final
    // byte write before switching to RX.
    uint32_t tx_complete_delay_us{ 0 };

    // Maximum bytes to drain when clearing RX before a transaction.
    uint16_t drain_limit{ 256 };
  };

  UartEngine () = default;

  void
  configure (Callbacks callbacks,
             Config config)
  {
    callbacks_ = callbacks;
    config_ = config;
    resetToIdle_ ();
  }

  bool
  configured () const
  {
    return (callbacks_.available != nullptr)
           && (callbacks_.read != nullptr)
           && (callbacks_.write != nullptr);
  }

  bool
  idle () const
  {
    return state_ == State::Idle;
  }

  bool
  busy () const
  {
    return (state_ != State::Idle) && (state_ != State::Done);
  }

  bool
  resultReady () const
  {
    return state_ == State::Done;
  }

  UartError
  lastError () const
  {
    return last_error_;
  }

  // Start a register read transaction. Completion is reported via resultReady()
  // and takeReadResult().
  Result<void>
  startRead (uint8_t node_address,
             uint8_t register_address)
  {
    Result<void> r;
    if (!configured ())
      {
        r.error = UartError::NotInitialized;
        return r;
      }
    if (!idle ())
      {
        r.error = UartError::Busy;
        return r;
      }

    op_ = Op::Read;
    node_address_ = node_address;
    register_address_ = static_cast<uint8_t> (register_address & 0x7F);

    tx_size_ = uart::READ_REQUEST_SIZE;
    uart::packReadRequest (node_address_, register_address_, tx_);
    tx_index_ = 0;

    rx_size_ = uart::REPLY_SIZE;
    rx_index_ = 0;

    retries_left_ = config_.max_retries;

    last_error_ = UartError::None;
    read_data_ = 0;

    // Timers are initialized on first poll.
    reply_deadline_us_ = 0;
    tx_done_us_ = 0;

    state_ = State::DrainBefore;
    return r;
  }

  // Start a register write transaction (no reply expected).
  Result<void>
  startWrite (uint8_t node_address,
              uint8_t register_address,
              uint32_t data)
  {
    Result<void> r;
    if (!configured ())
      {
        r.error = UartError::NotInitialized;
        return r;
      }
    if (!idle ())
      {
        r.error = UartError::Busy;
        return r;
      }

    op_ = Op::Write;
    node_address_ = node_address;
    register_address_ = static_cast<uint8_t> (register_address & 0x7F);
    write_data_ = data;

    tx_size_ = uart::WRITE_REQUEST_SIZE;
    uart::packWriteRequest (node_address_, register_address_, write_data_, tx_);
    tx_index_ = 0;

    rx_size_ = 0;
    rx_index_ = 0;

    retries_left_ = 0;

    last_error_ = UartError::None;
    read_data_ = 0;

    reply_deadline_us_ = 0;
    tx_done_us_ = 0;

    state_ = State::DrainBefore;
    return r;
  }

  // Drive the state machine forward. Call this regularly from your event-loop
  // (or from a blocking wrapper).
  void
  poll (uint32_t now_us)
  {
    if (state_ == State::Idle || state_ == State::Done)
      {
        return;
      }

    // Limit the amount of work we do per call to avoid pathological loops.
    for (uint8_t steps = 0; steps < 24; ++steps)
      {
        bool progressed = false;
        switch (state_)
          {
          case State::DrainBefore:
            progressed = drainRx_ ();
            // Even if we didn't drain anything, move forward.
            state_ = State::EnableTx;
            progressed = true;
            break;

          case State::EnableTx:
            if (callbacks_.set_tx_enable)
              {
                callbacks_.set_tx_enable (callbacks_.ctx, true);
              }
            state_start_us_ = now_us;
            state_ = State::WaitEnableDelay;
            progressed = true;
            break;

          case State::WaitEnableDelay:
            if ((callbacks_.set_tx_enable == nullptr)
                || (config_.enable_delay_us == 0)
                || elapsedUs_ (now_us, state_start_us_) >= config_.enable_delay_us)
              {
                state_ = State::Send;
                progressed = true;
              }
            break;

          case State::Send:
            while (tx_index_ < tx_size_)
              {
                (void)callbacks_.write (callbacks_.ctx, tx_[tx_index_]);
                ++tx_index_;
              }

            // Establish when it's safe to switch to RX in half-duplex mode.
            if (config_.baud_rate != 0)
              {
                tx_done_us_ = now_us + computeTxTimeUs_ (tx_size_, config_.baud_rate);
              }
            else if (config_.tx_complete_delay_us != 0)
              {
                tx_done_us_ = now_us + config_.tx_complete_delay_us;
              }
            else if (callbacks_.flush)
              {
                // May block depending on the platform.
                callbacks_.flush (callbacks_.ctx);
                tx_done_us_ = now_us;
              }
            else
              {
                tx_done_us_ = now_us;
              }

            state_ = State::WaitTxDone;
            progressed = true;
            break;

          case State::WaitTxDone:
            if (timeAfterEq_ (now_us, tx_done_us_))
              {
                if (callbacks_.set_tx_enable)
                  {
                    // Switch to RX.
                    callbacks_.set_tx_enable (callbacks_.ctx, false);
                  }

                if (op_ == Op::Write)
                  {
                    // No reply expected.
                    finishOk_ ();
                    progressed = true;
                  }
                else
                  {
                    // Start reply timeout when we enter reply wait.
                    reply_deadline_us_ = now_us + config_.reply_timeout_us;
                    state_ = State::WaitReply;
                    progressed = true;
                  }
              }
            break;

          case State::WaitReply:
            if (callbacks_.available (callbacks_.ctx) > 0)
              {
                state_ = State::ReadReply;
                progressed = true;
              }
            else if (timeAfterEq_ (now_us, reply_deadline_us_))
              {
                handleErrorOrRetry_ (UartError::ReplyTimeout, now_us);
                progressed = true;
              }
            break;

          case State::ReadReply:
            while ((rx_index_ < rx_size_)
                   && (callbacks_.available (callbacks_.ctx) > 0))
              {
                int v = callbacks_.read (callbacks_.ctx);
                if (v < 0)
                  {
                    break;
                  }
                rx_[rx_index_] = static_cast<uint8_t> (v);
                ++rx_index_;
              }

            if (rx_index_ >= rx_size_)
              {
                state_ = State::Validate;
                progressed = true;
              }
            else if (timeAfterEq_ (now_us, reply_deadline_us_))
              {
                handleErrorOrRetry_ (UartError::ReplyTimeout, now_us);
                progressed = true;
              }
            break;

          case State::Validate:
            if (!uart::checkSyncByte (rx_[0]))
              {
                handleErrorOrRetry_ (UartError::UnexpectedFrame, now_us);
                progressed = true;
                break;
              }

            if (!uart::checkCrc (rx_, rx_size_))
              {
                handleErrorOrRetry_ (UartError::CrcMismatch, now_us);
                progressed = true;
                break;
              }

            if ((uart::replyNode (rx_) != node_address_)
                || (uart::replyRegister (rx_) != register_address_))
              {
                handleErrorOrRetry_ (UartError::UnexpectedFrame, now_us);
                progressed = true;
                break;
              }

            read_data_ = uart::replyData (rx_);
            finishOk_ ();
            progressed = true;
            break;

          case State::Idle:
          case State::Done:
          default:
            return;
          }

        if (!progressed)
          {
            break;
          }
        if (state_ == State::Done)
          {
            break;
          }
      }
  }

  Result<uint32_t>
  takeReadResult ()
  {
    Result<uint32_t> r;
    if (state_ != State::Done || op_ != Op::Read)
      {
        r.error = UartError::Busy;
        return r;
      }

    r.value = read_data_;
    r.error = last_error_;
    resetToIdle_ ();
    return r;
  }

  Result<void>
  takeWriteResult ()
  {
    Result<void> r;
    if (state_ != State::Done || op_ != Op::Write)
      {
        r.error = UartError::Busy;
        return r;
      }

    r.error = last_error_;
    resetToIdle_ ();
    return r;
  }

private:
  enum class Op
  {
    None,
    Read,
    Write,
  };

  enum class State
  {
    Idle,
    DrainBefore,
    EnableTx,
    WaitEnableDelay,
    Send,
    WaitTxDone,
    WaitReply,
    ReadReply,
    Validate,
    Done,
  };

  Callbacks callbacks_{};
  Config config_{};

  Op op_{ Op::None };
  State state_{ State::Idle };

  uint8_t node_address_{ 0 };
  uint8_t register_address_{ 0 };
  uint32_t write_data_{ 0 };

  uint8_t tx_[uart::WRITE_REQUEST_SIZE] = { 0 };
  size_t tx_size_{ 0 };
  size_t tx_index_{ 0 };

  uint8_t rx_[uart::REPLY_SIZE] = { 0 };
  size_t rx_size_{ 0 };
  size_t rx_index_{ 0 };

  uint32_t state_start_us_{ 0 };
  uint32_t tx_done_us_{ 0 };
  uint32_t reply_deadline_us_{ 0 };

  uint8_t retries_left_{ 0 };

  UartError last_error_{ UartError::None };
  uint32_t read_data_{ 0 };

  void
  resetToIdle_ ()
  {
    op_ = Op::None;
    state_ = State::Idle;
    node_address_ = 0;
    register_address_ = 0;
    write_data_ = 0;
    tx_size_ = 0;
    tx_index_ = 0;
    rx_size_ = 0;
    rx_index_ = 0;
    state_start_us_ = 0;
    tx_done_us_ = 0;
    reply_deadline_us_ = 0;
    retries_left_ = 0;
    last_error_ = UartError::None;
    read_data_ = 0;
  }

  // Unsigned wrap-safe elapsed time.
  static uint32_t
  elapsedUs_ (uint32_t now_us,
              uint32_t start_us)
  {
    return now_us - start_us;
  }

  // Wrap-safe "now >= t" comparison.
  static bool
  timeAfterEq_ (uint32_t now_us,
                uint32_t t_us)
  {
    return static_cast<int32_t> (now_us - t_us) >= 0;
  }

  static uint32_t
  computeTxTimeUs_ (size_t bytes,
                    uint32_t baud_rate)
  {
    // Estimate: 1 start + 8 data + 1 stop = 10 bits per byte.
    const uint64_t bits = static_cast<uint64_t> (bytes) * 10ULL;
    const uint64_t us = (bits * 1000000ULL + static_cast<uint64_t> (baud_rate) - 1ULL)
                        / static_cast<uint64_t> (baud_rate);
    return (us > 0xFFFFFFFFULL) ? 0xFFFFFFFFUL : static_cast<uint32_t> (us);
  }

  bool
  drainRx_ ()
  {
    // Drain up to drain_limit bytes.
    uint16_t drained = 0;
    while ((callbacks_.available (callbacks_.ctx) > 0)
           && (drained < config_.drain_limit))
      {
        (void)callbacks_.read (callbacks_.ctx);
        ++drained;
      }

    // If we hit the drain limit and there are still bytes available, report an
    // error (or let retry logic handle it).
    if ((drained >= config_.drain_limit) && (callbacks_.available (callbacks_.ctx) > 0))
      {
        // Leave remaining bytes in the buffer; the caller may want to inspect.
        last_error_ = UartError::RxGarbage;
      }

    return drained > 0;
  }

  void
  finishOk_ ()
  {
    last_error_ = UartError::None;
    state_ = State::Done;
  }

  void
  handleErrorOrRetry_ (UartError e,
                       uint32_t now_us)
  {
    last_error_ = e;

    // Ensure we are in RX mode before deciding what to do.
    if (callbacks_.set_tx_enable)
      {
        callbacks_.set_tx_enable (callbacks_.ctx, false);
      }

    if (retries_left_ > 0)
      {
        --retries_left_;
        tx_index_ = 0;
        rx_index_ = 0;

        // Reset timing state; we will re-send.
        reply_deadline_us_ = 0;
        tx_done_us_ = 0;
        state_start_us_ = now_us;

        state_ = State::DrainBefore;
        return;
      }

    state_ = State::Done;
  }
};
} // namespace tmc51x0

#endif // TMC51X0_UART_ENGINE_HPP
