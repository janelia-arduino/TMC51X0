# UART guide

This library now treats the poll-driven UART engine as the single source of truth for both blocking and non-blocking register transactions.

## Public UART surfaces

The existing blocking API remains available:

- `writeRegister(register_address, data)`
- `readRegister(register_address)`
- `writeRegisterResult(register_address, data)`
- `readRegisterResult(register_address)`
- `getLastUartError()`

For event loops and QP-style integrations, use the non-blocking API through either:

- `TMC51X0::uartBus()`
- `TMC51X0::uartInterface()`

`uartBus()` is the preferred family-style alias. `uartInterface()` is retained for compatibility with existing code.

The non-blocking surface is:

- `startRead(register_address)` / `startReadRegister(register_address)`
- `startWrite(register_address, data)` / `startWriteRegister(register_address, data)`
- `poll(now_us)` or `poll()`
- `busy()`
- `done()` or `resultReady()`
- `takeReadResult()`
- `takeWriteResult()`

## Error handling

UART operations report typed failures via `tmc51x0::Result<T>` and `tmc51x0::UartError`.

Current UART errors are:

- `None`
- `Busy`
- `NotInitialized`
- `ReplyTimeout`
- `CrcMismatch`
- `UnexpectedFrame`
- `RxGarbage`

The blocking wrappers call into the same engine as the non-blocking path and preserve the last UART error state.

## Optional TX/RX-enable pin behavior

`UartParameters::enable_txrx_pin` is optional.

- If you do not configure it, the library does not touch any direction-control pin.
- If you do configure it, the UART layer toggles it around each transaction in half-duplex mode.

This keeps transport ownership explicit and avoids hidden pin manipulation by default.

## Timing configuration

The UART engine can avoid blocking `flush()` behavior when timing is configured explicitly.

Recommended configuration for half-duplex links:

```cpp
const auto uart_parameters = tmc51x0::UartParameters{}
                                 .withUart(&Serial2)
                                 .withEnableTxRxPin(14)
                                 .withBaudRate(115200)
                                 .withReplyTimeoutUs(2000)
                                 .withMaxRetries(1)
                                 .withDrainLimit(256);
```

Notes:

- `withBaudRate(...)` lets the engine estimate when transmission is complete.
- `withTxCompleteDelayUs(...)` is available if the baud rate is not known.
- `withDrainLimit(...)` bounds how many stale RX bytes are discarded before a transaction begins.
- If stale bytes exceed the configured drain limit, the transaction fails with `UartError::RxGarbage` (or retries if configured).

## Non-blocking example

See [`examples/UART/NonBlockingReadRegister`](../examples/UART/NonBlockingReadRegister) for a complete sketch.

The typical loop shape is:

```cpp
auto &bus = stepper.uartBus();
bus.poll();

if (!in_flight) {
  auto start = bus.startRead(tmc51x0::Registers::GconfAddress);
  if (start.ok()) {
    in_flight = true;
  }
}

if (in_flight && bus.done()) {
  auto result = bus.takeReadResult();
  in_flight = false;
}
```
