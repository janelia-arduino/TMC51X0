# Migration notes

This update keeps the existing UART API working while aligning the public surface with the intended family conventions.

## What changed

- `tmc51x0::UartError` now has an explicit `uint8_t` underlying type.
- `TMC51X0::uartBus()` was added as the preferred family-style alias for async UART access.
- `TMC51X0::uartInterface()` remains available for compatibility.
- `UartInterface` now exposes family-style async aliases:
  - `startRead(...)`
  - `startWrite(...)`
  - `done()`
  - `lastError()`
- `UartParameters` now exposes `withDrainLimit(...)` for stale-RX handling.
- RX drain overflow now surfaces as `UartError::RxGarbage` instead of being silently absorbed and later reported ambiguously.
- The software-side register mirror now updates only after explicit successful transport operations. Failed UART reads/writes no longer poison the stored cache with fallback values.
- `TMC51X0::reinitialize()` now reseeds the register mirror to known reset defaults before replaying library setup state.
- UART transport errors are now prevented from poisoning the software mirror. SPI remains optimistic because the current SPI transport does not surface explicit transaction failures; after suspected SPI faults or external resets, re-read key registers or call `reinitialize()` to resynchronize.
- Advanced recovery code can call `stepper.registers.assumeDeviceReset()` directly and inspect `stepper.registers.storedValid(...)` when needed.

## Existing code compatibility

Existing blocking code continues to work:

```cpp
stepper.setupUart(uart_parameters);
auto value = stepper.registers.read(tmc51x0::Registers::GconfAddress);
```

Existing async code that used `uartInterface()` also continues to work:

```cpp
auto &uart = stepper.uartInterface();
uart.poll();
```

## Preferred style going forward

Use `uartBus()` when writing new non-blocking code so this library matches the broader UART-driver family more closely:

```cpp
auto &bus = stepper.uartBus();
auto start = bus.startRead(tmc51x0::Registers::GconfAddress);
if (start.ok())
  {
    // keep polling until bus.done()
  }
```
