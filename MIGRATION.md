# v3 to v4 Migration Guide

`TMC51X0` v4 keeps the broad library structure intact, but it tightens UART
error reporting, adds family-style async UART names, and makes reset recovery
and register-mirror behavior more explicit.

Most sketches do not need a full rewrite. The usual migration path is:

1. Keep existing SPI or blocking UART code working first.
2. Update any new async UART code to prefer the v4 naming.
3. Adopt the new recovery helpers if your hardware can reset independently of
   the MCU.
4. Audit any code that assumed the software register mirror always matched chip
   state.

## Executive summary

What changes for most users:

- Existing blocking register access continues to work.
- Existing `uartInterface()`-based async code continues to work.
- New async code should prefer `uartBus()`.
- UART failures now use typed `tmc51x0::Result<T>` and `tmc51x0::UartError`.
- Recovery after reset is more explicit and more conservative.
- If you know the device model up front, pass it into `setupSpi(...)` or
  `setupUart(...)`.

## Compatibility overview

Source compatibility expectations:

- SPI sketches using `setupSpi(...)` still compile.
- Blocking UART sketches using `setupUart(...)`, `readRegister(...)`, and
  `writeRegister(...)` still compile.
- Async UART sketches using `uartInterface()` still compile.
- Older async names such as `startReadRegister(...)`,
  `startWriteRegister(...)`, and `resultReady()` remain available.

Behavior changes you should review:

- UART drain overflow now reports `UartError::RxGarbage`.
- Failed reads and writes no longer update the software-side register mirror.
- SPI reset detection now marks the mirror as needing recovery.
- Recovery replays high-level configuration state but does not reconstruct
  arbitrary in-flight motion.

## Old to new API map

### Async UART access

Preferred v4 naming:

- `stepper.uartInterface()` -> `stepper.uartBus()`
- `startReadRegister(address)` -> `startRead(address)`
- `startWriteRegister(address, data)` -> `startWrite(address, data)`
- `resultReady()` -> `done()`

Compatibility note:

- The old names are still available. Migration is recommended for consistency,
  not because the older code is immediately broken.

### Typed UART results

The async and result-returning UART surfaces now center on:

- `tmc51x0::Result<T>`
- `tmc51x0::Result<void>`
- `tmc51x0::UartError`

Current UART errors:

- `None`
- `Busy`
- `NotInitialized`
- `ReplyTimeout`
- `CrcMismatch`
- `UnexpectedFrame`
- `RxGarbage`

### Reset and recovery helpers

Newer recovery-oriented helpers on `TMC51X0` include:

- `notePossibleMirrorDrift()`
- `mirrorResyncRequired()`
- `recoverFromDeviceReset()`
- `recoverIfNeeded()`
- `recoverIfUnhealthy()`
- `readHealthStatus()`
- `resyncReadableConfiguration()`

## Before and after examples

### 1. Blocking UART code

Existing blocking code generally does not need structural changes.

v3-style code that still works in v4:

```cpp
stepper.setupUart(uart_parameters);

while (!stepper.communicating()) {
  delay(1000);
}

uint32_t gconf = stepper.registers.read(tmc51x0::Registers::GconfAddress);
stepper.registers.write(tmc51x0::Registers::GconfAddress, gconf);
```

Recommended v4 adjustment when the target chip is known:

```cpp
stepper.setupUart(uart_parameters, tmc51x0::Registers::DeviceModel::TMC5160A);

while (!stepper.communicating()) {
  delay(1000);
}

uint32_t gconf = stepper.registers.read(tmc51x0::Registers::GconfAddress);
stepper.registers.write(tmc51x0::Registers::GconfAddress, gconf);
```

Why update:

- Passing the expected device model gives deterministic reset seeding before
  the first successful identity read.

### 2. Async UART code

Older async code:

```cpp
auto &uart = stepper.uartInterface();
uart.poll();

auto start = uart.startReadRegister(tmc51x0::Registers::GconfAddress);
if (start.ok()) {
  while (!uart.resultReady()) {
    uart.poll();
  }

  auto result = uart.takeReadResult();
}
```

Preferred v4 style:

```cpp
auto &bus = stepper.uartBus();
bus.poll();

auto start = bus.startRead(tmc51x0::Registers::GconfAddress);
if (start.ok()) {
  while (!bus.done()) {
    bus.poll();
  }

  auto result = bus.takeReadResult();
  if (!result.ok()) {
    // Handle result.error.
  }
}
```

Why update:

- The new names align `TMC51X0` with the wider family of poll-driven UART
  drivers.

### 3. UART error handling

If your code previously depended on ad hoc error inspection, migrate to typed
results explicitly.

Recommended v4 pattern:

```cpp
auto result = stepper.readRegisterResult(tmc51x0::Registers::GconfAddress);
if (!result.ok()) {
  if (result.error == tmc51x0::UartError::ReplyTimeout) {
    // Retry or flag transport failure.
  }
  return;
}

uint32_t gconf = result.value;
```

### 4. Recovery after external reset

Older application code often assumed a reinitialize-style call or a fresh setup
was enough after power loss. In v4, recovery is explicit and should follow the
reset semantics documented by the library.

Recommended v4 flow:

```cpp
stepper.notePossibleMirrorDrift();

// Optionally perform the external reset or power cycle here.

if (stepper.mirrorResyncRequired()) {
  bool ok = stepper.recoverFromDeviceReset();
  if (!ok) {
    // Transport is still unhealthy.
  }
}
```

For polling maintenance loops:

```cpp
(void)stepper.recoverIfNeeded();
```

For applications that want one conservative poll covering reset, driver-fault,
charge-pump-undervoltage, mirror drift, and communication-loss scenarios:

```cpp
(void)stepper.recoverIfUnhealthy();
```

### 5. Stall-home completion semantics

If your v3-era code treated any stop during stall homing as success, review it
carefully.

Recommended v4 pattern:

```cpp
stepper.beginHomeToStall(home_parameters, stall_parameters);

while ((!stepper.homed()) && (!stepper.homeFailed())) {
  (void)stepper.recoverIfUnhealthy();
  delay(10);
}

if (stepper.homeFailed()) {
  // Motion stopped without a confirmed stall-stop event.
  stepper.endHome();
  return;
}

stepper.endHome();
```

Why update:

- v4 no longer treats a generic zero-velocity stop as a successful stall home
- `homed()` now requires a confirmed stall-stop event for stall-home mode
- `homeFailed()` lets the application reject ambiguous stops explicitly

## Semantic changes that matter

### The register mirror is not chip truth

In v4, the software-side mirror is explicitly treated as best-known intended
state.

This means:

- successful transport operations update the mirror
- failed transport operations do not poison the mirror with fallback data
- external resets can invalidate confidence in mirrored values

If your v3-era code assumed a failed read still refreshed cache state, that
assumption is no longer valid.

### Recovery replays high-level desired state

Recovery is stronger than before, but it is deliberately scoped.

What is replayed:

- high-level configuration writes made through the driver, controller, and
  encoder layers
- setup-derived configuration state

What is not replayed automatically:

- arbitrary raw `registers.write(...)` calls
- in-flight motion history
- exact runtime motion continuity after a chip reset

### Conservative health polling

Projects with independent power control, watchdog recovery, or fault
supervision should prefer the new conservative health poll.

Recommended pattern:

```cpp
tmc51x0::HealthStatus health = stepper.readHealthStatus();
if (!health.communication_ok || health.driver_error ||
    health.charge_pump_undervoltage) {
  (void)stepper.recoverIfUnhealthy();
}
```

This is especially relevant if your v3-era project sometimes power-cycled the
driver IC after stall-home failures or suspected driver faults.

If your application writes critical configuration through raw
`registers.write(...)`, consider moving that logic behind higher-level helpers
or explicitly reapplying it after recovery.

### Device-model-aware reset defaults

`TMC5130A` and `TMC5160A` are similar but not identical. v4 makes that
difference matter earlier in setup and recovery.

Recommended pattern:

```cpp
stepper.setupSpi(spi_parameters, tmc51x0::Registers::DeviceModel::TMC5130A);
```

or:

```cpp
stepper.setupUart(uart_parameters, tmc51x0::Registers::DeviceModel::TMC5160A);
```

## Migration checklist

Use this checklist when updating an existing project:

1. Build the project without changing logic to confirm baseline compatibility.
2. If you use async UART, replace `uartInterface()` with `uartBus()` in new or
   touched code.
3. Replace `startReadRegister(...)` / `startWriteRegister(...)` with
   `startRead(...)` / `startWrite(...)` where practical.
4. Replace `resultReady()` with `done()` where practical.
5. Audit UART error handling and branch on `tmc51x0::UartError` explicitly.
6. Pass `Registers::DeviceModel` to setup when your hardware target is known.
7. Add reset-recovery handling if the driver can reset independently of the
   host MCU.
8. For stall-home flows, switch polling loops to use `homed()` together with
   `homeFailed()` rather than assuming any stop is success.
9. Review any raw `registers.write(...)` configuration that should be restored
   after reset.

## Suggested LLM-assisted or scripted refactors

If you are using Codex, another LLM, or a scripted rename pass, make the
migration in narrow stages instead of one broad rewrite.

Recommended order:

1. Rename `uartInterface()` to `uartBus()` in non-blocking code only.
2. Rename `startReadRegister` to `startRead` and `startWriteRegister` to
   `startWrite`.
3. Rename `resultReady()` to `done()`.
4. Inspect each call site that checks UART completion or error handling.
5. Add explicit `DeviceModel` setup arguments where the board target is fixed.
6. Add recovery-flow calls only where the application truly has reset or power
   supervision responsibilities.
7. Inspect any stall-home loops that previously assumed zero velocity implied a
   successful home and update them to use `homeFailed()`.

Safe search targets:

- `uartInterface(`
- `startReadRegister(`
- `startWriteRegister(`
- `resultReady(`
- `reinitialize(`
- `registers.write(`

Review each raw `registers.write(...)` hit manually. Those call sites are the
most likely places where recovery expectations need human judgment.

## Validation after migration

At minimum, run:

```sh
python3 tools/version_sync.py check
```

If the environment has the required tools, also run:

```sh
python3 tools/clang_format_all.py --check
python3 tools/pio_task.py test --env native
python3 tools/pio_task.py build --example examples/SPI/TestCommunication --env pico
python3 tools/pio_task.py build --example examples/UART/TestCommunication --env pico
```

See [docs/UART.md](./docs/UART.md) for UART usage details and
[docs/RECOVERY.md](./docs/RECOVERY.md) for the reset-recovery model.
