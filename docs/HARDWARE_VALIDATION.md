# Hardware Validation Checklist

This document defines the recommended bench-validation sequence for `TMC51X0`
before release or after any hardware-facing firmware change.

The goal is not exhaustive characterization. The goal is to confirm that the
library's documented SPI, UART, motion, switch, and recovery behavior matches
real hardware on representative setups.

## Scope

Validate all of the following on real hardware when possible:

- SPI communication bring-up
- UART communication bring-up
- blocking UART access
- non-blocking UART access
- reset and recovery behavior
- basic motion commands
- reference switch behavior
- chip-family coverage for `TMC5130A` and `TMC5160A`

## Test matrix

At minimum, try to cover:

- one `TMC5130A` setup
- one `TMC5160A` setup
- one SPI wiring path
- one UART wiring path
- one board from the RP2040 family
- one second MCU family if available

Representative repository examples:

- SPI smoke test:
  `examples/SPI/TestCommunication`
- UART smoke test:
  `examples/UART/TestCommunication`
- UART blocking read:
  `examples/UART/ReadRegister`
- UART non-blocking read:
  `examples/UART/NonBlockingReadRegister`
- SPI position / velocity:
  `examples/SPI/TestPosition`
  `examples/SPI/TestVelocity`
- UART position / velocity:
  `examples/UART/TestPosition`
  `examples/UART/TestVelocity`
- switch and homing checks:
  `examples/SPI/TestSwitches`
  `examples/SPI/HomeToSwitch`
  `examples/SPI/HomeToStall`

## Bench setup notes

Before running tests:

- verify motor power is present and within the board's supported range
- verify logic voltage and MCU I/O levels are correct
- verify common ground between MCU and driver board
- verify coil wiring and current limits are safe for the motor
- verify any emergency-stop or power cutoff path is working
- verify switch wiring and polarity before enabling homing tests

For UART benches:

- confirm the bus is wired for the expected half-duplex topology
- document whether `enable_txrx_pin` is used
- document baud rate, reply timeout, retry count, and drain limit

For recovery testing:

- prepare a safe way to power-cycle or reset the chip independently of the MCU
- record whether reset is visible through SPI reset flags or only by observed
  behavior

## Pre-bench software validation

Before touching hardware, run what the local environment supports:

```sh
python3 tools/version_sync.py check
python3 tools/pio_task.py test --env native
python3 tools/pio_task.py build --example examples/SPI/TestCommunication --env pico
python3 tools/pio_task.py build --example examples/UART/TestCommunication --env pico
```

Helpful extra coverage when available:

```sh
python3 tools/pio_task.py build --example examples/SPI/TestCommunication --env teensy40
python3 tools/pio_task.py build --example examples/SPI/TestCommunication --env esp32
python3 tools/pio_task.py build --example examples/SPI/TestCommunication --env giga_r1_m7
python3 tools/pio_task.py build --example examples/SPI/TestCommunication --env nanoatmega328
python3 tools/pio_task.py build --example examples/UART/TestCommunication --env teensy40
```

## Validation procedure

### 1. SPI smoke test

Load `examples/SPI/TestCommunication`.

Confirm:

- `stepper.communicating()` succeeds repeatedly
- IO and status register prints are stable
- no unexpected reset indication appears during idle communication

Record:

- board
- chip model
- power supply
- MCU environment
- pass/fail

### 2. UART smoke test

Load `examples/UART/TestCommunication`.

Confirm:

- communication comes up reliably after boot
- repeated reads remain stable
- no persistent timeout or CRC failures occur during normal operation

Run this in both configurations if possible:

- with `enable_txrx_pin` configured
- without `enable_txrx_pin` configured when hardware wiring allows it

### 3. UART blocking read path

Load `examples/UART/ReadRegister`.

Confirm:

- repeated register reads succeed
- error reporting remains clean during steady-state communication
- startup behavior is deterministic after repeated power cycles

### 4. UART non-blocking path

Load `examples/UART/NonBlockingReadRegister`.

Confirm:

- `uartBus().poll()`-driven reads complete correctly
- `startRead(...)`, `done()`, and `takeReadResult()` behave as expected
- no hidden blocking behavior breaks the application event loop

If the bench can inject stale RX bytes or timing stress, observe whether
failures surface as `UartError::RxGarbage`, `ReplyTimeout`, or `CrcMismatch`
instead of ambiguous behavior.

### 5. Motion smoke tests

Run at least one position and one velocity example on each chip family being
validated.

Confirm:

- commanded motion begins and ends as expected
- direction is correct
- standstill and enable behavior are sane
- no obvious missed-step or runaway behavior appears in the tested range

Recommended examples:

- `examples/SPI/TestPosition`
- `examples/SPI/TestVelocity`
- `examples/UART/TestPosition`
- `examples/UART/TestVelocity`

### 6. Switch and homing behavior

Run switch-sensitive examples only after confirming switch polarity and travel
limits.

Confirm:

- switch inputs are detected correctly
- stop and latch behavior match expectations
- homing stops in a safe and repeatable way

Recommended examples:

- `examples/SPI/TestSwitches`
- `examples/SPI/HomeToSwitch`
- `examples/SPI/HomeToStall`
- `examples/SPI/PauseWithSwitches`

### 7. Reset and recovery drills

This is the most important behavior check for the v4 release line.

Validate at least one external reset or power-cycle drill per supported
transport.

Procedure:

1. Start from a known communicating configuration.
2. Write a recognizable high-level configuration state.
3. Mark possible drift with `notePossibleMirrorDrift()` if the application is
   aware of the event.
4. Power-cycle or reset the chip while keeping the MCU alive.
5. Call `recoverFromDeviceReset()` or `recoverIfNeeded()`.
6. Re-read a safe subset of readable configuration registers.

Confirm:

- recovery succeeds when transport health is restored
- high-level configuration state is replayed
- raw `registers.write(...)` state is not assumed to be replayed unless the
  application reapplies it
- motion is not resumed automatically in an unsafe way

If SPI is used, also confirm that reset-flag handling causes recovery rather
than silent trust in stale mirror contents.

### 8. Repeated power-cycle stability

Repeat the chosen SPI and UART smoke tests across multiple cold and warm restarts.

Confirm:

- communication remains deterministic
- setup does not require accidental timing delays to succeed
- known device-model seeding behaves as expected when the model is supplied

## What to log during validation

For each bench run, record:

- date
- engineer
- board and MCU
- chip model
- transport type
- wiring notes
- firmware example used
- library commit or version
- result
- observed failures
- follow-up action

## Failure triage guidance

When a bench test fails, classify it before changing firmware:

- wiring or board issue
- host setup or upload issue
- transport timing issue
- library logic bug
- chip-specific behavior difference
- documentation gap

Do not make speculative firmware changes from a single ambiguous symptom. Prefer
capturing the bench conditions first, then reproducing with the smallest
possible example.

## Release gate suggestion

Before calling the repository release-ready, aim for:

- at least one passing SPI bench on real hardware
- at least one passing UART bench on real hardware
- at least one passing recovery drill
- at least one passing validation run on `TMC5130A`
- at least one passing validation run on `TMC5160A`

Any remaining limitations should be documented explicitly in the README or
relevant transport/recovery docs.
