# TMC51X0 plan and execution notes

Use this file for work that is multi-step, risky, or likely to span multiple sessions.

## Current goal

Finish `TMC51X0` as a release-ready library without reopening broad architecture churn.

## Current state

- The repository already has typed UART results, `uartBus()`, mirror-confidence tracking, recovery helpers, native tests, and CI.
- The repo is in finish-line mode: most remaining work is documentation, migration clarity, README polish, and hardware validation.
- The canonical and only repo README is now `README.org` at the repository root.
- `.metadata/README.org` and a generated `README.md` workflow are intentionally not part of the plan.
- Real hardware bring-up should drive any remaining firmware fixes after the docs / migration pass.

## Completed 2026-09-02: TMC5130A correctness fixes, released as 4.1.0

Four defects, all found on bench hardware during a TMC5130A + RP2040 bring-up
with 0.15 ohm sense resistors. Details and the upgrade notes are in
`CHANGELOG.md` and `MIGRATION.md`; the durable summary:

- **`CHOPCONF.vsense` was unreachable.** `Registers::Chopconf` was the TMC5160
  layout, where bit 17 is reserved. Added `SenseVoltageMode`,
  `DriverParameters::withSenseVoltageMode()`,
  `Driver::writeSenseVoltageMode()` and `Chopconf::vsense()`, gated on the
  device model because the bit is reserved on the TMC5160.
- **`CHOPCONF`'s seeded reset default was the TMC5160's for both parts.** Bits
  23:20 are `TPFD` on the 5160 and `SYNC` on the 5130, so the seed enabled
  chopSync on a 5130 through the read-modify-write path.
  `chopconfResetDefault()` now mirrors `pwmconfResetDefault()`, and
  `setDeviceModel()` re-seeds model-dependent defaults when the model is
  learned after seeding -- which is what `setupSpi()` does when the caller does
  not declare it.
- **`Controller::initialize()` commanded the motor to turn.** The default
  `ControllerParameters` were `VelocityPositiveMode` with `max_velocity` 10.
  Now `HoldMode` with 0. `reinitialize()` no longer replays a velocity ramp
  mode either.
- **`VSTOP` and `D1` could be written as zero**, which the datasheet forbids in
  positioning mode and which makes the chip crawl at `VSTOP` rather than fault.
  Both writers clamp to 1.

- **A reversed encoder needed a signed fixed-point value composed by hand.**
  `ENC_CONST`'s fractional part is always added to a signed integer part, so
  -12.8 is -13 + 0.2 and the obvious -12 + 0.8 is -11.2 -- a silent 12.5% scale
  error. Added
  `EncoderParameters::withMicrostepsPerPulse(FractionalMode, int32_t scaled)`
  and `Encoder::writeMicrostepsPerPulseScaled()`, which take one signed value in
  the mode's units and split it with floor division. An encoder on the far end
  of a shaft reverses as a matter of geometry, so this is a recurring condition
  rather than a one-rig quirk.

Also: `beginHomeToSwitch()`/`beginHomeToStall()` replay the caller's driver and
controller configuration instead of default-constructing it; agent guidance
moved from `.codex/` to `.agents/` behind `AGENTS.md` and `CLAUDE.md` shims; CI
gained a formatting gate.

Validation: 50/50 native tests, formatting clean, version metadata consistent,
and `examples/SPI/TestCommunication` (pico, teensy40),
`examples/UART/TestCommunication` (pico) and `examples/SPI/HomeToSwitch` (pico)
all build.

### Known follow-ups, not done here

- `homed()` reports switch-homing success on standstill alone, so "stopped at
  the switch" and "reached the travel bound" are indistinguishable; the stall
  path already got this treatment and the switch path did not. There is also a
  race where the first `homed()` after `beginHomeToSwitch()` can return true
  before the ramp accelerates.
- `leftSwitchActive()` / `leftLatchActive()` / `leftStopEvent()` each read
  `RAMP_STAT` separately, and several of those bits are read-and-clear, so
  calling two in a row loses a flag. A single accessor returning the decoded
  struct would fix it.
- The register access tables are not device-model gated, so TMC5160-only
  registers are writeable on a TMC5130A. `driver.setup()` therefore always
  writes `GLOBALSCALER` (0x0B), which does not exist on that part. Harmless,
  but a wasted transaction and a meaningless mirror entry.
- `actions/checkout@v4` and `actions/setup-python@v5` in CI are probably behind
  current majors. Not bumped here, because a wrong version breaks CI and it
  could not be verified offline.

## Ordered milestones

1. README polish patch
   - keep `README.org` useful as a standalone, GitHub-visible root README
   - expand setup, UART, recovery, and example guidance without reintroducing duplicate README sources
   - keep `tools/version_sync.py` and related task descriptions aligned if README metadata handling changes
2. Migration guide patch
   - rewrite `MIGRATION.md` as a real v3 -> v4 guide
   - add old -> new API mapping
   - add before / after snippets
   - add guidance for LLM-assisted or scripted refactors
3. Hardware validation doc patch
   - add `docs/HARDWARE_VALIDATION.md`
   - document SPI and UART smoke tests, reset drills, motion / switch checks, and model-variant checks
4. Bench bring-up and bug-fix patches
   - run the hardware validation checklist
   - make only the fixes that real hardware findings justify
5. Final polish
   - README / docs / example consistency
   - family-style naming and tooling polish where helpful
   - no speculative redesign

## Architecture checkpoints

Keep these truths intact unless a task explicitly changes them and documents/tests the change:

- caller-owned transport setup
- `uartBus()` preferred alias, `uartInterface()` compatibility alias
- typed `Result<T>` / `UartError` surface
- poll-driven UART engine shared by blocking and non-blocking paths
- mirror is best-known intended state, not guaranteed device truth
- mirror only updates on successful transport outcomes
- chip-aware reset defaults for `TMC5130A` and `TMC5160A`
- recovery restores configuration conservatively; it does not reconstruct arbitrary motion history
- raw `registers.write(...)` is not replay-tracked desired state

## Verification menu

Docs / README patch:

- `python tools/version_sync.py check`
- `python tools/clang_format_all.py --check`

Firmware / behavior patch:

- `python tools/version_sync.py check`
- `python tools/pio_task.py test --env native`

Representative builds when integration risk exists:

- `python tools/pio_task.py build --example examples/SPI/TestCommunication --env pico`
- `python tools/pio_task.py build --example examples/UART/TestCommunication --env pico`
- `python tools/pio_task.py build --example examples/SPI/TestCommunication --env teensy40`

Optional extra CI-parity coverage:

- `python tools/pio_task.py build --example examples/SPI/TestCommunication --env esp32`
- `python tools/pio_task.py build --example examples/SPI/TestCommunication --env giga_r1_m7`
- `python tools/pio_task.py build --example examples/SPI/TestCommunication --env nanoatmega328`

Pixi equivalents are acceptable when Pixi is installed.

## Progress log

- [x] root `README.org` source-of-truth cleanup
- [x] `README.org` release-facing expansion
- [x] `MIGRATION.md` rewrite
- [x] `docs/HARDWARE_VALIDATION.md`
- [ ] real hardware bring-up
  - SPI bench validated on 2026-03-20 with `TMC5130A + Pico W5500`
  - `examples/SPI/PrismValidation` now covers bring-up, motion smoke, bounded stop waits, and a controlled recovery drill
  - controlled chip power-cycle recovery passed on hardware via `recoverFromDeviceReset()`
  - stop diagnostics show `HoldMode` with `VMAX=0` and `VSTART=0`, but nonzero `VACTUAL` and `vzero=0` at timeout
  - explicit deceleration values were added to `examples/SPI/PrismValidation` and rerun on hardware; the stop timeout still reproduces in both stop directions
  - current evidence says the timeout is not explained solely by missing deceleration configuration in the example
  - latest follow-up patch removes the premature `HoldMode` switch in `PrismValidation` so stop attempts now follow the velocity-mode ramp-down path before entering hold
  - bench rerun after that patch showed the main stop phases now reach zero; remaining timeout noise came from redundant post-stop waits after entering `HoldMode`
  - cleanup patch removed those redundant waits and the full Prism loop now runs cleanly end-to-end on hardware, while recovery still passes
  - library hardening added `readHealthStatus()`, `recoverIfUnhealthy()`, and stricter stall-home completion semantics via `homeFailed()`
  - native tests now cover the new health and stall-home behavior, and `examples/SPI/HomeToStall` builds on `pico`
  - remaining bring-up gaps are UART validation, switch / homing validation, and broader chip / MCU coverage
- [ ] final release polish
  - added a repo-local PlatformIO core directory via `tools/pio_task.py` so local validation no longer depends on `~/.platformio`
  - added `tools/release_check.py` and `pixi run release-check` as a single pre-release gate
  - cleaned up `platformio.ini` so it no longer advertises the old comment/uncomment example-selection workflow
  - expanded CI coverage to include `examples/SPI/HomeToStall` and `examples/SPI/PrismValidation` on `pico`
  - added release-status notes to `README.org`, bench-status recording to `docs/HARDWARE_VALIDATION.md`, and a top-level `CHANGELOG.md`
  - ran full-repo clang-format and completed the release-check validation successfully on 2026-03-24
  - synced `.clang-format` to match `TCA6408`, added a `pixi run check-format` alias alongside the existing formatting tasks, reformatted the repo, and bumped version metadata to `4.0.1` on 2026-03-26
  - extended `tools/clang_format_all.py` so format-all / format-check now also scan tracked `.org` and `.md` files for supported embedded C-family code blocks and format those snippets with the repo `.clang-format`
  - added `test/test_clang_format_all.py` regression coverage for Org and Markdown embedded block formatting, unsupported-language no-op behavior, indentation preservation, and idempotence
