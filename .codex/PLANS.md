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
