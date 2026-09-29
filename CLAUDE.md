# CLAUDE.md

Guidance for AI assistants (and humans) working in this repo.

## What this is
Research software for a **PHANToM Premium** haptic device (OpenHaptics 3.5 HD API,
Windows 11 lab PC, Visual Studio 2022) used to assess finger/hand movement after
stroke or surgery. Background: `docs/PROJECT_CONTEXT.md`. Past failure modes:
`docs/TROUBLESHOOTING.md`. The users are biomedical engineering undergrads who
are not C++ build experts, so keep the workflow simple.

## Layout
- `common/phantom.h` — the ONLY place that talks to the HD API: device open, info,
  calibration, servo loop, force safety (clamp, fade-in, speed trip), sample queue.
- `common/util.h`, `common/console.h` — args, CSV, data dir, keyboard.
- `apps/<name>/main.cpp` — one program per folder, built to `bin/<name>.exe`.
  `apps/template` is the starting point for new programs.
- `tools/hd_mock/` — fake `<HD/hd.h>` + simulated arm for building/testing without hardware.
- `analysis/analyze.py` — metrics and plots for any CSV the apps write.
- `legacy/` — the original Dec 2025 PositionTracker (reference only, not built).

## Build & test
- Lab PC: `build.bat` (or `build.bat <app>`). Uses cl.exe directly. No .vcxproj.
- Anywhere without hardware:
  `cmake -S . -B build-mock -DPHANTOM_MOCK=ON && cmake --build build-mock`, then run
  `bin/<app>` and pipe keys, e.g. `(printf '\n'; sleep 5; printf q) | ./bin/position_tracker`.
  `HD_MOCK_BUTTON_PERIOD=1` makes the fake button press every second.
- Always build with the mock (no warnings under -Wall -Wextra) before handing code
  to someone to run on the real device.

## Rules for new code
- Units: mm, mm/s, N, N/mm, N·s/mm, rad, s. Device frame: +x right, +y up, +z toward user.
- Force functions run in the servo thread at 1 kHz: no printf, file I/O, allocation,
  or locks. Change their inputs from the main thread only through `Device::runInServo`.
  Never call `latest()`/`runInServo()` from inside the servo thread (deadlock).
- Never anchor a spring to a fixed absolute point. Anchor to the current position
  and move set points with `phantom::minJerk`.
- Cap stiffness relative to `info().maxStiffness` (≤ 50 %) and add some damping.
- Keep the per-sample CSV columns from `util::writeSampleHeader` so `analyze.py`
  works everywhere. Put trial-level results in a separate `_trials.csv`.
- Never commit anything in `data/` (participant data) or OpenHaptics SDK files.

## Unverified on real hardware (check first on the lab PC)
The code has only been run against the mock. Checked Sept 28 2026 against the real
OpenHaptics 3.5.0 headers and lib layout (from `matthewdkim2025/Towles-research-2026`):
- DONE: every `HD_*` name and HD function used exists in the real headers, and all
  apps type-check against them. `hd.lib` is at `lib\x64\Release`, where `build.bat`
  looks first. Still to confirm: an actual `cl.exe` build on the lab PC.
- DONE: `Device::calibrate` follows the SDK "Calibration" example's encoder-reset path
  (`hdUpdateCalibration(HD_CALIBRATION_ENCODER_RESET)` before the scheduler starts).
Still open:
1. Whether calibration reports OK on this Premium, and what the reset pose is exactly.
2. Axis directions and the button bit on this particular device.
3. Default safety limits (60 % of continuous force, 1000 mm/s trip) feel sensible.
