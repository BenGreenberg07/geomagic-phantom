# geomagic-phantom

Programs for measuring hand and finger movement with a **PHANToM Premium**
haptic device (OpenHaptics 3.5). The goal is assessment and training after stroke
or surgery.

**New here?** Read [`docs/SETUP_WINDOWS.md`](docs/SETUP_WINDOWS.md), then run the
three commands below. If something misbehaves or the arm "spazzes", read
[`docs/TROUBLESHOOTING.md`](docs/TROUBLESHOOTING.md).

```
build.bat                               # compile everything into bin\
bin\device_check.exe                    # FIRST, every session: calibrate + sanity check
bin\position_tracker.exe --id P01       # record movement at 1000 Hz
python analysis\analyze.py data\<file>.csv
```

## Programs

| Program | Forces? | What it does |
|---|---|---|
| `device_check` | only a weak test spring (press S) | Prints device limits, calibrates, shows live position / gimbal / buttons / servo rate. |
| `position_tracker` | never | Logs every servo tick (~1 kHz) to CSV: position, velocity, joint and gimbal angles, buttons. SPACE marks segments. |
| `position_matching` | yes, gentle | Proprioception test: the device guides the hand to a hidden target and back, then the participant reproduces the position and presses the button. Outputs error per trial. |
| `template` | yes | A virtual floor. **Copy this folder to start a new program.** |
| `axis_check` | never | Guided moves and button presses. Checks +x right, +y up, +z toward you, and the button bits. |
| `force_direction` | yes, 0.3 N pulses | Pushes gently along each axis and checks the stylus moves the same way. Catches a flipped motor/axis. |
| `damping_field` | yes, drag only | "Moving through honey", levels 0-3. Can't store energy, so it's the safest force to feel first. |
| `virtual_box` | yes | Invisible box around where you place it (ENTER or button 1). One-sided walls with damping. |

All output goes to `data/` (git-ignored, because it's participant data).

## First session on a new device (in this order)

Each step only if the one before looked right. Close every other haptics program first.

1. `bin\device_check.exe`: calibrate (hold the reset pose, press ENTER), watch the numbers. Don't press S yet.
2. `bin\axis_check.exe`: no forces. All three axes should say PASS.
3. `bin\force_direction.exe`: hold the stylus loosely. All six pushes should say PASS.
4. `bin\damping_field.exe`: try levels 1, 2, 3. Should feel smooth, not gritty.
5. `bin\virtual_box.exe`: press ENTER, then feel for the walls. Should feel solid, not buzzy.

Stop at the first thing that surprises you. Q always turns forces off immediately.

## Making a new program

1. Copy `apps/template` to `apps/my_experiment`.
2. Edit `apps/my_experiment/main.cpp`. The three places to change are marked `<-- EDIT`.
3. Run `build.bat my_experiment`, then `bin\my_experiment.exe`.

There are no Visual Studio project files to copy or fix. Nothing goes in the
OpenHaptics examples folder. All device access goes through
[`common/phantom.h`](common/phantom.h), which handles calibration, force
limits, fade-in, a speed cut-out and thread-safe logging, so a bug in your
program can't fling the arm.

## No device handy?

Build against the simulator (Mac/Linux/Windows):
```
cmake -S . -B build-mock -DPHANTOM_MOCK=ON && cmake --build build-mock
./bin/position_tracker --id test
```
`HD_MOCK_HAND=still` stops the simulated hand wandering (so `force_direction` gives
clean answers), and `HD_MOCK_FLIP=x` makes the fake x motor push backwards, to
check that `force_direction` catches it.

## Layout

```
common/       phantom.h (safe device layer), util.h, console.h
apps/         one folder per program -> bin/<name>.exe
analysis/     analyze.py (speed, smoothness/SPARC, path length, matching errors + plots)
tools/hd_mock simulated device for testing without hardware
docs/         setup, troubleshooting, project context
legacy/       original Dec 2025 PositionTracker (reference)
```
