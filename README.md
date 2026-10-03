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

## Using it on the lab computer (step by step)

### 1. Open a Command Prompt (not PowerShell)
PowerShell is blocked by IT on the lab PC. Press **Win + R**, type `cmd`, press Enter.
(Or in Windows Terminal: the **˅** arrow next to the tab -> **Command Prompt**.)

### 2. Go to the repo folder
```
cd /d C:\Users\bgreenb2\source\repos\geomagic-phantom
```
Every command below is typed **from this folder**. If a command says "not recognized", you're
probably in the wrong folder: run the `cd` line again.

First time on a new computer only (instead of `cd`): `git clone https://github.com/BenGreenberg07/geomagic-phantom.git`,
then `cd geomagic-phantom`.

### 3. Get the latest code and build it
```
git pull
build.bat
```
- `git pull` downloads whatever was pushed to GitHub (e.g. from someone's laptop).
- `build.bat` compiles every program into `bin\`. It should end with `Done.` If it says
  `BUILD FAILED`, the compiler error is printed right above it. Copy it and ask.
- `build.bat grab_ball` builds just one program (faster).

### 4. Close other haptics programs, then run one
Only **one** program can use the arm at a time: close PHANToM Test, Phantom Configuration's test,
and any OpenHaptics example first. Then type the program's command, for example:
```
bin\device_check.exe
```
Every program calibrates first. On this Premium it usually just says `Calibration status: OK`;
if it asks, put the stylus in its holder (inkwell). **Q always quits and turns forces off at once.**
3D programs open a window: **click the window first** so it gets your key presses.

### 5. Your data
Every program saves to `data\` as `<program>_<date>_<time>.csv` (every 1 ms sample) plus an
`.info.txt` with the settings. `data\` never goes to GitHub (participant data); copy it off yourself.
To analyse a recording (needs Python, once: `pip install -r analysis\requirements.txt`):
```
python analysis\analyze.py data\<file>.csv
```

### 6. Saving code changes back to GitHub
```
git add -A
git commit -m "what you changed"
git push
```
(`data\`, `bin\` and `build\` are ignored automatically.)

### Command list

**Checks (run at the start of a session, in this order)**

| Command | Forces | What it does / what to do |
|---|---|---|
| `bin\device_check.exe` | weak spring only if you press S | Device info, calibration, live position/speed/buttons/1000 Hz. Run first, every session. |
| `bin\axis_check.exe` | none | Guided moves (right, up, toward you) + buttons. All should say PASS. |
| `bin\force_direction.exe` | 0.3 N pulses | Hold the stylus loosely; ENTER sends each of 6 pushes. Checks motors push the right way. |
| `bin\damping_field.exe` | drag only | "Honey": keys 0-3 change thickness. Should feel smooth. |
| `bin\virtual_box.exe` | walls | ENTER puts an invisible box around the stylus. Walls should feel solid, not buzzy. |

**Research tasks**

| Command | What it does |
|---|---|
| `bin\position_tracker.exe --id P01` | Records movement at 1000 Hz, no forces. SPACE marks a new segment, Q saves. |
| `bin\position_matching.exe --id P01` | Proprioception test: the arm guides the hand to a hidden target and back, the participant finds it again and presses the button. Saves errors per trial. |
| `bin\reach_game.exe` | 3D center-out reaching, 24 trials. Saves reaction time, movement time, path ratio, peak speed, submovements per trial (`_trials.csv`). |

**3D programs (see and feel)** — click the window; ENTER places the room around the stylus;
drag/arrows rotate, wheel zooms, F force arrow, X remove room, Q quit.

| Command | What to try |
|---|---|
| `bin\haptic_playground.exe` | Hard cube, soft sphere, a ball to push. |
| `bin\grab_ball.exe` | Hold the stylus button near the ball to pick it up; 1/2/3 = 40/80/150 g; put it on the shelf or throw it. R resets. |
| `bin\surfaces.exe` | Friction block (M = friction level), bumpy floor, magnet, honey zone. |
| `bin\mechanisms.exe` | Clicky push button; hold the stylus button on the drawer handle or lever knob to grab them. |
| `bin\physics_toys.exe` | Bumping balls, pendulum, wobbly soft blob. R resets. |

**Options most programs accept:** `--k 0.2` (stiffness, N/mm, capped for safety), `--b 0.001` (damping),
`--device "<name>"` (if Phantom Configuration has several devices), `--skip-calibration`,
`--allow-uncalibrated`. Each program's options are listed at the top of its `apps\<name>\main.cpp`.

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

### 3D programs (a window you can see while you feel)

All of these open a 3D window, place their "room" around the stylus when you press ENTER, and share
[`common/viz.h`](common/viz.h) (window, camera, safety, logging). Click the window first; drag or arrows
rotate the view, wheel zooms, F hides the force arrow, X removes the room, Q quits.

| Program | What it does |
|---|---|
| `haptic_playground` | Room with a hard cube, a soft sphere and a ball to push. The first demo. |
| `grab_ball` | Hold button 1 near the ball to pick it up, feel its weight (1/2/3 = 40/80/150 g), carry it to the shelf, drop or throw it. |
| `surfaces` | Friction block (stick-slip, M = level), bumpy floor texture, a magnet, a honey (viscous) zone. |
| `reach_game` | Center-out reaching task: 8 targets x 3 rounds. Writes a `_trials.csv` with reaction time, movement time, path ratio, peak speed, number of submovements. |
| `mechanisms` | Click button (force drop at 2.5 mm), a drawer on a rail with end stops and a magnetic catch, a lever with notches. Grab with button 1. |
| `physics_toys` | Balls that bump each other, a 3D pendulum, a soft blob that wobbles and visibly dents. |

All output goes to `data/` (git-ignored, because it's participant data).

## First session on a new device (in this order)

Each step only if the one before looked right. Close every other haptics program first.

1. `bin\device_check.exe`: calibrate, watch the numbers. Don't press S yet. (On the lab's Premium the
   calibration is inkwell-style and usually already OK; if asked, put the stylus in its holder.)
2. `bin\axis_check.exe`: no forces. All three axes should say PASS.
3. `bin\force_direction.exe`: hold the stylus loosely. All six pushes should say PASS.
4. `bin\damping_field.exe`: try levels 1, 2, 3. Should feel smooth, not gritty.
5. `bin\virtual_box.exe`: press ENTER, then feel for the walls. Should feel solid, not buzzy.

Stop at the first thing that surprises you. Q always turns forces off immediately.
Results of the first session on the lab PC (Oct 2 2026) are in [`docs/LAB_LOG.md`](docs/LAB_LOG.md).

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
common/       phantom.h (safe device layer), viz.h (3D apps), util.h, console.h
apps/         one folder per program -> bin/<name>.exe
analysis/     analyze.py (speed, smoothness/SPARC, path length, matching errors + plots)
tools/hd_mock simulated device for testing without hardware
docs/         setup, troubleshooting, project context
legacy/       original Dec 2025 PositionTracker (reference)
```
