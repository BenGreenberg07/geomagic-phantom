# ADL task simulator (`adl_tasks`)

Haptic simulation of the seven key-pinch ADL tasks in

> Smaby N, Johanson ME, Baker B, Kenney DE, Murray WM, Hentz VR. *Identification of key pinch
> forces required to complete functional tasks.* J Rehabil Res Dev 2004;41(2):215–224.

Fork into putty · key in/out · 3-prong plug in/out · vertical zipper close/open · horizontal zipper
close/open · ATM card in/out · doorbell remote button.

```
build.bat adl_tasks            real device          bin\adl_tasks.exe
build.bat sim adl_tasks        simulator (mouse)    bin\sim\adl_tasks.exe
bin\adl_tasks.exe --selftest   physics self-test, no device needed (exit code 0 = pass)
bin\adl_tasks.exe --force-gain 0.5     gentler first run on the arm
```

## What the paper measured (and what that means for comparisons)
- Each object was mounted on a **robot arm** (Stäubli, ATI Mini-40 force sensor, 100 Hz) that pushed it a
  set distance. The **Table 1 times are robot execution times** (onset → cessation of force) at three speeds
  (Time 1/2/3 = slow/medium/fast, 3 trials each; the mean (SD) column pools all 9). They are shown and logged
  as *robot reference times*, not human norms.
- The force requirement per task is the **peak object force (mean + 2 SD)**. The paper reports it as a pinch
  range for skin friction μ = 0.56–0.33 (pinch = object force / 2μ), so object force = low pinch × 1.12:

| Task | Peak object force (N) | Task | Peak object force (N) |
|---|---|---|---|
| Remote button | 1.4 | ATM out / in | 4.5 / 5.9 |
| Vertical zipper open / close | 2.2 / 4.5 | Fork in | 6.8 |
| Key out / in | 3.4 / 4.1 | Horizontal zipper open / close | 6.2 / 12.2 |
| | | Plug out / in | 15.6 / 20.7 |

## Force mapping
The arm is limited to 3.68 N (60 % of its 6.14 N continuous force), so one compressive curve maps every
task, keeping their order and ratios of difficulty:

**F_sim = 3.0 · (F_real / 20.7)^0.4**  and back  **F_real = 20.7 · (F_sim / 3.0)^2.5**

| Action | Real (N) | Simulated (N) | | Action | Real (N) | Simulated (N) |
|---|---|---|---|---|---|---|
| Plug in | 20.7 | 3.00 | | Fork in | 6.8 | 1.92 |
| Plug out | 15.6 | 2.68 | | ATM in | 5.9 | 1.82 |
| H-zipper close | 12.2 | 2.43 | | V-zipper close / ATM out | 4.5 | 1.63 |
| H-zipper open | 6.2 | 1.85 | | Key in / out | 4.1 / 3.4 | 1.57 / 1.46 |
| | | | | V-zipper open | 2.2 | 1.22 |
| | | | | Remote | 1.4 | 1.02 |

A single linear scale (×0.145) would make the remote 0.2 N, below the arm's own friction. The trials CSV
logs every peak in simulated N **and** converted back to real N, plus the pinch force it would need at
μ = 0.56 and 0.33 — ready for the planned pinch-sensing device.

## Interaction
1. The object starts **in your hand** (attached to the stylus, like the robot's end-effector).
2. Push it in / pull the zipper tab closed / press the button: you feel that task's force profile.
3. When the first action is complete and you **ease off**, the object stays where it is (in the lock,
   outlet, slot, putty; zipper closed).
4. **Press the stylus button on it** to take hold again, then pull it out / open the zipper.
5. **Light rotation:** the object twists with the stylus. Key (10°), plug (16°, the tolerance the paper
   found) and card (12°) jam at the entrance if twisted further; `R` disables this. The arm renders forces
   only (no torques), so a twist is a visual/alignment constraint. `--roll-axis` selects the gimbal axis
   (default 2, **to confirm on the device**).

Keys: `1`–`7` practise a task, `A` protocol (paper order, 3 cycles each), `N` skip task, `C` restart cycle,
`R` rotation tolerance, ENTER place the room, `Q` quit.

## Force models
All tasks use a **proxy ("god-object")**: the object's real position is kept inside what the world allows,
and the hand feels a spring (kc = 0.25 N/mm, ≤ 30 % of the device's max stiffness) between the stylus and
the object. Along the task axis the proxy advances only when the spring force exceeds the task's
resistance profile R(depth) (a stick-slip window: different profiles for pushing in and pulling out), so the
felt force *is* the profile; the proxy moves at most 300 mm/s, so clicks are crisp but never one-tick steps.
Force changes are limited to ≤ 0.1 N per servo tick; contacts fade in/out over 40 ms; task switches fade in
over 0.5 s. Shapes come from standards, datasheets and patents (sources below); every shape is scaled to
the task's mapped peak.

| Task | Geometry (mm) | In (push / close) | Out (pull / open) |
|---|---|---|---|
| **Key** (Kwikset KW1) | blade 27 × 8.5 × 2.1; 5 pins at 6.27 + 3.81·j; keyway 2.3 × 8.7; 2.5 mm chamfer capture; lateral clearance 0.25 / 0.4 | 0.16·P floor + 5 pin bumps (rise 1.5, fall 0.5 mm), growing 0.6→1.0·P toward the back pins; shoulder stop at 27 | mirror image at 0.83 (3.4 / 4.1 N) |
| **Plug** (NEMA 5-15) | blades 6.35 × 1.52 × 15.9 at 12.7; ground pin Ø4.8, 3.2 longer; 1.8 mm chamfer; clearance 0.4 | ground pin only 0.08·P to 3.2 mm; blade contacts spread to P by 6.2; 0.67·P plateau; face stop at 19.1 | breakaway P_out at the seat, 0.6·P_out sliding grip, small snap as blades leave |
| **ATM card** (ISO/IEC 7810 ID-1) | 85.6 × 54 × 0.76, short edge first; slot ~58 × 3 with a 4 mm flared bezel | 0.1·P friction; shutter ramp 10→20 mm to P; at 20 mm + 100 ms the reader takes the card (190 mm/s) and it leaves your fingers | ejected after 1.5 s with 30 mm protruding; roller breakaway P_out falling to 0 over 10 mm |
| **Fork / putty** | 4 tines 40 × 2.2 at 3.5 mm pitch; putty Ø60 × 20 on a plate | Bingham putty: yield (0.10 + 0.25·d/12)·P plus viscous rate limit (relaxes when held still, τ ≈ 0.3 s); lateral ploughing; dents remembered (re-entry 40 %) | 0.3 × yield, 0.35 × viscosity, suction 0.3 N decaying over 3 mm (no paper value; ~0.8–1.1 N) |
| **Vertical zipper** (#5 coil) | chain 5.8, tooth pitch 1.8, tab 20 (tether), track **45** | breakaway P at the box, then 0.75–1.0·P with tooth ripple; pulling sideways binds (+0.3·lateral) | 0.7–1.0·P_out ripple, breakaway at the top stop |
| **Horizontal zipper** (#8 molded) | chain 7.6, pitch 3.4, tab 15, track **60** | as above, stronger ripple (±20 %) | as above |
| **Remote** (doorbell) | button Ø13.6, 1.5 above the housing | conservative dome: 0.15 N + 0.2 N/mm pre-travel to 1.0 mm, rise to P at 1.2, snap to 0.43·P at 1.25, bottom at 1.6; 150 Hz 0.12 N click pulse | — (cap springs back) |

P = mapped in-peak, P_out = mapped out-peak (table above).

## Timing (like the paper: onset → cessation of the task force)
- **First action** (in / close / press): starts when the object force first exceeds 0.05 N, stops when
  the goal is reached (key at the shoulder, plug seated, card taken by the reader, tines 12 mm in,
  zipper at the top stop, button clicked). Hunting for the keyhole counts.
- **Second action** (out / open): starts at the button press on the object, stops when it is out
  (8 mm clear) / fully open.

## Output (`data/`)
- `adl_tasks_<stamp>.csv` — every servo sample (1 kHz), standard columns. `phase` = 10·task + stage
  (task 1 fork … 7 remote; stage 1 in hand, 2 pushing/closing, 3 done/easing off, 4 left in place,
  5 pulling/opening, 6 ATM reader moving the card, 7 waiting for the zipper tab); `trial` = cycle.
- `adl_tasks_<stamp>_trials.csv` — one row per timed action:
  `mode, cycle, rep, task_id, task, action, time_s, t_start_s, t_goal_s,
  robot_ref_mean_s, robot_ref_sd_s, robot_t1_slow_s, robot_t2_med_s, robot_t3_fast_s, ratio_to_robot_mean,
  real_peak_N (paper), sim_peak_target_N, sim_peak_profile_N (peak while moving along the profile),
  sim_peak_any_N (incl. pushing on walls), implied_real_peak_N (profile peak mapped back),
  pinch_req_mu056_N, pinch_req_mu033_N, max_hand_force_N, path_mm, jammed_s, force_gain, rotation_on, simulator`
- `.info.txt` — device, safety limits, force map and per-task peaks.

## Validation (simulator, 2026-10-19)
**Self-test** (`--selftest`, simulated arm 0.12 kg + 0.15 N/mm hand, scripted autopilot incl. a 1.2 mm
off-centre key (guided in by the chamfer), a far off-centre key (must be blocked) and a 25°-twisted plug
(must jam)): all 7 tasks × 3 cycles and the 21-cycle protocol pass; measured profile peaks within 1–4 % of
target; max force change 0.095 N/tick; no NaN.

**Live simulator run** (full protocol, 39 timed actions, 400 k samples): max force change 0.081 N/tick,
max speed 284 mm/s, no trips. Profile peaks mapped back to real newtons:

| Action | Simulated → real (N) | Paper (N) |
|---|---|---|
| Key in / out | 4.2 / 3.6 | 4.1 / 3.4 |
| Plug in / out | 21 / 16 | 20.7 / 15.6 |
| V-zipper close / open | 4.6 / 2.3 | 4.5 / 2.2 |
| H-zipper close / open | 12.5 / 6.8 | 12.2 / 6.2 |
| ATM in / out | 6.0 / 4.7 | 5.9 / 4.5 |
| Fork in | 7.3–8.7 | 6.8 |
| Remote | 1.1–1.2 | 1.4 |

## Limitations (read before using the times)
- **Robot reference times**, not human: and the simulated objects are position-only with fixed orientation
  (rotation is a visual/jam constraint), with no real grip — so times are not directly comparable.
- **Zipper tracks are shortened** (45 / 60 mm) to fit the arm's workspace; real zippers are longer, so zipper
  times are not comparable to the robot's at all. Profiles are per mm (real tooth pitch).
- **Forces are compressed** into the arm's range (map above); stiff mechanisms feel spongier, since rendered
  stiffness is ≤ 0.25 N/mm (e.g. the button needs ~5 mm of hand travel to click).
- **Shapes are partly engineering estimates** where no data exists (pin bump widths, putty viscosity, ATM
  shutter position, zipper ripple); peaks are from the paper. Fork-out has no paper value.
- **Not yet run on the real arm.** Start with `--force-gain 0.5`, one task at a time, and confirm the roll axis.

## Sources for geometry and profile shapes
Kwikset KW1 spacing/depths: lockwiki.com (Kwikset Classic); Schlage: Allegion KB "combinating rule 15".
NEMA 5-15 dimensions: ANSI/NEMA WD-6 (via Wikipedia "NEMA connector"); UL 498 insertion/retention limits
(IAEI Magazine, "A journey back to basics with receptacles"). ID-1 card: ISO/IEC 7810. ATM motorized reader
behaviour: US7195172, WO2001084486A1. Zipper chain widths: ceozipper.com, commercialtoolry.com; operating force
test method ASTM D2062 / D4465. Tactile switch curve: Same Sky TS13 datasheet, Electronic Design.
Putty viscoelasticity: Corman & Ewoldt, Appl. Rheol. 29:141 (2019).
