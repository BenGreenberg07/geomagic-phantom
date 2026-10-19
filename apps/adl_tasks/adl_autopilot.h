#pragma once
// adl_autopilot.h - a scripted "hand" that performs every ADL task, for testing.
//
// Used two ways: by the offline self-test (main.cpp --selftest, simulated arm, no
// window) and by the simulator (ADL_AUTOPILOT=1, drives the mock hand via simHand).
// It only reads the View (plus the models' constant geometry) and returns where the
// hand should be, whether the button is pressed, and a stylus twist (for rotation tests).
//
// It deliberately makes mistakes on some repetitions so the tests cover them:
//   key,  rep 1: 1.2 mm off-centre (inside the chamfer) - must be guided in
//   key,  rep 2: far off-centre (outside the capture)   - must be blocked, then corrected
//   plug, rep 2: twisted 25 deg (tolerance 16 deg)      - must jam, then straightened
//   zipper:      pulls along the track
//   remote:      presses straight down

#include <cmath>

#include "adl_core.h"

namespace adl {

struct Autopilot {
    int lastTask = -1, lastStage = -1, lastCycle = -1;
    int sub = 0;
    double tSub = 0, tLast = -1;
    Vec3 target;
    bool haveTarget = false;
    double pressUntil = -1;
    Vec3 pullGoal, holdPt;
    // blocked-entry checks (the test asserts these)
    int blockedChecks = 0, blockedOk = 0, testedCycle = -1;
    Vec3 retreatLat;
    bool blockTest = false, rollTest = false;
    double roll = 0;

    void moveToward(const Vec3& goal, double speed, double dt) {
        Vec3 d = goal - target;
        double n = d.norm(), lim = speed * dt;
        target = n > lim ? target + d * (lim / n) : goal;
    }

    // Returns true while it is in control; fills the hand target (room coords), buttons and twist.
    bool drive(const AdlCore& c, const View& v, double t, Vec3& outTarget, int& buttons, double& outRoll) {
        double dt = tLast < 0 ? 0 : std::fmin(0.05, t - tLast);
        tLast = t;
        if (!haveTarget) { target = v.p; haveTarget = true; }
        if (v.task != lastTask || v.cycle != lastCycle) {
            sub = 0; tSub = t; lastTask = v.task; lastCycle = v.cycle;
            blockTest = rollTest = false; roll = 0;
        }
        if (v.stage != lastStage) { lastStage = v.stage; tSub = t; if (v.stage == HAND) sub = 0; }
        buttons = 0;
        double since = t - tSub;
        Vec3 tipOff = v.objTip - v.p;  // where the held object's tip is relative to the stylus

        if (v.pauseLeft > 0 || v.protoDone) {   // between tasks: hold still
            outTarget = target; outRoll = 0; return true;
        }

        auto pressAt = [&](const Vec3& h) {
            moveToward(h, 120, dt);
            if ((v.p - h).norm() < 2.0 && pressUntil < 0) pressUntil = t + 0.12;
        };

        if (v.task == KEY || v.task == PLUG || v.task == ATM || v.task == FORK) {
            Vec3 E, n, u;
            double pushDepth;
            if (v.task == FORK) {
                double lat = -10 + 10 * (v.rep % 3);
                E = c.fork.top + Vec3(lat, 0, 0); n = c.fork.n; u = Vec3(1, 0, 0); pushDepth = c.fork.goalDepth;
            } else {
                const Insertion& ins = v.task == KEY ? c.key : v.task == PLUG ? c.plug : c.atm;
                E = ins.E; n = ins.n; u = ins.u;
                pushDepth = v.task == ATM ? 30 : ins.dMax;
                if (v.stage == HAND && sub == 0 && since < 0.02 && testedCycle != v.cycle) {
                    testedCycle = v.cycle;  // each mistake is tried once per cycle
                    blockTest = (v.task == KEY && v.rep == 2);
                    rollTest = (v.task == PLUG && v.rep == 2);
                }
            }
            Vec3 lateral;
            if (v.task == KEY && v.rep == 1) lateral = u * 1.2;
            if (blockTest) lateral = u * 6.0;
            // twist only after the trial has zeroed the stylus roll (people start straight, then twist)
            roll = (rollTest && (sub > 0 || since > 0.2)) ? 25 * 3.14159265 / 180 : 0;

            if (v.stage == HAND || v.stage == PUSHING) {
                if (sub == 0) {                                   // line up in front of the entry
                    Vec3 g = E - n * 10 - tipOff + lateral;
                    moveToward(g, 150, dt);
                    if ((target - g).norm() < 0.01 && since > 0.3) { sub = 1; tSub = t; }
                } else if (sub == 1) {                            // push in (overshoot: the hand spring needs it)
                    Vec3 g = E + n * (pushDepth + 25) - tipOff + lateral;
                    moveToward(g, 50, dt);
                    if ((blockTest || rollTest) && since > 1.4) {
                        blockedChecks++;
                        if (v.stage == HAND) blockedOk++;            // must NOT have entered
                        retreatLat = lateral;
                        blockTest = rollTest = false;
                        sub = 2; tSub = t;
                    }
                } else if (sub == 2) {                            // back straight out (still off-centre / twisted)
                    Vec3 g = E - n * 8 - tipOff + retreatLat;
                    moveToward(g, 80, dt);
                    if ((target - g).norm() < 0.01 && (v.p - g).norm() < 1.5) { sub = 3; tSub = t; }
                } else if (sub == 3) {                            // then line up properly and try again
                    Vec3 g = E - n * 8 - tipOff;
                    moveToward(g, 40, dt);
                    if ((target - g).norm() < 0.01 && since > 0.3) { sub = 1; tSub = t; }
                }
            } else if (v.stage == DONE_IN) {                      // goal reached: ease off so the object is left
                moveToward(v.p, 200, dt);
            } else if (v.stage == MACHINE) {                      // ATM took the card: step back once
                if (since < 0.05) holdPt = v.p - n * 12;
                moveToward(holdPt, 60, dt);
            } else if (v.stage == LEFT) {                         // go to the object and press the button
                pressAt(v.handle);
                pullGoal = v.handle - n * (pushDepth + 45);
                if (v.task == ATM) pullGoal = v.handle - n * 100;
            } else if (v.stage == PULLING) {
                moveToward(pullGoal, 60, dt);
            }
        } else if (v.task == VZIP || v.task == HZIP) {
            const Zipper& z = v.task == VZIP ? c.vzip : c.hzip;
            if (v.stage == WAITING) {
                moveToward(v.handle, 120, dt);
            } else if (v.stage == PUSHING) {  // keep pulling a tab's length ahead of the slider
                moveToward(z.s0 + z.t * std::fmin(z.L + z.tabLen + 30, v.zipS + z.tabLen + 30) + z.nf * 5, 30, dt);
            } else if (v.stage == DONE_IN) {
                moveToward(v.p, 200, dt);
            } else if (v.stage == LEFT) {
                pressAt(v.handle);
            } else if (v.stage == PULLING) {
                moveToward(z.s0 + z.t * std::fmax(-z.tabLen - 30, v.zipS - z.tabLen - 30) + z.nf * 5, 30, dt);
            }
        } else {  // REMOTE
            const Remote& r = c.remote;
            if (v.stage == HAND || v.stage == PUSHING) {
                if (sub == 0) {
                    Vec3 g = r.btn + Vec3(0, 14, 0);
                    moveToward(g, 150, dt);
                    if ((target - g).norm() < 0.01 && since > 0.3) { sub = 1; tSub = t; }
                } else {
                    moveToward(Vec3(r.btn.x, r.capTop0() - 16, r.btn.z), 25, dt);  // press firmly (hand spring needs it)
                }
            } else {
                moveToward(r.btn + Vec3(0, 16, 0), 60, dt);
            }
        }
        if (pressUntil >= 0) {
            buttons = 1;
            if (t > pressUntil) pressUntil = -1;
        }
        outTarget = target;
        outRoll = roll;
        return true;
    }
};

}  // namespace adl
