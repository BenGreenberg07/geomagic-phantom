#pragma once
// adl_models.h - force models for the 7 ADL tasks of Smaby et al. 2004 (JRRD 41(2):215-224).
//
// No graphics here and no device calls: every model is a pure function of the stylus
// state, so the same code runs in the 1 kHz servo loop, in the offline self-test
// (--selftest) and in the simulator. No allocation, no locks, no printing.
//
// HOW EVERY TASK WORKS (the "god-object" / proxy method)
//   The task object (key, plug, card, fork, zipper tab, button cap) has a PROXY: where
//   the object really is, kept inside what the world allows (it can't pass through the
//   door, it slides only along the keyway...). The stylus says where the object WANTS
//   to be. The hand feels a stiff spring (kc, <= 0.3 N/mm) between the two, so the
//   rendered stiffness is always safe. Along the task axis the proxy only advances when
//   the spring force beats the task's resistance profile R(depth) - so the felt force
//   IS the profile (pins, contact springs, shutters, putty, zipper teeth) - and the proxy
//   moves at most vmax per tick, so clicks and snaps are crisp but never one-tick steps.
//
// FORCES: Smaby's peaks are object forces measured by a robot (mean + 2 SD), up to
// 20.7 N; the arm is limited to 3.68 N. One curve maps them all, keeping the order:
//     F_sim = 3.0 * (F_real / 20.7)^0.4        F_real = 20.7 * (F_sim / 3.0)^2.5
// Every profile below is a SHAPE (from standards/datasheets, see docs) scaled to that
// task's mapped peak. Logged forces can be turned back into real newtons with the inverse.
//
// Units: mm, s, N. Frame: +x right, +y up, +z toward the user. Room coordinates
// (origin = where the stylus was when the room was placed).

#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "phantom.h"

namespace adl {

using phantom::Vec3;

inline double dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline double clampd(double x, double lo, double hi) { return x < lo ? lo : (x > hi ? hi : x); }
inline double smooth(double a, double b, double x) {  // smoothstep 0..1 between a and b
    double t = clampd((x - a) / (b - a), 0, 1);
    return t * t * (3 - 2 * t);
}
inline double lerp(double a, double b, double t) { return a + (b - a) * t; }

// ---- force map (real object force -> simulated, and back) ----
const double kMapTop = 3.0, kMapRef = 20.7, kMapExp = 0.4;
inline double toSim(double fReal) { return fReal <= 0 ? 0 : kMapTop * std::pow(fReal / kMapRef, kMapExp); }
inline double toReal(double fSim) { return fSim <= 0 ? 0 : kMapRef * std::pow(fSim / kMapTop, 1.0 / kMapExp); }

// ---- the task table (Smaby et al. 2004, Table 1 + Fig. 3 / Table 3) ----
enum TaskId { FORK = 0, KEY, PLUG, VZIP, HZIP, ATM, REMOTE, NTASK };

struct RefTime { double mean, sd, t1, t2, t3; };  // robot times, s (Time 1/2/3 = slow/medium/fast)
struct TaskInfo {
    const char* name;      // short name for the HUD/CSV
    const char* inName;    // name of the first (push / close) action
    const char* outName;   // name of the second action (pull / open), "" if none in the paper
    double pinch_mm;       // pinch opening
    double realIn, realOut;  // peak object force, N (mean + 2 SD; recovered from the pinch ranges)
    RefTime refIn, refOut;   // robot reference times
    const char* desc;
};

inline const TaskInfo& info(int t) {
    static const TaskInfo T[NTASK] = {
        {"FORK", "FORK IN", "FORK OUT", 2.0, 6.8, 0.0, {2.0, 0.4, 2.3, 2.3, 1.5}, {0, 0, 0, 0, 0},
         "4-tine fork into Air Putty, medium soft"},
        {"KEY", "KEY IN", "KEY OUT", 2.4, 4.1, 3.4, {1.7, 0.8, 2.6, 1.7, 0.8}, {1.9, 0.7, 2.6, 2.1, 1.1},
         "Standard key into an entrance lock"},
        {"PLUG", "PLUG IN", "PLUG OUT", 22.9, 20.7, 15.6, {1.5, 0.4, 1.6, 1.3, 0.9}, {1.1, 0.5, 1.6, 1.1, 0.6},
         "3-prong plug into wall outlet"},
        {"VERTICAL ZIPPER", "V-ZIPPER CLOSE", "V-ZIPPER OPEN", 1.6, 4.5, 2.2, {11.4, 4.1, 15.7, 12.4, 6.2},
         {15.6, 6.3, 22.4, 16.3, 7.3}, "5 mm closure width, 2 cm zipper tab"},
        {"HORIZONTAL ZIPPER", "H-ZIPPER CLOSE", "H-ZIPPER OPEN", 1.8, 12.2, 6.2, {8.6, 3.0, 13.8, 9.2, 4.5},
         {9.2, 4.0, 11.5, 9.5, 4.7}, "8 mm closure width, 1.5 cm zipper tab"},
        {"ATM CARD", "ATM IN", "ATM OUT", 0.7, 5.9, 4.5, {3.8, 1.6, 5.5, 3.7, 2.3}, {3.0, 1.7, 4.8, 2.9, 1.4},
         "Standard card into (actual) ATM mechanical device"},
        {"REMOTE BUTTON", "BUTTON PRESS", "", 22.0, 1.4, 0.0, {0.9, 0.2, 1.1, 0.9, 0.7}, {0, 0, 0, 0, 0},
         "Doorbell remote, button diameter 13.6 mm"},
    };
    return T[t];
}
inline bool paperHasOut(int t) { return info(t).realOut > 0; }

// ---- per-tick input / output ----
struct Input {
    Vec3 p, v;          // stylus position (room coords, mm) and velocity (mm/s)
    bool btn = false;   // stylus button 1 held
    bool press = false; // button 1 just went down this tick
    double roll = 0;    // stylus twist about its axis since the trial started, rad
    bool rollOn = true; // rotation tolerance active
    double kc = 0.25;   // coupling stiffness N/mm
    double bc = 0.0015; // coupling damping N*s/mm
    double gain = 1.0;  // force gain (--force-gain), scales every profile
    double cursorR = 4; // stylus ball radius
    double t = 0, dt = 0.001;
};

// Stages shared by all tasks (also written to the CSV phase column).
enum Stage {
    HAND = 1,     // object in the hand / approaching (first action not started)
    PUSHING = 2,  // first action under way (engaged, closing, pressing)
    DONE_IN = 3,  // first action complete, object still held (waiting for the hand to ease off)
    LEFT = 4,     // object left in place; press the button on it to start the second action
    PULLING = 5,  // second action under way (attached again)
    MACHINE = 6,  // ATM reader moving the card by itself
    WAITING = 7,  // zipper: move to the tab;  remote: lift the finger
};

struct Events {          // edges, set for exactly one tick
    bool goalIn = false;   // first action complete
    bool regrip = false;   // second action started (button on the object)
    bool goalOut = false;  // second action complete
    bool click = false;    // remote: the dome snapped
};

struct Output {
    Vec3 f;              // force on the hand, N (before the scene gain ramp)
    double objF = 0;     // simulated force on the object along the task axis, N (for logging/pinch)
    bool onProfile = false;  // the object is moving along its task profile (not just pushing on a wall)
    bool contact = false;
    Events ev;
};

// ---- coupling: proxy <-> stylus spring, with attach/detach fades and a step limit ----
struct Coupler {
    double alpha = 0, alphaT = 0;  // current / target attachment 0..1 (40 ms fades)
    Vec3 corr, lastProxy, vProxy;
    bool have = false;
    void reset() { alpha = 0; alphaT = 0; have = false; corr = Vec3(); vProxy = Vec3(); }
    // proxy: where the object is; desired: where the stylus wants it; v: stylus velocity.
    Vec3 force(const Vec3& proxy, const Vec3& desired, const Vec3& v, const Input& in) {
        Vec3 c = proxy - desired;
        if (have) {
            // limit how fast the correction (= force) can change: <= 0.4 mm/tick (0.1 N at 0.25 N/mm)
            Vec3 dc = c - corr;
            double n = dc.norm(), lim = 0.4;
            if (n > lim) c = corr + dc * (lim / n);
            // the proxy moves in small stick-slip jumps: smooth its velocity (~16 Hz) before damping on it,
            // otherwise each jump looks like a velocity spike and kicks the hand
            Vec3 vp = (proxy - lastProxy) * (1.0 / in.dt);
            vProxy = vProxy + (vp - vProxy) * 0.1;
        }
        lastProxy = proxy;
        corr = c;
        have = true;
        double step = in.dt / 0.04;
        alpha = alphaT > alpha ? std::fmin(alphaT, alpha + step) : std::fmax(alphaT, alpha - step);
        Vec3 f = c * in.kc;
        double cn = c.norm();
        if (cn > 0.05) {  // damp only along the correction, and only while there is one
            Vec3 nc = c * (1.0 / cn);
            f += nc * (dot(vProxy - v, nc) * in.bc * clampd(cn / 0.5, 0, 1));
        }
        return f * alpha;
    }
    bool attached() const { return alphaT > 0.5; }
};

// God-object step along one axis: proxy depth sp chases the desired depth sd, but only
// when the spring force kc*(sd - sp) beats the resistance window [-rOut, +rIn].
// bv > 0 adds a viscous (Bingham) rate limit: advance speed = excess force / bv.
inline double proxyStep(double sp, double sd, double rIn, double rOut, double kc, double dt, double vmax,
                        double bv = 0) {
    double fp = kc * (sd - sp);
    double adv = 0;
    if (fp > rIn) adv = (fp - rIn) / kc;
    else if (fp < -rOut) adv = (fp + rOut) / kc;
    if (bv > 0) {
        double vv = std::fabs(fp > 0 ? fp - rIn : fp + rOut) / bv * dt;
        if (std::fabs(adv) > vv) adv = adv > 0 ? vv : -vv;
    }
    double lim = vmax * dt;
    return sp + clampd(adv, -lim, lim);
}

// ============================================================================
// INSERTION (key, plug, ATM card): approach -> chamfer -> channel -> stop; then out.
// ============================================================================
struct Insertion {
    // --- configuration (constant after setup) ---
    int id = KEY;
    Vec3 E;            // entry point (centre of the hole at the start of the channel)
    Vec3 n{0, 0, -1};  // insertion direction (into the wall)
    Vec3 u{1, 0, 0}, w{0, 1, 0};  // lateral axes
    double tolU = 0.3, tolW = 0.5;  // lateral clearance in the channel, mm
    double cham = 2.5;              // chamfer / funnel depth (capture radius), mm
    double dMax = 27, dSeat = 26.5; // hard stop, seated depth
    double dClear = 8;              // pulled this far out = done
    Vec3 tipDefault{0, 0, -40};     // object tip relative to the stylus when held
    double rollTol = 10 * 3.14159265 / 180;
    double pin = 0, pout = 0;       // mapped peaks (sim N) for in / out
    // --- state ---
    int stage = HAND;
    bool engaged = false, jam = false;
    double sp = 0, spPrev = 0;      // proxy depth along n (tip), mm
    double latU = 0, latW = 0;      // proxy lateral offset (channel)
    Vec3 tipOff, proxyTip;
    Coupler cp;
    int atm = 0;                    // ATM reader: 0 idle, 1 accepting, 2 intake, 3 inside, 4 eject, 5 presented
    double atmT = 0;
    bool goalInDone = false;
    bool rotBlocked = false, onFace = false;  // approach: twist block (with hysteresis), touching the flat face
    bool haveProxy = false;                   // proxy rate limit state
    Vec3 lastProxyW, lastD;

    void setup(int task) {
        id = task;
        pin = toSim(info(task).realIn);
        pout = toSim(info(task).realOut);
        if (task == KEY) {  // Kwikset KW1: 5 pins, 3.81 mm spacing, blade 27 x 8.5 x 2.1, keyway ~2.3 x 8.7
            E = Vec3(14, -6, -48); tolU = 0.25; tolW = 0.4; cham = 2.5; dMax = 27; dSeat = 26.6;
            tipDefault = Vec3(0, 0, -40); rollTol = 10 * 3.14159265 / 180;
        } else if (task == PLUG) {  // NEMA 5-15: ground pin leads by 3.2 mm; blades 15.9 long
            E = Vec3(-12, -8, -52); tolU = 0.4; tolW = 0.4; cham = 1.8; dMax = 19.1; dSeat = 18.7;
            tipDefault = Vec3(0, 0, -42); rollTol = 16 * 3.14159265 / 180;  // paper: 16 deg plug tolerance
        } else {  // ATM: ID-1 card 85.6 x 54 x 0.76, short edge first into a ~58 x 3 mm slot with a flared bezel
            E = Vec3(0, -4, -86); u = Vec3(1, 0, 0); w = Vec3(0, 1, 0);
            tolU = 2.0; tolW = 1.1; cham = 4.0; dMax = 85.6; dSeat = 1e9;
            tipDefault = Vec3(0, 0, -80.6); rollTol = 12 * 3.14159265 / 180;
        }
        reset();
    }
    void reset() {
        stage = HAND; engaged = false; jam = false; sp = spPrev = 0; latU = latW = 0;
        tipOff = tipDefault; cp.reset(); cp.alphaT = 1; atm = 0; atmT = 0; goalInDone = false;
        proxyTip = Vec3();
        rotBlocked = false; onFace = false; haveProxy = false;
    }
    Vec3 objTip() const { return E + n * sp + u * latU + w * latW; }  // where the object is when not held
    bool readerMoving() const { return id == ATM && atm >= 2 && atm <= 4; }  // the ATM is moving the card itself
    Vec3 handle() const { return (cp.attached() ? proxyTip : objTip()) - tipDefault; }  // where the stylus holds it

    // Resistance windows (sim N at gain 1). In = while pushing in, Out = while pulling out.
    double rIn(double d) const {
        if (id == KEY) {  // floor + 5 pin bumps (rise 1.5 mm, fall 0.5 mm), growing toward the back pins
            double r = 0.16 * pin;
            for (int j = 0; j < 5; ++j) {
                double c = 6.27 + 3.81 * j, h = pin * (0.6 + 0.1 * j), b = 0;
                if (d > c - 1.5 && d <= c) b = h * (d - (c - 1.5)) / 1.5;
                else if (d > c && d < c + 0.5) b = h * (1 - (d - c) / 0.5);
                if (b > r) r = b;
            }
            return r;
        }
        if (id == PLUG) {  // ground pin only; blade contacts spread (early peak); sliding plateau to the seat
            if (d < 3.2) return 0.08 * pin;
            if (d < 6.2) return lerp(0.08 * pin, pin, smooth(3.2, 6.2, d));
            if (d < 9.2) return lerp(pin, 0.67 * pin, smooth(6.2, 9.2, d));
            return 0.67 * pin;
        }
        // ATM: low friction, shutter/gate ramp at 10-20 mm, then the reader takes over
        if (d < 10) return 0.1 * pin;
        if (d < 20) return lerp(0.1 * pin, pin, (d - 10) / 10);
        return 0.33 * pin;
    }
    double rOut(double d) const {
        if (id == KEY) {  // mirror image of the pins at 0.83 (3.4 / 4.1 N)
            double r = 0.16 * pout;
            for (int j = 0; j < 5; ++j) {
                double c = 6.27 + 3.81 * j, h = pout * (0.6 + 0.1 * j), b = 0;
                if (d < c + 1.5 && d >= c) b = h * ((c + 1.5) - d) / 1.5;
                else if (d < c && d > c - 0.5) b = h * (1 - (c - d) / 0.5);
                if (b > r) r = b;
            }
            return r;
        }
        if (id == PLUG) {  // breakaway from the seat, sliding grip, small snap as the blades leave
            if (d > 17.6) return pout;
            if (d > 14.6) return lerp(0.6 * pout, pout, (d - 14.6) / 3.0);
            if (d > 3.2) return 0.6 * pout;
            return 0.06 * pout;
        }
        // ATM card out: front rollers break away (card presented at 55.6), free within ~10 mm
        if (d > 45.6) return lerp(0.04 * pout, pout, clampd((d - 45.6) / 10.0, 0, 1));
        return 0.04 * pout;
    }

    Output step(const Input& in) {
        Output o;
        double g = in.gain;
        spPrev = sp;
        bool held = cp.attached();
        Vec3 D = in.p + tipOff;  // where the stylus wants the tip
        Vec3 proxy = D;

        // ATM reader runs on its own clock
        if (id == ATM && atm >= 2) {
            atmT += in.dt;
            if (atm == 2) {                              // intake at 190 mm/s
                sp = std::fmin(dMax, sp + 190 * in.dt);
                if (sp >= dMax) { atm = 3; atmT = 0; if (!goalInDone) { o.ev.goalIn = true; goalInDone = true; } }
            } else if (atm == 3 && atmT > 1.5) {         // eject after 1.5 s
                atm = 4;
            } else if (atm == 4) {
                sp = std::fmax(55.6, sp - 190 * in.dt);  // card stops with 30 mm sticking out (reachable)
                if (sp <= 55.6) { atm = 5; stage = LEFT; }
            }
        }

        if (held) {
            Vec3 rel = D - E;
            double dd = dot(rel, n), eu = dot(rel, u), ew = dot(rel, w);
            jam = false;
            if (!engaged) {
                // Rotation block, with 1.5 deg hysteresis so a hand hovering at the limit doesn't flicker.
                double ar = std::fabs(in.roll);
                if (!in.rollOn) rotBlocked = false;
                else if (ar > rollTol) rotBlocked = true;
                else if (ar < rollTol - 0.026) rotBlocked = false;
                // Lateral excess r = distance from the tip to the hole's cross-section (a rectangle
                // tolU x tolW), Euclidean so the corners are rounded and the push direction never flips.
                double exU = std::fabs(eu) - tolU, exW = std::fabs(ew) - tolW;
                double cu = clampd(eu, -tolU, tolU), cw = clampd(ew, -tolW, tolW);  // nearest point of the hole
                double r, du = 0, dw = 0;
                if (exU > 0 || exW > 0) {
                    double ru = std::fmax(exU, 0), rw = std::fmax(exW, 0);
                    r = std::sqrt(ru * ru + rw * rw);
                    du = (eu > 0 ? 1 : -1) * ru / r; dw = (ew > 0 ? 1 : -1) * rw / r;
                } else {
                    r = std::fmax(exU, exW);
                }
                // In the (r, depth) plane the wall is solid where depth > -min(r, cham): a 45-degree
                // funnel from the hole's mouth (r = 0, depth 0) out to r = cham, then the flat face.
                double tu = eu, tw = ew, td = dd;  // target for the object's tip (lateral u, w and depth)
                int surf = 0;                      // 0 free, 1 flat face, 2 funnel slope
                if (rotBlocked) {
                    // twisted too far: it can't enter the funnel; the whole entrance acts as a flat face
                    if (dd > -cham) { td = -cham; surf = 1; jam = true; }
                } else if (r <= 0) {
                    if (dd > 0) engaged = true;  // straight into the mouth
                } else if (dd > -std::fmin(r, cham)) {
                    double t = (r - dd) / 2;     // projection onto the slope: point (t, -t)
                    bool face = r >= cham && (onFace || t > cham);
                    if (face) { td = -cham; surf = 1; }
                    else if (t <= 0) engaged = true;  // slid down the slope to the mouth: in
                    else {
                        t = std::fmin(t, cham);
                        tu = cu + du * t; tw = cw + dw * t; td = -t; surf = 2;
                    }
                }
                onFace = surf == 1;
                if (engaged) {  // enter at the mouth: the depth then advances along the profile from 0
                    sp = spPrev = 0; latU = cu; latW = cw;
                } else {
                    proxy = E + n * td + u * tu + w * tw;
                }
            }
            if (engaged) {
                latU = clampd(eu, -tolU, tolU);
                latW = clampd(ew, -tolW, tolW);
                if (!readerMoving()) {
                    double ri = g * rIn(sp), ro = g * rOut(sp);
                    sp = proxyStep(sp, dd, ri, ro, in.kc, in.dt, 300);
                    if (sp > dMax) sp = dMax;
                }
                if (sp < 0 && !readerMoving()) { engaged = false; sp = 0; }
                proxy = E + n * sp + u * latU + w * latW;
                o.objF = std::fabs(in.kc * (dd - sp));
                o.contact = true;
            }
            // The object can keep up with the hand, plus slide at most 60 mm/s (400 while engaged, where
            // the profile already moves it smoothly) when the contact changes surface, e.g. slipping off
            // an edge while pressing hard: a quick slide instead of a jump.
            if (haveProxy) {
                double lim = (D - lastD).norm() + (engaged ? 400.0 : 60.0) * in.dt;
                Vec3 dp = proxy - lastProxyW;
                double m = dp.norm();
                if (m > lim) proxy = lastProxyW + dp * (lim / m);
            }
            haveProxy = true; lastProxyW = proxy; lastD = D;
            if (!engaged) {
                o.contact = (proxy - D).norm() > 1e-6;
                if (o.contact) o.objF = in.kc * (proxy - D).norm();
            }
            proxyTip = proxy;
            o.f = cp.force(proxy, D, in.v, in);
            o.objF = std::fabs(dot(o.f, n));          // what the hand actually puts on the object
            o.onProfile = engaged;
            double push = in.kc * dot(D - proxy, n);  // > 0 = the hand pushing in

            if (stage == HAND && engaged) stage = PUSHING;
            if (stage == PUSHING && !engaged && sp <= 0) stage = HAND;  // backed out before finishing
            if (id != ATM && stage == PUSHING && sp >= dSeat) { stage = DONE_IN; o.ev.goalIn = true; goalInDone = true; }
            if (id == ATM && stage == PUSHING && atm == 0 && sp >= 20) { atm = 1; atmT = 0; }
            if (id == ATM && atm == 1) {
                atmT += in.dt;
                if (atmT > 0.1) { atm = 2; atmT = 0; }
            }
            if (id == ATM && atm == 2 && stage != PULLING) {
                // the reader pulls the card away from the fingers: let go after a short tug
                if (atmT > 0.08 || -push > 0.6 * g * pin) { cp.alphaT = 0; stage = MACHINE; }
            }
            if (stage == DONE_IN && push < 0.3 * g) { cp.alphaT = 0; stage = LEFT; }  // hand eased off: let go
            if (stage == PULLING && !engaged && dd < -dClear) {
                o.ev.goalOut = true;
                stage = HAND; atm = 0; goalInDone = false; sp = 0;
            }
        } else {
            // Not held: the stylus is a free ball; it can rest against the wall / fascia.
            double db = dot(in.p - E, n) + in.cursorR;  // ball front relative to the entry point
            if (db > -cham) { o.f = o.f + n * (-(in.kc) * (db + cham)); o.contact = true; }
            proxyTip = objTip();
            cp.force(proxyTip, proxyTip, in.v, in);  // keep fading out
            if (stage == LEFT && in.press && (in.p - handle()).norm() < 15) {
                tipOff = objTip() - in.p;   // grip it exactly where it is: no force step
                cp.alphaT = 1; cp.have = false; cp.corr = Vec3();
                stage = PULLING; engaged = true; o.ev.regrip = true;
                haveProxy = false;
            }
        }
        return o;
    }
};

// ============================================================================
// FORK into putty: tines down into a viscoplastic disc, stays stuck, pull out.
// ============================================================================
struct Fork {
    Vec3 top{0, -78, -8};      // centre of the putty's top surface
    Vec3 n{0, -1, 0};
    double radius = 30, thick = 20, goalDepth = 12, dClear = 8;
    Vec3 tipDefault{0, -70, 0};
    double pin = 0;
    struct Dent { double x, z, depth; };
    Dent dents[32];
    int nDents = 0, dentNext = 0;
    // state
    int stage = HAND;
    bool engaged = false;
    double sp = 0, spPrev = 0, maxDepth = 0;
    double ancX = 0, ancZ = 0;  // lateral anchor inside the putty
    Vec3 tipOff, proxyTip;
    Coupler cp;
    bool inDent = false;
    double dentDepth = 0;

    void setup() { pin = toSim(info(FORK).realIn); reset(); nDents = 0; dentNext = 0; }
    void reset() {
        stage = HAND; engaged = false; sp = spPrev = maxDepth = 0; tipOff = tipDefault;
        cp.reset(); cp.alphaT = 1; proxyTip = Vec3();
    }
    Vec3 objTip() const { return top + n * sp + Vec3(ancX, 0, ancZ); }
    Vec3 handle() const { return (cp.attached() ? proxyTip : objTip()) - tipDefault; }

    double yield(double d) const {
        double y = pin * (0.10 + 0.25 * clampd(d, 0, 12) / 12);
        if (inDent && d < dentDepth) y *= 0.4;
        return y;
    }
    double visc(double d) const {  // N*s/mm on the proxy (Maxwell element in series with kc: always passive)
        double b = pin * 0.65 / 20.0 * (0.3 + 0.7 * clampd(d, 0, 12) / 12);
        if (inDent && d < dentDepth) b *= 0.4;
        return b;
    }

    Output step(const Input& in) {
        Output o;
        double g = in.gain;
        spPrev = sp;
        bool held = cp.attached();
        Vec3 D = in.p + tipOff, proxy = D;
        if (held) {
            double dd = dot(D - top, n);
            double rx = D.x - top.x, rz = D.z - top.z, rr = std::sqrt(rx * rx + rz * rz);
            if (!engaged) {
                if (rr < radius - 6 && dd > 0) {  // tines enter the putty
                    engaged = true; sp = std::fmin(dd, 0.5); spPrev = sp; ancX = rx; ancZ = rz;
                    inDent = false;
                    for (int i = 0; i < nDents; ++i)
                        if (std::hypot(dents[i].x - rx, dents[i].z - rz) < 2.0) { inDent = true; dentDepth = dents[i].depth; }
                } else if (dd > thick - 1) {    // outside the disc: the dish under it
                    proxy = D + n * ((thick - 1) - dd);
                    o.contact = true; o.objF = in.kc * (dd - (thick - 1));
                }
            }
            if (engaged) {
                // Lateral: the tines plough through the putty (stick-slip anchor)
                double lx = rx - ancX, lz = rz - ancZ, ln = std::sqrt(lx * lx + lz * lz);
                double ly = g * pin * (0.12 + 0.03 * sp);
                if (in.kc * ln > ly && ln > 1e-9) { double keep = ly / (in.kc * ln); ancX = rx - lx * keep; ancZ = rz - lz * keep; }
                // Axial: Bingham putty - yield + viscous rate limit; suction while pulling out
                double suction = (stage == PULLING || dd < sp) ? 0.3 * g * std::exp(-(maxDepth - sp) / 3.0) : 0;
                double ri = g * yield(sp), ro = g * 0.3 * yield(sp) + suction;
                // pulling out flows easier than stabbing in (no paper value; ~0.8 N with suction)
                double bv = g * visc(sp) * (dd < sp ? 0.35 : 1.0) + 1e-4;
                sp = proxyStep(sp, dd, ri, ro, in.kc, in.dt, 250, bv);
                if (sp > thick - 2) sp = thick - 2;  // dish bottom
                if (sp > maxDepth) maxDepth = sp;
                if (sp < 0) {  // tines out: leave a dent
                    engaged = false;
                    Dent& dn = dents[dentNext];
                    dn.x = ancX; dn.z = ancZ; dn.depth = maxDepth;
                    dentNext = (dentNext + 1) % 32;
                    if (nDents < 32) nDents++;
                    sp = 0;
                }
                proxy = top + n * sp + Vec3(ancX, 0, ancZ);
                if (engaged) { o.objF = std::fabs(in.kc * (dd - sp)); o.contact = true; }
            }
            proxyTip = proxy;
            o.f = cp.force(proxy, D, in.v, in);
            o.objF = std::fabs(dot(o.f, n));
            o.onProfile = engaged;
            double push = in.kc * dot(D - proxy, n);
            if (stage == HAND && engaged) stage = PUSHING;
            if (stage == PUSHING && !engaged) stage = HAND;
            if (stage == PUSHING && sp >= goalDepth) { stage = DONE_IN; o.ev.goalIn = true; }
            if (stage == DONE_IN && push < 0.3 * g) { cp.alphaT = 0; stage = LEFT; }
            if (stage == PULLING && !engaged && dd < -dClear) { o.ev.goalOut = true; stage = HAND; maxDepth = 0; }
        } else {
            // free ball: rests on the putty / dish surface
            double db = dot(in.p - top, n) + in.cursorR;
            if (db > 0) { o.f = n * (-in.kc * db); o.contact = true; }
            proxyTip = objTip();
            cp.force(proxyTip, proxyTip, in.v, in);
            if (stage == LEFT && in.press && (in.p - handle()).norm() < 15) {
                tipOff = objTip() - in.p;
                cp.alphaT = 1; cp.have = false; cp.corr = Vec3();
                stage = PULLING; engaged = true; o.ev.regrip = true;
            }
        }
        return o;
    }
};

// ============================================================================
// ZIPPER: pull the tab; the slider runs along the track (closing, then opening).
// ============================================================================
// Track lengths are shortened to fit the arm's workspace (tab swing adds ~2 x tab length of
// hand travel); real jacket / bag zippers are longer, so zipper times are not comparable to the
// paper's robot times. The force profile (breakaway, tooth ripple at the real pitch) is per mm.
struct Zipper {
    bool vertical = true;
    Vec3 s0, t;                 // track start (open end), direction toward the closed end
    Vec3 nf{0, 0, 1};           // fabric normal (toward the user)
    double L = 70, pitch = 1.8, tabLen = 20, ripple = 0.10;
    double pin = 0, pout = 0;
    // state
    int stage = WAITING;
    double s = 0;               // slider position 0..L
    Vec3 tabEnd;
    Coupler cp;

    void setup(bool vert) {
        vertical = vert;
        if (vert) {  // #5 coil on a jacket: chain 5.8 mm, tab 20 mm, closing upward
            s0 = Vec3(0, -26, -24); t = Vec3(0, 1, 0); L = 45; pitch = 1.8; tabLen = 20; ripple = 0.10;
            pin = toSim(info(VZIP).realIn); pout = toSim(info(VZIP).realOut);
        } else {     // #8 molded on a backpack: chain 7.6 mm, tab 15 mm, closing to the right
            s0 = Vec3(-32, 2, -24); t = Vec3(1, 0, 0); L = 60; pitch = 3.4; tabLen = 15; ripple = 0.20;
            pin = toSim(info(HZIP).realIn); pout = toSim(info(HZIP).realOut);
        }
        reset();
    }
    void reset() { stage = WAITING; s = 0; cp.reset(); tabEnd = tabRest(); }
    Vec3 pivot() const { return s0 + t * s + nf * 3.0; }
    Vec3 tabRest() const { return pivot() + Vec3(0, -tabLen * 0.85, 0) + nf * 2.0; }  // hanging down
    Vec3 handle() const { return cp.attached() ? tabEnd : tabRest(); }

    double rClose(double x) const {
        double r = pin * (0.75 + 0.25 * ripple * 10 * std::pow(std::sin(3.14159265 * x / pitch), 2) / 2.5);
        if (x < 2) r = pin;  // breakaway at the box
        return r;
    }
    double rOpen(double x) const {
        double r = pout * (0.70 + 0.30 * ripple * 10 * std::pow(std::sin(3.14159265 * x / pitch), 2) / 2.5);
        if (x > L - 2) r = pout;  // breakaway at the top stop
        return r;
    }

    Output step(const Input& in) {
        Output o;
        double g = in.gain;
        bool held = cp.attached();
        if (held) {
            Vec3 D = in.p;
            // The tab is a tether of length tabLen from the slider, in front of the fabric.
            auto tether = [&](const Vec3& piv) {
                Vec3 r = D - piv;
                double rn = r.norm();
                Vec3 e = rn > tabLen ? piv + r * (tabLen / rn) : D;
                double front = dot(e - s0, nf);
                if (front < 1.0) e = e + nf * (1.0 - front);
                return e;
            };
            Vec3 e = tether(pivot());
            Vec3 T = (D - e) * in.kc;              // pull on the tab
            double ta = dot(T, t);
            Vec3 tperp = T - t * ta;
            tperp = tperp - nf * dot(tperp, nf);   // sideways (in the fabric plane): binds the slider
            double bind = 0.3 * tperp.norm();
            // reuse the god-object step with a "desired slider position" = s + ta / kc
            s = proxyStep(s, s + ta / in.kc, g * rClose(s) + bind, g * rOpen(s) + bind, in.kc, in.dt, 300);
            s = clampd(s, 0, L);
            e = tether(pivot());
            tabEnd = e;
            o.f = cp.force(e, D, in.v, in);
            o.objF = std::fabs(dot(o.f, t));
            o.onProfile = true;
            o.contact = (D - e).norm() > 0.05;
            if (stage == PUSHING && s >= L - 1.5) { stage = DONE_IN; o.ev.goalIn = true; }
            if (stage == DONE_IN && (D - e).norm() * in.kc < 0.3 * g) { cp.alphaT = 0; stage = LEFT; }
            if (stage == PULLING && s <= 1.5) { o.ev.goalOut = true; stage = PUSHING; }  // next close starts
        } else {
            // free ball against the fabric
            double db = dot(in.p - s0, nf) - in.cursorR;
            if (db < 0) { o.f = nf * (-in.kc * db); o.contact = true; }
            cp.force(tabEnd, tabEnd, in.v, in);
            tabEnd = tabRest();
            double dist = (in.p - tabRest()).norm();
            if (stage == WAITING && dist < 8) {  // pick the tab up by touching it
                cp.alphaT = 1; cp.have = false; cp.corr = Vec3(); stage = PUSHING;
                tabEnd = in.p;
            } else if (stage == LEFT && in.press && dist < 14) {
                cp.alphaT = 1; cp.have = false; cp.corr = Vec3(); stage = PULLING; o.ev.regrip = true;
                tabEnd = in.p;
            }
        }
        return o;
    }
};

// ============================================================================
// REMOTE: press the 13.6 mm doorbell button with the fingertip (no grasp).
// ============================================================================
struct Remote {
    Vec3 body{10, -33, -12}, bodyH{17.5, 7.5, 30};  // remote lying on the table (35 x 15 x 60)
    Vec3 btn{10, -25.5, -20};                       // button centre on the top face
    double capR = 6.8, protrude = 1.5, xMax = 1.6;
    double pk = 0;
    int stage = HAND;
    double x = 0, xPrev = 0;   // cap travel, mm
    double clickT = -1;
    bool clicked = false;

    void setup() { pk = toSim(info(REMOTE).realIn); reset(); }
    void reset() { stage = HAND; x = xPrev = 0; clickT = -1; clicked = false; }
    double capTop0() const { return btn.y + protrude; }
    // Conservative dome profile (sim N): pre-travel spring, dome peak, snap, plateau, bottom.
    double R(double xx) const {
        double s = pk / 1.4;
        if (xx < 1.0) return s * (0.15 + 0.2 * xx);
        if (xx < 1.2) return lerp(s * 0.35, pk, (xx - 1.0) / 0.2);
        if (xx < 1.25) return lerp(pk, 0.43 * pk, (xx - 1.2) / 0.05);
        return 0.43 * pk;
    }

    Output step(const Input& in) {
        Output o;
        double g = in.gain;
        xPrev = x;
        Vec3 p = in.p;
        double rr = std::hypot(p.x - btn.x, p.z - btn.z);
        double sd = capTop0() + in.cursorR - p.y;  // how far the fingertip is below the cap's rest contact
        double fp = 0;
        if (rr <= capR && sd > x) {               // on the cap
            double r = g * R(x);
            fp = in.kc * (sd - x);
            double adv = fp > r ? (fp - r) / in.kc : 0;
            x = clampd(x + clampd(adv, 0, 0.3), 0, xMax);   // vmax 300 mm/s
            fp = in.kc * std::fmax(0.0, sd - x);
            o.f = Vec3(0, fp, 0);
            o.objF = fp; o.contact = true; o.onProfile = true;
        } else {
            // cap springs back (rate limited), finger elsewhere
            x = std::fmax(0.0, x - 0.3);
            // housing top and sides (box), table under it
            double top = body.y + bodyH.y;
            bool over = std::fabs(p.x - body.x) < bodyH.x && std::fabs(p.z - body.z) < bodyH.z;
            if (over && rr > capR && p.y - in.cursorR < top) {
                double pen = top - (p.y - in.cursorR);
                if (pen < 6) { o.f = Vec3(0, in.kc * pen, 0) - Vec3(0, in.v.y * in.bc, 0); o.contact = true; }
            }
            double table = body.y - bodyH.y;
            if (p.y - in.cursorR < table) { o.f += Vec3(0, in.kc * (table - (p.y - in.cursorR)), 0); o.contact = true; }
        }
        // cap return when partly unloaded: keep x where R(x) balances the finger force
        if (fp < g * R(x) && x > 0) {
            double back = (g * R(x) - fp) / in.kc;
            x = std::fmax(0.0, x - clampd(back, 0, 0.3));
        }
        if (stage == HAND && o.contact && rr <= capR) stage = PUSHING;
        if (!clicked && x >= 1.3) {  // the dome snapped: click
            clicked = true; clickT = 0; o.ev.click = true;
            if (stage == PUSHING || stage == HAND) { stage = DONE_IN; o.ev.goalIn = true; }
        }
        if (clickT >= 0) {  // tactile click: one 150 Hz cycle, 0.12 N (smooth enough for a 1 kHz loop)
            clickT += in.dt;
            if (clickT < 1.0 / 150) o.f += Vec3(0, 0.12 * g * std::sin(2 * 3.14159265 * 150 * clickT), 0);
        }
        if (clicked && x < 0.2 && sd < -2) { clicked = false; clickT = -1; stage = HAND; }  // finger lifted: ready again
        return o;
    }
};

}  // namespace adl
