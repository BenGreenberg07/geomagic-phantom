// mechanisms - a push button, a drawer and a lever, each with its own "feel".
//
//   PUSH BUTTON (left): press down on the cap. Force rises, then drops suddenly
//     at 2.5 mm (the "click", like a keyboard key), then bottoms out at 5 mm.
//     Clicks are counted.
//   DRAWER (back): hold stylus button 1 on the handle to grab it, then pull it
//     toward you along its rail (0-40 mm). You feel the rail holding you on the
//     line, a little sliding friction, the end stops, and a magnetic catch in the
//     last 6 mm. Or push the drawer front to slide it shut.
//   LEVER (right): grab the red knob with button 1 and swing it forward/back
//     (+-50 deg). It has notches at 0 and +-29 deg and hard stops at the ends.
//
// Keys (click the 3D window first):
//   ENTER     place the room around the stylus (the button is for grabbing here)
//   X, F, arrows/drag, wheel, Q: see common/viz.h
//
// Options: --k 0.25  --b 0.001  --device "<name>"  --skip-calibration  --allow-uncalibrated
// Output: data/mechanisms_<stamp>.csv
//   phase: 0 no room, 1 free, 2 touching, 3 pressing the button, 4 drawer, 5 lever
//   trial: button clicks so far

#include <cmath>
#include <cstdio>

#include "phantom.h"
#include "util.h"
#include "viz.h"

#ifdef _WIN32

using viz::State;
using viz::Vec3;

struct Mechanisms : viz::SceneBase {
    static constexpr bool kButtonPlaces = false;

    // Push button
    const Vec3 btnBaseC{-35, -40, 0}, btnBaseH{13, 5, 13};
    const Vec3 capC{-35, -30, 0}, capH{8, 5, 8};  // cap top at y = -25 when up
    // Drawer: rail along z, 0 = closed, 40 mm = fully open
    const Vec3 cabC{0, -30, -33}, cabH{20, 15, 12};      // cabinet (front at z = -21)
    const Vec3 drawerC0{0, -30, -32}, drawerH{17, 12, 14}; // drawer body when closed (front at z = -18)
    const double handleOut = 5, drawerMax = 40, drawerMass = 0.15;
    // Lever: swings in the y-z plane about an axis along x
    const Vec3 leverBaseC{35, -40, 5}, leverBaseH{10, 5, 10};
    const Vec3 pivot{35, -35, 5};
    const double leverL = 45, leverMax = 0.9, knobR = 5;  // mm, rad, mm
    const double grabDist = 10;

    Vec3 handlePos(double dz) const { return Vec3(drawerC0.x, drawerC0.y, drawerC0.z + drawerH.z + handleOut + dz); }
    Vec3 knobPos(double th) const { return pivot + Vec3(0, leverL * std::cos(th), leverL * std::sin(th)); }

    // --- servo thread
    viz::BoxProxy btnBaseProxy, capProxy, cabProxy, drawerProxy, leverBaseProxy;
    double btnDepth = 0;
    bool btnDown = false, prevBtn = false;
    int clicks = 0;
    double dz = 0, dzVel = 0, theta = 0;
    int grabbed = 0;  // 0 none, 1 drawer, 2 lever

    struct Snap {
        double btnDepth = 0, dz = 0, theta = 0;
        bool btnDown = false;
        int clicks = 0, grabbed = 0;
    };

    void reset() {
        btnBaseProxy.reset(); capProxy.reset(); cabProxy.reset(); drawerProxy.reset(); leverBaseProxy.reset();
        btnDepth = 0; btnDown = false; clicks = 0; dz = 0; dzVel = 0; theta = 0; grabbed = 0;
    }
    void snap(Snap& o) const {
        o.btnDepth = btnDepth; o.dz = dz; o.theta = theta; o.btnDown = btnDown; o.clicks = clicks; o.grabbed = grabbed;
    }

    // Button force vs. depth (mm -> N): rise, sudden drop ("click"), plateau, hard bottom.
    double buttonProfile(double d) const {
        if (d <= 0) return 0;
        if (d <= 2.5) return 0.6 * d / 2.5;
        if (d <= 3.5) return 0.6 - 0.35 * (d - 2.5);
        if (d <= 5.0) return 0.25 + 0.1 * (d - 3.5) / 1.5;
        return 0.35 + k * (d - 5.0);
    }

    Vec3 force(State& s, const Vec3& p, double dt) {
        const Vec3& v = s.vel;
        bool btn = s.button1(), press = btn && !prevBtn;
        prevBtn = btn;
        Vec3 f = viz::roomForce(room, p, v, cursorR, k, b);
        f += btnBaseProxy.force(p, v, btnBaseC, btnBaseH, cursorR, k, b);
        f += leverBaseProxy.force(p, v, leverBaseC, leverBaseH, cursorR, k, b);
        f += cabProxy.force(p, v, cabC, cabH, cursorR, k, b);
        if (f.norm() > 0) s.phase = 2;

        // --- push button: the cap is a box; pressed from the top it follows the click profile.
        int face = -1;
        Vec3 fc = capProxy.force(p, v, capC, capH, cursorR, k, b, &face);
        if (face == 2) {
            btnDepth = capH.y * 2 + cursorR - (p.y - (capC.y - capH.y));  // = penetration of the top face
            double up = buttonProfile(btnDepth) - b * v.y;
            f += Vec3(0, up > 0 ? up : 0, 0);
            if (!btnDown && btnDepth > 3.2) { btnDown = true; clicks++; }
            s.phase = 3;
        } else {
            btnDepth = 0;
            f += fc;
        }
        if (btnDown && btnDepth < 1.5) btnDown = false;

        // --- grabbing (one thing at a time, nearest within grabDist)
        if (!btn) grabbed = 0;
        if (press) {
            double dh = (p - handlePos(dz)).norm(), dk = (p - knobPos(theta)).norm();
            if (dh < grabDist && dh <= dk) grabbed = 1;
            else if (dk < grabDist) grabbed = 2;
        }

        // --- drawer
        Vec3 dc = drawerC0 + Vec3(0, 0, dz);
        if (grabbed == 1) {
            Vec3 h0 = handlePos(0);
            double want = p.z - h0.z;  // drawer opening the stylus is asking for
            dz = viz::clampd(want, 0, drawerMax);
            dzVel = v.z;
            Vec3 h = handlePos(dz);
            Vec3 fd((h.x - p.x) * k, (h.y - p.y) * k, 0);              // the rail holds you on the line
            if (want > drawerMax) fd.z -= k * (want - drawerMax);        // end stops
            if (want < 0) fd.z += k * (-want);
            fd.z -= (maxDamping > 0 ? 0.3 * maxDamping : 0.0015) * v.z; // sliding friction
            if (dz < 6) fd.z -= 0.4 * (1 - dz / 6);                      // magnetic catch near closed
            fd = fd - Vec3(v.x, v.y, 0) * b;
            f += fd;
            s.phase = 4;
        } else {
            // Free drawer: touchable box; pushing its front slides it.
            int dface = -1;
            Vec3 fd = drawerProxy.force(p, v, dc, drawerH, cursorR, k, b, &dface);
            f += fd;
            double push = (dface == 4) ? -fd.z : 0;
            if (dface >= 0) s.phase = 4;
            double catchF = dz < 6 ? -0.4 * (1 - dz / 6) : 0;
            dzVel += ((push + catchF) * 1000.0 / drawerMass - 8.0 * dzVel) * dt;
            dz += dzVel * dt;
            if (dz < 0) { dz = 0; if (dzVel < 0) dzVel = 0; }
            if (dz > drawerMax) { dz = drawerMax; if (dzVel > 0) dzVel = 0; }
        }

        // --- lever
        if (grabbed == 2) {
            double want = std::atan2(p.z - pivot.z, p.y - pivot.y);
            theta = viz::clampd(want, -leverMax, leverMax);
            Vec3 kp = knobPos(theta);
            Vec3 fl = (kp - p) * k - v * b;  // holds you on the arc, and at the stops
            static const double notches[3] = {-0.5, 0.0, 0.5};
            for (double nd : notches) {
                if (std::fabs(theta - nd) < 0.12) {
                    Vec3 tan(0, -std::sin(theta), std::cos(theta));
                    fl += tan * (-3.0 * (theta - nd));  // 3 N/rad pull into the notch
                }
            }
            f += fl;
            s.phase = 5;
        } else {
            Vec3 fk = viz::sphereForce(p, v, knobPos(theta), knobR, cursorR, k, b);
            if (fk.norm() > 0) s.phase = 2;
            f += fk;
        }

        s.trial = clicks;
        return f;
    }

    void draw(const Snap& sn, const Vec3&) const {
        // Button
        viz::drawShadow(btnBaseC, btnBaseH.x * 1.1, room.floor);
        viz::drawBox(btnBaseC, btnBaseH, 0.35, 0.35, 0.4);
        double d = viz::clampd(sn.btnDepth, 0, 6);
        Vec3 cap(capC.x, capC.y - d / 2, capC.z), capHalf(capH.x, capH.y - d / 2, capH.z);
        if (sn.btnDown) viz::drawBox(cap, capHalf, 0.2, 0.85, 0.3);
        else viz::drawBox(cap, capHalf, 0.85, 0.2, 0.2);
        // Cabinet + drawer
        viz::drawShadow(cabC, cabH.x * 1.1, room.floor);
        viz::drawBox(cabC, cabH, 0.5, 0.42, 0.32);
        viz::drawBox(drawerC0 + Vec3(0, 0, sn.dz), drawerH, 0.7, 0.58, 0.42);
        Vec3 h = handlePos(sn.dz);
        bool dg = sn.grabbed == 1;
        viz::drawBox(h, Vec3(8, 1.5, 1.5), dg ? 1.0 : 0.85, dg ? 0.85 : 0.85, dg ? 0.2 : 0.9);
        viz::drawLine(h + Vec3(-7, 0, -handleOut), h + Vec3(-7, 0, 0), 0.8, 0.8, 0.85, 3);
        viz::drawLine(h + Vec3(7, 0, -handleOut), h + Vec3(7, 0, 0), 0.8, 0.8, 0.85, 3);
        // Lever
        viz::drawShadow(leverBaseC, leverBaseH.x * 1.1, room.floor);
        viz::drawBox(leverBaseC, leverBaseH, 0.35, 0.35, 0.4);
        Vec3 kp = knobPos(sn.theta);
        viz::drawLine(pivot, kp, 0.75, 0.75, 0.8, 6);
        bool lg = sn.grabbed == 2;
        viz::drawSphere(kp, knobR, lg ? 1.0 : 0.85, lg ? 0.85 : 0.15, lg ? 0.2 : 0.15);
        viz::drawShadow(kp, knobR, room.floor);
    }

    void status(const Snap& sn, char* buf, size_t n) const {
        std::snprintf(buf, n, "clicks %d - drawer %.0f mm - lever %+.0f deg - hold button on handle/knob to grab",
                      sn.clicks, sn.dz, sn.theta * 57.2958);
    }
};

int main(int argc, char** argv) {
    Mechanisms scene;
    return viz::run(argc, argv, scene, "mechanisms",
                    "Press the button (left). Hold stylus button 1 on the drawer handle (back) or lever knob (right) to grab.");
}

#else
int main() { std::printf("mechanisms needs Windows (Win32 + OpenGL). Build it on the lab PC.\n"); return 0; }
#endif
