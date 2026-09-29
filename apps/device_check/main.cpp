// device_check - run this FIRST every session.
//
// Prints device info, calibrates, then shows live position / speed / gimbal /
// buttons / servo rate. No forces unless you press S.
//
//   S  toggle a very weak spring anchored where the arm is right now
//      (checks that forces push the right way; should feel like a soft pull back)
//   Q  quit
//
// Options: --device "<name>"  --skip-calibration  --k <N/mm> (spring, default 0.05)
//          --allow-uncalibrated (let S turn the spring on even if calibration failed)

#include <cstdio>

#include "phantom.h"
#include "util.h"

using phantom::Vec3;

int main(int argc, char** argv) {
    util::Args args(argc, argv);
    double k = args.num("--k", 0.05);
    bool allowUncal = args.flag("--allow-uncalibrated");

    phantom::Device dev;
    if (!dev.open(args.get("--device", "").c_str(), args.flag("--skip-calibration"))) return 1;

    // Clamp the test spring to 20% of what the device claims it can render.
    if (dev.info().maxStiffness > 0 && k > 0.2 * dev.info().maxStiffness) k = 0.2 * dev.info().maxStiffness;

    // Written from the main thread only through runInServo(), read in the servo loop.
    Vec3 anchor;
    dev.setForceFunction([&](phantom::State& s) { return phantom::springDamper(s, anchor, Vec3(), k, 0.0); });
    if (!dev.start(true)) return 1;

    std::printf("Live readout. S = toggle weak spring (k=%.3f N/mm), Q = quit.\n\n", k);
    bool springOn = false;
    for (;;) {
        if (console::keyPressed()) {
            int c = console::readKey();
            if (c == 'q' || c == 'Q' || c == 27 || c < 0) break;
            if ((c == 's' || c == 'S') && !springOn && !dev.info().calibrated && !allowUncal) {
                std::printf("\nNot calibrated, so the spring stays off (rerun and calibrate, or pass --allow-uncalibrated).\n");
            } else if (c == 's' || c == 'S') {
                springOn = !springOn;
                if (springOn) {
                    dev.resetTrip();
                    Vec3 here = dev.latest().pos;  // never call latest() inside runInServo
                    dev.runInServo([&] { anchor = here; });
                }
                dev.enableForces(springOn);
                std::printf("\nspring %s\n", springOn ? "ON (anchored here)" : "OFF");
            }
        }
        phantom::State s = dev.latest();
        std::printf("\rpos %7.1f %7.1f %7.1f mm | speed %6.0f mm/s | gimbal %6.1f %6.1f %6.1f deg | "
                    "btn %d%d | F %.2f N | %4.0f Hz ",
                    s.pos.x, s.pos.y, s.pos.z, s.vel.norm(), s.gimbal[0] * 57.2958, s.gimbal[1] * 57.2958,
                    s.gimbal[2] * 57.2958, s.button1(), s.button2(), s.force.norm(), dev.servoRateHz());
        std::fflush(stdout);
        if (dev.tripped() != phantom::Trip::None) {
            std::printf("\nSAFETY TRIP: %s. Forces off. Press S to re-arm.\n", phantom::tripName(dev.tripped()));
            dev.enableForces(false);
            springOn = false;
            dev.resetTrip();
        }
        util::sleepMs(50);
    }
    std::printf("\n");
    if (dev.lastError() != HD_SUCCESS) std::printf("Last HD error seen: %s\n", hdGetErrorString(dev.lastError()));
    dev.close();
    return 0;
}
