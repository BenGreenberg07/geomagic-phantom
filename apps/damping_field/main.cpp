// damping_field - moving through honey. The gentlest force test there is.
//
// Force = -b * velocity. Hold still and there is no force at all; move and it
// resists in proportion to your speed. Because it never stores energy (no spring),
// it can't pull or fling the arm, which makes it the safest first "real" force
// to feel on a new device. It also shows whether the velocity signal is clean:
// a buzz or grittiness here points at noisy velocity, not at your force law.
//
// Keys:
//   0      off
//   1 2 3  light / medium / heavy honey (25 / 50 / 100 % of --b-max)
//   Q      quit (forces off immediately)
//
// Options:
//   --b-max 0      N*s/mm for level 3. 0 = 50% of the device's max damping
//                  (capped there anyway; falls back to 0.002 if the device reports 0)
//   --device "<name>"  --skip-calibration  --allow-uncalibrated
// Output: data/damping_field_<stamp>.csv (phase column = level 0-3)

#include <cstdio>

#include "phantom.h"
#include "util.h"

using phantom::State;
using phantom::Vec3;

int main(int argc, char** argv) {
    util::Args args(argc, argv);

    phantom::Device dev;
    if (!dev.open(args.get("--device", "").c_str(), args.flag("--skip-calibration"))) return 1;
    if (!dev.info().calibrated && !args.flag("--allow-uncalibrated")) {
        std::printf("Refusing to drive forces on an uncalibrated device (pass --allow-uncalibrated to override).\n");
        return 1;
    }

    double cap = dev.info().maxDamping > 0 ? 0.5 * dev.info().maxDamping : 0.002;
    double bMax = args.num("--b-max", 0);
    if (bMax <= 0 || bMax > cap) bMax = cap;
    const double kLevel[4] = {0, 0.25, 0.5, 1.0};

    // Servo-thread state; main thread changes it only through runInServo.
    double b = 0, bTarget = 0;
    int level = 0;
    dev.setForceFunction([&](State& s) -> Vec3 {
        s.phase = level;
        b += (bTarget - b) * 0.005;  // glide to a new level over ~0.2 s instead of stepping
        return s.vel * (-b);
    });

    if (!dev.start(true)) return 1;
    dev.enableForces(true);

    auto csvPath = util::dataDir(argv[0]) / ("damping_field_" + util::stamp() + ".csv");
    FILE* csv = util::openCsv(csvPath);
    if (!csv) { dev.close(); return 1; }
    util::writeSampleHeader(csv);
    char extra[96];
    std::snprintf(extra, sizeof extra, "b_max_Ns_mm: %.5f\nlevels: 0 0.25 0.5 1.0\n", bMax);
    util::writeInfoFile(csvPath, dev, extra);
    dev.startRecording();

    std::printf("Damping field. Level 3 = %.4f N*s/mm (at 200 mm/s that's %.2f N).\n"
                "Keys: 0 off, 1 light, 2 medium, 3 heavy, Q quit. Starts OFF.\n\n",
                bMax, bMax * 200);
    int shown = 0;
    for (;;) {
        State s;
        while (dev.popSample(s)) util::writeSample(csv, s, 0);
        if (console::keyPressed()) {
            int c = console::readKey();
            if (c == 'q' || c == 'Q' || c == 27 || c < 0) break;
            if (c >= '0' && c <= '3') {
                int lv = c - '0';
                double nb = bMax * kLevel[lv];
                dev.runInServo([&] { level = lv; bTarget = nb; });
                shown = lv;
                std::printf("\nlevel %d  (b = %.4f N*s/mm)\n", lv, nb);
            }
        }
        if (dev.tripped() != phantom::Trip::None) {
            std::printf("\nSAFETY TRIP: %s\n", phantom::tripName(dev.tripped()));
            break;
        }
        State now = dev.latest();
        std::printf("\rlevel %d | speed %6.0f mm/s | force %5.2f N ", shown, now.vel.norm(), now.force.norm());
        std::fflush(stdout);
        util::sleepMs(30);
    }

    dev.enableForces(false);
    util::sleepMs(50);
    State s;
    while (dev.popSample(s)) util::writeSample(csv, s, 0);
    dev.stopRecording();
    std::fclose(csv);
    dev.close();
    std::printf("\nSaved %s\n", csvPath.string().c_str());
    return 0;
}
