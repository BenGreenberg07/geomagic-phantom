// axis_check - confirms the device frame and buttons. NO FORCES, ever.
//
// Run this once on a new device (or after rewiring / a new end effector). It
// walks you through a few moves and says whether the device matches what the
// rest of this repo assumes:
//   +x = right, +y = up, +z = toward you (facing the device), buttons 1 and 2.
//
// Steps (ENTER to go on at each one):
//   1. put the stylus in the middle of the workspace          -> start point
//   2. move RIGHT about 5 cm                                  -> checks +x
//   3. back to the middle, then UP about 5 cm                 -> checks +y
//   4. back to the middle, then TOWARD YOU about 5 cm         -> checks +z
//   5. press stylus button 1, then button 2 (10 s each)       -> button bits
//   6. twist and tilt the stylus for 5 s                      -> gimbal angles
// At the end it prints a PASS/CHECK summary and the ranges seen.
//
// Output: data/axis_check_<stamp>.csv (every servo sample; trial = step 0-2 axes, 3-4 buttons, 5 gimbal)
//
// Options: --device "<name>"  --skip-calibration  --move 50 (mm asked for)
// Keys: Q = quit at any prompt.

#include <cmath>
#include <cstdio>

#include "phantom.h"
#include "util.h"

using phantom::State;
using phantom::Vec3;

namespace {

// Waits for ENTER while showing the live position. Returns false on Q / end of input.
bool waitEnterLive(phantom::Device& dev, const char* prompt) {
    std::printf("\n%s\n", prompt);
    for (;;) {
        if (console::keyPressed()) {
            int c = console::readKey();
            if (c == 'q' || c == 'Q' || c == 27 || c < 0) return false;
            if (c == '\n' || c == '\r') { std::printf("\n"); return true; }
        }
        State s = dev.latest();
        std::printf("\r  pos %7.1f %7.1f %7.1f mm   (ENTER when there, Q quits) ", s.pos.x, s.pos.y, s.pos.z);
        std::fflush(stdout);
        util::sleepMs(30);
    }
}

// Which axis moved most (0/1/2) and in which direction (+1/-1).
void dominant(const Vec3& d, int& axis, int& sign) {
    double a[3] = {std::fabs(d.x), std::fabs(d.y), std::fabs(d.z)};
    axis = (a[0] >= a[1] && a[0] >= a[2]) ? 0 : (a[1] >= a[2] ? 1 : 2);
    double v = axis == 0 ? d.x : axis == 1 ? d.y : d.z;
    sign = v >= 0 ? 1 : -1;
}

struct Ranges {
    Vec3 pmin{1e9, 1e9, 1e9}, pmax{-1e9, -1e9, -1e9};
    double gmin[3] = {1e9, 1e9, 1e9}, gmax[3] = {-1e9, -1e9, -1e9};
    double jmin[3] = {1e9, 1e9, 1e9}, jmax[3] = {-1e9, -1e9, -1e9};
    void add(const State& s) {
        pmin = Vec3(std::fmin(pmin.x, s.pos.x), std::fmin(pmin.y, s.pos.y), std::fmin(pmin.z, s.pos.z));
        pmax = Vec3(std::fmax(pmax.x, s.pos.x), std::fmax(pmax.y, s.pos.y), std::fmax(pmax.z, s.pos.z));
        for (int i = 0; i < 3; ++i) {
            gmin[i] = std::fmin(gmin[i], s.gimbal[i]); gmax[i] = std::fmax(gmax[i], s.gimbal[i]);
            jmin[i] = std::fmin(jmin[i], s.joint[i]);  jmax[i] = std::fmax(jmax[i], s.joint[i]);
        }
    }
    void resetGimbal() { for (int i = 0; i < 3; ++i) { gmin[i] = 1e9; gmax[i] = -1e9; } }
};

const double kDeg = 57.2958;

}  // namespace

int main(int argc, char** argv) {
    util::Args args(argc, argv);
    double move = args.num("--move", 50);

    phantom::Device dev;
    if (!dev.open(args.get("--device", "").c_str(), args.flag("--skip-calibration"))) return 1;
    if (!dev.start(false)) return 1;  // read-only: motors are never driven
    if (!dev.info().calibrated) std::printf("WARNING: not calibrated, positions may be offset (axes still testable).\n");

    auto csvPath = util::dataDir(argv[0]) / ("axis_check_" + util::stamp() + ".csv");
    FILE* csv = util::openCsv(csvPath);
    if (!csv) { dev.close(); return 1; }
    util::writeSampleHeader(csv);
    util::writeInfoFile(csvPath, dev, "");

    Ranges r;
    int step = -1;  // written to the CSV trial column; set on the main thread, so no servo race
    dev.startRecording();  // every servo sample -> queue -> ranges + CSV
    auto drain = [&] { State s; while (dev.popSample(s)) { r.add(s); s.trial = step; util::writeSample(csv, s, 0); } };

    const char* words[3] = {"RIGHT", "UP", "TOWARD YOU"};
    const char* expect[3] = {"+x", "+y", "+z"};
    const char* names[3] = {"x", "y", "z"};
    const char* result[3] = {"not run", "not run", "not run"};
    bool axisOk[3] = {false, false, false};
    int seen[2] = {0, 0};
    bool quit = false;
    char msg[200];

    for (int i = 0; i < 3 && !quit; ++i) {
        drain();
        step = i;
        if (!waitEnterLive(dev, "Put the stylus in the MIDDLE of the workspace and hold it still.")) { quit = true; break; }
        drain();
        Vec3 p0 = dev.latest().pos;
        std::snprintf(msg, sizeof msg, "Now move %s about %.0f mm (keep the other directions still).", words[i], move);
        if (!waitEnterLive(dev, msg)) { quit = true; break; }
        drain();
        Vec3 d = dev.latest().pos - p0;
        int axis, sign;
        dominant(d, axis, sign);
        if (d.norm() < 10) {
            result[i] = "moved < 10 mm (rerun with a bigger move)";
        } else {
            axisOk[i] = (axis == i && sign > 0);
            result[i] = axisOk[i] ? "PASS" : "CHECK";
        }
        std::printf("  moved %.0f mm, mostly along %s%s  (dx %.0f, dy %.0f, dz %.0f)  -> %s\n", d.norm(),
                    sign > 0 ? "+" : "-", names[axis], d.x, d.y, d.z, result[i]);
    }

    // Buttons: watch for each press for up to 10 s.
    for (int b = 0; b < 2 && !quit; ++b) {
        drain();
        step = 3 + b;
        std::printf("\nPress and release stylus button %d (10 s, ENTER skips).\n", b + 1);
        for (int t = 0; t < 1000; ++t) {
            if (console::keyPressed()) {
                int c = console::readKey();
                if (c == 'q' || c == 'Q' || c == 27 || c < 0) { quit = true; break; }
                if (c == '\n' || c == '\r') break;
            }
            int btn = dev.latest().buttons;
            if (btn != 0) { seen[b] = btn; break; }
            util::sleepMs(10);
        }
        if (seen[b]) {
            std::printf("  bitmask 0x%x (%s)\n", seen[b],
                        seen[b] == HD_DEVICE_BUTTON_1   ? "HD_DEVICE_BUTTON_1"
                        : seen[b] == HD_DEVICE_BUTTON_2 ? "HD_DEVICE_BUTTON_2"
                                                        : "another bit");
            for (int t = 0; t < 500 && dev.latest().buttons != 0; ++t) util::sleepMs(10);  // wait for release
        } else if (!quit) {
            std::printf("  no press seen\n");
        }
    }

    if (!quit) {
        std::printf("\nTwist and tilt the stylus for 5 seconds...\n");
        drain();
        step = 5;
        r.resetGimbal();
        for (int t = 0; t < 50; ++t) { util::sleepMs(100); drain(); }
        std::printf("  gimbal ranges (deg): %.0f  %.0f  %.0f  (each should be well above 10)\n",
                    (r.gmax[0] - r.gmin[0]) * kDeg, (r.gmax[1] - r.gmin[1]) * kDeg, (r.gmax[2] - r.gmin[2]) * kDeg);
    }

    drain();
    dev.stopRecording();
    std::fclose(csv);
    std::printf("\n================ SUMMARY ================\n");
    for (int i = 0; i < 3; ++i) std::printf(" %-11s expected %s : %s\n", words[i], expect[i], result[i]);
    std::printf(" Buttons     : 0x%x, 0x%x  (apps treat 0x%x as button 1)\n", seen[0], seen[1],
                (unsigned)HD_DEVICE_BUTTON_1);
    if (r.pmin.x < 1e8) {
        std::printf(" Position range seen (mm): x[%.0f, %.0f] y[%.0f, %.0f] z[%.0f, %.0f]\n", r.pmin.x, r.pmax.x,
                    r.pmin.y, r.pmax.y, r.pmin.z, r.pmax.z);
        std::printf(" Joint ranges seen (deg) : %.0f  %.0f  %.0f\n", (r.jmax[0] - r.jmin[0]) * kDeg,
                    (r.jmax[1] - r.jmin[1]) * kDeg, (r.jmax[2] - r.jmin[2]) * kDeg);
    }
    bool allRun = result[2] != std::string("not run");
    if (allRun && axisOk[0] && axisOk[1] && axisOk[2])
        std::printf(" All three axes match the frame the apps assume.\n");
    else if (allRun)
        std::printf(" Some axes did NOT match. Don't run force programs until this is understood.\n");
    std::printf("=========================================\n");
    std::printf("Saved %s\n", csvPath.string().c_str());
    dev.close();
    return 0;
}
