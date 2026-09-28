// template - START HERE for a new program. Copy this whole folder:
//
//     apps/template  ->  apps/my_new_thing
//
// then edit main.cpp and run build.bat. You get bin\my_new_thing.exe.
// No Visual Studio project files, no copying into the OpenHaptics examples folder.
//
// This example renders a virtual FLOOR 20 mm below wherever the stylus is when
// you press the button. Push down into it and you feel it push back.
//
//   button 1  place the floor (20 mm below the stylus)
//   Q         quit
//
// The three things to change for your own program are marked  <-- EDIT

#include <cstdio>

#include "phantom.h"
#include "util.h"

using phantom::State;
using phantom::Vec3;

int main(int argc, char** argv) {
    util::Args args(argc, argv);

    // <-- EDIT 1: your parameters. HD units: mm, N, so stiffness is N/mm.
    double k = args.num("--k", 0.3);  // floor stiffness, N/mm
    double b = 0.001;                 // damping inside the floor, N*s/mm (kills buzzing)

    phantom::Device dev;
    if (!dev.open(args.get("--device", "").c_str(), args.flag("--skip-calibration"))) return 1;
    // Too stiff = vibration ("spazzing"). Stay well under what the device can render.
    if (dev.info().maxStiffness > 0 && k > 0.5 * dev.info().maxStiffness) k = 0.5 * dev.info().maxStiffness;

    // <-- EDIT 2: state your force law needs. Only touched inside the servo thread.
    bool floorPlaced = false;
    double floorY = 0;
    bool prevBtn = false;

    // <-- EDIT 3: the force law. Runs ~1000x/s in the servo thread.
    // Rules: be fast; no printf, no file I/O, no new/delete, no locks.
    dev.setForceFunction([&](State& s) -> Vec3 {
        if (s.button1() && !prevBtn) {  // button just pressed
            floorY = s.pos.y - 20.0;
            floorPlaced = true;
        }
        prevBtn = s.button1();

        if (!floorPlaced) return Vec3();
        double depth = floorY - s.pos.y;  // > 0 means we are inside the floor
        if (depth <= 0) return Vec3();
        double fy = k * depth - b * s.vel.y;  // spring out + damping
        return Vec3(0, fy > 0 ? fy : 0, 0);   // a floor can only push, never pull
    });

    if (!dev.start(true)) return 1;
    dev.enableForces(true);
    std::printf("Press the stylus button to place a floor 20 mm below. Q = quit.\n");

    auto csvPath = util::dataDir(argv[0]) / ("template_" + util::stamp() + ".csv");
    FILE* csv = util::openCsv(csvPath);
    if (!csv) return 1;
    util::writeSampleHeader(csv);
    dev.startRecording();

    for (;;) {
        State s;
        while (dev.popSample(s)) util::writeSample(csv, s, 0);
        if (console::keyPressed()) {
            int c = console::readKey();
            if (c == 'q' || c == 'Q' || c == 27 || c < 0) break;
        }
        if (dev.tripped() != phantom::Trip::None) {
            std::printf("\nSAFETY TRIP: %s\n", phantom::tripName(dev.tripped()));
            break;
        }
        State now = dev.latest();
        std::printf("\ry=%7.1f mm  force=%5.2f N ", now.pos.y, now.force.norm());
        std::fflush(stdout);
        util::sleepMs(20);
    }

    dev.enableForces(false);
    dev.stopRecording();
    std::fclose(csv);
    dev.close();
    std::printf("\nSaved %s\n", csvPath.string().c_str());
    return 0;
}
