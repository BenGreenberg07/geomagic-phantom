// grab_ball - pick up a ball, feel its weight, carry it, put it on a shelf.
//
// Hold stylus button 1 near the ball to grab it. While you hold it, the ball
// hangs from the stylus on a stiff virtual spring ("virtual coupling"), so you
// feel its weight pull down, its inertia when you swing it, and the floor or
// shelf when you set it down. Let go of the button to drop or throw it.
// Without the button you can still push it around.
//
// Keys (click the 3D window first):
//   ENTER     place the room around the stylus (the button is for grabbing here)
//   1 / 2 / 3 light (40 g) / medium (80 g) / heavy (150 g) ball
//   R         put the ball back
//   X, F, arrows/drag, wheel, Q: see common/viz.h
//
// Options: --k 0.25  --b 0.001  --device "<name>"  --skip-calibration  --allow-uncalibrated
// Output: data/grab_ball_<stamp>.csv
//   phase: 0 no room, 1 free, 2 touching, 3 holding the ball;  trial: balls put on the shelf so far
//
// =============================================================================
// HOW TO READ THIS FILE (learning notes)
// =============================================================================
// This file is the best one to learn from: it shows every piece a 3D haptic
// program needs, and nothing else. The boring parts (opening the device,
// calibration, the window, the camera, safety limits, saving the CSV) live in
// common/viz.h and common/phantom.h, and you never have to touch them.
//
// A program like this is ONE "scene": a C++ struct with a few functions that
// the framework calls for you:
//
//   force()  - called 1000 times per second. Given where the stylus is, return
//              the force (in newtons) the motors should push on your hand.
//              This is where all the "feel" lives. It also moves the ball.
//   reset()  - called when you press ENTER to place the room. Put things back.
//   key()    - called when you press a key in the 3D window.
//   snap()   - copies what the picture needs (ball position etc.) for draw().
//   draw()   - called ~60 times per second to draw the picture.
//   status() - the text in the window's title bar.
//
// TWO THREADS. The computer runs two things at the same time:
//   * the SERVO thread runs force() every 1 ms. It must be FAST: no printing,
//     no files, no waiting. If it's late, the forces feel rough.
//   * the MAIN thread draws the window and reads the keyboard (~60x/s).
// They must not read/write the same variables at the same moment. The rule here:
// variables marked "servo thread" are only touched by force()/reset()/key()/snap()
// (the framework calls key() and snap() safely inside the servo thread), and
// draw() only looks at the Snap copy. Follow that and you can't get it wrong.
//
// UNITS everywhere: millimetres (mm), seconds (s), newtons (N), kilograms (kg).
// Stiffness is N/mm (0.25 N/mm = pushing 4 mm into a wall gives 1 N back).
// AXES (facing the device): +x right, +y up, +z toward you.
//
// C++ in 30 seconds, for this file:
//   double x = 1.5;        a decimal number.     int n = 3;   a whole number.
//   bool b = true;         true/false.           const ...    can never change.
//   Vec3 v(1, 2, 3);       a 3D vector (x, y, z) from phantom.h. You can add (+),
//                          subtract (-), multiply by a number (*), and v.norm()
//                          gives its length.  v.x, v.y, v.z are the parts.
//   a += b;                same as a = a + b.
//   cond ? A : B           "if cond then A else B" in one expression.
//   //                     a comment: the computer ignores the rest of the line.
//
// TRY THIS once it builds (edit, run `build.bat grab_ball`, run bin\grab_ball.exe):
//   1. Make the ball bigger: change ballR below from 10 to 15.
//   2. Make it heavier: change 0.15 in masses[] to 0.25 (stays under the force cap).
//   3. Make it bouncier: in collideRoom(), change -0.35 to -0.8.
//   4. Turn gravity sideways for fun: change Vec3(0, -9810, 0) to Vec3(-9810, 0, 0).
// =============================================================================

// "#include" pulls in code from other files, like an import.
#include <cstdio>  // std::printf / std::snprintf (printing text)

#include "phantom.h"  // the safe device layer: Vec3, State, force limits
#include "util.h"     // small helpers (CSV files, command-line options)
#include "viz.h"      // the 3D framework: window, drawing, the scene runner

// Everything between here and "#else" is only compiled when a 3D window is
// available (Windows, or the CMake build that downloads GLFW on a Mac). Otherwise
// the build skips to the tiny main() at the bottom of the file.
#if VIZ_AVAILABLE

// Short names, so we can write "Vec3" instead of "viz::Vec3".
using viz::State;  // one 1 ms snapshot of the device: position, velocity, buttons...
using viz::Vec3;   // 3D vector (x, y, z)

// Our scene. ": viz::SceneBase" means it starts from the framework's default
// scene, which already has: room (floor and walls), cursorR (stylus radius,
// 4 mm), k (surface stiffness), b (damping), maxDamping (the device's limit).
struct GrabBall : viz::SceneBase {
    // In most 3D programs the stylus button places the room. Here we want the
    // button for grabbing, so we tell the framework "only ENTER places the room".
    static constexpr bool kButtonPlaces = false;

    // ---- Settings (const = fixed numbers that never change while running) ----
    // The shelf is a box: shelfC = its centre, shelfH = HALF its size in x, y, z.
    // Centre y = -35 and half-height 10 puts its top at -35 + 10 = -25 mm.
    const Vec3 shelfC{30, -35, -10}, shelfH{16, 10, 16};  // shelf top at y = -25
    const double ballR = 10;                      // ball radius, mm
    const double masses[3] = {0.04, 0.08, 0.15};  // kg; keys 1/2/3 pick one
    // How stiff the invisible spring between the stylus and a held ball is.
    // Stiffer = the ball follows your hand more tightly.
    const double couplingK = 0.3;                 // N/mm, ball <-> stylus while held
    // Where the ball starts: centred in x, resting on the floor, 20 mm toward you.
    // (room.floor is -45, so its centre sits at y = -45 + 10 = -35.)
    Vec3 ballStart() const { return Vec3(0, room.floor + ballR, 20); }

    // ---- State that changes while running (servo thread only, see notes) ----
    int massIdx = 1;          // which mass: 0, 1 or 2 (starts at 80 g)
    Vec3 ball, ballVel;       // ball position (mm) and velocity (mm/s)
    bool grabbed = false;     // is the ball being held right now?
    bool prevBtn = false;     // was the button down last millisecond? (to spot a new press)
    bool onShelf = false;     // is the ball resting on the shelf?
    int shelfCount = 0;       // how many times it has been put on the shelf
    viz::BoxProxy shelfProxy; // handles touching the shelf box smoothly (see viz.h)

    // ---- The copy that draw() is allowed to look at ----
    // snap() fills one of these every frame, safely, from the servo thread.
    struct Snap {
        Vec3 ball;
        bool grabbed = false, onShelf = false;
        int massIdx = 1, shelfCount = 0;
    };

    // ENTER pressed: room placed. Put everything back to the start.
    void reset() {
        ball = ballStart(); ballVel = Vec3();  // Vec3() = (0, 0, 0): not moving
        grabbed = false; onShelf = false; shelfCount = 0;
        shelfProxy.reset();
    }

    // A key was pressed in the 3D window. vk is the key's code; letters and
    // digits use their character, so '1' means the "1" key.
    void key(int vk) {
        // '1' -> 0, '2' -> 1, '3' -> 2  (characters are numbers underneath)
        if (vk >= '1' && vk <= '3') massIdx = vk - '1';
        if (vk == 'R') { ball = ballStart(); ballVel = Vec3(); grabbed = false; }
    }

    // Copy the things draw() needs into o ("Snap& o" = fill in the caller's copy).
    void snap(Snap& o) const {
        o.ball = ball; o.grabbed = grabbed; o.onShelf = onShelf; o.massIdx = massIdx; o.shelfCount = shelfCount;
    }

    // =========================================================================
    // force(): THE HEART OF THE PROGRAM. Runs every 1 ms.
    //   s  = this millisecond's device state (s.vel = stylus velocity, buttons...)
    //        We can also write tags into it (s.phase, s.trial) that get saved
    //        in the CSV, so you can tell later what was happening.
    //   p  = stylus position in mm, measured from where you pressed ENTER.
    //   dt = time since the last call, in seconds (~0.001).
    // Returns the force to push on your hand, in newtons. The framework then
    // applies the safety rules (cap at 3.68 N, fade-in, speed cut-out) for you.
    // =========================================================================
    Vec3 force(State& s, const Vec3& p, double dt) {
        // A "press" is the moment the button goes from up to down, not every
        // millisecond it is held. So compare with last millisecond.
        bool btn = s.button1(), press = btn && !prevBtn;
        prevBtn = btn;
        double m = masses[massIdx];  // current ball mass, kg

        // Start with the walls, floor and shelf. Each helper returns the force
        // that pushes the stylus back out if it has gone into that surface
        // (zero if it hasn't). Forces just add up.
        Vec3 f = viz::roomForce(room, p, s.vel, cursorR, k, b) + shelfProxy.force(p, s.vel, shelfC, shelfH, cursorR, k, b);

        // d = arrow from the ball's centre to the stylus; dist = its length.
        // onBall will hold the force the stylus puts on the ball (zero for now).
        Vec3 d = p - ball, onBall;  // onBall: force the stylus puts on the ball
        double dist = d.norm();

        // Grab if the button was JUST pressed and the stylus is close to the
        // ball (touching distance + 6 mm of slack). Let go when it's released.
        if (press && dist < ballR + cursorR + 6) grabbed = true;
        if (!btn) grabbed = false;

        // Damping for the coupling spring (it stops the held ball bouncing on
        // the spring). 40 % of what the device can handle, or a safe default.
        double bc = maxDamping > 0 ? 0.4 * maxDamping : 0.002;

        if (grabbed) {
            // HOLDING: an invisible spring ties the ball to the stylus.
            // Spring force on the ball = stiffness x stretch (d) plus damping
            // x the difference in speed. Newton's 3rd law: the stylus gets the
            // exact opposite (* -1.0). That opposite force is what you FEEL:
            // when the ball hangs below the stylus, gravity stretches the spring
            // downward, so you feel the ball's weight pulling your hand down.
            onBall = d * couplingK + (s.vel - ballVel) * bc;
            f += onBall * -1.0;
            s.phase = 3;  // tag the CSV: "holding"
        } else if (dist < ballR + cursorR && dist > 1e-6) {
            // NOT HOLDING but TOUCHING: the stylus is inside the ball's surface.
            // viz::contact pushes the stylus back out along the line between the
            // centres (d / dist is that direction with length 1), with force =
            // k x how deep it went in. The ball gets pushed the other way.
            // (dist > 1e-6 just avoids dividing by zero if they're exactly on top.)
            Vec3 fc = viz::contact(d * (1.0 / dist), ballR + cursorR - dist, s.vel - ballVel, k, b);
            f += fc;
            onBall = fc * -1.0;
        }

        // ---- Move the ball: a tiny physics simulation, one step per ms ----
        // Newton: acceleration = force / mass. In our units (N, kg, mm/s^2):
        //   a = 1000 * F / m      (x1000 because 1 m = 1000 mm)
        // plus gravity, 9.81 m/s^2 = 9810 mm/s^2 downward (-y),
        // minus a little air drag (0.5 x velocity).
        // Then velocity += a * dt, and position += velocity * dt.
        // (This simple step-by-step method is called Euler integration.)
        ballVel += (onBall * (1000.0 / m) + Vec3(0, -9810, 0) - ballVel * 0.5) * dt;
        double sp = ballVel.norm();
        if (sp > 1500) ballVel = ballVel * (1500 / sp);  // speed limit, so it can't fly off crazily
        ball += ballVel * dt;
        collideRoom(dt);   // bounce off the floor/walls (function below)
        collideShelf();    // and off the shelf

        // Is the ball resting on the shelf? Released, over the shelf in x and z,
        // sitting on the top (within 1 mm), and not moving up/down much.
        bool nowOnShelf = !grabbed && std::fabs(ball.x - shelfC.x) < shelfH.x && std::fabs(ball.z - shelfC.z) < shelfH.z &&
                          ball.y < shelfC.y + shelfH.y + ballR + 1.0 && std::fabs(ballVel.y) < 20;
        // Count it once, at the moment it lands (wasn't on the shelf, now is).
        if (nowOnShelf && !onShelf) shelfCount++;
        onShelf = nowOnShelf;
        s.trial = shelfCount;  // tag the CSV with the count so far
        return f;              // the force for this millisecond
    }

    // Keep the ball inside the room. If it has gone through a surface, put it
    // back on the surface and reverse its speed into that surface (a bounce).
    // -0.35 means it bounces back at 35 % of the speed it hit with.
    void collideRoom(double dt) {
        if (ball.y < room.floor + ballR) {        // below the floor?
            ball.y = room.floor + ballR;          // put it back on the floor
            if (ballVel.y < 0) ballVel.y = -0.35 * ballVel.y;  // bounce up a bit
            ballVel.x -= ballVel.x * 4.0 * dt;  // rolling friction
            ballVel.z -= ballVel.z * 4.0 * dt;  // (slows rolling a little every ms)
        }
        // Ceiling and the four walls: same idea, bounce at 50 %.
        if (ball.y > room.ceil - ballR) { ball.y = room.ceil - ballR; if (ballVel.y > 0) ballVel.y *= -0.5; }
        if (ball.x < -room.x + ballR) { ball.x = -room.x + ballR; if (ballVel.x < 0) ballVel.x *= -0.5; }
        if (ball.x > room.x - ballR)  { ball.x = room.x - ballR;  if (ballVel.x > 0) ballVel.x *= -0.5; }
        if (ball.z < -room.z + ballR) { ball.z = -room.z + ballR; if (ballVel.z < 0) ballVel.z *= -0.5; }
        if (ball.z > room.z - ballR)  { ball.z = room.z - ballR;  if (ballVel.z > 0) ballVel.z *= -0.5; }
    }

    // Closest point on the shelf to the ball centre; push out along that direction.
    // How: "clamp" the ball's centre into the box. That gives q, the point of the
    // box nearest the ball. If the ball centre is closer to q than its radius,
    // the ball is overlapping the box, so slide it out along the q -> ball line.
    void collideShelf() {
        Vec3 q(viz::clampd(ball.x, shelfC.x - shelfH.x, shelfC.x + shelfH.x),
               viz::clampd(ball.y, shelfC.y - shelfH.y, shelfC.y + shelfH.y),
               viz::clampd(ball.z, shelfC.z - shelfH.z, shelfC.z + shelfH.z));
        Vec3 d = ball - q;
        double dist = d.norm();
        Vec3 n;  // the direction to push the ball out (unit length)
        if (dist < 1e-6) { n = Vec3(0, 1, 0); ball.y = shelfC.y + shelfH.y + ballR; }  // centre inside: pop to top
        else if (dist < ballR) { n = d * (1.0 / dist); ball = q + n * ballR; }
        else return;  // not touching the shelf: nothing to do
        // vn = how fast the ball moves along n. Negative = moving into the shelf,
        // so remove that and bounce back a little (1.35 = remove + 35 % bounce).
        double vn = viz::dot(ballVel, n);
        if (vn < 0) ballVel += n * (-1.35 * vn);
        if (n.y > 0.7) { ballVel.x *= 0.996; ballVel.z *= 0.996; }  // friction on the shelf top
    }

    // =========================================================================
    // draw(): the picture. Runs ~60 times per second on the MAIN thread, so it
    // only uses the Snap copy (sn) and the fixed settings. The framework already
    // drew the room, and draws the stylus dot and force arrow after this.
    // Colours are (red, green, blue), each 0 to 1.
    // =========================================================================
    void draw(const Snap& sn, const Vec3&) const {
        // Shadows on the floor help your eyes judge height and depth.
        viz::drawShadow(shelfC, shelfH.x * 1.2, room.floor);
        viz::drawShadow(sn.ball, ballR * 0.9, room.floor);
        viz::drawBox(shelfC, shelfH, 0.55, 0.38, 0.25);  // brown shelf
        // Target square on the shelf top: four white lines, 0.3 mm above it so
        // they aren't hidden inside the box surface.
        Vec3 top(shelfC.x, shelfC.y + shelfH.y + 0.3, shelfC.z);
        double h = shelfH.x - 3;
        viz::drawLine(top + Vec3(-h, 0, -h), top + Vec3(h, 0, -h), 1, 1, 1, 2);
        viz::drawLine(top + Vec3(h, 0, -h), top + Vec3(h, 0, h), 1, 1, 1, 2);
        viz::drawLine(top + Vec3(h, 0, h), top + Vec3(-h, 0, h), 1, 1, 1, 2);
        viz::drawLine(top + Vec3(-h, 0, h), top + Vec3(-h, 0, -h), 1, 1, 1, 2);
        double shade = 1.0 - 0.3 * sn.massIdx;  // heavier = darker
        if (sn.grabbed) viz::drawSphere(sn.ball, ballR, 1.0, 0.85, 0.2);  // yellow while held
        else viz::drawSphere(sn.ball, ballR, 0.3 * shade, 0.85 * shade, 0.4 * shade);  // green
    }

    // Title-bar text. snprintf writes formatted text into buf: %s is replaced by
    // a piece of text, %d by a whole number, in order.
    void status(const Snap& sn, char* buf, size_t n) const {
        static const char* w[3] = {"40 g", "80 g", "150 g"};
        std::snprintf(buf, n, "%s %s ball - on shelf %d times - 1/2/3 weight, R reset",
                      sn.grabbed ? "HOLDING" : "hold button near the", w[sn.massIdx], sn.shelfCount);
    }
};

// main() is where every C++ program starts. Ours just makes the scene and hands
// it to the framework, which runs everything until you press Q. The two text
// arguments are the program name (used for the window title and the CSV file
// name) and the help line printed in the console.
int main(int argc, char** argv) {
    GrabBall scene;
    return viz::run(argc, argv, scene, "grab_ball",
                    "Hold stylus button 1 near the ball to pick it up; release to drop/throw. 1/2/3 = weight, R = reset.");
}

#else
// Not Windows (e.g. the simulator build on a Mac): just say so.
int main() { std::printf("grab_ball needs a 3D window: build on Windows, or with CMake (it fetches GLFW) elsewhere.\n"); return 0; }
#endif
