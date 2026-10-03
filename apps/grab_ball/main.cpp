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

#include <cstdio>

#include "phantom.h"
#include "util.h"
#include "viz.h"

#ifdef _WIN32

using viz::State;
using viz::Vec3;

struct GrabBall : viz::SceneBase {
    static constexpr bool kButtonPlaces = false;

    const Vec3 shelfC{30, -35, -10}, shelfH{16, 10, 16};  // shelf top at y = -25
    const double ballR = 10;
    const double masses[3] = {0.04, 0.08, 0.15};  // kg
    const double couplingK = 0.3;                 // N/mm, ball <-> stylus while held
    Vec3 ballStart() const { return Vec3(0, room.floor + ballR, 20); }

    // --- servo thread
    int massIdx = 1;
    Vec3 ball, ballVel;
    bool grabbed = false, prevBtn = false, onShelf = false;
    int shelfCount = 0;
    viz::BoxProxy shelfProxy;

    struct Snap {
        Vec3 ball;
        bool grabbed = false, onShelf = false;
        int massIdx = 1, shelfCount = 0;
    };

    void reset() {
        ball = ballStart(); ballVel = Vec3();
        grabbed = false; onShelf = false; shelfCount = 0;
        shelfProxy.reset();
    }
    void key(int vk) {
        if (vk >= '1' && vk <= '3') massIdx = vk - '1';
        if (vk == 'R') { ball = ballStart(); ballVel = Vec3(); grabbed = false; }
    }
    void snap(Snap& o) const {
        o.ball = ball; o.grabbed = grabbed; o.onShelf = onShelf; o.massIdx = massIdx; o.shelfCount = shelfCount;
    }

    Vec3 force(State& s, const Vec3& p, double dt) {
        bool btn = s.button1(), press = btn && !prevBtn;
        prevBtn = btn;
        double m = masses[massIdx];
        Vec3 f = viz::roomForce(room, p, s.vel, cursorR, k, b) + shelfProxy.force(p, s.vel, shelfC, shelfH, cursorR, k, b);

        Vec3 d = p - ball, onBall;  // onBall: force the stylus puts on the ball
        double dist = d.norm();
        if (press && dist < ballR + cursorR + 6) grabbed = true;
        if (!btn) grabbed = false;
        double bc = maxDamping > 0 ? 0.4 * maxDamping : 0.002;
        if (grabbed) {
            onBall = d * couplingK + (s.vel - ballVel) * bc;
            f += onBall * -1.0;
            s.phase = 3;
        } else if (dist < ballR + cursorR && dist > 1e-6) {
            Vec3 fc = viz::contact(d * (1.0 / dist), ballR + cursorR - dist, s.vel - ballVel, k, b);
            f += fc;
            onBall = fc * -1.0;
        }

        // Ball: gravity + stylus force, a little air drag. mm/s^2 = 1000 * N / kg.
        ballVel += (onBall * (1000.0 / m) + Vec3(0, -9810, 0) - ballVel * 0.5) * dt;
        double sp = ballVel.norm();
        if (sp > 1500) ballVel = ballVel * (1500 / sp);
        ball += ballVel * dt;
        collideRoom(dt);
        collideShelf();

        bool nowOnShelf = !grabbed && std::fabs(ball.x - shelfC.x) < shelfH.x && std::fabs(ball.z - shelfC.z) < shelfH.z &&
                          ball.y < shelfC.y + shelfH.y + ballR + 1.0 && std::fabs(ballVel.y) < 20;
        if (nowOnShelf && !onShelf) shelfCount++;
        onShelf = nowOnShelf;
        s.trial = shelfCount;
        return f;
    }

    void collideRoom(double dt) {
        if (ball.y < room.floor + ballR) {
            ball.y = room.floor + ballR;
            if (ballVel.y < 0) ballVel.y = -0.35 * ballVel.y;
            ballVel.x -= ballVel.x * 4.0 * dt;  // rolling friction
            ballVel.z -= ballVel.z * 4.0 * dt;
        }
        if (ball.y > room.ceil - ballR) { ball.y = room.ceil - ballR; if (ballVel.y > 0) ballVel.y *= -0.5; }
        if (ball.x < -room.x + ballR) { ball.x = -room.x + ballR; if (ballVel.x < 0) ballVel.x *= -0.5; }
        if (ball.x > room.x - ballR)  { ball.x = room.x - ballR;  if (ballVel.x > 0) ballVel.x *= -0.5; }
        if (ball.z < -room.z + ballR) { ball.z = -room.z + ballR; if (ballVel.z < 0) ballVel.z *= -0.5; }
        if (ball.z > room.z - ballR)  { ball.z = room.z - ballR;  if (ballVel.z > 0) ballVel.z *= -0.5; }
    }

    // Closest point on the shelf to the ball centre; push out along that direction.
    void collideShelf() {
        Vec3 q(viz::clampd(ball.x, shelfC.x - shelfH.x, shelfC.x + shelfH.x),
               viz::clampd(ball.y, shelfC.y - shelfH.y, shelfC.y + shelfH.y),
               viz::clampd(ball.z, shelfC.z - shelfH.z, shelfC.z + shelfH.z));
        Vec3 d = ball - q;
        double dist = d.norm();
        Vec3 n;
        if (dist < 1e-6) { n = Vec3(0, 1, 0); ball.y = shelfC.y + shelfH.y + ballR; }  // centre inside: pop to top
        else if (dist < ballR) { n = d * (1.0 / dist); ball = q + n * ballR; }
        else return;
        double vn = viz::dot(ballVel, n);
        if (vn < 0) ballVel += n * (-1.35 * vn);
        if (n.y > 0.7) { ballVel.x *= 0.996; ballVel.z *= 0.996; }  // friction on the shelf top
    }

    void draw(const Snap& sn, const Vec3&) const {
        viz::drawShadow(shelfC, shelfH.x * 1.2, room.floor);
        viz::drawShadow(sn.ball, ballR * 0.9, room.floor);
        viz::drawBox(shelfC, shelfH, 0.55, 0.38, 0.25);
        // Target square on the shelf top.
        Vec3 top(shelfC.x, shelfC.y + shelfH.y + 0.3, shelfC.z);
        double h = shelfH.x - 3;
        viz::drawLine(top + Vec3(-h, 0, -h), top + Vec3(h, 0, -h), 1, 1, 1, 2);
        viz::drawLine(top + Vec3(h, 0, -h), top + Vec3(h, 0, h), 1, 1, 1, 2);
        viz::drawLine(top + Vec3(h, 0, h), top + Vec3(-h, 0, h), 1, 1, 1, 2);
        viz::drawLine(top + Vec3(-h, 0, h), top + Vec3(-h, 0, -h), 1, 1, 1, 2);
        double shade = 1.0 - 0.3 * sn.massIdx;  // heavier = darker
        if (sn.grabbed) viz::drawSphere(sn.ball, ballR, 1.0, 0.85, 0.2);
        else viz::drawSphere(sn.ball, ballR, 0.3 * shade, 0.85 * shade, 0.4 * shade);
    }

    void status(const Snap& sn, char* buf, size_t n) const {
        static const char* w[3] = {"40 g", "80 g", "150 g"};
        std::snprintf(buf, n, "%s %s ball - on shelf %d times - 1/2/3 weight, R reset",
                      sn.grabbed ? "HOLDING" : "hold button near the", w[sn.massIdx], sn.shelfCount);
    }
};

int main(int argc, char** argv) {
    GrabBall scene;
    return viz::run(argc, argv, scene, "grab_ball",
                    "Hold stylus button 1 near the ball to pick it up; release to drop/throw. 1/2/3 = weight, R = reset.");
}

#else
int main() { std::printf("grab_ball needs Windows (Win32 + OpenGL). Build it on the lab PC.\n"); return 0; }
#endif
