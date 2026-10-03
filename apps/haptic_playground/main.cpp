// haptic_playground - a small 3D world you can SEE and FEEL.
//
// A room with a floor, walls, a hard cube (right), a soft sphere (left) and a
// ball (front, near you) you can push around. The white dot is the stylus; it
// turns red when it touches something, and its shadow on the floor helps you
// judge depth. The yellow line is the force the device is pushing on your hand.
//
// Press ENTER (or stylus button 1) and the room appears CENTERED where the
// stylus is, so nothing pushes until you reach for it. Forces fade in each time
// the room is placed. All forces go through phantom.h (clamp, fade-in, speed trip).
//
// Keys (click the 3D window first):
//   ENTER / button 1  place / re-center the room here (also resets the ball)
//   R                 put the ball back
//   X, F, arrows/drag, wheel, Q: see common/viz.h
//
// Options: --k 0.25  --b 0.001  --device "<name>"  --skip-calibration  --allow-uncalibrated
// Output: data/haptic_playground_<stamp>.csv (phase: 0 no room, 1 free, 2 touching)

#include <cmath>
#include <cstdio>

#include "phantom.h"
#include "util.h"
#include "viz.h"

#if VIZ_AVAILABLE

using viz::State;
using viz::Vec3;

struct Playground : viz::SceneBase {
    const Vec3 sphereC{-32, -27, -5};
    const double sphereR = 18;                 // soft sphere, sits on the floor
    const Vec3 cubeC{32, -30, -5}, cubeH{15, 15, 15};  // hard cube
    const double ballR = 12, ballMass = 0.03, ballDrag = 3.0;
    Vec3 ballStart() const { return Vec3(0, room.floor + ballR, 22); }  // front: the back is hard to reach

    // --- servo thread
    viz::BoxProxy cubeProxy;
    Vec3 ball, ballVel;

    struct Snap { Vec3 ball; };

    void reset() { cubeProxy.reset(); ball = ballStart(); ballVel = Vec3(); }
    void key(int vk) { if (vk == 'R') { ball = ballStart(); ballVel = Vec3(); } }
    void snap(Snap& o) const { o.ball = ball; }

    Vec3 force(State& s, const Vec3& p, double dt) {
        const Vec3& v = s.vel;
        Vec3 f = viz::roomForce(room, p, v, cursorR, k, b);
        f += viz::sphereForce(p, v, sphereC, sphereR, cursorR, 0.4 * k, b);  // soft: 40 % stiffness
        f += cubeProxy.force(p, v, cubeC, cubeH, cursorR, k, b);
        // Ball: equal and opposite contact force, then roll it along the floor.
        Vec3 d = p - ball, onBall;
        double dist = d.norm();
        if (dist < ballR + cursorR && dist > 1e-6) {
            Vec3 fc = viz::contact(d * (1.0 / dist), ballR + cursorR - dist, v - ballVel, k, b);
            f += fc;
            onBall = fc * -1.0;
        }
        stepBall(onBall, dt);
        return f;
    }

    void stepBall(const Vec3& force, double dt) {
        // mm/s^2 = 1000 * N / kg. The ball stays on the floor, so only x and z move.
        ballVel.x += (1000.0 * force.x / ballMass - ballDrag * ballVel.x) * dt;
        ballVel.z += (1000.0 * force.z / ballMass - ballDrag * ballVel.z) * dt;
        double sp = std::sqrt(ballVel.x * ballVel.x + ballVel.z * ballVel.z);
        if (sp > 800) { ballVel.x *= 800 / sp; ballVel.z *= 800 / sp; }
        ball.x += ballVel.x * dt;
        ball.z += ballVel.z * dt;
        ball.y = room.floor + ballR;
        if (ball.x < -room.x + ballR) { ball.x = -room.x + ballR; ballVel.x = 0.5 * std::fabs(ballVel.x); }
        if (ball.x > room.x - ballR)  { ball.x = room.x - ballR;  ballVel.x = -0.5 * std::fabs(ballVel.x); }
        if (ball.z < -room.z + ballR) { ball.z = -room.z + ballR; ballVel.z = 0.5 * std::fabs(ballVel.z); }
        if (ball.z > room.z - ballR)  { ball.z = room.z - ballR;  ballVel.z = -0.5 * std::fabs(ballVel.z); }
        // Bounce off the sphere (in the floor plane).
        double dx = ball.x - sphereC.x, dz = ball.z - sphereC.z, dd = std::sqrt(dx * dx + dz * dz);
        double minD = sphereR + ballR - 2;
        if (dd < minD && dd > 1e-6) {
            dx /= dd; dz /= dd;
            ball.x = sphereC.x + dx * minD; ball.z = sphereC.z + dz * minD;
            double vn = ballVel.x * dx + ballVel.z * dz;
            if (vn < 0) { ballVel.x -= 1.5 * vn * dx; ballVel.z -= 1.5 * vn * dz; }
        }
        // Bounce off the cube (in the floor plane).
        double qx = ball.x - cubeC.x, qz = ball.z - cubeC.z, hb = cubeH.x + ballR;
        if (std::fabs(qx) < hb && std::fabs(qz) < hb) {
            if (hb - std::fabs(qx) < hb - std::fabs(qz)) {
                ball.x = cubeC.x + (qx >= 0 ? hb : -hb);
                if (ballVel.x * qx < 0) ballVel.x *= -0.5;
            } else {
                ball.z = cubeC.z + (qz >= 0 ? hb : -hb);
                if (ballVel.z * qz < 0) ballVel.z *= -0.5;
            }
        }
    }

    void draw(const Snap& sn, const Vec3&) const {
        viz::drawShadow(sphereC, sphereR * 0.9, room.floor);
        viz::drawShadow(cubeC, cubeH.x * 1.1, room.floor);
        viz::drawShadow(sn.ball, ballR * 0.9, room.floor);
        viz::drawSphere(sphereC, sphereR, 0.3, 0.55, 0.95);
        viz::drawBox(cubeC, cubeH, 0.95, 0.55, 0.15);
        viz::drawSphere(sn.ball, ballR, 0.3, 0.85, 0.4);
    }

    void status(const Snap&, char* buf, size_t n) const {
        std::snprintf(buf, n, "hard cube, soft sphere, push the ball - R resets the ball");
    }
};

int main(int argc, char** argv) {
    Playground scene;
    return viz::run(argc, argv, scene, "haptic_playground",
                    "Hard cube (right), soft sphere (left), ball to push (front). R = reset ball.");
}

#else
int main() {
    std::printf("haptic_playground needs a 3D window: build on Windows, or with CMake (it fetches GLFW) elsewhere.\n");
    return 0;
}
#endif
