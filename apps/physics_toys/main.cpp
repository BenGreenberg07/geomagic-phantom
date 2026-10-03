// physics_toys - things that move when you touch them.
//
//   BALLS (front): four balls that roll on the floor and bump into each other,
//     the walls and the blob. Knock one into the others.
//   PENDULUM (back left): a heavy bob on a string. Tap it and feel it swing back
//     into the stylus; it is a real 3D (spherical) pendulum with gravity.
//   SOFT BLOB (back right): squishy, and the whole blob wobbles on a spring when
//     you poke it. The surface visibly dents where the stylus presses in.
//
// Keys (click the 3D window first):
//   ENTER / button 1  place the room around the stylus (also resets the toys)
//   R                 reset the toys
//   X, F, arrows/drag, wheel, Q: see common/viz.h
//
// Options: --k 0.25  --b 0.001  --device "<name>"  --skip-calibration  --allow-uncalibrated
// Output: data/physics_toys_<stamp>.csv  (phase: 0 no room, 1 free, 2 touching)

#include <cmath>
#include <cstdio>

#include "phantom.h"
#include "util.h"
#include "viz.h"

#ifdef _WIN32

using viz::State;
using viz::Vec3;

struct PhysicsToys : viz::SceneBase {
    static const int kBalls = 4;
    const double ballR = 9, ballMass = 0.03;
    const Vec3 pivot{-30, 42, -18};
    const double penL = 55, bobR = 9, bobMass = 0.05;
    const Vec3 blobRest{32, -27, -15};
    const double blobR = 18, blobMass = 0.05, blobSpring = 0.15;  // N/mm holding the blob in place

    Vec3 ballStart(int i) const {
        static const double xs[kBalls] = {-24, -8, 8, 24};
        return Vec3(xs[i], room.floor + ballR, 22 + (i % 2) * 6);
    }

    // --- servo thread
    Vec3 balls[kBalls], ballVel[kBalls];
    Vec3 bob, bobVel;
    Vec3 blob, blobVel;

    struct Snap {
        Vec3 balls[kBalls], bob, blob;
    };

    void reset() {
        for (int i = 0; i < kBalls; ++i) { balls[i] = ballStart(i); ballVel[i] = Vec3(); }
        bob = pivot + Vec3(0, -penL, 0); bobVel = Vec3();
        blob = blobRest; blobVel = Vec3();
    }
    void key(int vk) { if (vk == 'R') reset(); }
    void snap(Snap& o) const {
        for (int i = 0; i < kBalls; ++i) o.balls[i] = balls[i];
        o.bob = bob; o.blob = blob;
    }

    // Stylus vs a moving sphere: force on the stylus; the opposite goes to the sphere.
    Vec3 touch(const Vec3& p, const Vec3& v, const Vec3& c, const Vec3& cv, double R, double kk, Vec3& onSphere) const {
        Vec3 d = p - c;
        double dist = d.norm();
        onSphere = Vec3();
        if (dist >= R + cursorR || dist < 1e-6) return Vec3();
        Vec3 f = viz::contact(d * (1.0 / dist), R + cursorR - dist, v - cv, kk, b);
        onSphere = f * -1.0;
        return f;
    }

    Vec3 force(State& s, const Vec3& p, double dt) {
        const Vec3& v = s.vel;
        Vec3 f = viz::roomForce(room, p, v, cursorR, k, b), on;

        // --- balls (roll on the floor: x and z only)
        for (int i = 0; i < kBalls; ++i) {
            f += touch(p, v, balls[i], ballVel[i], ballR, k, on);
            ballVel[i].x += (1000.0 * on.x / ballMass - 2.5 * ballVel[i].x) * dt;
            ballVel[i].z += (1000.0 * on.z / ballMass - 2.5 * ballVel[i].z) * dt;
        }
        for (int i = 0; i < kBalls; ++i)
            for (int j = i + 1; j < kBalls; ++j) collide2d(balls[i], ballVel[i], balls[j], ballVel[j], 2 * ballR);
        for (int i = 0; i < kBalls; ++i) {
            double sp = std::sqrt(ballVel[i].x * ballVel[i].x + ballVel[i].z * ballVel[i].z);
            if (sp > 800) ballVel[i] = ballVel[i] * (800 / sp);
            balls[i].x += ballVel[i].x * dt;
            balls[i].z += ballVel[i].z * dt;
            balls[i].y = room.floor + ballR;
            bounceWalls(balls[i], ballVel[i], ballR);
            pushOffBlob(balls[i], ballVel[i]);
        }

        // --- pendulum: gravity + stylus, then keep the string length (spherical pendulum)
        f += touch(p, v, bob, bobVel, bobR, k, on);
        bobVel += (on * (1000.0 / bobMass) + Vec3(0, -9810, 0) - bobVel * 0.3) * dt;
        bob += bobVel * dt;
        Vec3 r = bob - pivot;
        double len = r.norm();
        if (len > 1e-6) {
            Vec3 u = r * (1.0 / len);
            bob = pivot + u * penL;
            bobVel = bobVel - u * viz::dot(bobVel, u);  // no stretching along the string
        }

        // --- soft blob: soft contact, and the blob centre sits on a damped spring
        f += touch(p, v, blob, blobVel, blobR, 0.35 * k, on);
        Vec3 spring = (blobRest - blob) * blobSpring - blobVel * 0.0015;
        blobVel += (on + spring) * (1000.0 / blobMass) * dt;
        blob += blobVel * dt;
        return f;
    }

    // Two balls on the floor: separate them and swap the normal velocity (restitution 0.8).
    static void collide2d(Vec3& a, Vec3& va, Vec3& c, Vec3& vc, double minD) {
        double dx = c.x - a.x, dz = c.z - a.z, d = std::sqrt(dx * dx + dz * dz);
        if (d >= minD || d < 1e-6) return;
        dx /= d; dz /= d;
        double push = (minD - d) / 2;
        a.x -= dx * push; a.z -= dz * push;
        c.x += dx * push; c.z += dz * push;
        double rel = (vc.x - va.x) * dx + (vc.z - va.z) * dz;
        if (rel < 0) {
            double j = -0.9 * rel;  // (1 + e) / 2 with e = 0.8
            va.x -= dx * j; va.z -= dz * j;
            vc.x += dx * j; vc.z += dz * j;
        }
    }
    void bounceWalls(Vec3& c, Vec3& vel, double R) const {
        if (c.x < -room.x + R) { c.x = -room.x + R; if (vel.x < 0) vel.x *= -0.6; }
        if (c.x > room.x - R)  { c.x = room.x - R;  if (vel.x > 0) vel.x *= -0.6; }
        if (c.z < -room.z + R) { c.z = -room.z + R; if (vel.z < 0) vel.z *= -0.6; }
        if (c.z > room.z - R)  { c.z = room.z - R;  if (vel.z > 0) vel.z *= -0.6; }
    }
    void pushOffBlob(Vec3& c, Vec3& vel) const {
        double dx = c.x - blob.x, dz = c.z - blob.z, d = std::sqrt(dx * dx + dz * dz);
        double minD = blobR + ballR - 3;
        if (d >= minD || d < 1e-6) return;
        dx /= d; dz /= d;
        c.x = blob.x + dx * minD; c.z = blob.z + dz * minD;
        double vn = vel.x * dx + vel.z * dz;
        if (vn < 0) { vel.x -= 1.6 * vn * dx; vel.z -= 1.6 * vn * dz; }
    }

    // Sphere mesh whose vertices are pushed out of the stylus ball (the dent).
    void drawBlob(const Vec3& c, const Vec3& cursor) const {
        const int nu = 32, nv = 20;
        double dentR = cursorR + 0.5;
        auto vert = [&](int i, int j, Vec3& pos, Vec3& nrm) {
            double th = viz::kPi * j / nv, ph = 2 * viz::kPi * i / nu;
            nrm = Vec3(std::sin(th) * std::cos(ph), std::cos(th), std::sin(th) * std::sin(ph));
            pos = c + nrm * blobR;
            Vec3 d = pos - cursor;
            double dd = d.norm();
            if (dd < dentR) {
                // slide the vertex toward the centre until it is just outside the stylus
                Vec3 in = nrm * -1.0;
                for (int it = 0; it < 12 && (pos - cursor).norm() < dentR; ++it) pos += in * 0.8;
            }
        };
        glColor3d(0.75, 0.35, 0.85);
        glBegin(GL_QUADS);
        for (int j = 0; j < nv; ++j)
            for (int i = 0; i < nu; ++i) {
                int ii[4] = {i, i + 1, i + 1, i}, jj[4] = {j, j, j + 1, j + 1};
                for (int q = 0; q < 4; ++q) {
                    Vec3 pos, nrm;
                    vert(ii[q], jj[q], pos, nrm);
                    glNormal3d(nrm.x, nrm.y, nrm.z);
                    glVertex3d(pos.x, pos.y, pos.z);
                }
            }
        glEnd();
    }

    void draw(const Snap& sn, const Vec3& cursor) const {
        static const double col[kBalls][3] = {{0.3, 0.85, 0.4}, {0.95, 0.75, 0.2}, {0.3, 0.6, 0.95}, {0.95, 0.4, 0.4}};
        for (int i = 0; i < kBalls; ++i) {
            viz::drawShadow(sn.balls[i], ballR * 0.9, room.floor);
            viz::drawSphere(sn.balls[i], ballR, col[i][0], col[i][1], col[i][2]);
        }
        viz::drawLine(pivot, sn.bob, 0.85, 0.85, 0.85, 2);
        viz::drawBox(pivot + Vec3(0, 2, 0), Vec3(6, 2, 6), 0.4, 0.4, 0.45);
        viz::drawShadow(sn.bob, bobR * 0.9, room.floor);
        viz::drawSphere(sn.bob, bobR, 0.8, 0.8, 0.85);
        viz::drawShadow(sn.blob, blobR * 0.9, room.floor);
        drawBlob(sn.blob, cursor);
    }

    void status(const Snap&, char* buf, size_t n) const {
        std::snprintf(buf, n, "knock the balls, swing the pendulum, poke the blob - R resets");
    }
};

int main(int argc, char** argv) {
    PhysicsToys scene;
    return viz::run(argc, argv, scene, "physics_toys",
                    "Balls (front), pendulum (back left), soft blob (back right). R = reset.");
}

#else
int main() { std::printf("physics_toys needs Windows (Win32 + OpenGL). Build it on the lab PC.\n"); return 0; }
#endif
