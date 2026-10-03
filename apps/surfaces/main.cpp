// surfaces - four things that feel different from a plain wall.
//
//   FRICTION BLOCK (brown, back left): slide along its top and it drags. Uses
//     the classic "stick point" friction model: the stylus is tied to a point on
//     the surface by a stiff spring; when that spring pulls harder than
//     mu * (normal force), the point slips along behind you (Coulomb friction).
//   BUMPY FLOOR (front right): an egg-crate texture, 1.2 mm high, 10 mm apart.
//   MAGNET (red dot, front left): pulls the stylus in when you get within 25 mm,
//     strongest halfway, zero at the centre, so it holds you gently in place.
//   HONEY (amber box, back right): thick fluid, pushes against your speed.
//
// Keys (click the 3D window first):
//   ENTER / button 1  place the room around the stylus
//   M                 friction: medium (0.6) / high (1.0) / none (0)
//   X, F, arrows/drag, wheel, Q: see common/viz.h
//
// Options: --k 0.25  --b 0.001  --device "<name>"  --skip-calibration  --allow-uncalibrated
// Output: data/surfaces_<stamp>.csv
//   phase: 0 no room, 1 free, 2 walls/block sides, 3 friction top, 4 bumps, 5 magnet, 6 honey

#include <cmath>
#include <cstdio>

#include "phantom.h"
#include "util.h"
#include "viz.h"

#ifdef _WIN32

using viz::State;
using viz::Vec3;

struct Surfaces : viz::SceneBase {
    const Vec3 blockC{-30, -35, -18}, blockH{22, 10, 18};       // friction block, top at y = -25
    const double bumpX0 = 0, bumpZ0 = 0;                          // bumpy floor: x > 0, z > 0
    const double bumpA = 1.2, bumpL = 10;                         // amplitude, wavelength (mm)
    const Vec3 magnetC{-30, -28, 25};
    const double magnetR = 25, magnetK = 0.12;                    // reach (mm), N/mm
    const Vec3 honeyC{33, -22, -22}, honeyH{22, 23, 20};
    const double mus[3] = {0.6, 1.0, 0.0};

    // --- servo thread
    viz::BoxProxy blockProxy;
    bool sticking = false;
    double anchorX = 0, anchorZ = 0;
    int muIdx = 0;

    struct Snap { int muIdx = 0; };

    void reset() { blockProxy.reset(); sticking = false; }
    void key(int vk) { if (vk == 'M') muIdx = (muIdx + 1) % 3; }
    void snap(Snap& o) const { o.muIdx = muIdx; }

    double bumpH(double x, double z) const {
        return bumpA * (1 + std::sin(2 * viz::kPi * x / bumpL) * std::sin(2 * viz::kPi * z / bumpL)) * 0.5;
    }
    bool inBumps(double x, double z) const { return x > bumpX0 && z > bumpZ0 && x < room.x && z < room.z; }

    Vec3 force(State& s, const Vec3& p, double dt) {
        (void)dt;
        const Vec3& v = s.vel;
        // Over the bumps the height field is the floor, so don't add the flat floor too.
        viz::Room walls = room;
        if (inBumps(p.x, p.z)) walls.floor -= 20;
        Vec3 f = viz::roomForce(walls, p, v, cursorR, k, b);
        if (f.norm() > 0) s.phase = 2;

        // Friction block: normal force from the proxy, friction only on the top face.
        int face = -1;
        Vec3 fb = blockProxy.force(p, v, blockC, blockH, cursorR, k, b, &face);
        if (face == 2 && fb.y > 0) {
            if (!sticking) { anchorX = p.x; anchorZ = p.z; sticking = true; }
            double tx = anchorX - p.x, tz = anchorZ - p.z, td = std::sqrt(tx * tx + tz * tz);
            double fmax = mus[muIdx] * fb.y, kt = k;
            if (kt * td > fmax && td > 1e-9) {  // slipping: drag the stick point along
                double keep = fmax / (kt * td);
                anchorX = p.x + tx * keep; anchorZ = p.z + tz * keep;
                tx *= keep; tz *= keep;
            }
            fb += Vec3(kt * tx, 0, kt * tz);
            s.phase = 3;
        } else {
            sticking = false;
            if (fb.norm() > 0) s.phase = 2;
        }
        f += fb;

        // Bumpy floor: a height field above the floor, pushed out along its normal.
        if (inBumps(p.x, p.z)) {
            double w = 2 * viz::kPi / bumpL;
            double surf = room.floor + bumpH(p.x, p.z);
            double pen = surf + cursorR - p.y;
            if (pen > 0) {
                double dhx = bumpA * 0.5 * w * std::cos(w * p.x) * std::sin(w * p.z);
                double dhz = bumpA * 0.5 * w * std::sin(w * p.x) * std::cos(w * p.z);
                Vec3 n(-dhx, 1, -dhz);
                n = n * (1.0 / n.norm());
                f += viz::contact(n, pen * n.y, v, k, b);
                s.phase = 4;
            }
        }

        // Magnet: pull toward the centre, fading to zero at the edge and at the centre.
        Vec3 d = magnetC - p;
        double dist = d.norm();
        if (dist < magnetR) {
            f += d * (magnetK * (1 - dist / magnetR)) - v * b;
            s.phase = 5;
        }

        // Honey: viscous drag, faded in over the outer 5 mm so the edge isn't a step.
        Vec3 q = p - honeyC;
        double depth = std::fmin(honeyH.x - std::fabs(q.x), std::fmin(honeyH.y - std::fabs(q.y), honeyH.z - std::fabs(q.z)));
        if (depth > 0) {
            double hb = maxDamping > 0 ? 0.5 * maxDamping : 0.0025;
            f += v * (-hb * viz::clampd(depth / 5.0, 0, 1));
            s.phase = 6;
        }
        return f;
    }

    void draw(const Snap&, const Vec3&) const {
        viz::drawShadow(blockC, blockH.x, room.floor);
        viz::drawBox(blockC, blockH, 0.55, 0.38, 0.25);
        // Bumps as a shaded height-field mesh.
        glBegin(GL_QUADS);
        double step = 2.0;
        for (double x = bumpX0; x < room.x - 0.01; x += step)
            for (double z = bumpZ0; z < room.z - 0.01; z += step) {
                double xs[4] = {x, x + step, x + step, x}, zs[4] = {z, z, z + step, z + step};
                for (int i = 0; i < 4; ++i) {
                    double h = bumpH(xs[i], zs[i]);
                    double c = 0.55 + 0.35 * h / bumpA;
                    glColor3d(c * 0.9, c * 0.8, c);
                    glNormal3d(0, 1, 0);
                    glVertex3d(xs[i], room.floor + h + 0.3, zs[i]);
                }
            }
        glEnd();
        viz::drawSphere(magnetC, magnetR, 0.9, 0.2, 0.2, 0.12);
        viz::drawSphere(magnetC, 3, 0.9, 0.15, 0.15);
        viz::drawShadow(magnetC, 3, room.floor);
        viz::drawBox(honeyC, honeyH, 0.95, 0.65, 0.1, 0.25);
    }

    void status(const Snap& sn, char* buf, size_t n) const {
        std::snprintf(buf, n, "friction mu %.1f (M changes) - block, bumps, magnet, honey", mus[sn.muIdx]);
    }
};

int main(int argc, char** argv) {
    Surfaces scene;
    return viz::run(argc, argv, scene, "surfaces",
                    "Friction block (back left), bumpy floor (front right), magnet (red), honey (amber). M = friction level.");
}

#else
int main() { std::printf("surfaces needs Windows (Win32 + OpenGL). Build it on the lab PC.\n"); return 0; }
#endif
