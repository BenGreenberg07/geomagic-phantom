// draw3d - paint glowing 3D tubes in the air, on an easel, or on a globe.
//
// Canvases (M cycles):
//   AIR     hold stylus button 1 and draw anywhere. A little drag steadies the hand.
//   EASEL   a sheet of paper in front of you. Press on it and it draws (no button
//           needed), with paper-like friction (stick-slip) under the pen.
//   GLOBE   a ball in the middle. Touch it and it draws on the surface.
//
// Keys (click the 3D window first):
//   ENTER   place the room around the stylus (the button is the pen here)
//   M       canvas: air / easel / globe
//   1 - 5   brush size (thin ... fat)
//   C       colour: rainbow over time / by speed (blue slow -> red fast) / by depth
//   S       symmetry: none / mirror (left-right) / kaleidoscope (6 copies around the middle)
//   H       feel your drawing: strokes become solid tubes you can touch
//   G       neon glow on / off
//   T       turntable: the view spins slowly
//   D       demo: draws a trefoil knot and two spirals for you
//   Z       undo the last stroke          N   clear everything
//   P       save the strokes now (they are also saved when you quit)
//   X, F, arrows/drag, wheel, Q: see common/viz.h
//
// Options: --k 0.25  --b 0.001  --device "<name>"  --skip-calibration  --allow-uncalibrated
// Output: data/draw3d_<stamp>.csv (every servo sample) and data/draw3d_<stamp>_strokes.csv
//   (one row per drawn point: stroke, x, y, z, brush, hue)
//   phase: 0 no room, 1 free, 2 touching a canvas or stroke, 3 drawing;  trial: strokes so far
//
// How it works: the servo loop (1 kHz) decides when the pen is down and appends
// points to a fixed array (no allocation in the servo thread). Every frame snap()
// hands only the NEW points to the main thread, which turns them into lit tube
// triangles once and keeps them, so even thousands of points draw quickly.
// "Feel your drawing" finds the nearest stroke segment with a coarse 3D grid, so
// the 1 ms servo budget holds even with a big drawing.

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>

#include "phantom.h"
#include "util.h"
#include "viz.h"

#if VIZ_AVAILABLE

using viz::State;
using viz::Vec3;

struct Pt {
    float x = 0, y = 0, z = 0, size = 0, hue = 0;
    int stroke = 0;
};

struct Draw3D : viz::SceneBase {
    static constexpr bool kButtonPlaces = false;
    static const int kMaxPts = 30000, kMaxStrokes = 5000, kSnapMax = 4096;
    static const int kRing = 8;  // sides of each tube
    enum { AIR = 0, EASEL = 1, GLOBE = 2 };

    const double brushes[5] = {0.8, 1.5, 2.5, 3.5, 5.0};  // tube radius, mm
    const Vec3 sheetC{0, 0, -16}, sheetH{55, 40, 1};       // easel sheet (front face z = -15)
    const Vec3 globeC{0, -22, -8};                         // below where the room is placed, so the
    const double globeR = 18, minStep = 0.8;               // stylus never starts inside it; mm between points

    // Coarse grid over the room for "feel your drawing" (10 mm cells).
    static const int kGX = 14, kGY = 11, kGZ = 11;
    const double cell = 10, gx0 = -70, gy0 = -55, gz0 = -55;

    // --- servo thread
    Pt pts[kMaxPts];
    int count = 0, strokes = 0;
    int strokeStart[kMaxStrokes] = {};
    int gridHead[kGX * kGY * kGZ];
    int gridNext[kMaxPts];
    bool penDown = false, touchNow = false;
    int canvas = AIR, brushIdx = 1, colorMode = 0, symmetry = 0;
    bool feel = false, glow = true, turntable = false;
    int saveRequests = 0;
    double t = 0;
    double rampStart = -10;  // canvas / stroke forces fade in over 1 s after M, H, D, Z, N
    viz::BoxProxy sheetProxy;
    bool sticking = false;
    double anchorX = 0, anchorY = 0;
    mutable int sentCount = 0;  // how many points the main thread already has (snap() keeps it)

    // --- main thread: the drawing as triangles, built once per point
    std::vector<Pt> mainPts;
    std::vector<float> triPos, triNrm, triCol;
    std::vector<size_t> triMark;     // triangle-array size before point i's triangles
    std::vector<float> ringPos, ringNrm;  // kRing vertices per point
    std::vector<unsigned char> ringOk;
    std::vector<Vec3> frameN;        // transported tube normal per point
    std::filesystem::path strokesPath;
    int savedRequests = 0;

    struct Snap {
        int truncate = -1, nFresh = 0, count = 0, strokes = 0;
        Pt fresh[kSnapMax];
        int canvas = AIR, brushIdx = 1, colorMode = 0, symmetry = 0, saveRequests = 0;
        bool feel = false, glow = true, turntable = false, penDown = false;
    };

    Draw3D() { clearGrid(); }

    // ---------------- servo thread ----------------
    int cellOf(double x, double y, double z) const {
        int ix = (int)std::floor((x - gx0) / cell), iy = (int)std::floor((y - gy0) / cell), iz = (int)std::floor((z - gz0) / cell);
        if (ix < 0 || iy < 0 || iz < 0 || ix >= kGX || iy >= kGY || iz >= kGZ) return -1;
        return (ix * kGY + iy) * kGZ + iz;
    }
    void clearGrid() { for (int& h : gridHead) h = -1; }
    void gridAdd(int i) {
        int c = cellOf(pts[i].x, pts[i].y, pts[i].z);
        if (c < 0) { gridNext[i] = -1; return; }
        gridNext[i] = gridHead[c];
        gridHead[c] = i;
    }
    void rebuildGrid() { clearGrid(); for (int i = 0; i < count; ++i) gridAdd(i); }

    double hueFor(const Vec3& p, double speed) const {
        if (colorMode == 1) return 0.66 * (1.0 - viz::clampd(speed / 400.0, 0, 1));  // blue slow -> red fast
        if (colorMode == 2) return 0.8 * viz::clampd((p.z + 45) / 90.0, 0, 1);       // by depth
        return std::fmod(t * 0.08, 1.0);                                             // rainbow over time
    }
    void addPoint(const Vec3& p, double hue, double size) {
        if (count >= kMaxPts) return;
        if (count > 0 && pts[count - 1].stroke == strokes - 1) {
            const Pt& l = pts[count - 1];
            double dx = p.x - l.x, dy = p.y - l.y, dz = p.z - l.z;
            if (dx * dx + dy * dy + dz * dz < minStep * minStep) return;
        }
        Pt& q = pts[count];
        q.x = (float)p.x; q.y = (float)p.y; q.z = (float)p.z;
        q.size = (float)size; q.hue = (float)hue; q.stroke = strokes - 1;
        gridAdd(count);
        count++;
    }
    void beginStroke() {
        if (strokes >= kMaxStrokes || count >= kMaxPts) return;
        strokeStart[strokes++] = count;
    }
    void endStroke() {
        // A stroke with fewer than 2 points draws nothing: drop it.
        if (strokes > 0 && count - strokeStart[strokes - 1] < 2) { count = strokeStart[strokes - 1]; strokes--; rebuildGrid(); }
    }

    void reset() {
        sheetProxy.reset(); sticking = false;
        if (penDown) { penDown = false; endStroke(); }
    }

    void key(int vk) {
        // Anything that can put a surface where the stylus already is fades its forces in again.
        if (vk == 'M' || vk == 'H' || vk == 'D' || vk == 'Z' || vk == 'N') rampStart = t;
        if (vk == 'M') { canvas = (canvas + 1) % 3; sheetProxy.reset(); }
        if (vk >= '1' && vk <= '5') brushIdx = vk - '1';
        if (vk == 'C') colorMode = (colorMode + 1) % 3;
        if (vk == 'S') symmetry = (symmetry + 1) % 3;
        if (vk == 'H') feel = !feel;
        if (vk == 'G') glow = !glow;
        if (vk == 'T') turntable = !turntable;
        if (vk == 'P') saveRequests++;
        if (vk == 'Z' && !penDown && strokes > 0) { count = strokeStart[--strokes]; rebuildGrid(); }
        if (vk == 'N' && !penDown) { count = 0; strokes = 0; clearGrid(); }
        if (vk == 'D' && !penDown) demo();
    }

    // A trefoil knot and two spirals, generated straight into the point array.
    void demo() {
        const double pi = viz::kPi;
        beginStroke();
        for (int i = 0; i <= 900; ++i) {
            double a = 2 * pi * i / 900;
            Vec3 p(12 * (std::sin(a) + 2 * std::sin(2 * a)), 12 * (std::cos(a) - 2 * std::cos(2 * a)) - 5, -12 * std::sin(3 * a));
            addPoint(p, std::fmod(i / 900.0 * 2, 1.0), 2.5);
        }
        endStroke();
        for (int sgn = -1; sgn <= 1; sgn += 2) {
            beginStroke();
            for (int i = 0; i <= 600; ++i) {
                double a = 6 * pi * i / 600;
                Vec3 p(sgn * 45 + 8 * std::cos(a), -40 + 75.0 * i / 600, 8 * std::sin(a) - 10);
                addPoint(p, 0.5 + 0.5 * i / 600.0 * (sgn > 0 ? 1 : -1), 1.5);
            }
            endStroke();
        }
    }

    // Nearest stroke segment within reach, for "feel your drawing".
    Vec3 strokeForce(const Vec3& p, const Vec3& v, bool& touching) {
        touching = false;
        double best = 1e9;
        Vec3 bestC;
        double bestR = 0;
        int cx = (int)std::floor((p.x - gx0) / cell), cy = (int)std::floor((p.y - gy0) / cell), cz = (int)std::floor((p.z - gz0) / cell);
        for (int ix = cx - 1; ix <= cx + 1; ++ix)
            for (int iy = cy - 1; iy <= cy + 1; ++iy)
                for (int iz = cz - 1; iz <= cz + 1; ++iz) {
                    if (ix < 0 || iy < 0 || iz < 0 || ix >= kGX || iy >= kGY || iz >= kGZ) continue;
                    for (int i = gridHead[(ix * kGY + iy) * kGZ + iz]; i >= 0; i = gridNext[i]) {
                        if (i == 0 || pts[i - 1].stroke != pts[i].stroke) continue;  // segment (i-1, i)
                        Vec3 a(pts[i - 1].x, pts[i - 1].y, pts[i - 1].z), bb(pts[i].x, pts[i].y, pts[i].z);
                        Vec3 ab = bb - a;
                        double L2 = viz::dot(ab, ab);
                        double u = L2 > 1e-9 ? viz::clampd(viz::dot(p - a, ab) / L2, 0, 1) : 0;
                        Vec3 c = a + ab * u;
                        double d = (p - c).norm();
                        if (d < best) { best = d; bestC = c; bestR = pts[i].size; }
                    }
                }
        double R = bestR + cursorR;
        if (best >= R || best < 1e-6) return Vec3();
        touching = true;
        return viz::contact((p - bestC) * (1.0 / best), R - best, v, 0.6 * k, b);
    }

    Vec3 force(State& s, const Vec3& p, double dt) {
        t += dt;
        const Vec3& v = s.vel;
        Vec3 f = viz::roomForce(room, p, v, cursorR, k, b);
        Vec3 fc;  // canvas + stroke forces (faded in after a change)
        bool touching = false, onCanvas = false;
        Vec3 drawAt = p;

        if (canvas == AIR) {
            double drag = maxDamping > 0 ? 0.25 * maxDamping : 0.001;
            f += v * -drag;  // steadies the hand a little
        } else if (canvas == EASEL) {
            int face = -1;
            Vec3 fs = sheetProxy.force(p, v, sheetC, sheetH, cursorR, k, b, &face);
            if (face == 4 && fs.z > 0) {  // pressing on the front of the sheet
                if (!sticking) { anchorX = p.x; anchorY = p.y; sticking = true; }
                double tx = anchorX - p.x, ty = anchorY - p.y, td = std::sqrt(tx * tx + ty * ty), fmax = 0.4 * fs.z;
                if (k * td > fmax && td > 1e-9) {
                    double keep = fmax / (k * td);
                    anchorX = p.x + tx * keep; anchorY = p.y + ty * keep;
                    tx *= keep; ty *= keep;
                }
                fs += Vec3(k * tx, k * ty, 0);
                onCanvas = true;
                drawAt = Vec3(p.x, p.y, sheetC.z + sheetH.z + 0.6);
            } else {
                sticking = false;
            }
            if (fs.norm() > 0) touching = true;
            fc += fs;
        } else {
            Vec3 fg = viz::sphereForce(p, v, globeC, globeR, cursorR, k, b);
            if (fg.norm() > 0) {
                touching = onCanvas = true;
                Vec3 d = p - globeC;
                drawAt = globeC + d * ((globeR + 0.6) / d.norm());
            }
            fc += fg;
        }
        if (feel) {
            bool ts = false;
            fc += strokeForce(p, v, ts);
            touching = touching || ts;
        }
        f += fc * viz::clampd(t - rampStart, 0, 1);

        // Pen down: the button anywhere, or touching the easel / globe.
        bool want = s.button1() || onCanvas;
        if (want && !penDown) { penDown = true; beginStroke(); }
        if (!want && penDown) { penDown = false; endStroke(); }
        if (penDown) addPoint(drawAt, hueFor(drawAt, v.norm()), brushes[brushIdx]);

        s.phase = penDown ? 3 : (touching ? 2 : 1);
        s.trial = strokes;
        return f;
    }

    void snap(Snap& o) const {
        o.truncate = -1;
        if (count < sentCount) { o.truncate = count; sentCount = count; }
        int n = count - sentCount;
        if (n > kSnapMax) n = kSnapMax;
        for (int i = 0; i < n; ++i) o.fresh[i] = pts[sentCount + i];
        o.nFresh = n;
        sentCount += n;
        o.count = count; o.strokes = strokes;
        o.canvas = canvas; o.brushIdx = brushIdx; o.colorMode = colorMode; o.symmetry = symmetry;
        o.saveRequests = saveRequests; o.feel = feel; o.glow = glow; o.turntable = turntable; o.penDown = penDown;
    }

    // ---------------- main thread ----------------
    void ringFor(size_t i, const Vec3& tan) {
        // Parallel transport: keep the previous ring's normal, minus its part along the tangent.
        const Pt& q = mainPts[i];
        Vec3 n = (i > 0 && mainPts[i - 1].stroke == q.stroke) ? frameN[i - 1] : Vec3(0, 1, 0);
        n = n - tan * viz::dot(n, tan);
        if (n.norm() < 1e-3) { n = std::fabs(tan.y) < 0.9 ? Vec3(0, 1, 0) : Vec3(1, 0, 0); n = n - tan * viz::dot(n, tan); }
        n = n * (1.0 / n.norm());
        Vec3 bn = viz::cross(tan, n);
        frameN[i] = n;
        for (int k2 = 0; k2 < kRing; ++k2) {
            double a = 2 * viz::kPi * k2 / kRing;
            Vec3 dir = n * std::cos(a) + bn * std::sin(a);
            size_t o = (i * kRing + k2) * 3;
            ringNrm[o] = (float)dir.x; ringNrm[o + 1] = (float)dir.y; ringNrm[o + 2] = (float)dir.z;
            ringPos[o] = (float)(q.x + dir.x * q.size);
            ringPos[o + 1] = (float)(q.y + dir.y * q.size);
            ringPos[o + 2] = (float)(q.z + dir.z * q.size);
        }
        ringOk[i] = 1;
    }
    void pushVertex(size_t i, int k2) {
        size_t o = (i * kRing + k2) * 3;
        for (int c = 0; c < 3; ++c) { triPos.push_back(ringPos[o + c]); triNrm.push_back(ringNrm[o + c]); }
        double r, g, bl;
        viz::hsv(mainPts[i].hue, 0.75, 1.0, r, g, bl);
        triCol.push_back((float)r); triCol.push_back((float)g); triCol.push_back((float)bl);
    }
    void appendPoint(const Pt& q) {
        size_t i = mainPts.size();
        mainPts.push_back(q);
        triMark.push_back(triPos.size());
        ringPos.resize((i + 1) * kRing * 3); ringNrm.resize((i + 1) * kRing * 3);
        ringOk.push_back(0);
        frameN.push_back(Vec3(0, 1, 0));
        if (i == 0 || mainPts[i - 1].stroke != q.stroke) return;  // first point: ring comes with the second
        const Pt& a = mainPts[i - 1];
        Vec3 tan(q.x - a.x, q.y - a.y, q.z - a.z);
        double L = tan.norm();
        if (L < 1e-6) tan = Vec3(1, 0, 0); else tan = tan * (1.0 / L);
        if (!ringOk[i - 1]) ringFor(i - 1, tan);
        ringFor(i, tan);
        for (int k2 = 0; k2 < kRing; ++k2) {  // two triangles per side between ring i-1 and ring i
            int k3 = (k2 + 1) % kRing;
            pushVertex(i - 1, k2); pushVertex(i, k2); pushVertex(i, k3);
            pushVertex(i - 1, k2); pushVertex(i, k3); pushVertex(i - 1, k3);
        }
    }
    void truncateTo(size_t n) {
        if (n >= mainPts.size()) return;
        size_t m = triMark[n];
        triPos.resize(m); triNrm.resize(m); triCol.resize(m);
        mainPts.resize(n); triMark.resize(n); ringOk.resize(n); frameN.resize(n);
        ringPos.resize(n * kRing * 3); ringNrm.resize(n * kRing * 3);
    }

    void onStart(const std::filesystem::path& csv) {
        strokesPath = csv;
        strokesPath.replace_filename(csv.stem().string() + "_strokes.csv");
    }
    void saveStrokes() {
        FILE* f = std::fopen(strokesPath.string().c_str(), "w");
        if (!f) { std::printf("Could not save %s\n", strokesPath.string().c_str()); return; }
        std::fprintf(f, "stroke,x_mm,y_mm,z_mm,brush_mm,hue\n");
        for (const Pt& q : mainPts) std::fprintf(f, "%d,%.2f,%.2f,%.2f,%.2f,%.3f\n", q.stroke, q.x, q.y, q.z, q.size, q.hue);
        std::fclose(f);
        std::printf("saved %zu points -> %s\n", mainPts.size(), strokesPath.string().c_str());
    }
    void mainTick(const Snap& sn) {
        if (sn.truncate >= 0) truncateTo((size_t)sn.truncate);
        for (int i = 0; i < sn.nFresh; ++i) appendPoint(sn.fresh[i]);
        if (sn.turntable) viz::ui().yaw += 0.25;
        if (sn.saveRequests != savedRequests) { savedRequests = sn.saveRequests; saveStrokes(); }
    }
    void onStop() { if (!mainPts.empty()) saveStrokes(); }

    void drawStrokes(bool glowOn) const {
        if (!triPos.empty()) {
            glEnableClientState(GL_VERTEX_ARRAY);
            glEnableClientState(GL_NORMAL_ARRAY);
            glEnableClientState(GL_COLOR_ARRAY);
            glVertexPointer(3, GL_FLOAT, 0, triPos.data());
            glNormalPointer(GL_FLOAT, 0, triNrm.data());
            glColorPointer(3, GL_FLOAT, 0, triCol.data());
            glDrawArrays(GL_TRIANGLES, 0, (GLsizei)(triPos.size() / 3));
            glDisableClientState(GL_VERTEX_ARRAY);
            glDisableClientState(GL_NORMAL_ARRAY);
            glDisableClientState(GL_COLOR_ARRAY);
        }
        if (glowOn && mainPts.size() > 1) {
            // Neon halo: wide, faint, additive lines along each stroke.
            glDisable(GL_LIGHTING);
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE);
            glDepthMask(GL_FALSE);
            for (int pass = 0; pass < 2; ++pass) {
                glLineWidth(pass == 0 ? 14.0f : 6.0f);
                int prevStroke = -1;
                for (size_t i = 0; i < mainPts.size(); ++i) {
                    const Pt& q = mainPts[i];
                    if (q.stroke != prevStroke) {
                        if (prevStroke >= 0) glEnd();
                        glBegin(GL_LINE_STRIP);
                        prevStroke = q.stroke;
                    }
                    double r, g, bl;
                    viz::hsv(q.hue, 0.8, 1.0, r, g, bl);
                    glColor4d(r, g, bl, pass == 0 ? 0.08 : 0.15);
                    glVertex3f(q.x, q.y, q.z);
                }
                if (prevStroke >= 0) glEnd();
            }
            glLineWidth(1);
            glDepthMask(GL_TRUE);
            glDisable(GL_BLEND);
            glEnable(GL_LIGHTING);
        }
    }

    void draw(const Snap& sn, const Vec3& cursor) const {
        if (sn.canvas == EASEL) {
            viz::drawBox(sheetC, sheetH, 0.95, 0.94, 0.9);
            viz::drawShadow(Vec3(sheetC.x, 0, sheetC.z), 20, room.floor);
        } else if (sn.canvas == GLOBE) {
            viz::drawShadow(globeC, globeR * 0.9, room.floor);
            viz::drawSphere(globeC, globeR, 0.2, 0.22, 0.3);
        }
        // The drawing, plus its symmetric copies.
        int copies = sn.symmetry == 2 ? 6 : (sn.symmetry == 1 ? 2 : 1);
        for (int c = 0; c < copies; ++c) {
            glPushMatrix();
            if (sn.symmetry == 1 && c == 1) glScaled(-1, 1, 1);
            if (sn.symmetry == 2) glRotated(60.0 * c, 0, 1, 0);
            drawStrokes(sn.glow);
            glPopMatrix();
        }
        // Brush preview around the stylus.
        double r, g, bl;
        viz::hsv(sn.colorMode == 0 && !mainPts.empty() ? mainPts.back().hue : 0.55, 0.75, 1.0, r, g, bl);
        viz::drawSphere(cursor, brushes[sn.brushIdx] + cursorR, r, g, bl, sn.penDown ? 0.35 : 0.15);
        if (sn.symmetry == 1) viz::drawSphere(Vec3(-cursor.x, cursor.y, cursor.z), cursorR, r, g, bl, 0.3);
    }

    void status(const Snap& sn, char* buf, size_t n) const {
        static const char* cv[3] = {"AIR (hold button)", "EASEL (press on paper)", "GLOBE (touch it)"};
        static const char* cm[3] = {"rainbow", "speed", "depth"};
        static const char* sy[3] = {"", " mirror", " kaleido"};
        std::snprintf(buf, n, "%s - brush %d - %s%s%s - %d strokes, %d pts - M/1-5/C/S/H/G/T/D/Z/N/P", cv[sn.canvas],
                      sn.brushIdx + 1, cm[sn.colorMode], sy[sn.symmetry], sn.feel ? " feel" : "", sn.strokes, sn.count);
    }
};

int main(int argc, char** argv) {
    static Draw3D scene;  // static: the point arrays are about 1 MB
    return viz::run(argc, argv, scene, "draw3d",
                    "Hold button 1 to draw (AIR), press on the paper (EASEL) or the ball (GLOBE). M canvas, 1-5 brush,\n"
                    "C colour, S symmetry, H feel strokes, G glow, T turntable, D demo, Z undo, N clear, P save.");
}

#else
int main() {
    std::printf("draw3d needs a 3D window: build on Windows, or with CMake (it fetches GLFW) elsewhere.\n");
    return 0;
}
#endif
