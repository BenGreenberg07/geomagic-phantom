#pragma once
// adl_draw.h - graphics for the ADL simulator: one realistic station per task,
// the objects (posed from the physics), the hand, and the on-screen HUD.
//
// OpenGL 1.x fixed pipeline (works with Win32 and GLFW): two lights with
// specular highlights, a gradient background, blob shadows, procedural meshes at
// real dimensions (mm). Everything is drawn from the View (a copy of the servo
// state) plus the models' constant geometry, so it never touches servo data.

#include <cmath>
#include <cstdio>
#include <cstring>

#include "viz.h"
#include "hud_font.h"
#include "adl_core.h"

namespace adl_draw {

using adl::Vec3;
const double kPi = 3.14159265358979;

// ---------------------------------------------------------------------------
// small GL helpers
// ---------------------------------------------------------------------------
inline void shiny(double spec, double shin) {
    GLfloat s[4] = {(GLfloat)spec, (GLfloat)spec, (GLfloat)spec, 1.0f};
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, s);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, (GLfloat)shin);
}
inline void matte() { shiny(0.0, 1.0); }

// Local frame: object x,y,z axes map to X,Y,Z (unit, orthogonal) at origin O.
inline void pushFrame(const Vec3& O, const Vec3& X, const Vec3& Y, const Vec3& Z) {
    GLdouble m[16] = {X.x, X.y, X.z, 0, Y.x, Y.y, Y.z, 0, Z.x, Z.y, Z.z, 0, O.x, O.y, O.z, 1};
    glPushMatrix();
    glMultMatrixd(m);
}

inline void box(const Vec3& c, const Vec3& h, double r, double g, double b, double a = 1.0) { viz::drawBox(c, h, r, g, b, a); }

// Cylinder from base along unit dir.
inline void cyl(const Vec3& base, const Vec3& dir, double rad, double len, double r, double g, double b, bool caps = true,
                int slices = 28) {
    Vec3 z(0, 0, 1);
    double d = viz::dot(z, dir);
    Vec3 ax = viz::cross(z, dir);
    double an = std::acos(adl::clampd(d, -1, 1)) * 180 / kPi;
    glColor3d(r, g, b);
    glPushMatrix();
    glTranslated(base.x, base.y, base.z);
    if (ax.norm() > 1e-9) glRotated(an, ax.x, ax.y, ax.z);
    else if (d < 0) glRotated(180, 1, 0, 0);
    gluCylinder(viz::quadric(), rad, rad, len, slices, 1);
    if (caps) {
        glPushMatrix();
        glRotated(180, 1, 0, 0);
        gluDisk(viz::quadric(), 0, rad, slices, 1);
        glPopMatrix();
        glTranslated(0, 0, len);
        gluDisk(viz::quadric(), 0, rad, slices, 1);
    }
    glPopMatrix();
}

// Box with rounded edges (radius rad): 3 slabs + 12 edge cylinders + 8 corner spheres.
inline void roundBox(const Vec3& c, const Vec3& h, double rad, double r, double g, double b) {
    box(c, Vec3(h.x, h.y - rad, h.z - rad), r, g, b);
    box(c, Vec3(h.x - rad, h.y, h.z - rad), r, g, b);
    box(c, Vec3(h.x - rad, h.y - rad, h.z), r, g, b);
    double ix = h.x - rad, iy = h.y - rad, iz = h.z - rad;
    for (int a = -1; a <= 1; a += 2)
        for (int bb = -1; bb <= 1; bb += 2) {
            cyl(c + Vec3(-ix, a * iy, bb * iz), Vec3(1, 0, 0), rad, 2 * ix, r, g, b, false, 12);
            cyl(c + Vec3(a * ix, -iy, bb * iz), Vec3(0, 1, 0), rad, 2 * iy, r, g, b, false, 12);
            cyl(c + Vec3(a * ix, bb * iy, -iz), Vec3(0, 0, 1), rad, 2 * iz, r, g, b, false, 12);
        }
    glColor3d(r, g, b);
    for (int a = -1; a <= 1; a += 2)
        for (int bb = -1; bb <= 1; bb += 2)
            for (int cc = -1; cc <= 1; cc += 2) {
                glPushMatrix();
                glTranslated(c.x + a * ix, c.y + bb * iy, c.z + cc * iz);
                gluSphere(viz::quadric(), rad, 12, 8);
                glPopMatrix();
            }
}

inline void quad(const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& d, const Vec3& n) {
    glNormal3d(n.x, n.y, n.z);
    glVertex3d(a.x, a.y, a.z); glVertex3d(b.x, b.y, b.z); glVertex3d(c.x, c.y, c.z); glVertex3d(d.x, d.y, d.z);
}

// Stroke text lying in a plane: origin O, right axis R, up axis U (both unit), height h (mm).
inline void text3D(const Vec3& O, const Vec3& R, const Vec3& U, double h, const char* s, double r, double g, double b,
                   double width = 1.5) {
    glDisable(GL_LIGHTING);
    glLineWidth((GLfloat)width);
    glColor3d(r, g, b);
    Vec3 Z = viz::cross(R, U);
    pushFrame(O, R, U, Z);
    hud::strokes(0, 0, h, s);
    glPopMatrix();
    glLineWidth(1);
    glEnable(GL_LIGHTING);
}

// Glowing emissive sphere (LEDs).
inline void glow(const Vec3& c, double rad, double r, double g, double b, double strength) {
    glDisable(GL_LIGHTING);
    glColor3d(r * strength, g * strength, b * strength);
    glPushMatrix();
    glTranslated(c.x, c.y, c.z);
    gluSphere(viz::quadric(), rad, 14, 10);
    glPopMatrix();
    if (strength > 0.6) {
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE);
        glDepthMask(GL_FALSE);
        glColor4d(r, g, b, 0.25 * strength);
        glPushMatrix();
        glTranslated(c.x, c.y, c.z);
        gluSphere(viz::quadric(), rad * 2.6, 14, 10);
        glPopMatrix();
        glDepthMask(GL_TRUE);
        glDisable(GL_BLEND);
    }
    glEnable(GL_LIGHTING);
}

// Wood / tile / paint helpers ------------------------------------------------
inline void woodTable(double y, double x0, double x1, double z0, double z1) {
    matte();
    box(Vec3((x0 + x1) / 2, y - 6, (z0 + z1) / 2), Vec3((x1 - x0) / 2, 6, (z1 - z0) / 2), 0.55, 0.36, 0.2);
    glDisable(GL_LIGHTING);
    glColor4d(0.35, 0.21, 0.1, 1);
    glBegin(GL_LINES);
    for (double x = x0 + 7; x < x1; x += 13.7) {  // grain lines
        glVertex3d(x, y + 0.05, z0); glVertex3d(x + 4, y + 0.05, z1);
    }
    glEnd();
    glEnable(GL_LIGHTING);
}
inline void tileFloor(double y) {
    matte();
    glDisable(GL_LIGHTING);
    glBegin(GL_QUADS);
    for (int i = -6; i < 6; ++i)
        for (int j = -6; j < 4; ++j) {
            double c = ((i + j) & 1) ? 0.30 : 0.34;
            glColor3d(c, c, c * 1.05);
            double x = i * 30.0, z = j * 30.0;
            glVertex3d(x, y, z); glVertex3d(x, y, z + 30); glVertex3d(x + 30, y, z + 30); glVertex3d(x + 30, y, z);
        }
    glEnd();
    glEnable(GL_LIGHTING);
}

inline void cameraFor(int task) {
    viz::Ui& u = viz::ui();
    switch (task) {
    case adl::KEY: u.yaw = 32; u.pitch = 10; u.dist = 175; u.tx = 8; u.ty = -6; u.tz = -26; break;
    case adl::PLUG: u.yaw = -32; u.pitch = 10; u.dist = 185; u.tx = -8; u.ty = -4; u.tz = -28; break;
    case adl::ATM: u.yaw = 38; u.pitch = 16; u.dist = 240; u.tx = 0; u.ty = -6; u.tz = -45; break;
    case adl::FORK: u.yaw = 22; u.pitch = 32; u.dist = 210; u.tx = 0; u.ty = -55; u.tz = -6; break;
    case adl::VZIP: u.yaw = 24; u.pitch = 8; u.dist = 200; u.tx = 0; u.ty = 2; u.tz = -18; break;
    case adl::HZIP: u.yaw = 18; u.pitch = 12; u.dist = 200; u.tx = 0; u.ty = 0; u.tz = -18; break;
    default: u.yaw = 25; u.pitch = 42; u.dist = 165; u.tx = 10; u.ty = -28; u.tz = -12; break;
    }
}

// Pinch axis per task (the object's thickness direction) and the paper's pinch opening.
inline Vec3 pinchAxis(int task) {
    switch (task) {
    case adl::KEY: case adl::PLUG: return Vec3(1, 0, 0);
    case adl::ATM: return Vec3(0, 1, 0);
    case adl::FORK: case adl::VZIP: case adl::HZIP: return Vec3(0, 0, 1);
    default: return Vec3(0, 1, 0);
    }
}

// ---------------------------------------------------------------------------
// Renderer
// ---------------------------------------------------------------------------
struct Renderer {
    GLuint lists[adl::NTASK] = {};  // static station geometry, compiled once
    double t = 0;

    void lights() {
        GLfloat d0[4] = {1.0f, 0.96f, 0.9f, 1}, s0[4] = {1, 1, 1, 1};
        glLightfv(GL_LIGHT0, GL_DIFFUSE, d0);
        glLightfv(GL_LIGHT0, GL_SPECULAR, s0);
        GLfloat p1[4] = {-160.0f, 60.0f, 120.0f, 0.0f}, d1[4] = {0.28f, 0.31f, 0.38f, 1}, z[4] = {0, 0, 0, 1};
        glLightfv(GL_LIGHT1, GL_POSITION, p1);
        glLightfv(GL_LIGHT1, GL_DIFFUSE, d1);
        glLightfv(GL_LIGHT1, GL_SPECULAR, z);
        glEnable(GL_LIGHT1);
        glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE);
        GLfloat amb[4] = {0.28f, 0.28f, 0.31f, 1};
        glLightModelfv(GL_LIGHT_MODEL_AMBIENT, amb);
    }

    void background() {
        int w = viz::ui().width, h = viz::ui().height;
        glPushAttrib(GL_ENABLE_BIT | GL_DEPTH_BUFFER_BIT);
        glDisable(GL_LIGHTING);
        glDisable(GL_DEPTH_TEST);
        glDepthMask(GL_FALSE);
        glMatrixMode(GL_PROJECTION); glPushMatrix(); glLoadIdentity(); glOrtho(0, w, 0, h, -1, 1);
        glMatrixMode(GL_MODELVIEW); glPushMatrix(); glLoadIdentity();
        glBegin(GL_QUADS);
        glColor3d(0.05, 0.055, 0.07); glVertex2d(0, 0); glVertex2d(w, 0);
        glColor3d(0.20, 0.23, 0.29); glVertex2d(w, h); glVertex2d(0, h);
        glEnd();
        glMatrixMode(GL_PROJECTION); glPopMatrix();
        glMatrixMode(GL_MODELVIEW); glPopMatrix();
        glDepthMask(GL_TRUE);
        glPopAttrib();
    }

    // ---------------- stations (static) ----------------
    void stationKey(const adl::Insertion& k) {
        double zf = k.E.z + k.cham;  // lock face
        tileFloor(-75);
        matte();
        // door with raised panels, frame
        box(Vec3(0, -5, zf - 22), Vec3(110, 72, 22), 0.46, 0.29, 0.15);
        box(Vec3(-45, 20, zf + 0.6), Vec3(38, 32, 0.6), 0.5, 0.32, 0.17);
        box(Vec3(-45, -45, zf + 0.6), Vec3(38, 24, 0.6), 0.5, 0.32, 0.17);
        box(Vec3(118, -5, zf - 10), Vec3(8, 72, 14), 0.38, 0.23, 0.12);
        // brass rose + cylinder face + keyway
        shiny(0.8, 60);
        cyl(Vec3(k.E.x, k.E.y, zf - 1), Vec3(0, 0, 1), 26, 2.5, 0.78, 0.6, 0.25);
        cyl(Vec3(k.E.x, k.E.y, zf - 0.5), Vec3(0, 0, 1), 9.5, 2.3, 0.86, 0.7, 0.32);
        cyl(Vec3(k.E.x, k.E.y, zf - 0.5), Vec3(0, 0, 1), 6.3, 2.45, 0.92, 0.78, 0.38);
        glDisable(GL_LIGHTING);
        glColor3d(0.03, 0.02, 0.01);  // keyway (2.3 x 8.7 with a ward notch)
        double zk = zf + 2.0;
        glBegin(GL_QUADS);
        glVertex3d(k.E.x - 1.15, k.E.y - 4.35, zk); glVertex3d(k.E.x + 1.15, k.E.y - 4.35, zk);
        glVertex3d(k.E.x + 1.15, k.E.y + 4.35, zk); glVertex3d(k.E.x - 1.15, k.E.y + 4.35, zk);
        glVertex3d(k.E.x + 1.15, k.E.y - 0.5, zk); glVertex3d(k.E.x + 2.0, k.E.y - 0.5, zk);
        glVertex3d(k.E.x + 2.0, k.E.y + 1.2, zk); glVertex3d(k.E.x + 1.15, k.E.y + 1.2, zk);
        glEnd();
        glEnable(GL_LIGHTING);
        // lever handle beside the lock
        cyl(Vec3(k.E.x - 52, k.E.y + 30, zf - 1), Vec3(0, 0, 1), 18, 4, 0.75, 0.75, 0.78);
        cyl(Vec3(k.E.x - 52, k.E.y + 30, zf + 3), Vec3(0, 0, 1), 6, 18, 0.8, 0.8, 0.83);
        box(Vec3(k.E.x - 30, k.E.y + 30, zf + 21), Vec3(26, 4.5, 3.5), 0.82, 0.82, 0.85);
        matte();
    }
    void stationPlug(const adl::Insertion& pl) {
        double zf = pl.E.z + pl.cham;
        tileFloor(-75);
        matte();
        box(Vec3(0, -5, zf - 15), Vec3(120, 72, 15), 0.82, 0.79, 0.72);   // painted wall
        box(Vec3(0, -71, zf + 3), Vec3(120, 4, 3), 0.95, 0.95, 0.93);     // baseboard
        // duplex outlet faceplate (ivory), the upper receptacle is the one used
        Vec3 c = pl.E + Vec3(0, 11.9 / 2 - 18, 0);
        shiny(0.35, 30);
        roundBox(Vec3(c.x, c.y, zf + 2.0), Vec3(35, 57, 2.0), 1.5, 0.93, 0.9, 0.82);
        for (int r = 0; r < 2; ++r) {
            Vec3 g = pl.E + Vec3(0, -38.1 * r, 0);  // receptacles 38.1 mm apart
            roundBox(Vec3(g.x, g.y + 6, zf + 4.2), Vec3(18, 16, 1.0), 0.9, 0.95, 0.92, 0.85);
            glDisable(GL_LIGHTING);
            glColor3d(0.05, 0.05, 0.05);
            double zz = zf + 5.25;
            for (int s = -1; s <= 1; s += 2) {   // blade slots: 1.5 x 6.35 (neutral 7.9)
                double hh = s < 0 ? 3.95 : 3.2;
                double bx = g.x + s * 6.35, by = g.y + 11.9;
                glBegin(GL_QUADS);
                glVertex3d(bx - 0.85, by - hh, zz); glVertex3d(bx + 0.85, by - hh, zz);
                glVertex3d(bx + 0.85, by + hh, zz); glVertex3d(bx - 0.85, by + hh, zz);
                glEnd();
            }
            glBegin(GL_POLYGON);  // D-shaped ground hole
            for (int i = 0; i <= 16; ++i) {
                double a = kPi * i / 16;
                glVertex3d(g.x + 2.6 * std::cos(a), g.y - 0.6 + 2.6 * std::sin(a), zz);
            }
            glVertex3d(g.x - 2.6, g.y - 2.4, zz); glVertex3d(g.x + 2.6, g.y - 2.4, zz);
            glEnd();
            glEnable(GL_LIGHTING);
        }
        cyl(Vec3(c.x, c.y, zf + 4.0), Vec3(0, 0, 1), 1.8, 0.6, 0.7, 0.7, 0.7);  // centre screw
        matte();
    }
    void stationAtm(const adl::Insertion& a) {
        double zf = a.E.z + a.cham;
        tileFloor(-75);
        shiny(0.5, 40);
        box(Vec3(0, -5, zf - 20), Vec3(115, 72, 20), 0.55, 0.57, 0.6);     // brushed steel fascia
        box(Vec3(0, 45, zf + 0.5), Vec3(52, 22, 0.5), 0.15, 0.16, 0.18);  // screen bezel
        matte();
        glDisable(GL_LIGHTING);
        glBegin(GL_QUADS);  // screen
        glColor3d(0.05, 0.16, 0.38); glVertex3d(-48, 26, zf + 1.05); glVertex3d(48, 26, zf + 1.05);
        glColor3d(0.12, 0.3, 0.6); glVertex3d(48, 64, zf + 1.05); glVertex3d(-48, 64, zf + 1.05);
        glEnd();
        glEnable(GL_LIGHTING);
        text3D(Vec3(-38, 50, zf + 1.2), Vec3(1, 0, 0), Vec3(0, 1, 0), 6, "WELCOME", 0.9, 0.95, 1.0, 1.5);
        text3D(Vec3(-42, 37, zf + 1.2), Vec3(1, 0, 0), Vec3(0, 1, 0), 4.2, "INSERT CARD", 0.8, 0.9, 1.0, 1.2);
        // keypad
        shiny(0.4, 30);
        box(Vec3(-50, -40, zf + 1.2), Vec3(26, 22, 1.2), 0.2, 0.2, 0.22);
        const char* keys = "123456789*0#";
        for (int i = 0; i < 12; ++i) {
            double kx = -64 + (i % 3) * 14, ky = -25 - (i / 3) * 10;
            box(Vec3(kx, ky, zf + 3.0), Vec3(5.5, 3.8, 0.8), 0.72, 0.73, 0.76);
            char s[2] = {keys[i], 0};
            text3D(Vec3(kx - 1.5, ky - 2, zf + 3.9), Vec3(1, 0, 0), Vec3(0, 1, 0), 4, s, 0.1, 0.1, 0.1, 1.2);
        }
        // card reader bezel (slot at E)
        shiny(0.6, 50);
        roundBox(Vec3(a.E.x, a.E.y, zf + 4), Vec3(40, 10, 4), 2.5, 0.12, 0.12, 0.14);
        matte();
        text3D(Vec3(a.E.x - 22, a.E.y + 14, zf + 0.6), Vec3(1, 0, 0), Vec3(0, 1, 0), 3.5, "CARD", 0.9, 0.9, 0.9, 1.2);
    }
    void stationFork(const adl::Fork& f) {
        double plateY = f.top.y - f.thick;
        woodTable(plateY - 3, -110, 110, -80, 60);
        shiny(0.6, 70);
        cyl(Vec3(f.top.x, plateY - 3, f.top.z), Vec3(0, 1, 0), 52, 3, 0.96, 0.96, 0.94);   // plate
        cyl(Vec3(f.top.x, plateY - 0.2, f.top.z), Vec3(0, 1, 0), 40, 0.4, 0.9, 0.9, 0.88);
        matte();
    }
    void stationZip(const adl::Zipper& z, bool vertical) {
        tileFloor(-75);
        matte();
        double zf = z.s0.z;
        if (vertical) {  // navy jacket front with a collar and pocket seams
            box(Vec3(0, 0, zf - 8), Vec3(100, 75, 8), 0.12, 0.17, 0.32);
            box(Vec3(0, 62, zf - 2), Vec3(45, 12, 6), 0.1, 0.14, 0.27);
            glDisable(GL_LIGHTING);
            glColor3d(0.3, 0.38, 0.6);
            glEnable(GL_LINE_STIPPLE);
            glLineStipple(2, 0x00FF);
            glBegin(GL_LINES);
            glVertex3d(-75, -30, zf + 0.2); glVertex3d(-35, -30, zf + 0.2);
            glVertex3d(35, -30, zf + 0.2); glVertex3d(75, -30, zf + 0.2);
            glEnd();
            glDisable(GL_LINE_STIPPLE);
            glEnable(GL_LIGHTING);
        } else {         // red backpack: body, front pocket, straps
            roundBox(Vec3(0, -5, zf - 30), Vec3(95, 70, 30), 12, 0.62, 0.1, 0.1);
            box(Vec3(0, 50, zf - 5), Vec3(60, 6, 5), 0.15, 0.15, 0.17);  // top handle
        }
        // lining visible in the open part of the zipper
        Vec3 across = vertical ? Vec3(1, 0, 0) : Vec3(0, 1, 0);
        Vec3 c = z.s0 + z.t * (z.L / 2) + z.nf * 0.05;
        double chainW = vertical ? 5.8 : 7.6;
        Vec3 hs = z.t * (z.L / 2) + across * (chainW / 2 + 12);
        if (vertical) box(c, Vec3(std::fabs(hs.x) + 0.01, std::fabs(hs.y) + 0.01, 0.05), 0.06, 0.07, 0.12);
        else box(c, Vec3(std::fabs(hs.x) + 0.01, std::fabs(hs.y) + 0.01, 0.05), 0.25, 0.05, 0.05);
        // bottom box / top stop
        shiny(0.7, 60);
        box(z.s0 - z.t * 3 + z.nf * 1.2, (z.t * 3 + across * 4 + z.nf * 1.2), 0.6, 0.6, 0.62);
        box(z.s0 + z.t * (z.L + 2.5) + z.nf * 1.2, Vec3(1.5, 1.5, 1.2) + Vec3(0, 0, 0), 0.6, 0.6, 0.62);
        matte();
    }
    void stationRemote(const adl::Remote& r) {
        woodTable(r.body.y - r.bodyH.y, -110, 110, -90, 60);
        shiny(0.45, 40);
        roundBox(r.body, r.bodyH, 5, 0.94, 0.94, 0.95);
        // button bezel ring
        cyl(Vec3(r.btn.x, r.btn.y - 1.0, r.btn.z), Vec3(0, 1, 0), r.capR + 1.8, 1.2, 0.7, 0.72, 0.75);
        matte();
        text3D(Vec3(r.body.x - 12, r.btn.y + 0.06, r.body.z + 12), Vec3(1, 0, 0), Vec3(0, 0, -1), 3.6, "DOORBELL", 0.45,
               0.45, 0.5, 1.2);
        glDisable(GL_LIGHTING);  // speaker grille
        glColor3d(0.6, 0.6, 0.62);
        glBegin(GL_LINES);
        for (int i = 0; i < 5; ++i) {
            double x = r.body.x - 8 + i * 4;
            glVertex3d(x, r.btn.y + 0.05, r.body.z + 20); glVertex3d(x, r.btn.y + 0.05, r.body.z + 26);
        }
        glEnd();
        glEnable(GL_LIGHTING);
    }

    void station(const adl::AdlCore& c, int task) {
        if (!lists[task]) {
            lists[task] = glGenLists(1);
            glNewList(lists[task], GL_COMPILE);
            switch (task) {
            case adl::KEY: stationKey(c.key); break;
            case adl::PLUG: stationPlug(c.plug); break;
            case adl::ATM: stationAtm(c.atm); break;
            case adl::FORK: stationFork(c.fork); break;
            case adl::VZIP: stationZip(c.vzip, true); break;
            case adl::HZIP: stationZip(c.hzip, false); break;
            default: stationRemote(c.remote); break;
            }
            matte();
            glEndList();
        }
        glCallList(lists[task]);
    }

    // ---------------- objects ----------------
    // Frame for an inserted object: origin at the tip, local z points back toward the user,
    // local y up, rotated by the stylus twist about the insertion axis.
    void objFrame(const Vec3& tip, const Vec3& n, double roll) {
        Vec3 Z = n * -1.0, Y(0, 1, 0);
        Vec3 X = viz::cross(Y, Z);
        double cr = std::cos(roll), sr = std::sin(roll);
        Vec3 X2 = X * cr + Y * sr, Y2 = Y * cr - X * sr;
        pushFrame(tip, X2, Y2, Z);
    }

    void key(const adl::View& v, const adl::Insertion& k, bool jam) {
        objFrame(v.objTip, k.n, v.held ? v.roll : 0);
        shiny(0.9, 80);
        double br = jam ? 0.9 : 0.85, bg = jam ? 0.45 : 0.68, bb = jam ? 0.25 : 0.3;
        // blade: bottom edge straight, top edge with 5 bitting cuts; 2.1 thick, 27 long, tip bevelled
        const int N = 60;
        auto top = [](double z) {
            double h = 3.6;
            for (int j = 0; j < 5; ++j) {
                double c = 6.27 + 3.81 * j, dep = 0.6 + 0.35 * ((j * 3) % 4);
                double dz = std::fabs(z - c);
                if (dz < 1.6) h = std::fmin(h, 3.6 - dep * (1 - dz / 1.6));
            }
            if (z < 3) h = std::fmin(h, -1.5 + z * 1.7);  // tip bevel
            return h;
        };
        glColor3d(br, bg, bb);
        glBegin(GL_QUADS);
        for (int i = 0; i < N; ++i) {
            double z0 = 27.0 * i / N, z1 = 27.0 * (i + 1) / N;
            double t0 = top(z0), t1 = top(z1), b0 = -4.6 + (z0 < 2 ? (2 - z0) * 1.2 : 0), b1 = -4.6 + (z1 < 2 ? (2 - z1) * 1.2 : 0);
            for (int s = -1; s <= 1; s += 2) {
                double x = s * 1.05;
                quad(Vec3(x, b0, z0), Vec3(x, b1, z1), Vec3(x, t1, z1), Vec3(x, t0, z0), Vec3(s, 0, 0));
            }
            Vec3 tn(0, z1 - z0, -(t1 - t0));
            tn = tn * (1.0 / tn.norm());
            quad(Vec3(-1.05, t0, z0), Vec3(1.05, t0, z0), Vec3(1.05, t1, z1), Vec3(-1.05, t1, z1), tn);
            quad(Vec3(-1.05, b0, z0), Vec3(-1.05, b1, z1), Vec3(1.05, b1, z1), Vec3(1.05, b0, z0), Vec3(0, -1, 0));
        }
        glEnd();
        // keyway grooves (darker lines along the blade)
        glDisable(GL_LIGHTING);
        glColor3d(br * 0.55, bg * 0.55, bb * 0.55);
        glBegin(GL_LINES);
        for (int s = -1; s <= 1; s += 2) {
            glVertex3d(s * 1.06, -1.0, 2); glVertex3d(s * 1.06, -1.0, 27);
            glVertex3d(s * 1.06, 1.4, 4); glVertex3d(s * 1.06, 1.4, 27);
        }
        glEnd();
        glEnable(GL_LIGHTING);
        // shoulder + bow (rounded, with a hole)
        box(Vec3(0, 0, 28.2), Vec3(1.2, 5.6, 1.2), br, bg, bb);
        glColor3d(br, bg, bb);
        for (int s = -1; s <= 1; s += 2) {
            glPushMatrix();
            glTranslated(s * 1.2, 0, 41);
            glRotated(90, 0, 1, 0);
            gluDisk(viz::quadric(), 3.2, 12.5, 32, 1);
            glPopMatrix();
        }
        glPushMatrix();
        glTranslated(-1.2, 0, 41);
        glRotated(90, 0, 1, 0);
        gluCylinder(viz::quadric(), 12.5, 12.5, 2.4, 32, 1);
        gluCylinder(viz::quadric(), 3.2, 3.2, 2.4, 16, 1);
        glPopMatrix();
        glPopMatrix();
        matte();
    }

    void plugObj(const adl::View& v, const adl::Insertion& pl, bool jam) {
        objFrame(v.objTip, pl.n, v.held ? v.roll : 0);
        shiny(0.95, 90);
        cyl(Vec3(0, 0, 0), Vec3(0, 0, 1), 2.4, 19.6, 0.78, 0.76, 0.7);       // ground pin (leads by 3.2)
        glColor3d(0.8, 0.78, 0.72);
        gluSphere(viz::quadric(), 2.4, 16, 8);
        for (int s = -1; s <= 1; s += 2) {                                       // blades
            double hh = s < 0 ? 3.95 : 3.18;
            box(Vec3(s * 6.35, 11.9, 3.2 + 8.2), Vec3(0.76, hh, 8.2), 0.82, 0.8, 0.74);
            cyl(Vec3(s * 6.35 - 0.8, 11.9, 6.0), Vec3(1, 0, 0), 1.3, 1.62, 0.25, 0.25, 0.25);  // blade holes
        }
        shiny(0.25, 20);
        double br = jam ? 0.5 : 0.1, bgc = 0.1, bbc = 0.11;
        roundBox(Vec3(0, 7, 19.1 + 12.5), Vec3(11.5, 14.5, 12.5), 3.5, br, bgc, bbc);
        for (int i = 0; i < 4; ++i)  // grip ridges
            box(Vec3(0, 7, 24 + i * 4.5), Vec3(11.8, 14.8, 0.6), br * 1.6 + 0.04, bgc * 1.6, bbc * 1.6);
        // cord: a sagging tube out of the back
        glColor3d(0.12, 0.12, 0.13);
        Vec3 prev(0, 5, 44);
        for (int i = 1; i <= 16; ++i) {
            double tt = i / 16.0;
            Vec3 pt(4 * tt, 5 - 60 * tt * tt, 44 + 40 * tt);
            Vec3 d = pt - prev;
            double L = d.norm();
            cyl(prev, d * (1.0 / L), 2.6, L, 0.12, 0.12, 0.13, false, 10);
            prev = pt;
        }
        glPopMatrix();
        matte();
    }

    void card(const adl::View& v, const adl::Insertion& a, bool jam) {
        objFrame(v.objTip, a.n, v.held ? v.roll : 0);
        shiny(0.55, 50);
        // card: 54 (x) x 0.76 (y) x 85.6 (z), leading edge at z = 0
        box(Vec3(0, 0, 42.8), Vec3(27, 0.38, 42.8), jam ? 0.6 : 0.1, jam ? 0.2 : 0.35, jam ? 0.25 : 0.75);
        glDisable(GL_LIGHTING);
        glBegin(GL_QUADS);  // face gradient
        glColor3d(0.12, 0.42, 0.85); glVertex3d(-27, 0.39, 0); glVertex3d(27, 0.39, 0);
        glColor3d(0.05, 0.2, 0.5); glVertex3d(27, 0.39, 85.6); glVertex3d(-27, 0.39, 85.6);
        glColor3d(0.08, 0.08, 0.08);  // magnetic stripe on the back
        glVertex3d(15, -0.39, 0); glVertex3d(15, -0.39, 85.6); glVertex3d(24, -0.39, 85.6); glVertex3d(24, -0.39, 0);
        glEnd();
        glEnable(GL_LIGHTING);
        shiny(0.9, 90);
        box(Vec3(-13, 0.45, 70), Vec3(5, 0.06, 6), 0.85, 0.72, 0.35);  // chip
        text3D(Vec3(-20, 0.42, 62), Vec3(1, 0, 0), Vec3(0, 0, -1), 6, "BANK", 0.95, 0.95, 1.0, 1.5);
        text3D(Vec3(-22, 0.42, 40), Vec3(1, 0, 0), Vec3(0, 0, -1), 3.5, "1234 5678", 0.85, 0.85, 0.9, 1.0);
        glPopMatrix();
        matte();
    }

    void forkObj(const adl::View& v) {
        Vec3 tip = v.objTip;
        shiny(0.95, 100);
        for (int i = 0; i < 4; ++i) {  // 4 tines, 2.2 wide, 40 long, tapered tips
            double x = tip.x + (-5.25 + 3.5 * i);
            box(Vec3(x, tip.y + 22, tip.z), Vec3(1.1, 18, 0.9), 0.8, 0.82, 0.86);
            glColor3d(0.8, 0.82, 0.86);
            glBegin(GL_TRIANGLES);
            glNormal3d(0, 0, 1);
            glVertex3d(x - 1.1, tip.y + 4, tip.z + 0.9); glVertex3d(x + 1.1, tip.y + 4, tip.z + 0.9); glVertex3d(x, tip.y, tip.z);
            glNormal3d(0, 0, -1);
            glVertex3d(x - 1.1, tip.y + 4, tip.z - 0.9); glVertex3d(x, tip.y, tip.z); glVertex3d(x + 1.1, tip.y + 4, tip.z - 0.9);
            glEnd();
        }
        box(Vec3(tip.x, tip.y + 43, tip.z), Vec3(7.6, 3.5, 1.0), 0.8, 0.82, 0.86);   // bridge
        box(Vec3(tip.x, tip.y + 55, tip.z), Vec3(3.2, 9, 1.0), 0.8, 0.82, 0.86);     // neck
        box(Vec3(tip.x, tip.y + 100, tip.z), Vec3(6.0, 37, 1.0), 0.82, 0.84, 0.88);  // handle (2 mm thick: the pinch)
        matte();
    }

    void putty(const adl::View& v, const adl::Fork& f) {
        shiny(0.25, 25);
        double r = f.radius, top = f.top.y, bottom = f.top.y - f.thick;
        cyl(Vec3(f.top.x, bottom, f.top.z), Vec3(0, 1, 0), r, f.thick - 3, 0.22, 0.66, 0.6, false, 40);
        // rounded rim + top
        glColor3d(0.24, 0.7, 0.63);
        glBegin(GL_QUAD_STRIP);
        for (int i = 0; i <= 40; ++i) {
            double a = 2 * kPi * i / 40, ca = std::cos(a), sa = std::sin(a);
            glNormal3d(ca * 0.7, 0.7, sa * 0.7);
            glVertex3d(f.top.x + ca * r, top - 3, f.top.z + sa * r);
            glVertex3d(f.top.x + ca * (r - 3), top, f.top.z + sa * (r - 3));
        }
        glEnd();
        glPushMatrix();
        glTranslated(f.top.x, top, f.top.z);
        glRotated(-90, 1, 0, 0);
        glNormal3d(0, 1, 0);
        gluDisk(viz::quadric(), 0, r - 3, 40, 2);
        glPopMatrix();
        // dents: 4 tine holes per stab
        glDisable(GL_LIGHTING);
        for (int i = 0; i < v.nDents; ++i) {
            const adl::Fork::Dent& d = v.dents[i];
            double dark = 0.35 - 0.012 * d.depth;
            glColor3d(0.05, std::fmax(0.1, dark), 0.15);
            for (int k = 0; k < 4; ++k) {
                double x = f.top.x + d.x - 5.25 + 3.5 * k, z = f.top.z + d.z;
                glBegin(GL_TRIANGLE_FAN);
                glVertex3d(x, top + 0.08, z);
                for (int j = 0; j <= 10; ++j) {
                    double a = 2 * kPi * j / 10;
                    glVertex3d(x + 1.2 * std::cos(a), top + 0.08, z + 0.8 * std::sin(a));
                }
                glEnd();
            }
        }
        glEnable(GL_LIGHTING);
        matte();
    }

    void zipper(const adl::View& v, const adl::Zipper& z) {
        bool vertical = z.vertical;
        Vec3 across = vertical ? Vec3(1, 0, 0) : Vec3(0, 1, 0);
        double chain = vertical ? 5.8 : 7.6;
        double tr = vertical ? 0.78 : 0.42, tg = vertical ? 0.76 : 0.43, tb = vertical ? 0.7 : 0.46;  // nickel / grey molded
        double tapeR = vertical ? 0.08 : 0.07, tapeG = vertical ? 0.1 : 0.07, tapeB = vertical ? 0.2 : 0.08;
        auto spread = [&](double s) { return s > v.zipS + 4 ? std::fmin(9.0, (s - v.zipS - 4) * 0.22) : 0.0; };
        // tapes (8 mm wide each side), following the V above the slider
        glColor3d(tapeR, tapeG, tapeB);
        glBegin(GL_QUADS);
        for (int side = -1; side <= 1; side += 2) {
            for (double s = -4; s < z.L + 4; s += 2) {
                double a0 = chain / 2 + spread(s), a1 = chain / 2 + spread(s + 2);
                Vec3 p0 = z.s0 + z.t * s + across * (side * a0) + z.nf * 0.6;
                Vec3 p1 = z.s0 + z.t * (s + 2) + across * (side * a1) + z.nf * 0.6;
                Vec3 q0 = p0 + across * (side * 8.0), q1 = p1 + across * (side * 8.0);
                quad(p0, p1, q1, q0, z.nf);
            }
        }
        glEnd();
        // teeth: alternating rows at the real pitch; interlocked below the slider, spreading above
        shiny(vertical ? 0.8 : 0.3, vertical ? 70 : 20);
        double hp = z.pitch * 0.3, hw = chain * 0.3;
        for (int side = -1; side <= 1; side += 2) {
            for (double s = (side > 0 ? z.pitch / 2 : 0); s < z.L; s += z.pitch) {
                double off = chain * 0.22 + spread(s);
                if (s < v.zipS - 3) off = chain * 0.15;  // meshed
                Vec3 c = z.s0 + z.t * s + across * (side * off) + z.nf * 1.3;
                box(c, Vec3(std::fabs((z.t * hp + across * hw).x), std::fabs((z.t * hp + across * hw).y), 0.9), tr, tg, tb);
            }
        }
        // slider body + tab
        shiny(0.9, 90);
        Vec3 piv = z.s0 + z.t * v.zipS + z.nf * 3.0;
        Vec3 sh = z.t * (vertical ? 6.0 : 7.5) + across * (vertical ? 5.0 : 6.5);
        roundBox(piv - z.nf * 1.2, Vec3(std::fabs(sh.x), std::fabs(sh.y), 2.4), 1.2, 0.72, 0.72, 0.75);
        Vec3 te = v.held ? v.tabEnd : v.handle;
        Vec3 d = te - piv;
        double L = d.norm();
        if (L > 0.5) {
            Vec3 Y = d * (1.0 / L);
            Vec3 X = across - Y * viz::dot(across, Y);
            if (X.norm() < 1e-3) X = z.t;
            X = X * (1.0 / X.norm());
            Vec3 Z = viz::cross(X, Y);
            pushFrame(piv, X, Y, Z);
            double w = vertical ? 4.0 : 3.5;
            roundBox(Vec3(0, L / 2, 0), Vec3(w, L / 2 + 1.0, 0.7), 0.6, 0.8, 0.8, 0.82);
            cyl(Vec3(0, L - 2.5, -0.8), Vec3(0, 0, 1), 1.6, 1.6, 0.25, 0.25, 0.27, true, 12);  // hole in the pull
            glPopMatrix();
        }
        matte();
    }

    void remote(const adl::View& v, const adl::Remote& r) {
        shiny(0.4, 30);
        double top = r.capTop0() - v.capX;
        cyl(Vec3(r.btn.x, r.btn.y - 1.0, r.btn.z), Vec3(0, 1, 0), r.capR, top - (r.btn.y - 1.0), 0.78, 0.8, 0.84, true, 32);
        glow(Vec3(r.btn.x + 11, r.btn.y + 0.6, r.btn.z - 14), 1.4, 1.0, 0.15, 0.1, v.clicked ? 1.0 : 0.25);
        matte();
    }

    // Thumb + index fingertip pinching the object at the stylus (the paper's pinch opening).
    void hand(const adl::View& v, const Vec3& cur) {
        double skin[3] = {0.93, 0.74, 0.62};
        if (v.task == adl::REMOTE || !v.held) {
            shiny(0.2, 15);
            glColor3d(skin[0], skin[1], skin[2]);
            glPushMatrix();
            glTranslated(cur.x, cur.y, cur.z);
            glScaled(1.0, 1.0, 1.25);
            gluSphere(viz::quadric(), 3.2, 18, 12);
            glPopMatrix();
            matte();
            return;
        }
        Vec3 at = v.handle;  // fingers stay on the object (the stylus may be pressed past it)
        Vec3 ax = pinchAxis(v.task);
        double half = adl::info(v.task).pinch_mm / 2 + 3.0;
        if (v.task == adl::ATM) half = 0.38 + 3.0;
        shiny(0.2, 15);
        glColor3d(skin[0], skin[1], skin[2]);
        for (int s = -1; s <= 1; s += 2) {
            Vec3 c = at + ax * (s * half);
            glPushMatrix();
            glTranslated(c.x, c.y, c.z);
            glScaled(ax.x != 0 ? 0.8 : 1.15, ax.y != 0 ? 0.8 : 1.15, ax.z != 0 ? 0.8 : 1.15);
            gluSphere(viz::quadric(), 3.6, 18, 12);
            glPopMatrix();
        }
        matte();
    }

    // Pulsing ring around the place to press the button (second action).
    void hint(const adl::View& v) {
        if (v.stage != adl::LEFT) return;
        double a = 0.5 + 0.5 * std::sin(t * 6);
        glDisable(GL_LIGHTING);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE);
        glDepthMask(GL_FALSE);
        glColor4d(1.0, 0.85, 0.2, 0.25 + 0.35 * a);
        glPushMatrix();
        glTranslated(v.handle.x, v.handle.y, v.handle.z);
        gluSphere(viz::quadric(), 6 + 2 * a, 16, 12);
        glPopMatrix();
        glDepthMask(GL_TRUE);
        glDisable(GL_BLEND);
        glEnable(GL_LIGHTING);
    }

    void drawScene(const adl::AdlCore& c, const adl::View& v, const Vec3& cur) {
        t += 1.0 / 60;
        background();
        lights();
        station(c, v.task);
        // the moving parts
        switch (v.task) {
        case adl::KEY: key(v, c.key, v.jam); break;
        case adl::PLUG: plugObj(v, c.plug, v.jam); break;
        case adl::ATM: {
            card(v, c.atm, v.jam);
            double zf = c.atm.E.z + c.atm.cham;  // reader LED: green = ready, blinking = card ejected
            double led = v.atm == 0 ? 1.0 : (v.atm == 5 ? (std::fmod(t, 0.6) < 0.3 ? 1.0 : 0.2) : 0.5);
            glow(Vec3(c.atm.E.x + 34, c.atm.E.y + 5, zf + 8.5), 1.2, 0.2, 1.0, 0.3, led);
            break;
        }
        case adl::FORK: putty(v, c.fork); forkObj(v); break;
        case adl::VZIP: zipper(v, c.vzip); break;
        case adl::HZIP: zipper(v, c.hzip); break;
        default: remote(v, c.remote); break;
        }
        hint(v);
        hand(v, cur);
        // shadow of the hand on the floor / table
        double floorY = (v.task == adl::FORK) ? c.fork.top.y - c.fork.thick - 3
                        : (v.task == adl::REMOTE) ? c.remote.body.y - c.remote.bodyH.y : -75;
        viz::drawShadow(cur, 4.5, floorY);
    }

    // ---------------- HUD ----------------
    static const char* instruction(const adl::View& v) {
        bool ins = v.task == adl::KEY || v.task == adl::PLUG || v.task == adl::ATM;
        const char* obj = v.task == adl::KEY ? "KEY" : v.task == adl::PLUG ? "PLUG" : v.task == adl::ATM ? "CARD" : "FORK";
        static char b[120];
        switch (v.stage) {
        case adl::HAND:
            if (v.task == adl::REMOTE) return "PRESS THE DOORBELL BUTTON";
            if (v.task == adl::FORK) return "STAB THE FORK INTO THE PUTTY";
            std::snprintf(b, sizeof b, "PUSH THE %s INTO THE %s", obj,
                          v.task == adl::KEY ? "LOCK" : v.task == adl::PLUG ? "OUTLET" : "CARD SLOT");
            return b;
        case adl::PUSHING:
            if (v.task == adl::VZIP || v.task == adl::HZIP) return "PULL THE TAB TO CLOSE THE ZIPPER";
            if (v.task == adl::REMOTE) return "PRESS UNTIL IT CLICKS";
            return ins || v.task == adl::FORK ? "KEEP PUSHING" : "KEEP GOING";
        case adl::DONE_IN: return "DONE - EASE OFF TO LET GO";
        case adl::LEFT:
            if (v.task == adl::VZIP || v.task == adl::HZIP) return "PRESS THE BUTTON ON THE TAB, THEN OPEN THE ZIPPER";
            if (v.task == adl::ATM) return "TAKE YOUR CARD: PRESS THE BUTTON ON IT, THEN PULL";
            std::snprintf(b, sizeof b, "PRESS THE BUTTON ON THE %s, THEN PULL IT OUT", obj);
            return b;
        case adl::PULLING:
            if (v.task == adl::VZIP || v.task == adl::HZIP) return "PULL THE TAB BACK TO OPEN";
            return "PULL IT OUT";
        case adl::MACHINE: return v.atm == 4 ? "THE ATM IS RETURNING YOUR CARD..." : "THE ATM IS READING YOUR CARD...";
        case adl::WAITING: return "MOVE TO THE ZIPPER TAB TO PICK IT UP";
        }
        return "";
    }

    void drawHud(const adl::View& v) {
        int W = viz::ui().width, H = viz::ui().height;
        hud::begin2D(W, H);
        const adl::TaskInfo& ti = adl::info(v.task);
        char b[200];
        // --- top-left: task + instruction
        hud::panel(12, H - 132, 560, 120, 0.02, 0.03, 0.05, 0.62);
        std::snprintf(b, sizeof b, "%d/7  %s", v.task + 1, ti.name);
        hud::text(26, H - 46, 22, b, 1, 1, 1);
        hud::text(26, H - 70, 11, ti.desc, 0.7, 0.78, 0.9);
        hud::text(26, H - 100, 15, instruction(v), 1.0, 0.86, 0.3);
        if (v.jam) {
            std::snprintf(b, sizeof b, "TWISTED %.0f DEG - STRAIGHTEN THE STYLUS", std::fabs(v.roll) * 180 / kPi);
            hud::text(26, H - 124, 13, b, 1.0, 0.35, 0.3);
        } else if (v.task == adl::KEY || v.task == adl::PLUG || v.task == adl::ATM) {
            std::snprintf(b, sizeof b, "TWIST %+.0f DEG   ROTATION %s (R)", v.roll * 180 / kPi, v.rollOn ? "ON" : "OFF");
            hud::text(26, H - 124, 11, b, 0.6, 0.65, 0.72);
        }
        // --- top-right: timer + reference
        double px = W - 372;
        hud::panel(px, H - 132, 360, 120, 0.02, 0.03, 0.05, 0.62);
        const char* act = v.timingDir == 0 ? ti.inName : ti.outName;
        const adl::RefTime& ref = v.timingDir == 0 ? ti.refIn : ti.refOut;
        if (v.timing) {
            hud::text(px + 14, H - 40, 13, act, 0.75, 0.85, 1.0);
            std::snprintf(b, sizeof b, "%5.2f S", v.timer);
            hud::text(px + 14, H - 82, 34, b, 1, 1, 1);
        } else {
            hud::text(px + 14, H - 40, 13, "TIMER STARTS AT CONTACT", 0.6, 0.65, 0.72);
            hud::text(px + 14, H - 82, 34, " -.-- S", 0.45, 0.5, 0.55);
        }
        if (ref.mean > 0) {
            std::snprintf(b, sizeof b, "ROBOT REF %.1f S (SD %.1f)", ref.mean, ref.sd);
            hud::text(px + 14, H - 110, 11, b, 0.65, 0.72, 0.8);
        }
        // --- right: recent results
        int shown = 0;
        double ry = H - 156;
        for (int i = v.nResults - 1; i >= 0 && shown < 6 && i >= v.nResults - adl::kRing; --i, ++shown) {
            const adl::Result& r = v.ring[i % adl::kRing];
            const adl::TaskInfo& rt = adl::info(r.task);
            const adl::RefTime& rr = r.dir == 0 ? rt.refIn : rt.refOut;
            bool within = rr.mean > 0 && std::fabs(r.time - rr.mean) <= 2 * rr.sd;
            std::snprintf(b, sizeof b, "%-14s %5.2f S", r.dir == 0 ? rt.inName : rt.outName, r.time);
            if (shown == 0) hud::panel(px, ry - 6 * 22 + 8, 360, 6 * 22 + 6, 0.02, 0.03, 0.05, 0.45);
            hud::text(px + 14, ry - shown * 22, 12, b, within ? 0.5 : 1.0, within ? 0.95 : 0.8, within ? 0.55 : 0.4);
        }
        // --- bottom: force bar with real-equivalent force and required pinch
        double fsim = v.objF;
        double freal = adl::toReal(fsim / (v.gain > 0 ? v.gain : 1));
        double target = adl::toSim(v.timingDir == 0 ? ti.realIn : (ti.realOut > 0 ? ti.realOut : ti.realIn)) * v.gain;
        hud::panel(12, 12, W - 24, 70, 0.02, 0.03, 0.05, 0.62);
        double mx = W - 360;              // status column on the right
        double bw = std::fmin(W * 0.42, mx - 60);
        hud::bar(26, 50, bw, 18, fsim, 3.68, 0.2 + 0.8 * adl::clampd(fsim / 3.0, 0, 1), 0.8, 0.35, target);
        std::snprintf(b, sizeof b, "FORCE %.2f N  REAL %.1f N  PINCH %.1f-%.1f N", fsim, freal, freal / (2 * 0.56),
                      freal / (2 * 0.33));
        hud::text(26, 24, 11, b, 0.85, 0.9, 0.95);
        // mode lines, right column
        if (v.protoActive)
            std::snprintf(b, sizeof b, "PROTOCOL  TASK %d/7  CYCLE %d/3", v.protoIdx + 1, v.rep + (v.rep < 3 ? 1 : 0));
        else
            std::snprintf(b, sizeof b, "PRACTICE  1-7 TASK  A PROTOCOL");
        hud::text(mx, 54, 11, b, 0.75, 0.8, 0.9);
        std::snprintf(b, sizeof b, "%sROTATION %s  GAIN %.2f", viz::ui().sim ? "SIMULATOR  " : "", v.rollOn ? "ON" : "OFF", v.gain);
        hud::text(mx, 28, 10, b, 0.55, 0.6, 0.7);
        // --- centre messages
        if (v.pauseLeft > 0 && v.protoActive && v.protoIdx + 1 < adl::NTASK) {
            std::snprintf(b, sizeof b, "NEXT: %s", adl::info(adl::kProtoOrder[v.protoIdx + 1]).name);
            double tw = hud::textWidth(b, 28);
            hud::panel(W / 2 - tw / 2 - 20, H / 2 - 30, tw + 40, 64, 0.02, 0.03, 0.05, 0.7);
            hud::text(W / 2 - tw / 2, H / 2 - 10, 28, b, 1, 0.9, 0.4);
        }
        if (v.protoDone) {
            const char* m = "PROTOCOL COMPLETE - RESULTS SAVED";
            double tw = hud::textWidth(m, 24);
            hud::panel(W / 2 - tw / 2 - 20, H / 2 - 30, tw + 40, 60, 0.02, 0.05, 0.03, 0.75);
            hud::text(W / 2 - tw / 2, H / 2 - 10, 24, m, 0.5, 1, 0.6);
        }
        hud::end2D();
    }
};

}  // namespace adl_draw
