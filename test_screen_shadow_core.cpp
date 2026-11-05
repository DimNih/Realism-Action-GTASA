// Host-side check of the sun-map math. Not part of the Android shared library.
//   g++ -std=c++17 -O2 test_screen_shadow_core.cpp -o test_ss_core && ./test_ss_core
#include "screen_shadow_core.h"
#include <cstdio>
using namespace ShadowMap;

static int g_fail = 0;
#define CHECK(cond, ...) do { if (!(cond)) { ++g_fail; std::printf("FAIL: " __VA_ARGS__); std::printf("\n"); } else { std::printf("ok:   " __VA_ARGS__); std::printf("\n"); } } while (0)

int main() {
    // --- sun vector: shadows pointing +y at noon mean the sun is south (-y) and above
    float s[3]; SunVector(0.0f, 1.0f, 60.0f * kDeg, s);
    CHECK(std::fabs(Dot3(s, s) - 1.0f) < 1e-5f && s[1] < 0 && s[2] > 0.8f, "sun vector unit, south, high (%.2f %.2f %.2f)", s[0], s[1], s[2]);
    SunVector(1.0f, 0.0f, 2.0f * kDeg, s);
    CHECK(std::fabs(s[2] - std::sin(kMinSunElevation)) < 1e-4f, "elevation clamped to the minimum (%.3f)", s[2]);

    // --- basis is orthonormal for a range of suns
    bool ortho = true;
    for (int az = 0; az < 360; az += 30) for (int el = 12; el <= 68; el += 8) {
        float v[3]; SunVector(std::cos(az * kDeg), std::sin(az * kDeg), el * kDeg, v);
        float R[3], U[3]; LightBasis(v, R, U);
        if (std::fabs(Dot3(R, v)) > 1e-4f || std::fabs(Dot3(U, v)) > 1e-4f || std::fabs(Dot3(R, U)) > 1e-4f ||
            std::fabs(Dot3(R, R) - 1) > 1e-4f || std::fabs(Dot3(U, U) - 1) > 1e-4f || U[2] <= 0) ortho = false;
    }
    CHECK(ortho, "light basis orthonormal, U points up");

    // --- a point ON the sun ray keeps its light-space x/y, and depth grows away from the sun
    const double cam[3] = {2437.3, -1782.1, 11.0}; const float fwd[3] = {0, 1, 0};
    float toSun[3]; SunVector(0.3f, 0.9f, 40.0f * kDeg, toSun);
    Cascade c; CHECK(BuildCascade(cam, fwd, toSun, 30.0f, 150.0f, 0.4f, 1536, c), "cascade builds");
    float p0[3] = {5, 7, 1}, p1[3] = {p0[0] + toSun[0] * 9, p0[1] + toSun[1] * 9, p0[2] + toSun[2] * 9};
    float l0[3], l1[3]; ToLight(c, p0, l0); ToLight(c, p1, l1);
    CHECK(std::fabs(l0[0] - l1[0]) < 1e-5f && std::fabs(l0[1] - l1[1]) < 1e-5f, "same texel along the sun ray");
    CHECK(l1[2] < l0[2], "point nearer the sun has the smaller depth (%.4f < %.4f)", l1[2], l0[2]);

    // --- texel snapping: a fixed world point must land on the SAME fractional texel position however the
    // camera moves, otherwise shadow edges shimmer. Drift is measured in texels.
    double maxDrift = 0, fx0 = 0, fy0 = 0;
    const double W[3] = {2441.0, -1775.0, 12.0};
    for (int i = 0; i < 400; ++i) {
        const double cam2[3] = {cam[0] + 0.0173 * i, cam[1] - 0.0091 * i, cam[2] + 0.002 * i};
        Cascade cc; BuildCascade(cam2, fwd, toSun, 30.0f, 150.0f, 0.4f, 1536, cc);
        const float wp[3] = {float(W[0] - cam2[0]), float(W[1] - cam2[1]), float(W[2] - cam2[2])};
        float lp[3]; ToLight(cc, wp, lp);
        const double tx = (lp[0] * 0.5 + 0.5) * 1536.0, ty = (lp[1] * 0.5 + 0.5) * 1536.0;
        const double fx = tx - std::floor(tx), fy = ty - std::floor(ty);
        if (i == 0) { fx0 = fx; fy0 = fy; }
        double dx = std::fabs(fx - fx0), dy = std::fabs(fy - fy0);
        dx = std::min(dx, 1 - dx); dy = std::min(dy, 1 - dy);
        maxDrift = std::max(maxDrift, std::max(dx, dy));
    }
    CHECK(maxDrift < 0.02, "texel snapping keeps the map world-anchored (max drift %.4f texel)", maxDrift);

    // --- the region in front of the camera lies inside the cascade
    float inFront[3] = {0.0f, 12.0f, -1.5f}, behind[3] = {0.0f, -60.0f, 0.0f}, l[3];
    ToLight(c, inFront, l);
    CHECK(std::fabs(l[0]) < 1 && std::fabs(l[1]) < 1 && std::fabs(l[2]) < 1, "point 12 m ahead is inside the 30 m cascade");
    ToLight(c, behind, l);
    CHECK(std::fabs(l[0]) > 1 || std::fabs(l[1]) > 1, "point 60 m behind is outside it");

    // --- ViewForward from an inverse view-projection (camera looking along +y, z up)
    float proj[16] = {}; const float t = 1.0f / std::tan(30.0f * kDeg);
    proj[0] = t / 1.7778f; proj[5] = t; proj[10] = -1.002f; proj[11] = -1.0f; proj[14] = -0.6f;
    float v2[16] = {}; v2[0] = 1; v2[9] = 1; v2[6] = -1; v2[15] = 1;      // rows: right (1,0,0), up (0,0,1), back (0,-1,0)
    float vp[16]; Mul4(proj, v2, vp);
    double a4[4][8] = {};
    for (int r = 0; r < 4; ++r) { for (int cc = 0; cc < 4; ++cc) a4[r][cc] = vp[cc * 4 + r]; a4[r][r + 4] = 1; }
    for (int cc = 0; cc < 4; ++cc) {
        int p = cc; for (int r = cc + 1; r < 4; ++r) if (std::fabs(a4[r][cc]) > std::fabs(a4[p][cc])) p = r;
        for (int k = 0; k < 8; ++k) std::swap(a4[p][k], a4[cc][k]);
        const double d = a4[cc][cc]; for (double& v : a4[cc]) v /= d;
        for (int r = 0; r < 4; ++r) if (r != cc) { const double fct = a4[r][cc]; for (int k = 0; k < 8; ++k) a4[r][k] -= fct * a4[cc][k]; }
    }
    float inv[16]; for (int r = 0; r < 4; ++r) for (int cc = 0; cc < 4; ++cc) inv[cc * 4 + r] = float(a4[r][cc + 4]);
    float f[3];
    CHECK(ViewForward(inv, f) && std::fabs(f[1] - 1.0f) < 1e-3f, "view forward recovered (%.3f %.3f %.3f)", f[0], f[1], f[2]);

    std::printf(g_fail ? "\n%d FAILED\n" : "\nall passed\n", g_fail);
    return g_fail ? 1 : 0;
}
