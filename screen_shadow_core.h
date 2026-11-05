#pragma once

// Pure math for the screen-built sun shadow map. No game, GL or Android dependency, so it is
// unit-testable on the host (see test_screen_shadow_core.cpp).
//
// Everything is camera-relative: the shaders reconstruct visible surface points as
// (world - cameraWorld), so the light matrices below map that camera-relative position straight to
// light clip space. Absolute world coordinates only appear here, in double precision, to snap the
// shadow map to whole texels (otherwise the shadow edges shimmer while the camera moves).
//
// Light frame: Lz points TOWARD the sun. R and U span the plane the map is drawn in.
//   x_ndc = (R . p + ta) / radius
//   y_ndc = (U . p + tb) / radius
//   z_ndc = -(Lz . p + tc) / depthHalf        (-1 = nearest to the sun, +1 = farthest)
// so a larger depth value means "further from the sun", and a surface is in shadow when something
// with a smaller depth already sits in the same texel.

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace ShadowMap {

constexpr float kDeg = 0.01745329252f;
constexpr float kMinSunElevation = 24.0f * kDeg;   // lower suns give streaks longer than any map and unstable, noisy edges

struct Cascade {
    float vp[16] = {};          // column-major, camera-relative world -> light clip
    float texel = 0.05f;        // metres per texel
    float radius = 20.0f;       // half width of the covered square, metres
    float depthHalf = 140.0f;   // half of the covered depth range, metres
    float lz[3] = {0, 0, 1};
    float ta = 0, tb = 0, tc = 0;
};

inline float Dot3(const float* a, const float* b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }

inline void Normalize3(float* v) {
    const float l = std::sqrt(Dot3(v, v));
    if (l > 1.0e-8f) { v[0] /= l; v[1] /= l; v[2] /= l; } else { v[0] = 0; v[1] = 0; v[2] = 1; }
}

// Unit vector toward the sun from the horizontal shadow direction (the way shadows point) and the
// sun elevation. The same model the legacy decal shadows use, so both stay consistent.
inline void SunVector(float shadowDirX, float shadowDirY, float elevation, float out[3]) {
    const float e = std::max(elevation, kMinSunElevation);
    const float h = std::sqrt(shadowDirX * shadowDirX + shadowDirY * shadowDirY);
    const float nx = h > 1.0e-6f ? shadowDirX / h : 0.0f, ny = h > 1.0e-6f ? shadowDirY / h : 1.0f;
    out[0] = -nx * std::cos(e);
    out[1] = -ny * std::cos(e);
    out[2] = std::sin(e);
    Normalize3(out);
}

// Orthonormal light basis. R is horizontal, U completes the frame (U . up > 0).
inline void LightBasis(const float toSun[3], float R[3], float U[3]) {
    const float up[3] = {0, 0, 1};
    R[0] = up[1] * toSun[2] - up[2] * toSun[1];
    R[1] = up[2] * toSun[0] - up[0] * toSun[2];
    R[2] = up[0] * toSun[1] - up[1] * toSun[0];
    if (Dot3(R, R) < 1.0e-8f) { R[0] = 1; R[1] = 0; R[2] = 0; }
    Normalize3(R);
    U[0] = toSun[1] * R[2] - toSun[2] * R[1];
    U[1] = toSun[2] * R[0] - toSun[0] * R[2];
    U[2] = toSun[0] * R[1] - toSun[1] * R[0];
    Normalize3(U);
}

// Fits one cascade. cam = camera world position, forward = camera view direction (world),
// centerFwd = how far in front of the camera the square is centred, as a fraction of radius.
inline bool BuildCascade(const double cam[3], const float forward[3], const float toSun[3], float radius,
                         float depthHalf, float centerFwd, int mapSize, Cascade& out) {
    if (!(radius > 0.5f) || !(depthHalf > 1.0f) || mapSize < 16) return false;
    float R[3], U[3];
    float L[3] = {toSun[0], toSun[1], toSun[2]};
    Normalize3(L);
    LightBasis(L, R, U);

    // Horizontal look direction; straight up/down views centre on the camera.
    float fh[3] = {forward[0], forward[1], 0.0f};
    const float fl = std::sqrt(fh[0] * fh[0] + fh[1] * fh[1]);
    if (fl > 0.2f) { fh[0] /= fl; fh[1] /= fl; } else { fh[0] = fh[1] = 0.0f; }
    const double c[3] = {cam[0] + fh[0] * radius * centerFwd, cam[1] + fh[1] * radius * centerFwd, cam[2]};

    const double texel = 2.0 * radius / mapSize;
    const double ca = c[0] * R[0] + c[1] * R[1] + c[2] * R[2];
    const double cb = c[0] * U[0] + c[1] * U[1] + c[2] * U[2];
    const double cc = c[0] * L[0] + c[1] * L[1] + c[2] * L[2];
    const double a0 = std::floor(ca / texel + 0.5) * texel;      // snapped to whole texels
    const double b0 = std::floor(cb / texel + 0.5) * texel;
    const double c0 = std::floor(cc / 0.5 + 0.5) * 0.5;

    const double camA = cam[0] * R[0] + cam[1] * R[1] + cam[2] * R[2];
    const double camB = cam[0] * U[0] + cam[1] * U[1] + cam[2] * U[2];
    const double camC = cam[0] * L[0] + cam[1] * L[1] + cam[2] * L[2];
    out.ta = static_cast<float>(camA - a0);
    out.tb = static_cast<float>(camB - b0);
    out.tc = static_cast<float>(camC - c0);
    out.radius = radius; out.depthHalf = depthHalf; out.texel = static_cast<float>(texel);
    out.lz[0] = L[0]; out.lz[1] = L[1]; out.lz[2] = L[2];

    std::fill(out.vp, out.vp + 16, 0.0f);
    const float ir = 1.0f / radius, iz = -1.0f / depthHalf;
    for (int k = 0; k < 3; ++k) {
        out.vp[k * 4 + 0] = R[k] * ir;
        out.vp[k * 4 + 1] = U[k] * ir;
        out.vp[k * 4 + 2] = L[k] * iz;
    }
    out.vp[12] = out.ta * ir; out.vp[13] = out.tb * ir; out.vp[14] = out.tc * iz; out.vp[15] = 1.0f;
    return true;
}

// Camera-relative point -> light clip (ndc, w = 1).
inline void ToLight(const Cascade& c, const float p[3], float out[3]) {
    for (int r = 0; r < 3; ++r) out[r] = c.vp[r] * p[0] + c.vp[4 + r] * p[1] + c.vp[8 + r] * p[2] + c.vp[12 + r];
}

// 4x4 helpers (column-major) shared by the host tests.
inline void Mul4(const float* a, const float* b, float* out) {
    for (int col = 0; col < 4; ++col)
        for (int row = 0; row < 4; ++row)
            out[col * 4 + row] = a[row] * b[col * 4] + a[4 + row] * b[col * 4 + 1] + a[8 + row] * b[col * 4 + 2] + a[12 + row] * b[col * 4 + 3];
}

// World-space view direction through the middle of the screen, from the inverse view-projection.
inline bool ViewForward(const float invVp[16], float out[3]) {
    float n[4], f[4];
    const float pn[4] = {0, 0, -1, 1}, pf[4] = {0, 0, 1, 1};
    for (int r = 0; r < 4; ++r) {
        n[r] = invVp[r] * pn[0] + invVp[4 + r] * pn[1] + invVp[8 + r] * pn[2] + invVp[12 + r] * pn[3];
        f[r] = invVp[r] * pf[0] + invVp[4 + r] * pf[1] + invVp[8 + r] * pf[2] + invVp[12 + r] * pf[3];
    }
    if (std::fabs(n[3]) < 1.0e-9f || std::fabs(f[3]) < 1.0e-9f) return false;
    for (int k = 0; k < 3; ++k) out[k] = f[k] / f[3] - n[k] / n[3];
    const float l = std::sqrt(Dot3(out, out));
    if (!(l > 1.0e-6f) || !std::isfinite(l)) return false;
    for (int k = 0; k < 3; ++k) out[k] /= l;
    return true;
}

// Size in texels of one splat for a visible point at view depth `w` (metres), given the projection
// scale p5 (= 1 / tan(fovY / 2)), the viewport height in pixels, the grid stride, and the texel size.
// Returned per metre of view depth so the vertex shader only multiplies by the distance.
inline float SplatTexelsPerMetre(float p5, int viewportHeight, int stride, float texelWorld) {
    const float pixelWorldPerMetre = 2.0f / (std::max(p5, 0.05f) * std::max(viewportHeight, 1));
    return pixelWorldPerMetre * static_cast<float>(std::max(stride, 1)) / std::max(texelWorld, 1.0e-4f);
}

} // namespace ShadowMap
