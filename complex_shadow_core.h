#pragma once

// Pure math for sun-shadow silhouettes. No game or Android dependency, so it is unit-testable
// on the host (see test_complex_shadow_core.cpp).
//
// The game's CShadows renderer draws ONE textured parallelogram per stored shadow and drapes it
// over the ground polygons. A rectangle texture gives the flat "box" shadow. Instead we project the
// caster's real geometry (collision triangles / boxes / spheres) onto the ground along the sun
// direction and rasterise that silhouette into a small alpha mask. The mask becomes the shadow
// texture, so every object casts a shadow that follows its own outline.
//
// Coordinates ("sun frame"), all relative to the caster's origin, in metres:
//   a = position along the shadow direction d (includes the height shear: a += (z - zRef) * L)
//   c = position across the shadow, along n = (d.y, -d.x)   (right-hand side of d)
//   L = horizontal shadow length per metre of height = 1 / tan(sun elevation)

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace ShadowCore {

// A modestly larger mask keeps building silhouettes readable at distance
// without increasing the number of shadow casters or stored decals.
constexpr int kMaskSize = 80;               // output mask is kMaskSize x kMaskSize
constexpr int kSuper = 2;                   // supersampling factor used while rasterising
constexpr int kInternal = kMaskSize * kSuper;
constexpr int kMaxShapes = 8192;

struct V3 { float x, y, z; };

// Convex polygon (<= 8 vertices) or ellipse in the sun frame.
struct Shape {
    uint8_t kind = 0;                       // 0 = convex polygon, 1 = ellipse
    uint8_t n = 0;
    float a[8], c[8];                       // polygon vertices CCW (a,c); ellipse: a[0],c[0] centre, a[1]=ra, c[1]=rc
    float minA = 0, maxA = 0, minC = 0, maxC = 0;
};

// Placement of the mask on the ground: centre and half extents in the sun frame.
struct Quad {
    float aC = 0, cC = 0, halfA = 1, halfC = 1;
};

// How the engine's texture axes map to the quad. bit0: mirror U (side), bit1: mirror V (front),
// bit2: swap the axes. Default 0 = front of the shadow at the TOP of the texture, side to the right.
inline void PixelToAC(const Quad& q, int orient, float px, float py, float& a, float& c) {
    const float u = px / kInternal, v = py / kInternal;
    float su = 2.0f * u - 1.0f;              // +1 at right edge
    float sv = 1.0f - 2.0f * v;              // +1 at top edge
    if (orient & 1) su = -su;
    if (orient & 2) sv = -sv;
    if (orient & 4) { a = q.aC + q.halfA * su; c = q.cC + q.halfC * sv; }
    else            { c = q.cC + q.halfC * su; a = q.aC + q.halfA * sv; }
}
inline void ACToPixel(const Quad& q, int orient, float a, float c, float& px, float& py) {
    float su, sv;
    if (orient & 4) { su = (a - q.aC) / q.halfA; sv = (c - q.cC) / q.halfC; }
    else            { su = (c - q.cC) / q.halfC; sv = (a - q.aC) / q.halfA; }
    if (orient & 1) su = -su;
    if (orient & 2) sv = -sv;
    px = (su + 1.0f) * 0.5f * kInternal;
    py = (1.0f - sv) * 0.5f * kInternal;
}

inline float Clamp(float v, float lo, float hi) { return std::max(lo, std::min(hi, v)); }
inline float Smooth01(float t) { t = Clamp(t, 0.0f, 1.0f); return t * t * (3.0f - 2.0f * t); }

class Builder {
public:
    // d = unit horizontal shadow direction, L = shadow length per metre of height,
    // zRef = height (relative to the caster origin) of the ground the shadow falls on.
    void Begin(float dx, float dy, float L, float zRef) {
        dx_ = dx; dy_ = dy; nx_ = dy; ny_ = -dx; L_ = L; zRef_ = zRef; count_ = 0; overflow_ = false;
    }
    int ShapeCount() const { return count_; }
    bool Overflowed() const { return overflow_; }

    void ToSun(const V3& p, float& a, float& c) const {
        a = p.x * dx_ + p.y * dy_ + (p.z - zRef_) * L_;
        c = p.x * nx_ + p.y * ny_;
    }

    void AddTriangle(const V3& p0, const V3& p1, const V3& p2) {
        float pa[3], pc[3];
        ToSun(p0, pa[0], pc[0]); ToSun(p1, pa[1], pc[1]); ToSun(p2, pa[2], pc[2]);
        AddPolygon(pa, pc, 3);
    }

    // Convex hull of the projected corners of a (possibly rotated) box.
    void AddBox(const V3 corner[8]) {
        float pa[8], pc[8];
        for (int i = 0; i < 8; ++i) ToSun(corner[i], pa[i], pc[i]);
        float ha[8], hc[8];
        const int n = Hull(pa, pc, 8, ha, hc);
        if (n >= 3) AddPolygon(ha, hc, n, true);
    }

    // Parallel projection of a sphere is an ellipse: across radius r, along radius r*sqrt(1+L^2).
    void AddSphere(const V3& centre, float r) {
        if (!(r > 0.01f) || count_ >= kMaxShapes) { overflow_ |= count_ >= kMaxShapes; return; }
        Shape& s = shapes_[count_];
        s.kind = 1; s.n = 2;
        float a, c; ToSun(centre, a, c);
        const float ra = r * std::sqrt(1.0f + L_ * L_);
        s.a[0] = a; s.c[0] = c; s.a[1] = ra; s.c[1] = r;
        s.minA = a - ra; s.maxA = a + ra; s.minC = c - r; s.maxC = c + r;
        ++count_;
    }

    // Bounds of everything added so far, before any clamping.
    bool Bounds(float& aMin, float& aMax, float& cMin, float& cMax) const {
        if (count_ <= 0) return false;
        aMin = cMin = 1.0e9f; aMax = cMax = -1.0e9f;
        for (int i = 0; i < count_; ++i) {
            aMin = std::min(aMin, shapes_[i].minA); aMax = std::max(aMax, shapes_[i].maxA);
            cMin = std::min(cMin, shapes_[i].minC); cMax = std::max(cMax, shapes_[i].maxC);
        }
        return std::isfinite(aMin + aMax + cMin + cMax);
    }

    // Fits the quad around the shapes (anchored at the caster end when it must be clamped) and
    // rasterises the mask. mask must hold kMaskSize*kMaskSize bytes. Returns false if empty.
    bool Finish(int orient, float maxHalfA, float maxHalfC, uint8_t* mask, Quad& quad) {
        float aMin, aMax, cMin, cMax;
        if (!Bounds(aMin, aMax, cMin, cMax)) return false;
        const float padA = std::max(0.25f, (aMax - aMin) * 0.04f);
        const float padC = std::max(0.20f, (cMax - cMin) * 0.04f);
        aMin -= padA; aMax += padA; cMin -= padC; cMax += padC;
        // Clamp: keep the end nearest the caster (the root of the shadow) when too long.
        if (aMax - aMin > 2.0f * maxHalfA) {
            // the root of the shadow is at the lower 'a' end for L >= 0
            aMax = aMin + 2.0f * maxHalfA;
        }
        if (cMax - cMin > 2.0f * maxHalfC) {
            const float mid = 0.5f * (cMin + cMax);
            cMin = mid - maxHalfC; cMax = mid + maxHalfC;
        }
        quad.aC = 0.5f * (aMin + aMax); quad.cC = 0.5f * (cMin + cMax);
        quad.halfA = std::max(0.6f, 0.5f * (aMax - aMin));
        quad.halfC = std::max(0.5f, 0.5f * (cMax - cMin));

        // Rasterise at kSuper x resolution into a 1-bit buffer.
        static uint8_t hi[kInternal * kInternal];
        std::fill(hi, hi + kInternal * kInternal, uint8_t(0));
        for (int i = 0; i < count_; ++i) Raster(shapes_[i], quad, orient, hi);

        // Downsample (box) -> blur ([1 2 1]^2) -> fade the quad border so a clamped tip never cuts hard.
        static float lo[kMaskSize * kMaskSize], tmp[kMaskSize * kMaskSize];
        constexpr float inv = 1.0f / float(kSuper * kSuper);
        for (int y = 0; y < kMaskSize; ++y)
            for (int x = 0; x < kMaskSize; ++x) {
                int sum = 0;
                for (int sy = 0; sy < kSuper; ++sy)
                    for (int sx = 0; sx < kSuper; ++sx) sum += hi[(y * kSuper + sy) * kInternal + x * kSuper + sx];
                lo[y * kMaskSize + x] = sum * inv;
            }
        for (int y = 0; y < kMaskSize; ++y)
            for (int x = 0; x < kMaskSize; ++x) {
                const float l = lo[y * kMaskSize + std::max(0, x - 1)], m = lo[y * kMaskSize + x],
                            r = lo[y * kMaskSize + std::min(kMaskSize - 1, x + 1)];
                tmp[y * kMaskSize + x] = (l + 6.0f * m + r) * 0.125f;
            }
        for (int y = 0; y < kMaskSize; ++y)
            for (int x = 0; x < kMaskSize; ++x) {
                const float u = tmp[std::max(0, y - 1) * kMaskSize + x], m = tmp[y * kMaskSize + x],
                            d = tmp[std::min(kMaskSize - 1, y + 1) * kMaskSize + x];
                float v = (u + 6.0f * m + d) * 0.125f;
                const float ex = std::min(x + 0.5f, kMaskSize - x - 0.5f);
                const float ey = std::min(y + 0.5f, kMaskSize - y - 0.5f);
                v *= Smooth01(std::min(ex, ey) / 2.0f);           // 2-pixel border fade
                mask[y * kMaskSize + x] = static_cast<uint8_t>(Clamp(v, 0.0f, 1.0f) * 255.0f + 0.5f);
            }
        return true;
    }

private:
    void AddPolygon(const float* pa, const float* pc, int n, bool alreadyHull = false) {
        if (count_ >= kMaxShapes) { overflow_ = true; return; }
        Shape& s = shapes_[count_];
        s.kind = 0; s.n = static_cast<uint8_t>(n);
        float area2 = 0.0f;
        for (int i = 0; i < n; ++i) {
            s.a[i] = pa[i]; s.c[i] = pc[i];
            const int j = (i + 1) % n;
            area2 += pa[i] * pc[j] - pa[j] * pc[i];
        }
        if (!std::isfinite(area2)) return;
        if (!alreadyHull && std::fabs(area2) < 1.0e-7f) return;      // degenerate projection
        if (area2 < 0.0f) {                                          // force CCW
            for (int i = 0; i < n / 2; ++i) { std::swap(s.a[i], s.a[n - 1 - i]); std::swap(s.c[i], s.c[n - 1 - i]); }
        }
        s.minA = s.maxA = s.a[0]; s.minC = s.maxC = s.c[0];
        for (int i = 1; i < n; ++i) {
            s.minA = std::min(s.minA, s.a[i]); s.maxA = std::max(s.maxA, s.a[i]);
            s.minC = std::min(s.minC, s.c[i]); s.maxC = std::max(s.maxC, s.c[i]);
        }
        ++count_;
    }

    // Andrew's monotone chain; returns CCW hull.
    static int Hull(const float* xa, const float* xc, int n, float* ha, float* hc) {
        int idx[16]; for (int i = 0; i < n; ++i) idx[i] = i;
        std::sort(idx, idx + n, [&](int p, int q) { return xa[p] < xa[q] || (xa[p] == xa[q] && xc[p] < xc[q]); });
        int h[32], k = 0;
        auto cross = [&](int o, int a, int b) { return (xa[a] - xa[o]) * (xc[b] - xc[o]) - (xc[a] - xc[o]) * (xa[b] - xa[o]); };
        for (int i = 0; i < n; ++i) { while (k >= 2 && cross(h[k - 2], h[k - 1], idx[i]) <= 0.0f) --k; h[k++] = idx[i]; }
        for (int i = n - 2, t = k + 1; i >= 0; --i) { while (k >= t && cross(h[k - 2], h[k - 1], idx[i]) <= 0.0f) --k; h[k++] = idx[i]; }
        --k;
        for (int i = 0; i < k && i < 8; ++i) { ha[i] = xa[h[i]]; hc[i] = xc[h[i]]; }
        return std::min(k, 8);
    }

    static void Raster(const Shape& s, const Quad& q, int orient, uint8_t* hi) {
        float px[4], py[4];
        ACToPixel(q, orient, s.minA, s.minC, px[0], py[0]);
        ACToPixel(q, orient, s.maxA, s.minC, px[1], py[1]);
        ACToPixel(q, orient, s.minA, s.maxC, px[2], py[2]);
        ACToPixel(q, orient, s.maxA, s.maxC, px[3], py[3]);
        float x0 = px[0], x1 = px[0], y0 = py[0], y1 = py[0];
        for (int i = 1; i < 4; ++i) { x0 = std::min(x0, px[i]); x1 = std::max(x1, px[i]); y0 = std::min(y0, py[i]); y1 = std::max(y1, py[i]); }
        const int ix0 = std::max(0, static_cast<int>(std::floor(x0 - 0.5f)));
        const int ix1 = std::min(kInternal - 1, static_cast<int>(std::ceil(x1 + 0.5f)));
        const int iy0 = std::max(0, static_cast<int>(std::floor(y0 - 0.5f)));
        const int iy1 = std::min(kInternal - 1, static_cast<int>(std::ceil(y1 + 0.5f)));
        for (int y = iy0; y <= iy1; ++y)
            for (int x = ix0; x <= ix1; ++x) {
                float a, c; PixelToAC(q, orient, x + 0.5f, y + 0.5f, a, c);
                bool in = true;
                if (s.kind == 1) {
                    const float ea = (a - s.a[0]) / s.a[1], ec = (c - s.c[0]) / s.c[1];
                    in = ea * ea + ec * ec <= 1.0f;
                } else {
                    for (int e = 0; e < s.n && in; ++e) {
                        const int f = (e + 1) % s.n;
                        const float cr = (s.a[f] - s.a[e]) * (c - s.c[e]) - (s.c[f] - s.c[e]) * (a - s.a[e]);
                        if (cr < 0.0f) in = false;
                    }
                }
                if (in) hi[y * kInternal + x] = 1;
            }
    }

    Shape shapes_[kMaxShapes];
    int count_ = 0;
    bool overflow_ = false;
    float dx_ = 0, dy_ = 1, nx_ = 1, ny_ = 0, L_ = 1, zRef_ = 0;
};

// World offsets of the 8 corners of a local box, for an entity with axes right / forward / up.
inline void BoxCorners(const float mn[3], const float mx[3], const V3& right, const V3& fwd, const V3& up, V3 out[8]) {
    for (int i = 0; i < 8; ++i) {
        const float lx = (i & 1) ? mx[0] : mn[0];
        const float ly = (i & 2) ? mx[1] : mn[1];
        const float lz = (i & 4) ? mx[2] : mn[2];
        out[i] = { right.x * lx + fwd.x * ly + up.x * lz,
                   right.y * lx + fwd.y * ly + up.y * lz,
                   right.z * lx + fwd.z * ly + up.z * lz };
    }
}

} // namespace ShadowCore
