// Host-side check of the shadow silhouette math. Not part of the Android shared library.
//   g++ -std=c++17 -O2 test_complex_shadow_core.cpp -o test_shadow && ./test_shadow [dump_dir]
#include "complex_shadow_core.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>

using namespace ShadowCore;

static int g_fail = 0;
#define CHECK(cond, ...) do { if (!(cond)) { ++g_fail; std::printf("FAIL: " __VA_ARGS__); std::printf("\n"); } else { std::printf("ok:   " __VA_ARGS__); std::printf("\n"); } } while (0)

static const V3 kRight{1, 0, 0}, kFwd{0, 1, 0}, kUp{0, 0, 1};
static uint8_t g_mask[kMaskSize * kMaskSize];

static void AddBox(Builder& b, float x0, float y0, float z0, float x1, float y1, float z1) {
    const float mn[3] = {x0, y0, z0}, mx[3] = {x1, y1, z1};
    V3 c[8]; BoxCorners(mn, mx, kRight, kFwd, kUp, c);
    b.AddBox(c);
}
static float Coverage(const uint8_t* m) {
    float s = 0; for (int i = 0; i < kMaskSize * kMaskSize; ++i) s += m[i] / 255.0f;
    return s / (kMaskSize * kMaskSize);
}
static void Dump(const char* dir, const char* name, const uint8_t* m) {
    if (!dir) return;
    char path[512]; std::snprintf(path, sizeof(path), "%s/%s.pgm", dir, name);
    FILE* f = std::fopen(path, "wb"); if (!f) return;
    std::fprintf(f, "P5\n%d %d\n255\n", kMaskSize, kMaskSize);
    std::fwrite(m, 1, kMaskSize * kMaskSize, f); std::fclose(f);
}

int main(int argc, char** argv) {
    const char* dump = argc > 1 ? argv[1] : nullptr;
    static Builder b;
    const float dxs = 0.8f, dys = 0.6f;              // unit shadow direction
    const float L = 1.2f;
    Quad q;

    // 1) L-shaped building: must NOT fill its bounding rectangle.
    b.Begin(dxs, dys, L, 0.0f);
    AddBox(b, -10, -6, 0, 10, 6, 20);
    AddBox(b, -10, 6, 0, -2, 18, 12);
    CHECK(b.Finish(0, 70, 45, g_mask, q), "L building rasterised");
    const float covL = Coverage(g_mask);
    CHECK(covL > 0.15f && covL < 0.80f, "L building coverage %.2f is not a full rectangle", covL);
    Dump(dump, "building_L", g_mask);

    // Same footprint as ONE plain box for comparison: much fuller (the old 'kotak polos').
    b.Begin(dxs, dys, L, 0.0f);
    AddBox(b, -10, -6, 0, 10, 18, 20);
    b.Finish(0, 70, 45, g_mask, q);
    const float covBox = Coverage(g_mask);
    CHECK(covBox > covL, "plain box (%.2f) fills more than the L building (%.2f)", covBox, covL);
    Dump(dump, "building_box", g_mask);

    // 2) Thin lamp post: a thin streak, not a square.
    b.Begin(dxs, dys, L, 0.0f);
    AddBox(b, -0.12f, -0.12f, 0, 0.12f, 0.12f, 8);
    CHECK(b.Finish(0, 70, 45, g_mask, q), "pole rasterised");
    const float covPole = Coverage(g_mask);
    int widest = 0;
    for (int y = 0; y < kMaskSize; ++y) { int n = 0; for (int x = 0; x < kMaskSize; ++x) n += g_mask[y * kMaskSize + x] > 128; widest = std::max(widest, n); }
    CHECK(covPole > 0.01f && covPole < 0.40f, "pole coverage %.3f is a thin streak", covPole);
    CHECK(widest <= kMaskSize * 4 / 10, "pole is narrow: widest row covers %d of %d texels", widest, kMaskSize);
    CHECK(q.halfA > 3.0f * q.halfC, "pole quad is long (%.1f) and narrow (%.1f)", q.halfA, q.halfC);
    Dump(dump, "pole", g_mask);

    // 3) Tree: trunk + round canopy; the canopy blob must be at the far end of the shadow.
    b.Begin(dxs, dys, L, 0.0f);
    AddBox(b, -0.3f, -0.3f, 0, 0.3f, 0.3f, 5);
    { const V3 c{0, 0, 7.5f}; b.AddSphere(c, 3.0f); }
    CHECK(b.Finish(0, 70, 45, g_mask, q), "tree rasterised");
    // orient 0: far tip at the TOP of the texture -> the blob (heavy weight) sits in the top half.
    float top = 0, bot = 0;
    for (int y = 0; y < kMaskSize; ++y) for (int x = 0; x < kMaskSize; ++x) (y < kMaskSize / 2 ? top : bot) += g_mask[y * kMaskSize + x];
    CHECK(top > bot * 1.5f, "orient 0: canopy blob at the far end is at the top (top=%.0f bottom=%.0f)", top, bot);
    Dump(dump, "tree", g_mask);

    // Same caster, V mirrored: the blob moves to the bottom.
    b.Begin(dxs, dys, L, 0.0f);
    AddBox(b, -0.3f, -0.3f, 0, 0.3f, 0.3f, 5);
    { const V3 c{0, 0, 7.5f}; b.AddSphere(c, 3.0f); }
    b.Finish(2, 70, 45, g_mask, q);
    top = bot = 0;
    for (int y = 0; y < kMaskSize; ++y) for (int x = 0; x < kMaskSize; ++x) (y < kMaskSize / 2 ? top : bot) += g_mask[y * kMaskSize + x];
    CHECK(bot > top * 1.5f, "orient 2 (mirror V): blob flips to the bottom (top=%.0f bottom=%.0f)", top, bot);
    Dump(dump, "tree_flipV", g_mask);

    // 4) Axis mapping round trip for every orient value.
    Quad qq{3.0f, -2.0f, 10.0f, 4.0f};
    bool rt = true;
    for (int o = 0; o < 8; ++o)
        for (int i = 0; i < 20; ++i) {
            const float a = qq.aC + (i - 10) * 0.8f, c = qq.cC + (i - 10) * 0.3f;
            float px, py, a2, c2; ACToPixel(qq, o, a, c, px, py); PixelToAC(qq, o, px, py, a2, c2);
            if (std::fabs(a - a2) > 1e-3f || std::fabs(c - c2) > 1e-3f) rt = false;
        }
    CHECK(rt, "ACToPixel/PixelToAC round trip for all 8 orient values");

    // 5) Very tall building: the quad is clamped to the root of the shadow and never empty.
    b.Begin(dxs, dys, 4.5f, 0.0f);
    AddBox(b, -20, -20, 0, 20, 20, 110);
    CHECK(b.Finish(0, 70, 45, g_mask, q), "tall building rasterised");
    CHECK(q.halfA <= 70.01f && q.halfC <= 45.01f, "clamped quad %.1f x %.1f", q.halfA, q.halfC);
    CHECK(Coverage(g_mask) > 0.3f, "clamped shadow still has substance (%.2f)", Coverage(g_mask));
    Dump(dump, "tall_clamped", g_mask);

    // 6) Border of the mask is always empty (no hard cut when the tip is clamped).
    int border = 0;
    for (int i = 0; i < kMaskSize; ++i) border += g_mask[i] + g_mask[(kMaskSize - 1) * kMaskSize + i] + g_mask[i * kMaskSize] + g_mask[i * kMaskSize + kMaskSize - 1];
    CHECK(border < 4 * kMaskSize * 40, "border pixels are faded (sum=%d)", border);

    // 7) Mesh building made from triangles (a plus-shaped tower footprint extruded as roof+walls).
    b.Begin(dxs, dys, L, 0.0f);
    auto tri = [&](V3 p0, V3 p1, V3 p2) { b.AddTriangle(p0, p1, p2); };
    // two crossing slabs built from triangles (top faces only; projection of top plus walls would be larger)
    const float h = 15;
    tri({-12, -2, h}, {12, -2, h}, {12, 2, h}); tri({-12, -2, h}, {12, 2, h}, {-12, 2, h});
    tri({-2, -12, h}, {2, -12, h}, {2, 12, h}); tri({-2, -12, h}, {2, 12, h}, {-2, 12, h});
    tri({-12, -2, 0}, {12, -2, 0}, {12, 2, 0}); tri({-12, -2, 0}, {12, 2, 0}, {-12, 2, 0});
    tri({-2, -12, 0}, {2, -12, 0}, {2, 12, 0}); tri({-2, -12, 0}, {2, 12, 0}, {-2, 12, 0});
    CHECK(b.Finish(0, 70, 45, g_mask, q), "triangle-mesh building rasterised");
    Dump(dump, "plus_mesh", g_mask);

    // 8) Pedestrian: three stacked spheres -> a stretched, blob-chain streak.
    b.Begin(dxs, dys, 1.6f, 0.0f);
    { const V3 c0{0, 0, 0.4f}, c1{0, 0, 0.95f}, c2{0, 0, 1.5f}; b.AddSphere(c0, 0.3f); b.AddSphere(c1, 0.3f); b.AddSphere(c2, 0.22f); }
    CHECK(b.Finish(0, 70, 45, g_mask, q), "ped rasterised");
    CHECK(q.halfA > q.halfC * 1.5f, "ped quad is stretched along the shadow (%.2f x %.2f)", q.halfA, q.halfC);
    Dump(dump, "ped", g_mask);

    std::printf("\n%s (%d failure%s)\n", g_fail ? "SOME CHECKS FAILED" : "ALL CHECKS PASSED", g_fail, g_fail == 1 ? "" : "s");
    return g_fail ? 1 : 0;
}
