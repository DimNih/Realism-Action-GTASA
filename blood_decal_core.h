#pragma once
// ============================================================================
// blood_decal_core.h  -  logika murni (tanpa dependensi game) untuk bercak darah
// yang MENEMPEL di kulit player.
//
// Dua tingkat (tier):
//   Tier 1  "Skin-mesh decal"  : bercak dibuat dari segitiga mesh player itu
//           sendiri (seperti decal proyeksi). Tiap frame, vertex yang dipakai
//           di-skin ulang dengan matriks tulang LIVE -> bercak PASTI menempel di
//           kulit, tidak melayang, tidak tembus, ikut animasi (aim, jongkok, lari).
//   Tier 2  "Torso ellipse"    : cadangan bila data skin tidak bisa dibaca.
//           Badan dimodelkan sebagai tumpukan elips yang terikat ke kerangka
//           (pelvis, leher, bahu kiri/kanan), bukan ke matriks root ped.
//
// Semua struct di sini memakai float biasa supaya bisa dites di PC.
// ============================================================================

#include <cmath>
#include <cstdint>
#include <cstring>
#include <algorithm>

namespace BloodCore {

struct V3 {
    float x = 0, y = 0, z = 0;
    V3() = default;
    V3(float X, float Y, float Z) : x(X), y(Y), z(Z) {}
    V3 operator+(const V3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    V3 operator-(const V3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    V3 operator*(float s) const { return {x * s, y * s, z * s}; }
    V3& operator+=(const V3& o) { x += o.x; y += o.y; z += o.z; return *this; }
};
inline float Dot(const V3& a, const V3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline V3 Cross(const V3& a, const V3& b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline float Len(const V3& a) { return sqrtf(Dot(a, a)); }
inline V3 Norm(const V3& a, const V3& fallback = V3(0, 0, 1)) {
    const float l = Len(a);
    return l > 1e-8f ? a * (1.0f / l) : fallback;
}
inline float Clamp01(float v) { return v < 0.f ? 0.f : (v > 1.f ? 1.f : v); }

// Matriks gaya RenderWare (row-vector): p' = x*right + y*up + z*at + pos
struct M4 {
    V3 right{1, 0, 0}, up{0, 1, 0}, at{0, 0, 1}, pos{0, 0, 0};
};
inline V3 XformPoint(const M4& m, const V3& p) {
    return m.right * p.x + m.up * p.y + m.at * p.z + m.pos;
}
inline V3 XformDir(const M4& m, const V3& d) {
    return m.right * d.x + m.up * d.y + m.at * d.z;
}

// ----------------------------------------------------------------------------
// Skinning CPU (rumus RenderWare): v' = sum_i w_i * ( (v * skinToBone[idx_i]) * boneMatrix[idx_i] )
// ----------------------------------------------------------------------------
struct SkinView {
    const V3*   bindPos = nullptr;      // posisi vertex pose-bind (morphTarget[0].verts)
    const float* weights = nullptr;     // 4 float per vertex (RwMatrixWeights)
    const uint32_t* indices = nullptr;  // 4 byte tulang per vertex, dipak dalam uint32
    const M4*   skinToBone = nullptr;   // matriks inverse-bind, per tulang
    const M4*   boneMatrix = nullptr;   // matriks tulang LIVE (RpHAnim), per tulang
    int         numBones = 0;
    int         numVerts = 0;
};

inline V3 SkinVertex(const SkinView& s, int vi) {
    const float* w = s.weights + vi * 4;
    const uint32_t packed = s.indices[vi];
    V3 out(0, 0, 0);
    float wsum = 0.f;
    for (int k = 0; k < 4; ++k) {
        const float wk = w[k];
        if (wk <= 1e-5f) continue;
        const int b = static_cast<int>((packed >> (8 * k)) & 0xFFu);
        if (b >= s.numBones) continue;
        const V3 inBone = XformPoint(s.skinToBone[b], s.bindPos[vi]);
        out += XformPoint(s.boneMatrix[b], inBone) * wk;
        wsum += wk;
    }
    // bobot tidak ternormalisasi sempurna? normalkan supaya tidak menyusut ke origin
    if (wsum > 1e-5f && fabsf(wsum - 1.0f) > 1e-3f) out = out * (1.0f / wsum);
    return out;
}

inline int DominantBone(const SkinView& s, int vi) {
    const float* w = s.weights + vi * 4;
    const uint32_t packed = s.indices[vi];
    int best = 0; float bw = -1.f;
    for (int k = 0; k < 4; ++k) {
        if (w[k] > bw) { bw = w[k]; best = static_cast<int>((packed >> (8 * k)) & 0xFFu); }
    }
    return best;
}

// ----------------------------------------------------------------------------
// Ray / segitiga (Moller-Trumbore, dua sisi)
// ----------------------------------------------------------------------------
inline bool RayTriangle(const V3& o, const V3& d, const V3& a, const V3& b, const V3& c, float& t) {
    const V3 e1 = b - a, e2 = c - a;
    const V3 p = Cross(d, e2);
    const float det = Dot(e1, p);
    if (fabsf(det) < 1e-9f) return false;
    const float inv = 1.0f / det;
    const V3 s = o - a;
    const float u = Dot(s, p) * inv;
    if (u < 0.f || u > 1.f) return false;
    const V3 q = Cross(s, e1);
    const float v = Dot(d, q) * inv;
    if (v < 0.f || u + v > 1.f) return false;
    t = Dot(e2, q) * inv;
    return t >= 0.f;
}

// titik terdekat pada segitiga (Ericson, Real-Time Collision Detection)
inline V3 ClosestPointOnTriangle(const V3& p, const V3& a, const V3& b, const V3& c) {
    const V3 ab = b - a, ac = c - a, ap = p - a;
    const float d1 = Dot(ab, ap), d2 = Dot(ac, ap);
    if (d1 <= 0 && d2 <= 0) return a;
    const V3 bp = p - b;
    const float d3 = Dot(ab, bp), d4 = Dot(ac, bp);
    if (d3 >= 0 && d4 <= d3) return b;
    const float vc = d1 * d4 - d3 * d2;
    if (vc <= 0 && d1 >= 0 && d3 <= 0) return a + ab * (d1 / (d1 - d3));
    const V3 cp = p - c;
    const float d5 = Dot(ab, cp), d6 = Dot(ac, cp);
    if (d6 >= 0 && d5 <= d6) return c;
    const float vb = d5 * d2 - d1 * d6;
    if (vb <= 0 && d2 >= 0 && d6 <= 0) return a + ac * (d2 / (d2 - d6));
    const float va = d3 * d6 - d5 * d4;
    if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0)
        return b + (c - b) * ((d4 - d3) / ((d4 - d3) + (d5 - d6)));
    const float denom = 1.0f / (va + vb + vc);
    return a + ab * (vb * denom) + ac * (vc * denom);
}

// ----------------------------------------------------------------------------
// TIER 1 : decal dari segitiga mesh
// ----------------------------------------------------------------------------
constexpr int kDecalMaxTris  = 150;
constexpr int kDecalMaxVerts = 160;

struct MeshDecal {
    bool     valid = false;
    const void* geometryKey = nullptr;  // identitas geometry (cek ganti baju/model)
    int      geometryVerts = 0;
    int      triCount = 0;
    int      vertCount = 0;
    uint16_t vertRef[kDecalMaxVerts];            // indeks vertex di geometry
    uint16_t tri[kDecalMaxTris][3];              // indeks LOKAL (ke vertRef)
    bool     triFlip[kDecalMaxTris];             // balik normal agar menghadap keluar
    float    uv[kDecalMaxVerts][2];              // UV dipanggang saat kena tembak (-1..1)
    V3       axisHint;                           // arah "turun" (untuk menetes)
};

// Hasil raycast ke mesh yang sudah di-skin
struct SurfaceHit {
    bool ok = false;
    int  tri = -1;
    V3   point, normal;
};

// vertex yang boleh dipakai (mis. hanya tulang batang tubuh)
struct BoneFilter {
    const uint8_t* allowedBones = nullptr; // tabel [numBones] 1 = boleh
    int count = 0;
    bool Allows(int b) const { return !allowedBones || (b >= 0 && b < count && allowedBones[b]); }
};

// Cari titik di kulit untuk titik tembak. Urutan:
//  1) raycast dari luar (hit + out*reach) ke dalam sepanjang -out  -> perpotongan pertama
//  2) bila gagal: titik terdekat di segitiga mana pun dalam maxSnap
// `out` = arah keluar-badan (dari sumbu badan menuju titik kena).
inline SurfaceHit FindSkinSurface(const SkinView& s, const uint16_t* triIdx, int triCount,
                                  const V3* skinned, const BoneFilter& filter,
                                  const V3& hit, const V3& out, float reach, float maxSnap) {
    SurfaceHit r;
    const V3 o = hit + out * reach;
    const V3 d = out * -1.0f;
    float bestT = 1e9f;
    for (int t = 0; t < triCount; ++t) {
        const uint16_t i0 = triIdx[t * 3], i1 = triIdx[t * 3 + 1], i2 = triIdx[t * 3 + 2];
        if (!filter.Allows(DominantBone(s, i0)) && !filter.Allows(DominantBone(s, i1)) &&
            !filter.Allows(DominantBone(s, i2))) continue;
        float tt;
        if (RayTriangle(o, d, skinned[i0], skinned[i1], skinned[i2], tt) && tt < bestT) {
            bestT = tt; r.tri = t;
        }
    }
    if (r.tri >= 0) {
        r.ok = true;
        r.point = o + d * bestT;
    } else {
        float bestD = maxSnap;
        for (int t = 0; t < triCount; ++t) {
            const uint16_t i0 = triIdx[t * 3], i1 = triIdx[t * 3 + 1], i2 = triIdx[t * 3 + 2];
            if (!filter.Allows(DominantBone(s, i0)) && !filter.Allows(DominantBone(s, i1)) &&
                !filter.Allows(DominantBone(s, i2))) continue;
            const V3 cp = ClosestPointOnTriangle(hit, skinned[i0], skinned[i1], skinned[i2]);
            const float dist = Len(cp - hit);
            if (dist < bestD) { bestD = dist; r.tri = t; r.point = cp; r.ok = true; }
        }
    }
    if (r.ok) {
        const uint16_t i0 = triIdx[r.tri * 3], i1 = triIdx[r.tri * 3 + 1], i2 = triIdx[r.tri * 3 + 2];
        V3 n = Norm(Cross(skinned[i1] - skinned[i0], skinned[i2] - skinned[i0]));
        if (Dot(n, out) < 0) n = n * -1.0f;   // hadapkan keluar
        r.normal = n;
    }
    return r;
}

// Kumpulkan segitiga di sekitar titik kena -> MeshDecal.
//  center, normal : titik & normal permukaan (dunia, pose saat kena)
//  tangent        : arah "aliran" di permukaan (UV +v)
//  radius         : radius maksimum bercak (UV -1..1 memetakan -radius..+radius)
//  axisPoint      : titik pada sumbu badan terdekat (untuk menentukan sisi luar segitiga)
inline bool BuildMeshDecal(MeshDecal& out, const SkinView& s, const uint16_t* triIdx, int triCount,
                           const V3* skinned, const BoneFilter& filter,
                           const V3& center, const V3& normal, const V3& tangent,
                           const V3& axisPoint, float radius, const void* geometryKey) {
    out = MeshDecal();
    out.geometryKey = geometryKey;
    out.geometryVerts = s.numVerts;
    out.axisHint = V3(0, 0, -1);

    const V3 T = Norm(tangent - normal * Dot(tangent, normal), V3(0, 0, -1));
    const V3 B = Cross(normal, T);

    auto findOrAdd = [&](uint16_t gv) -> int {
        for (int i = 0; i < out.vertCount; ++i) if (out.vertRef[i] == gv) return i;
        if (out.vertCount >= kDecalMaxVerts) return -1;
        const int li = out.vertCount++;
        out.vertRef[li] = gv;
        const V3 rel = skinned[gv] - center;
        out.uv[li][0] = Dot(rel, T) / radius;
        out.uv[li][1] = Dot(rel, B) / radius;
        return li;
    };

    const float reachDist = radius * 1.10f;
    for (int t = 0; t < triCount && out.triCount < kDecalMaxTris; ++t) {
        const uint16_t i0 = triIdx[t * 3], i1 = triIdx[t * 3 + 1], i2 = triIdx[t * 3 + 2];
        // semua vertex harus dari tulang yang diizinkan (cegah lengan/kepala ikut terbungkus)
        if (!filter.Allows(DominantBone(s, i0)) || !filter.Allows(DominantBone(s, i1)) ||
            !filter.Allows(DominantBone(s, i2))) continue;
        const V3 a = skinned[i0], b = skinned[i1], c = skinned[i2];
        V3 n = Cross(b - a, c - a);
        const float nl = Len(n);
        if (nl < 1e-9f) continue;
        n = n * (1.0f / nl);
        const V3 cen = (a + b + c) * (1.0f / 3.0f);
        // sisi luar menurut sumbu badan, bukan menurut winding (aman utk mesh winding terbalik)
        const bool flip = Dot(n, cen - axisPoint) < 0.0f;
        const V3 no = flip ? n * -1.0f : n;
        if (Dot(no, normal) < 0.25f) continue;                 // hanya sisi yang searah
        // segitiga besar tetap ikut selama bagian mana pun-nya dekat pusat bercak
        if (Len(ClosestPointOnTriangle(center, a, b, c) - center) > reachDist) continue;

        const int l0 = findOrAdd(i0), l1 = findOrAdd(i1), l2 = findOrAdd(i2);
        if (l0 < 0 || l1 < 0 || l2 < 0) break;
        out.tri[out.triCount][0] = static_cast<uint16_t>(l0);
        out.tri[out.triCount][1] = static_cast<uint16_t>(l1);
        out.tri[out.triCount][2] = static_cast<uint16_t>(l2);
        out.triFlip[out.triCount] = flip;
        ++out.triCount;
    }
    out.valid = out.triCount >= 1;
    return out.valid;
}

// Per frame: hitung posisi + normal vertex untuk MeshDecal dari pose LIVE.
// Hasil: pos[i] sudah digeser `lift` meter sepanjang normal (anti z-fight).
inline bool EvalMeshDecal(const MeshDecal& d, const SkinView& s, float lift, V3* pos, V3* nrm) {
    if (!d.valid || s.numVerts != d.geometryVerts) return false;
    V3 acc[kDecalMaxVerts];
    V3 base[kDecalMaxVerts];
    for (int i = 0; i < d.vertCount; ++i) {
        if (d.vertRef[i] >= s.numVerts) return false;
        base[i] = SkinVertex(s, d.vertRef[i]);
        acc[i] = V3(0, 0, 0);
    }
    for (int t = 0; t < d.triCount; ++t) {
        const int a = d.tri[t][0], b = d.tri[t][1], c = d.tri[t][2];
        V3 n = Cross(base[b] - base[a], base[c] - base[a]);   // panjang = 2*luas -> pembobotan luas
        if (d.triFlip[t]) n = n * -1.0f;
        acc[a] += n; acc[b] += n; acc[c] += n;
    }
    for (int i = 0; i < d.vertCount; ++i) {
        nrm[i] = Norm(acc[i], V3(0, 0, 1));
        pos[i] = base[i] + nrm[i] * lift;
    }
    return true;
}

// ----------------------------------------------------------------------------
// TIER 2 : sumbu tulang belakang (poligon tulang) + penampang elips
// ----------------------------------------------------------------------------
constexpr int kMaxSpinePts = 4;

struct TorsoFrame {
    V3    P[kMaxSpinePts];      // titik sumbu: pelvis, spine1, torso atas, leher (dunia, pose LIVE)
    V3    Zj[kMaxSpinePts];     // arah sumbu di tiap sendi (rata-rata dua segmen)
    float cum[kMaxSpinePts];    // panjang kumulatif sepanjang poligon
    int   n = 0;
    float length = 0;
    V3    lateral;              // arah kiri->kanan (dari bahu), belum ortogonal
    bool  valid = false;
};

inline TorsoFrame BuildTorsoFrame(const V3* pts, int n, const V3& rShoulder, const V3& lShoulder) {
    TorsoFrame f;
    if (n < 2 || n > kMaxSpinePts) return f;
    f.n = n;
    f.cum[0] = 0;
    for (int i = 0; i < n; ++i) f.P[i] = pts[i];
    for (int i = 1; i < n; ++i) f.cum[i] = f.cum[i - 1] + Len(f.P[i] - f.P[i - 1]);
    f.length = f.cum[n - 1];
    if (f.length < 0.2f || f.length > 1.3f) return f;
    V3 seg[kMaxSpinePts];
    for (int i = 0; i < n - 1; ++i) seg[i] = Norm(f.P[i + 1] - f.P[i]);
    f.Zj[0] = seg[0];
    f.Zj[n - 1] = seg[n - 2];
    for (int i = 1; i < n - 1; ++i) f.Zj[i] = Norm(seg[i - 1] + seg[i]);
    f.lateral = rShoulder - lShoulder;
    if (Len(f.lateral) < 0.1f) return f;
    f.lateral = Norm(f.lateral);
    f.valid = true;
    return f;
}

struct AxisSample { V3 point, X, Y, Z; };

// frame lokal pada tinggi s (0..1) sepanjang poligon
inline AxisSample AxisAt(const TorsoFrame& f, float s) {
    const float S = Clamp01(s) * f.length;
    int i = 0;
    while (i < f.n - 2 && S > f.cum[i + 1]) ++i;
    const float segLen = f.cum[i + 1] - f.cum[i];
    const float t = segLen > 1e-6f ? Clamp01((S - f.cum[i]) / segLen) : 0.f;
    AxisSample a;
    a.point = f.P[i] + (f.P[i + 1] - f.P[i]) * t;
    a.Z = Norm(f.Zj[i] * (1.f - t) + f.Zj[i + 1] * t);
    V3 x = f.lateral - a.Z * Dot(f.lateral, a.Z);
    a.X = Norm(x, V3(1, 0, 0));
    a.Y = Norm(Cross(a.Z, a.X));     // Y = Z x X  (X x Y = Z)
    return a;
}

// Profil penampang badan (meter) pada tinggi s = 0..1 (pelvis -> leher).
// NILAI AWAL UNTUK CJ; kalibrasi dengan log "Tier2 calib" (lihat blood_stain_system.cpp).
struct TorsoSection { float s, a, bFront, bBack, cx; };
constexpr TorsoSection kTorsoProfile[] = {
    { 0.00f, 0.185f, 0.115f, 0.105f, 0.0f },   // pinggul
    { 0.30f, 0.160f, 0.105f, 0.095f, 0.0f },   // pinggang
    { 0.60f, 0.195f, 0.125f, 0.100f, 0.0f },   // dada
    { 0.85f, 0.205f, 0.115f, 0.100f, 0.0f },   // dada atas / garis bahu
    { 1.00f, 0.150f, 0.095f, 0.090f, 0.0f },   // pangkal leher
};
constexpr int kTorsoProfileCount = sizeof(kTorsoProfile) / sizeof(kTorsoProfile[0]);

inline TorsoSection SampleProfile(float s, float shrink) {
    s = Clamp01(s);
    int i = 0;
    while (i < kTorsoProfileCount - 2 && s > kTorsoProfile[i + 1].s) ++i;
    const TorsoSection& p = kTorsoProfile[i];
    const TorsoSection& q = kTorsoProfile[i + 1];
    const float t = Clamp01((s - p.s) / (q.s - p.s));
    TorsoSection r;
    r.s = s;
    r.a      = (p.a      + (q.a      - p.a)      * t) * shrink;
    r.bFront = (p.bFront + (q.bFront - p.bFront) * t) * shrink;
    r.bBack  = (p.bBack  + (q.bBack  - p.bBack)  * t) * shrink;
    r.cx     = p.cx + (q.cx - p.cx) * t;
    return r;
}

// titik di permukaan model: (s, theta). theta=0 kanan, 90 depan, 180 kiri, 270 belakang
inline void TorsoSurface(const TorsoFrame& f, float s, float theta, float shrink, V3& pos, V3& normal) {
    const AxisSample ax = AxisAt(f, s);
    const TorsoSection sec = SampleProfile(s, shrink);
    const float c = cosf(theta), sn = sinf(theta);
    const float b = sn >= 0.f ? sec.bFront : sec.bBack;
    pos = ax.point + ax.X * (sec.a * c + sec.cx) + ax.Y * (b * sn);
    normal = Norm(ax.X * (c / sec.a) + ax.Y * (sn / b));
}

// titik dunia -> (s, theta)
inline void TorsoParams(const TorsoFrame& f, const V3& p, float shrink, float& s, float& theta) {
    // proyeksi ke poligon: segmen terdekat
    float bestD = 1e9f, bestS = 0.f;
    for (int i = 0; i < f.n - 1; ++i) {
        const V3 ab = f.P[i + 1] - f.P[i];
        const float l2 = Dot(ab, ab);
        const float t = l2 > 1e-8f ? Clamp01(Dot(p - f.P[i], ab) / l2) : 0.f;
        const float d = Len(p - (f.P[i] + ab * t));
        if (d < bestD) { bestD = d; bestS = (f.cum[i] + t * (f.cum[i + 1] - f.cum[i])) / f.length; }
    }
    s = bestS;
    const AxisSample ax = AxisAt(f, s);
    const TorsoSection sec = SampleProfile(s, shrink);
    const V3 rel = p - ax.point;
    const float x = Dot(rel, ax.X) - sec.cx, y = Dot(rel, ax.Y);
    const float b = y >= 0.f ? sec.bFront : sec.bBack;
    theta = atan2f(y / b, x / sec.a);
}

// panjang busur per radian pada theta (meter -> delta theta)
inline float TorsoArcScale(float s, float theta, float shrink) {
    const TorsoSection sec = SampleProfile(s, shrink);
    const float c = cosf(theta), sn = sinf(theta);
    const float b = sn >= 0.f ? sec.bFront : sec.bBack;
    return sqrtf(sec.a * sec.a * sn * sn + b * b * c * c);
}

} // namespace BloodCore
