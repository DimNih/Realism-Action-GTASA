#pragma once
// ============================================================================
// vehicle_light_beam_core.h  (v2 - "splat" lighting)
//
// Pengganti pool putih bawaan GTA (vehiclelights128) yang tembus tembok.
//
// IDE
//   Dari lampu ditembakkan grid sinar (kCols x kRows). Setiap sinar yang kena
//   permukaan menjadi satu SPLAT: persegi kecil lembut yang menempel pada
//   permukaan itu (sejajar normal), dengan kernel Hann. Kernel Hann dengan
//   setengah-lebar = jarak antar sampel menjumlah ~1 (partition of unity),
//   sehingga splat-splat yang tumpang tindih membentuk pool yang mulus
//   seperti interpolasi bilinear - TANPA menyambung vertex lintas permukaan.
//
//   - Tiap splat dipotong tepat di batas permukaannya (dasar tembok, kerb,
//     tiang) bila tetangganya berada di permukaan lain -> tidak bocor ke
//     seberang objek.
//   - Baris sinar menukik = pool di jalan; baris mendatar/naik = cahaya
//     memanjat tembok, pagar, mobil lain.
//   - Kerucut volumetrik: pita tipis menghadap kamera sepanjang sinar sampai
//     penghalang pertama.
// ============================================================================

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace BeamCore {

struct V3 {
    float x = 0, y = 0, z = 0;
    V3() = default;
    V3(float X, float Y, float Z) : x(X), y(Y), z(Z) {}
    V3 operator+(const V3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    V3 operator-(const V3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    V3 operator*(float s) const { return {x * s, y * s, z * s}; }
};
inline float Dot(const V3& a, const V3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline V3 Cross(const V3& a, const V3& b) { return {a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x}; }
inline float Len(const V3& a) { return std::sqrt(Dot(a, a)); }
inline V3 Norm(const V3& a, const V3& fb = V3(0, 1, 0)) {
    const float l = Len(a);
    return l > 1e-8f ? a * (1.0f / l) : fb;
}
inline float Clamp01(float v) { return v < 0.f ? 0.f : (v > 1.f ? 1.f : v); }
inline float Clampf(float v, float a, float b) { return v < a ? a : (v > b ? b : v); }
inline float Smooth(float a, float b, float x) {
    const float t = Clamp01((x - a) / (b - a));
    return t * t * (3.0f - 2.0f * t);
}

constexpr int kCols = 17;
constexpr int kGroundRows = 10;
constexpr int kWallRows = 3;
constexpr int kRows = kGroundRows + kWallRows;
constexpr int kVerts = kCols * kRows;
constexpr float kGroundDist[kGroundRows] = {0.6f, 1.6f, 2.8f, 4.4f, 6.4f, 8.8f, 12.0f, 16.0f, 21.0f, 26.0f};
constexpr float kWallPitchDeg[kWallRows] = {3.0f, 9.0f, 17.0f};
constexpr float kWallWeight[kWallRows] = {0.90f, 0.60f, 0.28f};

struct Frame {
    V3 origin;
    V3 fwd, right, up;
};

struct BeamSpec {
    float lampHeight = 0.7f;
    float halfSep = 0.55f;
    float range = 26.0f;
    float yawMaxDeg = 46.0f;
    float tiltDeg = -1.5f;
};

struct Beam {
    bool built = false;
    V3 local[kVerts];
    V3 normal[kVerts];
    V3 t1[kVerts];
    V3 t2[kVerts];
    float ext[kVerts][4] = {};
    float target[kVerts] = {};
    float current[kVerts] = {};
    bool hit[kVerts] = {};
    bool placed[kVerts] = {};
    V3 shaftFrom[kCols];
    V3 shaftDir[kCols];
    float shaftLen[kCols] = {};
    float shaftU[kCols] = {};
    bool shaftOk[kCols] = {};
    float strength = 0.0f;
    float shaftFade = 0.0f;
};

inline V3 ToLocal(const V3& p, const V3& o, const V3& r, const V3& f, const V3& u) {
    const V3 d = p - o;
    return {Dot(d, r), Dot(d, f), Dot(d, u)};
}
inline V3 ToWorld(const V3& l, const V3& o, const V3& r, const V3& f, const V3& u) {
    return o + r * l.x + f * l.y + u * l.z;
}
inline V3 DirToLocal(const V3& d, const Frame& v) { return {Dot(d, v.right), Dot(d, v.fwd), Dot(d, v.up)}; }
inline V3 DirToWorld(const V3& l, const Frame& v) { return v.right * l.x + v.fwd * l.y + v.up * l.z; }
inline V3 PointToWorld(const V3& l, const Frame& v) { return v.origin + DirToWorld(l, v); }

constexpr float kDeg = 0.017453292519943295f;

inline float BeamProfile(float u, float v) {
    const float x = 2.0f * (u - 0.5f);
    const float core = std::exp(-(x * x) / (2.0f * 0.17f * 0.17f));
    const float mid  = std::exp(-(x * x) / (2.0f * 0.42f * 0.42f));
    const float wide = std::exp(-(x * x) / (2.0f * 0.80f * 0.80f));
    float lateral = 0.50f * core + 0.34f * mid + 0.16f * wide;
    lateral *= 1.0f - Smooth(0.72f, 1.0f, std::fabs(x));
    const float hot = std::exp(-((v - 0.14f) * (v - 0.14f)) / (2.0f * 0.11f * 0.11f));
    float along = (0.50f + 0.50f * hot) *
        (0.38f + 0.62f * (1.0f - Smooth(0.72f, 1.0f, v)));
    along *= Smooth(0.0f, 0.03f, v);
    const float grain = 1.0f + 0.04f * std::sin(u * 37.0f + v * 11.0f) * std::sin(v * 23.0f - u * 5.0f);
    return Clamp01(lateral * along * grain * 1.30f);
}

inline float SoftKernel(float u, float v) {
    auto hann = [](float t) {
        const float x = Clampf(2.0f * t - 1.0f, -1.0f, 1.0f);
        return 0.5f * (1.0f + std::cos(3.14159265f * x));
    };
    return hann(u) * hann(v);
}

template <class CastFn>
int BuildBeam(const BeamSpec& spec, const Frame& beam, const Frame& vehicle,
              float strength, CastFn&& cast, Beam& out) {
    int hits = 0;
    const float tilt = spec.tiltDeg * kDeg;
    const float stepRad = 2.0f * spec.yawMaxDeg * kDeg / (kCols - 1);
    float pitchRow[kRows], rowWeight[kRows];
    for (int r = 0; r < kRows; ++r) {
        if (r < kGroundRows) {
            pitchRow[r] = -std::atan2(spec.lampHeight, kGroundDist[r]);
            rowWeight[r] = 1.0f;
        } else {
            pitchRow[r] = kWallPitchDeg[r - kGroundRows] * kDeg + tilt * 0.5f;
            rowWeight[r] = kWallWeight[r - kGroundRows];
        }
    }
    float gapAng[kRows];
    for (int r = 0; r < kRows; ++r) {
        float sum = 0.0f;
        int count = 0;
        if (r > 0) { sum += std::fabs(pitchRow[r] - pitchRow[r - 1]); ++count; }
        if (r + 1 < kRows) { sum += std::fabs(pitchRow[r + 1] - pitchRow[r]); ++count; }
        gapAng[r] = sum / count;
    }

    static thread_local V3 worldPos[kVerts], worldNormal[kVerts], worldDir[kVerts];
    static thread_local float worldDist[kVerts];
    static thread_local bool worldHit[kVerts];

    for (int c = 0; c < kCols; ++c) {
        const float t = 2.0f * c / (kCols - 1) - 1.0f;
        const float yaw = t * spec.yawMaxDeg * kDeg;
        const V3 from = beam.origin + beam.right * (t * spec.halfSep);
        const V3 dirH = Norm(beam.fwd * std::cos(yaw) + beam.right * std::sin(yaw));
        const float u = (t + 1.0f) * 0.5f;
        for (int r = 0; r < kRows; ++r) {
            const int i = r * kCols + c;
            worldHit[i] = false;
            if (r < kGroundRows && kGroundDist[r] > spec.range) {
                out.hit[i] = false;
                out.target[i] = 0.0f;
                continue;
            }
            float length = r < kGroundRows
                ? (kGroundDist[r] / std::cos(pitchRow[r])) * 1.30f + 1.0f
                : spec.range;
            length = std::min(length, spec.range * 1.08f);
            const V3 dir = Norm(dirH * std::cos(pitchRow[r]) + beam.up * std::sin(pitchRow[r]));
            V3 point, normal;
            if (!cast(from, from + dir * length, point, normal)) {
                out.hit[i] = false;
                out.target[i] = 0.0f;
                continue;
            }
            normal = Norm(normal, V3(0, 0, 1));
            if (Dot(normal, dir) > 0.0f) normal = normal * -1.0f;
            worldHit[i] = true;
            worldPos[i] = point;
            worldNormal[i] = normal;
            worldDir[i] = dir;
            worldDist[i] = Len(point - from);
            ++hits;
        }
        const int wallIndex = kGroundRows * kCols + c;
        out.shaftOk[c] = true;
        out.shaftU[c] = u;
        out.shaftFrom[c] = ToLocal(from, vehicle.origin, vehicle.right, vehicle.fwd, vehicle.up);
        out.shaftDir[c] = DirToLocal(Norm(dirH * std::cos(tilt) + beam.up * std::sin(tilt)), vehicle);
        const float newLength = worldHit[wallIndex] ? std::max(0.5f, worldDist[wallIndex] - 0.1f) : spec.range;
        out.shaftLen[c] = out.built ? out.shaftLen[c] + (newLength - out.shaftLen[c]) * 0.5f : newLength;
    }

    for (int r = 0; r < kRows; ++r) {
        for (int c = 0; c < kCols; ++c) {
            const int i = r * kCols + c;
            if (!worldHit[i]) { out.hit[i] = false; out.target[i] = 0.0f; continue; }
            const V3 normal = worldNormal[i], dir = worldDir[i];
            const float distance = worldDist[i];
            const float facing = Clamp01(-Dot(normal, dir));
            const float facingSafe = std::max(facing, 0.02f);
            const float t = 2.0f * c / (kCols - 1) - 1.0f;
            const float u = (t + 1.0f) * 0.5f;
            const float v = Clamp01(distance / spec.range);
            const float halfAlong = Clampf(distance * gapAng[r] / facingSafe, 0.25f, 7.0f);
            const float halfAcross = Clampf(distance * std::cos(pitchRow[r]) * stepRad * 1.18f +
                2.0f * spec.halfSep / (kCols - 1), 0.30f, 6.5f);
            V3 tangent = dir - normal * Dot(dir, normal);
            if (Len(tangent) < 0.2f) tangent = beam.up - normal * Dot(beam.up, normal);
            if (Len(tangent) < 0.2f) tangent = beam.right - normal * Dot(beam.right, normal);
            tangent = Norm(tangent);
            const V3 bitangent = Norm(Cross(normal, tangent), beam.right);
            float ext[4] = {halfAlong, halfAlong, halfAcross, halfAcross};
            const int neighbors[4][2] = {{r + 1, c}, {r - 1, c}, {r, c - 1}, {r, c + 1}};
            const float reach = std::max(halfAlong, halfAcross);
            for (int side = 0; side < 4; ++side) {
                const int rr = neighbors[side][0], cc = neighbors[side][1];
                if (rr < 0 || rr >= kRows || cc < 0 || cc >= kCols) continue;
                const int j = rr * kCols + cc;
                if (!worldHit[j]) continue;
                const V3 delta = worldPos[j] - worldPos[i];
                const bool differentSurface = Dot(worldNormal[j], normal) < 0.80f ||
                    Len(delta) > 2.4f * reach + 0.5f;
                const float along = Dot(delta, tangent), across = Dot(delta, bitangent);
                if (!differentSurface) {
                    const float projection = std::fabs(side < 2 ? along : across);
                    ext[side] = std::min(ext[side], std::max(0.20f, projection));
                    continue;
                }
                const float limit = Clampf(0.5f * Len(delta), 0.12f, 99.0f);
                if (std::fabs(along) >= std::fabs(across)) {
                    if (along >= 0.0f) ext[0] = std::min(ext[0], limit);
                    else ext[1] = std::min(ext[1], limit);
                } else {
                    if (across >= 0.0f) ext[3] = std::min(ext[3], limit);
                    else ext[2] = std::min(ext[2], limit);
                }
            }
            const float facingGain = 0.60f + 0.40f * std::sqrt(facing);
            out.target[i] = Clamp01(strength * facingGain * rowWeight[r] * BeamProfile(u, v));
            const V3 local = ToLocal(worldPos[i], vehicle.origin, vehicle.right, vehicle.fwd, vehicle.up);
            if (out.built && out.hit[i] && Len(local - out.local[i]) < 0.9f)
                out.local[i] = out.local[i] * 0.4f + local * 0.6f;
            else out.local[i] = local;
            out.normal[i] = DirToLocal(normal, vehicle);
            out.t1[i] = DirToLocal(tangent, vehicle);
            out.t2[i] = DirToLocal(bitangent, vehicle);
            for (int side = 0; side < 4; ++side) out.ext[i][side] = ext[side];
            out.hit[i] = true;
            out.placed[i] = true;
        }
    }
    out.strength = strength;
    out.built = true;
    return hits;
}

inline void Smoothen(Beam& beam, float rate) {
    for (int i = 0; i < kVerts; ++i) beam.current[i] += (beam.target[i] - beam.current[i]) * rate;
    beam.shaftFade += ((beam.strength > 0.0f ? 1.0f : 0.0f) - beam.shaftFade) * rate;
}

struct Quad { V3 p[4]; float u[4], v[4]; };
inline void SplatQuad(const Beam& beam, int i, const Frame& vehicle, Quad& quad) {
    const V3 point = PointToWorld(beam.local[i], vehicle);
    const V3 tangent = DirToWorld(beam.t1[i], vehicle);
    const V3 bitangent = DirToWorld(beam.t2[i], vehicle);
    const float front = beam.ext[i][0], back = beam.ext[i][1];
    const float left = beam.ext[i][2], right = beam.ext[i][3];
    const float alongExtent = std::max(front, back), acrossExtent = std::max(left, right);
    const float along[4] = {-back, front, front, -back};
    const float across[4] = {-left, -left, right, right};
    for (int corner = 0; corner < 4; ++corner) {
        quad.p[corner] = point + tangent * along[corner] + bitangent * across[corner];
        quad.v[corner] = 0.5f + 0.5f * along[corner] / alongExtent;
        quad.u[corner] = 0.5f + 0.5f * across[corner] / acrossExtent;
    }
}

} // namespace BeamCore
