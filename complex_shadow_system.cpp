#include "complex_shadow_system.h"
#include "complex_shadow_core.h"
#include "vehicle_light_gl.h"

#include <mod/amlmod.h>
#include <mod/logger.h>

#include "screen_shadow_core.h"

#include <aml-psdk/game_sa/Events.h>
#include <aml-psdk/game_sa/base/Timer.h>
#include <aml-psdk/game_sa/base/ColModel.h>
#include <aml-psdk/game_sa/base/Lists.h>
#include <aml-psdk/game_sa/engine/Game.h>
#include <aml-psdk/game_sa/entity/Camera.h>
#include <aml-psdk/game_sa/entity/Entity.h>
#include <aml-psdk/game_sa/engine/Shadows.h>
#include <aml-psdk/game_sa/other/Clock.h>
#include <aml-psdk/game_sa/other/Pools.h>
#include <aml-psdk/game_sa/other/Weather.h>
#include <aml-psdk/renderware/RwImage.h>
#include <aml-psdk/renderware/RwRaster.h>
#include <aml-psdk/renderware/RwRender.h>
#include <aml-psdk/renderware/RwTexture.h>
#include <aml-psdk/renderware/RpAtomic.h>
#include <aml-psdk/renderware/RpClump.h>
#include <aml-psdk/renderware/RpGeometry.h>
#include <aml-psdk/renderware/RwFrame.h>
#include <cstddef>

#include <algorithm>
#include <vector>
#include "screen_shadow_gl.h"
#include <cmath>
#include <cstdint>
#include <cstdarg>
#include <cstring>
#include <cstdio>
#include <ctime>

#include <android/log.h>

// COMPLEX SHADOW
// Realtime sun shadows for every building / map object whose outline comes from the model's own
// DFF (the RpGeometry triangles of each rendered atomic), not from a hand-made box or mesh.
//
//  * every triangle of the DFF is projected onto the ground along the sun direction and rasterised
//    into an 80x80 mask (complex_shadow_core.h) -> the silhouette of the real model, including
//    L-shaped buildings, poles, signs and the leaf-cards of trees;
//  * the mask is the texture of the engine's own CShadows quad, so the game still drapes it over the
//    real ground polygons (same renderer as ped/vehicle shadows);
//  * models whose geometry cannot be read on the CPU fall back to the collision model;
//  * light by design: nearby casters are selected from building/object/dummy pools, masks are cached
//    per caster and rebuilt only when the sun or the caster has really moved, at most one rebuild per frame.
namespace ComplexShadowSystem {
namespace {
    using namespace ShadowCore;

    constexpr float kDeg = 0.01745329252f;
    constexpr float kPi = 3.14159265359f;
    constexpr float kSunrise = 6.0f;
    constexpr float kSunset = 20.0f;
    constexpr float kMaxSunElevation = 68.0f * kDeg;
    constexpr float kMinShadowElevation = 11.0f * kDeg;
    constexpr float kMaxLengthFactor = 4.5f;
    constexpr float kBaseBrightness = 120.0f;
    // Keep the full projected footprint for large buildings. The old limits
    // cropped the far end of long models and made the shadow look detached.
    constexpr float kMaxHalfA = 110.0f, kMaxHalfC = 75.0f;

    constexpr int kMaxTracks = 24;          // live + fading
    constexpr int kWanted = 18;             // nearby buildings, props and LOD dummies
    constexpr int kStoredLimit = 30;        // leave headroom for GTA's own decals/effects
    constexpr int kScanInterval = 20;       // detect newly visible casters sooner without per-frame scans
    constexpr int kRegenPerFrame = 3;
    constexpr int kTriBudgetPerFrame = 3000;
    constexpr int kMaxTris = 2800;          // collision/shadow-mesh triangles per caster
    constexpr int kMaxDffTris = 5000;       // DFF triangles per caster
    constexpr int kMaxDffVerts = 12000;
    constexpr int kMaxDffAtomics = 64;
    constexpr int kMaxBoxes = 160;
    constexpr int kMaxSpheres = 64;
    constexpr float kRegenSunCos = 0.99905f;    // ~2.5 degrees
    constexpr float kRegenAxisCos = 0.9976f;    // ~4 degrees

    enum Kind : uint8_t { kBuilding = 0, kObject, kVehicle, kPed, kDummy };

    struct Sun {
        float dirX = 0.0f, dirY = 1.0f, lengthFactor = 1.0f, strength = 0.0f, elevation = 0.0f;
    };

    struct Info { float height, extent, importance; };

    struct Track {
        bool used = false, selected = false, hasMask = false, triOk = false, triChecked = false;
        uint8_t dffState = 0;                // 0 unknown, 1 DFF geometry usable, 2 not readable -> collision fallback
        Kind kind = kBuilding;
        int index = 0; uint8_t id = 0; uint16_t model = 0;
        int slot = -1;
        float alpha = 0.0f;
        float importance = 1.0f;
        // last generated state
        float genDirX = 0, genDirY = 0, genL = 0; int genOrient = 0;
        float genAxes[9] = {};
        Quad quad; float zRef = 0.0f;
        CVector lastOrigin;
        float lastDist = 0.0f;
    };

    struct Slot { RwTexture* tex = nullptr; RwRaster* raster = nullptr; bool used = false; };

    Track g_tracks[kMaxTracks];
    Slot g_slots[kMaxTracks];
    RwImage* g_image = nullptr;
    bool g_imageFailed = false;
    Builder g_builder;                       // large (static) shape buffer
    uint8_t g_mask[kMaskSize * kMaskSize];

    using RenderStoredShadowsFn = void (*)(bool);
    RenderStoredShadowsFn g_originalRender = nullptr;
    RenderStoredShadowsFn g_originalStaticRender = nullptr;
    bool g_hookInstalled = false;
    bool g_staticHookInstalled = false;
    bool g_inStoredHook = false;
    bool g_inStaticHook = false;
    uint32_t g_lastFrame = 0xFFFFFFFFu, g_lastMs = 0, g_lastSummary = 0;
    int g_scanCountdown = 0, g_regenThisFrame = 0, g_trisThisFrame = 0;
    int g_lastStored = 0, g_lastCands = 0, g_lastOrient = -1;
    float g_lastStrength = 0.0f;
    Sun g_sun;
    bool g_sunOn = false;
    FILE* g_log = nullptr;
    long long g_logStartMs = 0;
    uint32_t g_renderCalls = 0, g_staticCalls = 0, g_staticSkipped = 0;
    uint32_t g_regenFails = 0, g_nativeSuppressed = 0;
    int g_logBudget = 16;

    long long NowMs() {
        timespec ts{};
        clock_gettime(CLOCK_MONOTONIC, &ts);
        return static_cast<long long>(ts.tv_sec) * 1000LL + ts.tv_nsec / 1000000LL;
    }
    void ShadowLog(const char* fmt, ...) {
        char buffer[900];
        va_list args;
        va_start(args, fmt);
        vsnprintf(buffer, sizeof(buffer), fmt, args);
        va_end(args);
        __android_log_write(ANDROID_LOG_INFO, "ComplexShadow", buffer);
        if (g_log) {
            std::fprintf(g_log, "[%8lld ms] %s\n", NowMs() - g_logStartMs, buffer);
            std::fflush(g_log);
        }
    }
    void OpenShadowLog() {
        char path[600];
        const char* base = aml ? aml->GetAndroidDataPath() : nullptr;
        if (base && base[0]) std::snprintf(path, sizeof(path), "%s/ComplexShadow.log", base);
        else std::snprintf(path, sizeof(path), "/storage/emulated/0/Android/data/com.rockstargames.gtasa/files/ComplexShadow.log");
        g_log = std::fopen(path, "w");
        g_logStartMs = NowMs();
        if (g_log) ShadowLog("initialize: path=%s", path);
        else __android_log_print(ANDROID_LOG_ERROR, "ComplexShadow", "cannot open log: %s", path);
    }

    // The engine's own blob shadows (vehicle, ped, pole) would double the real ones: skip them while the
    // sun-shadow pass runs in daylight. Night and any failure fall straight back to them.
    using StoreVehicleFn = void (*)(CVehicle*, VEH_SHD_TYPE);
    using StorePedFn = void (*)(CEntity*, float, float, float, float, float, float);
    using StorePoleFn = void (*)(CEntity*, float, float, float, float, float, uint32_t);
    StoreVehicleFn g_origVehicle = nullptr;
    StorePedFn g_origPed = nullptr;
    StorePoleFn g_origPole = nullptr;
    bool ComplexActive() {
        const VehicleLightGL::Settings& s = VehicleLightGL::GetSettings();
        return g_sunOn && s.shadowBuilding && g_lastStrength > 0.25f;
    }
    bool ShaderShadowRequested() {
        // Shader-only mode deliberately suppresses native shadows even during
        // a failed/initialising shader frame. This prevents a native shadow
        // from appearing for a few frames and then being multiplied by GLSL.
        return VehicleLightGL::SunShadowRequested();
    }
    void H_StoreShadowForVehicle(CVehicle* v, VEH_SHD_TYPE t) {
        // Building/object shadows must not make cars shadowless. Suppress the
        // native vehicle decal only when vehicles are actively covered by shader/complex shadows.
        const VehicleLightGL::Settings& s = VehicleLightGL::GetSettings();
        const bool suppress = (ShaderShadowRequested() && VehicleLightGL::IsSunShadowActive()) ||
                              (ComplexActive() && s.shadowVehicles);
        if (suppress) ++g_nativeSuppressed;
        if (!suppress && g_origVehicle) g_origVehicle(v, t);
    }
    void H_StoreShadowForPedObject(CEntity* e, float a, float b, float c, float d, float f, float g) {
        const VehicleLightGL::Settings& s = VehicleLightGL::GetSettings();
        const bool suppress = (ShaderShadowRequested() && VehicleLightGL::IsSunShadowActive()) ||
                              (ComplexActive() && s.shadowPeds);
        if (suppress) ++g_nativeSuppressed;
        if (!suppress && g_origPed) g_origPed(e, a, b, c, d, f, g);
    }
    void H_StoreShadowForPole(CEntity* e, float a, float b, float c, float d, float f, uint32_t n) {
        // Pole decals overlap the object caster mask and were a common source
        // of the large detached/double shadow around street lamps.
        const bool suppress = ShaderShadowRequested() || ComplexActive();
        if (suppress) ++g_nativeSuppressed;
        if (!suppress && g_origPole) g_origPole(e, a, b, c, d, f, n);
    }

    float Smooth(float e0, float e1, float x) { return Smooth01((x - e0) / (e1 - e0)); }
    bool Finite3(float a, float b, float c) { return std::isfinite(a + b + c); }

    bool ComputeSun(Sun& sun) {
        const float t = static_cast<float>(CClock::ms_nGameClockHours) + static_cast<float>(CClock::ms_nGameClockMinutes) / 60.0f +
                        static_cast<float>(std::min<int>(CClock::ms_nGameClockSeconds, 59)) / 3600.0f;
        const float u = (t - kSunrise) / (kSunset - kSunrise);
        if (!(u > 0.0f && u < 1.0f)) return false;
        const float elevation = kMaxSunElevation * std::sin(kPi * u);
        const float phi = kPi * u;                    // 0 = east, pi/2 = south, pi = west
        sun.dirX = -std::cos(phi);
        sun.dirY = std::sin(phi);
        sun.lengthFactor = std::min(1.0f / std::tan(std::max(elevation, kMinShadowElevation)), kMaxLengthFactor);
        sun.elevation = std::atan(1.0f / sun.lengthFactor);       // same clamp as the shadow length
        float s = Smooth(0.0f, 0.36f, std::sin(elevation));
        s *= 1.0f - 0.85f * Clamp(CWeather::Rain, 0.0f, 1.0f);
        s *= 1.0f - 0.60f * Clamp(CWeather::UnderWaterness, 0.0f, 1.0f);
        sun.strength = s;
        return s > 0.02f;
    }

    // ---------------------------------------------------------------- pools / entities
    template <typename PoolT>
    CEntity* PoolAt(PoolT* pool, int index, uint8_t id) {
        if (!pool || !pool->m_pObjects || !pool->m_byteMap || index < 0 || index >= pool->m_nSize) return nullptr;
        if (pool->m_byteMap[index].bEmpty || pool->m_byteMap[index].nId != id) return nullptr;
        return reinterpret_cast<CEntity*>(&pool->m_pObjects[index]);
    }
    CEntity* Resolve(const Track& t) {
        switch (t.kind) {
            case kBuilding: return PoolAt(CPools::ms_pBuildingPool, t.index, t.id);
            case kObject:   return PoolAt(CPools::ms_pObjectPool, t.index, t.id);
            case kVehicle:  return PoolAt(CPools::ms_pVehiclePool, t.index, t.id);
            case kPed:      return PoolAt(CPools::ms_pPedPool, t.index, t.id);
            case kDummy:    return PoolAt(CPools::ms_pDummyPool, t.index, t.id);
            default:        return nullptr;
        }
    }

    bool ColBounds(CEntity* e, const CColModel* col, float mn[3], float mx[3]) {
        (void)e;
        mn[0] = col->m_boundBox.m_vecMin.x; mn[1] = col->m_boundBox.m_vecMin.y; mn[2] = col->m_boundBox.m_vecMin.z;
        mx[0] = col->m_boundBox.m_vecMax.x; mx[1] = col->m_boundBox.m_vecMax.y; mx[2] = col->m_boundBox.m_vecMax.z;
        for (int i = 0; i < 3; ++i) if (!std::isfinite(mn[i]) || !std::isfinite(mx[i]) || mx[i] < mn[i] || std::fabs(mn[i]) > 2000.0f || std::fabs(mx[i]) > 2000.0f) return false;
        return true;
    }

    bool Classify(CEntity* e, Kind kind, Info& info) {
        if (!e || !e->bIsVisible || e->bTunnel) return false;
        const bool mapEntity = kind == kBuilding || kind == kObject || kind == kDummy;
        if (mapEntity && (!e->bUsesCollision || e->m_nNumLodChildren > 0)) return false;   // LOD parents
        CColModel* col = e->GetColModel();
        if (!col) return false;
        float mn[3], mx[3];
        if (!ColBounds(e, col, mn, mx)) return false;
        const float dx = mx[0] - mn[0], dy = mx[1] - mn[1], dz = mx[2] - mn[2];
        const float ext = std::max(dx, dy);
        float minH = 3.0f;
        switch (kind) {
            case kBuilding: minH = 2.5f; break;
            case kObject:   minH = 0.8f; break;
            case kDummy:    minH = 1.0f; break;
            default:        minH = 0.5f; break;
        }
        if (!(dz >= minH) || dz > 130.0f || !(ext > 0.05f) || ext > 95.0f) return false;
        info.height = dz; info.extent = ext;
        info.importance = Clamp(0.35f + dz / 12.0f, 0.35f, 2.0f);
        if (kind == kVehicle) info.importance = 1.4f;
        else if (kind == kPed) info.importance = 0.9f;
        return true;
    }

    // ---------------------------------------------------------------- geometry -> builder
    struct Axes { V3 r, f, u; };
    V3 Rot(const Axes& a, float x, float y, float z) {
        return { a.r.x * x + a.f.x * y + a.u.x * z, a.r.y * x + a.f.y * y + a.u.y * z, a.r.z * x + a.f.z * y + a.u.z * z };
    }

    struct TriangleSource {
        const CVector* vertices = nullptr;
        int vertexCount = 0;
        const CColTriangle* triangles = nullptr;
        int triangleCount = 0;
    };

    TriangleSource GetTriangleSource(const CCollisionData* d) {
        if (!d) return {};
        // GTA stores a reduced shadow mesh for many map models. Prefer it: it
        // follows the authored silhouette and avoids projecting every collision
        // volume, which was both noisy and expensive.
        if (d->m_pShadowVertices && d->m_pShadowTriangles && d->m_nNumShadowVertices > 2 &&
            d->m_nNumShadowTriangles > 0 && d->m_nNumShadowTriangles <= kMaxTris)
            return {d->m_pShadowVertices, static_cast<int>(d->m_nNumShadowVertices),
                    d->m_pShadowTriangles, static_cast<int>(d->m_nNumShadowTriangles)};
        if (d->m_pVertices && d->m_pTriangles && d->m_nNumTriangles > 0 && d->m_nNumTriangles <= kMaxTris)
            // CCollisionData does not expose the regular vertex count on this
            // ABI. GTA's collision vertex arena is bounded to 8192 entries.
            return {d->m_pVertices, 8192, d->m_pTriangles, static_cast<int>(d->m_nNumTriangles)};
        return {};
    }

    void ProbeTriangles(Track& t, const CColModel* col, const float mn[3], const float mx[3]) {
        t.triChecked = true; t.triOk = false;
        const TriangleSource src = GetTriangleSource(col ? col->m_pColData : nullptr);
        if (!src.triangles || !src.vertices || src.triangleCount <= 0) return;
        const int n = src.triangleCount, step = std::max(1, n / 24);
        const float pad = std::max(2.0f, 0.1f * std::max(mx[0] - mn[0], std::max(mx[1] - mn[1], mx[2] - mn[2])));
        int good = 0, tested = 0;
        for (int i = 0; i < n; i += step) {
            const CColTriangle& tri = src.triangles[i];
            const int32_t idx[3] = {tri.m_nVertA, tri.m_nVertB, tri.m_nVertC};
            bool ok = true;
            for (int k = 0; k < 3 && ok; ++k) {
                if (idx[k] < 0 || idx[k] >= src.vertexCount) { ok = false; break; }
                const CVector& v = src.vertices[idx[k]];
                if (!Finite3(v.x, v.y, v.z) || v.x < mn[0] - pad || v.x > mx[0] + pad || v.y < mn[1] - pad || v.y > mx[1] + pad ||
                    v.z < mn[2] - pad || v.z > mx[2] + pad) ok = false;
            }
            ++tested; if (ok) ++good;
        }
        t.triOk = tested > 0 && good * 10 >= tested * 9;
    }

    // ---------------------------------------------------------------- DFF geometry
    float g_dffVerts[kMaxDffVerts * 3];
    uint32_t g_dffTris[kMaxDffTris * 3];

    // Reads the real model. Fills g_builder with the projected triangles of every rendered atomic.
    // Returns the triangle count, or 0 when the geometry is not readable (then collision is used).
    int BuildFromDff(Track& t, CEntity* e, const Sun& sun) {
        RwObject* obj = e->m_pRwObject;
        if (!obj) return 0;
        RpAtomic* atomics[kMaxDffAtomics]; int nAtomics = 0;
        if (obj->type == rpATOMIC) atomics[nAtomics++] = reinterpret_cast<RpAtomic*>(obj);
        else if (obj->type == rpCLUMP) {
            RpClump* clump = reinterpret_cast<RpClump*>(obj);
            const RwLLLink* end = &clump->atomicList.link;
            for (const RwLLLink* l = end->next; l && l != end && nAtomics < kMaxDffAtomics; l = l->next)
                atomics[nAtomics++] = reinterpret_cast<RpAtomic*>(reinterpret_cast<char*>(const_cast<RwLLLink*>(l)) - offsetof(RpAtomic, inClumpLink));
        } else return 0;

        const CVector origin = e->GetPosition();
        int nv = 0, nt = 0;
        float zLow = 1.0e9f;
        for (int ai = 0; ai < nAtomics; ++ai) {
            RpAtomic* a = atomics[ai];
            if (!a || !(a->object.object.flags & rpATOMICRENDER)) continue;
            RpGeometry* g = a->geometry;
            if (!g || g->numVertices <= 0 || g->numTriangles <= 0 || g->numVertices > 30000 || !g->triangles ||
                !g->morphTarget || !g->morphTarget[0].verts) continue;
            if (nv + g->numVertices > kMaxDffVerts || nt + g->numTriangles > kMaxDffTris) return 0;     // too heavy: use collision
            const RwFrame* frame = static_cast<const RwFrame*>(a->object.object.parent);
            if (!frame) continue;
            const RwMatrix& m = frame->ltm;
            const RwV3d* src = g->morphTarget[0].verts;
            for (int i = 0; i < g->numVertices; ++i) {
                const RwV3d& v = src[i];
                const float x = m.right.x * v.x + m.up.x * v.y + m.at.x * v.z + m.pos.x - origin.x;
                const float y = m.right.y * v.x + m.up.y * v.y + m.at.y * v.z + m.pos.y - origin.y;
                const float z = m.right.z * v.x + m.up.z * v.y + m.at.z * v.z + m.pos.z - origin.z;
                if (!std::isfinite(x + y + z) || std::fabs(x) > 600.0f || std::fabs(y) > 600.0f || std::fabs(z) > 600.0f) return 0;   // not real positions
                float* o = &g_dffVerts[(nv + i) * 3]; o[0] = x; o[1] = y; o[2] = z;
                zLow = std::min(zLow, z);
            }
            for (int i = 0; i < g->numTriangles; ++i) {
                const RpTriangle& tr = g->triangles[i];
                if (tr.vertIndex[0] >= g->numVertices || tr.vertIndex[1] >= g->numVertices || tr.vertIndex[2] >= g->numVertices) return 0;
                uint32_t* o = &g_dffTris[nt * 3];
                o[0] = nv + tr.vertIndex[0]; o[1] = nv + tr.vertIndex[1]; o[2] = nv + tr.vertIndex[2];
                ++nt;
            }
            nv += g->numVertices;
        }
        if (nt < 4 || !(zLow < 1.0e8f)) return 0;
        t.zRef = zLow;
        g_builder.Begin(sun.dirX, sun.dirY, sun.lengthFactor, zLow);
        for (int i = 0; i < nt; ++i) {
            const float* p0 = &g_dffVerts[g_dffTris[i * 3] * 3];
            const float* p1 = &g_dffVerts[g_dffTris[i * 3 + 1] * 3];
            const float* p2 = &g_dffVerts[g_dffTris[i * 3 + 2] * 3];
            g_builder.AddTriangle({p0[0], p0[1], p0[2]}, {p1[0], p1[1], p1[2]}, {p2[0], p2[1], p2[2]});
        }
        return g_builder.ShapeCount() > 0 ? nt : 0;
    }

    // Collects every primitive into g_builder. zRef is the lowest point (relative to the origin).
    bool BuildShapes(Track& t, CEntity* e, const Sun& sun, const Axes& ax, int& trisUsed) {
        if (VehicleLightGL::GetSettings().shadowDff && t.dffState != 2) {
            const int n = BuildFromDff(t, e, sun);
            if (n > 0) { t.dffState = 1; trisUsed = n; return true; }
            if (t.dffState == 0) t.dffState = 2;           // first attempt failed: stay on collision for this caster
        }
        CColModel* col = e->GetColModel();
        if (!col) return false;
        float mn[3], mx[3];
        if (!ColBounds(e, col, mn, mx)) return false;
        const CCollisionData* d = col->m_pColData;
        if (!t.triChecked) ProbeTriangles(t, col, mn, mx);
        const TriangleSource triSrc = GetTriangleSource(d);

        // pass 1: lowest point
        float zLow = 1.0e9f;
        auto low = [&](float z) { zLow = std::min(zLow, z); };
        int nBox = 0, nSph = 0, nTri = 0;
        if (d && d->m_pBoxes) nBox = std::min<int>(d->m_nNumBoxes, kMaxBoxes);
        if (d && d->m_pSpheres) nSph = std::min<int>(d->m_nNumSpheres, kMaxSpheres);
        if (t.triOk) nTri = triSrc.triangleCount;
        for (int i = 0; i < nBox; ++i) {
            const CBox& b = d->m_pBoxes[i];
            if (!Finite3(b.m_vecMin.x, b.m_vecMin.y, b.m_vecMin.z) || !Finite3(b.m_vecMax.x, b.m_vecMax.y, b.m_vecMax.z)) continue;
            for (int c = 0; c < 8; ++c) low(Rot(ax, (c & 1) ? b.m_vecMax.x : b.m_vecMin.x, (c & 2) ? b.m_vecMax.y : b.m_vecMin.y, (c & 4) ? b.m_vecMax.z : b.m_vecMin.z).z);
        }
        for (int i = 0; i < nSph; ++i) {
            const CSphere& s = d->m_pSpheres[i];
            if (!Finite3(s.m_vecCentre.x, s.m_vecCentre.y, s.m_vecCentre.z) || !(s.m_fRadius > 0.0f) || s.m_fRadius > 40.0f) continue;
            low(Rot(ax, s.m_vecCentre.x, s.m_vecCentre.y, s.m_vecCentre.z).z - s.m_fRadius);
        }
        for (int i = 0; i < nTri; ++i) {
            const CColTriangle& tri = triSrc.triangles[i];
            const int32_t idx[3] = {tri.m_nVertA, tri.m_nVertB, tri.m_nVertC};
            for (int k = 0; k < 3; ++k) {
                if (idx[k] < 0 || idx[k] >= triSrc.vertexCount) continue;
                const CVector& v = triSrc.vertices[idx[k]];
                if (Finite3(v.x, v.y, v.z)) low(Rot(ax, v.x, v.y, v.z).z);
            }
        }
        // bounding box always contributes the ground reference when nothing else exists
        const bool haveAny = nBox + nSph + nTri > 0;
        if (!haveAny) { for (int c = 0; c < 8; ++c) low(Rot(ax, (c & 1) ? mx[0] : mn[0], (c & 2) ? mx[1] : mn[1], (c & 4) ? mx[2] : mn[2]).z); }
        if (!(zLow < 1.0e8f)) return false;
        t.zRef = zLow;

        // pass 2: shapes
        g_builder.Begin(sun.dirX, sun.dirY, sun.lengthFactor, zLow);
        for (int i = 0; i < nBox; ++i) {
            const CBox& b = d->m_pBoxes[i];
            if (!Finite3(b.m_vecMin.x, b.m_vecMin.y, b.m_vecMin.z) || !Finite3(b.m_vecMax.x, b.m_vecMax.y, b.m_vecMax.z)) continue;
            const float bmn[3] = {b.m_vecMin.x, b.m_vecMin.y, b.m_vecMin.z}, bmx[3] = {b.m_vecMax.x, b.m_vecMax.y, b.m_vecMax.z};
            V3 c[8]; BoxCorners(bmn, bmx, ax.r, ax.f, ax.u, c); g_builder.AddBox(c);
        }
        for (int i = 0; i < nSph; ++i) {
            const CSphere& s = d->m_pSpheres[i];
            if (!Finite3(s.m_vecCentre.x, s.m_vecCentre.y, s.m_vecCentre.z) || !(s.m_fRadius > 0.0f) || s.m_fRadius > 40.0f) continue;
            g_builder.AddSphere(Rot(ax, s.m_vecCentre.x, s.m_vecCentre.y, s.m_vecCentre.z), s.m_fRadius);
        }
        for (int i = 0; i < nTri; ++i) {
            const CColTriangle& tri = triSrc.triangles[i];
            if (tri.m_nVertA < 0 || tri.m_nVertB < 0 || tri.m_nVertC < 0 ||
                tri.m_nVertA >= triSrc.vertexCount || tri.m_nVertB >= triSrc.vertexCount || tri.m_nVertC >= triSrc.vertexCount)
                continue;
            const CVector& a = triSrc.vertices[tri.m_nVertA];
            const CVector& b = triSrc.vertices[tri.m_nVertB];
            const CVector& c = triSrc.vertices[tri.m_nVertC];
            g_builder.AddTriangle(Rot(ax, a.x, a.y, a.z), Rot(ax, b.x, b.y, b.z), Rot(ax, c.x, c.y, c.z));
        }
        trisUsed = nTri;
        if (!haveAny) {
            V3 c[8]; BoxCorners(mn, mx, ax.r, ax.f, ax.u, c); g_builder.AddBox(c);
        }
        // Trees / bushes: collision is only the trunk, the visible model is much bigger. Add a canopy.
        if ((t.kind == kBuilding || t.kind == kObject || t.kind == kDummy) && e->m_pRwObject && e->m_pRwObject->type == rpATOMIC) {
            const RpAtomic* atom = reinterpret_cast<const RpAtomic*>(e->m_pRwObject);
            const RwSphere& bs = atom->boundingSphere;
            const float r = bs.radius;
            const float halfH = 0.5f * std::max(mx[0] - mn[0], mx[1] - mn[1]);
            if (std::isfinite(r + bs.center.x + bs.center.y + bs.center.z) && r >= 1.8f && r < 25.0f && r > 2.0f * halfH &&
                bs.center.z + r > mx[2] + 1.5f)
                g_builder.AddSphere(Rot(ax, bs.center.x, bs.center.y, bs.center.z + 0.1f * r), r * 0.82f);
        }
        return g_builder.ShapeCount() > 0;
    }

    // ---------------------------------------------------------------- textures
    bool EnsureImage() {
        if (g_image) return true;
        if (g_imageFailed) return false;
        g_image = RwImageCreate(kMaskSize, kMaskSize, 32);
        if (g_image) RwImageAllocatePixels(g_image);
        if (!g_image || !g_image->cpPixels) { g_imageFailed = true; logger->Info("ComplexShadow: RwImage allocation failed"); return false; }
        return true;
    }
    void FillImage(const uint8_t* mask) {
        for (int y = 0; y < kMaskSize; ++y) {
            uint8_t* row = g_image->cpPixels + y * g_image->stride;
            for (int x = 0; x < kMaskSize; ++x) { const uint8_t v = mask[y * kMaskSize + x]; row[x * 4 + 0] = v; row[x * 4 + 1] = v; row[x * 4 + 2] = v; row[x * 4 + 3] = v; }
        }
    }
    int AcquireSlot() {
        if (!EnsureImage()) return -1;
        for (int i = 0; i < kMaxTracks; ++i) {
            if (g_slots[i].used) continue;
            if (!g_slots[i].tex) {
                RwInt32 w = g_image->width, h = g_image->height, dp = g_image->depth, fmt = 0;
                if (!RwImageFindRasterFormat(g_image, rwRASTERTYPETEXTURE, &w, &h, &dp, &fmt)) { g_imageFailed = true; return -1; }
                RwRaster* raster = RwRasterCreate(w, h, dp, rwRASTERTYPETEXTURE | fmt);
                if (!raster) return -1;
                FillImage(g_mask);
                if (!RwRasterSetFromImage(raster, g_image)) { RwRasterDestroy(raster); return -1; }
                RwTexture* tex = RwTextureCreate(raster);
                if (!tex) { RwRasterDestroy(raster); return -1; }
                RwTextureSetName(tex, "beta_sun_shadow");
                tex->filterAddressing = 0x2u | (3u << 8) | (3u << 12);     // linear, clamp/clamp
                g_slots[i].tex = tex; g_slots[i].raster = raster;
            }
            g_slots[i].used = true;
            return i;
        }
        return -1;
    }
    void ReleaseSlot(Track& t) { if (t.slot >= 0) g_slots[t.slot].used = false; t.slot = -1; t.hasMask = false; }

    void StoreAxes(Track& t, const Axes& ax) {
        const float v[9] = {ax.r.x, ax.r.y, ax.r.z, ax.f.x, ax.f.y, ax.f.z, ax.u.x, ax.u.y, ax.u.z};
        std::memcpy(t.genAxes, v, sizeof(v));
    }
    Axes GetAxes(CEntity* e) {
        const CVector r = e->GetRightDirection(), f = e->GetTopDirection(), u = e->GetAtDirection();
        return { {r.x, r.y, r.z}, {f.x, f.y, f.z}, {u.x, u.y, u.z} };
    }
    bool NeedsRegen(const Track& t, const Sun& sun, const Axes& ax, int orient) {
        if (!t.hasMask || t.genOrient != orient) return true;
        if (t.genDirX * sun.dirX + t.genDirY * sun.dirY < kRegenSunCos) return true;
        if (std::fabs(t.genL - sun.lengthFactor) > 0.05f * (t.genL + 0.5f)) return true;
        const float v[9] = {ax.r.x, ax.r.y, ax.r.z, ax.f.x, ax.f.y, ax.f.z, ax.u.x, ax.u.y, ax.u.z};
        for (int k = 0; k < 3; ++k)
            if (t.genAxes[k * 3] * v[k * 3] + t.genAxes[k * 3 + 1] * v[k * 3 + 1] + t.genAxes[k * 3 + 2] * v[k * 3 + 2] < kRegenAxisCos) return true;
        return false;
    }

    bool Regenerate(Track& t, CEntity* e, const Sun& sun, int orient) {
        const Axes ax = GetAxes(e);
        int tris = 0;
        if (!BuildShapes(t, e, sun, ax, tris)) return false;
        Quad q;
        if (!g_builder.Finish(orient, kMaxHalfA, kMaxHalfC, g_mask, q)) return false;
        if (t.slot < 0) { t.slot = AcquireSlot(); if (t.slot < 0) return false; }
        if (!EnsureImage()) return false;
        FillImage(g_mask);
        if (!RwRasterSetFromImage(g_slots[t.slot].raster, g_image)) return false;
        t.quad = q; t.genDirX = sun.dirX; t.genDirY = sun.dirY; t.genL = sun.lengthFactor; t.genOrient = orient;
        StoreAxes(t, ax);
        t.hasMask = true;
        ++g_regenThisFrame; g_trisThisFrame += tris;
        return true;
    }

    // ---------------------------------------------------------------- scanning / tracking
    struct Cand { Kind kind; int index; uint8_t id; CEntity* e; float d2; float rank; Info info; };
    // Building and dummy pools contain the map's authored LOD proxies. Keep
    // a generous candidate list, but only the nearest 18 are rasterised.
    constexpr int kCandMax = 160;
    constexpr int kQuota[5] = {kCandMax, kCandMax, 18, 18, kCandMax};
    Cand g_cands[5][kCandMax];
    int g_candN[5];

    void ConsiderCand(Kind kind, int index, uint8_t id, CEntity* e, const CVector& cam, float range2, int area) {
        if (!e || static_cast<int>(e->m_nAreaCode) != area) return;
        const CVector& p = e->GetPosition();
        const float dx = p.x - cam.x, dy = p.y - cam.y, d2 = dx * dx + dy * dy;
        if (!(d2 < range2)) return;
        int& n = g_candN[kind];
        const int quota = kQuota[kind];
        int slot;
        if (n < quota) slot = n++;
        else {
            slot = 0;
            for (int i = 1; i < n; ++i) if (g_cands[kind][i].d2 > g_cands[kind][slot].d2) slot = i;
            if (d2 >= g_cands[kind][slot].d2) return;
        }
        g_cands[kind][slot] = Cand{kind, index, id, e, d2, 0.0f, Info{}};
    }
    template <typename PoolT>
    void ScanPool(PoolT* pool, Kind kind, const CVector& cam, float range2, int area) {
        if (!pool || !pool->m_pObjects || !pool->m_byteMap) return;
        for (int i = 0; i < pool->m_nSize; ++i) {
            if (pool->m_byteMap[i].bEmpty) continue;
            ConsiderCand(kind, i, pool->m_byteMap[i].nId, reinterpret_cast<CEntity*>(&pool->m_pObjects[i]), cam, range2, area);
        }
    }
    Track* FindTrack(Kind kind, int index, uint8_t id) {
        for (Track& t : g_tracks) if (t.used && t.kind == kind && t.index == index && t.id == id) return &t;
        return nullptr;
    }

    void Scan(const VehicleLightGL::Settings& s, const CVector& cam, int area) {
        const float range = s.shadowBuildingRange, range2 = range * range;
        const float dynRange = std::min(range, 48.0f);
        for (int& n : g_candN) n = 0;
        ScanPool(CPools::ms_pBuildingPool, kBuilding, cam, range2, area);
        ScanPool(CPools::ms_pObjectPool, kObject, cam, range2, area);
        ScanPool(CPools::ms_pDummyPool, kDummy, cam, range2, area);
        if (s.shadowVehicles) ScanPool(CPools::ms_pVehiclePool, kVehicle, cam, dynRange * dynRange, area);
        if (s.shadowPeds) ScanPool(CPools::ms_pPedPool, kPed, cam, dynRange * dynRange, area);

        static Cand all[5 * kCandMax];
        int total = 0;
        for (int k = 0; k < 5; ++k)
            for (int i = 0; i < g_candN[k]; ++i) {
                Cand c = g_cands[k][i];
                if (!Classify(c.e, c.kind, c.info)) continue;
                const bool tracked = FindTrack(c.kind, c.index, c.id) != nullptr;
                c.rank = std::sqrt(c.d2) / c.info.importance * (tracked ? 0.70f : 1.0f);   // hysteresis: tracked casters stay
                all[total++] = c;
            }
        g_lastCands = total;
        std::sort(all, all + total, [](const Cand& a, const Cand& b) { return a.rank < b.rank; });
        for (Track& t : g_tracks) t.selected = false;
        for (int i = 0; i < total && i < kWanted; ++i) {
            const Cand& c = all[i];
            Track* t = FindTrack(c.kind, c.index, c.id);
            if (!t) {
                for (Track& f : g_tracks) if (!f.used) { t = &f; break; }
                if (!t) continue;                                   // all slots busy fading: join later
                *t = Track{};
                t->used = true; t->kind = c.kind; t->index = c.index; t->id = c.id; t->model = c.e->m_nModelIndex;
            }
            if (t->model != c.e->m_nModelIndex) { t->model = c.e->m_nModelIndex; t->hasMask = false; t->triChecked = false; }
            t->selected = true;
            t->importance = c.info.importance;
        }
    }

    float KindStrength(Kind k, const VehicleLightGL::Settings& s) {
        switch (k) {
            case kBuilding: return 1.0f;
            case kObject:   return 0.9f;
            case kDummy:    return 0.95f;
            case kVehicle:  return s.shadowVehicles ? 0.80f : 0.0f;
            default:        return s.shadowPeds ? 0.70f : 0.0f;
        }
    }

    // ---------------------------------------------------------------- geometry casters (shader sun shadow)
    // The screen-built maps only contain what the camera currently sees, so off-screen buildings, poles and
    // the hidden sides of objects cast nothing and shadows appear/disappear as the camera moves. Here the
    // nearest static models are read from the DFF (collision as fallback) ONCE, and handed to the GL side as
    // triangles that are drawn straight into the sun maps. Result: shadows that depend only on world + sun.
    constexpr int kMaxMeshTracks = 96;
    constexpr int kMeshScanInterval = 10, kMeshBuildsPerFrame = 3, kMeshGraceFrames = 150, kMeshRetryFrames = 45, kMeshUpgradeFrames = 90, kMeshMaxAttempts = 8;
    constexpr int kMeshMaxVerts = 30000, kMeshMaxTris = 15000;

    struct MeshTrack {
        bool used = false, selected = false, built = false, failed = false, fromDff = false;
        int idle = 0, retry = 0, attempts = 0, upgrade = 0;
        Kind kind = kBuilding; int index = 0; uint8_t id = 0; uint16_t model = 0;
        CVector origin; float axes[9] = {};
    };
    MeshTrack g_meshTracks[kMaxMeshTracks];
    bool g_meshWasOn = false;
    int g_meshScanCountdown = 0;
    std::vector<float> g_mv;
    std::vector<uint32_t> g_mi;

    uint32_t MeshKey(const MeshTrack& t) {
        return ((static_cast<uint32_t>(t.kind) + 1u) << 28) | ((static_cast<uint32_t>(t.index) & 0xFFFFFu) << 8) | t.id;
    }
    CEntity* ResolveMesh(const MeshTrack& t) {
        switch (t.kind) {
            case kBuilding: return PoolAt(CPools::ms_pBuildingPool, t.index, t.id);
            case kObject:   return PoolAt(CPools::ms_pObjectPool, t.index, t.id);
            case kDummy:    return PoolAt(CPools::ms_pDummyPool, t.index, t.id);
            default:        return nullptr;
        }
    }
    MeshTrack* FindMeshTrack(Kind kind, int index, uint8_t id) {
        for (MeshTrack& t : g_meshTracks) if (t.used && t.kind == kind && t.index == index && t.id == id) return &t;
        return nullptr;
    }

    void MeshTri(const V3& a, const V3& b, const V3& c) {
        if (g_mi.size() / 3 >= static_cast<size_t>(kMeshMaxTris) || g_mv.size() / 3 + 3 > static_cast<size_t>(kMeshMaxVerts)) return;
        const uint32_t base = static_cast<uint32_t>(g_mv.size() / 3);
        const V3* p[3] = {&a, &b, &c};
        for (int i = 0; i < 3; ++i) { g_mv.push_back(p[i]->x); g_mv.push_back(p[i]->y); g_mv.push_back(p[i]->z); g_mi.push_back(base + i); }
    }
    void MeshBox(const V3 c[8]) {
        for (int a = 0; a < 3; ++a)
            for (int side = 0; side < 2; ++side) {
                const int u = (a + 1) % 3, v = (a + 2) % 3, base = side << a;
                const int i0 = base, i1 = base | (1 << u), i2 = base | (1 << u) | (1 << v), i3 = base | (1 << v);
                MeshTri(c[i0], c[i1], c[i2]); MeshTri(c[i0], c[i2], c[i3]);
            }
    }
    void MeshSphere(const V3& ctr, float r) {
        constexpr int kLon = 10, kLat = 6;
        auto pt = [&](int la, int lo) {
            const float th = kPi * static_cast<float>(la) / kLat, ph = 2.0f * kPi * static_cast<float>(lo % kLon) / kLon;
            return V3{ctr.x + r * std::sin(th) * std::cos(ph), ctr.y + r * std::sin(th) * std::sin(ph), ctr.z + r * std::cos(th)};
        };
        for (int la = 0; la < kLat; ++la)
            for (int lo = 0; lo < kLon; ++lo) {
                const V3 a = pt(la, lo), b = pt(la, lo + 1), c = pt(la + 1, lo + 1), d = pt(la + 1, lo);
                MeshTri(a, b, c); MeshTri(a, c, d);
            }
    }

    // Every rendered atomic of the model, transformed to world space (relative to the entity position).
    bool MeshFromDff(CEntity* e, const CVector& origin) {
        RwObject* obj = e->m_pRwObject;
        if (!obj) return false;
        RpAtomic* atomics[kMaxDffAtomics]; int nAtomics = 0;
        if (obj->type == rpATOMIC) atomics[nAtomics++] = reinterpret_cast<RpAtomic*>(obj);
        else if (obj->type == rpCLUMP) {
            RpClump* clump = reinterpret_cast<RpClump*>(obj);
            const RwLLLink* end = &clump->atomicList.link;
            for (const RwLLLink* l = end->next; l && l != end && nAtomics < kMaxDffAtomics; l = l->next)
                atomics[nAtomics++] = reinterpret_cast<RpAtomic*>(reinterpret_cast<char*>(const_cast<RwLLLink*>(l)) - offsetof(RpAtomic, inClumpLink));
        } else return false;
        size_t nTri = 0;
        // Atomics on the entity's own frame use the entity matrix: the frame's cached ltm can be stale until the model is rendered.
        const RwFrame* rootFrame = static_cast<const RwFrame*>(obj->parent);
        const Axes eax = GetAxes(e);
        for (int ai = 0; ai < nAtomics; ++ai) {
            RpAtomic* a = atomics[ai];
            if (!a || !(a->object.object.flags & rpATOMICRENDER)) continue;
            RpGeometry* g = a->geometry;
            if (!g || g->numVertices <= 0 || g->numTriangles <= 0 || g->numVertices > kMeshMaxVerts || !g->triangles ||
                !g->morphTarget || !g->morphTarget[0].verts) continue;
            if (g_mv.size() / 3 + g->numVertices > static_cast<size_t>(kMeshMaxVerts) || nTri + g->numTriangles > static_cast<size_t>(kMeshMaxTris)) return false;
            const RwFrame* frame = static_cast<const RwFrame*>(a->object.object.parent);
            if (!frame) continue;
            const RwMatrix& m = frame->ltm;
            const RwV3d* src = g->morphTarget[0].verts;
            const uint32_t base = static_cast<uint32_t>(g_mv.size() / 3);
            for (int i = 0; i < g->numVertices; ++i) {
                const RwV3d& v = src[i];
                if (frame == rootFrame) {
                    const float ex = eax.r.x * v.x + eax.f.x * v.y + eax.u.x * v.z;
                    const float ey = eax.r.y * v.x + eax.f.y * v.y + eax.u.y * v.z;
                    const float ez = eax.r.z * v.x + eax.f.z * v.y + eax.u.z * v.z;
                    if (!std::isfinite(ex + ey + ez) || std::fabs(ex) > 600.0f || std::fabs(ey) > 600.0f || std::fabs(ez) > 600.0f) return false;
                    g_mv.push_back(ex); g_mv.push_back(ey); g_mv.push_back(ez);
                    continue;
                }
                const float x = m.right.x * v.x + m.up.x * v.y + m.at.x * v.z + m.pos.x - origin.x;
                const float y = m.right.y * v.x + m.up.y * v.y + m.at.y * v.z + m.pos.y - origin.y;
                const float z = m.right.z * v.x + m.up.z * v.y + m.at.z * v.z + m.pos.z - origin.z;
                if (!std::isfinite(x + y + z) || std::fabs(x) > 600.0f || std::fabs(y) > 600.0f || std::fabs(z) > 600.0f) return false;
                g_mv.push_back(x); g_mv.push_back(y); g_mv.push_back(z);
            }
            for (int i = 0; i < g->numTriangles; ++i) {
                const RpTriangle& tr = g->triangles[i];
                if (tr.vertIndex[0] >= g->numVertices || tr.vertIndex[1] >= g->numVertices || tr.vertIndex[2] >= g->numVertices) return false;
                g_mi.push_back(base + tr.vertIndex[0]); g_mi.push_back(base + tr.vertIndex[1]); g_mi.push_back(base + tr.vertIndex[2]);
                ++nTri;
            }
        }
        return nTri >= 4;
    }

    // Collision boxes/spheres/triangles (+ a canopy sphere for trees) when the DFF cannot be read.
    bool MeshFromCollision(CEntity* e) {
        CColModel* col = e->GetColModel();
        if (!col) return false;
        float mn[3], mx[3];
        if (!ColBounds(e, col, mn, mx)) return false;
        const CCollisionData* d = col->m_pColData;
        const Axes ax = GetAxes(e);
        Track probe;
        ProbeTriangles(probe, col, mn, mx);
        const TriangleSource src = GetTriangleSource(d);
        bool any = false;
        if (d && d->m_pBoxes)
            for (int i = 0; i < std::min<int>(d->m_nNumBoxes, kMaxBoxes); ++i) {
                const CBox& b = d->m_pBoxes[i];
                if (!Finite3(b.m_vecMin.x, b.m_vecMin.y, b.m_vecMin.z) || !Finite3(b.m_vecMax.x, b.m_vecMax.y, b.m_vecMax.z)) continue;
                const float bmn[3] = {b.m_vecMin.x, b.m_vecMin.y, b.m_vecMin.z}, bmx[3] = {b.m_vecMax.x, b.m_vecMax.y, b.m_vecMax.z};
                V3 c[8]; BoxCorners(bmn, bmx, ax.r, ax.f, ax.u, c); MeshBox(c); any = true;
            }
        if (d && d->m_pSpheres)
            for (int i = 0; i < std::min<int>(d->m_nNumSpheres, kMaxSpheres); ++i) {
                const CSphere& sp = d->m_pSpheres[i];
                if (!Finite3(sp.m_vecCentre.x, sp.m_vecCentre.y, sp.m_vecCentre.z) || !(sp.m_fRadius > 0.0f) || sp.m_fRadius > 40.0f) continue;
                MeshSphere(Rot(ax, sp.m_vecCentre.x, sp.m_vecCentre.y, sp.m_vecCentre.z), sp.m_fRadius); any = true;
            }
        if (probe.triOk)
            for (int i = 0; i < src.triangleCount; ++i) {
                const CColTriangle& tri = src.triangles[i];
                if (tri.m_nVertA < 0 || tri.m_nVertB < 0 || tri.m_nVertC < 0 || tri.m_nVertA >= src.vertexCount ||
                    tri.m_nVertB >= src.vertexCount || tri.m_nVertC >= src.vertexCount) continue;
                const CVector &a = src.vertices[tri.m_nVertA], &b = src.vertices[tri.m_nVertB], &c = src.vertices[tri.m_nVertC];
                MeshTri(Rot(ax, a.x, a.y, a.z), Rot(ax, b.x, b.y, b.z), Rot(ax, c.x, c.y, c.z)); any = true;
            }
        if (!any) { V3 c[8]; BoxCorners(mn, mx, ax.r, ax.f, ax.u, c); MeshBox(c); any = true; }
        if (e->m_pRwObject && e->m_pRwObject->type == rpATOMIC) {
            const RwSphere& bs = reinterpret_cast<const RpAtomic*>(e->m_pRwObject)->boundingSphere;
            const float r = bs.radius, halfH = 0.5f * std::max(mx[0] - mn[0], mx[1] - mn[1]);
            if (std::isfinite(r + bs.center.x + bs.center.y + bs.center.z) && r >= 1.8f && r < 25.0f && r > 2.0f * halfH && bs.center.z + r > mx[2] + 1.5f)
                MeshSphere(Rot(ax, bs.center.x, bs.center.y, bs.center.z + 0.1f * r), r * 0.82f);
        }
        return any;
    }

    bool BuildAndSubmitMesh(MeshTrack& t, CEntity* e, const VehicleLightGL::Settings& s) {
        const CVector origin = e->GetPosition();
        g_mv.clear(); g_mi.clear();
        bool ok = s.shadowDff && MeshFromDff(e, origin);
        t.fromDff = ok;
        if (!ok) { g_mv.clear(); g_mi.clear(); ok = MeshFromCollision(e); }
        if (!ok || g_mi.size() < 12) return false;
        const double o[3] = {origin.x, origin.y, origin.z};
        ScreenShadow::CasterSet(MeshKey(t), o, g_mv.data(), static_cast<int>(g_mv.size() / 3), g_mi.data(), static_cast<int>(g_mi.size()));
        t.origin = origin;
        const Axes ax = GetAxes(e);
        const float v[9] = {ax.r.x, ax.r.y, ax.r.z, ax.f.x, ax.f.y, ax.f.z, ax.u.x, ax.u.y, ax.u.z};
        for (int i = 0; i < 9; ++i) t.axes[i] = v[i];
        return true;
    }

    bool MeshMoved(const MeshTrack& t, CEntity* e) {
        const CVector& p = e->GetPosition();
        if (std::fabs(p.x - t.origin.x) + std::fabs(p.y - t.origin.y) + std::fabs(p.z - t.origin.z) > 0.03f) return true;
        const Axes ax = GetAxes(e);
        const float v[9] = {ax.r.x, ax.r.y, ax.r.z, ax.f.x, ax.f.y, ax.f.z, ax.u.x, ax.u.y, ax.u.z};
        for (int k = 0; k < 3; ++k)
            if (t.axes[k * 3] * v[k * 3] + t.axes[k * 3 + 1] * v[k * 3 + 1] + t.axes[k * 3 + 2] * v[k * 3 + 2] < 0.9999f) return true;
        return false;
    }

    // Picks the nearest static casters (map buildings, props, LOD dummies) with hysteresis.
    void ScanMesh(const VehicleLightGL::Settings& s, const CVector& cam, int area) {
        // Wider than the decal range: a tall building up-sun of the camera shadows ground well beyond itself.
        const float range = std::min(150.0f, s.shadowBuildingRange * 1.4f), range2 = range * range;
        for (int& n : g_candN) n = 0;
        ScanPool(CPools::ms_pBuildingPool, kBuilding, cam, range2, area);
        ScanPool(CPools::ms_pObjectPool, kObject, cam, range2, area);
        ScanPool(CPools::ms_pDummyPool, kDummy, cam, range2, area);
        static Cand all[5 * kCandMax];
        int total = 0;
        // Rank by how close the caster's SHADOW comes to the camera, not by where the caster stands: the buildings
        // that matter are on the sun side (often behind the camera), their shadow falls toward the player.
        const float hl = std::sqrt(g_sun.dirX * g_sun.dirX + g_sun.dirY * g_sun.dirY);
        const float dx = hl > 1.0e-4f ? g_sun.dirX / hl : 0.0f, dy = hl > 1.0e-4f ? g_sun.dirY / hl : 1.0f;
        const float lf = 1.0f / std::tan(std::max(g_sun.elevation, ShadowMap::kMinSunElevation));
        for (int k = 0; k < 5; ++k)
            for (int i = 0; i < g_candN[k]; ++i) {
                Cand c = g_cands[k][i];
                if (!Classify(c.e, c.kind, c.info)) continue;
                const bool tracked = FindMeshTrack(c.kind, c.index, c.id) != nullptr;
                const CVector& p = c.e->GetPosition();
                const float px = p.x - cam.x, py = p.y - cam.y;
                const float len = std::min(c.info.height * lf, 160.0f);
                const float t = std::max(0.0f, std::min(len, -(px * dx + py * dy)));
                const float cx = px + dx * t, cy = py + dy * t;
                const float reach = std::max(0.0f, std::sqrt(cx * cx + cy * cy) - 0.5f * c.info.extent);
                c.rank = (reach + 0.25f * std::sqrt(c.d2)) / c.info.importance * (tracked ? 0.70f : 1.0f);
                all[total++] = c;
            }
        std::sort(all, all + total, [](const Cand& a, const Cand& b) { return a.rank < b.rank; });
        for (MeshTrack& t : g_meshTracks) t.selected = false;
        const int wanted = std::max(8, std::min(kMaxMeshTracks, s.sunShadowMeshCount));
        for (int i = 0; i < total && i < wanted; ++i) {
            const Cand& c = all[i];
            MeshTrack* t = FindMeshTrack(c.kind, c.index, c.id);
            if (!t) {
                for (MeshTrack& f : g_meshTracks) if (!f.used) { t = &f; break; }
                if (!t) {                                           // full: drop the longest-idle unselected caster
                    for (MeshTrack& f : g_meshTracks) if (f.used && !f.selected && (!t || f.idle > t->idle)) t = &f;
                    if (!t) continue;
                    if (t->built) ScreenShadow::CasterRemove(MeshKey(*t));
                }
                *t = MeshTrack{};
                t->used = true; t->kind = c.kind; t->index = c.index; t->id = c.id; t->model = c.e->m_nModelIndex;
            }
            if (t->model != c.e->m_nModelIndex) { t->model = c.e->m_nModelIndex; t->built = false; t->failed = false; t->attempts = 0; t->retry = 0; }
            t->selected = true;
        }
    }

    void UpdateMeshCasters(const VehicleLightGL::Settings& s, const CVector& cam, bool on) {
        if (!on) {
            if (g_meshWasOn) {
                ScreenShadow::CasterClear();
                for (MeshTrack& t : g_meshTracks) t = MeshTrack{};
                g_meshWasOn = false;
            }
            return;
        }
        g_meshWasOn = true;
        if (--g_meshScanCountdown <= 0) { g_meshScanCountdown = kMeshScanInterval; ScanMesh(s, cam, CGame::currArea); }
        int builds = 0;
        for (MeshTrack& t : g_meshTracks) {
            if (!t.used) continue;
            CEntity* e = ResolveMesh(t);
            const CVector* p = e ? &e->GetPosition() : nullptr;
            if (!e || !Finite3(p->x, p->y, p->z)) {
                if (t.built) ScreenShadow::CasterRemove(MeshKey(t));
                t = MeshTrack{};
                continue;
            }
            // Leaving the nearest set does not drop the shadow at once (no popping while walking); it goes after a grace period.
            if (t.selected) t.idle = 0;
            else if (++t.idle > kMeshGraceFrames) {
                if (t.built) ScreenShadow::CasterRemove(MeshKey(t));
                t = MeshTrack{};
                continue;
            }
            if (t.failed) {                                          // model not streamed in yet / no data: try again later
                if (t.attempts < kMeshMaxAttempts && --t.retry <= 0) t.failed = false;
                else continue;
            }
            if (builds >= kMeshBuildsPerFrame) continue;
            bool rebuild = !t.built || MeshMoved(t, e);
            // A caster built from the collision fallback is upgraded as soon as the real model is available.
            if (!rebuild && !t.fromDff && s.shadowDff && e->m_pRwObject && --t.upgrade <= 0) { t.upgrade = kMeshUpgradeFrames; rebuild = true; }
            if (!rebuild) continue;
            ++builds;
            const bool wasBuilt = t.built, wasDff = t.fromDff;
            if (BuildAndSubmitMesh(t, e, s)) { t.built = true; t.attempts = 0; }
            else if (wasBuilt) { t.built = true; t.fromDff = wasDff; }     // keep the previous good mesh
            else { t.failed = true; t.retry = kMeshRetryFrames; ++t.attempts; }
        }
    }

    // ---------------------------------------------------------------- moving casters (vehicles, pedestrians)
    // The depth-splat grid is too sparse for thin or moving shapes (legs, bikes): it dots and flickers.
    // Simple solid proxies are used instead: stacked spheres for a pedestrian, a two-box hull for a vehicle,
    // rebuilt only when the entity moved.
    struct DynTrack { bool used = false; uint32_t key = 0; int seen = -1; CVector origin; float axes[9] = {}; };
    constexpr int kDynMax = 64, kDynVeh = 24, kDynPed = 24;
    DynTrack g_dyn[kDynMax];
    bool g_dynWasOn = false;
    int g_dynFrame = 0;

    void DynClear() {
        for (DynTrack& d : g_dyn) { if (d.used) ScreenShadow::CasterRemove(d.key); d = DynTrack{}; }
    }
    void DynPedMesh(CEntity* e) {
        const Axes ax = GetAxes(e);
        static const float sp[5][2] = {{0.62f, 0.14f}, {0.32f, 0.22f}, {0.00f, 0.20f}, {-0.42f, 0.16f}, {-0.80f, 0.14f}};
        for (const auto& q : sp) MeshSphere(Rot(ax, 0.0f, 0.0f, q[0]), q[1]);
    }
    bool DynVehicleMesh(CEntity* e) {
        if (VehicleLightGL::GetSettings().shadowDff && e->m_pRwObject) {
            const size_t vBase = g_mv.size(), iBase = g_mi.size();
            const CVector origin = e->GetPosition();
            if (MeshFromDff(e, origin)) return true;
            g_mv.resize(vBase);
            g_mi.resize(iBase);
        }
        CColModel* col = e->GetColModel();
        if (!col) return false;
        float mn[3], mx[3];
        if (!ColBounds(e, col, mn, mx)) return false;
        const float h = mx[2] - mn[2], len = mx[1] - mn[1], wid = mx[0] - mn[0];
        if (!(h > 0.3f) || !(len > 0.5f) || !(wid > 0.3f) || len > 40.0f) return false;
        const Axes ax = GetAxes(e);
        const float zm = mn[2] + 0.55f * h;
        const float lowMax[3] = {mx[0], mx[1], zm};
        V3 c[8];
        BoxCorners(mn, lowMax, ax.r, ax.f, ax.u, c); MeshBox(c);
        const float cabMin[3] = {mn[0] + 0.07f * wid, mn[1] + 0.26f * len, zm};
        const float cabMax[3] = {mx[0] - 0.07f * wid, mx[1] - 0.30f * len, mx[2]};
        BoxCorners(cabMin, cabMax, ax.r, ax.f, ax.u, c); MeshBox(c);
        return true;
    }

    void UpdateDynamicCasters(const VehicleLightGL::Settings& s, const CVector& cam, bool on) {
        if (!on) {
            if (g_dynWasOn) { DynClear(); g_dynWasOn = false; }
            return;
        }
        g_dynWasOn = true;
        ++g_dynFrame;
        const float dyn = std::min(s.shadowBuildingRange, 48.0f);
        for (int& n : g_candN) n = 0;
        if (s.shadowVehicles) ScanPool(CPools::ms_pVehiclePool, kVehicle, cam, dyn * dyn, CGame::currArea);
        if (s.shadowPeds) ScanPool(CPools::ms_pPedPool, kPed, cam, dyn * dyn, CGame::currArea);
        for (int k = kVehicle; k <= kPed; ++k) {
            Cand* cs = g_cands[k];
            const int n = g_candN[k];
            std::sort(cs, cs + n, [](const Cand& a, const Cand& b) { return a.d2 < b.d2; });
            const int lim = k == kVehicle ? kDynVeh : kDynPed;
            for (int i = 0; i < n && i < lim; ++i) {
                CEntity* e = cs[i].e;
                const CVector origin = e->GetPosition();
                if (!Finite3(origin.x, origin.y, origin.z)) continue;
                const uint32_t key = ((static_cast<uint32_t>(k) + 1u) << 28) | ((static_cast<uint32_t>(cs[i].index) & 0xFFFFFu) << 8) | cs[i].id;
                DynTrack* t = nullptr;
                for (DynTrack& d : g_dyn) if (d.used && d.key == key) { t = &d; break; }
                if (!t) {
                    for (DynTrack& d : g_dyn) if (!d.used) { t = &d; break; }
                    if (!t) continue;
                    *t = DynTrack{};
                    t->used = true; t->key = key; t->seen = -2;           // -2: needs a first build
                }
                const bool first = t->seen == -2;
                t->seen = g_dynFrame;
                const Axes ax = GetAxes(e);
                const float v[9] = {ax.r.x, ax.r.y, ax.r.z, ax.f.x, ax.f.y, ax.f.z, ax.u.x, ax.u.y, ax.u.z};
                bool moved = first || std::fabs(origin.x - t->origin.x) + std::fabs(origin.y - t->origin.y) + std::fabs(origin.z - t->origin.z) > 0.02f;
                for (int a = 0; a < 3 && !moved; ++a)
                    if (t->axes[a * 3] * v[a * 3] + t->axes[a * 3 + 1] * v[a * 3 + 1] + t->axes[a * 3 + 2] * v[a * 3 + 2] < 0.9998f) moved = true;
                if (!moved) continue;
                g_mv.clear(); g_mi.clear();
                bool ok = true;
                if (k == kPed) DynPedMesh(e); else ok = DynVehicleMesh(e);
                if (!ok || g_mi.size() < 12) { t->seen = g_dynFrame; continue; }
                const double o[3] = {origin.x, origin.y, origin.z};
                ScreenShadow::CasterSet(key, o, g_mv.data(), static_cast<int>(g_mv.size() / 3), g_mi.data(), static_cast<int>(g_mi.size()));
                t->origin = origin;
                for (int a = 0; a < 9; ++a) t->axes[a] = v[a];
            }
        }
        for (DynTrack& d : g_dyn)
            if (d.used && d.seen != g_dynFrame) { ScreenShadow::CasterRemove(d.key); d = DynTrack{}; }
    }

    int MeshStat(int what) {
        int n = 0;
        if (what == 3) { for (const DynTrack& d : g_dyn) if (d.used) ++n; return n; }
        for (const MeshTrack& t : g_meshTracks) {
            if (!t.used) continue;
            if (what == 0 && t.built && t.fromDff) ++n;
            else if (what == 1 && t.built && !t.fromDff) ++n;
            else if (what == 2 && t.failed) ++n;
        }
        return n;
    }

    // ---------------------------------------------------------------- sun for the screen shadows
    // Optional: the game's own sun direction (CTimeCycle). Validated; the clock model is used otherwise.
    bool ReadGameSun(float out[3]) {
        static float* sym = nullptr; static bool looked = false;
        if (!looked) {
            looked = true;
            sym = GetMainLibrarySymbol<float*>("_ZN10CTimeCycle20m_vecDirnLightToSunE");
            logger->Info("ComplexShadow: CTimeCycle::m_vecDirnLightToSun %s", sym ? "found" : "missing (clock sun used)");
        }
        if (!sym) return false;
        const float x = sym[0], y = sym[1], z = sym[2];
        if (!Finite3(x, y, z)) return false;
        const float l = std::sqrt(x * x + y * y + z * z);
        if (!(l > 0.5f && l < 1.5f) || z / l < 0.15f) return false;
        out[0] = x / l; out[1] = y / l; out[2] = z / l;
        return true;
    }

    void PublishSunToGL(bool daylight, const VehicleLightGL::Settings& s) {
        const CVector cam = TheCamera.GetPosition();
        const float camWorld[3] = {cam.x, cam.y, cam.z};
        if (!daylight || !Finite3(cam.x, cam.y, cam.z)) { VehicleLightGL::PublishSun(nullptr, 0.0f, camWorld); return; }
        float toSun[3];
        ShadowMap::SunVector(g_sun.dirX, g_sun.dirY, g_sun.elevation, toSun);
        static uint32_t lastLog = 0;
        float game[3];
        const bool haveGame = ReadGameSun(game);
        const uint32_t now = CTimer::GetTimeMS();
        if (now - lastLog > 15000) {
            lastLog = now;
            logger->Info("ComplexShadow: sun clock=(%.2f %.2f %.2f) game=%s(%.2f %.2f %.2f) source=%d strength=%.2f time=%02d:%02d",
                         toSun[0], toSun[1], toSun[2], haveGame ? "ok" : "n/a", haveGame ? game[0] : 0.0f, haveGame ? game[1] : 0.0f,
                         haveGame ? game[2] : 0.0f, s.sunSource, g_sun.strength,
                         static_cast<int>(CClock::ms_nGameClockHours), static_cast<int>(CClock::ms_nGameClockMinutes));
        }
        if (s.sunSource == 1 && haveGame) { toSun[0] = game[0]; toSun[1] = game[1]; toSun[2] = std::max(game[2], std::sin(ShadowMap::kMinSunElevation)); ShadowMap::Normalize3(toSun); }
        VehicleLightGL::PublishSun(toSun, g_sun.strength, camWorld);
    }

    // Once per game frame: sun, scan, fades, mask regeneration.
    void UpdateFrame() {
        const uint32_t frame = CTimer::m_FrameCounter;
        if (frame == g_lastFrame) return;
        g_lastFrame = frame;
        g_regenThisFrame = 0; g_trisThisFrame = 0;

        const VehicleLightGL::Settings& s = VehicleLightGL::GetSettings();
        const uint32_t now = CTimer::GetTimeMS();
        const float dt = Clamp(static_cast<float>(now - g_lastMs) * 0.001f, 0.0f, 0.1f);
        g_lastMs = now;

        const bool daylight = s.shadowBuilding && CGame::currArea == 0 && ComputeSun(g_sun);
        PublishSunToGL(daylight, s);
        // The screen-built sun shadows (every caster, every receiver) replace the ground decals while they run.
        // The decal silhouette pass and the screen-depth sun pass are
        // exclusive. Mixing them produces double shadows and can alternate
        // when the GL pass misses a frame.
        // The shader receiver is the high-quality interactive mode. It sees
        // every visible GTA surface and must replace the ground decal pass;
        // otherwise the same sun shadow is multiplied twice on the road.
        g_sunOn = daylight && s.shadowBuilding && !VehicleLightGL::SunShadowRequested() &&
                  s.shadowBuildingStrength > 0.0f;
        g_lastStrength = daylight ? g_sun.strength : 0.0f;
        const int orient = s.shadowOrient & 7;
        const CVector cam = TheCamera.GetPosition();

        UpdateMeshCasters(s, cam, daylight && VehicleLightGL::SunShadowRequested() && s.sunShadowMesh != 0 &&
                                      s.shadowBuildingStrength > 0.0f);
        UpdateDynamicCasters(s, cam, daylight && VehicleLightGL::SunShadowRequested() && s.sunShadowMesh != 0 &&
                                         s.shadowBuildingStrength > 0.0f);
        if (g_sunOn && --g_scanCountdown <= 0) { g_scanCountdown = kScanInterval; Scan(s, cam, CGame::currArea); }
        if (!g_sunOn) for (Track& t : g_tracks) t.selected = false;

        const float range = s.shadowBuildingRange;
        // Tracks needing regeneration: lacking a mask first, then nearest.
        for (Track& t : g_tracks) {
            if (!t.used) continue;
            CEntity* e = Resolve(t);
            bool valid = e != nullptr;
            float dist = t.lastDist;
            if (valid) {
                const CVector& p = e->GetPosition();
                if (!Finite3(p.x, p.y, p.z)) valid = false;
                else { t.lastOrigin = p; const float dx = p.x - cam.x, dy = p.y - cam.y; dist = std::sqrt(dx * dx + dy * dy); t.lastDist = dist; }
            }
            const bool keep = t.selected && valid && g_sunOn;
            const float fade = 1.0f - Smooth(range * 0.65f, range, dist);
            const float target = keep ? fade : 0.0f;
            const float tau = target > t.alpha ? 0.22f : 0.55f;
            t.alpha += (target - t.alpha) * (dt > 0.0f ? 1.0f - std::exp(-dt / tau) : 0.0f);
            if (keep && g_regenThisFrame < kRegenPerFrame && g_trisThisFrame < kTriBudgetPerFrame) {
                const Axes ax = GetAxes(e);
                if (NeedsRegen(t, g_sun, ax, orient)) {
                    if (!Regenerate(t, e, g_sun, orient)) {
                        ++g_regenFails;
                        if (g_logBudget > 0) {
                            --g_logBudget;
                            ShadowLog("mask regeneration failed: kind=%d index=%d model=%u hasMask=%d slots=%d",
                                      static_cast<int>(t.kind), t.index, static_cast<unsigned>(t.model), t.hasMask ? 1 : 0,
                                      t.slot >= 0 ? 1 : 0);
                        }
                        if (!t.hasMask) t.selected = false;   // unusable geometry: drop
                    }
                }
            }
            if (!t.hasMask && t.alpha < 0.02f && !keep) { ReleaseSlot(t); t.used = false; continue; }
            if (t.alpha < 0.01f && target <= 0.0f) { ReleaseSlot(t); t.used = false; }
        }
    }

    bool OursPresent() {
        const int count = CShadows::ShadowsStoredToBeRendered;
        for (int i = 0; i < count && i < 64; ++i) {
            const RwTexture* tex = CShadows::asShadowsStored[i].m_pTexture;
            if (!tex) continue;
            for (const Slot& s : g_slots) if (s.tex == tex) return true;
        }
        return false;
    }

    void StoreShadows() {
        g_lastStored = 0;
        if (!g_sunOn) return;
        const VehicleLightGL::Settings& s = VehicleLightGL::GetSettings();
        if (OursPresent()) return;                       // already stored for this render
        // stable order: track array order; fully visible casters first so they never lose a slot
        for (int pass = 0; pass < 2; ++pass)
            for (Track& t : g_tracks) {
                if (!t.used || !t.hasMask || t.slot < 0) continue;
                const bool full = t.alpha > 0.95f && t.selected;
                if ((pass == 0) != full) continue;
                if (CShadows::ShadowsStoredToBeRendered >= kStoredLimit) return;
                const float brightness = kBaseBrightness * s.shadowBuildingStrength * g_sun.strength * t.alpha * KindStrength(t.kind, s);
                if (brightness < 3.0f) continue;
                // Keep the placement vector in lockstep with the mask. Using
                // the current sun direction with a cached mask makes the
                // silhouette slide and appear to flicker between regenerations.
                const float dX = t.genDirX, dY = t.genDirY, nX = dY, nY = -dX;
                CVector centre(t.lastOrigin.x + dX * t.quad.aC + nX * t.quad.cC,
                               t.lastOrigin.y + dY * t.quad.aC + nY * t.quad.cC,
                               t.lastOrigin.z + t.zRef + 0.3f);
                CShadows::StoreShadowToBeRendered(SHADOW_DEFAULT, g_slots[t.slot].tex, &centre,
                                                  dX * t.quad.halfA, dY * t.quad.halfA, nX * t.quad.halfC, nY * t.quad.halfC,
                                                  static_cast<int16_t>(std::min(brightness, 255.0f)), 0, 0, 0,
                                                  15.0f, false, 1.0f, nullptr, false);
                ++g_lastStored;
            }
    }

    void Summary() {
        const uint32_t now = CTimer::GetTimeMS();
        if (now - g_lastSummary < 5000) return;
        g_lastSummary = now;
        int live = 0, slots = 0;
        for (const Track& t : g_tracks) live += t.used; for (const Slot& sl : g_slots) slots += sl.used;
        const VehicleLightGL::Settings& s = VehicleLightGL::GetSettings();
        ShadowLog("summary: mode=%s requested=%d shaderActive=%d build=%d storedHook=%d staticHook=%d cands=%d tracks=%d slots=%d stored=%d regenFail=%u nativeBlocked=%u renderCalls=%u staticCalls=%u staticSkipped=%u sun=%.2f strength=%.2f range=%.1f time=%02d:%02d meshCasters=%d meshDff=%d meshCol=%d meshFail=%d dyn=%d",
                  VehicleLightGL::SunShadowRequested() ? "shader-only" : (g_sunOn ? "complex-decal" : "off"),
                  VehicleLightGL::SunShadowRequested() ? 1 : 0, VehicleLightGL::IsSunShadowActive() ? 1 : 0,
                  s.shadowBuilding ? 1 : 0, g_hookInstalled ? 1 : 0, g_staticHookInstalled ? 1 : 0,
                  g_lastCands, live, slots, g_lastStored, g_regenFails, g_nativeSuppressed, g_renderCalls,
                  g_staticCalls, g_staticSkipped, g_lastStrength, s.shadowBuildingStrength, s.shadowBuildingRange,
                  static_cast<int>(CClock::ms_nGameClockHours), static_cast<int>(CClock::ms_nGameClockMinutes), ScreenShadow::CasterCount(), MeshStat(0), MeshStat(1), MeshStat(2), MeshStat(3));
    }

    // Runs right before the engine renders the stored shadows. No per-frame guard on the store step:
    // if the engine clears its list between its two RenderStoredShadows calls we simply store again.
    void Run() { UpdateFrame(); StoreShadows(); Summary(); }
    void HookRenderStoredShadows(bool additive) {
        if (g_inStoredHook) return;
        g_inStoredHook = true;
        ++g_renderCalls;
        Run();
        if (ShaderShadowRequested()) {
            if (VehicleLightGL::IsSunShadowActive()) {
                const int count = std::max(0, std::min(48, static_cast<int>(CShadows::ShadowsStoredToBeRendered)));
                for (int i = 0; i < count; ++i) CShadows::asShadowsStored[i].m_nIntensity = 0;
            }
            if (g_originalRender) g_originalRender(additive);
            if (VehicleLightGL::IsSunShadowActive()) {
                CShadows::ShadowsStoredToBeRendered = 0;
            }
            g_inStoredHook = false;
            return;
        }
        if (g_originalRender) g_originalRender(additive);
        g_inStoredHook = false;
    }
    void HookRenderStaticShadows(bool additive) {
        if (g_inStaticHook) return;
        g_inStaticHook = true;
        ++g_staticCalls;
        // Static/permanent GTA shadow meshes are also disabled in shader-only
        // mode, but the original renderer must still run its bookkeeping and
        // cleanup. Temporarily mute each registered static shadow instead of
        // returning before RenderWare resets its state.
        if (ShaderShadowRequested()) {
            ++g_staticSkipped;
            uint16_t saved[48]{};
            for (int i = 0; i < 48; ++i) {
                saved[i] = CShadows::aStaticShadows[i].m_nIntensity;
                CShadows::aStaticShadows[i].m_nIntensity = 0;
            }
            if (g_originalStaticRender) g_originalStaticRender(additive);
            for (int i = 0; i < 48; ++i) CShadows::aStaticShadows[i].m_nIntensity = saved[i];
            g_inStaticHook = false;
            return;
        }
        if (g_originalStaticRender) g_originalStaticRender(additive);
        g_inStaticHook = false;
    }
}

void Initialize() {
    OpenShadowLog();
    ShadowLog("settings at init: shaderRequested=%d shadowBuilding=%d range=%.1f strength=%.2f dff=%d",
              VehicleLightGL::SunShadowRequested() ? 1 : 0,
              VehicleLightGL::GetSettings().shadowBuilding,
              VehicleLightGL::GetSettings().shadowBuildingRange,
              VehicleLightGL::GetSettings().shadowBuildingStrength,
              VehicleLightGL::GetSettings().shadowDff);
    void* address = GetMainLibrarySymbol<void*>("_ZN8CShadows19RenderStoredShadowsEb");
    g_hookInstalled = address && aml &&
        aml->Hook(address, reinterpret_cast<void*>(HookRenderStoredShadows), reinterpret_cast<void**>(&g_originalRender));
    if (!g_hookInstalled) Events::renderEffectsEvent.before += []() { Run(); };
    // Keep the sun snapshot alive even on builds where RenderStoredShadows is
    // skipped/renamed. UpdateFrame is frame-guarded, so this does not duplicate
    // work when the stored-shadow hook also fires.
    Events::gameProcessEvent.after += []() { UpdateFrame(); };
    {
        void* staticAddress = GetMainLibrarySymbol<void*>("_ZN8CShadows19RenderStaticShadowsEb");
        g_staticHookInstalled = staticAddress && aml &&
            aml->Hook(staticAddress, reinterpret_cast<void*>(HookRenderStaticShadows), reinterpret_cast<void**>(&g_originalStaticRender));
        void* v = GetMainLibrarySymbol<void*>("_ZN8CShadows21StoreShadowForVehicleEP8CVehicle12VEH_SHD_TYPE");
        void* p = GetMainLibrarySymbol<void*>("_ZN8CShadows23StoreShadowForPedObjectEP7CEntityffffff");
        void* o = GetMainLibrarySymbol<void*>("_ZN8CShadows18StoreShadowForPoleEP7CEntityfffffj");
        const bool hv = v && aml && aml->Hook(v, reinterpret_cast<void*>(H_StoreShadowForVehicle), reinterpret_cast<void**>(&g_origVehicle));
        const bool hp = p && aml && aml->Hook(p, reinterpret_cast<void*>(H_StoreShadowForPedObject), reinterpret_cast<void**>(&g_origPed));
        const bool ho = o && aml && aml->Hook(o, reinterpret_cast<void*>(H_StoreShadowForPole), reinterpret_cast<void**>(&g_origPole));
        logger->Info("ComplexShadow: native shadow hooks static=%d stored=%d vehicle=%d ped=%d pole=%d shaderOnly=%d",
                     g_staticHookInstalled ? 1 : 0, g_hookInstalled ? 1 : 0, hv ? 1 : 0, hp ? 1 : 0, ho ? 1 : 0,
                     VehicleLightGL::SunShadowRequested() ? 1 : 0);
        ShadowLog("hooks: static=%d stored=%d vehicle=%d ped=%d pole=%d shaderOnly=%d",
                  g_staticHookInstalled ? 1 : 0, g_hookInstalled ? 1 : 0, hv ? 1 : 0, hp ? 1 : 0, ho ? 1 : 0,
                  VehicleLightGL::SunShadowRequested() ? 1 : 0);
    }
    logger->Info("ComplexShadow initialized (DFF silhouette shadows, RenderStoredShadows hook=%s).",
                 g_hookInstalled ? "OK" : (address ? "FAILED, using renderEffects fallback" : "missing, using renderEffects fallback"));
    ShadowLog("initialized: storedHook=%s staticHook=%s fallbackRenderEffects=%d",
              g_hookInstalled ? "OK" : "FAILED", g_staticHookInstalled ? "OK" : "FAILED", g_hookInstalled ? 0 : 1);
}
}
