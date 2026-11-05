#include "dynamic_vehicle_light_system.h"
#include "vehicle_light_gl.h"

#include <mod/amlmod.h>
#include <mod/logger.h>

#include <aml-psdk/game_sa/Events.h>
#include <aml-psdk/game_sa/base/ColModel.h>
#include <aml-psdk/game_sa/base/Timer.h>
#include <aml-psdk/game_sa/engine/PointLights.h>
#include <aml-psdk/gta_base/Rect.h>
#include <aml-psdk/gta_base/RGBA.h>
#include <aml-psdk/game_sa/engine/Sprite.h>
#include <aml-psdk/game_sa/engine/World.h>
#include <aml-psdk/game_sa/entity/Camera.h>
#include <aml-psdk/game_sa/entity/Automobile.h>
#include <aml-psdk/game_sa/entity/PlayerPed.h>
#include <aml-psdk/game_sa/entity/Vehicle.h>
#include <aml-psdk/game_sa/other/Clock.h>
#include <aml-psdk/renderware/RwImage.h>
#include <aml-psdk/renderware/RwCamera.h>
#include <aml-psdk/renderware/RwRaster.h>
#include <aml-psdk/renderware/RwRender.h>
#include <aml-psdk/renderware/RwTexture.h>
#include <aml-psdk/gta_base/DrawVertices.h>

#include "vehicle_light_beam_core.h"

#include <android/log.h>

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <ctime>

namespace DynamicVehicleLightSystem {
namespace {
    constexpr float kMaxSourceDistance = 55.0f;
    constexpr float kHeadlightRange = 34.0f;
    constexpr float kReflectRange = 32.0f;
    constexpr float kRayStepPastSelf = 0.12f;
    constexpr float kEndpointTolerance = 0.24f;
    constexpr float kGateConeCos = 0.35f;
    constexpr float kGroundNormalZ = 0.70f;
    constexpr float kMinHitDistance = 1.2f;
    constexpr float kNightMinimum = 0.10f;
    constexpr int kMaxVehiclesPerFrame = 4;
    // GTA SA 2.10 AddLight accepts NumLights < 32 (valid slots 0..31).
    // Reserve eight slots for two native surface-bounce lights on each of four cars.
    constexpr int kMaxPointLights = 32;
    constexpr int kMaxHeadlightLights = 24;
    constexpr int kSurfaceLightsPerVehicle = 2;
    constexpr int kMaxTrackedLights = kMaxVehiclesPerFrame * 4;
    constexpr int kMaxStates = 8;             // was 6: more lit cars than states made states thrash and restart their fade
    constexpr int kMaxRegisteredPerFrame = 5; // candidates per frame; only kMaxVehiclesPerFrame of them are lit (best by priority)
    constexpr unsigned kHoldFrames = 3;       // keep a vehicle's lights at full strength this long after its last registration
    constexpr unsigned kFadeFrames = 10;      // ...then fade them out over this many frames instead of cutting them
    constexpr int kYawCount = 5;
    constexpr int kPitchCount = 1;
    constexpr int kPatchesPerLamp = kYawCount * kPitchCount;
    constexpr int kMaxPatches = 2 * kPatchesPerLamp;
    constexpr uint32_t kUpdateIntervalMs = 80;
    constexpr uint32_t kSummaryIntervalMs = 3000;
    // Broader horizontal sampling spreads wall hits across the beam; the mild
    // downward pitch keeps the added reach from shooting above close targets.
    constexpr float kYawDeg[kYawCount] = {-18.0f, -9.0f, 0.0f, 9.0f, 18.0f};
    constexpr float kPitchDeg[kPitchCount] = {-3.5f};
    // The old collision bounce point lights could create a second, misplaced
    // light/shadow footprint around nearby geometry, so keep them disabled.
    constexpr bool kBounceLight = false;

    struct Stats {
        unsigned spots = 0, updates = 0, rays = 0, hits = 0;
        unsigned surfaceAttempts = 0, surfaceLights = 0, surfaceCulled = 0, surfacePoolFull = 0;
        unsigned gated = 0, gateRays = 0, poolsSeen = 0, poolsBlocked = 0;
    } g_stats;
    constexpr int kMaxShaderSpotLights = 8;
    constexpr int kMaxShaderRearLights = VehicleLightGL::kMaxLights - kMaxShaderSpotLights;
    // Spots and rear glows are kept in separate arrays and packed contiguously on publish, so
    // the shader never loops over empty gap entries and a later car can never lose its lamps.
    bool g_firstShaderLightLogged = false;
    bool g_firstShaderPublishLogged = false;

    FILE* g_log = nullptr;
    long long g_logStartMs = 0;
    uint32_t g_lastSummary = 0;
    bool g_firstLightLogged = false;
    bool g_firstHitLogged = false;
    bool g_firstSurfaceLightLogged = false;

    long long NowMs() {
        timespec ts{};
        clock_gettime(CLOCK_MONOTONIC, &ts);
        return static_cast<long long>(ts.tv_sec) * 1000LL + ts.tv_nsec / 1000000LL;
    }

    void Log(const char* fmt, ...) {
        char buffer[700];
        va_list args;
        va_start(args, fmt);
        vsnprintf(buffer, sizeof(buffer), fmt, args);
        va_end(args);
        __android_log_write(ANDROID_LOG_INFO, "VehicleLight", buffer);
        if (g_log) {
            fprintf(g_log, "[%8lld ms] %s\n", NowMs() - g_logStartMs, buffer);
            fflush(g_log);
        }
    }

    void OpenLog() {
        char path[600];
        const char* base = aml ? aml->GetAndroidDataPath() : nullptr;
        if (base && base[0]) snprintf(path, sizeof(path), "%s/VehicleLight.log", base);
        else snprintf(path, sizeof(path), "/storage/emulated/0/Android/data/com.rockstargames.gtasa/files/VehicleLight.log");
        g_log = fopen(path, "w");
        g_logStartMs = NowMs();
        if (g_log) Log("Vehicle light diagnostics: %s", path);
        else __android_log_print(ANDROID_LOG_ERROR, "VehicleLight", "cannot open log: %s", path);
    }

    bool IsFinite(const CVector& v) {
        return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
    }
    float LengthSquared(const CVector& v) { return v.x * v.x + v.y * v.y + v.z * v.z; }
    bool Normalize(CVector& v) {
        const float l2 = LengthSquared(v);
        if (!std::isfinite(l2) || l2 < 1.0e-8f) return false;
        const float inv = 1.0f / std::sqrt(l2);
        v.x *= inv; v.y *= inv; v.z *= inv;
        return true;
    }
    float Clamp01(float x) { return std::max(0.0f, std::min(1.0f, x)); }
    float Lerp(float a, float b, float t) { return a + (b - a) * t; }
    float SmoothStep(float a, float b, float x) {
        const float t = Clamp01((x - a) / (b - a));
        return t * t * (3.0f - 2.0f * t);
    }
    RwV3d ToRw(const CVector& v) { return {v.x, v.y, v.z}; }
    CVector TransformOffset(const CMatrix& m, float x, float y, float z) {
        const CVector& r = m.GetRight();
        const CVector& f = m.GetForward();
        const CVector& u = m.GetUp();
        const CVector& o = m.GetPosition();
        return {o.x + r.x*x + f.x*y + u.x*z,
                o.y + r.y*x + f.y*y + u.y*z,
                o.z + r.z*x + f.z*y + u.z*z};
    }
    float NightFactor() {
        const float hour = static_cast<float>(CClock::ms_nGameClockHours) +
                           static_cast<float>(CClock::ms_nGameClockMinutes) / 60.0f;
        float night;
        if (hour >= 20.0f || hour < 5.0f) night = 1.0f;
        else if (hour < 7.5f) night = 1.0f - SmoothStep(5.0f, 7.5f, hour);
        else if (hour < 17.5f) night = 0.0f;
        else night = SmoothStep(17.5f, 20.0f, hour);
        return Lerp(kNightMinimum, 1.0f, night);
    }

    // Continue through the source vehicle and retain collisions even without an entity pointer.
    bool CastRay(const CVector& start, const CVector& end, CEntity* ignore,
                 CColPoint& outPoint, CEntity*& outEntity) {
        CVector delta(end.x-start.x, end.y-start.y, end.z-start.z);
        const float length = std::sqrt(LengthSquared(delta));
        if (!std::isfinite(length) || length < 0.05f) return false;
        const CVector dir(delta.x/length, delta.y/length, delta.z/length);
        CVector from = start;
        for (int retry = 0; retry < 4; ++retry) {
            CColPoint cp{};
            CEntity* entity = nullptr;
            const bool hit = CWorld::ProcessLineOfSight(from, end, cp, entity,
                true, true, false, true, false, false, false, false);
            ++g_stats.rays;
            if (!hit) return false;
            if (entity == ignore) {
                from = {cp.m_vecPoint.x + dir.x*kRayStepPastSelf,
                        cp.m_vecPoint.y + dir.y*kRayStepPastSelf,
                        cp.m_vecPoint.z + dir.z*kRayStepPastSelf};
                if (LengthSquared(CVector(end.x-from.x, end.y-from.y, end.z-from.z)) < 0.04f) return false;
                continue;
            }
            outPoint = cp;
            outEntity = entity;
            return true;
        }
        return false;
    }

    CVector Cross3(const CVector& a, const CVector& b) {
        return {a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x};
    }
    // Vehicle-local geometry is kept so each lamp can describe the source vehicle's box in its own
    // frame (the shader must never let the source vehicle shadow or hide its own light).
    struct LampSet {
        CVector pos[2]; int count = 0; CVector dir, right, up, lampUp, fwd, rearPos;
        float groundH = 0.7f, halfSep = 0.0f;
        float frontY = 2.1f, rearY = -2.1f, lampZ = 0.4f;
        float boxX0 = -1.0f, boxX1 = 1.0f, boxY0 = -2.4f, boxY1 = 2.4f, boxZ0 = -0.05f, boxZ1 = 1.4f;
        float lateral = 0.0f;
    };
    bool ComputeLamps(CVehicle* vehicle, LampSet& lamps) {
        CMatrix* matrix = vehicle->GetMatrix();
        if (!matrix) return false;
        CVector forward = matrix->GetForward();
        lamps.right = matrix->GetRight();
        lamps.up = matrix->GetUp();
        if (!Normalize(forward) || !Normalize(lamps.right) || !Normalize(lamps.up)) return false;
        float halfWidth = 0.9f, front = 2.3f, lampZ = 0.4f, rear = -2.3f, groundH = 0.7f;
        float minZ = lampZ - 0.45f, maxZ = lampZ + 1.0f;
        if (CColModel* col = vehicle->GetColModel()) {
            const CVector& mn = col->m_boundBox.m_vecMin;
            const CVector& mx = col->m_boundBox.m_vecMax;
            if (std::isfinite(mn.x) && std::isfinite(mx.x) && mx.y > 0.5f && mx.y < 14.0f &&
                mx.x > 0.2f && mx.x < 4.0f && mx.z > mn.z) {
                halfWidth = mx.x;
                front = mx.y;
                lampZ = mn.z + std::min(0.95f, std::max(0.45f, 0.30f*(mx.z-mn.z)));
                groundH = lampZ - mn.z;
                minZ = mn.z; maxZ = mx.z;
                if (mn.y < -0.5f && mn.y > -14.0f) rear = mn.y;
            }
        }
        const bool bike = vehicle->IsSubBike();
        const float lateral = bike ? 0.0f : halfWidth*0.62f;
        lamps.count = bike ? 1 : 2;
        const float frontY = front*0.93f;
        lamps.pos[0] = TransformOffset(*matrix, -lateral, frontY, lampZ);
        lamps.pos[1] = TransformOffset(*matrix,  lateral, frontY, lampZ);
        const float tilt = 2.0f*0.0174533f;
        lamps.fwd = forward; lamps.groundH = groundH; lamps.halfSep = lateral; lamps.lateral = lateral;
        lamps.frontY = frontY; lamps.rearY = rear*0.93f; lamps.lampZ = lampZ;
        lamps.rearPos = TransformOffset(*matrix, 0.0f, rear*0.93f, lampZ);
        lamps.dir = {forward.x*std::cos(tilt)-lamps.up.x*std::sin(tilt),
                     forward.y*std::cos(tilt)-lamps.up.y*std::sin(tilt),
                     forward.z*std::cos(tilt)-lamps.up.z*std::sin(tilt)};
        if (!Normalize(lamps.dir)) return false;
        // Orthonormal lamp frame (right, dir, lampUp): right x dir = up in GTA's right-handed space.
        lamps.lampUp = Cross3(lamps.right, lamps.dir);
        if (!Normalize(lamps.lampUp)) return false;
        // Source-vehicle box (vehicle space), slightly inflated. Bikes get headroom for the rider.
        lamps.boxX0 = -(halfWidth + 0.10f); lamps.boxX1 = halfWidth + 0.10f;
        lamps.boxY0 = rear - 0.12f;         lamps.boxY1 = front + 0.14f;
        lamps.boxZ0 = minZ - 0.06f;         lamps.boxZ1 = maxZ + (bike ? 1.15f : 0.10f);
        return IsFinite(lamps.pos[0]) && IsFinite(lamps.pos[1]);
    }

    struct Patch { bool has=false; CVector pos, normal, rayDir; CEntity* entity=nullptr; float distance=0.0f, target=0.0f, current=0.0f; };
    struct VehicleState { CVehicle* vehicle=nullptr; unsigned lastSeenFrame=0; uint32_t lastUpdate=0; Patch patches[kMaxPatches];
        BeamCore::Beam front, rear; uint32_t beamUpdate=0; unsigned beamFrame=0; bool rearOn=false; bool rearTick=false;
        float lampFade[2]={0.0f,0.0f}; bool brokenLogged=false;
        // Last GLES lights this vehicle produced. They are held/faded when the game skips a frame and are
        // ranked against the other vehicles with hysteresis, so lights never blink or swap abruptly.
        VehicleLightGL::Light glSpot[2]; bool glSpotOn[2]={false,false}; VehicleLightGL::Light glRear; bool glRearOn=false;
        unsigned cacheFrame=0; bool cacheValid=false; float shown=0.0f; bool selected=false; unsigned shownFrame=0xFFFFFFFFu; };
    void UpdateBeams(CVehicle* vehicle, VehicleState& state, const LampSet& lamps, float night);
    VehicleState g_states[kMaxStates];
    VehicleState* GetState(CVehicle* vehicle, unsigned frame) {
        VehicleState* freeSlot=nullptr; VehicleState* oldest=&g_states[0];
        for (VehicleState& state : g_states) {
            if (state.vehicle == vehicle) return &state;
            if ((!state.vehicle || frame-state.lastSeenFrame > 60) && !freeSlot) freeSlot=&state;
            if (state.lastSeenFrame < oldest->lastSeenFrame) oldest=&state;
        }
        VehicleState* state=freeSlot ? freeSlot : oldest;
        *state=VehicleState{}; state->vehicle=vehicle; return state;
    }

    void UpdatePatches(CVehicle* vehicle, VehicleState& state, const LampSet& lamps, float night) {
        ++g_stats.updates;
        for (int lamp=0; lamp<lamps.count; ++lamp) for (int yi=0; yi<kYawCount; ++yi) for (int pi=0; pi<kPitchCount; ++pi) {
            Patch& patch=state.patches[lamp*kPatchesPerLamp+yi*kPitchCount+pi];
            const float yaw=kYawDeg[yi]*0.0174533f, pitch=kPitchDeg[pi]*0.0174533f;
            CVector dir(lamps.dir.x*std::cos(yaw)+lamps.right.x*std::sin(yaw)+lamps.up.x*std::sin(pitch),
                        lamps.dir.y*std::cos(yaw)+lamps.right.y*std::sin(yaw)+lamps.up.y*std::sin(pitch),
                        lamps.dir.z*std::cos(yaw)+lamps.right.z*std::sin(yaw)+lamps.up.z*std::sin(pitch));
            if (!Normalize(dir)) { patch.target=0.0f; continue; }
            const CVector start(lamps.pos[lamp].x+dir.x*0.15f, lamps.pos[lamp].y+dir.y*0.15f, lamps.pos[lamp].z+dir.z*0.15f);
            const CVector end(lamps.pos[lamp].x+dir.x*kReflectRange, lamps.pos[lamp].y+dir.y*kReflectRange, lamps.pos[lamp].z+dir.z*kReflectRange);
            CColPoint cp{}; CEntity* hitEntity=nullptr;
            if (!CastRay(start,end,vehicle,cp,hitEntity)) { patch.target=0.0f; continue; }
            ++g_stats.hits;
            CVector normal=cp.m_vecNormal;
            if (!Normalize(normal)) { patch.target=0.0f; continue; }
            float dot=normal.x*dir.x+normal.y*dir.y+normal.z*dir.z;
            if (dot>0.0f) { normal=CVector(-normal.x,-normal.y,-normal.z); dot=-dot; }
            const CVector offset(cp.m_vecPoint.x-lamps.pos[lamp].x,cp.m_vecPoint.y-lamps.pos[lamp].y,cp.m_vecPoint.z-lamps.pos[lamp].z);
            const float distance=std::sqrt(LengthSquared(offset));
            if (distance<kMinHitDistance || normal.z>kGroundNormalZ) { patch.target=0.0f; continue; }
            const float facing=Clamp01(-dot);
            const float falloff=std::pow(Clamp01(1.0f-distance/kReflectRange),1.6f);
            const float strength=night*std::pow(facing,0.7f)*falloff;
            const CVector hit(cp.m_vecPoint.x+normal.x*0.02f,cp.m_vecPoint.y+normal.y*0.02f,cp.m_vecPoint.z+normal.z*0.02f);
            // Keep the decal on the latest collision point. World-space smoothing
            // can leave it hovering between two surfaces while the car turns.
            patch.pos=hit;
            patch.has=true; patch.normal=normal; patch.rayDir=dir; patch.entity=hitEntity; patch.distance=distance; patch.target=strength;
            if (!g_firstHitLogged) { g_firstHitLogged=true; Log("first reflected patch: distance=%.1f strength=%.2f at=(%.1f,%.1f,%.1f)",distance,strength,hit.x,hit.y,hit.z); }
        }
    }

    unsigned g_frame=0;
    CVehicle* g_registeredVehicles[kMaxRegisteredPerFrame]={};
    int g_registeredCount=0;
    struct TrackedLight { int slot=-1; CEntity* source=nullptr; CVector position,direction; unsigned frame=0; bool surfaceBounce=false; };
    TrackedLight g_trackedLights[kMaxTrackedLights];
    int g_trackedLightCount=0;
    constexpr float kDeg = 0.01745329252f;
    // Source-vehicle box in the local frame of a lamp: x right, y along the lamp axis, z up.
    struct SelfBox { float x0, x1, y0, y1, z0, z1; };
    SelfBox SpotBox(const LampSet& lamps, int lamp) {
        const float lx = lamp == 0 ? -lamps.lateral : lamps.lateral;
        return {lamps.boxX0 - lx, lamps.boxX1 - lx,
                lamps.boxY0 - lamps.frontY, lamps.boxY1 - lamps.frontY,
                lamps.boxZ0 - lamps.lampZ, lamps.boxZ1 - lamps.lampZ};
    }
    SelfBox RearBox(const LampSet& lamps) {
        // Rear glow looks backwards: its forward axis is the vehicle's -forward.
        return {lamps.boxX0, lamps.boxX1,
                -(lamps.boxY1 - lamps.rearY), -(lamps.boxY0 - lamps.rearY),
                lamps.boxZ0 - lamps.lampZ, lamps.boxZ1 - lamps.lampZ};
    }
    void FillSelfBox(VehicleLightGL::Light& light, const SelfBox& b) {
        light.selfBoxXY[0] = b.x0; light.selfBoxXY[1] = b.x1; light.selfBoxXY[2] = b.y0; light.selfBoxXY[3] = b.y1;
        light.selfBoxZ[0] = b.z0; light.selfBoxZ[1] = b.z1;
    }
    void AddShaderSpot(VehicleLightGL::Light& light, const CVector& position, const CVector& direction,
                       const CVector& right, const CVector& up, float range, float gain, const SelfBox& box) {
        light = VehicleLightGL::Light{};
        light.posRange[0] = position.x; light.posRange[1] = position.y; light.posRange[2] = position.z; light.posRange[3] = range;
        light.fwdKind[0] = direction.x; light.fwdKind[1] = direction.y; light.fwdKind[2] = direction.z; light.fwdKind[3] = 1.0f;
        light.rightYaw[0] = right.x; light.rightYaw[1] = right.y; light.rightYaw[2] = right.z;
        light.rightYaw[3] = std::cos(50.0f * kDeg);
        light.upGain[0] = up.x; light.upGain[1] = up.y; light.upGain[2] = up.z; light.upGain[3] = gain;
        light.boxC[0] = position.x; light.boxC[1] = position.y; light.boxC[2] = position.z;
        light.boxC[3] = std::cos(14.0f * kDeg);
        light.boxE[0] = range; light.boxE[1] = range * 0.75f; light.boxE[2] = 3.0f;
        light.boxE[3] = 1.0f; // vehicle headlights cast screen-space shadows
        // Near-white: the shader adds an albedo-independent white lift on top of this tint.
        light.color[0] = 1.0f; light.color[1] = 0.985f; light.color[2] = 0.95f; light.color[3] = 1.0f;
        FillSelfBox(light, box);
        if (!g_firstShaderLightLogged) {
            g_firstShaderLightLogged = true;
            Log("first GLES headlight source: range=%.1f gain=%.2f at=(%.1f,%.1f,%.1f) selfBox x[%.2f,%.2f] y[%.2f,%.2f] z[%.2f,%.2f]",
                range, gain, position.x, position.y, position.z, box.x0, box.x1, box.y0, box.y1, box.z0, box.z1);
        }
    }
    void AddShaderRearGlow(VehicleLightGL::Light& light, const CVector& position, const CVector& backward,
                           const CVector& right, const CVector& up, float gain, const SelfBox& box) {
        light = VehicleLightGL::Light{};
        constexpr float kRange = 8.0f;
        light.posRange[0] = position.x; light.posRange[1] = position.y; light.posRange[2] = position.z; light.posRange[3] = kRange;
        light.fwdKind[0] = backward.x; light.fwdKind[1] = backward.y; light.fwdKind[2] = backward.z; light.fwdKind[3] = 0.0f; // point light
        light.rightYaw[0] = right.x; light.rightYaw[1] = right.y; light.rightYaw[2] = right.z; light.rightYaw[3] = -1.0f;
        light.upGain[0] = up.x; light.upGain[1] = up.y; light.upGain[2] = up.z; light.upGain[3] = gain * 0.45f;
        light.boxC[0] = position.x; light.boxC[1] = position.y; light.boxC[2] = position.z; light.boxC[3] = 1.0f;
        light.boxE[0] = kRange; light.boxE[1] = kRange; light.boxE[2] = 2.0f;
        light.boxE[3] = 0.0f; // tail glow is ambient and should not cast a hard shadow
        light.color[0] = 1.0f; light.color[1] = 0.06f; light.color[2] = 0.03f; light.color[3] = 0.0f; // no air scattering
        FillSelfBox(light, box);
    }

    // Rebuilds the vehicle light list from every vehicle's cached lamps (fresh or held), choosing the
    // best kMaxVehiclesPerFrame by distance with hysteresis and cross-fading the rest in/out.
    void PublishNow() {
        const unsigned frame = CTimer::m_FrameCounter;
        const CVector camera = TheCamera.GetPosition();
        struct Cand { VehicleState* st; float score; float presence; };
        Cand cands[kMaxStates]; int n = 0;
        for (VehicleState& st : g_states) {
            if (!st.vehicle || !st.cacheValid) continue;
            const unsigned age = frame >= st.cacheFrame ? frame - st.cacheFrame : 0;
            if (age > kHoldFrames + kFadeFrames) { st.cacheValid = false; st.shown = 0.0f; st.selected = false; continue; }
            const float presence = age <= kHoldFrames ? 1.0f : 1.0f - static_cast<float>(age - kHoldFrames) / static_cast<float>(kFadeFrames);
            const VehicleLightGL::Light& ref = st.glSpotOn[0] ? st.glSpot[0] : st.glSpotOn[1] ? st.glSpot[1] : st.glRear;
            const float dx = ref.posRange[0] - camera.x, dy = ref.posRange[1] - camera.y, dz = ref.posRange[2] - camera.z;
            const float d2 = dx * dx + dy * dy + dz * dz;
            cands[n++] = {&st, d2 * (st.selected ? 0.55f : 1.0f) / (0.2f + 0.8f * presence), presence};
        }
        std::sort(cands, cands + n, [](const Cand& a, const Cand& b) { return a.score < b.score; });
        for (int i = 0; i < n; ++i) {
            VehicleState& st = *cands[i].st;
            st.selected = i < kMaxVehiclesPerFrame;
            if (st.shownFrame != frame) {
                st.shownFrame = frame;
                st.shown += ((st.selected ? 1.0f : 0.0f) - st.shown) * (st.selected ? 0.35f : 0.18f);
            }
        }
        VehicleLightGL::Light packed[VehicleLightGL::kMaxLights];
        int count = 0, spots = 0, rears = 0;
        for (int i = 0; i < n; ++i) {                                  // spots first (most important)
            const VehicleState& st = *cands[i].st; const float g = st.shown * cands[i].presence;
            if (g < 0.02f) continue;
            for (int k = 0; k < 2; ++k)
                if (st.glSpotOn[k] && spots < kMaxShaderSpotLights && count < VehicleLightGL::kMaxLights) {
                    packed[count] = st.glSpot[k]; packed[count].upGain[3] *= g; ++count; ++spots;
                }
        }
        for (int i = 0; i < n; ++i) {
            const VehicleState& st = *cands[i].st; const float g = st.shown * cands[i].presence;
            if (g < 0.02f) continue;
            if (st.glRearOn && rears < kMaxShaderRearLights && count < VehicleLightGL::kMaxLights) {
                packed[count] = st.glRear; packed[count].upGain[3] *= g; ++count; ++rears;
            }
        }
        if (!g_firstShaderPublishLogged && count > 0) {
            g_firstShaderPublishLogged = true;
            Log("first GLES snapshot publish: lights=%d gameFrame=%u", count, frame);
        }
        const float cameraWorld[3] = {camera.x, camera.y, camera.z};
        VehicleLightGL::PublishVehicle(packed, count, cameraWorld);
    }
    void PublishShaderLights() { PublishNow(); }
    void DisableVehicleState(CVehicle* vehicle) {
        for (VehicleState& state : g_states) {
            if (state.vehicle != vehicle) continue;
            state.glSpotOn[0] = state.glSpotOn[1] = false;
            state.glRearOn = false;
            state.cacheValid = false;
            state.selected = false;
            state.shown = 0.0f;
            state.lampFade[0] = state.lampFade[1] = 0.0f;
            return;
        }
    }
    void BeginFrame() {
        const unsigned frame=CTimer::m_FrameCounter;
        if (frame!=g_frame) {
            g_frame=frame; g_registeredCount=0; g_trackedLightCount=0;
        }
    }
    bool AlreadyRegistered(CVehicle* vehicle) {
        for (int i=0;i<g_registeredCount;++i) if (g_registeredVehicles[i]==vehicle) return true;
        return false;
    }

    bool IsLightVisible(const TrackedLight& light,const CVector& point,CEntity* target) {
        // A null hit entity is common for static world collision. Do not equate
        // null target with null source: that previously bypassed every occlusion
        // test for those receivers and produced misleading light leaks.
        if (!IsFinite(point) || (light.source&&target==light.source)) return true;
        const CVector offset(point.x-light.position.x,point.y-light.position.y,point.z-light.position.z);
        const float d2=LengthSquared(offset);
        if (!std::isfinite(d2)||d2<0.09f||d2>=kHeadlightRange*kHeadlightRange) return true;
        const float distance=std::sqrt(d2);
        const CVector dir(offset.x/distance,offset.y/distance,offset.z/distance);
        if (!light.surfaceBounce &&
            dir.x*light.direction.x+dir.y*light.direction.y+dir.z*light.direction.z<kGateConeCos) return true;
        const float endDistance=std::max(0.0f,distance-kEndpointTolerance);
        if (endDistance<0.1f) return true;
        CVector start(light.position.x+dir.x*0.18f,light.position.y+dir.y*0.18f,light.position.z+dir.z*0.18f);
        const CVector end(light.position.x+dir.x*endDistance,light.position.y+dir.y*endDistance,light.position.z+dir.z*endDistance);
        ++g_stats.gateRays;
        for (int retry=0;retry<4;++retry) {
            CColPoint cp{}; CEntity* entity=nullptr;
            if (!CWorld::ProcessLineOfSight(start,end,cp,entity,true,true,true,true,false,false,false,false)) return true;
            if (!entity) {
                // Static buildings/world geometry can collide without returning
                // an entity pointer. Accept only if the collision is at the
                // intended receiver endpoint; a nearer null collision blocks it.
                const CVector toEnd(cp.m_vecPoint.x-end.x,cp.m_vecPoint.y-end.y,cp.m_vecPoint.z-end.z);
                if (LengthSquared(toEnd)<=0.36f) return true;
                return false;
            }
            if (entity==target) return true;
            if (entity!=light.source) return false;
            start=CVector(cp.m_vecPoint.x+dir.x*kRayStepPastSelf,cp.m_vecPoint.y+dir.y*kRayStepPastSelf,cp.m_vecPoint.z+dir.z*kRayStepPastSelf);
            if (LengthSquared(CVector(end.x-start.x,end.y-start.y,end.z-start.z))<0.01f) return true;
        }
        return false;
    }

    struct GateEntry { int slot; CEntity* entity; CVector point; bool visible; };
    constexpr int kGateCacheSize=96;
    GateEntry g_gateCache[kGateCacheSize]; int g_gateCount=0; unsigned g_gateFrame=0;
    bool GateVisible(const TrackedLight& light,const CVector& point,CEntity* entity) {
        const unsigned frame=CTimer::m_FrameCounter;
        if (frame!=g_gateFrame) { g_gateFrame=frame; g_gateCount=0; }
        for (int i=0;i<g_gateCount;++i) {
            const GateEntry& e=g_gateCache[i];
            if (e.slot==light.slot&&e.entity==entity&&e.point.x==point.x&&e.point.y==point.y&&e.point.z==point.z) return e.visible;
        }
        const bool visible=IsLightVisible(light,point,entity);
        if (g_gateCount<kGateCacheSize) g_gateCache[g_gateCount++]={light.slot,entity,point,visible};
        return visible;
    }

    using GenerateLightsFn=float (*)(const CVector*,float*,CEntity*);
    GenerateLightsFn g_originalGenerateLights=nullptr;
    bool g_lightHookInstalled=false;
    bool g_shadowHookInstalled=false;
    thread_local bool t_inLightHook=false;
    float HookGenerateLightsAffectingObject(const CVector* point,float* totalLighting,CEntity* entity) {
        if (!g_originalGenerateLights) return 0.0f;
        if (t_inLightHook||!point||!totalLighting||!g_trackedLightCount) return g_originalGenerateLights(point,totalLighting,entity);
        t_inLightHook=true;
        struct Saved { int slot; float r,g,b; } saved[kMaxTrackedLights]; int count=0;
        const unsigned frame=CTimer::m_FrameCounter;
        for (int i=0;i<g_trackedLightCount;++i) {
            const TrackedLight& light=g_trackedLights[i];
            if (light.frame!=frame||light.slot<0||light.slot>=CPointLights::NumLights||light.slot>=kMaxPointLights) continue;
            CPointLight& pl=CPointLights::aLights[light.slot];
            if (!GateVisible(light,*point,entity)) {
                saved[count++]={light.slot,pl.m_fColorRed,pl.m_fColorGreen,pl.m_fColorBlue};
                pl.m_fColorRed=pl.m_fColorGreen=pl.m_fColorBlue=0.0f; ++g_stats.gated;
            }
        }
        const float result=g_originalGenerateLights(point,totalLighting,entity);
        for (int i=0;i<count;++i) { CPointLight& pl=CPointLights::aLights[saved[i].slot]; pl.m_fColorRed=saved[i].r; pl.m_fColorGreen=saved[i].g; pl.m_fColorBlue=saved[i].b; }
        t_inLightHook=false;
        return result;
    }

    constexpr char kBeamTexName[] = "beta_vehicle_beam";
    using Im3DRenderIndexedPrimitiveFn = RwBool (*)(RwPrimitiveType, RwImVertexIndex*, RwInt32);
    Im3DRenderIndexedPrimitiveFn g_im3d = nullptr;
    RwRaster* g_beamRaster = nullptr;
    RwTexture* g_beamTexture = nullptr;
    uint32_t g_lastBeamTexTry = 0;
    unsigned g_beamUpdateFrame = 0xFFFFFFFFu;
    int g_beamUpdatesThisFrame = 0;
    unsigned g_lastBeamRenderFrame = 0xFFFFFFFFu;
    unsigned g_beamDrawn = 0;
    bool g_beamLogged = false;
    constexpr float kBeamOpacity = 0.62f;     // brighter fallback, still soft at the edge
    constexpr uint32_t kBeamUpdateMs = 100;
    constexpr float kBeamSmooth = 0.30f;
    constexpr bool kRearGlow = true;
    constexpr int kBeamUpdatesPerFrame = 2;

    bool BeamReady() { return g_im3d && g_beamTexture && g_beamTexture->raster; }
    bool IsBeamOwned(CVehicle* car) {
        // Once the depth-light interceptor is installed, never mix its
        // textured fullscreen result with the old Im3D quad fallback. The
        // fallback could flash as visible mesh polygons during a missed or
        // late pass and also doubled the light cost.
        if (VehicleLightGL::IsInstalled()) return true;
        if (!BeamReady() || !car) return false;
        const unsigned frame = CTimer::m_FrameCounter;
        for (const VehicleState& st : g_states)
            if (st.vehicle == car && st.front.built && frame - st.beamFrame <= 4u && st.front.strength > 0.0f) return true;
        return false;
    }

    // GTA stores the four lamp damage states in CDamageManager (2 bits each, 0 = intact,
    // non-zero = smashed). Each lamp is tested on its own: a broken left lamp never disables
    // its healthy right mate and vice versa. Layout is validated before use so a wrong
    // struct offset can never silently switch the lights off.
    enum LampId { kFrontLeft = 0, kFrontRight = 1, kRearLeft = 2, kRearRight = 3 };
    bool HasDamageManager(CVehicle* vehicle) {
        return vehicle->IsSubAutomobile() || vehicle->IsSubMonsterTruck() ||
               vehicle->IsSubQuad() || vehicle->IsSubTrailer();
    }
    bool DamageManagerPlausible(const CDamageManager& dm) {
        if (!std::isfinite(dm.m_fWheelDamageEffect) || dm.m_fWheelDamageEffect < -0.01f || dm.m_fWheelDamageEffect > 8.0f) return false;
        for (int i = 0; i < 4; ++i) if (dm.m_anWheelsStatus[i] > 3) return false;
        for (int i = 0; i < 6; ++i) if (dm.m_anDoorsStatus[i] > 4) return false;
        return (dm.m_nLightsStatus & ~0xFFu) == 0;     // only the low 8 bits carry lamp state
    }
    int g_damageLogBudget = 10;
    bool IsLampWorking(CVehicle* vehicle, int lamp) {
        if (!vehicle || lamp < 0 || lamp > 3) return false;
        if (!HasDamageManager(vehicle)) return true;
        CAutomobile* automobile = vehicle->AsAutomobile();
        if (!automobile) return true;
        const CDamageManager& dm = automobile->m_damageManager;
        if (!DamageManagerPlausible(dm)) {
            if (g_damageLogBudget > 0) {
                --g_damageLogBudget;
                Log("damage manager layout implausible for vehicle %p (lights=0x%x wheelFx=%.2f); assuming lamps intact",
                    (void*)vehicle, dm.m_nLightsStatus, dm.m_fWheelDamageEffect);
            }
            return true;
        }
        unsigned status = 0;
        switch (lamp) {
            case kFrontLeft:  status = dm.m_nLeftFrontLightStatus;  break;
            case kFrontRight: status = dm.m_nRightFrontLightStatus; break;
            case kRearLeft:   status = dm.m_nLeftRearLightStatus;   break;
            default:          status = dm.m_nRightRearLightStatus;  break;
        }
        return status == 0;
    }

    using StoreCarLightShadowFn=void (*)(CVehicle*,int,RwTexture*,CVector*,float,float,float,float,uint8_t,uint8_t,uint8_t,float);
    StoreCarLightShadowFn g_originalStoreCarLightShadow=nullptr;
    void HookStoreCarLightShadow(CVehicle* car,int id,RwTexture* tex,CVector* pos,float fx,float fy,float sx,float sy,uint8_t r,uint8_t g,uint8_t b,float radius) {
        ++g_stats.poolsSeen;
        ++g_stats.poolsBlocked;
        // Deliberately do not call the original function. This removes the
        // stock vehiclelights128 textured pool entirely; otherwise it creates
        // the large detached halo/shadow around cars and doubles the work.
        (void)car; (void)id; (void)tex; (void)pos; (void)fx; (void)fy;
        (void)sx; (void)sy; (void)r; (void)g; (void)b; (void)radius;
    }

    void AddSurfaceBounceLights(VehicleState& state) {
        Patch* selected[kSurfaceLightsPerVehicle]={};
        for (int choice=0;choice<kSurfaceLightsPerVehicle;++choice) {
            Patch* best=nullptr;
            for (Patch& patch:state.patches) {
                if (!patch.has||patch.current<0.16f||patch.target<0.08f) continue;
                bool separated=true;
                for (Patch* previous:selected) {
                    if (!previous) continue;
                    const CVector d(patch.pos.x-previous->pos.x,
                                    patch.pos.y-previous->pos.y,
                                    patch.pos.z-previous->pos.z);
                    // Permit adjacent left/right samples to contribute separate
                    // light pools while still preventing nearly coincident lamps.
                    if (LengthSquared(d)<0.64f) { separated=false; break; }
                }
                if (separated&&(!best||patch.current>best->current)) best=&patch;
            }
            if (!best) break;
            selected[choice]=best;
        }

        for (Patch* patch:selected) {
            if (!patch||g_trackedLightCount>=kMaxTrackedLights) continue;
            if (CPointLights::NumLights>=kMaxPointLights) { ++g_stats.surfacePoolFull; continue; }
            // These are actual GTA point lights placed just outside the struck
            // surface, not camera-facing sprites/decal circles. GTA's normal
            // object-lighting pass shades the wall and nearby receiver geometry.
            const CVector position(patch->pos.x+patch->normal.x*0.07f,
                                   patch->pos.y+patch->normal.y*0.07f,
                                   patch->pos.z+patch->normal.z*0.07f);
            // Bounce light stays below the direct beam: it softly lifts
            // nearby surfaces instead of looking like a second lamp.
            const float power=Clamp01(patch->current * 0.72f);
            const int slot=CPointLights::NumLights;
            ++g_stats.surfaceAttempts;
            CPointLights::AddLight(PLTYPE_POINTLIGHT,position,CVector(0,0,0),22.0f,
                                   0.92f*power,0.78f*power,0.60f*power,
                                   0,false,nullptr);
            if (CPointLights::NumLights<=slot) { ++g_stats.surfaceCulled; continue; } // engine camera/range cull rejected it
            TrackedLight& light=g_trackedLights[g_trackedLightCount++];
            light={slot,patch->entity,position,patch->normal,CTimer::m_FrameCounter,true};
            ++g_stats.surfaceLights;
            if (!g_firstSurfaceLightLogged) {
                g_firstSurfaceLightLogged=true;
                Log("first native surface light accepted: range=22.0 power=%.2f slot=%d at=(%.1f,%.1f,%.1f)",
                    power,slot,position.x,position.y,position.z);
            }
        }
    }

    void RegisterHeadlights(CVehicle* vehicle) {
        BeginFrame();
        if (!vehicle) return;
        if (!vehicle->bLightsOn || vehicle->m_fHealth <= 0.0f) {
            // An explicit game light-off/damage state must not be held by the
            // anti-flicker cache. This makes the dynamic light disappear with
            // the GTA lamp instead of lingering for the grace frames.
            DisableVehicleState(vehicle);
            return;
        }
        if (!vehicle->GetMatrix() || AlreadyRegistered(vehicle) || g_registeredCount>=kMaxRegisteredPerFrame) return;
        CPlayerPed* player=FindPlayerPed(0);
        if (!player||player->GetInterior()!=vehicle->GetInterior()) return;
        const CVector vp=vehicle->GetPosition(),pp=player->GetPosition();
        const float dx=vp.x-pp.x,dy=vp.y-pp.y,dz=vp.z-pp.z;
        if (dx*dx+dy*dy+dz*dz>kMaxSourceDistance*kMaxSourceDistance) return;
        LampSet lamps; if (!ComputeLamps(vehicle,lamps)) return;
        VehicleState* state=GetState(vehicle,CTimer::m_FrameCounter); state->lastSeenFrame=CTimer::m_FrameCounter;
        {
            const float night = NightFactor();
            const VehicleLightGL::Settings& settings = VehicleLightGL::GetSettings();
            state->glSpotOn[0] = state->glSpotOn[1] = false; state->glRearOn = false;
            for (int i = 0; i < lamps.count && i < 2; ++i) {
                float& fade = state->lampFade[i];
                // Intact lamp fades in smoothly; a smashed lamp goes dark immediately, per side.
                if (IsLampWorking(vehicle, i)) fade += (1.0f - fade) * 0.25f;
                else {
                    if (fade > 0.0f && !state->brokenLogged) {
                        state->brokenLogged = true;
                        Log("front lamp %d smashed on vehicle %p: its dynamic light is off", i, (void*)vehicle);
                    }
                    fade = 0.0f;
                }
                if (fade < 0.02f) continue;
                AddShaderSpot(state->glSpot[i], lamps.pos[i], lamps.dir, lamps.right, lamps.lampUp, settings.range, night * fade, SpotBox(lamps, i));
                state->glSpotOn[i] = true;
            }
            if (settings.rear) {
                const int rearWorking = (IsLampWorking(vehicle, kRearLeft) ? 1 : 0) + (IsLampWorking(vehicle, kRearRight) ? 1 : 0);
                if (rearWorking > 0) {
                    const CVector backward(-lamps.fwd.x, -lamps.fwd.y, -lamps.fwd.z);
                    AddShaderRearGlow(state->glRear, lamps.rearPos, backward, lamps.right, lamps.lampUp,
                                      night * (0.5f * rearWorking) * std::max(state->lampFade[0], state->lampFade[lamps.count - 1]),
                                      RearBox(lamps));
                    state->glRearOn = true;
                }
            }
            state->cacheFrame = CTimer::m_FrameCounter; state->cacheValid = true;
            // Publish right away so the light pass of this very frame uses fresh positions.
            PublishNow();
        }
        bool added=false;
        // The GLES pass is the normal dynamic-only path. The native spotlight
        // below is only a fallback for devices where the GLES interceptor did
        // not install; it follows the same per-lamp damage mask.
        if (!VehicleLightGL::IsInstalled()) {
            for (int i=0; g_registeredCount<kMaxVehiclesPerFrame && i<lamps.count && g_trackedLightCount<kMaxTrackedLights &&
                        CPointLights::NumLights<kMaxHeadlightLights; ++i) {
                if (!IsLampWorking(vehicle, i)) continue;
                const int slot=CPointLights::NumLights;
                CPointLights::AddLight(PLTYPE_SPOTLIGHT,lamps.pos[i],lamps.dir,kHeadlightRange,
                                       0.92f,0.90f,0.84f,0,false,nullptr);
                if (CPointLights::NumLights>slot) {
                    TrackedLight& light=g_trackedLights[g_trackedLightCount++];
                    light={slot,vehicle,lamps.pos[i],lamps.dir,CTimer::m_FrameCounter,false};
                    ++g_stats.spots;
                    added=true;
                }
            }
        }

        g_registeredVehicles[g_registeredCount++]=vehicle;
        if (added&&!g_firstLightLogged) {
            Log("first vehicle spotlights registered; range=%.1f",kHeadlightRange);
            g_firstLightLogged=true;
        }
        const uint32_t now=CTimer::GetTimeMS();
        const bool glPath = VehicleLightGL::IsInstalled();
        if (!glPath && kBounceLight) {
            if (now-state->lastUpdate>=kUpdateIntervalMs) {
                state->lastUpdate=now;
                UpdatePatches(vehicle,*state,lamps,NightFactor());
            }
            for (Patch& patch:state->patches)
                patch.current+=(patch.target-patch.current)*0.35f;
        }
        // GLES surface lighting is the active path. Do not also run collision
        // patch raycasts, native
        // bounce lights, or Im3D beam preparation in that mode; those were
        // responsible for most of the remaining frame spikes and dark gated
        // vehicle patches.
        if (!glPath && kBounceLight) AddSurfaceBounceLights(*state);
        if (!glPath && BeamReady()) {
            if (g_beamUpdateFrame != CTimer::m_FrameCounter) {
                g_beamUpdateFrame = CTimer::m_FrameCounter;
                g_beamUpdatesThisFrame = 0;
            }
            if (now - state->beamUpdate >= kBeamUpdateMs && g_beamUpdatesThisFrame < kBeamUpdatesPerFrame) {
                state->beamUpdate = now;
                ++g_beamUpdatesThisFrame;
                UpdateBeams(vehicle, *state, lamps, NightFactor());
            }
            if (state->front.built) {
                BeamCore::Smoothen(state->front, kBeamSmooth);
                BeamCore::Smoothen(state->rear, kBeamSmooth);
                state->beamFrame = CTimer::m_FrameCounter;
            }
        }
    }

    using PreRenderFn=void (*)(CVehicle*);
    PreRenderFn g_originalVehiclePreRender=nullptr,g_originalAutoPreRender=nullptr,g_originalBikePreRender=nullptr;
    void HookVehiclePreRender(CVehicle* v) { if (g_originalVehiclePreRender) g_originalVehiclePreRender(v); RegisterHeadlights(v); }
    void HookAutoPreRender(CVehicle* v) { if (g_originalAutoPreRender) g_originalAutoPreRender(v); RegisterHeadlights(v); }
    void HookBikePreRender(CVehicle* v) { if (g_originalBikePreRender) g_originalBikePreRender(v); RegisterHeadlights(v); }
    bool TryHook(const char* symbol,void* detour,void** original) {
        void* address=GetMainLibrarySymbol<void*>(symbol);
        const bool ok=address&&aml&&aml->Hook(address,detour,original);
        Log("hook %-60s %s",symbol,ok?"OK":(address?"FAILED":"symbol missing")); return ok;
    }

    RwTexture* CreateBeamTexture() {
        constexpr int kSize = 128;
        RwImage* image = RwImageCreate(kSize, kSize, 32);
        if (!image || !RwImageAllocatePixels(image) || !image->cpPixels || image->stride < image->width * 4) {
            if (image) RwImageDestroy(image);
            Log("beam texture: image allocation failed");
            return nullptr;
        }
        for (int y = 0; y < image->height; ++y) {
            RwUInt8* row = image->cpPixels + y * image->stride;
            const float v = (static_cast<float>(y) + 0.5f) / kSize;
            for (int x = 0; x < image->width; ++x) {
                const float u = (static_cast<float>(x) + 0.5f) / kSize;
                float a = BeamCore::SoftKernel(u, v);
                if (x == 0 || x == kSize - 1 || y == kSize - 1) a = 0.0f;
                RwUInt8* px = row + x * 4;
                px[0] = px[1] = px[2] = 255;
                px[3] = static_cast<RwUInt8>(Clamp01(a) * 255.0f);
            }
        }
        RwInt32 width = image->width, height = image->height, depth = image->depth, format = 0;
        if (!RwImageFindRasterFormat(image, rwRASTERTYPETEXTURE, &width, &height, &depth, &format)) {
            RwImageDestroy(image);
            Log("beam texture: no raster format");
            return nullptr;
        }
        depth = 32;
        format = (format & ~static_cast<RwInt32>(rwRASTERFORMATPIXELFORMATMASK)) | static_cast<RwInt32>(rwRASTERFORMAT8888);
        RwRaster* raster = RwRasterCreate(width, height, depth, format);
        if (!raster || !RwRasterSetFromImage(raster, image)) {
            if (raster) RwRasterDestroy(raster);
            RwImageDestroy(image);
            Log("beam texture: raster upload failed");
            return nullptr;
        }
        RwImageDestroy(image);
        RwTexture* texture = RwTextureCreate(raster);
        if (!texture) {
            RwRasterDestroy(raster);
            Log("beam texture: RwTextureCreate failed");
            return nullptr;
        }
        RwTextureSetName(texture, kBeamTexName);
        g_beamRaster = raster;
        Log("beam texture created %dx%d depth=%d format=0x%x", width, height, depth, format);
        return texture;
    }

    BeamCore::V3 B3(const CVector& v) { return {v.x, v.y, v.z}; }

    void EnsureBeamResources() {
        if (BeamReady()) return;
        const uint32_t now = CTimer::GetTimeMS();
        if (now - g_lastBeamTexTry < 1000u) return;
        g_lastBeamTexTry = now;
        if (!g_im3d)
            g_im3d = GetMainLibrarySymbol<Im3DRenderIndexedPrimitiveFn>("_Z28RwIm3DRenderIndexedPrimitive15RwPrimitiveTypePti");
        if (!g_beamTexture) g_beamTexture = CreateBeamTexture();
    }

    void UpdateBeams(CVehicle* vehicle, VehicleState& state, const LampSet& lamps, float night) {
        CMatrix* matrix = vehicle->GetMatrix();
        if (!matrix) return;
        BeamCore::Frame vehicleFrame;
        vehicleFrame.origin = B3(matrix->GetPosition());
        vehicleFrame.right = B3(lamps.right);
        vehicleFrame.fwd = B3(lamps.fwd);
        vehicleFrame.up = B3(lamps.up);
        auto cast = [&](const BeamCore::V3& from, const BeamCore::V3& to,
                        BeamCore::V3& point, BeamCore::V3& normal) {
            CColPoint collision{};
            CEntity* entity = nullptr;
            if (!CastRay(CVector(from.x, from.y, from.z), CVector(to.x, to.y, to.z),
                         vehicle, collision, entity)) return false;
            point = B3(collision.m_vecPoint);
            normal = B3(collision.m_vecNormal);
            return true;
        };
        const CVector midpoint((lamps.pos[0].x + lamps.pos[lamps.count - 1].x) * 0.5f,
                               (lamps.pos[0].y + lamps.pos[lamps.count - 1].y) * 0.5f,
                               (lamps.pos[0].z + lamps.pos[lamps.count - 1].z) * 0.5f);
        BeamCore::BeamSpec spec;
        spec.lampHeight = lamps.groundH;
        spec.halfSep = lamps.halfSep;
        BeamCore::Frame beamFrame = vehicleFrame;
        beamFrame.origin = B3(midpoint);
        beamFrame.fwd = B3(lamps.dir);
        const int hits = BeamCore::BuildBeam(spec, beamFrame, vehicleFrame,
                                              night * kBeamOpacity, cast, state.front);
        if (kRearGlow && (state.rearTick = !state.rearTick)) {
            BeamCore::BeamSpec rearSpec;
            rearSpec.lampHeight = lamps.groundH;
            rearSpec.halfSep = lamps.halfSep;
            rearSpec.range = 6.5f;
            rearSpec.yawMaxDeg = 50.0f;
            rearSpec.tiltDeg = 0.0f;
            BeamCore::Frame rearFrame = vehicleFrame;
            rearFrame.origin = B3(lamps.rearPos);
            rearFrame.fwd = B3(lamps.fwd) * -1.0f;
            rearFrame.right = B3(lamps.right) * -1.0f;
            BeamCore::BuildBeam(rearSpec, rearFrame, vehicleFrame,
                                night * kBeamOpacity * 0.55f, cast, state.rear);
            state.rearOn = true;
        }
        if (!g_beamLogged && hits > 0) {
            g_beamLogged = true;
            Log("first fake beam built: %d/%d rays, lampH=%.2f", hits, BeamCore::kVerts, lamps.groundH);
        }
    }

    constexpr int kChunkQuads = 32;
    constexpr float kShaftOpacity = 0.11f;

    BeamCore::Frame VehicleFrame(const CMatrix& matrix) {
        BeamCore::Frame frame;
        frame.origin = B3(matrix.GetPosition());
        frame.right = B3(matrix.GetRight());
        frame.fwd = B3(matrix.GetForward());
        frame.up = B3(matrix.GetUp());
        return frame;
    }

    void FlushQuads(RwIm3DVertex* vertices, int quadCount) {
        if (quadCount <= 0) return;
        static RwImVertexIndex indices[kChunkQuads * 6];
        static bool initialized = false;
        if (!initialized) {
            for (int q = 0; q < kChunkQuads; ++q) {
                const RwImVertexIndex offset = static_cast<RwImVertexIndex>(q * 4);
                indices[q * 6] = offset;
                indices[q * 6 + 1] = offset + 1;
                indices[q * 6 + 2] = offset + 2;
                indices[q * 6 + 3] = offset;
                indices[q * 6 + 4] = offset + 2;
                indices[q * 6 + 5] = offset + 3;
            }
            initialized = true;
        }
        if (!RwIm3DTransform(reinterpret_cast<RwIm2DVertex*>(vertices), quadCount * 4, nullptr, 0x0FU)) return;
        if (g_im3d(rwPRIMTYPETRILIST, indices, quadCount * 6)) ++g_beamDrawn;
        RwIm3DEnd();
    }

    void DrawSplats(const BeamCore::Beam& beam, const BeamCore::Frame& vehicle,
                    const CVector& camera, uint8_t red, uint8_t green, uint8_t blue, float gain) {
        RwIm3DVertex vertices[kChunkQuads * 4];
        int quadCount = 0;
        for (int i = 0; i < BeamCore::kVerts; ++i) {
            if (!beam.placed[i] || beam.current[i] < 0.012f) continue;
            const BeamCore::V3 point = BeamCore::PointToWorld(beam.local[i], vehicle);
            const float dx = point.x - camera.x, dy = point.y - camera.y, dz = point.z - camera.z;
            const float cameraDistance = std::sqrt(dx * dx + dy * dy + dz * dz);
            const float alpha = Clamp01(beam.current[i] * gain * BeamCore::Smooth(0.8f, 3.5f, cameraDistance));
            if (alpha < 0.008f) continue;
            const BeamCore::V3 normal = BeamCore::Norm(BeamCore::DirToWorld(beam.normal[i], vehicle), BeamCore::V3(0, 0, 1));
            const float lift = 0.045f + cameraDistance * 0.006f;
            BeamCore::Quad quad;
            BeamCore::SplatQuad(beam, i, vehicle, quad);
            for (int corner = 0; corner < 4; ++corner) {
                RwIm3DVertex& vertex = vertices[quadCount * 4 + corner];
                vertex.x = quad.p[corner].x + normal.x * lift;
                vertex.y = quad.p[corner].y + normal.y * lift;
                vertex.z = quad.p[corner].z + normal.z * lift;
                vertex.normal = CVector(normal.x, normal.y, normal.z);
                vertex.r = red; vertex.g = green; vertex.b = blue;
                vertex.a = static_cast<uint8_t>(alpha * 255.0f);
                vertex.u = quad.u[corner]; vertex.v = quad.v[corner];
            }
            if (++quadCount == kChunkQuads) {
                FlushQuads(vertices, quadCount);
                quadCount = 0;
            }
        }
        FlushQuads(vertices, quadCount);
    }

    void DrawShafts(const BeamCore::Beam& beam, const BeamCore::Frame& vehicle,
                    const CVector& camera, uint8_t red, uint8_t green, uint8_t blue) {
        if (kShaftOpacity <= 0.0f || beam.shaftFade < 0.02f) return;
        constexpr int kSegments = 5;
        constexpr int kRibbonVertices = (kSegments + 1) * 2;
        constexpr int kRibbonsPerCall = 10;
        RwIm3DVertex vertices[kRibbonsPerCall * kRibbonVertices];
        RwImVertexIndex indices[kRibbonsPerCall * kSegments * 6];
        int ribbonCount = 0, indexCount = 0;
        auto flush = [&]() {
            if (!ribbonCount) return;
            if (RwIm3DTransform(reinterpret_cast<RwIm2DVertex*>(vertices), ribbonCount * kRibbonVertices, nullptr, 0x0FU)) {
                if (g_im3d(rwPRIMTYPETRILIST, indices, indexCount)) ++g_beamDrawn;
                RwIm3DEnd();
            }
            ribbonCount = 0;
            indexCount = 0;
        };
        for (int column = 0; column < BeamCore::kCols; column += 2) {
            if (!beam.shaftOk[column]) continue;
            const BeamCore::V3 from = BeamCore::PointToWorld(beam.shaftFrom[column], vehicle);
            const BeamCore::V3 direction = BeamCore::Norm(BeamCore::DirToWorld(beam.shaftDir[column], vehicle));
            const float length = beam.shaftLen[column];
            const float lateral = BeamCore::BeamProfile(beam.shaftU[column], 0.14f);
            const float baseAlpha = kShaftOpacity * beam.strength * beam.shaftFade * lateral;
            if (baseAlpha < 0.004f) continue;
            const int vertexBase = ribbonCount * kRibbonVertices;
            bool degenerate = false;
            for (int segment = 0; segment <= kSegments; ++segment) {
                const float along = static_cast<float>(segment) / kSegments;
                const BeamCore::V3 point = from + direction * (length * along);
                const BeamCore::V3 toCamera(camera.x - point.x, camera.y - point.y, camera.z - point.z);
                BeamCore::V3 side = BeamCore::Cross(direction, toCamera);
                if (BeamCore::Len(side) < 1.0e-3f) { degenerate = true; break; }
                side = BeamCore::Norm(side);
                const float width = 0.12f + 0.17f * (length * along);
                const float cameraDistance = BeamCore::Len(toCamera);
                const float alpha = baseAlpha * BeamCore::Smooth(0.0f, 0.07f, along) *
                    std::pow(1.0f - along, 1.7f) * BeamCore::Smooth(0.8f, 4.0f, cameraDistance);
                for (int sideIndex = 0; sideIndex < 2; ++sideIndex) {
                    RwIm3DVertex& vertex = vertices[vertexBase + segment * 2 + sideIndex];
                    const float sign = sideIndex ? 1.0f : -1.0f;
                    vertex.x = point.x + side.x * width * sign;
                    vertex.y = point.y + side.y * width * sign;
                    vertex.z = point.z + side.z * width * sign;
                    vertex.normal = CVector(0, 0, 1);
                    vertex.r = red; vertex.g = green; vertex.b = blue;
                    vertex.a = static_cast<uint8_t>(Clamp01(alpha) * 255.0f);
                    vertex.u = sideIndex ? 1.0f : 0.0f; vertex.v = 0.5f;
                }
            }
            if (degenerate) continue;
            for (int segment = 0; segment < kSegments; ++segment) {
                const RwImVertexIndex offset = static_cast<RwImVertexIndex>(vertexBase + segment * 2);
                indices[indexCount++] = offset; indices[indexCount++] = offset + 1; indices[indexCount++] = offset + 2;
                indices[indexCount++] = offset + 1; indices[indexCount++] = offset + 3; indices[indexCount++] = offset + 2;
            }
            if (++ribbonCount == kRibbonsPerCall) flush();
        }
        flush();
    }

    void RenderBeams() {
        // GLES3 is the texture-preserving path. Do not draw the collision
        // splat/ribbon mesh on top of it; that was the source of intermittent
        // polygon-shaped light artifacts.
        if (VehicleLightGL::IsInstalled()) return;
        const unsigned frame = CTimer::m_FrameCounter;
        if (frame == g_lastBeamRenderFrame || !BeamReady()) return;
        g_lastBeamRenderFrame = frame;
        bool any = false;
        for (const VehicleState& state : g_states)
            if (state.vehicle && state.front.built && frame - state.beamFrame <= 2u) { any = true; break; }
        if (!any) return;
        const CVector camera = TheCamera.GetPosition();
        void *oldRaster=nullptr, *oldZWrite=nullptr, *oldZTest=nullptr, *oldVertexAlpha=nullptr;
        void *oldSourceBlend=nullptr, *oldDestinationBlend=nullptr, *oldCull=nullptr;
        void *oldAddressU=nullptr, *oldAddressV=nullptr, *oldFilter=nullptr;
        RwRenderStateGet(rwRENDERSTATETEXTURERASTER, &oldRaster);
        RwRenderStateGet(rwRENDERSTATEZWRITEENABLE, &oldZWrite);
        RwRenderStateGet(rwRENDERSTATEZTESTENABLE, &oldZTest);
        RwRenderStateGet(rwRENDERSTATEVERTEXALPHAENABLE, &oldVertexAlpha);
        RwRenderStateGet(rwRENDERSTATESRCBLEND, &oldSourceBlend);
        RwRenderStateGet(rwRENDERSTATEDESTBLEND, &oldDestinationBlend);
        RwRenderStateGet(rwRENDERSTATECULLMODE, &oldCull);
        RwRenderStateGet(rwRENDERSTATETEXTUREADDRESSU, &oldAddressU);
        RwRenderStateGet(rwRENDERSTATETEXTUREADDRESSV, &oldAddressV);
        RwRenderStateGet(rwRENDERSTATETEXTUREFILTER, &oldFilter);
        RwRenderStateSet(rwRENDERSTATEZWRITEENABLE, (void*)false);
        RwRenderStateSet(rwRENDERSTATEZTESTENABLE, (void*)true);
        RwRenderStateSet(rwRENDERSTATEVERTEXALPHAENABLE, (void*)true);
        RwRenderStateSet(rwRENDERSTATESRCBLEND, (void*)rwBLENDSRCALPHA);
        RwRenderStateSet(rwRENDERSTATEDESTBLEND, (void*)rwBLENDONE);
        RwRenderStateSet(rwRENDERSTATECULLMODE, (void*)rwCULLMODECULLNONE);
        RwRenderStateSet(rwRENDERSTATETEXTUREADDRESSU, (void*)rwTEXTUREADDRESSCLAMP);
        RwRenderStateSet(rwRENDERSTATETEXTUREADDRESSV, (void*)rwTEXTUREADDRESSCLAMP);
        RwRenderStateSet(rwRENDERSTATETEXTUREFILTER, (void*)rwFILTERLINEAR);
        RwRenderStateSet(rwRENDERSTATETEXTURERASTER, (void*)g_beamTexture->raster);
        for (VehicleState& state : g_states) {
            if (!state.vehicle || !state.front.built || frame - state.beamFrame > 2u) continue;
            CMatrix* matrix = state.vehicle->GetMatrix();
            if (!matrix) continue;
            const BeamCore::Frame vehicleFrame = VehicleFrame(*matrix);
            DrawSplats(state.front, vehicleFrame, camera, 255, 243, 214, 1.0f);
            DrawShafts(state.front, vehicleFrame, camera, 255, 246, 225);
            if (state.rearOn && state.rear.built)
                DrawSplats(state.rear, vehicleFrame, camera, 255, 38, 26, 0.9f);
        }
        RwRenderStateSet(rwRENDERSTATETEXTURERASTER, oldRaster);
        RwRenderStateSet(rwRENDERSTATEZWRITEENABLE, oldZWrite);
        RwRenderStateSet(rwRENDERSTATEZTESTENABLE, oldZTest);
        RwRenderStateSet(rwRENDERSTATEVERTEXALPHAENABLE, oldVertexAlpha);
        RwRenderStateSet(rwRENDERSTATESRCBLEND, oldSourceBlend);
        RwRenderStateSet(rwRENDERSTATEDESTBLEND, oldDestinationBlend);
        RwRenderStateSet(rwRENDERSTATECULLMODE, oldCull);
        RwRenderStateSet(rwRENDERSTATETEXTUREADDRESSU, oldAddressU);
        RwRenderStateSet(rwRENDERSTATETEXTUREADDRESSV, oldAddressV);
        RwRenderStateSet(rwRENDERSTATETEXTUREFILTER, oldFilter);
    }

    // Retired: screen-space discs and optional contrast-shader glows were the
    // source of the fake circular artifacts. Surface hits now use AddLight.
#if 0
    unsigned g_lastRenderFrame=0xFFFFFFFFu;
    void DrawMarker(const CVector& pos,uint8_t r,uint8_t g,uint8_t b,float size) {
        RwV3d screen{}; float sx=0.0f,sy=0.0f;
        if (!CSprite::CalcScreenCoors(ToRw(pos),&screen,&sx,&sy,true,true)) return;
        CSprite::RenderOneXLUSprite_Rotate_Aspect(screen.x,screen.y,screen.z,sx*size,sy*size,r,g,b,255,(1.0f/screen.z)*20.0f,0.0f,255);
    }

    RwTexture* CreateReflectionTexture() {
        constexpr int kSize = 128;
        RwImage* image = RwImageCreate(kSize, kSize, 32);
        if (!image || !RwImageAllocatePixels(image) || !image->cpPixels || image->stride < image->width * 4) {
            if (image) RwImageDestroy(image);
            Log("reflection texture: image allocation failed");
            return nullptr;
        }

        for (int y = 0; y < image->height; ++y) {
            CRGBA* row = reinterpret_cast<CRGBA*>(image->cpPixels + y * image->stride);
            for (int x = 0; x < image->width; ++x) {
                const float px = (static_cast<float>(x) + 0.5f) * (2.0f / kSize) - 1.0f;
                const float py = (static_cast<float>(y) + 0.5f) * (2.0f / kSize) - 1.0f;
                const float radius = std::sqrt(px * px + py * py);
                // Broad, smooth penumbra with a restrained hot center. The sampled
                // world texture remains visible underneath the additive light.
                const float edge = Clamp01((1.0f - radius) / 0.52f);
                const float softEdge = edge * edge * (3.0f - 2.0f * edge);
                const float glow = std::exp(-radius * radius * 2.7f) * softEdge;
                row[x].Set(255, 237, 202, static_cast<uint8_t>(Clamp01(glow) * 255.0f));
            }
        }

        RwInt32 width = image->width;
        RwInt32 height = image->height;
        RwInt32 depth = image->depth;
        RwInt32 format = 0;
        if (!RwImageFindRasterFormat(image, rwRASTERTYPETEXTURE, &width, &height, &depth, &format)) {
            RwImageDestroy(image);
            Log("reflection texture: no compatible raster format");
            return nullptr;
        }

        RwRaster* raster = RwRasterCreate(width, height, depth, rwRASTERTYPETEXTURE | format);
        if (!raster || !RwRasterSetFromImage(raster, image)) {
            if (raster) RwRasterDestroy(raster);
            RwImageDestroy(image);
            Log("reflection texture: raster creation/upload failed");
            return nullptr;
        }
        RwImageDestroy(image);

        RwTexture* texture = RwTextureCreate(raster);
        if (!texture) {
            RwRasterDestroy(raster);
            Log("reflection texture: RwTextureCreate failed");
            return nullptr;
        }
        RwTextureSetName(texture, "beta_vehicle_light_glow");
        Log("created procedural reflection texture (%dx%d, depth=%d, format=0x%x)", width, height, depth, format);
        return texture;
    }

    RwTexture* GetReflectionTexture(uint32_t now) {
        if (g_reflectionTexture && g_reflectionTexture->raster) return g_reflectionTexture;
        if (now - g_lastTextureAttempt >= 1000u) {
            g_lastTextureAttempt = now;
            g_reflectionTexture = CreateReflectionTexture();
        }
        return g_reflectionTexture && g_reflectionTexture->raster ? g_reflectionTexture : nullptr;
    }

    bool DrawSurfaceGlow(const CVector& center,const CVector& normal,
                         const CVector& tangent,const CVector& bitangent,
                         float majorRadius,float minorRadius,uint8_t alpha) {
        if (!g_im3dRenderIndexedPrimitive || alpha==0) return false;
        RwIm3DVertex vertices[kGlowVertexCount];
        RwImVertexIndex indices[kGlowIndexCount];
        constexpr float kSurfaceLift=0.018f;
        auto setVertex=[&](int index,float diskX,float diskY) {
            const CVector p(center.x+tangent.x*(diskX*majorRadius)+bitangent.x*(diskY*minorRadius)+normal.x*kSurfaceLift,
                            center.y+tangent.y*(diskX*majorRadius)+bitangent.y*(diskY*minorRadius)+normal.y*kSurfaceLift,
                            center.z+tangent.z*(diskX*majorRadius)+bitangent.z*(diskY*minorRadius)+normal.z*kSurfaceLift);
            RwIm3DVertex& v=vertices[index];
            v.x=p.x; v.y=p.y; v.z=p.z;
            v.normal=ToRw(normal);
            v.r=255; v.g=242; v.b=205; v.a=alpha;
            v.u=0.5f+0.5f*diskX; v.v=0.5f+0.5f*diskY;
        };
        setVertex(0,0.0f,0.0f);
        for (int ring=1;ring<=kGlowRings;++ring) {
            const float radius=static_cast<float>(ring)/kGlowRings;
            for (int segment=0;segment<kGlowSegments;++segment) {
                const float angle=6.28318530718f*segment/kGlowSegments;
                setVertex(1+(ring-1)*kGlowSegments+segment,
                          std::cos(angle)*radius,std::sin(angle)*radius);
            }
        }
        int indexCount=0;
        for (int segment=0;segment<kGlowSegments;++segment) {
            const int next=(segment+1)%kGlowSegments;
            indices[indexCount++]=0;
            indices[indexCount++]=static_cast<RwImVertexIndex>(1+segment);
            indices[indexCount++]=static_cast<RwImVertexIndex>(1+next);
        }
        for (int ring=1;ring<kGlowRings;++ring) {
            const int inner=1+(ring-1)*kGlowSegments;
            const int outer=1+ring*kGlowSegments;
            for (int segment=0;segment<kGlowSegments;++segment) {
                const int next=(segment+1)%kGlowSegments;
                const RwImVertexIndex i0=static_cast<RwImVertexIndex>(inner+segment);
                const RwImVertexIndex i1=static_cast<RwImVertexIndex>(inner+next);
                const RwImVertexIndex o0=static_cast<RwImVertexIndex>(outer+segment);
                const RwImVertexIndex o1=static_cast<RwImVertexIndex>(outer+next);
                indices[indexCount++]=i0; indices[indexCount++]=o0; indices[indexCount++]=i1;
                indices[indexCount++]=i1; indices[indexCount++]=o0; indices[indexCount++]=o1;
            }
        }
        void* transformed=RwIm3DTransform(reinterpret_cast<RwIm2DVertex*>(vertices),
                                          kGlowVertexCount,nullptr,kIm3DVertexFlags);
        if (!transformed) return false;
        const bool rendered=g_im3dRenderIndexedPrimitive(rwPRIMTYPETRILIST,indices,indexCount)!=0;
        RwIm3DEnd();
        if (rendered) ++g_stats.patchesDrawn;
        return rendered;
    }

    bool ConnectShaderLoader(uint32_t now) {
        if (g_shaderUniformsReady) return true;
        if (now-g_lastShaderLookup<1000u) return false;
        g_lastShaderLookup=now;
        g_shaderLoader=static_cast<ISASL*>(GetInterface("SASL"));
        if (!g_shaderLoader) {
            if (!g_shaderStatusLogged) {
                Log("SAShaderL interface unavailable; using RenderWare reflection fallback");
                g_shaderStatusLogged=true;
            }
            return false;
        }
        char shaderPath[600];
        snprintf(shaderPath,sizeof(shaderPath),"%s/shaders/fragment/contrast.glsl",
                 aml?aml->GetAndroidDataPath():"");
        FILE* shaderFile=fopen(shaderPath,"rb");
        if (!shaderFile) {
            if (!g_shaderStatusLogged) {
                Log("SAShaderL found but shaders/fragment/contrast.glsl is missing; using RenderWare fallback");
                g_shaderStatusLogged=true;
            }
            return false;
        }
        char shaderHeader[2048]{};
        const size_t bytesRead=fread(shaderHeader,1,sizeof(shaderHeader)-1,shaderFile);
        fclose(shaderFile);
        if (!bytesRead || !strstr(shaderHeader,"VL_GlowPower")) {
            if (!g_shaderStatusLogged) {
                Log("contrast.glsl lacks vehicle-light uniforms; using RenderWare fallback");
                g_shaderStatusLogged=true;
            }
            return false;
        }
        const int version=g_shaderLoader->GetFeaturesVersion();
        if (version<1) {
            Log("SAShaderL interface version %d is unsupported",version);
            g_shaderLoader=nullptr;
            return false;
        }
        if (g_shaderRegistrationAttempted) return false;
        g_shaderRegistrationAttempted=true;
        const bool ok=
            g_shaderLoader->RegisterUniform("VL_ScreenSize",SASL_UNIFORM_FLOAT,2,true,g_shaderScreenSize)>=0 &&
            g_shaderLoader->RegisterUniform("VL_GlowX",SASL_UNIFORM_FLOAT,kShaderGlowCount,true,g_shaderGlowX)>=0 &&
            g_shaderLoader->RegisterUniform("VL_GlowY",SASL_UNIFORM_FLOAT,kShaderGlowCount,true,g_shaderGlowY)>=0 &&
            g_shaderLoader->RegisterUniform("VL_GlowRadiusX",SASL_UNIFORM_FLOAT,kShaderGlowCount,true,g_shaderGlowRadiusX)>=0 &&
            g_shaderLoader->RegisterUniform("VL_GlowRadiusY",SASL_UNIFORM_FLOAT,kShaderGlowCount,true,g_shaderGlowRadiusY)>=0 &&
            g_shaderLoader->RegisterUniform("VL_GlowAngle",SASL_UNIFORM_FLOAT,kShaderGlowCount,true,g_shaderGlowAngle)>=0 &&
            g_shaderLoader->RegisterUniform("VL_GlowPower",SASL_UNIFORM_FLOAT,kShaderGlowCount,true,g_shaderGlowPower)>=0;
        if (!ok) {
            Log("SAShaderL found (v%d), but vehicle-light uniforms could not be registered",version);
            g_shaderLoader=nullptr;
            return false;
        }
        g_shaderUniformsReady=true;
        g_shaderStatusLogged=true;
        Log("SAShaderL v%d connected; GLSL contrast override can render vehicle reflections",version);
        return true;
    }

    struct ShaderGlowCandidate {
        Patch* patch=nullptr;
        CVector tangent, bitangent;
        float major=0.0f, minor=0.0f, strength=0.0f;
    };

    void ConsiderShaderGlow(ShaderGlowCandidate (&best)[kShaderGlowCount],Patch& patch,
                            const CVector& tangent,const CVector& bitangent,
                            float major,float minor,float strength) {
        int slot=-1;
        float weakest=strength;
        for (int i=0;i<kShaderGlowCount;++i) {
            if (!best[i].patch) { slot=i; break; }
            if (best[i].strength<weakest) { weakest=best[i].strength; slot=i; }
        }
        if (slot>=0) best[slot]={&patch,tangent,bitangent,major,minor,strength};
    }

    bool IsPatchVisibleToCamera(const CVector& camera,const Patch& patch) {
        CVector target(patch.pos.x-patch.normal.x*0.04f,
                       patch.pos.y-patch.normal.y*0.04f,
                       patch.pos.z-patch.normal.z*0.04f);
        CColPoint cp{};
        CEntity* entity=nullptr;
        ++g_stats.rays;
        if (!CWorld::ProcessLineOfSight(camera,target,cp,entity,
            true,true,true,true,false,false,false,false)) return true;
        const CVector delta(cp.m_vecPoint.x-target.x,
                    cp.m_vecPoint.y-target.y,
                    cp.m_vecPoint.z-target.z);
        return LengthSquared(delta)<0.36f;
    }

    void BuildShaderGlowUniforms(const CVector& camera,
                                 ShaderGlowCandidate (&best)[kShaderGlowCount]) {
        for (int i=0;i<kShaderGlowCount;++i) {
            g_shaderGlowPower[i]=0.0f;
            g_shaderGlowX[i]=g_shaderGlowY[i]=0.0f;
            g_shaderGlowRadiusX[i]=g_shaderGlowRadiusY[i]=1.0f;
            g_shaderGlowAngle[i]=0.0f;
        }
        int width=0,height=0;
        if (TheCamera.m_pRwCamera&&TheCamera.m_pRwCamera->frameBuffer) {
            width=TheCamera.m_pRwCamera->frameBuffer->width;
            height=TheCamera.m_pRwCamera->frameBuffer->height;
        }
        if ((width<=0||height<=0)&&aml) aml->GetDisplaySize(&width,&height);
        g_shaderScreenSize[0]=static_cast<float>(std::max(1,width));
        g_shaderScreenSize[1]=static_cast<float>(std::max(1,height));
        int output=0;
        for (int i=0;i<kShaderGlowCount;++i) {
            const ShaderGlowCandidate& candidate=best[i];
            if (!candidate.patch || !IsPatchVisibleToCamera(camera,*candidate.patch)) continue;
            RwV3d center{},majorEdge{},minorEdge{};
            float scaleX=0.0f,scaleY=0.0f;
            const CVector& p=candidate.patch->pos;
            if (!CSprite::CalcScreenCoors(ToRw(p),&center,&scaleX,&scaleY,true,true) ||
                !CSprite::CalcScreenCoors(ToRw(CVector(p.x+candidate.tangent.x*candidate.major,
                                                       p.y+candidate.tangent.y*candidate.major,
                                                       p.z+candidate.tangent.z*candidate.major)),
                                          &majorEdge,&scaleX,&scaleY,true,true) ||
                !CSprite::CalcScreenCoors(ToRw(CVector(p.x+candidate.bitangent.x*candidate.minor,
                                                       p.y+candidate.bitangent.y*candidate.minor,
                                                       p.z+candidate.bitangent.z*candidate.minor)),
                                          &minorEdge,&scaleX,&scaleY,true,true)) continue;
            const float vx=majorEdge.x-center.x,vy=majorEdge.y-center.y;
            const float rx=std::sqrt(vx*vx+vy*vy);
            const float ux=minorEdge.x-center.x,uy=minorEdge.y-center.y;
            const float ry=std::sqrt(ux*ux+uy*uy);
            if (!std::isfinite(rx)||!std::isfinite(ry)||rx<2.0f||ry<2.0f) continue;
            g_shaderGlowX[output]=center.x;
            g_shaderGlowY[output]=center.y;
            g_shaderGlowRadiusX[output]=std::min(180.0f,rx*1.35f);
            g_shaderGlowRadiusY[output]=std::min(180.0f,ry*1.35f);
            g_shaderGlowAngle[output]=std::atan2(vy,vx);
            g_shaderGlowPower[output]=Clamp01(candidate.strength*0.18f);
            ++output;
        }
    }

#endif

    void RenderPatches() {
        const uint32_t now=CTimer::GetTimeMS();
        if (now-g_lastSummary<kSummaryIntervalMs) return;
        g_lastSummary=now;
        Log("summary: spots=%u reflectUpdates=%u rays=%u hits=%u surfaceTry=%u surfaceLights=%u surfaceCulled=%u surfacePoolFull=%u gateRays=%u gated=%u stockPools=%u blocked=%u",
            g_stats.spots,g_stats.updates,g_stats.rays,g_stats.hits,g_stats.surfaceAttempts,g_stats.surfaceLights,
            g_stats.surfaceCulled,g_stats.surfacePoolFull,
            g_stats.gateRays,g_stats.gated,g_stats.poolsSeen,g_stats.poolsBlocked);
    }
} // namespace

void Initialize() {
    OpenLog();
    VehicleLightGL::Install(Log);
    Log("initialize: aml=%p game=%s androidData=%s",(void*)aml,aml?aml->GetCurrentGame():"?",aml?aml->GetAndroidDataPath():"?");
    g_lightHookInstalled=TryHook("_ZN12CPointLights29GenerateLightsAffectingObjectEPK7CVectorPfP7CEntity",
        reinterpret_cast<void*>(HookGenerateLightsAffectingObject),reinterpret_cast<void**>(&g_originalGenerateLights));
    g_shadowHookInstalled=TryHook("_ZN8CShadows19StoreCarLightShadowEP8CVehicleiP9RwTextureP7CVectorffffhhhf",
        reinterpret_cast<void*>(HookStoreCarLightShadow),reinterpret_cast<void**>(&g_originalStoreCarLightShadow));
    TryHook("_ZN8CVehicle9PreRenderEv",reinterpret_cast<void*>(HookVehiclePreRender),reinterpret_cast<void**>(&g_originalVehiclePreRender));
    TryHook("_ZN11CAutomobile9PreRenderEv",reinterpret_cast<void*>(HookAutoPreRender),reinterpret_cast<void**>(&g_originalAutoPreRender));
    TryHook("_ZN5CBike9PreRenderEv",reinterpret_cast<void*>(HookBikePreRender),reinterpret_cast<void**>(&g_originalBikePreRender));
    Events::vehicleRenderEvent.before += [](CVehicle* vehicle) { RegisterHeadlights(vehicle); };
    Events::gameProcessEvent.after += []() {
        if (!VehicleLightGL::IsInstalled()) EnsureBeamResources();
    };
    Events::renderEffectsEvent.after += []() {
        if (!VehicleLightGL::IsInstalled()) VehicleLightGL::Install(Log);
        PublishShaderLights();
    };
    Events::renderEffectsEvent.after += []() { RenderBeams(); };
    Events::renderEffectsEvent.after += []() { RenderPatches(); };
    Events::renderCoronasEvent.after += []() { RenderPatches(); };
    logger->Info("Dynamic vehicle lights initialized (vehiclelights128 hook=%d, receiver occlusion=%d).",
                 g_shadowHookInstalled?1:0,g_lightHookInstalled?1:0);
}
} // namespace DynamicVehicleLightSystem
