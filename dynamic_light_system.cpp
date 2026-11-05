#include "dynamic_light_system.h"
#include "vehicle_light_gl.h"

#include <mod/amlmod.h>
#include <mod/logger.h>

#include <aml-psdk/game_sa/Events.h>
#include <aml-psdk/game_sa/base/Timer.h>
#include <aml-psdk/game_sa/engine/PointLights.h>
#include <aml-psdk/game_sa/entity/Entity.h>
#include <aml-psdk/game_sa/other/Coronas.h>
#include <aml-psdk/game_sa/entity/Camera.h>

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace DynamicLightSystem {
namespace {
    constexpr float kCaptureDistance = 60.0f;
    constexpr int kMaxWorldLights = 3;

    // Lights are TRACKED across frames instead of being rebuilt from scratch every frame.
    // The game registers lamp lights / coronas only when it feels like it (distance fades, LOD swaps,
    // corona flicker, visibility), so a per-frame "best 4" list used to blink and swap slots.
    // Now every light has a smoothed brightness, is held for a couple of frames if it is not
    // re-registered, and the 4 published slots are chosen with hysteresis and cross-faded.
    constexpr int kMaxTracked = 16;
    constexpr int kMaxPublished = kMaxWorldLights + 1;     // 4 chosen + 1 still fading out
    constexpr unsigned kHoldFrames = 2;

    struct Tracked {
        bool used = false, selected = false;
        VehicleLightGL::Light light;
        float base = 0.0f;       // latest registered strength
        float shown = 0.0f;      // smoothed strength
        float slotGain = 0.0f;   // smoothed "is in the 4 best" weight
        float rank = 0.0f;
        unsigned seen = 0;
    };
    Tracked g_tracked[kMaxTracked];
    VehicleLightGL::Light g_lights[kMaxPublished];
    int g_count = 0;
    unsigned g_frame = 0xFFFFFFFFu;
    uint32_t g_stepMs = 0;
    unsigned g_accepted = 0;
    uint32_t g_lastSummary = 0;

    using AddLightFn = void (*)(uint8_t, CVector, CVector, float, float, float, float, uint8_t, bool, CEntity*);
    AddLightFn g_originalAddLight = nullptr;
    using RegisterCoronaFn = void (*)(uintptr_t, CEntity*, uint8_t, uint8_t, uint8_t, uint8_t,
                                      const CVector&, float, float, eCoronaType, eCoronaFlareType,
                                      uint8_t, uint8_t, uint8_t, float, bool, float, bool, float, bool, bool);
    RegisterCoronaFn g_originalRegisterCorona = nullptr;

    bool g_addLightHookInstalled = false;
    bool g_coronaHookInstalled = false;

    bool IsFinite(const CVector& v) {
        return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
    }
    float Clamp01(float value) { return std::max(0.0f, std::min(1.0f, value)); }
    CVector Cross3(const CVector& a, const CVector& b) {
        return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
    }
    bool Normalize(CVector& value) {
        const float length2 = value.x * value.x + value.y * value.y + value.z * value.z;
        if (!std::isfinite(length2) || length2 < 1.0e-8f) return false;
        const float inv = 1.0f / std::sqrt(length2);
        value.x *= inv; value.y *= inv; value.z *= inv;
        return true;
    }
    CVector TransformOffset(const CMatrix& matrix, float x, float y, float z) {
        const CVector& right = matrix.GetRight();
        const CVector& forward = matrix.GetForward();
        const CVector& up = matrix.GetUp();
        const CVector& origin = matrix.GetPosition();
        return {origin.x + right.x * x + forward.x * y + up.x * z,
                origin.y + right.y * x + forward.y * y + up.y * z,
                origin.z + right.z * x + forward.z * y + up.z * z};
    }
    bool Enabled() {
        return VehicleLightGL::GetSettings().dynamicLight != 0;
    }
    void BeginFrame() {}

    // Once per game frame: smooth brightness, pick the best slots with hysteresis.
    void Step() {
        const unsigned frame = CTimer::m_FrameCounter;
        if (frame == g_frame) return;
        g_frame = frame;
        const uint32_t now = CTimer::GetTimeMS();
        const float dt = std::max(0.0f, std::min(0.1f, static_cast<float>(now - g_stepMs) * 0.001f));
        g_stepMs = now;
        const CVector camera = TheCamera.GetPosition();
        for (Tracked& t : g_tracked) {
            if (!t.used) continue;
            const unsigned age = frame - t.seen;
            const float eff = age <= kHoldFrames ? t.base : 0.0f;
            if (dt > 0.0f) {
                const float tau = eff > t.shown ? 0.10f : 0.35f;
                t.shown += (eff - t.shown) * (1.0f - std::exp(-dt / tau));
            }
            if (t.shown < 0.008f && age > kHoldFrames) { t = Tracked{}; continue; }
            const float dx = t.light.posRange[0] - camera.x, dy = t.light.posRange[1] - camera.y, dz = t.light.posRange[2] - camera.z;
            const float d = std::sqrt(dx * dx + dy * dy + dz * dz);
            t.rank = t.shown * (0.35f + 0.65f * Clamp01(1.0f - d / kCaptureDistance)) * (t.selected ? 1.35f : 1.0f);
        }
        // pick the best kMaxWorldLights by rank
        int order[kMaxTracked], n = 0;
        for (int i = 0; i < kMaxTracked; ++i) if (g_tracked[i].used) order[n++] = i;
        std::sort(order, order + n, [](int a, int b) { return g_tracked[a].rank > g_tracked[b].rank; });
        for (int i = 0; i < n; ++i) g_tracked[order[i]].selected = i < kMaxWorldLights && g_tracked[order[i]].rank > 0.01f;
        for (Tracked& t : g_tracked) {
            if (!t.used) continue;
            const float goal = t.selected ? 1.0f : 0.0f;
            if (dt > 0.0f) t.slotGain += (goal - t.slotGain) * (1.0f - std::exp(-dt / (goal > t.slotGain ? 0.15f : 0.30f)));
        }
    }

    void Publish() {
        Step();
        int order[kMaxTracked], n = 0;
        for (int i = 0; i < kMaxTracked; ++i)
            if (g_tracked[i].used && g_tracked[i].shown * g_tracked[i].slotGain > 0.01f) order[n++] = i;
        std::sort(order, order + n, [](int a, int b) { return g_tracked[a].rank > g_tracked[b].rank; });
        g_count = std::min(n, kMaxPublished);
        for (int i = 0; i < g_count; ++i) {
            const Tracked& t = g_tracked[order[i]];
            g_lights[i] = t.light;
            g_lights[i].upGain[3] = t.shown * t.slotGain;
        }
        const CVector camera = TheCamera.GetPosition();
        const float cameraWorld[3] = {camera.x, camera.y, camera.z};
        VehicleLightGL::PublishWorld(g_lights, g_count, cameraWorld);
    }
    void AddWorldLight(const CVector& position, const CVector& direction, bool spot,
                       float range, float red, float green, float blue, float strength,
                       bool castsShadow) {
        if (!Enabled() || !IsFinite(position) || !std::isfinite(range) || !std::isfinite(strength)) return;
        const CVector camera = TheCamera.GetPosition();
        const float dx = position.x - camera.x, dy = position.y - camera.y, dz = position.z - camera.z;
        const float distance2 = dx * dx + dy * dy + dz * dz;
        if (!std::isfinite(distance2) || distance2 > kCaptureDistance * kCaptureDistance) return;

        const VehicleLightGL::Settings& settings = VehicleLightGL::GetSettings();
        strength = Clamp01(strength) * settings.world;
        if (strength < 0.015f) return;
        range = std::max(4.0f, std::min(32.0f, range));

        CVector fwd = direction;
        if (!spot || !Normalize(fwd)) fwd = CVector(0.0f, 0.0f, 1.0f);
        CVector right = std::fabs(fwd.z) < 0.9f ? Cross3(fwd, CVector(0.0f, 0.0f, 1.0f))
                                                : Cross3(fwd, CVector(0.0f, 1.0f, 0.0f));
        if (!Normalize(right)) right = CVector(1.0f, 0.0f, 0.0f);
        CVector up = Cross3(right, fwd);
        if (!Normalize(up)) up = CVector(0.0f, 0.0f, 1.0f);

        const float distance = std::sqrt(distance2);
        const float score = strength * (0.35f + 0.65f * Clamp01(1.0f - distance / kCaptureDistance));
        const unsigned frame = CTimer::m_FrameCounter;
        int slot = -1, weakest = -1;
        for (int i = 0; i < kMaxTracked; ++i) {
            const Tracked& t = g_tracked[i];
            if (!t.used) { if (weakest < 0 || g_tracked[weakest].used) weakest = i; continue; }
            const float ox = t.light.posRange[0] - position.x, oy = t.light.posRange[1] - position.y, oz = t.light.posRange[2] - position.z;
            if (ox * ox + oy * oy + oz * oz < 0.49f) { slot = i; break; }
            if (weakest < 0 || (g_tracked[weakest].used && t.rank < g_tracked[weakest].rank)) weakest = i;
        }
        if (slot < 0) {
            if (weakest < 0) return;
            if (g_tracked[weakest].used && score <= g_tracked[weakest].rank) return;     // not better than the weakest kept light
            slot = weakest;
            g_tracked[slot] = Tracked{};
            g_tracked[slot].used = true;
        } else if (g_tracked[slot].seen == frame && strength < g_tracked[slot].base * 0.55f) {
            return;      // same-spot light already registered this frame with a clearly stronger value
        }

        Tracked& tr = g_tracked[slot];
        VehicleLightGL::Light& light = tr.light;
        light = VehicleLightGL::Light{};
        light.posRange[0] = position.x; light.posRange[1] = position.y; light.posRange[2] = position.z; light.posRange[3] = range;
        light.fwdKind[0] = fwd.x; light.fwdKind[1] = fwd.y; light.fwdKind[2] = fwd.z;
        light.fwdKind[3] = spot ? 1.0f : 2.0f;
        light.rightYaw[0] = right.x; light.rightYaw[1] = right.y; light.rightYaw[2] = right.z;
        light.rightYaw[3] = spot ? std::cos(48.0f * 0.01745329252f) : -1.0f;
        light.upGain[0] = up.x; light.upGain[1] = up.y; light.upGain[2] = up.z; light.upGain[3] = strength;
        light.boxC[0] = position.x; light.boxC[1] = position.y; light.boxC[2] = position.z;
        light.boxC[3] = spot ? std::cos(18.0f * 0.01745329252f) : 1.0f;
        light.boxE[0] = range; light.boxE[1] = range; light.boxE[2] = range; light.boxE[3] = castsShadow ? 1.0f : 0.0f;
        light.color[0] = Clamp01(red); light.color[1] = Clamp01(green); light.color[2] = Clamp01(blue); light.color[3] = 0.0f;
        if (tr.seen != frame || strength > tr.base) tr.base = strength;
        tr.seen = frame;
        ++g_accepted;
    }

    void HookAddLight(uint8_t type, CVector position, CVector direction, float range,
                      float red, float green, float blue, uint8_t fogEffect,
                      bool castsShadow, CEntity* source) {
        if (g_originalAddLight)
            g_originalAddLight(type, position, direction, range, red, green, blue, fogEffect, castsShadow, source);
        if (!Enabled() || type == PLTYPE_DARKLIGHT || (source && source->IsVehicle())) return;
        BeginFrame();
        AddWorldLight(position, direction, type == PLTYPE_SPOTLIGHT, range,
                      red, green, blue, std::max(red, std::max(green, blue)), castsShadow);
    }

    bool IsDynamicCorona(eCoronaType type, eCoronaFlareType flare) {
        return flare != FLARETYPE_SUN && type != CORONATYPE_MOON && type != CORONATYPE_REFLECTION;
    }
    void HookRegisterCorona(uintptr_t id, CEntity* attached, uint8_t red, uint8_t green, uint8_t blue,
                            uint8_t intensity, const CVector& coordinates, float size, float range,
                            eCoronaType coronaType, eCoronaFlareType flareType, uint8_t reflectionType,
                            uint8_t losCheck, uint8_t usesTrails, float normalAngle, bool neonFade,
                            float pullTowardsCamera, bool fullBrightAtStart, float fadeSpeed,
                            bool onlyFromBelow, bool whiteCore) {
        if (g_originalRegisterCorona)
            g_originalRegisterCorona(id, attached, red, green, blue, intensity, coordinates, size, range,
                                     coronaType, flareType, reflectionType, losCheck, usesTrails,
                                     normalAngle, neonFade, pullTowardsCamera, fullBrightAtStart,
                                     fadeSpeed, onlyFromBelow, whiteCore);
        if (!Enabled() || intensity < 8 || !IsDynamicCorona(coronaType, flareType) || (attached && attached->IsVehicle())) return;
        BeginFrame();
        CVector world = coordinates;
        if (attached && attached->GetMatrix())
            world = TransformOffset(*attached->GetMatrix(), coordinates.x, coordinates.y, coordinates.z);
        AddWorldLight(world, CVector(0.0f, 0.0f, 0.0f), false, std::max(6.0f, range),
                      static_cast<float>(red) / 255.0f,
                      static_cast<float>(green) / 255.0f,
                      static_cast<float>(blue) / 255.0f,
                      static_cast<float>(intensity) / 255.0f * 0.80f, losCheck != 0);
    }

    bool TryHook(const char* symbol, void* detour, void** original) {
        void* address = GetMainLibrarySymbol<void*>(symbol);
        const bool ok = address && aml && aml->Hook(address, detour, original);
        logger->Info("DynamicLight hook %-54s %s", symbol, ok ? "OK" : (address ? "FAILED" : "missing"));
        return ok;
    }

    void Summary() {
        const uint32_t now = CTimer::GetTimeMS();
        if (now - g_lastSummary < 3000) return;
        g_lastSummary = now;
        logger->Info("DynamicLight: active=%d lights=%d accepted=%u addHook=%d coronaHook=%d",
                     Enabled() ? 1 : 0, g_count, g_accepted,
                     g_addLightHookInstalled ? 1 : 0, g_coronaHookInstalled ? 1 : 0);
    }
}

void Initialize() {
    g_addLightHookInstalled = TryHook("_ZN12CPointLights8AddLightEh7CVectorS0_ffffhbP7CEntity",
        reinterpret_cast<void*>(HookAddLight), reinterpret_cast<void**>(&g_originalAddLight));
    g_coronaHookInstalled = TryHook("_ZN8CCoronas14RegisterCoronaEyP7CEntityhhhhRK7CVectorffhhhhhfbfbfbb",
        reinterpret_cast<void*>(HookRegisterCorona), reinterpret_cast<void**>(&g_originalRegisterCorona));
    Events::gameProcessEvent.after += []() { Publish(); };
    Events::renderEffectsEvent.after += []() { Publish(); };
    Events::renderCoronasEvent.after += []() { Publish(); Summary(); };
    logger->Info("DynamicLight initialized (world lights: AddLight=%d Corona=%d)",
                 g_addLightHookInstalled ? 1 : 0, g_coronaHookInstalled ? 1 : 0);
}
}
