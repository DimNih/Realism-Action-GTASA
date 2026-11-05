#include "camera_garage_system.h"

#include <mod/amlmod.h>
#include <mod/logger.h>

#include <aml-psdk/game_sa/Events.h>
#include <aml-psdk/game_sa/entity/Camera.h>
#include <aml-psdk/game_sa/entity/PlayerPed.h>

#include <algorithm>
#include <cmath>

namespace CameraGarageSystem {
namespace {

using GarageCameraOutsideFn = bool (*)();
GarageCameraOutsideFn g_originalCameraOutside = nullptr;
using IsPointWithinAnyGarageFn = bool (*)(CVector*);
IsPointWithinAnyGarageFn g_isPointWithinAnyGarage = nullptr;
bool g_cameraOutsideHook = false;
bool g_loggedInside = false;

float LengthSquared(const CVector& v) { return v.x * v.x + v.y * v.y + v.z * v.z; }
bool Finite(const CVector& v) { return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z); }

bool IsInsideGarage(CCamera* camera, CPlayerPed* player) {
    if (!camera || !player) return false;
    // These are fields maintained by CCamera itself. Avoid including the
    // repository's Garages.h here: its 64-bit CGarage CHECKSIZE is stale on
    // this AML ABI and would reject the entire module at compile time.
    if (camera->m_bPlayerIsInGarage || camera->m_pToGarageWeAreIn != nullptr) return true;
    CVector position = player->GetPosition();
    return g_isPointWithinAnyGarage && g_isPointWithinAnyGarage(&position);
}

bool HookCameraShouldBeOutside() {
    CPlayerPed* player = FindPlayerPed(0);
    // Do not call the garage query from inside this hook: some game builds
    // reach CameraShouldBeOutside while the garage list is being updated.
    // The camera's own fields are sufficient to stop the exterior override.
    if (player && (TheCamera.m_bPlayerIsInGarage || TheCamera.m_pToGarageWeAreIn != nullptr)) return false;
    return g_originalCameraOutside ? g_originalCameraOutside() : true;
}

void BuildCameraMatrix(CCamera* camera, const CVector& source, const CVector& target) {
    CVector forward(target.x - source.x, target.y - source.y, target.z - source.z);
    const float length2 = LengthSquared(forward);
    if (!std::isfinite(length2) || length2 < 0.04f) return;
    const float inv = 1.0f / std::sqrt(length2);
    forward.x *= inv; forward.y *= inv; forward.z *= inv;

    // CMatrix uses right, forward (named `up` in RenderWare/GTA), up (named
    // `at`). This produces a stable upright camera without roll inside rooms.
    CVector right(forward.y, -forward.x, 0.0f);
    const float right2 = LengthSquared(right);
    if (right2 < 1.0e-6f) right = CVector(1.0f, 0.0f, 0.0f);
    else {
        const float ri = 1.0f / std::sqrt(right2);
        right.x *= ri; right.y *= ri;
    }
    CVector up(right.y * forward.z - right.z * forward.y,
               right.z * forward.x - right.x * forward.z,
               right.x * forward.y - right.y * forward.x);

    CMatrix& matrix = camera->m_mCameraMatrix;
    matrix.right = right;
    matrix.up = forward;
    matrix.at = up;
    matrix.pos = source;
    camera->CopyCameraMatrixToRWCam(false);
}

void KeepCameraInside(CCamera* camera) {
    if (!camera || camera != &TheCamera) return;
    CPlayerPed* player = FindPlayerPed(0);
    if (!IsInsideGarage(camera, player)) return;

    const eCamMode mode = camera->GetActiveMode();
    const bool normalGameplayMode = mode == MODE_BEHINDCAR || mode == MODE_FOLLOWPED ||
        mode == MODE_GTACLASSIC || mode == MODE_AIMING || mode == MODE_AIMWEAPON_FROMCAR ||
        mode == MODE_WHEELCAM || mode == MODE_1STPERSON || mode == MODE_1STPERSON_RUNABOUT;
    if (!normalGameplayMode || camera->m_bFading || camera->m_bWideScreenOn) return;

    const CVector playerPos = player->GetPosition();
    CVector target(playerPos.x, playerPos.y, playerPos.z + 0.78f);
    CVector source = camera->m_mCameraMatrix.GetPosition();
    if (!Finite(source) || !Finite(playerPos)) return;

    const bool hasGaragePointer = camera->m_pToGarageWeAreIn != nullptr;
    const float maxDistance = hasGaragePointer ? 6.0f : 7.5f;
    bool outsideBounds = LengthSquared(CVector(source.x - playerPos.x,
                                               source.y - playerPos.y,
                                               source.z - playerPos.z)) > maxDistance * maxDistance;
    if (outsideBounds) {
        // Use the normal follow-camera side of CJ. This remains inside the
        // garage even when the game has not exposed the CGarage bounds yet.
        CVector backward(-player->GetForward().x, -player->GetForward().y, 0.0f);
        const float b2 = LengthSquared(backward);
        if (b2 > 0.04f) {
            const float inv = 1.0f / std::sqrt(b2);
            source = CVector(playerPos.x + backward.x * inv * 4.2f,
                             playerPos.y + backward.y * inv * 4.2f,
                             playerPos.z + 1.25f);
        } else {
            source = CVector(playerPos.x, playerPos.y - 4.2f, playerPos.z + 1.25f);
        }
    }
    if (!outsideBounds) return;

    CCam& active = camera->GetActiveCam();
    active.m_vecSource = source;
    active.m_vecTargetCoorsForFudgeInter = target;
    active.m_fCameraDistance = std::sqrt(LengthSquared(CVector(source.x - target.x, source.y - target.y, source.z - target.z)));
    BuildCameraMatrix(camera, source, target);
    if (!g_loggedInside) {
        g_loggedInside = true;
        logger->Info("GarageCamera: exterior camera corrected to interior follow position");
    }
}

} // namespace

void Initialize() {
    void* symbol = GetMainLibrarySymbol<void*>("_ZN8CGarages21CameraShouldBeOutsideEv");
    g_cameraOutsideHook = symbol && aml && aml->Hook(symbol, reinterpret_cast<void*>(HookCameraShouldBeOutside),
                                                      reinterpret_cast<void**>(&g_originalCameraOutside));
    g_isPointWithinAnyGarage = GetMainLibrarySymbol<IsPointWithinAnyGarageFn>("_ZN8CGarages22IsPointWithinAnyGarageER7CVector");
    Events::processCameraEvent.after += [](CCamera* camera) { KeepCameraInside(camera); };
    logger->Info("GarageCamera initialized (CameraShouldBeOutside hook=%d)", g_cameraOutsideHook ? 1 : 0);
}

} // namespace CameraGarageSystem
