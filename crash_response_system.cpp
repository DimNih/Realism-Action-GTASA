#include "crash_response_system.h"

#include <mod/amlmod.h>
#include <android/log.h>

#include <aml-psdk/game_sa/engine/World.h>
#include <aml-psdk/game_sa/entity/PlayerPed.h>
#include <aml-psdk/game_sa/entity/Vehicle.h>
#include <aml-psdk/game_sa/entity/Ped.h>
#include <aml-psdk/game_sa/other/Pools.h>
#include <aml-psdk/game_sa/utils/OpcodeCaller.h>

#include <math.h>
#include <stdarg.h>
#include <stdio.h>

namespace CrashResponseSystem {
namespace {
    constexpr float kApproachRadius = 85.0f;
    constexpr float kArrivalDistance = 8.0f;
    constexpr float kMaxStoppedTrafficSpeedKmh = 2.0f;
    constexpr int kMaxFootResponders = 8;
    constexpr int kMaxVehicleResponders = 4;
    constexpr int kMaxResponders = kMaxFootResponders + kMaxVehicleResponders;
    constexpr int kNearbyEntityCapacity = 128;
    constexpr int kDiagnosticIntervalFrames = 60;
    constexpr int kCooldownFrames = 300;
    constexpr int kExitTimeoutFrames = 300;
    constexpr int kApproachTimeoutFrames = 1800;
    constexpr int kReactionFrames = 720;
    constexpr int kReturnTimeoutFrames = 900;
    constexpr int kApproachMoveState = 6;
    constexpr int kApproachTaskTimeMs = 60000;
    constexpr int kReactionTimeMs = 12000;

    enum class ResponderState : unsigned char {
        ExitingVehicle,
        ApproachingScene,
        Reacting,
        ReturningToVehicle,
        Finished
    };

    struct Responder {
        int pedRef = -1;
        int vehicleRef = -1;
        int passengerSeat = -1;
        int framesInState = 0;
        bool wasDriver = false;
        bool isGangMember = false;
        ResponderState state = ResponderState::Finished;
    };

    Responder g_responders[kMaxResponders] = {};
    int g_responderCount = 0;
    int g_diagnosticFrames = 0;
    int g_cooldownFrames = 0;
    int g_playerVehicleRef = -1;
    float g_previousSpeedKmh = 0.0f;
    float g_previousVehicleHealth = 1000.0f;
    float g_previousCollisionImpactKmh = 0.0f;
    unsigned int g_previousCollisionTime = 0;
    bool g_previousHitWall = false;
    bool g_pendingImpact = false;
    int g_responseVehicleRef = -1;
    int g_reactedPedCount = 0;
    int g_monitorState = -1;
    FILE* g_crashLogFile = nullptr;
    CVector g_scenePosition(0.0f, 0.0f, 0.0f);

    struct FootCandidate {
        int pedRef = -1;
        float distanceSquared = 0.0f;
        bool isGangMember = false;
    };

    void CrashLog(int priority, const char* format, ...) {
        char message[512];
        va_list args;
        va_start(args, format);
        vsnprintf(message, sizeof(message), format, args);
        va_end(args);
        __android_log_write(priority, "CrashResponse", message);
        if (g_crashLogFile) {
            fprintf(g_crashLogFile, "%s\n", message);
            fflush(g_crashLogFile);
        }
    }

#define CRASH_INFO(...) CrashLog(ANDROID_LOG_INFO, "[CrashResponse] " __VA_ARGS__)
#define CRASH_WARN(...) CrashLog(ANDROID_LOG_WARN, "[CrashResponse] " __VA_ARGS__)
#define CRASH_ERROR(...) CrashLog(ANDROID_LOG_ERROR, "[CrashResponse] " __VA_ARGS__)

    bool IsAvailablePed(CPed* ped, CPlayerPed* player) {
        if (!ped || ped == player || ped->bInVehicle || ped->GetHealth() <= 0.0f ||
            ped->bFallenDown || ped->m_ePedState == PEDSTATE_DIE ||
            ped->m_ePedState == PEDSTATE_DEAD || ped->m_ePedState == PEDSTATE_FALL ||
            ped->m_ePedState == PEDSTATE_GETUP || ped->m_ePedState == PEDSTATE_ATTACK ||
            ped->m_ePedState == PEDSTATE_FIGHT) {
            return false;
        }
        return true;
    }

    bool IsGangMember(CPed* ped) {
        return ped && ped->m_nPedType >= PED_TYPE_GANG1 &&
               ped->m_nPedType <= PED_TYPE_GANG10;
    }

    bool IsAlreadyAssigned(int pedRef) {
        for (int i = 0; i < g_responderCount; ++i) {
            if (g_responders[i].pedRef == pedRef) return true;
        }
        return false;
    }

    float DistanceSquaredToScene(CPed* ped) {
        const CVector& position = ped->GetPosition();
        const float x = position.x - g_scenePosition.x;
        const float y = position.y - g_scenePosition.y;
        const float z = position.z - g_scenePosition.z;
        return x * x + y * y + z * z;
    }

    void AddFootCandidate(FootCandidate* candidates, int& count, CPed* ped,
                          CPlayerPed* player) {
        if (!IsAvailablePed(ped, player)) return;
        const int pedRef = CPools::GetPedRef(ped);
        if (pedRef < 0 || IsAlreadyAssigned(pedRef)) return;
        const float distanceSquared = DistanceSquaredToScene(ped);
        if (distanceSquared > kApproachRadius * kApproachRadius) return;
        int insertAt = count;
        if (insertAt >= kMaxFootResponders) {
            insertAt = kMaxFootResponders - 1;
            if (distanceSquared >= candidates[insertAt].distanceSquared) return;
        }
        while (insertAt > 0 &&
               distanceSquared < candidates[insertAt - 1].distanceSquared) {
            if (insertAt < kMaxFootResponders) {
                candidates[insertAt] = candidates[insertAt - 1];
            }
            --insertAt;
        }
        candidates[insertAt] = { pedRef, distanceSquared, IsGangMember(ped) };
        if (count < kMaxFootResponders) ++count;
    }

    bool IsEligibleVehicleOccupant(CPed* ped, CPlayerPed* player) {
        return ped && ped != player && ped->bInVehicle &&
               ped->GetHealth() > 0.0f && !ped->bFallenDown &&
               ped->m_ePedState != PEDSTATE_DIE && ped->m_ePedState != PEDSTATE_DEAD &&
               ped->m_ePedState != PEDSTATE_FALL && ped->m_ePedState != PEDSTATE_GETUP &&
               ped->m_ePedState != PEDSTATE_ATTACK && ped->m_ePedState != PEDSTATE_FIGHT;
    }

    void StartApproach(int responderIndex) {
        Responder& responder = g_responders[responderIndex];
        static const float kOffsets[kMaxResponders][2] = {
            { 0.0f, 5.5f }, { 5.5f, 0.0f }, { 0.0f, -5.5f }, { -5.5f, 0.0f },
            { 4.0f, 4.0f }, { 4.0f, -4.0f }, { -4.0f, -4.0f }, { -4.0f, 4.0f },
            { 0.0f, 7.5f }, { 7.5f, 0.0f }, { 0.0f, -7.5f }, { -7.5f, 0.0f }
        };
        const int offsetIndex = responderIndex % kMaxResponders;
        const float targetX = g_scenePosition.x + kOffsets[offsetIndex][0];
        const float targetY = g_scenePosition.y + kOffsets[offsetIndex][1];
        Command< TASK_GO_STRAIGHT_TO_COORD >(responder.pedRef, targetX, targetY,
                                              g_scenePosition.z, kApproachMoveState,
                                              kApproachTaskTimeMs);
        responder.state = ResponderState::ApproachingScene;
        responder.framesInState = 0;
        CRASH_INFO("Sent run-to-scene task to pedRef=%d target=(%.1f, %.1f, %.1f)%s",
                   responder.pedRef, targetX, targetY, g_scenePosition.z,
                   responder.isGangMember ? " gang" : "");
    }

    void StartResponse(CPlayerPed* player, CVehicle* vehicle) {
        g_scenePosition = vehicle->GetPosition();
        CEntity* nearbyPeds[kNearbyEntityCapacity] = {};
        i16 foundPeds = 0;
        CWorld::FindObjectsInRange(g_scenePosition, kApproachRadius, true,
                                   &foundPeds, kNearbyEntityCapacity, nearbyPeds,
                                   false, false, true, false, false);

        FootCandidate footCandidates[kMaxFootResponders] = {};
        int footCount = 0;
        for (int i = 0; i < kMaxFootResponders; ++i) {
            footCandidates[i].pedRef = -1;
            footCandidates[i].distanceSquared = kApproachRadius * kApproachRadius;
        }
        for (int i = 0; i < kMaxResponders; ++i) g_responders[i] = {};
        g_responderCount = 0;
        const int playerRef = CPools::GetPedRef(player);
        if (playerRef < 0) {
            CRASH_ERROR("Crash detected, but player pool reference is invalid");
            return;
        }
        g_responseVehicleRef = CPools::GetVehicleRef(vehicle);
        CRASH_INFO("Crash response scan: peds=%d radius=%.0fm",
                   foundPeds, kApproachRadius);

        for (i16 i = 0; i < foundPeds && i < kNearbyEntityCapacity; ++i) {
            CEntity* entity = nearbyPeds[i];
            if (entity && entity->IsPed()) {
                AddFootCandidate(footCandidates, footCount, entity->AsPed(), player);
            }
        }

        for (int i = 0; i < footCount; ++i) {
            Responder& responder = g_responders[g_responderCount];
            responder.pedRef = footCandidates[i].pedRef;
            responder.isGangMember = footCandidates[i].isGangMember;
            CRASH_INFO("Selected on-foot NPC pedRef=%d distance=%.1fm%s",
                       responder.pedRef, sqrtf(footCandidates[i].distanceSquared),
                       responder.isGangMember ? " gang" : "");
            StartApproach(g_responderCount);
            ++g_responderCount;
        }

        CEntity* nearbyVehicles[kNearbyEntityCapacity] = {};
        i16 foundVehicles = 0;
        CWorld::FindObjectsInRange(g_scenePosition, kApproachRadius, true,
                                   &foundVehicles, kNearbyEntityCapacity,
                                   nearbyVehicles, false, true, false, false, false);
        int occupantCount = 0;
        for (i16 i = 0; i < foundVehicles && i < kNearbyEntityCapacity &&
                         occupantCount < kMaxVehicleResponders; ++i) {
            CEntity* entity = nearbyVehicles[i];
            if (!entity || !entity->IsVehicle()) continue;
            CVehicle* nearbyVehicle = entity->AsVehicle();
            if (nearbyVehicle == vehicle ||
                nearbyVehicle->GetSpeedKmh() > kMaxStoppedTrafficSpeedKmh) continue;
            const int vehicleRef = CPools::GetVehicleRef(nearbyVehicle);
            if (vehicleRef < 0) continue;

            CPed* driver = nearbyVehicle->m_pDriver;
            if (IsEligibleVehicleOccupant(driver, player)) {
                const int pedRef = CPools::GetPedRef(driver);
                if (pedRef >= 0 && !IsAlreadyAssigned(pedRef)) {
                    Responder& responder = g_responders[g_responderCount++];
                    responder.pedRef = pedRef;
                    responder.vehicleRef = vehicleRef;
                    responder.wasDriver = true;
                    responder.isGangMember = IsGangMember(driver);
                    responder.state = ResponderState::ExitingVehicle;
                    Command< TASK_LEAVE_CAR >(pedRef, vehicleRef);
                    ++occupantCount;
                    CRASH_INFO("NPC pedRef=%d leaving vehicle=%d from driver seat%s",
                               pedRef, vehicleRef,
                               responder.isGangMember ? " gang" : "");
                }
            }
            for (int seat = 0; seat < nearbyVehicle->m_nNumPassengers &&
                               seat < 8 && occupantCount < kMaxVehicleResponders;
                 ++seat) {
                CPed* passenger = nearbyVehicle->m_apPassengers[seat];
                if (!IsEligibleVehicleOccupant(passenger, player)) continue;
                const int pedRef = CPools::GetPedRef(passenger);
                if (pedRef < 0 || IsAlreadyAssigned(pedRef)) continue;
                Responder& responder = g_responders[g_responderCount++];
                responder.pedRef = pedRef;
                responder.vehicleRef = vehicleRef;
                responder.passengerSeat = seat;
                responder.isGangMember = IsGangMember(passenger);
                responder.state = ResponderState::ExitingVehicle;
                Command< TASK_LEAVE_CAR >(pedRef, vehicleRef);
                ++occupantCount;
                CRASH_INFO("NPC pedRef=%d leaving vehicle=%d from passenger seat %d%s",
                           pedRef, vehicleRef, seat,
                           responder.isGangMember ? " gang" : "");
            }
        }

        g_reactedPedCount = 0;
        if (g_responderCount == 0) {
            CRASH_WARN("No eligible pedestrian or stopped-vehicle occupant found");
        } else {
            CRASH_INFO("Response started; assigned=%d on-foot=%d vehicleOccupants=%d",
                       g_responderCount, footCount, occupantCount);
        }
    }

    void ProcessResponders(CPlayerPed* player) {
        if (g_responderCount == 0) return;
        const int playerRef = player ? CPools::GetPedRef(player) : -1;
        if (playerRef < 0) {
            CRASH_WARN("Player reference lost; clearing active response");
            for (int i = 0; i < g_responderCount; ++i) {
                Responder& responder = g_responders[i];
                CPed* ped = CPools::GetPed(responder.pedRef);
                if (!ped) continue;
                if (responder.vehicleRef >= 0 && !ped->bInVehicle) {
                    CVehicle* vehicle = CPools::GetVehicle(responder.vehicleRef);
                    if (vehicle && responder.wasDriver) {
                        Command< TASK_ENTER_CAR_AS_DRIVER >(responder.pedRef,
                                                            responder.vehicleRef, 30000);
                    } else if (vehicle && responder.passengerSeat >= 0) {
                        Command< TASK_ENTER_CAR_AS_PASSENGER >(
                            responder.pedRef, responder.vehicleRef, 30000,
                            responder.passengerSeat);
                    } else {
                        Command< TASK_WANDER_STANDARD >(responder.pedRef, 4);
                    }
                } else if (responder.vehicleRef < 0) {
                    Command< TASK_WANDER_STANDARD >(responder.pedRef, 4);
                }
            }
            g_responderCount = 0;
            return;
        }

        for (int i = 0; i < kMaxResponders; ++i) {
            Responder& responder = g_responders[i];
            if (responder.pedRef < 0 || responder.state == ResponderState::Finished) continue;
            CPed* ped = CPools::GetPed(responder.pedRef);
            if (!ped) {
                CRASH_WARN("Assigned pedRef=%d disappeared", responder.pedRef);
                responder.state = ResponderState::Finished;
                continue;
            }

            ++responder.framesInState;
            switch (responder.state) {
                case ResponderState::ExitingVehicle:
                    if (!ped->bInVehicle) {
                        CRASH_INFO("NPC pedRef=%d exited vehicle=%d; moving to crash scene",
                                   responder.pedRef, responder.vehicleRef);
                        StartApproach(i);
                    } else if (responder.framesInState >= kExitTimeoutFrames) {
                        CRASH_WARN("NPC pedRef=%d could not exit vehicle=%d; leaving task alone",
                                   responder.pedRef, responder.vehicleRef);
                        responder.state = ResponderState::Finished;
                    }
                    break;

                case ResponderState::ApproachingScene:
                    if (DistanceSquaredToScene(ped) <= kArrivalDistance * kArrivalDistance) {
                        responder.state = ResponderState::Reacting;
                        responder.framesInState = 0;
                        ++g_reactedPedCount;
                        if (responder.isGangMember) {
                            Command< TASK_SHAKE_FIST >(responder.pedRef);
                            CRASH_INFO("Gang NPC pedRef=%d reached the scene; angry reaction started",
                                       responder.pedRef);
                        } else {
                            Command< TASK_SCRATCH_HEAD >(responder.pedRef);
                            CRASH_INFO("NPC pedRef=%d reached the scene; confused reaction started",
                                       responder.pedRef);
                        }
                    } else if (responder.framesInState >= kApproachTimeoutFrames) {
                        CRASH_WARN("NPC pedRef=%d did not reach the scene before timeout",
                                   responder.pedRef);
                        responder.state = ResponderState::Finished;
                        Command< TASK_WANDER_STANDARD >(responder.pedRef, 4);
                    }
                    break;

                case ResponderState::Reacting:
                    if (responder.framesInState == 90) {
                        Command< TASK_PLAY_ANIM >(responder.pedRef, "IDLE_CHAT", "PED",
                                                  4.0f, 1, 0, 0, 0, kReactionTimeMs);
                        CRASH_INFO("NPC pedRef=%d started extended scene interaction%s",
                                   responder.pedRef,
                                   responder.isGangMember ? " (gang member)" : "");
                    }
                    if (responder.framesInState == 360) {
                        // Run the look reaction after the chat gesture so the
                        // two primary tasks do not replace each other.
                        Command< TASK_LOOK_AT_CHAR >(responder.pedRef, playerRef,
                                                     4000, 0);
                    }
                    if (responder.framesInState >= kReactionFrames) {
                        if (responder.vehicleRef >= 0) {
                            CVehicle* originalVehicle = CPools::GetVehicle(responder.vehicleRef);
                            if (originalVehicle &&
                                originalVehicle->GetSpeedKmh() <= kMaxStoppedTrafficSpeedKmh) {
                                if (responder.wasDriver) {
                                    Command< TASK_ENTER_CAR_AS_DRIVER >(
                                        responder.pedRef, responder.vehicleRef, 30000);
                                } else {
                                    Command< TASK_ENTER_CAR_AS_PASSENGER >(
                                        responder.pedRef, responder.vehicleRef, 30000,
                                        responder.passengerSeat);
                                }
                                responder.state = ResponderState::ReturningToVehicle;
                                responder.framesInState = 0;
                                CRASH_INFO("NPC pedRef=%d returning to original vehicle=%d",
                                           responder.pedRef, responder.vehicleRef);
                            } else {
                                CRASH_WARN("NPC pedRef=%d original vehicle moved; returning to normal AI",
                                           responder.pedRef);
                                Command< TASK_WANDER_STANDARD >(responder.pedRef, 4);
                                responder.state = ResponderState::Finished;
                            }
                        } else {
                            Command< TASK_WANDER_STANDARD >(responder.pedRef, 4);
                            responder.state = ResponderState::Finished;
                            CRASH_INFO("NPC pedRef=%d finished checking the crash scene",
                                       responder.pedRef);
                        }
                    }
                    break;

                case ResponderState::ReturningToVehicle: {
                    CVehicle* originalVehicle = CPools::GetVehicle(responder.vehicleRef);
                    if (ped->bInVehicle && ped->m_pVehicle == originalVehicle) {
                        CRASH_INFO("NPC pedRef=%d returned to vehicle=%d",
                                   responder.pedRef, responder.vehicleRef);
                        responder.state = ResponderState::Finished;
                    } else if (responder.framesInState >= kReturnTimeoutFrames) {
                        CRASH_WARN("NPC pedRef=%d could not re-enter vehicle=%d; resuming normal AI",
                                   responder.pedRef, responder.vehicleRef);
                        Command< TASK_WANDER_STANDARD >(responder.pedRef, 4);
                        responder.state = ResponderState::Finished;
                    }
                    break;
                }

                case ResponderState::Finished:
                    break;
            }
        }

        bool allFinished = true;
        for (int i = 0; i < g_responderCount; ++i) {
            if (g_responders[i].state != ResponderState::Finished) {
                allFinished = false;
                break;
            }
        }
        if (allFinished) {
            CRASH_INFO("Response finished; assigned=%d, reached=%d",
                       g_responderCount, g_reactedPedCount);
            for (int i = 0; i < kMaxResponders; ++i) g_responders[i] = {};
            g_responderCount = 0;
        }
    }
}

void Initialize() {
    if (aml) {
        const char* androidDataPath = aml->GetAndroidDataPath();
        if (androidDataPath && androidDataPath[0]) {
            char logPath[512];
            snprintf(logPath, sizeof(logPath), "%s/CrashResponse.log", androidDataPath);
            g_crashLogFile = fopen(logPath, "a");
            if (g_crashLogFile) {
                fprintf(g_crashLogFile, "\n--- Crash response session started (GTA SA Android 2.10 / ARM64) ---\n");
                fflush(g_crashLogFile);
            } else {
                __android_log_print(ANDROID_LOG_ERROR, "CrashResponse",
                                    "[CrashResponse] Cannot open log file: %s", logPath);
            }
        } else {
            __android_log_write(ANDROID_LOG_ERROR, "CrashResponse",
                                "[CrashResponse] AML did not provide Android data path");
        }
    } else {
        __android_log_write(ANDROID_LOG_ERROR, "CrashResponse",
                            "[CrashResponse] AML interface is unavailable; file logging disabled");
    }
    CRASH_INFO("Crash response module loaded (GTA SA Android 2.10 / ARM64)");
}

void Process(CPlayerPed* player) {
    if (g_cooldownFrames > 0) --g_cooldownFrames;
    ProcessResponders(player);
    if (!player || !player->bInVehicle || !player->m_pVehicle) {
        if (g_monitorState != 0) {
            CRASH_INFO("Monitoring idle: player is not inside a vehicle");
            g_monitorState = 0;
        }
        g_playerVehicleRef = -1;
        g_previousSpeedKmh = 0.0f;
        g_previousVehicleHealth = 1000.0f;
        g_previousCollisionImpactKmh = 0.0f;
        g_previousCollisionTime = 0;
        g_previousHitWall = false;
        g_pendingImpact = false;
        return;
    }

    CVehicle* vehicle = player->m_pVehicle;
    if (vehicle->m_pDriver != player) {
        if (g_monitorState != 1) {
            CRASH_WARN("Monitoring idle: player is inside a vehicle but is not its driver");
            g_monitorState = 1;
        }
        g_playerVehicleRef = -1;
        g_previousSpeedKmh = 0.0f;
        g_previousVehicleHealth = 1000.0f;
        g_previousCollisionImpactKmh = 0.0f;
        g_previousCollisionTime = 0;
        g_previousHitWall = false;
        g_pendingImpact = false;
        return;
    }
    if (g_monitorState != 2) {
        CRASH_INFO("Monitoring enabled: player is driving a vehicle");
        g_monitorState = 2;
    }

    const int vehicleRef = CPools::GetVehicleRef(vehicle);
    const float speedKmh = vehicle->GetSpeedKmh();
    if (vehicleRef != g_playerVehicleRef) {
        g_playerVehicleRef = vehicleRef;
        g_previousSpeedKmh = speedKmh;
        g_previousVehicleHealth = vehicle->m_fHealth;
        g_previousCollisionImpactKmh =
            vehicle->m_vecLastCollisionImpactVelocity.Magnitude() * 200.0f;
        g_previousCollisionTime = vehicle->m_nLastCollisionTime;
        g_previousHitWall = vehicle->bHasHitWall;
        CRASH_INFO("Baseline set: vehicleRef=%d speed=%.1f km/h health=%.1f collisionTime=%u",
                   vehicleRef, speedKmh, vehicle->m_fHealth,
                   g_previousCollisionTime);
        if (g_pendingImpact && g_cooldownFrames == 0 && g_responderCount == 0) {
            g_pendingImpact = false;
            g_cooldownFrames = kCooldownFrames;
            StartResponse(player, vehicle);
        }
        return;
    }

    const float previousSpeedKmh = g_previousSpeedKmh;
    const float speedDropKmh = g_previousSpeedKmh - speedKmh;
    const float vehicleDamage = g_previousVehicleHealth - vehicle->m_fHealth;
    const bool newCollision = vehicle->m_nLastCollisionTime != g_previousCollisionTime;
    const float collisionImpactKmh = vehicle->m_vecLastCollisionImpactVelocity.Magnitude() * 200.0f;
    const bool wallHitStarted = vehicle->bHasHitWall && !g_previousHitWall;
    const bool impactVelocityChanged = collisionImpactKmh >= 1.0f &&
        fabsf(collisionImpactKmh - g_previousCollisionImpactKmh) >= 1.0f;
    const bool impactDetected = newCollision || wallHitStarted ||
                                vehicleDamage > 0.0f || impactVelocityChanged;
    if (newCollision) {
        CRASH_INFO("Collision timestamp changed: time=%u impact=%.1f km/h intensity=%.1f damage=%.1f",
                   vehicle->m_nLastCollisionTime, collisionImpactKmh,
                   vehicle->m_fDamageIntensity, vehicleDamage);
    }
    if (++g_diagnosticFrames >= kDiagnosticIntervalFrames) {
        g_diagnosticFrames = 0;
        CRASH_INFO("Monitor: speed=%.1f km/h previous=%.1f drop=%.1f wall=%d collision=%d impact=%.1f intensity=%.1f damage=%.1f cooldown=%d",
                     speedKmh, g_previousSpeedKmh, speedDropKmh,
                     static_cast<int>(vehicle->bHasHitWall),
                     static_cast<int>(newCollision), collisionImpactKmh,
                     vehicle->m_fDamageIntensity, vehicleDamage, g_cooldownFrames);
    }
    g_previousSpeedKmh = speedKmh;
    g_previousVehicleHealth = vehicle->m_fHealth;
    g_previousCollisionImpactKmh = collisionImpactKmh;
    g_previousCollisionTime = vehicle->m_nLastCollisionTime;
    g_previousHitWall = vehicle->bHasHitWall;
    if (impactDetected) {
        CRASH_INFO("Collision detected: previousSpeed=%.1f speedDrop=%.1f wall=%d wallStarted=%d collision=%d impact=%.1f intensity=%.1f vehicleDamage=%.1f",
                     previousSpeedKmh, speedDropKmh,
                     static_cast<int>(vehicle->bHasHitWall),
                     static_cast<int>(wallHitStarted), static_cast<int>(newCollision), collisionImpactKmh,
                     vehicle->m_fDamageIntensity,
                     vehicleDamage);
        g_pendingImpact = true;
    }
    if (g_pendingImpact && g_cooldownFrames == 0 && g_responderCount == 0) {
        g_pendingImpact = false;
        g_cooldownFrames = kCooldownFrames;
        StartResponse(player, vehicle);
    } else if (impactDetected) {
        CRASH_INFO("Collision queued until responders/cooldown clear (cooldown=%d responders=%d)",
                   g_cooldownFrames, g_responderCount);
    }
}
}
