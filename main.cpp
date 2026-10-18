#include <mod/amlmod.h>
#include <mod/logger.h>

#include <aml-psdk/game_sa/Events.h>
#include <aml-psdk/game_sa/engine/Streaming.h>
#include <aml-psdk/game_sa/engine/World.h>
#include <aml-psdk/game_sa/entity/PlayerPed.h>
#include <aml-psdk/game_sa/entity/Ped.h>
#include <aml-psdk/game_sa/entity/Physical.h>
#include <aml-psdk/game_sa/entity/Vehicle.h>
#include <aml-psdk/game_sa/enum/PedEnums.h>
#include <aml-psdk/game_sa/other/Pools.h>
#include <aml-psdk/game_sa/utils/OpcodeCaller.h>

#include "helmet_system.h"

#include <string.h>

// BetaRealismAction
// AML / GTA SA Android 2.10 ARM64
// Includes dead-ped response and motorcycle-only helmet updates.

MYMOD(net.dimnih.betarealismaction, BetaRealismAction, 0.9.0, DimNih)

namespace {
    constexpr int kScanIntervalFrames = 30;
    constexpr float kScanRadius = 25.0f;
    constexpr float kResponderRadius = 18.0f;
    constexpr float kVehicleExitMaxSpeedKmh = 2.0f;
    constexpr int kFallbackAmbulanceModel = 416;
    constexpr i16 kMaxNearbyEntities = 64;
    constexpr int kMaxResponders = 5;
    constexpr int kMaxCandidates = 24;
    constexpr int kVictimRetentionFrames = 12;
    constexpr int kVehicleTaskRetryFrames = 180;
    constexpr int kObserverLookDurationMs = 3500;

    int g_framesUntilScan = 0;
    int g_victimRetentionFrames = 0;
    int g_lastLoggedVictimRef = -1;
    bool g_lastVictimWasCorpse = false;
    int g_victimRef = -1;
    int g_carrierRef = -1;
    int g_phoneCallerRef = -1;
    int g_phoneVictimRef = -1;
    int g_phoneFrameCount = 0;
    int g_attachedVictimRef = -1;
    int g_attachedCarrierRef = -1;
    int g_completedCarryVictimRef = -1;
    int g_carryScanCount = 0;
    int g_carryFrameCount = 0;
    int g_medicalFrameCount = 0;
    int g_medicRef = -1;
    int g_replacementModel = -1;
    bool g_carryAnimRequested = false;
    bool g_medicAnimRequested = false;
    bool g_carryTaskIssued = false;
    bool g_liftAnimationStarted = false;
    bool g_replacementIsCorpse = false;
    bool g_emergencyDispatched = false;
    bool g_ambulanceModelRequested = false;
    bool g_fallbackAmbulanceLogged = false;

    enum class PhoneCallPhase : unsigned char {
        Idle,
        PhoneIn,
        PhoneTalk,
        PhoneOut
    };

    PhoneCallPhase g_phoneCallPhase = PhoneCallPhase::Idle;

    enum class RescuePhase : unsigned char {
        Idle,
        Lifting,
        Carrying,
        PuttingDown,
        PhoneIn,
        PhoneTalk,
        PhoneOut,
        WaitingForMedic,
        Reviving
    };

    RescuePhase g_rescuePhase = RescuePhase::Idle;
    enum class ResponderStage : unsigned char {
        AwaitingTask,
        WaitingForSafeVehicleExit,
        ApproachingVictim,
        InspectingVictim,
        CallingAmbulance,
        ObservingOnce,
        ReturningToVehicle,
        LeavingScene
    };

    struct Responder {
        int pedRef = -1;
        bool isCarrier = false;
        bool exitTaskSent = false;
        bool returnTaskSent = false;
        bool carryAttempted = false;
        bool wasDriver = false;
        int vehicleRef = -1;
        int inspectionScans = 0;
        int approachScans = 0;
        int vehicleTaskFrames = 0;
        ResponderStage stage = ResponderStage::AwaitingTask;
    };

    struct Candidate {
        CPed* ped = nullptr;
        CVehicle* vehicle = nullptr;
        float distanceSquared = 0.0f;
        bool wasDriver = false;
    };

    Responder g_responders[kMaxResponders] = {};
    int g_responderCount = 0;

    bool IsCorpse(CPed* ped) {
        return ped && (ped->GetHealth() <= 0.0f || ped->m_ePedState == PEDSTATE_DIE ||
                       ped->m_ePedState == PEDSTATE_DEAD);
    }

    int FindResponder(int pedRef) {
        for (int i = 0; i < g_responderCount; ++i) {
            if (g_responders[i].pedRef == pedRef) return i;
        }
        return -1;
    }

    void StartOneShotObservation(int pedRef) {
        // A brief, one-shot reaction; do not keep interrupting ordinary AI.
        Command< TASK_LOOK_ABOUT >(pedRef, kObserverLookDurationMs);
    }

    void ResetResponders(int newVictimRef) {
        g_victimRef = newVictimRef;
        g_victimRetentionFrames = 0;
        g_carrierRef = -1;
        g_phoneCallerRef = -1;
        g_phoneVictimRef = -1;
        g_phoneFrameCount = 0;
        g_phoneCallPhase = PhoneCallPhase::Idle;
        g_emergencyDispatched = false;
        g_ambulanceModelRequested = false;
        g_fallbackAmbulanceLogged = false;
        g_responderCount = 0;
        memset(g_responders, 0, sizeof(g_responders));
        for (int i = 0; i < kMaxResponders; ++i) {
            g_responders[i].pedRef = -1;
        }
    }

    bool IsEligibleResponder(CPed* ped, CPlayerPed* player, CPed* victim) {
        return ped && ped != player && ped != victim && !IsCorpse(ped) &&
               !ped->bFallenDown && ped->m_ePedState != PEDSTATE_FALL &&
               ped->m_ePedState != PEDSTATE_GETUP;
    }

    void RetainRespondersReturningToVehicles() {
        int kept = 0;
        for (int i = 0; i < g_responderCount; ++i) {
            const Responder& responder = g_responders[i];
            if (responder.stage == ResponderStage::ReturningToVehicle ||
                responder.stage == ResponderStage::LeavingScene) {
                if (kept != i) g_responders[kept] = responder;
                ++kept;
            }
        }
        for (int i = kept; i < kMaxResponders; ++i) {
            g_responders[i] = {};
            g_responders[i].pedRef = -1;
        }
        g_responderCount = kept;
        g_carrierRef = -1;

        // Keep the last confirmed victim for a short grace period so transient
        // world-query misses do not immediately cancel the response chain.
        if (g_victimRef >= 0 && g_victimRetentionFrames < kVictimRetentionFrames) {
            ++g_victimRetentionFrames;
            return;
        }
        g_victimRef = -1;
        g_victimRetentionFrames = 0;
    }

    void AddCandidate(Candidate* candidates, int& count, CPed* ped,
                      CPlayerPed* player, CPed* victim,
                      CVehicle* occupiedVehicle = nullptr, bool wasDriver = false) {
        if (!IsEligibleResponder(ped, player, victim)) return;
        const float distanceSquared = ped->GetDistanceSqrTo(victim);
        if (distanceSquared > kResponderRadius * kResponderRadius) return;

        for (int i = 0; i < count; ++i) {
            if (candidates[i].ped == ped) {
                if (occupiedVehicle) {
                    candidates[i].vehicle = occupiedVehicle;
                    candidates[i].wasDriver = wasDriver;
                }
                return;
            }
        }
        if (count >= kMaxCandidates) return;

        CVehicle* vehicle = occupiedVehicle ? occupiedVehicle :
                    (ped->bInVehicle ? ped->m_pVehicle : nullptr);
        if (vehicle && vehicle->GetSpeedKmh() > kVehicleExitMaxSpeedKmh) return;

        candidates[count++] = { ped, vehicle, distanceSquared, wasDriver };
    }

    void ClearCarryState() {
        g_attachedVictimRef = -1;
        g_attachedCarrierRef = -1;
        g_carryScanCount = 0;
        g_carryFrameCount = 0;
        g_medicalFrameCount = 0;
        g_medicRef = -1;
        g_replacementModel = -1;
        g_carryTaskIssued = false;
        g_liftAnimationStarted = false;
        g_replacementIsCorpse = false;
        g_emergencyDispatched = false;
        g_carryAnimRequested = false;
        g_medicAnimRequested = false;
        g_rescuePhase = RescuePhase::Idle;
    }

    void PrepareRespondersAfterRescue() {
        for (int i = 0; i < g_responderCount; ++i) {
            Responder& responder = g_responders[i];
            if (!responder.isCarrier) continue;
            if (responder.vehicleRef >= 0) {
                responder.stage = ResponderStage::ReturningToVehicle;
                responder.returnTaskSent = false;
                responder.vehicleTaskFrames = 0;
            } else {
                responder.stage = ResponderStage::LeavingScene;
            }
        }
    }

    bool IsAnimationLoaded(const char* animationFile, bool& requested) {
        if (strcmp(animationFile, "PED") == 0) return true;
        if (!requested) {
            Command< REQUEST_ANIMATION >(animationFile);
            requested = true;
        }
        return Command< HAS_ANIMATION_LOADED >(animationFile);
    }

    void PlayAnimation(int pedRef, const char* animation, const char* animationFile,
                       float blend, bool loop, bool lockX, bool lockY,
                       bool keepLastFrame, int time) {
        Command< TASK_PLAY_ANIM >(pedRef, animation, animationFile, blend,
                                  static_cast<int>(loop), static_cast<int>(lockX),
                                  static_cast<int>(lockY), static_cast<int>(keepLastFrame), time);
    }

    void PlaySecondaryAnimation(int pedRef, const char* animation,
                                const char* animationFile, float blend,
                                bool loop, bool lockX, bool lockY,
                                bool lockFrame, int time) {
        Command< TASK_PLAY_ANIM_SECONDARY >(pedRef, animation, animationFile, blend,
                                            static_cast<int>(loop), static_cast<int>(lockX),
                                            static_cast<int>(lockY), static_cast<int>(lockFrame),
                                            time);
    }

    bool AttachVictimToCarrier(CPed* carrier) {
        CPed* victim = CPools::GetPed(g_attachedVictimRef);
        if (!carrier || !victim || victim->bAttachedToEntity) return false;

        using AttachEntityFn = CEntity* (*)(CPhysical*, CEntity*, CVector, CVector);
        AttachEntityFn attachEntity = GetMainLibrarySymbol<AttachEntityFn>(
            "_ZN9CPhysical20AttachEntityToEntityEP7CEntity7CVectorS2_");
        if (!attachEntity) {
            logger->Error("Ped attachment symbol is unavailable; carry cancelled");
            return false;
        }

        // The victim is the attached entity; the carrier is its parent.
        const CVector offset(0.0f, 0.0f, 0.35f);
        const CVector tilt(1.5707963f, 0.0f, 0.0f);
        attachEntity(victim, carrier, offset, tilt);
        return victim->bAttachedToEntity;
    }

    bool ReplaceCorpseAndAttach(CPed* carrier) {
        CPed* corpse = CPools::GetPed(g_attachedVictimRef);
        if (!corpse || !carrier || !IsCorpse(corpse)) return false;

        if (g_replacementModel < 0) {
            g_replacementModel = static_cast<int>(corpse->m_nModelIndex);
            if (g_replacementModel <= 0) {
                logger->Error("Corpse model id invalid; keeping original corpse");
                return AttachVictimToCarrier(carrier);
            }
            Command< REQUEST_MODEL >(g_replacementModel);
        }
        if (!Command< HAS_MODEL_LOADED >(g_replacementModel)) return false;

        const CVector position = corpse->GetPosition();
        const float heading = corpse->m_placement.m_fHeading * 57.2957795f;
        const int pedType = corpse->m_nPedType;
        const int oldVictimRef = g_attachedVictimRef;
        int replacementRef = -1;
        Command< CREATE_CHAR >(pedType, g_replacementModel,
                               position.x, position.y, position.z, &replacementRef);
        CPed* replacement = replacementRef >= 0 ? CPools::GetPed(replacementRef) : nullptr;
        if (!replacement) {
            logger->Print(LogP_Warn, "Replacement ped spawn failed; original corpse retained");
            Command< MARK_MODEL_AS_NO_LONGER_NEEDED >(g_replacementModel);
            g_replacementModel = -1;
            return false;
        }

        Command< SET_CHAR_HEADING >(replacementRef, heading);
        replacement->SetHealthSafe(replacement->GetMaxHealth());
        replacement->bAllowMedicsToReviveMe = 1;
        replacement->bFallenDown = 1;
        replacement->bFadeOut = 0;
        PlayAnimation(replacementRef, "FLOOR_hit", "PED", 4.0f,
                      true, true, true, true, -1);

        g_attachedVictimRef = replacementRef;
        g_victimRef = replacementRef;
        if (!AttachVictimToCarrier(carrier)) {
            g_attachedVictimRef = oldVictimRef;
            g_victimRef = oldVictimRef;
            Command< DELETE_CHAR >(replacementRef);
            Command< MARK_MODEL_AS_NO_LONGER_NEEDED >(g_replacementModel);
            g_replacementModel = -1;
            logger->Print(LogP_Warn, "Replacement ped attachment failed; original corpse retained");
            return false;
        }

        // Delete the original only after the replacement is alive and attached.
        Command< DELETE_CHAR >(oldVictimRef);
        Command< MARK_MODEL_AS_NO_LONGER_NEEDED >(g_replacementModel);
        g_replacementModel = -1;
        g_replacementIsCorpse = true;
        g_completedCarryVictimRef = replacementRef;
        logger->Info("Dead ped %d replaced by living downed ped %d", oldVictimRef, replacementRef);
        return true;
    }

    bool DispatchAmbulance(CPed* victim) {
        if (!victim || g_emergencyDispatched) return false;

        using StreamEmergencyFn = bool (*)(bool);
        using GetAmbulanceModelFn = int (*)();
        StreamEmergencyFn streamEmergency = GetMainLibrarySymbol<StreamEmergencyFn>(
            "_ZN10CStreaming23StreamAmbulanceAndMedicEb");
        GetAmbulanceModelFn getAmbulanceModel = GetMainLibrarySymbol<GetAmbulanceModelFn>(
            "_ZN10CStreaming24GetDefaultAmbulanceModelEv");
        if (streamEmergency) streamEmergency(true);
        int ambulanceModel = getAmbulanceModel ? getAmbulanceModel() : kFallbackAmbulanceModel;
        if (ambulanceModel <= 0) {
            if (!g_fallbackAmbulanceLogged) {
                logger->Print(LogP_Warn, "Default ambulance model unavailable; using model 416 fallback");
                g_fallbackAmbulanceLogged = true;
            }
            ambulanceModel = kFallbackAmbulanceModel;
        }

        if (!streamEmergency) {
            if (!g_ambulanceModelRequested) {
                Command< REQUEST_MODEL >(ambulanceModel);
                g_ambulanceModelRequested = true;
            }
            if (!Command< HAS_MODEL_LOADED >(ambulanceModel)) return false;
        }

        const CVector position = victim->GetPosition();
        // SCM 085A signature: vehicle model, x, y, z. The engine owns dispatch
        // and does not return a script vehicle handle for this command.
        Command< CREATE_EMERGENCY_SERVICES_CAR >(ambulanceModel,
                                                 position.x, position.y, position.z);
        g_emergencyDispatched = true;
        logger->Info("Caller requested ambulance at corpse (model %d)", ambulanceModel);
        return true;
    }

    void StartAmbulanceCall(int callerRef, int victimRef) {
        g_phoneCallerRef = callerRef;
        g_phoneVictimRef = victimRef;
        g_phoneFrameCount = 0;
        g_phoneCallPhase = PhoneCallPhase::PhoneIn;
        g_emergencyDispatched = false;
        logger->Info("Caller %d reached corpse and is starting an ambulance call", callerRef);
    }

    bool ProcessAmbulanceCall() {
        if (g_phoneCallPhase == PhoneCallPhase::Idle) return false;

        CPed* caller = CPools::GetPed(g_phoneCallerRef);
        CPed* victim = CPools::GetPed(g_phoneVictimRef);
        if (!caller || !victim || !IsCorpse(victim)) {
            logger->Print(LogP_Warn, "Ambulance call cancelled because caller or corpse is no longer valid");
            const int responderIndex = FindResponder(g_phoneCallerRef);
            if (responderIndex >= 0) {
                Responder& responder = g_responders[responderIndex];
                responder.carryAttempted = true;
                responder.stage = responder.vehicleRef >= 0
                    ? ResponderStage::ReturningToVehicle
                    : ResponderStage::LeavingScene;
                responder.returnTaskSent = false;
                responder.vehicleTaskFrames = 0;
            }
            g_phoneCallPhase = PhoneCallPhase::Idle;
            g_phoneCallerRef = -1;
            g_phoneVictimRef = -1;
            g_phoneFrameCount = 0;
            return false;
        }

        ++g_phoneFrameCount;
        if (!g_emergencyDispatched &&
            (g_phoneCallPhase == PhoneCallPhase::PhoneTalk ||
             g_phoneCallPhase == PhoneCallPhase::PhoneOut)) {
            DispatchAmbulance(victim);
        }
        switch (g_phoneCallPhase) {
            case PhoneCallPhase::PhoneIn:
                if (g_phoneFrameCount == 1) {
                    PlayAnimation(g_phoneCallerRef, "phone_in", "PED", 4.0f,
                                  false, false, false, false, 700);
                }
                if (g_phoneFrameCount >= 42) {
                    g_phoneFrameCount = 0;
                    g_phoneCallPhase = PhoneCallPhase::PhoneTalk;
                    DispatchAmbulance(victim);
                }
                return true;

            case PhoneCallPhase::PhoneTalk:
                if (g_phoneFrameCount == 1) {
                    PlayAnimation(g_phoneCallerRef, "phone_talk", "PED", 4.0f,
                                  true, false, false, false, 2500);
                }
                if (g_phoneFrameCount >= 150) {
                    g_phoneFrameCount = 0;
                    g_phoneCallPhase = PhoneCallPhase::PhoneOut;
                    logger->Info("Caller is ending the phone call");
                }
                return true;

            case PhoneCallPhase::PhoneOut:
                if (g_phoneFrameCount == 1) {
                    PlayAnimation(g_phoneCallerRef, "phone_out", "PED", 4.0f,
                                  false, false, false, false, 500);
                }
                if (g_phoneFrameCount >= 35) {
                    const int callerRef = g_phoneCallerRef;
                    const int responderIndex = FindResponder(callerRef);
                    if (responderIndex >= 0) {
                        Responder& responder = g_responders[responderIndex];
                        responder.carryAttempted = true;
                        responder.stage = responder.vehicleRef >= 0
                            ? ResponderStage::ReturningToVehicle
                            : ResponderStage::LeavingScene;
                        responder.returnTaskSent = false;
                        responder.vehicleTaskFrames = 0;
                    }
                    logger->Info("Caller %d finished the call and is returning to normal behavior", callerRef);
                    g_phoneCallPhase = PhoneCallPhase::Idle;
                    g_phoneCallerRef = -1;
                    g_phoneVictimRef = -1;
                    g_phoneFrameCount = 0;
                    return false;
                }
                return true;

            case PhoneCallPhase::Idle:
                return false;
        }
        return false;
    }

    bool DetachVictim(CPed* victim) {
        if (!victim || !victim->bAttachedToEntity) return false;

        using DetachPedFn = void (*)(CPed*);
        DetachPedFn detachPed = GetMainLibrarySymbol<DetachPedFn>("_ZN4CPed20DettachPedFromEntityEv");
        if (!detachPed) {
            logger->Error("Ped detach symbol is unavailable; refusing unsafe carry cleanup");
            return false;
        }
        detachPed(victim);
        return !victim->bAttachedToEntity;
    }

    bool BeginCarry(CPed* carrier, CPed* victim, int carrierRef, int victimRef,
                    CPlayerPed* player) {
        if (!carrier || !victim || !player || !IsCorpse(victim) || victim->bAttachedToEntity ||
            g_completedCarryVictimRef == victimRef) {
            return false;
        }
        g_attachedVictimRef = victimRef;
        g_attachedCarrierRef = carrierRef;
        g_carrierRef = carrierRef;
        g_carryScanCount = 0;
        g_carryFrameCount = 0;
        g_medicalFrameCount = 0;
        g_medicRef = -1;
        g_carryTaskIssued = false;
        g_liftAnimationStarted = false;
        g_replacementIsCorpse = IsCorpse(victim);
        victim->bFadeOut = 0;
        victim->bAllowMedicsToReviveMe = 1;
        g_emergencyDispatched = false;
        g_carryAnimRequested = false;
        g_medicAnimRequested = false;
        g_rescuePhase = RescuePhase::Lifting;

        if (g_replacementIsCorpse) {
            g_replacementModel = static_cast<int>(victim->m_nModelIndex);
            if (g_replacementModel > 0) Command< REQUEST_MODEL >(g_replacementModel);
        }
        logger->Info("Rescue sequence started: carrier %d, victim %d%s",
                     carrierRef, victimRef,
                     g_replacementIsCorpse ? " (corpse clone pending)" : "");

        if (IsAnimationLoaded("CARRY", g_carryAnimRequested)) {
            PlayAnimation(carrierRef, "liftup", "CARRY", 4.0f,
                          false, false, false, false, 1530);
            g_liftAnimationStarted = true;
        }
        return true;
    }

    bool ProcessCarry(CPlayerPed* player) {
        if (g_rescuePhase == RescuePhase::Idle ||
            g_attachedVictimRef < 0 || g_attachedCarrierRef < 0) return false;

        CPed* carrier = CPools::GetPed(g_attachedCarrierRef);
        CPed* victim = CPools::GetPed(g_attachedVictimRef);
        if (!carrier || !victim) {
            logger->Print(LogP_Warn, "Rescue sequence cancelled because carrier or victim entity was removed");
            if (g_replacementModel > 0) Command< MARK_MODEL_AS_NO_LONGER_NEEDED >(g_replacementModel);
            g_completedCarryVictimRef = g_attachedVictimRef;
            PrepareRespondersAfterRescue();
            ClearCarryState();
            return false;
        }

        ++g_carryFrameCount;
        switch (g_rescuePhase) {
            case RescuePhase::Lifting: {
                if (!g_liftAnimationStarted && IsAnimationLoaded("CARRY", g_carryAnimRequested)) {
                    PlayAnimation(g_attachedCarrierRef, "liftup", "CARRY", 4.0f,
                                  false, false, false, false, 1530);
                    g_liftAnimationStarted = true;
                    g_carryFrameCount = 0;
                }
                if (g_carryFrameCount < 90 || !g_liftAnimationStarted) return true;

                const bool attached = g_replacementIsCorpse
                    ? ReplaceCorpseAndAttach(carrier)
                    : AttachVictimToCarrier(carrier);
                if (!attached) {
                    if (g_replacementIsCorpse && g_replacementModel > 0 &&
                        !Command< HAS_MODEL_LOADED >(g_replacementModel)) {
                        return true;
                    }
                    logger->Print(LogP_Warn, "Victim attachment failed; source entity was retained");
                    g_completedCarryVictimRef = g_attachedVictimRef;
                    ClearCarryState();
                    return false;
                }

                if (IsAnimationLoaded("CARRY", g_carryAnimRequested)) {
                    // Keep movement in the primary task slot; TASK_GOTO_CHAR
                    // would otherwise replace the carry animation.
                    PlaySecondaryAnimation(g_attachedCarrierRef, "crry_prtial", "CARRY", 4.0f,
                                           true, true, true, false, -1);
                }
                const int playerRef = player ? CPools::GetPedRef(player) : -1;
                if (playerRef >= 0) {
                    Command< TASK_GOTO_CHAR >(g_attachedCarrierRef, playerRef,
                                              static_cast<int>(PEDMOVE_WALK), 2.0f);
                    g_carryTaskIssued = true;
                }
                g_carryFrameCount = 0;
                g_rescuePhase = RescuePhase::Carrying;
                logger->Info("Victim %d attached; carrier %d moving to player",
                             g_attachedVictimRef, g_attachedCarrierRef);
                return true;
            }

            case RescuePhase::Carrying: {
                if (!victim->bAttachedToEntity) {
                    logger->Print(LogP_Warn, "Victim was detached unexpectedly; requesting put-down cleanup");
                    g_rescuePhase = RescuePhase::PuttingDown;
                    g_carryFrameCount = 90;
                    return true;
                }
                if (player && carrier->GetDistanceTo(player) <= 2.5f && g_carryFrameCount >= 30) {
                    PlayAnimation(g_attachedCarrierRef, "putdwn", "CARRY", 4.0f,
                                  false, false, false, false, 1130);
                    g_rescuePhase = RescuePhase::PuttingDown;
                    g_carryFrameCount = 0;
                    logger->Info("Carrier reached the player; starting put-down animation");
                } else if (player && !g_carryTaskIssued) {
                    const int playerRef = CPools::GetPedRef(player);
                    if (playerRef >= 0) {
                        Command< TASK_GOTO_CHAR >(g_attachedCarrierRef, playerRef,
                                                  static_cast<int>(PEDMOVE_WALK), 2.0f);
                        g_carryTaskIssued = true;
                    }
                }
                return true;
            }

            case RescuePhase::PuttingDown:
                if (g_carryFrameCount < 68) return true;
                if (victim->bAttachedToEntity && !DetachVictim(victim)) return true;
                victim = CPools::GetPed(g_attachedVictimRef);
                if (victim) {
                    victim->bFallenDown = 1;
                    victim->bAllowMedicsToReviveMe = 1;
                    PlayAnimation(g_attachedVictimRef, "FLOOR_hit", "PED", 4.0f,
                                  true, true, true, true, -1);
                }
                g_completedCarryVictimRef = g_attachedVictimRef;
                g_rescuePhase = RescuePhase::PhoneIn;
                g_carryFrameCount = 0;
                g_liftAnimationStarted = false;
                logger->Info("Victim placed beside player; carrier is calling for an ambulance");
                return true;

            case RescuePhase::PhoneIn:
                if (g_carryFrameCount == 1) {
                    PlayAnimation(g_attachedCarrierRef, "phone_in", "PED", 4.0f,
                                  false, false, false, false, 700);
                }
                if (g_carryFrameCount >= 42) {
                    g_carryFrameCount = 0;
                    g_rescuePhase = RescuePhase::PhoneTalk;
                }
                return true;

            case RescuePhase::PhoneTalk:
                if (g_carryFrameCount == 1) {
                    PlayAnimation(g_attachedCarrierRef, "phone_talk", "PED", 4.0f,
                                  true, false, false, false, 2500);
                }
                if (g_carryFrameCount >= 150) {
                    DispatchAmbulance(victim);
                    g_carryFrameCount = 0;
                    g_rescuePhase = RescuePhase::PhoneOut;
                }
                return true;

            case RescuePhase::PhoneOut:
                if (g_carryFrameCount == 1) {
                    PlayAnimation(g_attachedCarrierRef, "phone_out", "PED", 4.0f,
                                  false, false, false, false, 500);
                }
                if (g_carryFrameCount >= 35) {
                    g_carryFrameCount = 0;
                    g_medicalFrameCount = 0;
                    g_medicRef = -1;
                    g_rescuePhase = RescuePhase::WaitingForMedic;
                }
                return true;

            case RescuePhase::WaitingForMedic: {
                ++g_medicalFrameCount;
                if (g_medicalFrameCount % 30 != 0) return true;
                if (g_medicalFrameCount >= 3600) {
                    logger->Print(LogP_Warn, "No medic reached the victim within 60 seconds; leaving victim down and ending rescue wait");
                    PrepareRespondersAfterRescue();
                    ClearCarryState();
                    return false;
                }
                CEntity* nearby[kMaxNearbyEntities] = {};
                i16 found = 0;
                CWorld::FindObjectsInRange(victim->GetPosition(), 6.0f, true,
                                           &found, kMaxNearbyEntities, nearby,
                                           false, false, true, false, false);
                for (i16 i = 0; i < found && i < kMaxNearbyEntities; ++i) {
                    CEntity* entity = nearby[i];
                    if (!entity || !entity->IsPed()) continue;
                    CPed* medic = entity->AsPed();
                    if (medic->m_nPedType == PED_TYPE_MEDIC &&
                        medic->m_ePedState == PEDSTATE_CPR &&
                        medic->GetDistanceSqrTo(victim) <= 16.0f) {
                        g_medicRef = CPools::GetPedRef(medic);
                        g_rescuePhase = RescuePhase::Reviving;
                        g_medicalFrameCount = 0;
                        logger->Info("Medic %d reached victim %d; tracking treatment", g_medicRef,
                                     g_attachedVictimRef);
                        break;
                    }
                }
                return true;
            }

            case RescuePhase::Reviving:
                ++g_medicalFrameCount;
                if (g_medicalFrameCount == 500) {
                    victim->SetHealthSafe(victim->GetMaxHealth() * 0.5f);
                    victim->bFallenDown = 0;
                    victim->bAllowMedicsToReviveMe = 0;
                    PlayAnimation(g_attachedVictimRef, "getup", "PED", 4.0f,
                                  false, false, false, false, 1500);
                    logger->Info("Medic treatment complete; victim %d is recovering",
                                 g_attachedVictimRef);
                }
                if (g_medicalFrameCount >= 590) {
                    PrepareRespondersAfterRescue();
                    ClearCarryState();
                    logger->Info("Rescue sequence complete");
                    return false;
                }
                return true;

            case RescuePhase::Idle:
                return false;
        }
        return false;
    }

    void AssignResponderTasks(Candidate* candidates, int candidateCount,
                              int victimRef, CPed* victim) {
        // Prefer a pedestrian already on foot so the visible caller can start
        // approaching immediately instead of waiting for a vehicle exit task.
        for (int i = 1; i < candidateCount; ++i) {
            Candidate candidate = candidates[i];
            const bool candidateOnFoot = candidate.vehicle == nullptr;
            int j = i;
            while (j > 0) {
                const Candidate& previous = candidates[j - 1];
                const bool previousOnFoot = previous.vehicle == nullptr;
                const bool shouldMoveAhead =
                    (candidateOnFoot && !previousOnFoot) ||
                    (candidateOnFoot == previousOnFoot &&
                     candidate.distanceSquared < previous.distanceSquared);
                if (!shouldMoveAhead) break;
                candidates[j] = candidates[j - 1];
                --j;
            }
            candidates[j] = candidate;
        }

        for (int i = 0; i < candidateCount; ++i) {
            Candidate& candidate = candidates[i];
            const int pedRef = CPools::GetPedRef(candidate.ped);
            if (pedRef < 0) continue;

            int responderIndex = FindResponder(pedRef);
            if (responderIndex < 0) {
                if (g_responderCount >= kMaxResponders) break;

                responderIndex = g_responderCount++;
                Responder& newResponder = g_responders[responderIndex];
                newResponder.pedRef = pedRef;
                newResponder.isCarrier = (g_carrierRef < 0);
                newResponder.exitTaskSent = false;
                newResponder.returnTaskSent = false;
                newResponder.carryAttempted = false;
                newResponder.wasDriver = candidate.vehicle && candidate.wasDriver;
                newResponder.vehicleRef = candidate.vehicle
                    ? CPools::GetVehicleRef(candidate.vehicle)
                    : -1;
                newResponder.inspectionScans = 0;
                newResponder.approachScans = 0;
                newResponder.vehicleTaskFrames = 0;
                newResponder.stage = candidate.vehicle
                    ? ResponderStage::WaitingForSafeVehicleExit
                    : ResponderStage::AwaitingTask;
                if (newResponder.isCarrier) g_carrierRef = pedRef;

                logger->Info("Responder %d assigned as %s", pedRef,
                             newResponder.isCarrier ? "the ambulance caller" : "helper/observer");
            }

            Responder& responder = g_responders[responderIndex];
            // Pool handles are generation-tagged. Resolve them again immediately
            // before issuing any script task; never retain raw ped pointers here.
            if (CPools::GetPed(pedRef) != candidate.ped || CPools::GetPed(victimRef) != victim) {
                continue;
            }
            if (responder.stage == ResponderStage::ReturningToVehicle ||
                responder.stage == ResponderStage::LeavingScene) {
                continue;
            }

            if (!responder.isCarrier) {
                if (responder.stage != ResponderStage::ObservingOnce) {
                    if (!candidate.vehicle) {
                        StartOneShotObservation(pedRef);
                    }
                    responder.stage = ResponderStage::ObservingOnce;
                    logger->Info("Observer %d assigned once%s", pedRef,
                                 candidate.vehicle ? "; left in car to preserve vehicle AI" : "; brief head-look-around task");
                }
                continue;
            }

            if (responder.stage == ResponderStage::CallingAmbulance ||
                responder.stage == ResponderStage::ReturningToVehicle ||
                responder.stage == ResponderStage::LeavingScene) {
                continue;
            }

            if (candidate.vehicle) {
                const int vehicleRef = CPools::GetVehicleRef(candidate.vehicle);
                if (vehicleRef < 0 || candidate.vehicle->GetSpeedKmh() > kVehicleExitMaxSpeedKmh) {
                    continue;
                }
                if (responder.stage != ResponderStage::WaitingForSafeVehicleExit) {
                    responder.stage = ResponderStage::WaitingForSafeVehicleExit;
                    responder.exitTaskSent = false;
                    responder.vehicleRef = vehicleRef;
                    responder.wasDriver = candidate.wasDriver;
                }
                // GTA SA SCM 05CD: TASK_LEAVE_CAR(ped, vehicle, timeout, flag).
                if (!responder.exitTaskSent || ++responder.vehicleTaskFrames >= kVehicleTaskRetryFrames) {
                    Command< TASK_LEAVE_CAR >(pedRef, vehicleRef, 10000, 0);
                    responder.exitTaskSent = true;
                    responder.vehicleTaskFrames = 0;
                }
                continue;
            }

            if (responder.stage == ResponderStage::WaitingForSafeVehicleExit) {
                responder.stage = ResponderStage::AwaitingTask;
            }

            const float distance = sqrtf(candidate.distanceSquared);
            const float inspectDistance = 1.8f;
            if (distance > inspectDistance) {
                if (responder.stage != ResponderStage::ApproachingVictim ||
                    ++responder.approachScans >= 10) {
                    // GTA SA SCM 05D9: TASK_GOTO_CHAR(ped, target, move state, radius).
                    Command< TASK_GOTO_CHAR >(pedRef, victimRef, static_cast<int>(PEDMOVE_WALK),
                                              1.5f);
                    responder.stage = ResponderStage::ApproachingVictim;
                    responder.approachScans = 0;
                }
            } else if (responder.stage != ResponderStage::InspectingVictim) {
                responder.stage = ResponderStage::InspectingVictim;
                Command< TASK_LOOK_AT_CHAR >(pedRef, victimRef, 1200, 0);
                StartAmbulanceCall(pedRef, victimRef);
                responder.stage = ResponderStage::CallingAmbulance;
                logger->Info("Responder %d reached the corpse; calling instead of carrying", pedRef);
            }
        }
    }

    void ProcessReturningResponders() {
        for (int i = 0; i < g_responderCount; ++i) {
            Responder& responder = g_responders[i];
            if (responder.stage != ResponderStage::ReturningToVehicle &&
                responder.stage != ResponderStage::LeavingScene) {
                continue;
            }

            CPed* ped = CPools::GetPed(responder.pedRef);
            CVehicle* vehicle = responder.vehicleRef >= 0
                ? CPools::GetVehicle(responder.vehicleRef)
                : nullptr;
            if (!ped || !vehicle) {
                responder.stage = ResponderStage::LeavingScene;
                continue;
            }

            if (ped->bInVehicle && ped->m_pVehicle == vehicle) {
                if (responder.wasDriver && responder.stage != ResponderStage::LeavingScene) {
                    // Resume ordinary traffic AI once the original driver is back.
                    Command< TASK_CAR_DRIVE_WANDER >(responder.pedRef, responder.vehicleRef,
                                                     15.0f, 0);
                    logger->Info("Driver responder %d returned to vehicle %d and resumed driving",
                                 responder.pedRef, responder.vehicleRef);
                }
                responder.stage = ResponderStage::LeavingScene;
                continue;
            }

            if (responder.stage == ResponderStage::LeavingScene) continue;
            if (!ped->bInVehicle && vehicle->GetSpeedKmh() <= kVehicleExitMaxSpeedKmh &&
                (!responder.returnTaskSent || ++responder.vehicleTaskFrames >= kVehicleTaskRetryFrames)) {
                if (responder.wasDriver) {
                    Command< TASK_ENTER_CAR_AS_DRIVER >(responder.pedRef,
                                                        responder.vehicleRef, 10000);
                } else {
                    Command< TASK_ENTER_CAR_AS_PASSENGER >(responder.pedRef,
                                                           responder.vehicleRef, 10000);
                }
                responder.returnTaskSent = true;
                responder.vehicleTaskFrames = 0;
                logger->Info("Responder %d returning to original vehicle %d",
                             responder.pedRef, responder.vehicleRef);
            }
        }
    }

    void Tick() {
        CPlayerPed* player = FindPlayerPed(0);
        HelmetSystem::Process(player);
        ProcessReturningResponders();
        if (ProcessAmbulanceCall()) return;

        if (g_framesUntilScan-- > 0) return;
        g_framesUntilScan = kScanIntervalFrames;

        if (!player) return;

        CEntity* nearby[kMaxNearbyEntities] = {};
        i16 found = 0;
        CWorld::FindObjectsInRange(player->GetPosition(), kScanRadius, true,
                                   &found, kMaxNearbyEntities, nearby,
                                   false, false, true, false, false);

        CPed* closestVictim = nullptr;
        bool closestIsCorpse = false;
        float closestDistance = kScanRadius * kScanRadius;

        if (g_victimRef >= 0) {
            CPed* lockedVictim = CPools::GetPed(g_victimRef);
            if (lockedVictim && IsCorpse(lockedVictim)) {
                const float distance = player->GetDistanceSqrTo(lockedVictim);
                closestVictim = lockedVictim;
                closestIsCorpse = true;
                closestDistance = distance;
            }
        }

        for (i16 i = 0; i < found && i < kMaxNearbyEntities; ++i) {
            CEntity* entity = nearby[i];
            if (!entity || !entity->IsPed()) continue;

            CPed* ped = entity->AsPed();
            if (ped == player) continue;

            // A ped returned by this in-world query is a live entity allocation
            // for this frame. Do not keep/dereference its pointer after it drops
            // out of the scan: the game may remove the corpse at any time.
            const bool corpse = IsCorpse(ped);
            if (!corpse) continue;
            const int pedRef = CPools::GetPedRef(ped);
            if (pedRef < 0 || pedRef == g_completedCarryVictimRef) continue;
            if (g_victimRef >= 0 && pedRef != g_victimRef) continue;

            const float distance = player->GetDistanceSqrTo(ped);
            if (!closestVictim && distance < closestDistance) {
                closestDistance = distance;
                closestVictim = ped;
                closestIsCorpse = corpse;
            }
        }

        const int closestVictimRef = closestVictim ? CPools::GetPedRef(closestVictim) : -1;
        if (closestVictimRef != g_lastLoggedVictimRef || closestIsCorpse != g_lastVictimWasCorpse) {
            g_lastLoggedVictimRef = closestVictimRef;
            g_lastVictimWasCorpse = closestIsCorpse;
            if (closestVictim) {
                logger->Info("Dead corpse selected (health %.1f, state %u, distance %.1f m); dead-only response active",
                             closestVictim->GetHealth(),
                             static_cast<unsigned int>(closestVictim->m_ePedState),
                             sqrtf(closestDistance));
            } else {
                logger->Info("No dead NPC corpse in range");
            }
        }

        if (!closestVictim || closestVictimRef < 0) {
            if (g_victimRef != -1) {
                if (g_victimRetentionFrames < kVictimRetentionFrames) {
                    ++g_victimRetentionFrames;
                    return;
                }
                RetainRespondersReturningToVehicles();
            }
            return;
        }

        g_victimRetentionFrames = 0;
        if (g_victimRef != closestVictimRef) ResetResponders(closestVictimRef);

        Candidate candidates[kMaxCandidates] = {};
        int candidateCount = 0;
        CEntity* nearbyHelpers[kMaxNearbyEntities] = {};
        i16 helperCount = 0;
        CWorld::FindObjectsInRange(closestVictim->GetPosition(), kResponderRadius, true,
                                   &helperCount, kMaxNearbyEntities, nearbyHelpers,
                                   false, false, true, false, false);
        for (i16 i = 0; i < helperCount && i < kMaxNearbyEntities; ++i) {
            CEntity* entity = nearbyHelpers[i];
            if (entity && entity->IsPed()) {
                AddCandidate(candidates, candidateCount, entity->AsPed(), player, closestVictim);
            }
        }

        // Seated peds are not reliably returned by the ped world query. Check
        // only nearby, almost-stationary cars, then include their occupants.
        CEntity* nearbyVehicles[kMaxNearbyEntities] = {};
        i16 vehicleCount = 0;
        CWorld::FindObjectsInRange(closestVictim->GetPosition(), kResponderRadius, true,
                                   &vehicleCount, kMaxNearbyEntities, nearbyVehicles,
                                   false, true, false, false, false);
        for (i16 i = 0; i < vehicleCount && i < kMaxNearbyEntities; ++i) {
            CEntity* entity = nearbyVehicles[i];
            if (!entity || !entity->IsVehicle()) continue;
            CVehicle* vehicle = entity->AsVehicle();
            if (vehicle->GetSpeedKmh() > kVehicleExitMaxSpeedKmh) continue;

            AddCandidate(candidates, candidateCount, vehicle->m_pDriver, player,
                         closestVictim, vehicle, true);
            const unsigned int passengers = vehicle->m_nNumPassengers < 8 ? vehicle->m_nNumPassengers : 8;
            for (unsigned int passenger = 0; passenger < passengers; ++passenger) {
                AddCandidate(candidates, candidateCount, vehicle->m_apPassengers[passenger],
                             player, closestVictim, vehicle);
            }
        }

        // Drop assigned NPCs only after validating their pool reference.
        for (int i = 0; i < g_responderCount;) {
            if (!CPools::GetPed(g_responders[i].pedRef)) {
                if (g_responders[i].pedRef == g_carrierRef) g_carrierRef = -1;
                for (int j = i + 1; j < g_responderCount; ++j) g_responders[j - 1] = g_responders[j];
                --g_responderCount;
            } else {
                ++i;
            }
        }

        AssignResponderTasks(candidates, candidateCount, closestVictimRef, closestVictim);
    }
}

ON_MOD_LOAD() {
    logger->SetTag("BetaRealismAction");
    Events::gameProcessEvent += []() { Tick(); };
}
