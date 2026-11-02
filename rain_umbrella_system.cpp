#include "rain_umbrella_system.h"

#include <mod/amlmod.h>
#include <mod/logger.h>

#include <aml-psdk/game_sa/base/Timer.h>
#include <aml-psdk/game_sa/entity/Entity.h>
#include <aml-psdk/game_sa/entity/Ped.h>
#include <aml-psdk/game_sa/entity/PlayerPed.h>
#include <aml-psdk/game_sa/other/Pools.h>
#include <aml-psdk/game_sa/other/Weather.h>
#include <aml-psdk/game_sa/engine/World.h>
#include <aml-psdk/game_sa/utils/OpcodeCaller.h>
#include <aml-psdk/game_sa/enum/WeaponType.h>

#include <cstdlib>

namespace RainUmbrellaSystem {
namespace {
    constexpr int kUmbrellaModelId = 326; // gun_cane.dff; XkiLoader can replace it with an umbrella mesh.
    constexpr float kRainThreshold = 0.05f;
    constexpr float kSearchRadius = 48.0f;
    constexpr int kNearbyEntityCapacity = 64;
    constexpr int kMaxUmbrellaPeds = 12;
    constexpr unsigned int kScanIntervalMs = 1800;

    struct UmbrellaPed {
        int pedRef = -1;
        int previousWeapon = 0;
        bool hadCane = false;
        bool active = false;
    };

    UmbrellaPed g_umbrellaPeds[kMaxUmbrellaPeds] = {};
    unsigned int g_nextScanTime = 0;
    unsigned int g_lastAnimRequestTime = 0;
    bool g_animRequestSent = false;
    bool g_rainWasActive = false;
    bool g_modelRequestSent = false;
    bool g_missingAssetLogged = false;

    bool IsAnimationReady() {
        if (CommandByID(HAS_ANIMATION_LOADED, "WUZI")) return true;
        const unsigned int now = CTimer::GetTimeMS();
        if (!g_animRequestSent || static_cast<int>(now - g_lastAnimRequestTime) > 10000) {
            CommandByID(REQUEST_ANIMATION, "WUZI");
            g_animRequestSent = true;
            g_lastAnimRequestTime = now;
        }
        return CommandByID(HAS_ANIMATION_LOADED, "WUZI");
    }

    int CountActivePeds() {
        int count = 0;
        for (const UmbrellaPed& state : g_umbrellaPeds)
            if (state.active) ++count;
        return count;
    }

    UmbrellaPed* FindState(int pedRef) {
        for (UmbrellaPed& state : g_umbrellaPeds)
            if (state.active && state.pedRef == pedRef) return &state;
        return nullptr;
    }

    UmbrellaPed* AllocateState() {
        for (UmbrellaPed& state : g_umbrellaPeds)
            if (!state.active) return &state;
        return nullptr;
    }

    void StopUsingUmbrella(UmbrellaPed& state) {
        CPed* ped = CPools::GetPed(state.pedRef);
        if (ped && ped->IsAlive()) {
            int pedHandle = state.pedRef;
            CommandByID(CLEAR_CHAR_TASKS, &pedHandle);
            if (!state.hadCane)
                CommandByID(REMOVE_WEAPON_FROM_CHAR, &pedHandle, static_cast<int>(WEAPONTYPE_CANE));
            if (state.previousWeapon != static_cast<int>(WEAPONTYPE_CANE))
                CommandByID(SET_CURRENT_CHAR_WEAPON, &pedHandle, state.previousWeapon);
        }
        state = {};
    }

    void StopAllUmbrellas() {
        for (UmbrellaPed& state : g_umbrellaPeds)
            if (state.active) StopUsingUmbrella(state);
    }

    bool StartUsingUmbrella(CPed* ped) {
        UmbrellaPed* state = AllocateState();
        if (!state || !ped || !ped->IsAlive() || ped->bInVehicle || ped->IsPlayer()) return false;

        int previousWeapon = 0;
        int hadCane = 0;
        const int pedRef = CPools::GetPedRef(ped);
        if (pedRef < 0) return false;
        // Character arguments must be encoded as SCM variables for LocateCharCommand.
        int pedHandle = pedRef;
        CommandByID(GET_CURRENT_CHAR_WEAPON, &pedHandle, &previousWeapon);
        hadCane = CommandByID(HAS_CHAR_GOT_WEAPON, &pedHandle,
            static_cast<int>(WEAPONTYPE_CANE)) ? 1 : 0;

        if (!hadCane)
            CommandByID(GIVE_WEAPON_TO_CHAR, &pedHandle,
                static_cast<int>(WEAPONTYPE_CANE), 1);
        CommandByID(SET_CURRENT_CHAR_WEAPON, &pedHandle,
            static_cast<int>(WEAPONTYPE_CANE));

        // Match the supplied CSA: anim name payung inside the WUZI IFP block.
        CommandByID(TASK_PLAY_ANIM_SECONDARY, &pedHandle, "payung", "WUZI", 4.0f,
            1, 0, 0, 0, -1);

        state->pedRef = pedRef;
        state->previousWeapon = previousWeapon;
        state->hadCane = hadCane != 0;
        state->active = true;
        logger->Info("Rain umbrella enabled for NPC pedRef=%d (model=%d anim=WUZI/payung)",
            state->pedRef, kUmbrellaModelId);
        return true;
    }

    void PruneUmbrellaPeds(CPlayerPed* player) {
        CVector playerPosition = player->GetPosition();
        for (UmbrellaPed& state : g_umbrellaPeds) {
            if (!state.active) continue;
            CPed* ped = CPools::GetPed(state.pedRef);
            if (!ped || !ped->IsAlive() || ped->bInVehicle ||
                ped->GetPosition().DistSqr(playerPosition) > (kSearchRadius + 18.0f) * (kSearchRadius + 18.0f)) {
                StopUsingUmbrella(state);
            }
        }
    }

    void ScanNearbyPeds(CPlayerPed* player) {
        PruneUmbrellaPeds(player);
        CEntity* entities[kNearbyEntityCapacity] = {};
        i16 found = 0;
        const CVector playerPosition = player->GetPosition();
        CWorld::FindObjectsInRange(playerPosition, kSearchRadius, true, &found,
            kNearbyEntityCapacity, entities, false, false, true, false, false);

        int assigned = CountActivePeds();
        for (i16 i = 0; i < found && i < kNearbyEntityCapacity && assigned < kMaxUmbrellaPeds; ++i) {
            CEntity* entity = entities[i];
            if (!entity || !entity->IsPed()) continue;
            CPed* ped = entity->AsPed();
            if (!ped->IsAlive() || ped->bInVehicle || ped->IsPlayer() || !ped->IsPedInControl()) continue;

            const int pedRef = CPools::GetPedRef(ped);
            if (pedRef < 0 || FindState(pedRef)) continue;
            // Not every passerby carries an umbrella, for a more natural street scene.
            if ((std::rand() % 100) >= 48) continue;
            if (StartUsingUmbrella(ped)) ++assigned;
        }
    }
}

void Process(CPlayerPed* player) {
    if (!player) return;

    const bool raining = CWeather::Rain > kRainThreshold && CWeather::InTunnelness < 0.1f;
    if (!raining) {
        if (g_rainWasActive) {
            StopAllUmbrellas();
            logger->Info("Rain stopped; restored NPC weapons and tasks");
        }
        g_rainWasActive = false;
        return;
    }
    g_rainWasActive = true;

    if (!g_modelRequestSent) {
        CommandByID(REQUEST_MODEL, kUmbrellaModelId);
        g_modelRequestSent = true;
    }
    const bool modelReady = CommandByID(HAS_MODEL_LOADED, kUmbrellaModelId);
    const bool animationReady = IsAnimationReady();
    if (!modelReady || !animationReady) {
        if (!g_missingAssetLogged) {
            logger->Info("Rain umbrellas waiting for model 326 and animation block WUZI (model=%d anim=%d)",
                modelReady ? 1 : 0, animationReady ? 1 : 0);
            g_missingAssetLogged = true;
        }
        return;
    }
    g_missingAssetLogged = false;

    const unsigned int now = CTimer::GetTimeMS();
    if (static_cast<int>(now - g_nextScanTime) < 0) return;
    g_nextScanTime = now + kScanIntervalMs;
    ScanNearbyPeds(player);
}
}
