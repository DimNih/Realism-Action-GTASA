#include "beach_life_system.h"

#include <mod/logger.h>
#include <aml-psdk/game_sa/base/Timer.h>
#include <aml-psdk/game_sa/engine/World.h>
#include <aml-psdk/game_sa/entity/Entity.h>
#include <aml-psdk/game_sa/entity/Ped.h>
#include <aml-psdk/game_sa/entity/PlayerPed.h>
#include <aml-psdk/game_sa/entity/Vehicle.h>
#include <aml-psdk/game_sa/enum/PedEnums.h>
#include <aml-psdk/game_sa/other/Pools.h>
#include <aml-psdk/game_sa/utils/OpcodeCaller.h>

#include <cstdlib>
#include <cmath>
#include <algorithm>

namespace BeachLifeSystem {
namespace {
    constexpr int kMaxManagedPeds = 16;
    // Keep a small stream of visitors around without creating a dense crowd in
    // one frame. This cap includes swimmers, cyclists, and the boat driver.
    constexpr int kMaxSpawnedBeachPeds = 6;
    constexpr int kEntityCapacity = 64;
    constexpr float kSearchRadius = 38.0f;
    constexpr unsigned int kScanMs = 1400;
    constexpr unsigned int kSpawnIntervalMs = 6500;
    constexpr unsigned int kBeachEntryDelayMs = 8000;
    constexpr unsigned int kRescheduleMs = 30000;
    constexpr float kSpawnCleanupDistance = 82.0f;
    constexpr int kBeachVisitorModels[] = { 18, 45, 154, 138, 139, 140 };

    enum class Activity : unsigned char { LieDown, Swim, Sit, Chat, Bike, Stroll };
    struct ManagedPed {
        int ref = -1;
        int vehicleRef = -1;
        unsigned int expiresAt = 0;
        float targetX = 0.0f;
        float targetY = 0.0f;
        float swimShoreX = 0.0f;
        float swimShoreY = 0.0f;
        float swimShoreZ = 0.0f;
        unsigned char swimOutboundRetries = 0;
        Activity activity = Activity::Stroll;
        bool vehicleDriving = false;
        bool vehicleExitRequested = false;
        bool swimReachedWater = false;
        bool returningFromSwim = false;
        bool active = false;
    };
    struct SpawnedPed {
        int ref = -1;
        int vehicleRef = -1;
        unsigned int expiresAt = 0;
        float startX = 0.0f;
        float startY = 0.0f;
        bool isBoat = false;
        bool isSwimmer = false;
        bool returning = false;
        bool active = false;
    };
    struct BeachZone;
    ManagedPed g_managed[kMaxManagedPeds] = {};
    SpawnedPed g_spawned[kMaxSpawnedBeachPeds] = {};
    unsigned int g_nextScan = 0;
    unsigned int g_nextSpawnAt = 0;
    const BeachZone* g_currentBeach = nullptr;
    bool g_beachAnimRequested = false;
    bool g_bikeModelRequested = false;
    bool g_boatModelRequested = false;
    bool g_visitorModelRequested[sizeof(kBeachVisitorModels) / sizeof(kBeachVisitorModels[0])] = {};
    int g_spawnCount = 0;

    struct BeachZone { const char* name; float minX, maxX, minY, maxY; };
    // Original GTA SA zone bounds for Santa Maria and Verona beaches.
    constexpr BeachZone kBeaches[] = {
        { "Santa Maria Beach", 72.65f, 647.71f, -2173.29f, -1684.65f },
        { "Verona Beach", 647.71f, 930.22f, -2173.29f, -1804.21f }
    };

    const BeachZone* FindBeach(const CVector& p) {
        for (const BeachZone& b : kBeaches)
            if (p.x >= b.minX && p.x <= b.maxX && p.y >= b.minY && p.y <= b.maxY) return &b;
        return nullptr;
    }
    bool GetSwimTargets(const BeachZone& beach, const CVector& pedPos,
                        float& waterX, float& waterY, float& waterZ,
                        float& shoreX, float& shoreY) {
        (void)beach;
        // Beach zone rectangles include a lot of inland space, so a fixed
        // “sea edge” sends peds across the map. Search radially for the closest
        // real water point instead and only let nearby visitors start swimming.
        constexpr float kMaxShoreDistance = 42.0f;
        float nearestDistance = kMaxShoreDistance + 1.0f;
        bool foundWater = false;
        float bestX = 0.0f, bestY = 0.0f, bestZ = 0.0f;
        for (int direction = 0; direction < 16; ++direction) {
            const float angle = static_cast<float>(direction) * 6.2831853f / 16.0f;
            for (float distance = 6.0f; distance <= kMaxShoreDistance; distance += 6.0f) {
                const float x = pedPos.x + cosf(angle) * distance;
                const float y = pedPos.y + sinf(angle) * distance;
                float candidateZ = 0.0f;
                if (!Command< GET_WATER_HEIGHT_AT_COORDS >(x, y, 1, &candidateZ)) continue;
                if (distance < nearestDistance) {
                    nearestDistance = distance;
                    bestX = x;
                    bestY = y;
                    bestZ = candidateZ;
                    foundWater = true;
                }
                break;
            }
        }
        if (!foundWater) return false;

        // The first water cell can be shallow; aim a little farther out while
        // keeping the swim task anchored to this beach's actual shoreline.
        const float dx = bestX - pedPos.x;
        const float dy = bestY - pedPos.y;
        const float length = sqrtf(dx * dx + dy * dy);
        const float push = length > 0.1f ? 12.0f / length : 0.0f;
        waterX = bestX + dx * push;
        waterY = bestY + dy * push;
        if (!Command< GET_WATER_HEIGHT_AT_COORDS >(waterX, waterY, 1, &waterZ)) {
            waterX = bestX;
            waterY = bestY;
            waterZ = bestZ;
        }
        shoreX = pedPos.x;
        shoreY = pedPos.y;
        return nearestDistance <= kMaxShoreDistance;
    }
    bool HasWaterNearby(const CVector& pos) {
        for (int direction = 0; direction < 8; ++direction) {
            const float angle = static_cast<float>(direction) * 6.2831853f / 8.0f;
            for (float distance = 6.0f; distance <= 42.0f; distance += 6.0f) {
                float waterZ = 0.0f;
                if (Command< GET_WATER_HEIGHT_AT_COORDS >(
                    pos.x + cosf(angle) * distance,
                    pos.y + sinf(angle) * distance, 1, &waterZ)) return true;
            }
        }
        return false;
    }
    ManagedPed* FindState(int ref) {
        for (ManagedPed& s : g_managed) if (s.active && s.ref == ref) return &s;
        return nullptr;
    }
    ManagedPed* AllocateState() {
        for (ManagedPed& s : g_managed) if (!s.active) return &s;
        return nullptr;
    }
    SpawnedPed* AllocateSpawnedState() {
        for (SpawnedPed& s : g_spawned) if (!s.active) return &s;
        return nullptr;
    }
    void RemoveSpawnedPed(SpawnedPed& state) {
        CPed* ped = CPools::GetPed(state.ref);
        if (ped && ped->IsAlive()) Command< DELETE_CHAR >(state.ref);
        if (state.vehicleRef >= 0 && CPools::GetVehicle(state.vehicleRef))
            Command< DELETE_CAR >(state.vehicleRef);
        state = {};
        if (g_spawnCount > 0) --g_spawnCount;
    }
    void RemoveSpawnedPeds(CPlayerPed* player) {
        for (SpawnedPed& state : g_spawned) {
            if (!state.active) continue;
            CPed* ped = CPools::GetPed(state.ref);
            const CVector pos = ped ? ped->GetPosition() : player->GetPosition();
            if (!ped || !ped->IsAlive() ||
                !Command< IS_POINT_ON_SCREEN >(pos.x, pos.y, pos.z + 1.0f, 1.0f))
                RemoveSpawnedPed(state);
        }
        for (int i = 0; i < static_cast<int>(sizeof(kBeachVisitorModels) / sizeof(kBeachVisitorModels[0])); ++i) {
            if (g_visitorModelRequested[i]) Command< MARK_MODEL_AS_NO_LONGER_NEEDED >(kBeachVisitorModels[i]);
            g_visitorModelRequested[i] = false;
        }
        if (g_bikeModelRequested) {
            Command< MARK_MODEL_AS_NO_LONGER_NEEDED >(481);
            g_bikeModelRequested = false;
        }
        if (g_boatModelRequested) {
            Command< MARK_MODEL_AS_NO_LONGER_NEEDED >(473);
            g_boatModelRequested = false;
        }
    }
    void RequestVisitorModels() {
        for (int i = 0; i < static_cast<int>(sizeof(kBeachVisitorModels) / sizeof(kBeachVisitorModels[0])); ++i) {
            if (g_visitorModelRequested[i]) continue;
            Command< REQUEST_MODEL >(kBeachVisitorModels[i]);
            g_visitorModelRequested[i] = true;
        }
        if (!g_bikeModelRequested) {
            Command< REQUEST_MODEL >(481); // BMX
            g_bikeModelRequested = true;
        }
        if (!g_boatModelRequested) {
            Command< REQUEST_MODEL >(473); // Dinghy
            g_boatModelRequested = true;
        }
    }
    SpawnedPed* FindSpawnedState(int ref) {
        for (SpawnedPed& s : g_spawned) if (s.active && s.ref == ref) return &s;
        return nullptr;
    }
    int CountNearbyPeds(CPlayerPed* player) {
        CEntity* entities[kEntityCapacity] = {};
        i16 found = 0;
        CWorld::FindObjectsInRange(player->GetPosition(), 31.0f, true, &found,
            kEntityCapacity, entities, false, false, true, false, false);
        int count = 0;
        for (i16 i = 0; i < found && i < kEntityCapacity; ++i) {
            CEntity* entity = entities[i];
            if (!entity || !entity->IsPed()) continue;
            CPed* ped = entity->AsPed();
            if (ped && ped->IsAlive() && !ped->IsPlayer() && FindBeach(ped->GetPosition())) ++count;
        }
        return count;
    }
    bool StartActivity(CPed* ped, const BeachZone& beach, CPlayerPed* player,
                       unsigned int now, bool forceSwim = false);
    bool SpawnOneBeachVisitor(CPlayerPed* player) {
        if (!player || player->bInVehicle || g_spawnCount >= kMaxSpawnedBeachPeds) return false;
        bool modelsReady[sizeof(kBeachVisitorModels) / sizeof(kBeachVisitorModels[0])] = {};
        int readyCount = 0;
        for (int i = 0; i < static_cast<int>(sizeof(kBeachVisitorModels) / sizeof(kBeachVisitorModels[0])); ++i) {
            modelsReady[i] = Command< HAS_MODEL_LOADED >(kBeachVisitorModels[i]);
            if (modelsReady[i]) ++readyCount;
        }
        if (!readyCount) return false;

        static const float offsets[][2] = {
            { 50.0f, 0.0f }, { -50.0f, 0.0f }, { 0.0f, 50.0f }, { 0.0f, -50.0f },
            { 62.0f, 0.0f }, { -62.0f, 0.0f }, { 0.0f, 62.0f }, { 0.0f, -62.0f },
            { 48.0f, 34.0f }, { -48.0f, 34.0f }, { 48.0f, -34.0f }, { -48.0f, -34.0f },
            { 68.0f, 24.0f }, { -68.0f, 24.0f }, { 68.0f, -24.0f }, { -68.0f, -24.0f }
        };
        const CVector playerPos = player->GetPosition();
        for (const auto& offset : offsets) {
            const float x = playerPos.x + offset[0];
            const float y = playerPos.y + offset[1];
            if (!FindBeach(CVector(x, y, playerPos.z))) continue;
            bool groundFound = false;
            CEntity* groundEntity = nullptr;
            const float groundZ = CWorld::FindGroundZFor3DCoord(x, y, playerPos.z + 8.0f,
                &groundFound, &groundEntity);
            if (!groundFound || fabsf(groundZ - playerPos.z) > 2.5f ||
                Command< IS_POINT_ON_SCREEN >(x, y, groundZ + 1.0f, 0)) continue;

            int choice = std::rand() % readyCount;
            int modelIndex = -1;
            for (int i = 0; i < static_cast<int>(sizeof(kBeachVisitorModels) / sizeof(kBeachVisitorModels[0])); ++i) {
                if (!modelsReady[i]) continue;
                if (choice-- == 0) { modelIndex = i; break; }
            }
            if (modelIndex < 0) return false;
            const int model = kBeachVisitorModels[modelIndex];
            int pedRef = -1;
            const int pedType = model >= 138 && model <= 140 ? PED_TYPE_CIVFEMALE : PED_TYPE_CIVMALE;
            Command< CREATE_CHAR >(pedType, model, x, y, groundZ, &pedRef);
            CPed* ped = pedRef >= 0 ? CPools::GetPed(pedRef) : nullptr;
            if (!ped || !ped->IsAlive()) continue;
            Command< SET_CHAR_HEADING >(pedRef, static_cast<float>(std::rand() % 360));
            SpawnedPed* state = AllocateSpawnedState();
            if (!state) {
                Command< DELETE_CHAR >(pedRef);
                return false;
            }
            state->ref = pedRef;
            state->active = true;
            ++g_spawnCount;
            // Give newly created visitors a beach activity immediately. The next
            // scan can then rotate them to another activity when this one ends.
            const CVector spawnPos = ped->GetPosition();
            const BeachZone* beach = FindBeach(spawnPos);
            if (!beach || !StartActivity(ped, *beach, player, CTimer::GetTimeMS()))
                Command< TASK_WANDER_STANDARD >(pedRef);
            return true;
        }
        return false;
    }
    bool SpawnOneBeachSwimmer(CPlayerPed* player, const BeachZone& beach, unsigned int now) {
        if (!player || player->bInVehicle || g_spawnCount >= kMaxSpawnedBeachPeds ||
            !Command< HAS_MODEL_LOADED >(18)) return false;
        SpawnedPed* spawned = AllocateSpawnedState();
        if (!spawned || !AllocateState()) return false;

        const CVector playerPos = player->GetPosition();
        // Search the beach zone at a distance where a swimmer can be noticed
        // from shore, while keeping creation outside the camera view.
        for (int attempt = 0; attempt < 24; ++attempt) {
            const float angle = static_cast<float>(std::rand() % 628) / 100.0f;
            const float distance = 78.0f + static_cast<float>(std::rand() % 9200) / 100.0f;
            const float x = playerPos.x + cosf(angle) * distance;
            const float y = playerPos.y + sinf(angle) * distance;
            if (!FindBeach(CVector(x, y, playerPos.z))) continue;

            bool groundFound = false;
            CEntity* groundEntity = nullptr;
            const float groundZ = CWorld::FindGroundZFor3DCoord(x, y, playerPos.z + 12.0f,
                &groundFound, &groundEntity);
            if (!groundFound || fabsf(groundZ - playerPos.z) > 8.0f ||
                Command< IS_POINT_ON_SCREEN >(x, y, groundZ + 1.0f, 0)) continue;
            float surfaceWaterZ = 0.0f;
            if (Command< GET_WATER_HEIGHT_AT_COORDS >(x, y, 1, &surfaceWaterZ)) continue;

            if (!HasWaterNearby(CVector(x, y, groundZ))) continue;

            const int model = Command< HAS_MODEL_LOADED >(45) && (std::rand() & 1) ? 45 : 18;
            int pedRef = -1;
            Command< CREATE_CHAR >(static_cast<int>(PED_TYPE_CIVMALE), model, x, y, groundZ, &pedRef);
            CPed* ped = pedRef >= 0 ? CPools::GetPed(pedRef) : nullptr;
            if (!ped || !ped->IsAlive()) continue;

            Command< SET_CHAR_HEADING >(pedRef, static_cast<float>(std::rand() % 360));
            spawned->ref = pedRef;
            spawned->isSwimmer = true;
            spawned->expiresAt = now + 150000;
            spawned->active = true;
            ++g_spawnCount;
            if (!StartActivity(ped, beach, player, now, true)) {
                RemoveSpawnedPed(*spawned);
                return false;
            }
            ManagedPed* managed = FindState(pedRef);
            if (!managed || managed->activity != Activity::Swim) {
                if (managed) *managed = {};
                RemoveSpawnedPed(*spawned);
                return false;
            }
            return true;
        }
        return false;
    }
    bool SpawnOneBeachBoat(CPlayerPed* player, const BeachZone& beach, unsigned int now) {
        if (!player || player->bInVehicle || !g_boatModelRequested || !Command< HAS_MODEL_LOADED >(473) ||
            g_spawnCount >= kMaxSpawnedBeachPeds || !Command< HAS_MODEL_LOADED >(18)) return false;
        for (const SpawnedPed& spawned : g_spawned)
            if (spawned.active && spawned.isBoat) return false;
        SpawnedPed* state = AllocateSpawnedState();
        if (!state) return false;

        float edgeX, edgeY, edgeZ, shoreX, shoreY;
        if (!GetSwimTargets(beach, player->GetPosition(), edgeX, edgeY, edgeZ, shoreX, shoreY))
            return false;
        const float shoreToWaterX = edgeX - shoreX;
        const float shoreToWaterY = edgeY - shoreY;
        const float shoreToWaterLength = sqrtf(shoreToWaterX * shoreToWaterX + shoreToWaterY * shoreToWaterY);
        if (shoreToWaterLength < 1.0f) return false;
        const float seaDirX = shoreToWaterX / shoreToWaterLength;
        const float seaDirY = shoreToWaterY / shoreToWaterLength;
        const float boatX = edgeX + seaDirX * 24.0f;
        const float boatY = edgeY + seaDirY * 24.0f;
        const float targetX = boatX + seaDirX * 65.0f;
        const float targetY = boatY + seaDirY * 65.0f;

        float waterZ = 0.0f;
        float targetWaterZ = 0.0f;
        if (!Command< GET_WATER_HEIGHT_AT_COORDS >(boatX, boatY, 1, &waterZ) ||
            !Command< GET_WATER_HEIGHT_AT_COORDS >(targetX, targetY, 1, &targetWaterZ)) return false;
        const float distanceSqr = CVector(boatX, boatY, waterZ).DistSqr(player->GetPosition());
        if (distanceSqr < 38.0f * 38.0f || distanceSqr > 90.0f * 90.0f ||
            Command< IS_POINT_ON_SCREEN >(boatX, boatY, waterZ + 1.0f, 1.0f)) return false;

        int boatRef = -1;
        Command< CREATE_CAR >(473, boatX, boatY, waterZ, &boatRef);
        if (boatRef < 0 || !CPools::GetVehicle(boatRef)) return false;
        int pedRef = -1;
        Command< CREATE_CHAR >(static_cast<int>(PED_TYPE_CIVMALE), 18, boatX, boatY, waterZ + 1.0f, &pedRef);
        CPed* ped = pedRef >= 0 ? CPools::GetPed(pedRef) : nullptr;
        if (!ped || !ped->IsAlive()) {
            Command< DELETE_CAR >(boatRef);
            return false;
        }
        Command< TASK_WARP_CHAR_INTO_CAR_AS_DRIVER >(pedRef, boatRef);
        Command< SET_BOAT_CRUISE_SPEED >(boatRef, 10.0f);
        Command< BOAT_GOTO_COORDS >(boatRef, targetX, targetY, targetWaterZ);
        state->ref = pedRef;
        state->vehicleRef = boatRef;
        state->isBoat = true;
        state->expiresAt = now + 28000;
        state->startX = boatX;
        state->startY = boatY;
        state->active = true;
        ++g_spawnCount;
        return true;
    }
    void PruneSpawnedPeds(CPlayerPed* player, bool insideBeach, unsigned int now) {
        CVector pos = player->GetPosition();
        for (SpawnedPed& state : g_spawned) {
            if (!state.active) continue;
            CPed* ped = CPools::GetPed(state.ref);
            if (!ped || !ped->IsAlive()) {
                RemoveSpawnedPed(state);
                continue;
            }
            const CVector pedPos = ped->GetPosition();
            const float dx = pedPos.x - pos.x;
            const float dy = pedPos.y - pos.y;
            const float dz = pedPos.z - pos.z;
            const float distanceSqr = dx * dx + dy * dy + dz * dz;
            const bool onScreen = Command< IS_POINT_ON_SCREEN >(pedPos.x, pedPos.y, pedPos.z + 1.0f, 1.0f);
            if (state.isSwimmer) {
                const bool expired = static_cast<int>(now - state.expiresAt) >= 0;
                const bool outOfRange = distanceSqr > 180.0f * 180.0f;
                if ((expired || outOfRange || !insideBeach) && !onScreen)
                    RemoveSpawnedPed(state);
                continue;
            }
            if (state.isBoat) {
                if (!state.returning && static_cast<int>(now - state.expiresAt) >= 0 &&
                    state.vehicleRef >= 0 && CPools::GetVehicle(state.vehicleRef)) {
                    float waterZ = 0.0f;
                    if (Command< GET_WATER_HEIGHT_AT_COORDS >(state.startX, state.startY, 1, &waterZ)) {
                        Command< SET_BOAT_CRUISE_SPEED >(state.vehicleRef, 10.0f);
                        Command< BOAT_GOTO_COORDS >(state.vehicleRef, state.startX, state.startY, waterZ);
                    }
                    state.returning = true;
                }
                const bool returned = state.returning &&
                    (pedPos.x - state.startX) * (pedPos.x - state.startX) +
                    (pedPos.y - state.startY) * (pedPos.y - state.startY) < 14.0f * 14.0f;
                if ((distanceSqr > 95.0f * 95.0f || (!insideBeach && distanceSqr > 65.0f * 65.0f) || returned) && !onScreen)
                    RemoveSpawnedPed(state);
                continue;
            }
            if ((!insideBeach || distanceSqr > kSpawnCleanupDistance * kSpawnCleanupDistance) && !onScreen)
                RemoveSpawnedPed(state);
        }
    }
    bool IsBeachAnimLoaded() {
        if (!Command< HAS_ANIMATION_LOADED >("BEACH")) {
            if (!g_beachAnimRequested) {
                Command< REQUEST_ANIMATION >("BEACH");
                g_beachAnimRequested = true;
            }
            return Command< HAS_ANIMATION_LOADED >("BEACH");
        }
        return true;
    }
    bool IsAvailable(CPed* ped, CPlayerPed* player) {
        return ped && ped != player && ped->IsAlive() && !ped->bInVehicle && !ped->IsPlayer() &&
               !ped->bFallenDown && ped->IsPedInControl();
    }
    Activity ChooseActivity() {
        const int r = std::rand() % 100;
        if (r < 17) return Activity::Swim;
        if (r < 44) return Activity::LieDown;
        if (r < 59) return Activity::Sit;
        if (r < 76) return Activity::Chat;
        if (r < 90) return Activity::Bike;
        return Activity::Stroll;
    }
    CEntity* FindNearbySeatAttractor(CPed* ped, C2dEffect*& seatEffect) {
        CEntity* entities[kEntityCapacity] = {};
        i16 found = 0;
        CWorld::FindObjectsInRange(ped->GetPosition(), 3.0f, true, &found,
            kEntityCapacity, entities, true, false, false, true, false);
        for (i16 i = 0; i < found && i < kEntityCapacity; ++i) {
            CEntity* entity = entities[i];
            if (!entity || (!entity->IsObject() && !entity->IsBuilding())) continue;
            C2dEffect* effect = entity->GetRandom2dEffect(EFFECT_ATTRACTOR, 1);
            if (!effect) continue;
            const unsigned char type = effect->pedAttractor.m_nAttractorType;
            if (type != PED_ATTRACTOR_SEAT && type != PED_ATTRACTOR_STEP &&
                type != PED_ATTRACTOR_PARK) continue;
            seatEffect = effect;
            return entity;
        }
        seatEffect = nullptr;
        return nullptr;
    }
    int FindChatPartner(CPed* ped, CPlayerPed* player) {
        CEntity* entities[kEntityCapacity] = {};
        i16 found = 0;
        CWorld::FindObjectsInRange(ped->GetPosition(), 8.0f, true, &found,
            kEntityCapacity, entities, false, false, true, false, false);
        for (i16 i = 0; i < found && i < kEntityCapacity; ++i) {
            CEntity* entity = entities[i];
            if (!entity || !entity->IsPed()) continue;
            CPed* other = entity->AsPed();
            if (other && other != ped && other != player && other->IsAlive() &&
                !other->bInVehicle && !other->IsPlayer() && other->IsPedInControl() &&
                FindBeach(other->GetPosition())) {
                const int ref = CPools::GetPedRef(other);
                if (ref >= 0 && !FindState(ref)) return ref;
            }
        }
        return -1;
    }
    bool ChooseSandWaypoint(const BeachZone& beach, const CVector& from, float& targetX, float& targetY) {
        for (int attempt = 0; attempt < 12; ++attempt) {
            const float angle = static_cast<float>(std::rand() % 628) / 100.0f;
            const float distance = 9.0f + static_cast<float>(std::rand() % 800) / 100.0f;
            const float x = from.x + sinf(angle) * distance;
            const float y = from.y + cosf(angle) * distance;
            if (!FindBeach(CVector(x, y, from.z))) continue;
            float waterZ = 0.0f;
            if (Command< GET_WATER_HEIGHT_AT_COORDS >(x, y, 1, &waterZ)) continue;
            bool groundFound = false;
            CEntity* groundEntity = nullptr;
            CWorld::FindGroundZFor3DCoord(x, y, from.z + 8.0f, &groundFound, &groundEntity);
            if (!groundFound) continue;
            targetX = x;
            targetY = y;
            return true;
        }
        return false;
    }
    void DriveBikeAcrossSand(ManagedPed& state, CPed* ped, const BeachZone& beach) {
        CVehicle* bike = CPools::GetVehicle(state.vehicleRef);
        if (!ped || !bike) return;
        const CVector pos = bike->GetPosition();
        const float dx = state.targetX - pos.x;
        const float dy = state.targetY - pos.y;
        if ((dx * dx + dy * dy) < 16.0f ||
            !FindBeach(CVector(state.targetX, state.targetY, pos.z))) {
            if (!ChooseSandWaypoint(beach, pos, state.targetX, state.targetY)) {
                Command< SET_CAR_FORWARD_SPEED >(state.vehicleRef, 0.0f);
                return;
            }
        }
        const float angle = atan2f(state.targetX - pos.x, state.targetY - pos.y) * 57.2957795f;
        Command< SET_CAR_HEADING >(state.vehicleRef, angle);
        Command< SET_CAR_FORWARD_SPEED >(state.vehicleRef, 10.0f);
    }
    bool StartActivity(CPed* ped, const BeachZone& beach, CPlayerPed* player,
                       unsigned int now, bool forceSwim) {
        ManagedPed* state = AllocateState();
        if (!state) return false;
        const int ref = CPools::GetPedRef(ped);
        if (ref < 0) return false;
        Activity activity = forceSwim ? Activity::Swim : ChooseActivity();
        if (activity == Activity::Swim) {
            float waterX, waterY, waterZ, shoreX, shoreY;
            if (!GetSwimTargets(beach, ped->GetPosition(), waterX, waterY, waterZ, shoreX, shoreY))
                activity = Activity::Stroll;
            else {
                Command< TASK_SWIM_TO_COORD >(ref, waterX, waterY, waterZ);
                state->targetX = waterX;
                state->targetY = waterY;
                state->swimShoreX = shoreX;
                state->swimShoreY = shoreY;
                state->swimShoreZ = ped->GetPosition().z;
                state->expiresAt = now + 60000;
            }
        }
        if (activity == Activity::Bike &&
            (!FindSpawnedState(ref) || !g_bikeModelRequested || !Command< HAS_MODEL_LOADED >(481)))
            activity = Activity::Stroll;
        switch (activity) {
            case Activity::Swim:
                // The task is already started above after validating that this
                // pedestrian is close to real water.
                break;
            case Activity::LieDown:
                if (IsBeachAnimLoaded()) {
                    Command< TASK_PLAY_ANIM >(ref, "Lay_Bac_Loop", "BEACH", 4.0f,
                        1, 0, 0, 0, static_cast<int>(kRescheduleMs));
                }
                break;
            case Activity::Sit: {
                C2dEffect* effect = nullptr;
                CEntity* seat = FindNearbySeatAttractor(ped, effect);
                if (seat && effect) {
                    char attractorName[9] = {};
                    memcpy(attractorName, effect->pedAttractor.m_szScriptName, 8);
                    const CVector pos = seat->GetPosition();
                    Command< TASK_USE_CLOSEST_MAP_ATTRACTOR >(
                        ref, 3.0f, static_cast<int>(seat->m_nModelIndex), pos.x, pos.y, pos.z, attractorName);
                } else {
                    // Never issue TASK_SIT_DOWN in open space: it has no seat
                    // coordinate and can leave the pedestrian hovering.
                    activity = Activity::Stroll;
                    Command< TASK_WANDER_STANDARD >(ref);
                }
                break;
            }
            case Activity::Chat:
                {
                    const int partnerRef = FindChatPartner(ped, player);
                    ManagedPed* partnerState = partnerRef >= 0 ? AllocateState() : nullptr;
                    if (partnerRef >= 0 && partnerState) {
                        Command< TASK_CHAT_WITH_CHAR >(ref, partnerRef, 1, 1);
                        {
                            partnerState->ref = partnerRef;
                            partnerState->expiresAt = now + kRescheduleMs;
                            partnerState->activity = Activity::Chat;
                            partnerState->active = true;
                        }
                    } else {
                        // Never play a conversation animation with no partner.
                        Command< TASK_WANDER_STANDARD >(ref);
                    }
                }
                break;
            case Activity::Bike: {
                SpawnedPed* spawned = FindSpawnedState(ref);
                const CVector pos = ped->GetPosition();
                const float bikeX = pos.x + 2.5f;
                bool groundFound = false;
                CEntity* groundEntity = nullptr;
                const float groundZ = CWorld::FindGroundZFor3DCoord(bikeX, pos.y, pos.z + 5.0f,
                    &groundFound, &groundEntity);
                if (!spawned || !groundFound || fabsf(groundZ - pos.z) > 2.5f) {
                    activity = Activity::Stroll;
                    Command< TASK_WANDER_STANDARD >(ref);
                    break;
                }
                int bikeRef = -1;
                Command< CREATE_CAR >(481, bikeX, pos.y, groundZ, &bikeRef);
                if (bikeRef < 0 || !CPools::GetVehicle(bikeRef)) {
                    activity = Activity::Stroll;
                    Command< TASK_WANDER_STANDARD >(ref);
                    break;
                }
                spawned->vehicleRef = bikeRef;
                Command< SET_CAR_HEADING >(bikeRef, static_cast<float>(std::rand() % 360));
                Command< TASK_ENTER_CAR_AS_DRIVER >(ref, bikeRef, 12000);
                state->vehicleRef = bikeRef;
                ChooseSandWaypoint(beach, ped->GetPosition(), state->targetX, state->targetY);
                break;
            }
            case Activity::Stroll:
                Command< TASK_WANDER_STANDARD >(ref);
                break;
        }
        state->ref = ref;
        if (activity != Activity::Swim) state->expiresAt = now + kRescheduleMs;
        state->activity = activity;
        state->active = true;
        return true;
    }
    void RemoveManagedBike(ManagedPed& state) {
        if (state.vehicleRef >= 0 && CPools::GetVehicle(state.vehicleRef))
            Command< DELETE_CAR >(state.vehicleRef);
        SpawnedPed* spawned = FindSpawnedState(state.ref);
        if (spawned && spawned->vehicleRef == state.vehicleRef) spawned->vehicleRef = -1;
        state = {};
    }
    void Scan(CPlayerPed* player, const BeachZone& beach, unsigned int now) {
        int activityBudget = 3;
        for (ManagedPed& s : g_managed) {
            if (!s.active) continue;
            CPed* ped = CPools::GetPed(s.ref);
            if (!ped || !ped->IsAlive()) {
                if (s.activity == Activity::Bike) RemoveManagedBike(s);
                else s = {};
                continue;
            }
            if (s.activity == Activity::Bike) {
                const bool expired = static_cast<int>(now - s.expiresAt) >= 0;
                if (!s.vehicleDriving) {
                    if (ped->bInVehicle) {
                        if (s.vehicleRef >= 0 && CPools::GetVehicle(s.vehicleRef)) {
                            s.vehicleDriving = true;
                            s.expiresAt = now + 14000;
                            DriveBikeAcrossSand(s, ped, beach);
                        } else {
                            s = {};
                        }
                    } else if (expired) {
                        RemoveManagedBike(s);
                    }
                } else if (s.vehicleExitRequested) {
                    if (!ped->bInVehicle) {
                        RemoveManagedBike(s);
                    } else if (expired) {
                        Command< TASK_LEAVE_ANY_CAR >(s.ref);
                        s.expiresAt = now + 5000;
                    }
                } else if (expired) {
                    Command< TASK_LEAVE_ANY_CAR >(s.ref);
                    s.vehicleExitRequested = true;
                    s.expiresAt = now + 5000;
                } else {
                    DriveBikeAcrossSand(s, ped, beach);
                }
                continue;
            }
            if (s.activity == Activity::Swim) {
                const bool expired = static_cast<int>(now - s.expiresAt) >= 0;
                if (!expired) continue;
                const bool inWater = Command< IS_CHAR_IN_WATER >(s.ref);
                if (s.returningFromSwim) {
                    if (!inWater) {
                        s = {};
                    } else {
                        s.expiresAt = now + 10000;
                    }
                } else if (inWater && !s.swimReachedWater) {
                    // Keep swimmers in the water for a visible stretch before
                    // asking them to return. This prevents the old 30s timer
                    // from sending them back before they even reach the sea.
                    s.swimReachedWater = true;
                    s.expiresAt = now + 25000;
                    Command< TASK_SWIM_TO_COORD >(s.ref, s.targetX, s.targetY,
                        ped->GetPosition().z);
                } else if (!s.swimReachedWater && s.swimOutboundRetries < 3) {
                    // Path finding can be slow around beach props. Retry the
                    // outbound water target instead of instantly abandoning it.
                    ++s.swimOutboundRetries;
                    s.expiresAt = now + 20000;
                    float waterZ = ped->GetPosition().z;
                    Command< GET_WATER_HEIGHT_AT_COORDS >(s.targetX, s.targetY, 1, &waterZ);
                    Command< TASK_SWIM_TO_COORD >(s.ref, s.targetX, s.targetY, waterZ);
                } else if (s.swimReachedWater) {
                    Command< TASK_SWIM_TO_COORD >(s.ref,
                        s.swimShoreX, s.swimShoreY, s.swimShoreZ);
                    s.returningFromSwim = true;
                    s.expiresAt = now + 30000;
                } else {
                    // No water entry after several path attempts; release the
                    // ped to normal beach wandering rather than leaving a loop.
                    s = {};
                }
                continue;
            }
            if (ped->bInVehicle || static_cast<int>(now - s.expiresAt) >= 0) s = {};
        }
        CEntity* entities[kEntityCapacity] = {};
        i16 found = 0;
        CWorld::FindObjectsInRange(player->GetPosition(), kSearchRadius, true, &found,
            kEntityCapacity, entities, false, false, true, false, false);
        for (i16 i = 0; i < found && i < kEntityCapacity; ++i) {
            CEntity* entity = entities[i];
            if (!entity || !entity->IsPed()) continue;
            CPed* ped = entity->AsPed();
            if (!IsAvailable(ped, player)) continue;
            const CVector pos = ped->GetPosition();
            if (!FindBeach(pos) || FindState(CPools::GetPedRef(ped))) continue;
            // Keep the beach visibly active: process several eligible peds per
            // scan instead of changing only one every ~30 seconds.
            if (StartActivity(ped, beach, player, now) && --activityBudget <= 0) break;
        }
        // GTA can leave these beaches nearly empty on some traffic/population settings.
        // Top up only a few on-foot beach visitors, and only when the player is on foot.
        if (!player->bInVehicle && static_cast<int>(now - g_nextSpawnAt) >= 0 &&
            g_spawnCount < kMaxSpawnedBeachPeds) {
            bool spawned = false;
            int activeSwimmers = 0;
            for (const SpawnedPed& state : g_spawned)
                if (state.active && state.isSwimmer) ++activeSwimmers;

            // Establish distant swimmers first; later spawn slots can add more
            // visitors and boats. This attempt ignores nearby pedestrian count.
            const bool triedSwimmer = activeSwimmers < 2;
            if (triedSwimmer)
                spawned = SpawnOneBeachSwimmer(player, beach, now);

            if (!spawned) {
                const int spawnChoice = std::rand() % 10;
                if (spawnChoice < 5 && !triedSwimmer) {
                    spawned = SpawnOneBeachSwimmer(player, beach, now);
                } else if (spawnChoice < 7) {
                    spawned = SpawnOneBeachBoat(player, beach, now);
                }
            }
            if (!spawned && CountNearbyPeds(player) < 7)
                SpawnOneBeachVisitor(player);
            g_nextSpawnAt = now + kSpawnIntervalMs;
        }
    }
}

void Process(CPlayerPed* player) {
    if (!player) return;
    const CVector position = player->GetPosition();
    const BeachZone* beach = FindBeach(position);
    const unsigned int now = CTimer::GetTimeMS();
    PruneSpawnedPeds(player, beach != nullptr, now);
    if (!beach) {
        g_currentBeach = nullptr;
        g_nextSpawnAt = 0;
        RemoveSpawnedPeds(player);
        for (ManagedPed& state : g_managed) {
            if (!state.active) continue;
            CPed* ped = CPools::GetPed(state.ref);
            if (ped && ped->IsAlive() && !ped->bInVehicle)
                Command< TASK_WANDER_STANDARD >(state.ref);
            state = {};
        }
        return;
    }
    if (beach != g_currentBeach) {
        g_currentBeach = beach;
        g_nextSpawnAt = now + kBeachEntryDelayMs;
    }
    RequestVisitorModels();
    if (static_cast<int>(now - g_nextScan) < 0) return;
    g_nextScan = now + kScanMs;
    Scan(player, *beach, now);
}
}
