#include "helmet_system.h"

#include <mod/amlmod.h>
#include <mod/logger.h>

#include <aml-psdk/game_sa/entity/PlayerPed.h>
#include <aml-psdk/game_sa/entity/Vehicle.h>
#include <aml-psdk/game_sa/other/PlayerData.h>
#include <aml-psdk/game_sa/base/Timer.h>

#include <cstring>

namespace HelmetSystem {
namespace {
    struct PedClothesDescSnapshot {
        unsigned int modelKeys[10];
        unsigned int textureKeys[18];
        float fatStat;
        float muscleStat;
    };
    static_assert(sizeof(PedClothesDescSnapshot) == 0x78,
                  "Unexpected CPedClothesDesc layout");

    struct HelmetClothesVariant {
        const char* texture;
        const char* model;
    };
    constexpr HelmetClothesVariant kHelmetVariants[] = {
        { "helmet", "helmet" },
        { "moto", "moto" }
    };
    constexpr unsigned int kHelmetVariantCount =
        sizeof(kHelmetVariants) / sizeof(kHelmetVariants[0]);
    using SetTextureAndModelFn = void (*)(void*, const char*, const char*, int);
    using RebuildPlayerFn = void (*)(CPlayerPed*, bool);

    SetTextureAndModelFn g_setTextureAndModel = nullptr;
    RebuildPlayerFn g_rebuildPlayer = nullptr;
    bool g_nativeFunctionsResolved = false;
    bool g_nativeFunctionErrorLogged = false;
    bool g_clothesSaved = false;
    PedClothesDescSnapshot g_originalClothes = {};
    unsigned int g_randomState = 0;
    int g_lastVariant = -1;

    unsigned int NextRandom() {
        if (!g_randomState) {
            g_randomState = CTimer::GetTimeMS() ^
                static_cast<unsigned int>(reinterpret_cast<uintptr_t>(&g_originalClothes));
            if (!g_randomState) g_randomState = 0xA341316Cu;
        }
        g_randomState ^= g_randomState << 13;
        g_randomState ^= g_randomState >> 17;
        g_randomState ^= g_randomState << 5;
        return g_randomState;
    }

    int SelectHelmetVariant() {
        int variant = static_cast<int>(NextRandom() % kHelmetVariantCount);
        if (variant == g_lastVariant) variant = 1 - variant;
        g_lastVariant = variant;
        return variant;
    }

    bool ResolveClothesFunctions() {
        if (!g_nativeFunctionsResolved) {
            g_setTextureAndModel = GetMainLibrarySymbol<SetTextureAndModelFn>(
                "_ZN15CPedClothesDesc18SetTextureAndModelEPKcS1_i");
            g_rebuildPlayer = GetMainLibrarySymbol<RebuildPlayerFn>(
                "_ZN8CClothes13RebuildPlayerEP10CPlayerPedb");
            g_nativeFunctionsResolved = true;
        }
        if (g_setTextureAndModel && g_rebuildPlayer) return true;
        if (!g_nativeFunctionErrorLogged) {
            logger->Error("Native clothing symbols unavailable (SetTextureAndModel=%s, RebuildPlayer=%s)",
                          g_setTextureAndModel ? "found" : "missing",
                          g_rebuildPlayer ? "found" : "missing");
            g_nativeFunctionErrorLogged = true;
        }
        return false;
    }

    bool IsChangingVehicle(CPlayerPed* player) {
        return player->m_ePedState == PEDSTATE_ENTER_CAR ||
               player->m_ePedState == PEDSTATE_EXIT_CAR;
    }

    bool IsMotorcycleRider(CPlayerPed* player) {
        if (!player->bInVehicle || !player->m_pVehicle) return false;

        // Only stock powered motorcycles/scooters equip a helmet. Exclude BMX,
        // Bike, Mountain Bike, and Quadbike even if the engine groups them as bikes.
        switch (player->m_pVehicle->m_nModelIndex) {
            case 448: // Pizzaboy
            case 461: // PCJ-600
            case 462: // Faggio
            case 463: // Freeway
            case 468: // Sanchez
            case 521: // FCR-900
            case 522: // NRG-500
            case 523: // HPV1000
            case 581: // BF-400
            case 586: // Wayfarer
                return true;
            default:
                return false;
        }
    }

    bool ApplyHelmet(CPlayerPed* player) {
        if (!player->m_pPlayerData || !player->m_pPlayerData->m_pPedClothesDesc ||
            !ResolveClothesFunctions()) {
            return false;
        }

        std::memcpy(&g_originalClothes, player->m_pPlayerData->m_pPedClothesDesc,
                    sizeof(g_originalClothes));
        const int variantIndex = SelectHelmetVariant();
        const HelmetClothesVariant& variant = kHelmetVariants[variantIndex];
        g_setTextureAndModel(player->m_pPlayerData->m_pPedClothesDesc,
                             variant.texture, variant.model, 16);
        logger->Info("Applying player helmet variant %d", variantIndex + 1);
        g_rebuildPlayer(player, false);
        g_clothesSaved = true;
        logger->Info("Player helmet applied: texture '%s', model '%s', body part 16",
                     variant.texture, variant.model);
        return true;
    }

    bool RestoreClothes(CPlayerPed* player) {
        if (!player->m_pPlayerData || !player->m_pPlayerData->m_pPedClothesDesc ||
            !ResolveClothesFunctions()) {
            return false;
        }

        std::memcpy(player->m_pPlayerData->m_pPedClothesDesc, &g_originalClothes,
                    sizeof(g_originalClothes));
        logger->Info("Restoring player clothes");
        g_rebuildPlayer(player, false);
        g_clothesSaved = false;
        logger->Info("Player clothes restored");
        return true;
    }
}

void Process(CPlayerPed* player) {
    if (!player) return;

    const bool ridingMotorcycle = IsMotorcycleRider(player);
    if (ridingMotorcycle == g_clothesSaved || IsChangingVehicle(player)) return;
    if (!player->m_pPlayerData || !player->m_pPlayerData->m_pPedClothesDesc ||
        !ResolveClothesFunctions()) {
        return;
    }

    if (ridingMotorcycle) ApplyHelmet(player);
    else RestoreClothes(player);
}
}
