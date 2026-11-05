#include <mod/amlmod.h>
#include <mod/logger.h>
#include <aml-psdk/game_sa/Events.h>
#include <aml-psdk/game_sa/entity/PlayerPed.h>

#include "helmet_system.h"
#include "npc_rescue_system.h"
#include "crash_response_system.h"
#include "rain_umbrella_system.h"
#include "truck_air_brake_system.h"
#include "beach_life_system.h"
#include "blood_stain_system.h"
#include "dynamic_vehicle_light_system.h"
#include "dynamic_light_system.h"
#include "complex_shadow_system.h"
#include "camera_garage_system.h"

// BetaRealismAction: independent gameplay systems are ticked from one entry point.
MYMOD(net.dimnih.betarealismaction, BetaRealismAction, 0.9.0, DimNih)

ON_MOD_LOAD() {
    logger->SetTag("BetaRealismAction");
    CrashResponseSystem::Initialize();
    BloodStainSystem::Initialize();
    DynamicVehicleLightSystem::Initialize();
    DynamicLightSystem::Initialize();
    ComplexShadowSystem::Initialize();
    CameraGarageSystem::Initialize();
    Events::gameProcessEvent += []() {
        CPlayerPed* player = FindPlayerPed(0);
        HelmetSystem::Process(player);
        NpcRescueSystem::Process();
        CrashResponseSystem::Process(player);
        RainUmbrellaSystem::Process(player);
        TruckAirBrakeSystem::Process(player);
        BeachLifeSystem::Process(player);
        BloodStainSystem::Process(player);
    };
}
