#pragma once

struct CPlayerPed;

// Bercak darah di badan player saat terkena peluru.
// Initialize(): pasang hook + callback render (panggil sekali di ON_MOD_LOAD).
// Process()   : dipanggil tiap frame dari gameProcessEvent.
namespace BloodStainSystem {
    void Initialize();
    void Process(CPlayerPed* player);
}
