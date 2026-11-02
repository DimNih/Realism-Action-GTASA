#include "truck_air_brake_system.h"

#include <mod/amlmod.h>
#include <mod/logger.h>
#include <aml-psdk/game_sa/base/Timer.h>
#include <aml-psdk/game_sa/entity/PlayerPed.h>
#include <aml-psdk/game_sa/entity/Vehicle.h>
#include <aml-psdk/game_sa/engine/Pad.h>

#include <SLES/OpenSLES.h>
#include <SLES/OpenSLES_Android.h>
#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

namespace TruckAirBrakeSystem {
namespace {
    constexpr float kBrakeThreshold = 0.08f;
    constexpr float kMovingThresholdKmh = 8.0f;
    constexpr float kReverseThresholdKmh = 1.0f;
    // The squeal WAV is two seconds; leave a short gap before the air release.
    constexpr unsigned int kPostSquealHissDelayMs = 2150;
    constexpr unsigned int kHissCooldownMs = 1800;
    constexpr float kSquealGain = 0.65f;
    constexpr float kHissGain = 0.60f;

    struct PcmWave {
        std::vector<int16_t> samples;
        bool loaded = false;
    };

    SLObjectItf g_engineObject = nullptr;
    SLEngineItf g_engine = nullptr;
    SLObjectItf g_outputMixObject = nullptr;
    SLObjectItf g_playerObject = nullptr;
    SLPlayItf g_player = nullptr;
    SLAndroidSimpleBufferQueueItf g_queue = nullptr;
    SLVolumeItf g_volume = nullptr;
    PcmWave g_squeal;
    PcmWave g_hiss;
    std::vector<int16_t> g_playbackSamples;
    bool g_audioInitialized = false;
    bool g_audioLogged = false;
    bool g_hissPlayedForBrake = false;
    bool g_brakeCycleActive = false;
    int g_trackedVehicleModel = -1;
    unsigned int g_brakeStartedAt = 0;
    unsigned int g_lastHissAt = 0;

    uint32_t ReadU32(const unsigned char* p) {
        return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
               (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
    }

    bool LoadWave(const std::string& path, PcmWave& out) {
        std::ifstream file(path, std::ios::binary);
        if (!file) return false;
        std::vector<unsigned char> bytes((std::istreambuf_iterator<char>(file)), {});
        if (bytes.size() < 44 || memcmp(bytes.data(), "RIFF", 4) || memcmp(bytes.data() + 8, "WAVE", 4)) return false;
        uint16_t format = 0, channels = 0, bits = 0;
        uint32_t sampleRate = 0;
        size_t dataOffset = 0, dataSize = 0;
        for (size_t p = 12; p + 8 <= bytes.size();) {
            const uint32_t chunkSize = ReadU32(&bytes[p + 4]);
            const size_t chunkData = p + 8;
            if (chunkData + chunkSize > bytes.size()) return false;
            if (!memcmp(&bytes[p], "fmt ", 4) && chunkSize >= 16) {
                format = static_cast<uint16_t>(bytes[chunkData] | (bytes[chunkData + 1] << 8));
                channels = static_cast<uint16_t>(bytes[chunkData + 2] | (bytes[chunkData + 3] << 8));
                sampleRate = ReadU32(&bytes[chunkData + 4]);
                bits = static_cast<uint16_t>(bytes[chunkData + 14] | (bytes[chunkData + 15] << 8));
            } else if (!memcmp(&bytes[p], "data", 4)) {
                dataOffset = chunkData;
                dataSize = chunkSize;
            }
            p = chunkData + chunkSize + (chunkSize & 1U);
        }
        if (format != 1 || channels != 1 || bits != 16 || sampleRate != 44100 || !dataSize || dataSize % 2) return false;
        out.samples.resize(dataSize / 2);
        memcpy(out.samples.data(), bytes.data() + dataOffset, dataSize);
        out.loaded = true;
        return true;
    }

    bool InitializeAudio() {
        if (g_audioInitialized) return g_player != nullptr;
        if (!aml || !aml->GetAndroidDataPath()) return false;
        const std::string dir = std::string(aml->GetAndroidDataPath()) + "/AudioBetaRealismAction";
        LoadWave(dir + "/air_brake_squeal.wav", g_squeal);
        LoadWave(dir + "/air_brake_hiss.wav", g_hiss);

        if (slCreateEngine(&g_engineObject, 0, nullptr, 0, nullptr, nullptr) != SL_RESULT_SUCCESS ||
            !g_engineObject || (*g_engineObject)->Realize(g_engineObject, SL_BOOLEAN_FALSE) != SL_RESULT_SUCCESS ||
            (*g_engineObject)->GetInterface(g_engineObject, SL_IID_ENGINE, &g_engine) != SL_RESULT_SUCCESS) return false;
        if ((*g_engine)->CreateOutputMix(g_engine, &g_outputMixObject, 0, nullptr, nullptr) != SL_RESULT_SUCCESS ||
            (*g_outputMixObject)->Realize(g_outputMixObject, SL_BOOLEAN_FALSE) != SL_RESULT_SUCCESS) return false;

        SLDataLocator_AndroidSimpleBufferQueue queueLocator = { SL_DATALOCATOR_ANDROIDSIMPLEBUFFERQUEUE, 1 };
        SLDataFormat_PCM pcm = { SL_DATAFORMAT_PCM, 1, SL_SAMPLINGRATE_44_1,
            SL_PCMSAMPLEFORMAT_FIXED_16, SL_PCMSAMPLEFORMAT_FIXED_16,
            SL_SPEAKER_FRONT_CENTER, SL_BYTEORDER_LITTLEENDIAN };
        SLDataSource source = { &queueLocator, &pcm };
        SLDataLocator_OutputMix outputLocator = { SL_DATALOCATOR_OUTPUTMIX, g_outputMixObject };
        SLDataSink sink = { &outputLocator, nullptr };
        const SLInterfaceID ids[] = { SL_IID_ANDROIDSIMPLEBUFFERQUEUE, SL_IID_VOLUME };
        const SLboolean required[] = { SL_BOOLEAN_TRUE, SL_BOOLEAN_FALSE };
        if ((*g_engine)->CreateAudioPlayer(g_engine, &g_playerObject, &source, &sink, 2, ids, required) != SL_RESULT_SUCCESS ||
            (*g_playerObject)->Realize(g_playerObject, SL_BOOLEAN_FALSE) != SL_RESULT_SUCCESS ||
            (*g_playerObject)->GetInterface(g_playerObject, SL_IID_PLAY, &g_player) != SL_RESULT_SUCCESS ||
            (*g_playerObject)->GetInterface(g_playerObject, SL_IID_ANDROIDSIMPLEBUFFERQUEUE, &g_queue) != SL_RESULT_SUCCESS) return false;
        (*g_playerObject)->GetInterface(g_playerObject, SL_IID_VOLUME, &g_volume);
        if (g_volume) (*g_volume)->SetVolumeLevel(g_volume, -1000);
        g_audioInitialized = true;
        return true;
    }

    bool Play(const PcmWave& wave, float gain) {
        if (!wave.loaded || !InitializeAudio() || !g_player || !g_queue) return false;
        (*g_player)->SetPlayState(g_player, SL_PLAYSTATE_STOPPED);
        (*g_queue)->Clear(g_queue);
        g_playbackSamples.resize(wave.samples.size());
        for (size_t i = 0; i < wave.samples.size(); ++i)
            g_playbackSamples[i] = static_cast<int16_t>(wave.samples[i] * gain);
        if ((*g_queue)->Enqueue(g_queue, g_playbackSamples.data(), static_cast<SLuint32>(g_playbackSamples.size() * sizeof(int16_t))) != SL_RESULT_SUCCESS)
            return false;
        return (*g_player)->SetPlayState(g_player, SL_PLAYSTATE_PLAYING) == SL_RESULT_SUCCESS;
    }

    bool IsTruck(int model) {
        switch (model) {
            case 403: // Linerunner
            case 408: // Trashmaster
            case 414: // Mule
            case 443: // Packer
            case 455: // Flatbed
            case 456: // Yankee
            case 498: // Boxville
            case 499: // Benson
            case 514: // Tanker
            case 515: // Roadtrain
            case 524: // Cement Truck
            case 525: // Towtruck
            case 531: // Tractor
            case 552: // Utility Van
            case 578: // DFT-30
                return true;
            default: return false;
        }
    }
}

void Process(CPlayerPed* player) {
    if (!player || !player->bInVehicle || !player->m_pVehicle) {
        g_hissPlayedForBrake = false;
        g_brakeCycleActive = false;
        g_trackedVehicleModel = -1;
        return;
    }
    CVehicle* vehicle = player->m_pVehicle;
    const int model = vehicle->m_nModelIndex;
    if (!IsTruck(model)) {
        g_hissPlayedForBrake = false;
        g_brakeCycleActive = false;
        g_trackedVehicleModel = -1;
        return;
    }
    const unsigned int now = CTimer::GetTimeMS();
    if (g_trackedVehicleModel != model) {
        g_trackedVehicleModel = model;
        g_hissPlayedForBrake = false;
    }
    const CVector& forward = vehicle->GetForward();
    const float signedForwardSpeedKmh = 200.0f * (
        vehicle->m_vecMoveSpeed.x * forward.x +
        vehicle->m_vecMoveSpeed.y * forward.y +
        vehicle->m_vecMoveSpeed.z * forward.z);
    const float speed = vehicle->GetSpeedKmh();
    CPad* pad = CPad::GetPad(0);
    const bool brakeInput = vehicle->m_fBreakPedal > kBrakeThreshold || (pad && pad->GetBrake() > 0);
    const bool forwardBraking = brakeInput && signedForwardSpeedKmh > kMovingThresholdKmh;
    const bool movingBackward = signedForwardSpeedKmh < -kReverseThresholdKmh;
    if (forwardBraking && !g_brakeCycleActive) {
        g_brakeStartedAt = now;
        g_hissPlayedForBrake = false;
        g_brakeCycleActive = Play(g_squeal, kSquealGain);
    }
    if (g_brakeCycleActive && !movingBackward && !g_hissPlayedForBrake &&
        static_cast<int>(now - g_brakeStartedAt) >= static_cast<int>(kPostSquealHissDelayMs) &&
        static_cast<int>(now - g_lastHissAt) >= static_cast<int>(kHissCooldownMs)) {
        if (Play(g_hiss, kHissGain)) {
            g_hissPlayedForBrake = true;
            g_lastHissAt = now;
        }
    }
    if (movingBackward || (!brakeInput && speed > kMovingThresholdKmh)) {
        g_brakeCycleActive = false;
        g_hissPlayedForBrake = false;
    }
    if (!g_audioLogged) {
        g_audioLogged = true;
        const bool ok = InitializeAudio();
        logger->Info("Truck air-brake audio %s; WAVs at %s/AudioBetaRealismAction/ (squeal=%d hiss=%d)",
            ok ? "ready" : "unavailable", aml && aml->GetAndroidDataPath() ? aml->GetAndroidDataPath() : "AML data path unavailable",
            g_squeal.loaded ? 1 : 0, g_hiss.loaded ? 1 : 0);
    }
}
}
