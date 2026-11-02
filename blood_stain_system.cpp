#include "blood_stain_system.h"
#include "blood_decal_core.h"

#include <mod/amlmod.h>
#include <mod/logger.h>

#include <aml-psdk/game_sa/Events.h>
#include <aml-psdk/game_sa/base/Timer.h>
#include <aml-psdk/game_sa/engine/Fx.h>
#include <aml-psdk/gta_base/Rect.h>
#include <aml-psdk/gta_base/RGBA.h>
#include <aml-psdk/game_sa/entity/Camera.h>
#include <aml-psdk/game_sa/entity/PlayerPed.h>
#include <aml-psdk/game_sa/other/Coronas.h>
#include <aml-psdk/renderware/RwImage.h>
#include <aml-psdk/renderware/RwRaster.h>
#include <aml-psdk/renderware/RwRender.h>
#include <aml-psdk/renderware/RwTexture.h>
#include <aml-psdk/renderware/RwMatrix.h>
#include <aml-psdk/renderware/RwFrame.h>
#include <aml-psdk/renderware/RpAtomic.h>
#include <aml-psdk/renderware/RpClump.h>
#include <aml-psdk/renderware/RpGeometry.h>

#include <cmath>
#include <atomic>
#include <cstdarg>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <initializer_list>
#include <mutex>
#include <vector>
#include <android/log.h>

// ---------------------------------------------------------------------------
// ALUR (versi "menempel di kulit")
//
//  MASALAH LAMA
//   * Titik darah dari Fx_c::AddBlood adalah titik tabrak PELURU pada "bola tabrak"
//     (col sphere) ped, BUKAN titik di kulit. Di dada/perut bola itu bisa 5-12 cm
//     lebih tebal dari badan -> bercak melayang. Di sisi yang lebih sempit -> tembus.
//   * Anchor lama disimpan relatif ke matriks ROOT ped. Saat aim / jongkok / lari /
//     membungkuk, tulang punggung bergerak relatif ke root, sedangkan kulit ikut
//     tulang -> jarak muncul lagi walau titik awalnya benar.
//   * Patch datar dengan "sag" parabola menganggap badan silinder r=0.24 m. Badan
//     SA elips (lebar ~0.2, dalam ~0.12) -> tepi patch melayang / terkubur.
//
//  SOLUSI
//   Tier 1 (utama)  : bercak dibangun dari SEGITIGA MESH PLAYER itu sendiri
//                     (decal proyeksi). Saat render, vertex yang dipakai di-skin
//                     ulang dengan matriks tulang LIVE -> menempel di kulit di semua
//                     pose. Titik tembak di-snap ke kulit lewat raycast ke mesh
//                     ter-skin, jadi jarak col-sphere tidak berpengaruh.
//   Tier 2 (cadangan): bila API skin tidak bisa dibaca di build 2.10 ini, badan
//                     dimodelkan sebagai elips yang terikat poligon tulang punggung
//                     (pelvis-spine-torso-leher) + arah bahu, bukan matriks root.
//
//  Pilihan tier dicatat di log: cari baris "Tier1 validation" dan "Stain created".
// ---------------------------------------------------------------------------

namespace BC = BloodCore;

namespace BloodStainSystem {
namespace {

    // ----- Pengaturan (ubah di sini) ---------------------------------------
    constexpr bool     kForceTier2       = false;   // true = paksa model elips (untuk uji banding)
    constexpr bool     kLogCalibration   = true;    // log selisih mesh vs elips (kalibrasi Tier 2)
    constexpr int      kMaxStains        = 24;      // jumlah bercak maksimum di badan
    constexpr int      kMaxNewPerDamage  = 1;       // satu bercak per event damage; AddBlood dapat dipanggil beberapa kali per peluru
    constexpr uint32_t kPendingMaxAgeMs  = 500;     // toleransi urutan event/frame game
    constexpr uint32_t kSoakMs           = 1500;    // waktu bercak "meresap" membesar
    constexpr uint32_t kLifeMs           = 240000;  // umur bercak (4 menit)
    constexpr uint32_t kFadeMs           = 30000;   // fade-out di akhir umur
    constexpr float    kHitMaxAxisDist   = 0.60f;   // titik darah harus sedekat ini ke sumbu badan
    constexpr float    kMergeDistance    = 0.05f;   // hit berdekatan digabung jadi bercak lebih besar
    constexpr float    kMaxRadius        = 0.11f;   // radius maksimum setelah digabung (meter)
    constexpr float    kMaxDrawDistance  = 45.0f;
    constexpr bool     kBulletsOnly      = true;    // false = semua luka berdarah (pisau, dll)

    // Tier 1
    constexpr float    kMeshGatherRadius = 0.13f;   // segitiga dikumpulkan sampai radius ini (>= kMaxRadius)
    constexpr float    kRayReach         = 0.25f;   // raycast mulai dari titik kena + 25 cm ke arah luar
    constexpr float    kSnapMax          = 0.16f;   // titik kulit harus dalam jarak ini dari titik kena
    constexpr float    kMeshLiftBase     = 0.0035f; // angkat 3.5 mm di atas kulit (anti z-fight)
    constexpr float    kMeshLiftPerMeter = 0.0005f; // + 0.5 mm per meter jarak kamera (depth buffer 16 bit)
    constexpr float    kValidSaneRatio   = 0.97f;   // minimal vertex "masuk akal" saat validasi skin

    // Tier 2 fallback, aligned with the supplied BulletWound Mid Devices config:
    // BodyRadiusPercent=100, SkinOffsetMm=3, DepthBiasMm=12. This is still an
    // animated body-profile approximation, not the original mod's mesh decal.
    constexpr float    kTorsoShrink      = 1.00f;
    constexpr float    kTier2Lift        = 0.003f;
    // Do not move the fallback patch toward the camera: that is not a depth
    // bias and can push it through a thin torso or show it on the far side.

    constexpr int kPatchArcSegments = 24;
    constexpr int kPatchRadialRings = 5;
    constexpr int kPatchVertexCount = 1 + kPatchArcSegments * kPatchRadialRings;
    constexpr int kPatchIndexCount = kPatchArcSegments * 3 +
        (kPatchRadialRings - 1) * kPatchArcSegments * 6;
    constexpr uint32_t kIm3DVertexFlags = 0x0FU; // XYZ | NORMAL | RGBA | UV
    constexpr char kLogFileName[] = "/BetaRealismAction_BloodStain.log";

    // ----- Node tulang (eBoneTag GTA SA) --------------------------------------
    // Poligon tulang punggung: pelvis, spine1, torso atas, leher.
    constexpr unsigned kSpineNodes[4] = { 2, 3, 4, 5 };
    constexpr unsigned kNodeRShoulder = 22, kNodeLShoulder = 32;
    // Tulang yang boleh "memiliki" kulit bercak (batang tubuh + tulang selangka kiri/kanan).
    constexpr unsigned kAllowedNodes[] = { 0, 1, 2, 3, 4, 21, 31 };
    constexpr unsigned kBoneMapNodes[] = { 0, 1, 2, 3, 4, 5, 21, 22, 31, 32 };
    struct RestOffset { float x, y, z; };   // ped-local (kanan, depan, atas) bila tulang tak terbaca
    constexpr RestOffset kRestSpine[4] = {
        { 0.0f, 0.0f, -0.10f }, { 0.0f, 0.0f, 0.10f }, { 0.0f, 0.0f, 0.32f }, { 0.0f, 0.0f, 0.50f } };
    constexpr RestOffset kRestRShoulder = {  0.20f, 0.0f, 0.42f };
    constexpr RestOffset kRestLShoulder = { -0.20f, 0.0f, 0.42f };

    // ----- Struct RenderWare minimal (tata letak RW 3.6, offset diverifikasi) -----
    // RpHAnimHierarchy: {flags, numNodes, pMatrixArray(+8), pMatrixArrayUnaligned, pNodeInfo, ...}
    // Offset +8 untuk pMatrixArray sama dengan yang dipakai CPed::GetBonePosition.
    struct HAnimHierarchyLite {
        int32_t flags; int32_t numNodes;
        RwMatrix* matrixArray; void* matrixArrayUnaligned;
    };
    static_assert(offsetof(HAnimHierarchyLite, matrixArray) == 8, "RpHAnimHierarchy layout");

    // ----- Data bercak ---------------------------------------------------------
    enum StainMode : uint8_t { MODE_NONE = 0, MODE_MESH = 1, MODE_TORSO = 2 };
    struct Stain {
        bool        active = false;
        StainMode   mode = MODE_NONE;
        uint32_t    born = 0;
        float       radius = 0.045f;
        float       dripMax = 0.04f;
        float       shapeX = 1.0f;
        float       shapeY = 1.0f;
        float       shapeAngle = 0.0f;
        float       colorR = 1.0f;
        float       colorG = 1.0f;
        float       colorB = 1.0f;
        float       haloScale = 1.0f;
        uint8_t     haloAlpha = 0;
        // Tier 1: decal dari segitiga mesh + pusat (segitiga + barycentric)
        BC::MeshDecal mesh;
        uint16_t    cV[3] = {0, 0, 0};
        float       cB[3] = {1.0f, 0.0f, 0.0f};
        // Tier 2: parameter di permukaan elips
        float       s = 0.0f, theta = 0.0f;
    };

    struct Pending { CVector pos, dir; uint32_t time; };

    using GetBonePositionFn = void (*)(void* ped, RwV3d* out, unsigned int boneId, bool updateSkin);
    using AddBloodFn        = void (*)(void* self, CVector& pos, CVector& dir, int num, float light);
    using Im3DRenderIndexedPrimitiveFn = RwBool (*)(RwPrimitiveType type,
        RwImVertexIndex* indices, RwInt32 indicesNum);
    // API plugin skin RenderWare (opsional; dicari saat Initialize)
    using SkinGetSkinFn    = void* (*)(RpGeometry*);
    using SkinWeightsFn    = const float* (*)(const void* skin);      // RwMatrixWeights[numVerts] (4 float)
    using SkinIndicesFn    = const uint32_t* (*)(const void* skin);   // 4 byte indeks tulang per vertex
    using SkinMatricesFn   = const RwMatrix* (*)(const void* skin);   // inverse-bind per tulang
    using SkinNumBonesFn   = int32_t (*)(const void* skin);
    using AtomicHierFn     = void* (*)(const RpAtomic*);
    using ClumpHierFn      = void* (*)(RpClump*);
    using HierarchyMatricesFn = RwMatrix* (*)(void* hierarchy);

    Stain             g_stains[kMaxStains];
    Pending           g_pending[16];
    int               g_pendingCount = 0;
    std::mutex        g_mutex;        // antrean AddBlood
    std::mutex        g_stainMutex;   // array bercak (Process vs render)
    AddBloodFn        g_origAddBlood = nullptr;
    GetBonePositionFn g_getBonePos = nullptr;
    Im3DRenderIndexedPrimitiveFn g_im3dRenderIndexedPrimitive = nullptr;
    SkinGetSkinFn     g_skinGetSkin = nullptr;
    SkinWeightsFn     g_skinWeights = nullptr;
    SkinIndicesFn     g_skinIndices = nullptr;
    SkinMatricesFn    g_skinToBone = nullptr;
    SkinNumBonesFn    g_skinNumBones = nullptr;
    AtomicHierFn      g_atomicHier = nullptr;
    ClumpHierFn       g_clumpHier = nullptr;
    HierarchyMatricesFn g_hierarchyMatrices = nullptr;
    bool              g_skinApiReady = false;
    bool              g_hooked = false;
    bool              g_poseModeLogged = false;
    uint32_t          g_rng = 0x9E3779B9u;
    FILE*             g_logFile = nullptr;
    RwTexture*        g_fallbackBloodTexture = nullptr;
    std::atomic<uint32_t> g_hookCalls{0};
    uint32_t          g_hookDropped = 0;
    uint32_t          g_renderFrames = 0;
    uint32_t          g_lastTextureWarning = 0;
    uint32_t          g_lastRenderDiagnostic = 0;
    std::mutex        g_logMutex;

    enum Tier1State { T1_UNKNOWN, T1_OK, T1_FAILED };
    Tier1State        g_tier1 = T1_UNKNOWN;

    // status player antar frame
    bool     g_stateValid = false;
    float    g_prevHealthArmour = 0.0f;
    uint32_t g_prevLastDamaged = 0;
    bool     g_wasAlive = true;

    void BloodLog(int priority, const char* format, ...) {
        char message[768];
        va_list args;
        va_start(args, format);
        vsnprintf(message, sizeof(message), format, args);
        va_end(args);
        __android_log_write(priority, "BetaBloodStain", message);
        std::lock_guard<std::mutex> lock(g_logMutex);
        if (g_logFile) {
            fprintf(g_logFile, "%s\n", message);
            fflush(g_logFile);
        }
    }

    void OpenBloodLog() {
        if (!aml) {
            BloodLog(ANDROID_LOG_ERROR, "AML interface is null; cannot resolve GTA files path");
            return;
        }
        const char* path = aml->GetAndroidDataPath();
        const char* source = "GetAndroidDataPath";
        if (!path || !path[0]) {
            path = aml->GetDataPath();
            source = "GetDataPath fallback";
        }
        if (!path || !path[0]) {
            BloodLog(ANDROID_LOG_ERROR, "AML returned no data directory; file logging unavailable");
            return;
        }
        char logPath[512];
        const size_t pathLength = strlen(path);
        snprintf(logPath, sizeof(logPath), "%s%s%s", path,
             pathLength && path[pathLength - 1] == '/' ? "" : "/",
             kLogFileName[0] == '/' ? kLogFileName + 1 : kLogFileName);
        g_logFile = fopen(logPath, "a");
        if (g_logFile) {
            setvbuf(g_logFile, nullptr, _IOLBF, 0);
            BloodLog(ANDROID_LOG_INFO, "Log opened via %s: %s", source, logPath);
            BloodLog(ANDROID_LOG_INFO, "AML Android data path: %s", path);
        } else {
            BloodLog(ANDROID_LOG_ERROR, "Cannot open log file: %s", logPath);
        }
    }

    RwTexture* CreateFallbackBloodTexture() {
        constexpr int kTextureSize = 256;
        RwImage* image = RwImageCreate(kTextureSize, kTextureSize, 32);
        if (!image || !RwImageAllocatePixels(image) || !image->cpPixels || image->stride <= 0) {
            if (image) RwImageDestroy(image);
            BloodLog(ANDROID_LOG_ERROR, "Fallback blood texture: RwImage allocation failed");
            return nullptr;
        }

        for (int y = 0; y < image->height; ++y) {
            CRGBA* row = reinterpret_cast<CRGBA*>(image->cpPixels + y * image->stride);
            for (int x = 0; x < image->width; ++x) {
                const float u = (static_cast<float>(x) + 0.5f) / kTextureSize;
                const float v = (static_cast<float>(y) + 0.5f) / kTextureSize;
                const float px = (u - 0.5f) * 2.0f;
                const float py = (v - 0.5f) * 2.0f;
                const float angle = atan2f(py, px);
                // One soft, asymmetric stain silhouette. Low-frequency edge
                // variation avoids the dotted/noisy look of the old texture.
                const float radial = sqrtf((px * 0.90f) * (px * 0.90f) +
                                           (py * 1.10f) * (py * 1.10f));
                const float edge = 0.75f +
                    0.048f * sinf(angle * 2.0f + 0.4f) +
                    0.034f * sinf(angle * 3.0f - 1.0f) +
                    0.021f * sinf(angle * 5.0f + 0.7f) +
                    0.009f * sinf(angle * 7.0f - 0.3f);
                float coverage = clampfloat(0.0f, 1.0f, (edge - radial) / 0.085f + 0.5f);
                coverage = coverage * coverage * (3.0f - 2.0f * coverage);

                // A short, attached lower tail gives an organic wound edge;
                // unlike the former side lobes it never forms separate dots.
                const float tailX = px - 0.02f;
                const float tailY = py + 0.58f;
                const float tailDist = sqrtf((tailX / 0.25f) * (tailX / 0.25f) +
                                             (tailY / 0.38f) * (tailY / 0.38f));
                const float tail = clampfloat(0.0f, 1.0f, (1.0f - tailDist) * 3.0f);
                const float alpha = clampfloat(0.0f, 1.0f, fmaxf(coverage, tail * coverage));

                // Smooth, broad marbling creates dark clotted areas without
                // high-frequency noise, isolated dots, or pixel-sized grain.
                const float core = clampfloat(0.0f, 1.0f, (0.72f - radial) * 1.45f);
                const float cloud = 0.5f + 0.5f *
                    sinf(px * 4.0f + sinf(py * 2.6f + 0.4f)) *
                    sinf(py * 3.4f - px * 1.5f);
                const float clotA = expf(-(((px + 0.18f) * (px + 0.18f)) / 0.10f +
                                           ((py - 0.05f) * (py - 0.05f)) / 0.34f));
                const float clotB = expf(-(((px - 0.28f) * (px - 0.28f)) / 0.07f +
                                           ((py + 0.22f) * (py + 0.22f)) / 0.18f));
                const float darkMarbling = clampfloat(0.0f, 0.58f,
                    0.12f + core * 0.25f + (1.0f - cloud) * 0.20f +
                    fmaxf(clotA * 0.22f, clotB * 0.18f));
                const float shade = 0.94f + 0.06f * sinf(angle * 2.0f + radial * 3.0f);
                const unsigned char red = static_cast<unsigned char>(
                    clampfloat(0.0f, 255.0f, (112.0f * (1.0f - darkMarbling)) * shade));
                const unsigned char green = static_cast<unsigned char>(
                    clampfloat(0.0f, 255.0f, (4.0f + cloud * 3.0f) * shade));
                const unsigned char blue = static_cast<unsigned char>(
                    clampfloat(0.0f, 255.0f, (7.0f + cloud * 3.0f) * shade));
                row[x].Set(red, green, blue, static_cast<unsigned char>(alpha * 238.0f));
            }
        }

        RwInt32 width = image->width;
        RwInt32 height = image->height;
        RwInt32 depth = image->depth;
        RwInt32 format = 0;
        if (!RwImageFindRasterFormat(image, rwRASTERTYPETEXTURE,
                                     &width, &height, &depth, &format)) {
            RwImageDestroy(image);
            BloodLog(ANDROID_LOG_ERROR, "Fallback blood texture: no compatible RenderWare raster format");
            return nullptr;
        }

        RwRaster* raster = RwRasterCreate(width, height, depth,
                                          rwRASTERTYPETEXTURE | format);
        if (!raster || !RwRasterSetFromImage(raster, image)) {
            if (raster) RwRasterDestroy(raster);
            RwImageDestroy(image);
            BloodLog(ANDROID_LOG_ERROR, "Fallback blood texture: raster creation/upload failed");
            return nullptr;
        }
        RwImageDestroy(image);

        RwTexture* texture = RwTextureCreate(raster);
        if (!texture) {
            RwRasterDestroy(raster);
            BloodLog(ANDROID_LOG_ERROR, "Fallback blood texture: RwTextureCreate failed");
            return nullptr;
        }
        RwTextureSetName(texture, "beta_blood_stain");
        BloodLog(ANDROID_LOG_INFO, "Created smooth organic blood texture (%dx%d depth=%d format=0x%x)",
                 width, height, depth, format);
        return texture;
    }

    float Rand01() {
        g_rng ^= g_rng << 13; g_rng ^= g_rng >> 17; g_rng ^= g_rng << 5;
        return (g_rng & 0xFFFFFF) / 16777216.0f;
    }
    float RandRange(float a, float b) { return a + (b - a) * Rand01(); }

    void InitStainShape(Stain& stain) {
        stain.shapeX = RandRange(0.62f, 1.38f);
        stain.shapeY = RandRange(0.68f, 1.32f);
        stain.shapeAngle = RandRange(-3.14159265f, 3.14159265f);
        // Keep the palette within deep red/maroon; independently tint channels
        // so nearby impacts do not all look like identical stamps.
        stain.colorR = RandRange(0.78f, 1.00f);
        stain.colorG = RandRange(0.58f, 0.88f);
        stain.colorB = RandRange(0.62f, 0.92f);
        // A faint integrated splash halo appears only on a minority of hits;
        // it is a single soft underlay, never several detached droplets.
        if (Rand01() < 0.28f) {
            stain.haloScale = RandRange(1.28f, 1.62f);
            stain.haloAlpha = static_cast<uint8_t>(RandRange(22.0f, 48.0f));
        }
    }

    CVector FromPed(const CMatrix& m, const CVector& v) {
        return m.right * v.x + m.up * v.y + m.at * v.z;
    }
    template <typename T> BC::V3 ToV3(const T& v) { return BC::V3(v.x, v.y, v.z); }
    bool IsFiniteV(const BC::V3& v) {
        return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
    }
    BC::M4 ToM4(const RwMatrix& r) {
        BC::M4 m;
        m.right = ToV3(r.right); m.up = ToV3(r.up); m.at = ToV3(r.at); m.pos = ToV3(r.pos);
        return m;
    }
    BC::V3 PedOrigin(CPlayerPed* ped) {
        CMatrix* m = ped->GetMatrix();
        return m ? ToV3(m->GetPosition()) : BC::V3();
    }

    bool IsBulletWeapon(int w) {
        // pistol..sniper (22-34), minigun (38), uzi drive-by (75)
        return (w >= 22 && w <= 34) || w == 38 || w == 75;
    }

    float BaseRadiusFor(int w) {
        switch (w) {
            case 22: case 23: case 28: case 32: case 75: return 0.040f;  // pistol, uzi, tec9
            case 24: case 33: case 34:                   return 0.060f;  // deagle, rifle, sniper
            case 25: case 26: case 27:                   return 0.050f;  // shotgun
            case 29: case 30: case 31: case 38:          return 0.048f;  // mp5, ak, m4, minigun
            default:                                     return 0.045f;
        }
    }

    // ----- Pencarian simbol API skin ----------------------------------------------
    template <typename Fn>
    Fn ResolveAny(const char* label, std::initializer_list<const char*> names) {
        for (const char* n : names) {
            Fn f = GetMainLibrarySymbol<Fn>(n);
            if (f) {
                BloodLog(ANDROID_LOG_INFO, "Symbol %s resolved as %s at %p", label, n,
                         reinterpret_cast<void*>(f));
                return f;
            }
        }
        BloodLog(ANDROID_LOG_WARN, "Symbol %s NOT found (tried %d names)", label,
                 static_cast<int>(names.size()));
        return nullptr;
    }

    void ResolveSkinApi() {
        g_skinGetSkin = ResolveAny<SkinGetSkinFn>("RpSkinGeometryGetSkin",
            { "RpSkinGeometryGetSkin", "_Z21RpSkinGeometryGetSkinP10RpGeometry", "_Z21RpSkinGeometryGetSkinPK10RpGeometry" });
        g_skinWeights = ResolveAny<SkinWeightsFn>("RpSkinGetVertexBoneWeights",
            { "RpSkinGetVertexBoneWeights", "_Z26RpSkinGetVertexBoneWeightsPK6RpSkin", "_Z26RpSkinGetVertexBoneWeightsP6RpSkin" });
        g_skinIndices = ResolveAny<SkinIndicesFn>("RpSkinGetVertexBoneIndices",
            { "RpSkinGetVertexBoneIndices", "_Z26RpSkinGetVertexBoneIndicesPK6RpSkin", "_Z26RpSkinGetVertexBoneIndicesP6RpSkin" });
        g_skinToBone = ResolveAny<SkinMatricesFn>("RpSkinGetSkinToBoneMatrices",
            { "RpSkinGetSkinToBoneMatrices", "_Z27RpSkinGetSkinToBoneMatricesPK6RpSkin", "_Z27RpSkinGetSkinToBoneMatricesP6RpSkin" });
        g_skinNumBones = ResolveAny<SkinNumBonesFn>("RpSkinGetNumBones",
            { "RpSkinGetNumBones", "_Z17RpSkinGetNumBonesPK6RpSkin", "_Z17RpSkinGetNumBonesP6RpSkin" });
        g_atomicHier = ResolveAny<AtomicHierFn>("RpSkinAtomicGetHAnimHierarchy",
            { "RpSkinAtomicGetHAnimHierarchy", "_Z29RpSkinAtomicGetHAnimHierarchyPK8RpAtomic", "_Z29RpSkinAtomicGetHAnimHierarchyP8RpAtomic" });
        g_clumpHier = ResolveAny<ClumpHierFn>("GetAnimHierarchyFromSkinClump",
            { "_Z29GetAnimHierarchyFromSkinClumpP7RpClump" });
        g_hierarchyMatrices = ResolveAny<HierarchyMatricesFn>("RpHAnimHierarchyGetMatrixArray",
            { "_Z30RpHAnimHierarchyGetMatrixArrayP16RpHAnimHierarchy", "RpHAnimHierarchyGetMatrixArray" });
        g_skinApiReady = g_skinGetSkin && g_skinWeights && g_skinIndices && g_skinToBone &&
                         (g_atomicHier || g_clumpHier) && g_hierarchyMatrices;
        BloodLog(g_skinApiReady ? ANDROID_LOG_INFO : ANDROID_LOG_WARN,
                 "Skin API %s -> Tier 1 (skin-mesh decal) %s",
                 g_skinApiReady ? "complete" : "INCOMPLETE",
                 g_skinApiReady ? "candidate" : "unavailable, Tier 2 (torso ellipse) will be used");
    }

    // ----- Konteks skin per atomic ---------------------------------------------------
    constexpr int kMaxBones = 64;
    constexpr int kMaxCtx   = 3;

    struct SkinCtx {
        RpAtomic*  atomic = nullptr;
        RpGeometry* geom = nullptr;
        const HAnimHierarchyLite* hier = nullptr;
        int        numNodes = 0;     // jumlah node hierarki (matriks tulang)
        int        numTris = 0;
        bool       nodeMapAttempted = false;
        bool       nodeMapOk = false;
        int        nodeId[kMaxBones];
        BC::M4     boneM[kMaxBones];
        BC::M4     skinToBone[kMaxBones];
        uint8_t    allowed[kMaxBones];
        BC::SkinView view;
    };
    SkinCtx g_ctx[kMaxCtx];
    int     g_ctxCount = 0;

    struct AtomicList { RpAtomic* a[kMaxCtx + 2]; int n = 0; };

    RpAtomic* CollectAtomicCB(RpAtomic* atomic, void* data) {
        AtomicList* list = static_cast<AtomicList*>(data);
        if (list->n < kMaxCtx + 2) list->a[list->n++] = atomic;
        return atomic;
    }

    bool BuildNodeMap(CPlayerPed* ped, SkinCtx& c) {
        std::memset(c.allowed, 0, sizeof(c.allowed));
        for (int i = 0; i < kMaxBones; ++i) c.nodeId[i] = -1;
        c.nodeMapOk = false;
        c.nodeMapAttempted = true;
        if (!ped || !g_getBonePos) return false;

        // The reference plugin exposes RpHAnimHierarchyGetMatrixArray, but its
        // node-info plugin layout is not stable across GTA builds. Match node
        // IDs to the matrices by comparing against CPed::GetBonePosition rather
        // than dereferencing a guessed nodeInfo structure.
        constexpr int kMapCount = sizeof(kBoneMapNodes) / sizeof(kBoneMapNodes[0]);
        RwV3d positions[kMapCount];
        bool positionValid[kMapCount] = {};
        const BC::V3 origin = PedOrigin(ped);
        for (int n = 0; n < kMapCount; ++n) {
            positions[n] = {0.0f, 0.0f, 0.0f};
            g_getBonePos(ped, &positions[n], kBoneMapNodes[n], false);
            const BC::V3 p = ToV3(positions[n]);
            positionValid[n] = IsFiniteV(p) && BC::Len(p - origin) < 2.5f;
        }

        bool hasPelvis = false;
        int matched = 0;
        for (int i = 0; i < c.numNodes; ++i) {
            float best = 0.08f;
            int bestId = -1;
            for (int n = 0; n < kMapCount; ++n) {
                if (!positionValid[n]) continue;
                const float d = BC::Len(c.boneM[i].pos - ToV3(positions[n]));
                if (d < best) { best = d; bestId = static_cast<int>(kBoneMapNodes[n]); }
            }
            if (bestId < 0) continue;
            c.nodeId[i] = bestId;
            ++matched;
            if (bestId == 2) hasPelvis = true;
            for (unsigned allowed : kAllowedNodes)
                if (static_cast<unsigned>(bestId) == allowed) c.allowed[i] = 1;
        }
        c.nodeMapOk = hasPelvis && matched >= 4;
        BloodLog(c.nodeMapOk ? ANDROID_LOG_INFO : ANDROID_LOG_WARN,
            "Bone matrix map: %s matched=%d/%d pelvis=%d filter=%s",
            c.nodeMapOk ? "OK" : "unavailable", matched, c.numNodes, hasPelvis ? 1 : 0,
            c.nodeMapOk ? "torso bones" : "all mesh triangles (geometric filtering only)");
        return c.nodeMapOk;
    }

    bool FillSkinCtx(SkinCtx& c, RpAtomic* atomic, RpClump* clump, CPlayerPed* ped) {
        if (!atomic || !(atomic->object.object.flags & rpATOMICRENDER)) return false;
        RpGeometry* g = atomic->geometry;
        if (!g || g->numVertices <= 0 || g->numVertices > 30000 || g->numTriangles <= 0 ||
            !g->triangles || !g->morphTarget || !g->morphTarget[0].verts) return false;
        void* skin = g_skinGetSkin(g);
        if (!skin) return false;
        const float* weights = g_skinWeights(skin);
        const uint32_t* indices = g_skinIndices(skin);
        const RwMatrix* s2b = g_skinToBone(skin);
        if (!weights || !indices || !s2b) return false;
        void* h = g_atomicHier ? g_atomicHier(atomic) : nullptr;
        if (!h && g_clumpHier) h = g_clumpHier(clump);
        const HAnimHierarchyLite* hier = static_cast<const HAnimHierarchyLite*>(h);
        if (!hier || hier->numNodes <= 0 || hier->numNodes > kMaxBones)
            return false;
        RwMatrix* liveMatrices = g_hierarchyMatrices ? g_hierarchyMatrices(h) : hier->matrixArray;
        if (!liveMatrices) return false;

        const bool reuseNodeMap = c.hier == hier && c.nodeMapAttempted;
        const bool cachedNodeMapOk = c.nodeMapOk;
        int cachedNodeIds[kMaxBones];
        if (reuseNodeMap) {
            std::memcpy(cachedNodeIds, c.nodeId, sizeof(cachedNodeIds));
        }

        c.atomic = atomic;
        c.geom = g;
        c.hier = hier;
        c.numNodes = hier->numNodes;
        c.numTris = g->numTriangles;
        int skinBones = c.numNodes;
        if (g_skinNumBones) {
            const int sb = g_skinNumBones(skin);
            if (sb > 0 && sb < skinBones) skinBones = sb;
        }
        for (int i = 0; i < c.numNodes; ++i) c.boneM[i] = ToM4(liveMatrices[i]);
        for (int i = 0; i < skinBones; ++i) c.skinToBone[i] = ToM4(s2b[i]);

        if (reuseNodeMap) {
            std::memcpy(c.nodeId, cachedNodeIds, sizeof(c.nodeId));
            c.nodeMapOk = cachedNodeMapOk;
        } else {
            BuildNodeMap(ped, c);
        }

        c.view.bindPos = reinterpret_cast<const BC::V3*>(g->morphTarget[0].verts);
        c.view.weights = weights;
        c.view.indices = indices;
        c.view.skinToBone = c.skinToBone;
        c.view.boneMatrix = c.boneM;
        c.view.numBones = skinBones;
        c.view.numVerts = g->numVertices;
        return true;
    }

    int BuildSkinContexts(CPlayerPed* ped) {
        g_ctxCount = 0;
        if (!g_skinApiReady || !ped) return 0;
        RpClump* clump = ped->m_pRwClump;
        if (!clump) return 0;
        AtomicList list;
        RpClumpForAllAtomics(clump, CollectAtomicCB, &list);
        for (int i = 0; i < list.n && g_ctxCount < kMaxCtx; ++i) {
            if (FillSkinCtx(g_ctx[g_ctxCount], list.a[i], clump, ped)) ++g_ctxCount;
        }
        return g_ctxCount;
    }

    const SkinCtx* FindCtx(const void* geometryKey) {
        for (int i = 0; i < g_ctxCount; ++i) if (g_ctx[i].geom == geometryKey) return &g_ctx[i];
        return nullptr;
    }

    int FindNode(const SkinCtx& c, unsigned nodeId) {
        if (!c.nodeMapOk) return -1;
        for (int i = 0; i < c.numNodes; ++i)
            if (c.nodeId[i] == static_cast<int>(nodeId)) return i;
        return -1;
    }

    // ----- Posisi tulang -------------------------------------------------------------
    // Urutan: matriks hierarki LIVE -> CPed::GetBonePosition -> pose istirahat.
    BC::V3 BoneWorld(CPlayerPed* ped, const SkinCtx* c, unsigned node, const RestOffset& rest) {
        if (c) {
            const int i = FindNode(*c, node);
            if (i >= 0) return c->boneM[i].pos;
        }
        CMatrix* m = ped->GetMatrix();
        const CVector origin = m->GetPosition();
        if (g_getBonePos) {
            RwV3d v = {0.0f, 0.0f, 0.0f};
            g_getBonePos(ped, &v, node, false);
            const BC::V3 pv = ToV3(v);
            if (IsFiniteV(pv) && BC::Len(pv - ToV3(origin)) < 2.5f) return pv;
        }
        return ToV3(origin + FromPed(*m, CVector(rest.x, rest.y, rest.z)));
    }

    bool BuildFrame(CPlayerPed* ped, const SkinCtx* c, BC::TorsoFrame& f) {
        if (!ped->GetMatrix()) return false;
        BC::V3 pts[4];
        for (int i = 0; i < 4; ++i) pts[i] = BoneWorld(ped, c, kSpineNodes[i], kRestSpine[i]);
        const BC::V3 rs = BoneWorld(ped, c, kNodeRShoulder, kRestRShoulder);
        const BC::V3 ls = BoneWorld(ped, c, kNodeLShoulder, kRestLShoulder);
        f = BC::BuildTorsoFrame(pts, 4, rs, ls);
        if (!g_poseModeLogged) {
            BloodLog(ANDROID_LOG_INFO, "Torso frame bones from: %s",
                     (c && c->nodeMapOk) ? "live RpHAnim matrices" :
                     (g_getBonePos ? "CPed::GetBonePosition" : "rest pose (no animation follow)"));
            g_poseModeLogged = true;
        }
        return f.valid;
    }

    // ----- Validasi Tier 1 (sekali) ----------------------------------------------------
    // Memastikan data skin terbaca benar di build ini: vertex hasil skin harus berada di
    // sekitar player. Bila tidak -> Tier 1 dimatikan dan Tier 2 dipakai.
    void ValidateTier1(CPlayerPed* ped) {
        if (g_tier1 != T1_UNKNOWN) return;
        if (kForceTier2) {
            g_tier1 = T1_FAILED;
            BloodLog(ANDROID_LOG_INFO, "Tier1 validation: skipped (kForceTier2=true)");
            return;
        }
        if (!g_skinApiReady) {
            g_tier1 = T1_FAILED;
            BloodLog(ANDROID_LOG_WARN, "Tier1 validation: skin API incomplete -> Tier 2");
            return;
        }
        BuildSkinContexts(ped);
        static int retry = 0;
        if (g_ctxCount == 0) {
            if (retry++ < 5)
                BloodLog(ANDROID_LOG_WARN, "Tier1 validation: no readable skinned atomic yet (attempt %d)", retry);
            return;   // tetap UNKNOWN, coba lagi pada hit berikutnya
        }
        int best = 0;
        for (int i = 1; i < g_ctxCount; ++i)
            if (g_ctx[i].view.numVerts > g_ctx[best].view.numVerts) best = i;
        const SkinCtx& c = g_ctx[best];
        const BC::V3 org = PedOrigin(ped);
        int sane = 0;
        float minZ = 1e9f, maxZ = -1e9f;
        for (int i = 0; i < c.view.numVerts; ++i) {
            const BC::V3 v = BC::SkinVertex(c.view, i);
            if (IsFiniteV(v) && BC::Len(v - org) < 2.0f) {
                ++sane;
                minZ = std::min(minZ, v.z - org.z);
                maxZ = std::max(maxZ, v.z - org.z);
            }
        }
        const float ratio = c.view.numVerts > 0 ? static_cast<float>(sane) / c.view.numVerts : 0.0f;
        // silang: matriks hierarki vs CPed::GetBonePosition (hanya peringatan)
        if (g_getBonePos && c.nodeMapOk) {
            for (unsigned node : { 2u, 4u }) {
                const int i = FindNode(c, node);
                if (i < 0) continue;
                RwV3d v = {0.0f, 0.0f, 0.0f};
                g_getBonePos(ped, &v, node, false);
                BloodLog(ANDROID_LOG_INFO, "Tier1 cross-check node %u: matrix-vs-GetBonePosition diff=%.4f m",
                         node, BC::Len(c.boneM[i].pos - ToV3(v)));
            }
        }
        // Node-tag mapping is only used to exclude nearby arm/head triangles.
        // The build's exported skin APIs plus a sane world-space skin prove the
        // core data is usable, so do not disable mesh decals when node metadata
        // differs from our expected layout. In that case the surface search is
        // still constrained by the torso hit ray and local decal radius.
        const bool sanePose = std::isfinite(minZ) && std::isfinite(maxZ) &&
                              minZ > -2.0f && maxZ < 2.0f && (maxZ - minZ) > 0.6f;
        const bool ok = ratio >= kValidSaneRatio && sanePose;
        g_tier1 = ok ? T1_OK : T1_FAILED;
        BloodLog(ok ? ANDROID_LOG_INFO : ANDROID_LOG_ERROR,
            "Tier1 validation: %s | atomics=%d verts=%d tris=%d bones=%d/%d sane=%.1f%% nodeMap=%d "
            "hierFlags=0x%x skinnedZ=[%.2f..%.2f] -> %s",
            ok ? "OK" : "FAILED", g_ctxCount, c.view.numVerts, c.numTris, c.view.numBones, c.numNodes,
            ratio * 100.0f, c.nodeMapOk ? 1 : 0, static_cast<unsigned>(c.hier->flags), minZ, maxZ,
            ok ? (c.nodeMapOk ? "skin-mesh decals enabled; torso bone filter active" :
                                "skin-mesh decals enabled; geometric torso filtering only") :
                 "falling back to Tier 2 torso ellipse");
    }

    // ----- Utilitas pembuatan bercak -------------------------------------------------------
    void Barycentric(const BC::V3& p, const BC::V3& a, const BC::V3& b, const BC::V3& c, float out[3]) {
        const BC::V3 v0 = b - a, v1 = c - a, v2 = p - a;
        const float d00 = BC::Dot(v0, v0), d01 = BC::Dot(v0, v1), d11 = BC::Dot(v1, v1);
        const float d20 = BC::Dot(v2, v0), d21 = BC::Dot(v2, v1);
        const float den = d00 * d11 - d01 * d01;
        if (fabsf(den) < 1e-12f) { out[0] = 1.0f; out[1] = out[2] = 0.0f; return; }
        out[1] = (d11 * d20 - d01 * d21) / den;
        out[2] = (d00 * d21 - d01 * d20) / den;
        out[0] = 1.0f - out[1] - out[2];
    }

    Stain* FindSlot() {
        Stain* oldest = &g_stains[0];
        for (int i = 0; i < kMaxStains; ++i) {
            if (!g_stains[i].active) return &g_stains[i];
            if (g_stains[i].born < oldest->born) oldest = &g_stains[i];
        }
        return oldest;  // ganti yang paling lama
    }

    // Gabungkan ke bercak yang sudah ada bila cukup dekat. true = digabung.
    void GrowStain(Stain& s, uint32_t now) {
        s.radius = fminf(kMaxRadius, s.radius * 1.18f + 0.004f);
        s.born = (now - s.born > kSoakMs) ? now - kSoakMs : s.born;  // tidak mengulang animasi meresap
    }

    BC::V3 MeshStainCenter(const Stain& s, const SkinCtx& c) {
        BC::V3 p(0, 0, 0);
        for (int k = 0; k < 3; ++k) p += BC::SkinVertex(c.view, s.cV[k]) * s.cB[k];
        return p;
    }

    bool CreateStain(CPlayerPed* ped, const Pending& hit, uint32_t now, float damageAmount) {
        if (!ped->GetMatrix()) return false;
        if (g_tier1 == T1_UNKNOWN) ValidateTier1(ped);
        BuildSkinContexts(ped);   // pose terkini (juga dipakai ValidateTier1 di atas)
        const SkinCtx* boneCtx = g_ctxCount > 0 ? &g_ctx[0] : nullptr;
        BC::TorsoFrame frame;
        if (!BuildFrame(ped, boneCtx, frame)) {
            BloodLog(ANDROID_LOG_DEBUG, "Reject impact: torso frame invalid");
            return false;
        }
        const BC::V3 hp = ToV3(hit.pos);
        const BC::V3 hd = ToV3(hit.dir);

        // sumbu badan terdekat + arah "keluar"
        float sAxis, thAxis;
        BC::TorsoParams(frame, hp, 1.0f, sAxis, thAxis);
        const BC::AxisSample ax = BC::AxisAt(frame, sAxis);
        const float axisDist = BC::Len(hp - ax.point);
        if (axisDist > kHitMaxAxisDist) {
            BloodLog(ANDROID_LOG_DEBUG,
                "Reject impact: %.2f m from torso axis (limit %.2f) hit=(%.2f,%.2f,%.2f)",
                axisDist, kHitMaxAxisDist, hp.x, hp.y, hp.z);
            return false;
        }
        BC::V3 outDir = BC::Norm(hp - ax.point, ax.Y);
        if (axisDist < 0.03f) outDir = BC::Norm(hd * -1.0f, ax.Y);

        // Keep one wound per processed damage event. Higher damage enlarges
        // the wound gradually; it does not create extra copies for AddBlood's
        // particle callbacks.
        const float damageScale = clampfloat(0.78f, 1.42f,
            0.82f + sqrtf(fmaxf(damageAmount, 1.0f) / 6.0f) * 0.24f);
        const float baseRadius = BaseRadiusFor(ped->m_nLastWeaponDamage) *
                                 RandRange(0.82f, 1.22f) * damageScale;

        // =========================== TIER 1: decal dari mesh ===========================
        if (g_tier1 == T1_OK && g_ctxCount > 0) {
            static std::vector<uint16_t> triBest, triTmp;
            static std::vector<BC::V3>   skBest, skTmp;
            int bestCtx = -1;
            BC::SurfaceHit bestHit;
            float bestDist = 1e9f;
            for (int ci = 0; ci < g_ctxCount; ++ci) {
                const SkinCtx& c = g_ctx[ci];
                triTmp.resize(static_cast<size_t>(c.numTris) * 3);
                for (int t = 0; t < c.numTris; ++t) {
                    triTmp[t * 3]     = c.geom->triangles[t].vertIndex[0];
                    triTmp[t * 3 + 1] = c.geom->triangles[t].vertIndex[1];
                    triTmp[t * 3 + 2] = c.geom->triangles[t].vertIndex[2];
                }
                skTmp.resize(c.view.numVerts);
                for (int v = 0; v < c.view.numVerts; ++v) skTmp[v] = BC::SkinVertex(c.view, v);
                BC::BoneFilter flt;
                flt.allowedBones = c.nodeMapOk ? c.allowed : nullptr;
                flt.count = c.numNodes;
                const BC::SurfaceHit sh = BC::FindSkinSurface(c.view, triTmp.data(), c.numTris,
                    skTmp.data(), flt, hp, outDir, kRayReach, kSnapMax);
                if (!sh.ok) continue;
                const float d = BC::Len(sh.point - hp);
                if (d < bestDist) {
                    bestDist = d; bestCtx = ci; bestHit = sh;
                    triBest.swap(triTmp);
                    skBest.swap(skTmp);
                }
            }
            if (bestCtx < 0 || bestDist > kSnapMax) {
                BloodLog(ANDROID_LOG_DEBUG,
                    "Reject impact: no torso skin within %.2f m (limb/head hit?) best=%.3f", kSnapMax,
                    bestCtx < 0 ? -1.0f : bestDist);
                return false;
            }
            const SkinCtx& c = g_ctx[bestCtx];

            // gabung dengan bercak lama di kulit yang sama
            for (int i = 0; i < kMaxStains; ++i) {
                Stain& o = g_stains[i];
                if (!o.active || o.mode != MODE_MESH) continue;
                const SkinCtx* oc = FindCtx(o.mesh.geometryKey);
                if (!oc) continue;
                if (BC::Len(MeshStainCenter(o, *oc) - bestHit.point) < kMergeDistance) {
                    GrowStain(o, now);
                    return true;
                }
            }

            // arah aliran = gravitasi diproyeksikan ke permukaan
            const BC::V3 n = bestHit.normal;
            BC::V3 down(0.0f, 0.0f, -1.0f);
            BC::V3 flow = down - n * BC::Dot(down, n);
            if (BC::Len(flow) < 0.25f) {
                const BC::V3 alt = ax.Z * -1.0f;
                flow = alt - n * BC::Dot(alt, n);
            }

            Stain* s = FindSlot();
            *s = Stain();
            BC::BoneFilter flt;
            flt.allowedBones = c.nodeMapOk ? c.allowed : nullptr;
            flt.count = c.numNodes;
            if (!BC::BuildMeshDecal(s->mesh, c.view, triBest.data(), c.numTris, skBest.data(), flt,
                                    bestHit.point, n, flow, ax.point, kMeshGatherRadius, c.geom)) {
                BloodLog(ANDROID_LOG_DEBUG, "Reject impact: BuildMeshDecal produced no triangles");
                return false;
            }
            const int ti = bestHit.tri;
            for (int k = 0; k < 3; ++k) s->cV[k] = triBest[ti * 3 + k];
            Barycentric(bestHit.point, skBest[s->cV[0]], skBest[s->cV[1]], skBest[s->cV[2]], s->cB);
            s->active = true;
            s->mode = MODE_MESH;
            s->born = now;
            s->radius = baseRadius;
            s->dripMax = RandRange(0.03f, 0.07f);
            InitStainShape(*s);

            BloodLog(ANDROID_LOG_INFO,
                "Stain created [Tier1 mesh]: hitToSkin=%.3f m tris=%d verts=%d radius=%.3f damage=%.1f halo=%u "
                "weapon=%d "
                "hit=(%.2f,%.2f,%.2f) skin=(%.2f,%.2f,%.2f) n=(%.2f,%.2f,%.2f)",
                bestDist, s->mesh.triCount, s->mesh.vertCount, s->radius, damageAmount,
                static_cast<unsigned>(s->haloAlpha),
                static_cast<int>(ped->m_nLastWeaponDamage), hp.x, hp.y, hp.z,
                bestHit.point.x, bestHit.point.y, bestHit.point.z, n.x, n.y, n.z);

            if (kLogCalibration) {
                // bandingkan kulit mesh vs elips Tier 2 untuk kalibrasi kTorsoProfile
                float sm, tm;
                BC::TorsoParams(frame, bestHit.point, 1.0f, sm, tm);
                BC::V3 mp, mn;
                BC::TorsoSurface(frame, sm, tm, 1.0f, mp, mn);
                const BC::AxisSample am = BC::AxisAt(frame, sm);
                BloodLog(ANDROID_LOG_INFO,
                    "Tier2 calib: s=%.2f theta=%.0fdeg meshRadial=%.3f modelRadial=%.3f delta=%+.3f (positif = elips terlalu kecil)",
                    sm, tm * 57.2957795f, BC::Len(bestHit.point - am.point), BC::Len(mp - am.point),
                    BC::Len(bestHit.point - am.point) - BC::Len(mp - am.point));
            }
            return true;
        }

        // =========================== TIER 2: elips terikat kerangka =====================
        float sP, thP;
        BC::TorsoParams(frame, hp, kTorsoShrink, sP, thP);
        const float arc = BC::TorsoArcScale(sP, thP, kTorsoShrink);
        for (int i = 0; i < kMaxStains; ++i) {
            Stain& o = g_stains[i];
            if (!o.active || o.mode != MODE_TORSO) continue;
            const float dAlong = fabsf(o.s - sP) * frame.length;
            float dTh = fabsf(o.theta - thP);
            if (dTh > 3.14159265f) dTh = 6.2831853f - dTh;
            if (dAlong < kMergeDistance && dTh * arc < kMergeDistance) { GrowStain(o, now); return true; }
        }
        Stain* s = FindSlot();
        *s = Stain();
        s->active = true;
        s->mode = MODE_TORSO;
        s->s = sP;
        s->theta = thP;
        s->born = now;
        s->radius = baseRadius;
        s->dripMax = RandRange(0.03f, 0.07f);
        InitStainShape(*s);
        BloodLog(ANDROID_LOG_INFO,
            "Stain created [Tier2 ellipse]: s=%.2f theta=%.0fdeg radius=%.3f damage=%.1f halo=%u axisDist=%.3f weapon=%d tier1=%s",
            sP, thP * 57.2957795f, s->radius, damageAmount,
            static_cast<unsigned>(s->haloAlpha), axisDist, static_cast<int>(ped->m_nLastWeaponDamage),
            g_tier1 == T1_OK ? "ok" : (g_tier1 == T1_FAILED ? "failed" : "unknown"));
        return true;
    }

    void ClearStains() {
        std::lock_guard<std::mutex> lock(g_stainMutex);
        for (int i = 0; i < kMaxStains; ++i) g_stains[i].active = false;
    }

    // ----- Hook ------------------------------------------------------------------
    void HookAddBlood(void* self, CVector& pos, CVector& dir, int num, float light) {
        uint32_t callNumber;
        uint32_t dropped = 0;
        uint32_t totalDropped = 0;
        int pendingCount = 0;
        {
            std::lock_guard<std::mutex> lock(g_mutex);
            callNumber = ++g_hookCalls;
            const int capacity = static_cast<int>(sizeof(g_pending) / sizeof(g_pending[0]));
            if (g_pendingCount >= capacity) {
                memmove(g_pending, g_pending + 1, sizeof(g_pending[0]) * (capacity - 1));
                --g_pendingCount;
                dropped = ++g_hookDropped;
            }
            g_pending[g_pendingCount++] = { pos, dir, CTimer::GetTimeMS() };
            pendingCount = g_pendingCount;
            totalDropped = g_hookDropped;
        }
        if (callNumber <= 8 || callNumber % 100 == 0) {
            BloodLog(ANDROID_LOG_DEBUG,
                "Fx_c::AddBlood hit hook call=%u pending=%d dropped=%u pos=(%.2f,%.2f,%.2f) dir=(%.2f,%.2f,%.2f) num=%d",
                callNumber, pendingCount, dropped ? dropped : totalDropped,
                pos.x, pos.y, pos.z, dir.x, dir.y, dir.z, num);
        }
        if (g_origAddBlood) g_origAddBlood(self, pos, dir, num, light);
    }

    // ----- Menggambar --------------------------------------------------------------------
    // Decal dari mesh: vertex = vertex kulit ter-skin; UV dipanggang saat kena tembak.
    bool DrawMeshBlot(const BC::MeshDecal& d, const BC::V3* pos, const BC::V3* nrm,
                      float cu, float cv, float uvScale, float shapeX, float shapeY,
                      float shapeAngle, unsigned char tintR, unsigned char tintG,
                      unsigned char tintB, unsigned char alpha) {
        if (!g_im3dRenderIndexedPrimitive || d.vertCount <= 0 || uvScale <= 0.0f) return false;
        RwIm3DVertex vertices[BC::kDecalMaxVerts];
        RwImVertexIndex indices[BC::kDecalMaxTris * 3];
        float tu[BC::kDecalMaxVerts], tv[BC::kDecalMaxVerts];
        const float cs = cosf(shapeAngle), sn = sinf(shapeAngle);
        for (int i = 0; i < d.vertCount; ++i) {
            const float x = (d.uv[i][0] - cu) * uvScale;
            const float y = (d.uv[i][1] - cv) * uvScale;
            const float rotatedX = x * cs - y * sn;
            const float rotatedY = x * sn + y * cs;
            tu[i] = 0.5f + 0.5f * rotatedX / shapeX;
            tv[i] = 0.5f + 0.5f * rotatedY / shapeY;
            RwIm3DVertex& v = vertices[i];
            v.x = pos[i].x; v.y = pos[i].y; v.z = pos[i].z;
            v.normal = CVector(nrm[i].x, nrm[i].y, nrm[i].z);
            v.r = tintR; v.g = tintG; v.b = tintB; v.a = alpha;
            v.u = tu[i]; v.v = tv[i];
        }
        int n = 0;
        for (int t = 0; t < d.triCount; ++t) {
            const int a = d.tri[t][0], b = d.tri[t][1], c = d.tri[t][2];
            // lewati segitiga yang seluruhnya di luar tekstur (alpha pasti 0)
            const float minU = std::min(tu[a], std::min(tu[b], tu[c]));
            const float maxU = std::max(tu[a], std::max(tu[b], tu[c]));
            const float minV = std::min(tv[a], std::min(tv[b], tv[c]));
            const float maxV = std::max(tv[a], std::max(tv[b], tv[c]));
            if (maxU < 0.0f || minU > 1.0f || maxV < 0.0f || minV > 1.0f) continue;
            indices[n++] = static_cast<RwImVertexIndex>(a);
            indices[n++] = static_cast<RwImVertexIndex>(b);
            indices[n++] = static_cast<RwImVertexIndex>(c);
        }
        if (n == 0) return false;
        void* transformed = RwIm3DTransform(reinterpret_cast<RwIm2DVertex*>(vertices),
                                            d.vertCount, nullptr, kIm3DVertexFlags);
        if (!transformed) return false;
        const bool rendered = g_im3dRenderIndexedPrimitive(rwPRIMTYPETRILIST, indices, n) != 0;
        RwIm3DEnd();
        return rendered;
    }

    // Tier 2: patch (arc x ring) yang menempel pada permukaan elips model.
    bool DrawTorsoBlot(const BC::TorsoFrame& f, float s, float theta, float R, float downShift,
                       float acrossShift, float shapeX, float shapeY, float shapeAngle,
                       unsigned char tintR, unsigned char tintG, unsigned char tintB,
                       unsigned char alpha) {
        if (!g_im3dRenderIndexedPrimitive || R < 0.008f) return false;
        RwIm3DVertex vertices[kPatchVertexCount];
        RwImVertexIndex indices[kPatchIndexCount];
        const float arc = BC::TorsoArcScale(s, theta, kTorsoShrink);
        const float shapeCos = cosf(shapeAngle), shapeSin = sinf(shapeAngle);
        auto setVertex = [&](int index, float diskX, float diskY) {
            const float turnedX = diskX * shapeCos - diskY * shapeSin;
            const float turnedY = diskX * shapeSin + diskY * shapeCos;
            const float down   = turnedX * R * shapeX + downShift;      // meter ke bawah
            const float across = turnedY * R * shapeY + acrossShift;    // meter mengelilingi badan
            BC::V3 p, n;
            BC::TorsoSurface(f, s - down / f.length, theta + across / arc, kTorsoShrink, p, n);
            p = p + n * kTier2Lift;
            RwIm3DVertex& v = vertices[index];
            v.x = p.x; v.y = p.y; v.z = p.z;
            v.normal = CVector(n.x, n.y, n.z);
            v.r = tintR; v.g = tintG; v.b = tintB; v.a = alpha;
            v.u = 0.5f + diskX * 0.5f;
            v.v = 0.5f + diskY * 0.5f;
        };
        setVertex(0, 0.0f, 0.0f);
        for (int ring = 1; ring <= kPatchRadialRings; ++ring) {
            const float radial = static_cast<float>(ring) / kPatchRadialRings;
            for (int seg = 0; seg < kPatchArcSegments; ++seg) {
                const float angle = 6.28318530718f * seg / kPatchArcSegments;
                setVertex(1 + (ring - 1) * kPatchArcSegments + seg, cosf(angle) * radial, sinf(angle) * radial);
            }
        }
        int n = 0;
        for (int seg = 0; seg < kPatchArcSegments; ++seg) {
            const int next = (seg + 1) % kPatchArcSegments;
            indices[n++] = 0;
            indices[n++] = static_cast<RwImVertexIndex>(1 + seg);
            indices[n++] = static_cast<RwImVertexIndex>(1 + next);
        }
        for (int ring = 1; ring < kPatchRadialRings; ++ring) {
            const int innerStart = 1 + (ring - 1) * kPatchArcSegments;
            const int outerStart = 1 + ring * kPatchArcSegments;
            for (int seg = 0; seg < kPatchArcSegments; ++seg) {
                const int next = (seg + 1) % kPatchArcSegments;
                const RwImVertexIndex i0 = static_cast<RwImVertexIndex>(innerStart + seg);
                const RwImVertexIndex i1 = static_cast<RwImVertexIndex>(innerStart + next);
                const RwImVertexIndex o0 = static_cast<RwImVertexIndex>(outerStart + seg);
                const RwImVertexIndex o1 = static_cast<RwImVertexIndex>(outerStart + next);
                indices[n++] = i0; indices[n++] = o0; indices[n++] = i1;
                indices[n++] = i1; indices[n++] = o0; indices[n++] = o1;
            }
        }
        void* transformed = RwIm3DTransform(reinterpret_cast<RwIm2DVertex*>(vertices),
                                            kPatchVertexCount, nullptr, kIm3DVertexFlags);
        if (!transformed) return false;
        const bool rendered = g_im3dRenderIndexedPrimitive(rwPRIMTYPETRILIST, indices, n) != 0;
        RwIm3DEnd();
        return rendered;
    }

    void RenderStains(CPed* renderedPed) {
        ++g_renderFrames;
        bool any = false, anyMesh = false, anyTorso = false;
        int activeCount = 0;
        for (int i = 0; i < kMaxStains; ++i) {
            if (!g_stains[i].active) continue;
            any = true; ++activeCount;
            if (g_stains[i].mode == MODE_MESH) anyMesh = true;
            if (g_stains[i].mode == MODE_TORSO) anyTorso = true;
        }
        if (!any) {
            const uint32_t now = CTimer::GetTimeMS();
            if (g_renderFrames == 1 || now - g_lastRenderDiagnostic >= 10000) {
                g_lastRenderDiagnostic = now;
                BloodLog(ANDROID_LOG_DEBUG, "renderEffectsEvent active: stains=0 hookCalls=%u", g_hookCalls.load());
            }
            return;
        }

        CPlayerPed* ped = FindPlayerPed(0);
        if (!ped || renderedPed != ped || ped->m_fHealth <= 0.0f || !ped->GetMatrix()) {
            return;
        }
        RwTexture* tex = g_fallbackBloodTexture;
        if (!tex || !tex->raster) {
            const uint32_t now = CTimer::GetTimeMS();
            if (now - g_lastTextureWarning >= 5000) {
                g_lastTextureWarning = now;
                BloodLog(ANDROID_LOG_WARN,
                    "Render skipped: procedural blood texture unavailable (fallback=%p active=%d)",
                    static_cast<void*>(g_fallbackBloodTexture), activeCount);
            }
            return;
        }

        std::lock_guard<std::mutex> lock(g_stainMutex);
        // Matriks tulang sesudah CPed::PreRender = persis yang dipakai menggambar model frame ini.
        if (anyMesh) BuildSkinContexts(ped); else g_ctxCount = 0;
        BC::TorsoFrame frame;
        bool frameOk = false;
        if (anyTorso) frameOk = BuildFrame(ped, g_ctxCount > 0 ? &g_ctx[0] : nullptr, frame);

        const BC::V3 pedOrigin = PedOrigin(ped);
        const CVector camPosCV = TheCamera.GetPosition();
        const BC::V3 camPos = ToV3(camPosCV);
        const uint32_t now = CTimer::GetTimeMS();

        void* oldRaster = nullptr;
        void* oldZWrite = nullptr;
        void* oldZTest = nullptr;
        void* oldVertexAlpha = nullptr;
        void* oldSrcBlend = nullptr;
        void* oldDstBlend = nullptr;
        void* oldCull = nullptr;
        void* oldAddressU = nullptr;
        void* oldAddressV = nullptr;
        void* oldFilter = nullptr;
        RwRenderStateGet(rwRENDERSTATETEXTURERASTER, &oldRaster);
        RwRenderStateGet(rwRENDERSTATEZWRITEENABLE, &oldZWrite);
        RwRenderStateGet(rwRENDERSTATEZTESTENABLE, &oldZTest);
        RwRenderStateGet(rwRENDERSTATEVERTEXALPHAENABLE, &oldVertexAlpha);
        RwRenderStateGet(rwRENDERSTATESRCBLEND, &oldSrcBlend);
        RwRenderStateGet(rwRENDERSTATEDESTBLEND, &oldDstBlend);
        RwRenderStateGet(rwRENDERSTATECULLMODE, &oldCull);
        RwRenderStateGet(rwRENDERSTATETEXTUREADDRESSU, &oldAddressU);
        RwRenderStateGet(rwRENDERSTATETEXTUREADDRESSV, &oldAddressV);
        RwRenderStateGet(rwRENDERSTATETEXTUREFILTER, &oldFilter);

        RwRenderStateSet(rwRENDERSTATEZWRITEENABLE,      (void*)false);
        RwRenderStateSet(rwRENDERSTATEZTESTENABLE,       (void*)true);
        RwRenderStateSet(rwRENDERSTATEVERTEXALPHAENABLE, (void*)true);
        RwRenderStateSet(rwRENDERSTATESRCBLEND,          (void*)rwBLENDSRCALPHA);
        RwRenderStateSet(rwRENDERSTATEDESTBLEND,         (void*)rwBLENDINVSRCALPHA);
        RwRenderStateSet(rwRENDERSTATECULLMODE,           (void*)rwCULLMODECULLNONE);
        RwRenderStateSet(rwRENDERSTATETEXTUREADDRESSU,    (void*)rwTEXTUREADDRESSCLAMP);
        RwRenderStateSet(rwRENDERSTATETEXTUREADDRESSV,    (void*)rwTEXTUREADDRESSCLAMP);
        RwRenderStateSet(rwRENDERSTATETEXTUREFILTER,      (void*)rwFILTERLINEAR);
        RwRenderStateSet(rwRENDERSTATETEXTURERASTER,     (void*)tex->raster);

        int renderedBlots = 0;
        int inRange = 0;
        static BC::V3 meshPos[BC::kDecalMaxVerts], meshNrm[BC::kDecalMaxVerts];
        for (int i = 0; i < kMaxStains; ++i) {
            Stain& s = g_stains[i];
            if (!s.active) continue;

            const uint32_t age = now - s.born;
            if (age >= kLifeMs) { s.active = false; continue; }

            // animasi: meresap + menetes pelan sepanjang arah alir
            const float soak = clampfloat(0.0f, 1.0f, (float)age / (float)kSoakMs);
            const float ease = 1.0f - (1.0f - soak) * (1.0f - soak);
            const float scale = 0.55f + 0.45f * ease;
            const float drip = s.dripMax * clampfloat(0.0f, 1.0f, static_cast<float>(age) / 8000.0f);
            float life = 1.0f;
            if (age > kLifeMs - kFadeMs) life = (float)(kLifeMs - age) / (float)kFadeMs;
            const float dry = clampfloat(0.0f, 1.0f, (float)age / 60000.0f);
            const float dryTint = lerp(1.0f, 0.70f, dry);
            const unsigned char tintR = static_cast<unsigned char>(
                clampfloat(0.0f, 255.0f, 255.0f * s.colorR * dryTint));
            const unsigned char tintG = static_cast<unsigned char>(
                clampfloat(0.0f, 255.0f, 255.0f * s.colorG * dryTint));
            const unsigned char tintB = static_cast<unsigned char>(
                clampfloat(0.0f, 255.0f, 255.0f * s.colorB * dryTint));
            const unsigned char alphaMain = static_cast<unsigned char>(210.0f * life);
            const float blotR = s.radius * scale;

            if (s.mode == MODE_MESH) {
                const SkinCtx* c = FindCtx(s.mesh.geometryKey);
                if (!c) {
                    // atomic ada tapi geometry berbeda = ganti baju/model -> indeks vertex tak berlaku
                    if (g_ctxCount > 0) s.active = false;
                    continue;
                }
                const BC::V3 center = MeshStainCenter(s, *c);
                if (!IsFiniteV(center) || BC::Len(center - pedOrigin) > 2.5f) continue;
                const float dist = BC::Len(camPos - center);
                if (dist < 0.2f || dist > kMaxDrawDistance) continue;
                ++inRange;
                const float lift = kMeshLiftBase + dist * kMeshLiftPerMeter;
                if (!BC::EvalMeshDecal(s.mesh, c->view, lift, meshPos, meshNrm)) { s.active = false; continue; }

                const float gather = kMeshGatherRadius;
                const float cu = drip / gather;
                if (s.haloAlpha > 0) {
                    const unsigned char haloR = static_cast<unsigned char>(tintR * 0.78f);
                    const unsigned char haloG = static_cast<unsigned char>(tintG * 0.72f);
                    const unsigned char haloB = static_cast<unsigned char>(tintB * 0.76f);
                    const unsigned char haloA = static_cast<unsigned char>(s.haloAlpha * life);
                    if (DrawMeshBlot(s.mesh, meshPos, meshNrm, cu, 0.0f,
                                     gather / (blotR * s.haloScale),
                                     s.shapeX, s.shapeY, s.shapeAngle,
                                     haloR, haloG, haloB, haloA))
                        ++renderedBlots;
                }
                if (DrawMeshBlot(s.mesh, meshPos, meshNrm, cu, 0.0f, gather / blotR,
                                 s.shapeX, s.shapeY, s.shapeAngle,
                                 tintR, tintG, tintB, alphaMain))
                    ++renderedBlots;
            } else if (s.mode == MODE_TORSO) {
                if (!frameOk) continue;
                BC::V3 p, n;
                BC::TorsoSurface(frame, s.s, s.theta, kTorsoShrink, p, n);
                const float dist = BC::Len(camPos - p);
                if (dist < 0.2f || dist > kMaxDrawDistance) continue;
                ++inRange;
                if (s.haloAlpha > 0) {
                    const unsigned char haloR = static_cast<unsigned char>(tintR * 0.78f);
                    const unsigned char haloG = static_cast<unsigned char>(tintG * 0.72f);
                    const unsigned char haloB = static_cast<unsigned char>(tintB * 0.76f);
                    const unsigned char haloA = static_cast<unsigned char>(s.haloAlpha * life);
                    if (DrawTorsoBlot(frame, s.s, s.theta, blotR * s.haloScale, drip, 0.0f,
                                      s.shapeX, s.shapeY, s.shapeAngle,
                                      haloR, haloG, haloB, haloA))
                        ++renderedBlots;
                }
                if (DrawTorsoBlot(frame, s.s, s.theta, blotR, drip, 0.0f,
                                  s.shapeX, s.shapeY, s.shapeAngle,
                                  tintR, tintG, tintB, alphaMain))
                    ++renderedBlots;
            }
        }

        RwRenderStateSet(rwRENDERSTATETEXTURERASTER, oldRaster);
        RwRenderStateSet(rwRENDERSTATEZWRITEENABLE, oldZWrite);
        RwRenderStateSet(rwRENDERSTATEZTESTENABLE, oldZTest);
        RwRenderStateSet(rwRENDERSTATEVERTEXALPHAENABLE, oldVertexAlpha);
        RwRenderStateSet(rwRENDERSTATESRCBLEND, oldSrcBlend);
        RwRenderStateSet(rwRENDERSTATEDESTBLEND, oldDstBlend);
        RwRenderStateSet(rwRENDERSTATECULLMODE, oldCull);
        RwRenderStateSet(rwRENDERSTATETEXTUREADDRESSU, oldAddressU);
        RwRenderStateSet(rwRENDERSTATETEXTUREADDRESSV, oldAddressV);
        RwRenderStateSet(rwRENDERSTATETEXTUREFILTER, oldFilter);
        if (now - g_lastRenderDiagnostic >= 5000) {
            g_lastRenderDiagnostic = now;
            BloodLog(ANDROID_LOG_DEBUG,
                "Decal render: active=%d inRange=%d blotsSubmitted=%d tier1=%d skinCtx=%d texture=%p Im3D=%p",
                activeCount, inRange, renderedBlots, static_cast<int>(g_tier1), g_ctxCount,
                static_cast<void*>(tex), reinterpret_cast<void*>(g_im3dRenderIndexedPrimitive));
        }
    }

} // namespace

void Initialize() {
    if (g_hooked) return;

    OpenBloodLog();
    BloodLog(ANDROID_LOG_INFO, "Initializing for GTA SA Android 2.10 AML; symbol requested: _ZN4Fx_c8AddBloodER7CVectorS1_if");
    if (!aml) {
        BloodLog(ANDROID_LOG_ERROR, "Cannot install AddBlood hook: AML interface is null");
        return;
    }
    void* addBlood = GetMainLibrarySymbol<void*>("_ZN4Fx_c8AddBloodER7CVectorS1_if");
    BloodLog(ANDROID_LOG_INFO, "Fx_c::AddBlood address=%p", addBlood);
    if (addBlood && aml->Hook(addBlood, (void*)HookAddBlood, (void**)&g_origAddBlood)) {
        g_hooked = true;
        BloodLog(ANDROID_LOG_INFO, "Fx_c::AddBlood hook installed; original=%p", reinterpret_cast<void*>(g_origAddBlood));
        logger->Info("BloodStain: Fx_c::AddBlood hooked");
    } else {
        BloodLog(ANDROID_LOG_ERROR, "Fx_c::AddBlood symbol/hook failed (symbol=%p original=%p)",
                 addBlood, reinterpret_cast<void*>(g_origAddBlood));
        logger->Error("BloodStain: Fx_c::AddBlood symbol/hook failed, feature disabled");
        return;
    }

    // Opsional: dipakai sebagai cadangan posisi tulang untuk Tier 2.
    const char* boneSyms[] = {
        "_ZN4CPed15GetBonePositionER5RwV3djb",
    };
    for (const char* sym : boneSyms) {
        g_getBonePos = GetMainLibrarySymbol<GetBonePositionFn>(sym);
        if (g_getBonePos) break;
    }
    BloodLog(ANDROID_LOG_INFO, "CPed::GetBonePosition address=%p", reinterpret_cast<void*>(g_getBonePos));
    logger->Info("BloodStain: CPed::GetBonePosition %s", g_getBonePos ? "found" : "missing");

    g_im3dRenderIndexedPrimitive = GetMainLibrarySymbol<Im3DRenderIndexedPrimitiveFn>(
        "_Z28RwIm3DRenderIndexedPrimitive15RwPrimitiveTypePti");
    BloodLog(g_im3dRenderIndexedPrimitive ? ANDROID_LOG_INFO : ANDROID_LOG_ERROR,
        "RwIm3DRenderIndexedPrimitive %s at %p; body decal mesh %s",
        g_im3dRenderIndexedPrimitive ? "resolved" : "not found",
        reinterpret_cast<void*>(g_im3dRenderIndexedPrimitive),
        g_im3dRenderIndexedPrimitive ? "enabled" : "disabled");

    ResolveSkinApi();
    BloodLog(ANDROID_LOG_INFO,
        "Tier2 fallback profile: bodyRadius=%.0f%% normalLift=%.0fmm cameraTranslation=0mm",
        kTorsoShrink * 100.0f, kTier2Lift * 1000.0f);

    Events::pedRenderEvent.after += [](CPed* renderedPed) { RenderStains(renderedPed); };
    BloodLog(ANDROID_LOG_INFO,
        "CPed::Render after-callback registered; stains render after the live player model with its depth buffer");
}

void Process(CPlayerPed* player) {
    if (!g_hooked || !player) return;

    // Create RenderWare resources from game-process rather than while
    // CPed::Render is in progress.
    if (!g_fallbackBloodTexture) {
        g_fallbackBloodTexture = CreateFallbackBloodTexture();
    }

    const uint32_t now = CTimer::GetTimeMS();
    const bool alive = player->m_fHealth > 0.0f;
    const float healthArmour = player->m_fHealth + player->m_fArmour;

    if (alive && !g_wasAlive) ClearStains();  // respawn setelah wasted/busted
    g_wasAlive = alive;

    bool damaged = false;
    float damageAmount = 0.0f;
    const float previousHealthArmour = g_prevHealthArmour;
    const uint32_t previousLastDamaged = g_prevLastDamaged;
    if (g_stateValid) {
        damageAmount = fmaxf(0.0f, g_prevHealthArmour - healthArmour);
        damaged = (healthArmour < g_prevHealthArmour - 0.01f) ||
                  (player->m_nLastDamagedTime != g_prevLastDamaged);
        if (damaged && damageAmount < 1.0f) damageAmount = 1.0f;
    }
    g_prevHealthArmour = healthArmour;
    g_prevLastDamaged  = player->m_nLastDamagedTime;
    g_stateValid = true;

    // ambil catatan AddBlood
    Pending local[16];
    int count = 0;
    int pendingBeforeDrain = 0;
    {
        std::lock_guard<std::mutex> lock(g_mutex);
        pendingBeforeDrain = g_pendingCount;
        for (int i = 0; i < g_pendingCount; ++i) {
            if (now - g_pending[i].time <= kPendingMaxAgeMs) {
                local[count++] = g_pending[i];
            }
        }
        // yang belum "damaged" disimpan sebentar (urutan event bisa beda 1 frame)
        g_pendingCount = 0;
        if (!damaged) {
            for (int i = 0; i < count; ++i) g_pending[g_pendingCount++] = local[i];
            count = 0;
        }
    }
    if (damaged) {
        BloodLog(ANDROID_LOG_INFO,
            "Player damage detected: health=%.1f armour=%.1f total=%.1f previousTotal=%.1f lastDamaged=%u previousLastDamaged=%u weapon=%d pending=%d acceptedPending=%d",
            player->m_fHealth, player->m_fArmour, healthArmour, previousHealthArmour,
            player->m_nLastDamagedTime, previousLastDamaged,
            static_cast<int>(player->m_nLastWeaponDamage), pendingBeforeDrain, count);
    }
    if (count == 0) return;

    if (kBulletsOnly && !IsBulletWeapon((int)player->m_nLastWeaponDamage)) {
        BloodLog(ANDROID_LOG_DEBUG, "Discarding %d blood events: last damage weapon %d is not in bullet whitelist",
                 count, static_cast<int>(player->m_nLastWeaponDamage));
        return;
    }

    int created = 0;
    {
        std::lock_guard<std::mutex> lock(g_stainMutex);
        // Fx_c::AddBlood may be called repeatedly for one weapon impact (blood
        // particles/splashes). Treat the collected burst as one wound, not one
        // circular decal per callback. The newest sample is closest to the
        // damage event that Process just observed.
        if (count > 0 && kMaxNewPerDamage > 0 &&
            CreateStain(player, local[count - 1], now, damageAmount))
            created = 1;
    }
    BloodLog(ANDROID_LOG_INFO,
        "Damage event processed: AddBloodCandidates=%d selected=1 createdOrMerged=%d coalesced=%d",
        count, created, count > 0 ? count - 1 : 0);
}

} // namespace BloodStainSystem
