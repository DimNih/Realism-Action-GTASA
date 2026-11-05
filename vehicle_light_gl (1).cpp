#include "vehicle_light_gl.h"

#include <mod/amlmod.h>

#include <EGL/egl.h>
#include <GLES3/gl3.h>

#include <algorithm>
#include <atomic>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <string>

namespace VehicleLightGL {
namespace {

static_assert(sizeof(Light) == kVec4PerLight * 4 * sizeof(float), "Light must be exactly kVec4PerLight vec4 values");

// ---------------------------------------------------------------------------------------------
// Shared state
// ---------------------------------------------------------------------------------------------
struct Snapshot {
    Light lights[kMaxLights];
    int count = 0;
    float camera[3] = {};
    int stampFrame = -1000;
};

constexpr int kMinSceneDraws = 40;   // perspective draws before the world is considered rendered
constexpr int kMaxProg = 2048;
constexpr int kMaxFbos = 8;
// Vehicle render callbacks and the EGL swap hook are not guaranteed to land
// on the same frame boundary. Keep the last valid snapshot briefly so a
// scheduling jitter does not make the light blink out.
constexpr int kSnapshotMaxAge = 6;   // GL frames

LogFn g_log = nullptr;
Settings g_settings;
std::atomic<bool> g_enabled{false};
std::atomic<int> g_frames{0};
std::atomic<int> g_lastLitFrame{-1000};
Snapshot g_snap[2];
std::atomic<int> g_snapIndex{0};

bool g_inPass = false, g_gpuFailed = false, g_gpuReady = false;
GLuint g_program = 0, g_vao = 0, g_fbo = 0, g_depthTex = 0;
int g_depthW = 0, g_depthH = 0, g_depthAllocatedFmt = -1, g_fmtIndex = -1;
GLint g_uDepth = -1, g_uInvVP = -1, g_uVP = -1, g_uInvRes = -1, g_uOrigin = -1, g_uCam = -1;
GLint g_uCount = -1, g_uMode = -1, g_uShadow = -1, g_uGain = -1, g_uVolGain = -1, g_uBloomGain = -1, g_uMaxDist = -1, g_uL = -1;
GLint g_uWhiteGain = -1, g_uBounceGain = -1;

GLuint g_query[2] = {0, 0};
bool g_queryPending[2] = {false, false};
bool g_queryOk = true;

// Draw-call tracking (3D -> 2D transition detection)
enum : uint8_t { kClsUnknown = 0, kClsPerspective = 1, kClsOrtho = 2 };
struct ProgInfo { GLint view = -1, proj = -1; uint8_t cls = kClsUnknown; bool scanned = false; int checkedFrame = -1000; };
ProgInfo g_prog[kMaxProg];

struct FboStat { GLuint fbo = 0; int draws = 0; GLuint prog = 0; GLint vp[4] = {}; };
FboStat g_fbos[kMaxFbos];
int g_fboCount = 0;
bool g_injectedThisFrame = false;
GLuint g_curProgram = 0, g_curFbo = 0;
bool g_hookUseProgram = false, g_hookBindFbo = false;

// Counters / diagnostics
unsigned long long g_passCount = 0, g_passMicros = 0, g_injectCount = 0, g_fallbackCount = 0, g_litCount = 0, g_unlitCount = 0;
int g_matrixMode = -1;
int g_lastLightCount = 0;
bool g_firstPassLogged = false, g_framebufferLogged = false, g_nonEmptySnapshotLogged = false, g_swapContextLogged = false;
bool g_matrixInvalidLogged = false;
int g_debugLogBudget = 6;
long long g_lastInstallAttempt = -3000000, g_lastSettingsLoad = -10000000, g_lastDepthFailureLog = 0;
unsigned g_depthFailCount = 0;

EGLBoolean (*o_swap)(EGLDisplay, EGLSurface) = nullptr;
void (*o_drawElements)(GLenum, GLsizei, GLenum, const void*) = nullptr;
void (*o_drawArrays)(GLenum, GLint, GLsizei) = nullptr;
void (*o_useProgram)(GLuint) = nullptr;
void (*o_bindFramebuffer)(GLenum, GLuint) = nullptr;
bool g_hookSwap = false, g_hookDrawElements = false, g_hookDrawArrays = false;

struct DepthFmt { GLint internal; GLenum format; GLenum type; GLenum attach; const char* name; };
const DepthFmt kFmts[] = {
    {GL_DEPTH24_STENCIL8, GL_DEPTH_STENCIL, GL_UNSIGNED_INT_24_8, GL_DEPTH_STENCIL_ATTACHMENT, "D24S8"},
    {GL_DEPTH_COMPONENT24, GL_DEPTH_COMPONENT, GL_UNSIGNED_INT, GL_DEPTH_ATTACHMENT, "D24"},
    {GL_DEPTH_COMPONENT16, GL_DEPTH_COMPONENT, GL_UNSIGNED_SHORT, GL_DEPTH_ATTACHMENT, "D16"},
    {GL_DEPTH_COMPONENT32F, GL_DEPTH_COMPONENT, GL_FLOAT, GL_DEPTH_ATTACHMENT, "D32F"},
};
constexpr int kFmtCount = sizeof(kFmts) / sizeof(kFmts[0]);

long long NowMicros() {
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<long long>(ts.tv_sec) * 1000000LL + ts.tv_nsec / 1000;
}
void DrainGlErrors() { int guard = 0; while (glGetError() != GL_NO_ERROR && ++guard < 16) {} }

// ---------------------------------------------------------------------------------------------
// Settings file
// ---------------------------------------------------------------------------------------------
void LoadSettings() {
    const char* base = aml ? aml->GetAndroidDataPath() : nullptr;
    if (!base || !base[0]) return;
    char path[600];
    std::snprintf(path, sizeof(path), "%s/VehicleLight.ini", base);
    FILE* file = std::fopen(path, "r");
    if (!file) return;
    Settings next;
    char line[160];
    while (std::fgets(line, sizeof(line), file)) {
        char key[48]{};
        float value = 0.0f;
        if (std::sscanf(line, " %47[A-Za-z_] = %f", key, &value) != 2) continue;
        std::string k(key);
        for (char& ch : k) ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
        if (k == "intensity") next.intensity = std::max(0.0f, std::min(1.25f, value));
        else if (k == "volumetric") next.volumetric = std::max(0.0f, std::min(0.08f, value));
        else if (k == "range") next.range = std::max(10.0f, std::min(80.0f, value));
        else if (k == "albedo") next.albedo = value > 0.5f ? 1 : 0;
        else if (k == "shadow") next.shadow = value > 0.5f ? 1 : 0;
        else if (k == "rear") next.rear = value > 0.5f ? 1 : 0;
        else if (k == "bloom") next.bloom = std::max(0.0f, std::min(0.80f, value));
        else if (k == "white") next.white = std::max(0.0f, std::min(1.50f, value));
        else if (k == "bounce") next.bounce = std::max(0.0f, std::min(2.0f, value));
        else if (k == "debug") next.debug = static_cast<int>(value);
    }
    std::fclose(file);
    g_settings = next;
}

// ---------------------------------------------------------------------------------------------
// GL helpers
// ---------------------------------------------------------------------------------------------
GLuint Compile(GLenum type, const char* source) {
    const GLuint shader = glCreateShader(type);
    if (!shader) return 0;
    glShaderSource(shader, 1, &source, nullptr);
    glCompileShader(shader);
    GLint ok = GL_FALSE;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &ok);
    if (ok == GL_TRUE) return shader;
    char log[1024]{};
    glGetShaderInfoLog(shader, sizeof(log) - 1, nullptr, log);
    if (g_log) g_log("[GL] %s shader compile failed: %s", type == GL_VERTEX_SHADER ? "vertex" : "fragment", log);
    glDeleteShader(shader);
    return 0;
}

bool InitGpu() {
    if (g_gpuReady) return true;
    if (g_gpuFailed) return false;
    const GLuint vs = Compile(GL_VERTEX_SHADER, kVertexShader);
    const GLuint fs = vs ? Compile(GL_FRAGMENT_SHADER, kFragmentShader) : 0;
    if (!vs || !fs) {
        if (vs) glDeleteShader(vs);
        if (fs) glDeleteShader(fs);
        g_gpuFailed = true;
        return false;
    }
    g_program = glCreateProgram();
    glAttachShader(g_program, vs);
    glAttachShader(g_program, fs);
    glLinkProgram(g_program);
    glDeleteShader(vs);
    glDeleteShader(fs);
    GLint linked = GL_FALSE;
    glGetProgramiv(g_program, GL_LINK_STATUS, &linked);
    if (linked != GL_TRUE) {
        char log[1024]{};
        glGetProgramInfoLog(g_program, sizeof(log) - 1, nullptr, log);
        if (g_log) g_log("[GL] shader link failed: %s", log);
        glDeleteProgram(g_program);
        g_program = 0;
        g_gpuFailed = true;
        return false;
    }
    g_uDepth = glGetUniformLocation(g_program, "uDepth");
    g_uInvVP = glGetUniformLocation(g_program, "uInvVP");
    g_uVP = glGetUniformLocation(g_program, "uVP");
    g_uInvRes = glGetUniformLocation(g_program, "uInvRes");
    g_uOrigin = glGetUniformLocation(g_program, "uOrigin");
    g_uCam = glGetUniformLocation(g_program, "uCam");
    g_uCount = glGetUniformLocation(g_program, "uCount");
    g_uMode = glGetUniformLocation(g_program, "uMode");
    g_uShadow = glGetUniformLocation(g_program, "uShadow");
    g_uGain = glGetUniformLocation(g_program, "uGain");
    g_uVolGain = glGetUniformLocation(g_program, "uVolGain");
    g_uBloomGain = glGetUniformLocation(g_program, "uBloomGain");
    g_uMaxDist = glGetUniformLocation(g_program, "uMaxDist");
    g_uWhiteGain = glGetUniformLocation(g_program, "uWhiteGain");
    g_uBounceGain = glGetUniformLocation(g_program, "uBounceGain");
    g_uL = glGetUniformLocation(g_program, "uL[0]");

    GLint oldVao = 0;
    glGetIntegerv(GL_VERTEX_ARRAY_BINDING, &oldVao);
    glGenVertexArrays(1, &g_vao);
    glBindVertexArray(static_cast<GLuint>(oldVao));
    glGenFramebuffers(1, &g_fbo);
    glGenQueries(2, g_query);
    g_queryOk = g_query[0] != 0 && g_query[1] != 0;
    if (!g_vao || !g_fbo || g_uDepth < 0 || g_uInvVP < 0 || g_uVP < 0 || g_uL < 0 || g_uMode < 0) {
        if (g_log) g_log("[GL] GLES resource/uniform initialization failed (vao=%u fbo=%u depth=%d invvp=%d vp=%d L=%d mode=%d)",
                         g_vao, g_fbo, g_uDepth, g_uInvVP, g_uVP, g_uL, g_uMode);
        g_gpuFailed = true;
        return false;
    }
    g_gpuReady = true;
    if (g_log) g_log("[GL] GLES3 depth-light shader ready (program=%u queries=%d)", g_program, g_queryOk ? 1 : 0);
    return true;
}

// ---------------------------------------------------------------------------------------------
// Matrix math (column-major, GL convention)
// ---------------------------------------------------------------------------------------------
void Mat4Mul(const float* a, const float* b, float* out) {
    for (int col = 0; col < 4; ++col)
        for (int row = 0; row < 4; ++row)
            out[col * 4 + row] = a[row] * b[col * 4] + a[4 + row] * b[col * 4 + 1] +
                                 a[8 + row] * b[col * 4 + 2] + a[12 + row] * b[col * 4 + 3];
}
void Mat4MulVec4(const float* m, const float* v, float* out) {
    for (int row = 0; row < 4; ++row)
        out[row] = m[row] * v[0] + m[4 + row] * v[1] + m[8 + row] * v[2] + m[12 + row] * v[3];
}
void Mat4Transpose(const float* m, float* out) {
    for (int r = 0; r < 4; ++r) for (int c = 0; c < 4; ++c) out[c * 4 + r] = m[r * 4 + c];
}
bool Mat4Inverse(const float* m, float* inverse) {
    double a[4][8]{};
    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) a[row][col] = m[col * 4 + row];
        a[row][row + 4] = 1.0;
    }
    for (int col = 0; col < 4; ++col) {
        int pivot = col;
        for (int row = col + 1; row < 4; ++row)
            if (std::fabs(a[row][col]) > std::fabs(a[pivot][col])) pivot = row;
        if (std::fabs(a[pivot][col]) < 1.0e-12) return false;
        if (pivot != col) for (int k = 0; k < 8; ++k) std::swap(a[pivot][k], a[col][k]);
        const double divisor = a[col][col];
        for (double& value : a[col]) value /= divisor;
        for (int row = 0; row < 4; ++row) {
            if (row == col) continue;
            const double factor = a[row][col];
            for (int k = 0; k < 8; ++k) a[row][k] -= factor * a[col][k];
        }
    }
    for (int row = 0; row < 4; ++row)
        for (int col = 0; col < 4; ++col) inverse[col * 4 + row] = static_cast<float>(a[row][col + 4]);
    return true;
}

bool IsPerspectiveProjection(const float* p) {
    const float xScale = std::fabs(p[0]);
    const float yScale = std::fabs(p[5]);
    const bool perspectiveW = std::fabs(p[11]) > 0.1f && std::fabs(p[15]) < 0.1f;
    return xScale > 0.05f && xScale < 10.0f && yScale > 0.05f && yScale < 10.0f && perspectiveW;
}

// The game's view matrix is a RenderWare camera matrix uploaded as-is. The previous code derived the
// camera position with a transposed rotation, which is only right for axis-aligned cameras. Here the
// position comes from the full inverse and is cross-checked against the game's camera; the transposed
// layout is tried as well so a different upload convention cannot silently break the pass.
struct CameraSolve { float viewRel[16]; float cam[3]; int mode; float error; bool trusted; };
CameraSolve SolveCamera(const float* view, const float* gameCam) {
    CameraSolve best{};
    best.error = 1.0e30f;
    float transposed[16];
    Mat4Transpose(view, transposed);
    const float* candidates[2] = {view, transposed};
    for (int mode = 0; mode < 2; ++mode) {
        float inverse[16];
        if (!Mat4Inverse(candidates[mode], inverse)) continue;
        const float cx = inverse[12], cy = inverse[13], cz = inverse[14];
        const float error = gameCam ? std::sqrt((cx - gameCam[0]) * (cx - gameCam[0]) + (cy - gameCam[1]) * (cy - gameCam[1]) +
                                                (cz - gameCam[2]) * (cz - gameCam[2])) : 0.0f;
        if (error < best.error) {
            best.error = error; best.mode = mode;
            best.cam[0] = cx; best.cam[1] = cy; best.cam[2] = cz;
            std::memcpy(best.viewRel, candidates[mode], sizeof(best.viewRel));
        }
    }
    best.trusted = !gameCam || best.error < 4.0f;
    if (!best.trusted && gameCam) { best.cam[0] = gameCam[0]; best.cam[1] = gameCam[1]; best.cam[2] = gameCam[2]; }
    best.viewRel[12] = best.viewRel[13] = best.viewRel[14] = 0.0f;
    return best;
}

bool ComputeScissor(const Light* lights, int count, const float* vpMatrix, const GLint viewport[4], GLint out[4]) {
    // Beam volume (forward cone incl. the long upper lobe) plus the close-range spill sphere
    // around each lamp, so facades/upper floors and wall reflections are never clipped.
    static const float beamPts[12][3] = {
        {0,0,0}, {1,0,0}, {1,-1,0}, {1,1,0}, {1,0,1}, {1,-1,1}, {1,1,1},
        {0.3f,-1,0}, {0.3f,1,0}, {0.6f,0,1}, {0.6f,-1,1}, {0.6f,1,1}};
    static const float spill[8][3] = {
        {-1,-1,-1}, {1,-1,-1}, {-1,1,-1}, {1,1,-1}, {-1,-1,1}, {1,-1,1}, {-1,1,1}, {1,1,1}};
    float minX = 1e9f, minY = 1e9f, maxX = -1e9f, maxY = -1e9f;
    auto accumulate = [&](const float* p3) -> bool {
        float p[4] = {p3[0], p3[1], p3[2], 1.0f};
        float clip[4]; Mat4MulVec4(vpMatrix, p, clip);
        if (clip[3] <= 0.05f) return false;
        const float x = (clip[0] / clip[3] * 0.5f + 0.5f) * viewport[2];
        const float y = (clip[1] / clip[3] * 0.5f + 0.5f) * viewport[3];
        minX = std::min(minX, x); maxX = std::max(maxX, x);
        minY = std::min(minY, y); maxY = std::max(maxY, y);
        return true;
    };
    for (int i = 0; i < count; ++i) {
        const Light& light = lights[i];
        const float range = light.posRange[3];
        const bool spot = light.fwdKind[3] > 0.5f;
        if (spot) {
            for (const auto& q : beamPts) {
                float p[3];
                for (int axis = 0; axis < 3; ++axis)
                    p[axis] = light.posRange[axis] + light.fwdKind[axis] * range * q[0] +
                        light.rightYaw[axis] * range * 0.85f * q[1] * q[0] + light.upGain[axis] * 14.0f * q[2];
                if (!accumulate(p)) return false;
            }
            for (const auto& q : spill) {
                const float p[3] = {light.posRange[0] + q[0] * 9.5f, light.posRange[1] + q[1] * 9.5f,
                                    light.posRange[2] + (q[2] > 0 ? 14.0f : -1.5f)};
                if (!accumulate(p)) return false;
            }
        } else {
            for (const auto& q : spill) {
                const float p[3] = {light.posRange[0] + q[0] * range, light.posRange[1] + q[1] * range,
                                    light.posRange[2] + q[2] * range};
                if (!accumulate(p)) return false;
            }
        }
    }
    const float marginX = 0.14f * viewport[2], marginY = 0.14f * viewport[3];   // room for the glare around sources
    const int x0 = std::max(0, static_cast<int>(minX - marginX));
    const int y0 = std::max(0, static_cast<int>(minY - marginY));
    const int x1 = std::min(viewport[2], static_cast<int>(maxX + marginX));
    const int y1 = std::min(viewport[3], static_cast<int>(maxY + marginY));
    if (x1 <= x0 || y1 <= y0) return false;
    out[0] = viewport[0] + x0; out[1] = viewport[1] + y0;
    out[2] = x1 - x0; out[3] = y1 - y0;
    return true;
}

// ---------------------------------------------------------------------------------------------
// Program classification (which draw calls are 3D scene vs 2D HUD)
// ---------------------------------------------------------------------------------------------
void ScanProgram(GLuint program, ProgInfo& info) {
    info = ProgInfo{};
    info.scanned = true;
    GLint count = 0;
    glGetProgramiv(program, GL_ACTIVE_UNIFORMS, &count);
    for (GLint i = 0; i < count; ++i) {
        char name[96]{};
        GLsizei length = 0; GLint size = 0; GLenum type = 0;
        glGetActiveUniform(program, static_cast<GLuint>(i), sizeof(name) - 1, &length, &size, &type, name);
        if (type != GL_FLOAT_MAT4) continue;
        std::string lower(name);
        for (char& ch : lower) ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
        const GLint location = glGetUniformLocation(program, name);
        if (lower.find("proj") != std::string::npos) info.proj = location;
        else if (lower.find("view") != std::string::npos) info.view = location;
    }
}

void ClassifyProgram(GLuint program, ProgInfo& info, int frame) {
    info.checkedFrame = frame;
    float projection[16]{};
    glGetUniformfv(program, info.proj, projection);
    info.cls = IsPerspectiveProjection(projection) ? kClsPerspective : kClsOrtho;
}

FboStat* FindFbo(GLuint fbo, bool create) {
    for (int i = 0; i < g_fboCount; ++i) if (g_fbos[i].fbo == fbo) return &g_fbos[i];
    if (!create || g_fboCount >= kMaxFbos) return nullptr;
    FboStat& stat = g_fbos[g_fboCount++];
    stat = FboStat{};
    stat.fbo = fbo;
    return &stat;
}
FboStat* MainFbo() {
    FboStat* best = nullptr;
    for (int i = 0; i < g_fboCount; ++i) if (!best || g_fbos[i].draws > best->draws) best = &g_fbos[i];
    return best;
}

// ---------------------------------------------------------------------------------------------
// Depth copy
// ---------------------------------------------------------------------------------------------
bool CaptureDepth(GLuint sourceFbo, const GLint vp[4]) {
    if (vp[2] <= 0 || vp[3] <= 0) return false;
    GLint depthBits = 0, stencilBits = 0, sourceSamples = 0;
    glBindFramebuffer(GL_FRAMEBUFFER, sourceFbo);   // both bindings -> bit queries are unambiguous
    glGetIntegerv(GL_DEPTH_BITS, &depthBits);
    glGetIntegerv(GL_STENCIL_BITS, &stencilBits);
    glGetIntegerv(GL_SAMPLES, &sourceSamples);
    if (g_log && !g_framebufferLogged) {
        g_framebufferLogged = true;
        g_log("[GL] scene framebuffer=%u viewport=%d,%d %dx%d depthBits=%d stencilBits=%d samples=%d GL=%s",
              sourceFbo, vp[0], vp[1], vp[2], vp[3], depthBits, stencilBits, sourceSamples,
              reinterpret_cast<const char*>(glGetString(GL_VERSION)));
    }
    const int preferred = stencilBits > 0 ? 0 : (depthBits >= 32 ? 3 : (depthBits >= 24 ? 1 : 2));
    int order[kFmtCount], orderCount = 0;
    order[orderCount++] = preferred;
    for (int i = 0; i < kFmtCount; ++i) if (i != preferred) order[orderCount++] = i;

    GLint tex0 = 0, activeTex = GL_TEXTURE0;
    glGetIntegerv(GL_ACTIVE_TEXTURE, &activeTex);
    glActiveTexture(GL_TEXTURE0);
    glGetIntegerv(GL_TEXTURE_BINDING_2D, &tex0);
    bool ok = false;
    for (int attempt = 0; attempt < orderCount && !ok; ++attempt) {
        const int fmtIndex = order[attempt];
        const DepthFmt& fmt = kFmts[fmtIndex];
        DrainGlErrors();
        if (!g_depthTex) glGenTextures(1, &g_depthTex);
        glBindTexture(GL_TEXTURE_2D, g_depthTex);
        if (g_depthW != vp[2] || g_depthH != vp[3] || g_depthAllocatedFmt != fmtIndex) {
            glTexImage2D(GL_TEXTURE_2D, 0, fmt.internal, vp[2], vp[3], 0, fmt.format, fmt.type, nullptr);
            g_depthW = vp[2]; g_depthH = vp[3]; g_depthAllocatedFmt = fmtIndex;
        }
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_NONE);

        glBindFramebuffer(GL_DRAW_FRAMEBUFFER, g_fbo);
        glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, 0, 0);
        glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_TEXTURE_2D, 0, 0);
        glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, fmt.attach, GL_TEXTURE_2D, g_depthTex, 0);
        const GLenum none = GL_NONE;
        glDrawBuffers(1, &none);
        const GLenum status = glCheckFramebufferStatus(GL_DRAW_FRAMEBUFFER);
        bool blitted = false;
        GLenum blitError = GL_NO_ERROR;
        if (status == GL_FRAMEBUFFER_COMPLETE) {
            glBindFramebuffer(GL_READ_FRAMEBUFFER, sourceFbo);
            glDisable(GL_SCISSOR_TEST);   // caller restores its own scissor state afterwards
            glBlitFramebuffer(vp[0], vp[1], vp[0] + vp[2], vp[1] + vp[3], 0, 0, vp[2], vp[3], GL_DEPTH_BUFFER_BIT, GL_NEAREST);
            blitError = glGetError();
            blitted = blitError == GL_NO_ERROR;
        }
        if (blitted) {
            ok = true;
            g_fmtIndex = fmtIndex;
            g_depthFailCount = 0;
            if (g_log && !g_firstPassLogged)
                g_log("[GL] depth capture OK (%s, srcFbo=%u depth=%d stencil=%d)", fmt.name, sourceFbo, depthBits, stencilBits);
            break;
        }
        ++g_depthFailCount;
        const long long now = NowMicros();
        if (g_log && (g_depthFailCount <= 6 || now - g_lastDepthFailureLog >= 2000000)) {
            g_lastDepthFailureLog = now;
            g_log("[GL] depth blit rejected (%s): fbo=0x%x glError=0x%x srcFbo=%u viewport=%dx%d samples=%d depth=%d stencil=%d",
                  fmt.name, static_cast<unsigned>(status), static_cast<unsigned>(blitError), sourceFbo, vp[2], vp[3],
                  sourceSamples, depthBits, stencilBits);
        }
        g_depthW = 0; g_depthAllocatedFmt = -1;
    }
    glBindTexture(GL_TEXTURE_2D, static_cast<GLuint>(tex0));
    glActiveTexture(static_cast<GLenum>(activeTex));
    DrainGlErrors();
    return ok;
}

// ---------------------------------------------------------------------------------------------
// The light pass itself
// ---------------------------------------------------------------------------------------------
void PollQueries(int frame) {
    if (!g_queryOk) return;
    for (int i = 0; i < 2; ++i) {
        if (!g_queryPending[i]) continue;
        GLuint available = 0;
        glGetQueryObjectuiv(g_query[i], GL_QUERY_RESULT_AVAILABLE, &available);
        if (!available) continue;
        GLuint result = 0;
        glGetQueryObjectuiv(g_query[i], GL_QUERY_RESULT, &result);
        g_queryPending[i] = false;
        if (result) { g_lastLitFrame.store(frame, std::memory_order_relaxed); ++g_litCount; }
        else ++g_unlitCount;
    }
}

const Snapshot& CurrentSnapshot() { return g_snap[g_snapIndex.load(std::memory_order_acquire)]; }
bool SnapshotUsable(const Snapshot& snapshot) {
    return snapshot.count > 0 && g_frames.load(std::memory_order_relaxed) - snapshot.stampFrame <= kSnapshotMaxAge;
}

// Returns true if the pass was drawn.
bool RunLightPass(const FboStat& scene, const char* origin) {
    const Snapshot& snapshot = CurrentSnapshot();
    if (!SnapshotUsable(snapshot) || g_gpuFailed) return false;
    const ProgInfo& pi = g_prog[scene.prog];
    if (!scene.prog || pi.view < 0 || pi.proj < 0) return false;

    g_inPass = true;
    const long long start = NowMicros();
    const int frame = g_frames.load(std::memory_order_relaxed);
    DrainGlErrors();

    // ---- save every piece of state this pass touches
    GLint program = 0, vao = 0, activeTex = GL_TEXTURE0, texture0 = 0, sampler0 = 0;
    GLint blendSrcRgb = 0, blendDstRgb = 0, blendSrcA = 0, blendDstA = 0, blendEqRgb = GL_FUNC_ADD, blendEqA = GL_FUNC_ADD;
    GLint depthFunc = 0, scissorBox[4]{}, savedViewport[4]{}, drawFb = 0, readFb = 0;
    GLboolean depthMask = GL_TRUE, colorMask[4] = {GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE};
    glGetIntegerv(GL_CURRENT_PROGRAM, &program);
    glGetIntegerv(GL_VERTEX_ARRAY_BINDING, &vao);
    glGetIntegerv(GL_ACTIVE_TEXTURE, &activeTex);
    glGetIntegerv(GL_BLEND_SRC_RGB, &blendSrcRgb); glGetIntegerv(GL_BLEND_DST_RGB, &blendDstRgb);
    glGetIntegerv(GL_BLEND_SRC_ALPHA, &blendSrcA); glGetIntegerv(GL_BLEND_DST_ALPHA, &blendDstA);
    glGetIntegerv(GL_BLEND_EQUATION_RGB, &blendEqRgb); glGetIntegerv(GL_BLEND_EQUATION_ALPHA, &blendEqA);
    glGetIntegerv(GL_DEPTH_FUNC, &depthFunc); glGetIntegerv(GL_SCISSOR_BOX, scissorBox);
    glGetIntegerv(GL_VIEWPORT, savedViewport);
    glGetIntegerv(GL_DRAW_FRAMEBUFFER_BINDING, &drawFb);
    glGetIntegerv(GL_READ_FRAMEBUFFER_BINDING, &readFb);
    glGetBooleanv(GL_DEPTH_WRITEMASK, &depthMask); glGetBooleanv(GL_COLOR_WRITEMASK, colorMask);
    const GLboolean blend = glIsEnabled(GL_BLEND), depthTest = glIsEnabled(GL_DEPTH_TEST);
    const GLboolean cull = glIsEnabled(GL_CULL_FACE), scissor = glIsEnabled(GL_SCISSOR_TEST);
    const GLboolean stencil = glIsEnabled(GL_STENCIL_TEST), polygon = glIsEnabled(GL_POLYGON_OFFSET_FILL);
    const GLboolean dither = glIsEnabled(GL_DITHER), alphaToCoverage = glIsEnabled(GL_SAMPLE_ALPHA_TO_COVERAGE);
    const GLboolean rasterDiscard = glIsEnabled(GL_RASTERIZER_DISCARD);
    glActiveTexture(GL_TEXTURE0);
    glGetIntegerv(GL_TEXTURE_BINDING_2D, &texture0);
    glGetIntegerv(GL_SAMPLER_BINDING, &sampler0);

    bool drew = false;
    do {
        if (!InitGpu()) break;
        PollQueries(frame);

        // ---- fresh camera matrices from the last 3D program that drew into the scene framebuffer
        float view[16]{}, projection[16]{};
        glGetUniformfv(scene.prog, pi.view, view);
        glGetUniformfv(scene.prog, pi.proj, projection);
        if (!IsPerspectiveProjection(projection)) {
            if (!g_matrixInvalidLogged && g_log) { g_matrixInvalidLogged = true; g_log("[GL] pass aborted: projection of program %u is not perspective", scene.prog); }
            break;
        }
        const CameraSolve solve = SolveCamera(view, snapshot.camera);
        if (solve.mode != g_matrixMode || (!solve.trusted && g_debugLogBudget > 0)) {
            if (g_log) g_log("[GL] camera solve: mode=%d (%s) viewCam=(%.1f,%.1f,%.1f) gameCam=(%.1f,%.1f,%.1f) error=%.2fm %s",
                             solve.mode, solve.mode ? "transposed" : "as-is", solve.cam[0], solve.cam[1], solve.cam[2],
                             snapshot.camera[0], snapshot.camera[1], snapshot.camera[2], solve.error,
                             solve.trusted ? "OK" : "MISMATCH - using game camera position");
            g_matrixMode = solve.mode;
            if (!solve.trusted) --g_debugLogBudget;
        }
        float vpMatrix[16], inverseVp[16];
        Mat4Mul(projection, solve.viewRel, vpMatrix);
        if (!Mat4Inverse(vpMatrix, inverseVp)) break;

        Light relative[kMaxLights];
        const int count = std::min(snapshot.count, kMaxLights);
        for (int i = 0; i < count; ++i) {
            relative[i] = snapshot.lights[i];
            for (int axis = 0; axis < 3; ++axis) {
                relative[i].posRange[axis] -= solve.cam[axis];
                relative[i].boxC[axis] -= solve.cam[axis];
            }
        }
        GLint viewport[4] = {scene.vp[0], scene.vp[1], scene.vp[2], scene.vp[3]};
        if (viewport[2] <= 0 || viewport[3] <= 0) break;

        if (!CaptureDepth(static_cast<GLuint>(scene.fbo), viewport)) break;
        if (g_log && !g_firstPassLogged) {
            g_log("[GL] proj: [0]=%.4f [5]=%.4f [10]=%.4f [11]=%.4f [14]=%.4f [15]=%.4f", projection[0], projection[5], projection[10],
                  projection[11], projection[14], projection[15]);
        }

        GLint scissorRect[4]{};
        const bool useScissor = ComputeScissor(relative, count, vpMatrix, viewport, scissorRect);

        glBindFramebuffer(GL_FRAMEBUFFER, static_cast<GLuint>(scene.fbo));
        glViewport(viewport[0], viewport[1], viewport[2], viewport[3]);
        glUseProgram(g_program);
        glUniform1i(g_uDepth, 0);
        glUniformMatrix4fv(g_uInvVP, 1, GL_FALSE, inverseVp);
        glUniformMatrix4fv(g_uVP, 1, GL_FALSE, vpMatrix);
        glUniform2f(g_uInvRes, 1.0f / viewport[2], 1.0f / viewport[3]);
        glUniform2f(g_uOrigin, static_cast<float>(viewport[0]), static_cast<float>(viewport[1]));
        glUniform3f(g_uCam, solve.cam[0], solve.cam[1], solve.cam[2]);
        glUniform1i(g_uCount, count);
        glUniform1i(g_uShadow, g_settings.shadow);
        glUniform1f(g_uGain, 1.25f * g_settings.intensity);
        glUniform1f(g_uVolGain, g_settings.volumetric);
        if (g_uBloomGain >= 0) glUniform1f(g_uBloomGain, g_settings.bloom);
        if (g_uWhiteGain >= 0) glUniform1f(g_uWhiteGain, g_settings.white * std::min(1.0f, g_settings.intensity + 0.28f));
        if (g_uBounceGain >= 0) glUniform1f(g_uBounceGain, g_settings.bounce);
        glUniform1f(g_uMaxDist, g_settings.range + 4.0f);
        glUniform4fv(g_uL, count * kVec4PerLight, reinterpret_cast<const float*>(relative));
        glActiveTexture(GL_TEXTURE0);
        glBindSampler(0, 0);
        glBindTexture(GL_TEXTURE_2D, g_depthTex);
        glBindVertexArray(g_vao);
        glBlendEquationSeparate(GL_FUNC_ADD, GL_FUNC_ADD);
        glEnable(GL_BLEND);
        glDisable(GL_DEPTH_TEST); glDepthMask(GL_FALSE);
        glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_FALSE);
        glDisable(GL_CULL_FACE); glDisable(GL_STENCIL_TEST); glDisable(GL_POLYGON_OFFSET_FILL);
        glDisable(GL_DITHER); glDisable(GL_SAMPLE_ALPHA_TO_COVERAGE); glDisable(GL_RASTERIZER_DISCARD);
        if (useScissor) { glEnable(GL_SCISSOR_TEST); glScissor(scissorRect[0], scissorRect[1], scissorRect[2], scissorRect[3]); }
        else glDisable(GL_SCISSOR_TEST);

        if (g_settings.debug > 0) {
            glUniform1i(g_uMode, 2);
            glBlendFuncSeparate(GL_ONE, GL_ONE, GL_ZERO, GL_ONE);
            o_drawArrays(GL_TRIANGLES, 0, 3);
        } else {
            // Surface lighting: result = dst * (1 + L). The existing scene colour acts as albedo.
            glUniform1i(g_uMode, 0);
            if (g_settings.albedo) glBlendFuncSeparate(GL_DST_COLOR, GL_ONE, GL_ZERO, GL_ONE);
            else glBlendFuncSeparate(GL_ONE, GL_ONE, GL_ZERO, GL_ONE);
            int queryIndex = -1;
            if (g_queryOk) {
                if (!g_queryPending[0]) queryIndex = 0; else if (!g_queryPending[1]) queryIndex = 1;
            }
            if (queryIndex >= 0) glBeginQuery(GL_ANY_SAMPLES_PASSED, g_query[queryIndex]);
            o_drawArrays(GL_TRIANGLES, 0, 3);
            if (queryIndex >= 0) { glEndQuery(GL_ANY_SAMPLES_PASSED); g_queryPending[queryIndex] = true; }
            if (!g_queryOk) g_lastLitFrame.store(frame, std::memory_order_relaxed);

            // White lift: additive, independent of the road/wall colour, so the pool under the
            // lamps reads as white light (like vehiclelights128) instead of orange-tinted paint.
            if (g_settings.white > 0.001f && g_uWhiteGain >= 0) {
                glUniform1i(g_uMode, 4);
                glBlendFuncSeparate(GL_ONE, GL_ONE, GL_ZERO, GL_ONE);
                o_drawArrays(GL_TRIANGLES, 0, 3);
            }
            if (g_settings.volumetric > 0.001f) {
                glUniform1i(g_uMode, 1);
                glBlendFuncSeparate(GL_ONE, GL_ONE, GL_ZERO, GL_ONE);
                o_drawArrays(GL_TRIANGLES, 0, 3);
            }
            // Sun-like glare around lamps that face the camera. Depth-tested in
            // linear distance in the shader (hidden behind buildings and cars,
            // never hidden by the source vehicle) and capped so it stays soft.
            if (g_settings.bloom > 0.001f && g_uBloomGain >= 0) {
                glUniform1i(g_uMode, 3);
                glBlendFuncSeparate(GL_ONE, GL_ONE, GL_ZERO, GL_ONE);
                o_drawArrays(GL_TRIANGLES, 0, 3);
            }
        }
        drew = true;
        g_lastLightCount = count;

        if (g_log && !g_firstPassLogged) {
            g_firstPassLogged = true;
            const float d0 = std::sqrt(relative[0].posRange[0] * relative[0].posRange[0] + relative[0].posRange[1] * relative[0].posRange[1] +
                                       relative[0].posRange[2] * relative[0].posRange[2]);
            g_log("[GL] first light pass drawn via %s: %d lights, %dx%d, scissor=%d, light0 distance from camera=%.1fm",
                  origin, count, viewport[2], viewport[3], useScissor ? 1 : 0, d0);
        }
    } while (false);

    // ---- restore
    glBindTexture(GL_TEXTURE_2D, static_cast<GLuint>(texture0));
    glBindSampler(0, static_cast<GLuint>(sampler0));
    glActiveTexture(static_cast<GLenum>(activeTex));
    glBindVertexArray(static_cast<GLuint>(vao));
    glUseProgram(static_cast<GLuint>(program));
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, static_cast<GLuint>(drawFb));
    glBindFramebuffer(GL_READ_FRAMEBUFFER, static_cast<GLuint>(readFb));
    glViewport(savedViewport[0], savedViewport[1], savedViewport[2], savedViewport[3]);
    glBlendEquationSeparate(static_cast<GLenum>(blendEqRgb), static_cast<GLenum>(blendEqA));
    glBlendFuncSeparate(blendSrcRgb, blendDstRgb, blendSrcA, blendDstA);
    glDepthFunc(depthFunc); glDepthMask(depthMask);
    glColorMask(colorMask[0], colorMask[1], colorMask[2], colorMask[3]);
    glScissor(scissorBox[0], scissorBox[1], scissorBox[2], scissorBox[3]);
    if (blend) glEnable(GL_BLEND); else glDisable(GL_BLEND);
    if (depthTest) glEnable(GL_DEPTH_TEST); else glDisable(GL_DEPTH_TEST);
    if (cull) glEnable(GL_CULL_FACE); else glDisable(GL_CULL_FACE);
    if (scissor) glEnable(GL_SCISSOR_TEST); else glDisable(GL_SCISSOR_TEST);
    if (stencil) glEnable(GL_STENCIL_TEST); else glDisable(GL_STENCIL_TEST);
    if (polygon) glEnable(GL_POLYGON_OFFSET_FILL); else glDisable(GL_POLYGON_OFFSET_FILL);
    if (dither) glEnable(GL_DITHER); else glDisable(GL_DITHER);
    if (alphaToCoverage) glEnable(GL_SAMPLE_ALPHA_TO_COVERAGE); else glDisable(GL_SAMPLE_ALPHA_TO_COVERAGE);
    if (rasterDiscard) glEnable(GL_RASTERIZER_DISCARD); else glDisable(GL_RASTERIZER_DISCARD);
    const GLenum error = glGetError();
    DrainGlErrors();
    if (drew) {
        if (error != GL_NO_ERROR && g_log) g_log("[GL] light pass completed with error 0x%x", static_cast<unsigned>(error));
        ++g_passCount;
        g_passMicros += static_cast<unsigned long long>(NowMicros() - start);
    }
    g_inPass = false;
    return drew;
}

// ---------------------------------------------------------------------------------------------
// Hooks
// ---------------------------------------------------------------------------------------------
inline GLuint CurrentProgram() {
    if (g_hookUseProgram) return g_curProgram;
    GLint program = 0;
    glGetIntegerv(GL_CURRENT_PROGRAM, &program);
    return static_cast<GLuint>(program);
}
inline GLuint CurrentFbo() {
    if (g_hookBindFbo) return g_curFbo;
    GLint fbo = 0;
    glGetIntegerv(GL_DRAW_FRAMEBUFFER_BINDING, &fbo);
    return static_cast<GLuint>(fbo);
}

void OnDraw() {
    if (g_inPass) return;
    const GLuint program = CurrentProgram();
    if (!program || program >= kMaxProg) return;
    ProgInfo& info = g_prog[program];
    const int frame = g_frames.load(std::memory_order_relaxed);
    if (!info.scanned) ScanProgram(program, info);
    if (info.view < 0 || info.proj < 0) return;
    if (info.cls == kClsUnknown || frame - info.checkedFrame >= 30) ClassifyProgram(program, info, frame);

    if (info.cls == kClsPerspective) {
        const GLuint fbo = CurrentFbo();
        FboStat* stat = FindFbo(fbo, true);
        if (!stat) return;
        ++stat->draws;
        stat->prog = program;
        if (stat->draws == 1 || (stat->draws & 63) == 0) glGetIntegerv(GL_VIEWPORT, stat->vp);
        return;
    }

    // First 2D draw after the world has been drawn: scene depth is still intact and the HUD
    // has not been drawn yet, so the lights never overlay the interface.
    if (g_injectedThisFrame) return;
    FboStat* scene = MainFbo();
    if (!scene || scene->draws < kMinSceneDraws) return;
    g_injectedThisFrame = true;
    if (!SnapshotUsable(CurrentSnapshot())) return;
    if (RunLightPass(*scene, "3D->2D transition")) ++g_injectCount;
}

void H_drawElements(GLenum mode, GLsizei count, GLenum type, const void* indices) {
    OnDraw();
    o_drawElements(mode, count, type, indices);
}
void H_drawArrays(GLenum mode, GLint first, GLsizei count) {
    OnDraw();
    o_drawArrays(mode, first, count);
}
void H_useProgram(GLuint program) {
    if (!g_inPass) g_curProgram = program;
    o_useProgram(program);
}
void H_bindFramebuffer(GLenum target, GLuint fbo) {
    if (!g_inPass && (target == GL_FRAMEBUFFER || target == GL_DRAW_FRAMEBUFFER)) g_curFbo = fbo;
    o_bindFramebuffer(target, fbo);
}

EGLBoolean H_swap(EGLDisplay display, EGLSurface surface) {
    const int frame = g_frames.fetch_add(1, std::memory_order_relaxed) + 1;
    const long long now = NowMicros();
    if (now - g_lastSettingsLoad > 4000000) { g_lastSettingsLoad = now; LoadSettings(); }

    // Fallback: no 2D draw followed the world this frame (unexpected render order).
    if (!g_injectedThisFrame && !g_inPass) {
        FboStat* scene = MainFbo();
        if (scene && scene->draws >= kMinSceneDraws && SnapshotUsable(CurrentSnapshot())) {
            if (!g_swapContextLogged && g_log) {
                g_swapContextLogged = true;
                g_log("[GL] no 3D->2D transition seen; using the eglSwapBuffers fallback (context=%p)", reinterpret_cast<void*>(eglGetCurrentContext()));
            }
            if (RunLightPass(*scene, "eglSwapBuffers fallback")) ++g_fallbackCount;
        }
    }
    if (frame % 600 == 0 && g_log)
        g_log("[GL] frame=%d passes=%llu avg=%lluus inject=%llu fallback=%llu litQueries=%llu/%llu active=%d lights=%d fbo=%u fmt=%d mode=%d",
              frame, g_passCount, g_passCount ? g_passMicros / g_passCount : 0ULL, g_injectCount, g_fallbackCount,
              g_litCount, g_litCount + g_unlitCount, IsActive() ? 1 : 0, g_lastLightCount,
              g_fboCount ? MainFbo()->fbo : 0u, g_fmtIndex, g_matrixMode);
    g_fboCount = 0;
    g_injectedThisFrame = false;
    return o_swap(display, surface);
}

bool HookSym(void* library, const char* symbol, void* hook, void** original) {
    if (!library || !aml) return false;
    const uintptr_t address = aml->GetSym(library, symbol);
    if (!address) { if (g_log) g_log("[GL] symbol not found: %s", symbol); return false; }
    const bool ok = aml->Hook(reinterpret_cast<void*>(address), hook, original);
    if (!ok && g_log) g_log("[GL] hook failed: %s", symbol);
    return ok;
}
bool HookOnce(bool& installed, void* library, const char* symbol, void* hook, void** original) {
    if (installed) return true;
    installed = HookSym(library, symbol, hook, original);
    return installed;
}

} // namespace

Settings& GetSettings() { return g_settings; }

void Install(LogFn log) {
    if (log) g_log = log;
    if (g_enabled.load(std::memory_order_acquire)) return;
    const long long now = NowMicros();
    if (now - g_lastInstallAttempt < 2000000) return;
    g_lastInstallAttempt = now;
    void* egl = aml ? aml->GetLibHandle("libEGL.so") : nullptr;
    void* gles = aml ? aml->GetLibHandle("libGLESv2.so") : nullptr;
    bool ok = egl && gles;
    ok &= HookOnce(g_hookSwap, egl, "eglSwapBuffers", reinterpret_cast<void*>(H_swap), reinterpret_cast<void**>(&o_swap));
    ok &= HookOnce(g_hookDrawElements, gles, "glDrawElements", reinterpret_cast<void*>(H_drawElements), reinterpret_cast<void**>(&o_drawElements));
    ok &= HookOnce(g_hookDrawArrays, gles, "glDrawArrays", reinterpret_cast<void*>(H_drawArrays), reinterpret_cast<void**>(&o_drawArrays));
    if (ok) {
        // Optional: cheaper state tracking. Without them the draw hook queries GL instead.
        HookOnce(g_hookUseProgram, gles, "glUseProgram", reinterpret_cast<void*>(H_useProgram), reinterpret_cast<void**>(&o_useProgram));
        HookOnce(g_hookBindFbo, gles, "glBindFramebuffer", reinterpret_cast<void*>(H_bindFramebuffer), reinterpret_cast<void**>(&o_bindFramebuffer));
        if (g_hookUseProgram) { GLint p = 0; glGetIntegerv(GL_CURRENT_PROGRAM, &p); g_curProgram = static_cast<GLuint>(p); }
        if (g_hookBindFbo) { GLint f = 0; glGetIntegerv(GL_DRAW_FRAMEBUFFER_BINDING, &f); g_curFbo = static_cast<GLuint>(f); }
    }
    g_enabled.store(ok, std::memory_order_release);
    LoadSettings();
    if (g_log) g_log("[GL] GLES3 interceptor %s (useProgram=%d bindFbo=%d)", ok ? "installed" : "incomplete - retaining Im3D fallback",
                     g_hookUseProgram ? 1 : 0, g_hookBindFbo ? 1 : 0);
}

void Publish(const Light* lights, int count, const float* cameraWorld) {
    const int next = 1 - g_snapIndex.load(std::memory_order_relaxed);
    Snapshot& snapshot = g_snap[next];
    snapshot.count = lights && count > 0 ? std::min(count, kMaxLights) : 0;
    for (int i = 0; i < snapshot.count; ++i) snapshot.lights[i] = lights[i];
    if (cameraWorld) for (int i = 0; i < 3; ++i) snapshot.camera[i] = cameraWorld[i];
    snapshot.stampFrame = g_frames.load(std::memory_order_relaxed);
    if (snapshot.count > 0 && !g_nonEmptySnapshotLogged) {
        g_nonEmptySnapshotLogged = true;
        if (g_log) g_log("[GL] first non-empty vehicle-light snapshot: %d lights", snapshot.count);
    }
    g_snapIndex.store(next, std::memory_order_release);
}

void Render() {
    // Intentionally empty: the pass runs from the draw-call hooks at the 3D -> 2D transition.
}

bool IsActive() {
    return g_enabled.load(std::memory_order_acquire) &&
        g_frames.load(std::memory_order_relaxed) - g_lastLitFrame.load(std::memory_order_relaxed) < 45;
}
bool IsInstalled() { return g_enabled.load(std::memory_order_acquire); }

} // namespace VehicleLightGL
