#include "screen_shadow_gl.h"
#include "screen_shadow_core.h"

#include <GLES3/gl3.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <vector>

namespace ScreenShadow {
namespace {

constexpr int kCascades = 2;

// ---------------------------------------------------------------------------------------------
// Shaders
// ---------------------------------------------------------------------------------------------

// One vertex per depth-grid cell. The vertex rebuilds the visible surface point from the scene depth
// and draws it as a square splat into the sun's orthographic map. No attributes are used.
constexpr const char* kSplatVs = R"GLSL(#version 300 es
precision highp float;
precision highp int;
precision highp sampler2D;
uniform sampler2D uDepth;
uniform mat4 uInvVP;
uniform mat4 uLightVP;
uniform ivec2 uCells;       // grid cells (nodes - 1) in x and y
uniform ivec2 uTexSize;
uniform int uStride;
uniform float uExtrude;
uniform vec3 uToSun;        // unit vector from the receiver toward the sun
uniform vec3 uProjZ;        // proj[10], proj[14], proj[11]
uniform vec2 uCut;          // depth-discontinuity limit: relative, absolute (m)
uniform vec2 uPlanar;       // planarity tolerance: relative to the cell depth step, absolute (depth units)

float linearW(float d) {
    return uProjZ.z * uProjZ.y / ((d * 2.0 - 1.0) * uProjZ.z - uProjZ.x);
}
ivec2 nodePx(ivec2 node) {
    return min(node * uStride + ivec2(uStride / 2), uTexSize - 1);
}
void main() {
    // Two triangles per grid cell, built from the four neighbouring depth samples.
    int cell = gl_VertexID / 6;
    int k = gl_VertexID - cell * 6;
    int tri = k / 3;
    int corner = k - tri * 3;
    ivec2 c0 = ivec2(cell % uCells.x, cell / uCells.x);
    ivec2 n0 = tri == 0 ? c0 : c0 + ivec2(1, 0);
    ivec2 n1 = tri == 0 ? c0 + ivec2(1, 0) : c0 + ivec2(1, 1);
    ivec2 n2 = c0 + ivec2(0, 1);
    ivec2 p0 = nodePx(n0);
    ivec2 p1 = nodePx(n1);
    ivec2 p2 = nodePx(n2);
    float d0 = texelFetch(uDepth, p0, 0).r;
    float d1 = texelFetch(uDepth, p1, 0).r;
    float d2 = texelFetch(uDepth, p2, 0).r;
    bool cull = d0 >= 0.999999 || d1 >= 0.999999 || d2 >= 0.999999;
    if (!cull) {
        float w0 = linearW(d0);
        float w1 = linearW(d1);
        float w2 = linearW(d2);
        // A far-plane/quantised depth sample can make the perspective
        // denominator zero. Comparing NaN with <= is false, so explicitly
        // require every reconstructed depth to be positive before emitting a
        // vertex; malformed clip coordinates can otherwise crash some mobile
        // GLES drivers instead of merely being clipped.
        cull = !(w0 > 0.0 && w1 > 0.0 && w2 > 0.0);
        if (!cull) {
            float wmin = min(w0, min(w1, w2));
            float wmax = max(w0, max(w1, w2));
            cull = (wmax - wmin) > uCut.x * wmin + uCut.y;   // silhouette: never bridge it
        }
    }
    if (!cull && uPlanar.x > 0.0) {
        // Depth is affine in screen space on any flat surface, so the centre of a cell must equal the
        // mean of its four corners. When it does not, the cell straddles a silhouette or a crease and
        // the triangle would be a sheet bridging two different surfaces (a phantom shadow). Thin poles
        // and wires fall out here too; they are covered by the geometry casters instead.
        ivec2 q00 = nodePx(c0);
        ivec2 q11 = nodePx(c0 + ivec2(1, 1));
        float a00 = texelFetch(uDepth, q00, 0).r;
        float a10 = texelFetch(uDepth, nodePx(c0 + ivec2(1, 0)), 0).r;
        float a01 = texelFetch(uDepth, nodePx(c0 + ivec2(0, 1)), 0).r;
        float a11 = texelFetch(uDepth, q11, 0).r;
        // The probe pixel is an integer, so with an odd stride it is not exactly the cell centre. Compare it
        // with the bilinear value AT ITS OWN position (exact for an affine depth plane), not with the mean.
        ivec2 qc = (q00 + q11) / 2;
        float ac = texelFetch(uDepth, qc, 0).r;
        vec2 span = vec2(max(q11.x - q00.x, 1), max(q11.y - q00.y, 1));
        vec2 t = clamp(vec2(qc - q00) / span, 0.0, 1.0);
        float expected = mix(mix(a00, a10, t.x), mix(a01, a11, t.x), t.y);
        float dStep = max(max(abs(a10 - a00), abs(a01 - a00)), max(abs(a11 - a10), abs(a11 - a01)));
        cull = a00 >= 0.999999 || a10 >= 0.999999 || a01 >= 0.999999 || a11 >= 0.999999 || ac >= 0.999999 ||
               abs(ac - expected) > uPlanar.x * dStep + uPlanar.y;
    }
    if (cull) {
        gl_Position = vec4(2.0, 2.0, 2.0, 1.0);
        return;
    }
    ivec2 pp = corner == 0 ? p0 : (corner == 1 ? p1 : p2);
    float d = corner == 0 ? d0 : (corner == 1 ? d1 : d2);
    vec2 uv = (vec2(pp) + 0.5) / vec2(uTexSize);
    vec4 w = uInvVP * vec4(uv * 2.0 - 1.0, d * 2.0 - 1.0, 1.0);
    vec3 p = w.xyz / (abs(w.w) < 1.0e-6 ? 1.0e-6 : w.w);
    // Build a thin solid slab away from the light. Extruding in camera space
    // makes walls leak and causes detached shadows; the light direction keeps
    // the caster/receiver relationship correct for roofs, walls and vehicles.
    p -= normalize(uToSun) * uExtrude;
    vec4 lp = uLightVP * vec4(p, 1.0);
    lp.z = max(lp.z, -0.999);
    gl_Position = lp;
}
)GLSL";

constexpr const char* kSplatFs = R"GLSL(#version 300 es
precision mediump float;
void main() {}
)GLSL";

// Geometry casters: real model triangles drawn depth-only into the sun maps. Positions are stored
// relative to the model origin; uOrigin is that origin relative to the camera (computed in double).
constexpr const char* kMeshVs = R"GLSL(#version 300 es
precision highp float;
layout(location = 0) in vec3 aPos;
uniform mat4 uLightVP;
uniform vec3 uOrigin;
void main() {
    vec4 c = uLightVP * vec4(aPos + uOrigin, 1.0);
    c.z = max(c.z, -0.999);      // anything between the sun and the map keeps blocking instead of being clipped
    gl_Position = c;
}
)GLSL";

constexpr const char* kRecvVs = R"GLSL(#version 300 es
precision highp float;
void main() {
    vec2 p = vec2(float((gl_VertexID << 1) & 2), float(gl_VertexID & 2));
    gl_Position = vec4(p * 2.0 - 1.0, 0.0, 1.0);
}
)GLSL";

constexpr const char* kRecvFs = R"GLSL(#version 300 es
precision highp float;
precision highp int;
precision highp sampler2D;
layout(location = 0) out vec4 outColor;

uniform sampler2D uDepth;
uniform sampler2D uShadow0;
uniform sampler2D uShadow1;
uniform mat4 uInvVP;
uniform mat4 uVP;
uniform mat4 uLightVP0;
uniform mat4 uLightVP1;
uniform vec4 uCas0;      // texel (m), 1/mapSize, depthHalf (m), radius (m)
uniform vec4 uCas1;
uniform vec2 uInvRes;
uniform vec2 uOrigin;
uniform vec3 uToSun;
uniform vec3 uProjZ;     // proj[10], proj[14], proj[11]
uniform vec4 uTune;      // strength, ambient, selfShade, sunStrength
uniform vec4 uTune2;     // range, splat texels per metre, soft, depth quantisation coefficient
uniform int uQuality;
uniform int uContact;
uniform int uDebug;

vec3 reconstruct(vec2 uv, float depth) {
    vec4 clip = vec4(uv * 2.0 - 1.0, depth * 2.0 - 1.0, 1.0);
    vec4 p = uInvVP * clip;
    return p.xyz / (abs(p.w) < 1.0e-6 ? 1.0e-6 : p.w);
}

// Linear view depth (clip.w) of a depth-buffer value.
float linearW(float d) {
    return uProjZ.z * uProjZ.y / ((d * 2.0 - 1.0) * uProjZ.z - uProjZ.x);
}

vec3 surfaceNormal(vec2 uv, float depth, vec3 p) {
    vec2 texel = uInvRes * 2.5;
    float dL = texture(uDepth, uv - vec2(texel.x, 0.0)).r;
    float dR = texture(uDepth, uv + vec2(texel.x, 0.0)).r;
    float dD = texture(uDepth, uv - vec2(0.0, texel.y)).r;
    float dU = texture(uDepth, uv + vec2(0.0, texel.y)).r;
    if (dL >= 0.999999) dL = depth;
    if (dR >= 0.999999) dR = depth;
    if (dD >= 0.999999) dD = depth;
    if (dU >= 0.999999) dU = depth;
    vec3 pL = reconstruct(uv - vec2(texel.x, 0.0), dL);
    vec3 pR = reconstruct(uv + vec2(texel.x, 0.0), dR);
    vec3 pD = reconstruct(uv - vec2(0.0, texel.y), dD);
    vec3 pU = reconstruct(uv + vec2(0.0, texel.y), dU);
    vec3 dx = abs(dL - depth) < abs(dR - depth) ? p - pL : pR - p;
    vec3 dy = abs(dD - depth) < abs(dU - depth) ? p - pD : pU - p;
    vec3 n = cross(dx, dy);
    float nl = length(n);
    vec3 normal = nl > 1.0e-9 ? n / nl : vec3(0.0, 0.0, 1.0);
    if (dot(normal, -p) < 0.0) normal = -normal;
    // When looking down at surfaces below the camera, the upward surface normal must not point into the earth.
    if (normal.z < 0.0 && -p.z > 0.0) normal = -normal;
    // Roads and walls are flat: snapping removes depth-quantisation stair steps.
    float gs = smoothstep(0.60, 0.85, normal.z);
    normal = normalize(mix(normal, vec3(0.0, 0.0, 1.0), gs * 0.9));
    float ws = 1.0 - smoothstep(0.18, 0.38, abs(normal.z));
    normal = normalize(vec3(normal.xy, normal.z * (1.0 - ws * 0.8)));
    return normal;
}

float ign(vec2 fragCoord) {
    return fract(52.9829189 * fract(dot(fragCoord, vec2(0.06711056, 0.00583715))));
}

vec2 vogel(int i, int n, float phi) {
    float r = sqrt((float(i) + 0.5) / float(n));
    float th = float(i) * 2.399963 + phi;
    return r * vec2(cos(th), sin(th));
}

// Lit fraction (1 = fully lit) of camera-relative point p against one sun map.
float cascadeVis(sampler2D smap, mat4 M, vec4 cas, vec3 p, vec3 n, float ndl, float splatTex,
                 float quantW, float noise) {
    float texel = cas.x;
    vec3 q = p + n * texel * (0.7 + 0.9 * (1.0 - clamp(ndl, 0.0, 1.0)));      // normal offset
    vec4 lc = M * vec4(q, 1.0);
    vec2 uv = lc.xy * 0.5 + 0.5;
    if (any(lessThan(uv, vec2(0.0))) || any(greaterThan(uv, vec2(1.0))) || abs(lc.z) > 1.0) return 1.0;
    float zr = lc.z * 0.5 + 0.5;
    float metresPerDepth = 2.0 * cas.z;
    float tanT = sqrt(max(1.0 - ndl * ndl, 0.0)) / max(ndl, 0.12);
    float biasW = texel * (1.0 + 1.0 * min(tanT, 5.0)) + quantW;
    float bias = biasW / metresPerDepth;
    float soft = uTune2.z;
    float uvPerMetre = 1.0 / (2.0 * cas.w);
    float phi = noise * 6.2831853;

    float penUv;
    if (uQuality > 0) {
        // blocker search, then a penumbra that widens with the occluder-receiver gap
        float searchUv = (0.25 + 0.55 * soft) * uvPerMetre;
        float sum = 0.0;
        float cnt = 0.0;
        for (int i = 0; i < 8; ++i) {
            float s = texture(smap, uv + vogel(i, 8, phi) * searchUv).r;
            if (s < zr - bias) { sum += s; cnt += 1.0; }
        }
        if (cnt < 0.5) return 1.0;
        float gapW = (zr - sum / cnt) * metresPerDepth;
        float penW = clamp(0.05 * soft * gapW, 1.2 * texel, 0.30 + 0.55 * soft);
        penUv = penW * uvPerMetre;
    } else {
        penUv = 1.6 * texel * max(soft, 0.3) * uvPerMetre;
    }
    const int kTaps = 12;
    int taps = uQuality > 0 ? kTaps : 6;
    float lit = 0.0;
    for (int i = 0; i < kTaps; ++i) {
        if (i >= taps) break;
        float s = texture(smap, uv + vogel(i, taps, phi + 1.7) * penUv).r;
        lit += (s < zr - bias) ? 0.0 : 1.0;
    }
    return lit / float(taps);
}

// Short screen-space march toward the sun: crisp contact detail the map is too coarse for.
float contactShadow(vec3 p, vec3 n, float noise) {
    vec3 start = p + n * 0.03;
    float vis = 1.0;
    for (int i = 1; i <= 8; ++i) {
        float f = (float(i) - 0.5 + noise * 0.5) / 8.0;
        float t = 0.06 + 0.9 * f * f;
        vec3 qp = start + uToSun * t;
        vec4 clip = uVP * vec4(qp, 1.0);
        if (clip.w <= 0.1) break;
        vec2 suv = clip.xy / clip.w * 0.5 + 0.5;
        if (any(lessThan(suv, vec2(0.002))) || any(greaterThan(suv, vec2(0.998)))) break;
        float sd = texture(uDepth, suv).r;
        if (sd >= 0.999999) continue;
        float delta = clip.w - linearW(sd);           // > 0: the ray point is behind the visible surface
        float thick = 0.12 + 0.35 * t;
        float hit = smoothstep(0.02 + 0.01 * t, 0.05 + 0.02 * t, delta) *
                    (1.0 - smoothstep(thick, thick * 1.8, delta));
        vis *= 1.0 - 0.85 * hit;
    }
    return vis;
}

void main() {
    vec2 uv = (gl_FragCoord.xy - uOrigin) * uInvRes;
    float depth = texture(uDepth, uv).r;
    if (depth >= 0.999999) discard;
    vec3 p = reconstruct(uv, depth);
    float dist = length(p);
    float range = uTune2.x;
    float rangeFade = 1.0 - smoothstep(range * 0.72, range, dist);
    if (rangeFade <= 0.003 && uDebug == 0) discard;

    vec3 n = surfaceNormal(uv, depth, p);
    if (uDebug == 3) { outColor = vec4(n * 0.5 + 0.5, 1.0); return; }
    float ndl = dot(n, uToSun);
    float splatTex = clamp(dist * uTune2.y + 1.0, 1.0, 14.0);
    float quantW = dist * dist * uTune2.w;
    float noise = ign(gl_FragCoord.xy);

    float mapVis = 1.0;
    vec3 cascadeTint = vec3(0.0);
    if (ndl > -0.25) {
        vec4 l0 = uLightVP0 * vec4(p, 1.0);
        float e0 = max(abs(l0.x), abs(l0.y));
        float use1 = smoothstep(0.80, 0.96, e0);
        float v0 = 1.0;
        float v1 = 1.0;
        if (use1 < 0.999) v0 = cascadeVis(uShadow0, uLightVP0, uCas0, p, n, ndl, splatTex, quantW, noise);
        if (use1 > 0.001) v1 = cascadeVis(uShadow1, uLightVP1, uCas1, p, n, ndl, splatTex, quantW, noise);
        mapVis = mix(v0, v1, use1);
        cascadeTint = mix(vec3(1.0, 0.25, 0.25), vec3(0.25, 1.0, 0.25), use1);
    }
    float contactVis = 1.0;
    if (uContact != 0 && ndl > 0.2) contactVis = mix(1.0, contactShadow(p, n, noise), smoothstep(0.2, 0.45, ndl));  // grazing sun: depth noise makes false hits

    if (uDebug == 1) { outColor = vec4(vec3(mapVis), 1.0); return; }
    if (uDebug == 2) { outColor = vec4(cascadeTint * (0.4 + 0.6 * mapVis), 1.0); return; }
    if (uDebug == 4) { outColor = vec4(vec3(contactVis), 1.0); return; }

    float lam = smoothstep(-0.25, 0.55, ndl);
    // Faces turned away from the sun are dark because of the angle, not because of the map: ignore the
    // (noisy) map result there so they get a smooth tone instead of stripes.
    float mapUse = smoothstep(-0.10, 0.12, ndl);
    float lit = mix(1.0, mapVis, mapUse) * mix(1.0, contactVis, mapUse) * mix(1.0, lam, uTune.z);
    float strength = uTune.x;
    vec3 shadowMul = pow(uTune.y * vec3(0.86, 0.92, 1.0), vec3(max(strength, 1.0)));
    float amount = (1.0 - lit) * min(strength, 1.0) * rangeFade * uTune.w;
    outColor = vec4(mix(vec3(1.0), shadowMul, clamp(amount, 0.0, 1.0)), 1.0);
}
)GLSL";

// ---------------------------------------------------------------------------------------------
// GL state
// ---------------------------------------------------------------------------------------------
struct SplatLoc { GLint depth = -1, invVp = -1, lightVp = -1, cells = -1, texSize = -1, stride = -1, extrude = -1, toSun = -1, projZ = -1, cut = -1, planar = -1; };
struct MeshLoc { GLint lightVp = -1, origin = -1; };
struct RecvLoc {
    GLint depth = -1, shadow0 = -1, shadow1 = -1, invVp = -1, vp = -1, lightVp0 = -1, lightVp1 = -1, cas0 = -1, cas1 = -1,
          invRes = -1, origin = -1, toSun = -1, projZ = -1, tune = -1, tune2 = -1, quality = -1, contact = -1, debug = -1;
};

struct DepthFmt { GLint internal; GLenum format; GLenum type; const char* name; };
const DepthFmt kFmts[] = {
    {GL_DEPTH_COMPONENT24, GL_DEPTH_COMPONENT, GL_UNSIGNED_INT, "D24"},
    {GL_DEPTH_COMPONENT32F, GL_DEPTH_COMPONENT, GL_FLOAT, "D32F"},
    {GL_DEPTH_COMPONENT16, GL_DEPTH_COMPONENT, GL_UNSIGNED_SHORT, "D16"},
};
constexpr int kFmtCount = sizeof(kFmts) / sizeof(kFmts[0]);

struct State {
    bool failed = false, ready = false;
    GLuint splatProg = 0, recvProg = 0, meshProg = 0, vao = 0;
    GLuint fbo[kCascades] = {0, 0};
    GLuint tex[kCascades] = {0, 0};
    int mapSize = 0, fmt = -1;
    SplatLoc splat;
    MeshLoc mesh;
    RecvLoc recv;
    bool firstLogged = false;
    int errorLogBudget = 4;
};
State g;
Stats g_stats;
LogFn g_log = nullptr;

// ---------------------------------------------------------------------------------------------
// Geometry caster store (CPU copy owned here, GL objects created lazily on the GL thread)
// ---------------------------------------------------------------------------------------------
struct Caster {
    uint32_t key = 0;
    double origin[3] = {0, 0, 0};
    std::vector<float> verts;
    std::vector<uint32_t> idx;
    float center[3] = {0, 0, 0};     // bounding sphere, relative to origin
    float radius = 0.0f;
    bool dirty = true;
    GLuint vao = 0, vbo = 0, ibo = 0;
};
constexpr uint32_t kTombstone = 0xFFFFFFFFu;
std::mutex g_casterMutex;
std::vector<Caster> g_casters;
constexpr size_t kMaxCasters = 128;

void ReleaseCasterGl(Caster& c, bool contextAlive) {
    if (contextAlive) {
        if (c.vao) glDeleteVertexArrays(1, &c.vao);
        if (c.vbo) glDeleteBuffers(1, &c.vbo);
        if (c.ibo) glDeleteBuffers(1, &c.ibo);
    }
    c.vao = c.vbo = c.ibo = 0; c.dirty = true;
}

void Drain() { int guard = 0; while (glGetError() != GL_NO_ERROR && ++guard < 16) {} }

GLuint Compile(GLenum type, const char* src) {
    const GLuint sh = glCreateShader(type);
    if (!sh) return 0;
    glShaderSource(sh, 1, &src, nullptr);
    glCompileShader(sh);
    GLint ok = GL_FALSE;
    glGetShaderiv(sh, GL_COMPILE_STATUS, &ok);
    if (ok == GL_TRUE) return sh;
    char log[1024]{};
    glGetShaderInfoLog(sh, sizeof(log) - 1, nullptr, log);
    if (g_log) g_log("[SunShadow] %s shader compile failed: %s", type == GL_VERTEX_SHADER ? "vertex" : "fragment", log);
    glDeleteShader(sh);
    return 0;
}

GLuint Link(const char* vs, const char* fs, const char* name) {
    const GLuint v = Compile(GL_VERTEX_SHADER, vs);
    const GLuint f = v ? Compile(GL_FRAGMENT_SHADER, fs) : 0;
    if (!v || !f) { if (v) glDeleteShader(v); if (f) glDeleteShader(f); return 0; }
    const GLuint prog = glCreateProgram();
    glAttachShader(prog, v);
    glAttachShader(prog, f);
    glLinkProgram(prog);
    glDeleteShader(v);
    glDeleteShader(f);
    GLint ok = GL_FALSE;
    glGetProgramiv(prog, GL_LINK_STATUS, &ok);
    if (ok == GL_TRUE) return prog;
    char log[1024]{};
    glGetProgramInfoLog(prog, sizeof(log) - 1, nullptr, log);
    if (g_log) g_log("[SunShadow] %s program link failed: %s", name, log);
    glDeleteProgram(prog);
    return 0;
}

bool InitPrograms() {
    g.splatProg = Link(kSplatVs, kSplatFs, "splat");
    g.recvProg = g.splatProg ? Link(kRecvVs, kRecvFs, "receiver") : 0;
    if (!g.splatProg || !g.recvProg) return false;
    // The caster program is optional: without it the depth-built maps still work.
    g.meshProg = Link(kMeshVs, kSplatFs, "caster");
    if (g.meshProg) {
        g.mesh.lightVp = glGetUniformLocation(g.meshProg, "uLightVP");
        g.mesh.origin = glGetUniformLocation(g.meshProg, "uOrigin");
        if (g.mesh.lightVp < 0 || g.mesh.origin < 0) { glDeleteProgram(g.meshProg); g.meshProg = 0; }
    }
    SplatLoc& s = g.splat;
    s.depth = glGetUniformLocation(g.splatProg, "uDepth");
    s.invVp = glGetUniformLocation(g.splatProg, "uInvVP");
    s.lightVp = glGetUniformLocation(g.splatProg, "uLightVP");
    s.cells = glGetUniformLocation(g.splatProg, "uCells");
    s.texSize = glGetUniformLocation(g.splatProg, "uTexSize");
    s.stride = glGetUniformLocation(g.splatProg, "uStride");
    s.extrude = glGetUniformLocation(g.splatProg, "uExtrude");
    s.toSun = glGetUniformLocation(g.splatProg, "uToSun");
    s.projZ = glGetUniformLocation(g.splatProg, "uProjZ");
    s.cut = glGetUniformLocation(g.splatProg, "uCut");
    s.planar = glGetUniformLocation(g.splatProg, "uPlanar");
    RecvLoc& r = g.recv;
    r.depth = glGetUniformLocation(g.recvProg, "uDepth");
    r.shadow0 = glGetUniformLocation(g.recvProg, "uShadow0");
    r.shadow1 = glGetUniformLocation(g.recvProg, "uShadow1");
    r.invVp = glGetUniformLocation(g.recvProg, "uInvVP");
    r.vp = glGetUniformLocation(g.recvProg, "uVP");
    r.lightVp0 = glGetUniformLocation(g.recvProg, "uLightVP0");
    r.lightVp1 = glGetUniformLocation(g.recvProg, "uLightVP1");
    r.cas0 = glGetUniformLocation(g.recvProg, "uCas0");
    r.cas1 = glGetUniformLocation(g.recvProg, "uCas1");
    r.invRes = glGetUniformLocation(g.recvProg, "uInvRes");
    r.origin = glGetUniformLocation(g.recvProg, "uOrigin");
    r.toSun = glGetUniformLocation(g.recvProg, "uToSun");
    r.projZ = glGetUniformLocation(g.recvProg, "uProjZ");
    r.tune = glGetUniformLocation(g.recvProg, "uTune");
    r.tune2 = glGetUniformLocation(g.recvProg, "uTune2");
    r.quality = glGetUniformLocation(g.recvProg, "uQuality");
    r.contact = glGetUniformLocation(g.recvProg, "uContact");
    r.debug = glGetUniformLocation(g.recvProg, "uDebug");
    if (s.depth < 0 || s.invVp < 0 || s.lightVp < 0 || s.cells < 0 || s.toSun < 0 || r.depth < 0 || r.shadow0 < 0 || r.shadow1 < 0 ||
        r.invVp < 0 || r.lightVp0 < 0 || r.tune < 0) {
        if (g_log) g_log("[SunShadow] uniform lookup failed (splat depth=%d cells=%d recv depth=%d tune=%d)", s.depth, s.cells, r.depth, r.tune);
        return false;
    }
    return true;
}

void DestroyMaps() {
    for (int i = 0; i < kCascades; ++i) {
        if (g.fbo[i]) glDeleteFramebuffers(1, &g.fbo[i]);
        if (g.tex[i]) glDeleteTextures(1, &g.tex[i]);
        g.fbo[i] = 0; g.tex[i] = 0;
    }
    g.mapSize = 0;
}

bool AllocateMaps(int size) {
    // IMPORTANT: glReadBuffer()/glDrawBuffers() act on whatever framebuffer is bound to the matching
    // target. The old code bound only GL_DRAW_FRAMEBUFFER and then called glReadBuffer(GL_NONE), which
    // switched the GAME's scene framebuffer (still bound for reading) to "no read buffer". The game's
    // own colour blit/copy from that framebuffer then failed every frame and the picture froze until a
    // menu recreated the framebuffer. Bind GL_FRAMEBUFFER (both targets) to our FBO while configuring it
    // and put both original bindings back afterwards.
    GLint prevDrawFbo = 0, prevReadFbo = 0, prevTex = 0, prevActive = GL_TEXTURE0;
    glGetIntegerv(GL_DRAW_FRAMEBUFFER_BINDING, &prevDrawFbo);
    glGetIntegerv(GL_READ_FRAMEBUFFER_BINDING, &prevReadFbo);
    glGetIntegerv(GL_ACTIVE_TEXTURE, &prevActive);
    glActiveTexture(GL_TEXTURE1);
    glGetIntegerv(GL_TEXTURE_BINDING_2D, &prevTex);
    DestroyMaps();
    bool ok = false;
    for (int attempt = 0; attempt < kFmtCount && !ok; ++attempt) {
        Drain();
        ok = true;
        for (int i = 0; i < kCascades && ok; ++i) {
            if (!g.tex[i]) glGenTextures(1, &g.tex[i]);
            if (!g.fbo[i]) glGenFramebuffers(1, &g.fbo[i]);
            glBindTexture(GL_TEXTURE_2D, g.tex[i]);
            glTexImage2D(GL_TEXTURE_2D, 0, kFmts[attempt].internal, size, size, 0, kFmts[attempt].format, kFmts[attempt].type, nullptr);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_NONE);
            glBindFramebuffer(GL_FRAMEBUFFER, g.fbo[i]);                 // both targets: our FBO only
            glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, g.tex[i], 0);
            const GLenum none = GL_NONE;
            glDrawBuffers(1, &none);
            glReadBuffer(GL_NONE);
            if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE || glGetError() != GL_NO_ERROR) ok = false;
        }
        if (ok) { g.fmt = attempt; g_stats.depthFormat = attempt; }
    }
    glBindTexture(GL_TEXTURE_2D, static_cast<GLuint>(prevTex));
    glActiveTexture(static_cast<GLenum>(prevActive));
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, static_cast<GLuint>(prevDrawFbo));
    glBindFramebuffer(GL_READ_FRAMEBUFFER, static_cast<GLuint>(prevReadFbo));
    if (!ok) { DestroyMaps(); if (g_log) g_log("[SunShadow] could not create a complete depth framebuffer (%dx%d)", size, size); return false; }
    g.mapSize = size;
    if (g_log) g_log("[SunShadow] sun maps ready: 2 x %dx%d %s", size, size, kFmts[g.fmt].name);
    return true;
}

bool Ensure(int mapSize) {
    if (g.failed) return false;
    if (g.ready && (!glIsProgram(g.recvProg) || !glIsProgram(g.splatProg))) {   // context was recreated
        g = State{};
        if (g_log) g_log("[SunShadow] GL context changed, rebuilding");
    }
    if (!g.ready) {
        Drain();
        if (!InitPrograms()) { g.failed = true; return false; }
        GLint prevVao = 0;
        glGetIntegerv(GL_VERTEX_ARRAY_BINDING, &prevVao);
        glGenVertexArrays(1, &g.vao);
        glBindVertexArray(static_cast<GLuint>(prevVao));
        if (!g.vao) { g.failed = true; return false; }
        g.ready = true;
    }
    mapSize = std::max(512, std::min(2048, mapSize));
    if (g.mapSize != mapSize && !AllocateMaps(mapSize)) { g.failed = true; return false; }
    return g.mapSize > 0;
}

void SetMat(GLint loc, const float* m) { if (loc >= 0) glUniformMatrix4fv(loc, 1, GL_FALSE, m); }

// Uploads new/changed caster meshes and frees the GL objects of removed ones. Caller holds g_casterMutex.
void PrepareCasters() {
    for (Caster& c : g_casters) {
        if (c.key == kTombstone) { if (c.vao || c.vbo || c.ibo) ReleaseCasterGl(c, true); continue; }
        if (!c.dirty || c.idx.empty()) continue;
        if (!c.vao) glGenVertexArrays(1, &c.vao);
        if (!c.vbo) glGenBuffers(1, &c.vbo);
        if (!c.ibo) glGenBuffers(1, &c.ibo);
        if (!c.vao || !c.vbo || !c.ibo) continue;
        glBindVertexArray(c.vao);
        glBindBuffer(GL_ARRAY_BUFFER, c.vbo);
        glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(c.verts.size() * sizeof(float)), c.verts.data(), GL_STATIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, c.ibo);       // stored in this VAO, not in the game's
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, static_cast<GLsizeiptr>(c.idx.size() * sizeof(uint32_t)), c.idx.data(), GL_STATIC_DRAW);
        c.dirty = false;
    }
    glBindVertexArray(g.vao);
}

// Draws every caster that can touch this cascade, depth only. Caller holds g_casterMutex.
void DrawCasters(const ShadowMap::Cascade& cas, const double camWorld[3]) {
    glUseProgram(g.meshProg);
    SetMat(g.mesh.lightVp, cas.vp);
    for (const Caster& k : g_casters) {
        if (k.key == kTombstone || k.idx.empty() || !k.vao || k.dirty) continue;
        const float rel[3] = {static_cast<float>(k.origin[0] - camWorld[0]), static_cast<float>(k.origin[1] - camWorld[1]),
                              static_cast<float>(k.origin[2] - camWorld[2])};
        const float c3[3] = {rel[0] + k.center[0], rel[1] + k.center[1], rel[2] + k.center[2]};
        if (std::fabs(c3[0]) > 600.0f || std::fabs(c3[1]) > 600.0f) continue;
        float ndc[3];
        ShadowMap::ToLight(cas, c3, ndc);
        const float rn = k.radius / cas.radius;
        if (std::fabs(ndc[0]) > 1.0f + rn || std::fabs(ndc[1]) > 1.0f + rn) continue;       // outside this cascade
        glUniform3f(g.mesh.origin, rel[0], rel[1], rel[2]);
        glBindVertexArray(k.vao);
        glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(k.idx.size()), GL_UNSIGNED_INT, nullptr);
        ++g_stats.casterDraws;
        g_stats.casterTris += static_cast<int>(k.idx.size() / 3);
    }
    glBindVertexArray(g.vao);
}

} // namespace

bool IsReady() { return g.ready && g.mapSize > 0 && !g.failed; }
void Reset() {
    g = State{};
    std::lock_guard<std::mutex> lock(g_casterMutex);
    for (Caster& c : g_casters) ReleaseCasterGl(c, false);      // the old context (and its objects) is gone
}

void CasterSet(uint32_t key, const double origin[3], const float* verts, int vertexCount, const uint32_t* indices, int indexCount) {
    if (!verts || !indices || vertexCount < 3 || indexCount < 3) { CasterRemove(key); return; }
    Caster tmp;
    tmp.key = key;
    for (int k = 0; k < 3; ++k) tmp.origin[k] = origin[k];
    tmp.verts.assign(verts, verts + static_cast<size_t>(vertexCount) * 3);
    indexCount -= indexCount % 3;
    tmp.idx.assign(indices, indices + indexCount);
    float mn[3] = {1e30f, 1e30f, 1e30f}, mx[3] = {-1e30f, -1e30f, -1e30f};
    for (int i = 0; i < vertexCount; ++i)
        for (int k = 0; k < 3; ++k) { const float v = verts[i * 3 + k]; mn[k] = std::min(mn[k], v); mx[k] = std::max(mx[k], v); }
    float r2 = 0.0f;
    for (int k = 0; k < 3; ++k) tmp.center[k] = 0.5f * (mn[k] + mx[k]);
    for (int i = 0; i < vertexCount; ++i) {
        float d2 = 0.0f;
        for (int k = 0; k < 3; ++k) { const float d = verts[i * 3 + k] - tmp.center[k]; d2 += d * d; }
        r2 = std::max(r2, d2);
    }
    tmp.radius = std::sqrt(r2);
    if (!(tmp.radius > 0.0f) || !std::isfinite(tmp.radius)) return;
    if (key == kTombstone) return;
    std::lock_guard<std::mutex> lock(g_casterMutex);
    Caster* slot = nullptr;
    for (Caster& c : g_casters) if (c.key == key) { slot = &c; break; }
    if (!slot) for (Caster& c : g_casters) if (c.key == kTombstone) { slot = &c; break; }
    if (!slot) {
        if (g_casters.size() >= kMaxCasters) return;
        g_casters.emplace_back();
        slot = &g_casters.back();
    }
    const GLuint vao = slot->vao, vbo = slot->vbo, ibo = slot->ibo;      // keep the GL names, re-upload the data
    *slot = std::move(tmp);
    slot->vao = vao; slot->vbo = vbo; slot->ibo = ibo; slot->dirty = true;
}

void CasterRemove(uint32_t key) {
    std::lock_guard<std::mutex> lock(g_casterMutex);
    for (size_t i = 0; i < g_casters.size(); ++i) {
        if (g_casters[i].key != key) continue;
        // GL names are released on the GL thread (next Render) so no GL call happens here.
        g_casters[i].radius = 0.0f; g_casters[i].idx.clear(); g_casters[i].verts.clear();
        g_casters[i].key = kTombstone;
        return;
    }
}

void CasterClear() {
    std::lock_guard<std::mutex> lock(g_casterMutex);
    for (Caster& c : g_casters) { c.radius = 0.0f; c.idx.clear(); c.verts.clear(); c.key = kTombstone; }
}

int CasterCount() {
    std::lock_guard<std::mutex> lock(g_casterMutex);
    int n = 0;
    for (const Caster& c : g_casters) if (c.key != kTombstone && !c.idx.empty()) ++n;
    return n;
}
const Stats& GetStats() { return g_stats; }

bool Render(const Frame& f, const Params& pr, LogFn log) {
    g_log = log;
    if (f.depthTex == 0 || f.width <= 0 || f.height <= 0) return false;
    if (!Ensure(pr.mapSize)) return false;

    // ---- cascades
    float fwd[3] = {0, 1, 0};
    if (!ShadowMap::ViewForward(f.invVp, fwd)) { fwd[0] = 0; fwd[1] = 1; fwd[2] = 0; }
    const float r1 = std::max(20.0f, std::min(160.0f, pr.range));
    const float r0 = std::max(10.0f, std::min(34.0f, r1 * 0.26f));
    ShadowMap::Cascade cas[kCascades];
    if (!ShadowMap::BuildCascade(f.camWorld, fwd, f.toSun, r0, r0 + 120.0f, 0.0f, g.mapSize, cas[0]) ||
        !ShadowMap::BuildCascade(f.camWorld, fwd, f.toSun, r1, r1 + 120.0f, 0.0f, g.mapSize, cas[1]))
        return false;

    int stride = std::max(1, std::min(8, pr.density));
    // A 1512x680 scene with stride 3 creates 682k vertices per cascade
    // (1.36M for two cascades). That is unnecessarily aggressive on mobile
    // GPUs and can trigger a driver reset/FC after sustained rendering.
    // Keep the near/far receiver shader active, but coarsen only the caster
    // grid at large viewports; the depth edge cut still preserves silhouettes.
    const int pixels = f.width * f.height;
    const bool highLoad = pixels > 900000;
    if (highLoad) stride = std::max(stride, 6);
    else if (pixels > 600000) stride = std::max(stride, 4);
    const int nodesX = (f.width + stride - 1) / stride, nodesY = (f.height + stride - 1) / stride;
    const int cellsX = std::max(1, nodesX - 1), cellsY = std::max(1, nodesY - 1);
    const int vertices = cellsX * cellsY * 6;
    g_stats.pointsPerDraw = vertices; g_stats.mapSize = g.mapSize;
    g_stats.radius0 = cas[0].radius; g_stats.radius1 = cas[1].radius;
    g_stats.texel0 = cas[0].texel; g_stats.texel1 = cas[1].texel;

    // ---- save what only this function touches
    GLint activeTex = GL_TEXTURE0, tex1 = 0, tex2 = 0, smp1 = 0, smp2 = 0, smp0 = 0;
    GLfloat depthRange[2] = {0.0f, 1.0f};
    GLint arrayBuffer = 0;
    glGetIntegerv(GL_ARRAY_BUFFER_BINDING, &arrayBuffer);
    glGetIntegerv(GL_ACTIVE_TEXTURE, &activeTex);
    glGetFloatv(GL_DEPTH_RANGE, depthRange);
    glActiveTexture(GL_TEXTURE1); glGetIntegerv(GL_TEXTURE_BINDING_2D, &tex1); glGetIntegerv(GL_SAMPLER_BINDING, &smp1);
    glActiveTexture(GL_TEXTURE2); glGetIntegerv(GL_TEXTURE_BINDING_2D, &tex2); glGetIntegerv(GL_SAMPLER_BINDING, &smp2);
    glActiveTexture(GL_TEXTURE0); glGetIntegerv(GL_SAMPLER_BINDING, &smp0);
    Drain();

    glBindVertexArray(g.vao);
    glActiveTexture(GL_TEXTURE0);
    glBindSampler(0, 0);
    glBindTexture(GL_TEXTURE_2D, f.depthTex);

    // ---- pass 1: sun maps from the visible surfaces
    glUseProgram(g.splatProg);
    glDisable(GL_BLEND); glDisable(GL_SCISSOR_TEST); glDisable(GL_CULL_FACE); glDisable(GL_STENCIL_TEST);
    glDisable(GL_POLYGON_OFFSET_FILL); glDisable(GL_RASTERIZER_DISCARD); glDisable(GL_SAMPLE_ALPHA_TO_COVERAGE);
    glEnable(GL_DEPTH_TEST); glDepthFunc(GL_LESS); glDepthMask(GL_TRUE); glDepthRangef(0.0f, 1.0f);
    glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
    glUniform1i(g.splat.depth, 0);
    SetMat(g.splat.invVp, f.invVp);
    glUniform2i(g.splat.cells, cellsX, cellsY);
    glUniform3f(g.splat.toSun, f.toSun[0], f.toSun[1], f.toSun[2]);
    glUniform3f(g.splat.projZ, f.proj[10], f.proj[14], f.proj[11]);
    glUniform2f(g.splat.cut, 0.10f, 0.30f + 0.12f * static_cast<float>(stride));
    // Flatness test of each depth cell: relative to the cell's own depth step, plus a few depth LSBs.
    if (g.splat.planar >= 0)
        glUniform2f(g.splat.planar, 0.08f, 4.0f * std::ldexp(1.0f, -std::max(12, std::min(32, f.depthBits))));
    glUniform2i(g.splat.texSize, f.width, f.height);
    glUniform1i(g.splat.stride, stride);
    const float one = 1.0f;
    // The second extruded draw is useful on small viewports, but it doubles the
    // most expensive pass on phones. At 1512x680 the receiver still gets the
    // normal contact/PCF filtering, so omit only this optional slab draw.
    const float thickness = highLoad ? 0.0f : std::max(0.0f, std::min(2.0f, pr.thickness));
    g_stats.casterDraws = 0; g_stats.casterTris = 0;
    const bool drawCasters = pr.casters != 0 && g.meshProg != 0;
    const bool drawSplat = pr.splat != 0 || !drawCasters;      // never leave the maps empty
    std::unique_lock<std::mutex> casterLock(g_casterMutex, std::defer_lock);
    if (drawCasters) { casterLock.lock(); PrepareCasters(); }
    for (int c = 0; c < kCascades; ++c) {
        glBindFramebuffer(GL_DRAW_FRAMEBUFFER, g.fbo[c]);
        glViewport(0, 0, g.mapSize, g.mapSize);
        glClearBufferfv(GL_DEPTH, 0, &one);
        if (drawCasters) DrawCasters(cas[c], f.camWorld);
        if (!drawSplat) continue;
        glUseProgram(g.splatProg);
        SetMat(g.splat.lightVp, cas[c].vp);
        glUniform1f(g.splat.extrude, 0.0f);
        glDrawArrays(GL_TRIANGLES, 0, vertices);
        if (thickness > 0.01f) {
            glUniform1f(g.splat.extrude, thickness);
            glDrawArrays(GL_TRIANGLES, 0, vertices);
        }
    }

    if (casterLock.owns_lock()) casterLock.unlock();

    // ---- pass 2: receiver, multiplies the scene colour
    glBindFramebuffer(GL_FRAMEBUFFER, f.targetFbo);
    glViewport(f.vpX, f.vpY, f.width, f.height);
    glUseProgram(g.recvProg);
    glDisable(GL_DEPTH_TEST); glDepthMask(GL_FALSE);
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_FALSE);
    if (pr.debug > 0) glDisable(GL_BLEND);
    else {
        glEnable(GL_BLEND);
        glBlendEquationSeparate(GL_FUNC_ADD, GL_FUNC_ADD);
        glBlendFuncSeparate(GL_DST_COLOR, GL_ZERO, GL_ZERO, GL_ONE);          // result = scene * multiplier
    }
    glActiveTexture(GL_TEXTURE1); glBindSampler(1, 0); glBindTexture(GL_TEXTURE_2D, g.tex[0]);
    glActiveTexture(GL_TEXTURE2); glBindSampler(2, 0); glBindTexture(GL_TEXTURE_2D, g.tex[1]);
    glActiveTexture(GL_TEXTURE0);
    glUniform1i(g.recv.depth, 0); glUniform1i(g.recv.shadow0, 1); glUniform1i(g.recv.shadow1, 2);
    SetMat(g.recv.invVp, f.invVp); SetMat(g.recv.vp, f.vp);
    SetMat(g.recv.lightVp0, cas[0].vp); SetMat(g.recv.lightVp1, cas[1].vp);
    glUniform4f(g.recv.cas0, cas[0].texel, 1.0f / g.mapSize, cas[0].depthHalf, cas[0].radius);
    glUniform4f(g.recv.cas1, cas[1].texel, 1.0f / g.mapSize, cas[1].depthHalf, cas[1].radius);
    glUniform2f(g.recv.invRes, 1.0f / f.width, 1.0f / f.height);
    glUniform2f(g.recv.origin, static_cast<float>(f.vpX), static_cast<float>(f.vpY));
    glUniform3f(g.recv.toSun, f.toSun[0], f.toSun[1], f.toSun[2]);
    glUniform3f(g.recv.projZ, f.proj[10], f.proj[14], f.proj[11]);
    // Depth-buffer precision -> world-space error coefficient (error ~ dist^2 * coef).
    const float zNear = std::fabs(f.proj[14] / (-f.proj[11] - f.proj[10] + 1.0e-9f));
    const float quantCoef = std::ldexp(1.0f, 1 - std::max(12, std::min(32, f.depthBits))) / std::max(zNear, 0.05f);
    glUniform4f(g.recv.tune, pr.strength, std::max(0.0f, std::min(1.0f, pr.ambient)), std::max(0.0f, std::min(1.0f, pr.selfShade)),
                std::max(0.0f, std::min(1.0f, f.sunStrength)));
    glUniform4f(g.recv.tune2, r1, 0.0f, std::max(0.1f, pr.soft), quantCoef);
    // Contact marching is an optional detail. On the 1512x680 target it adds
    // eight dependent depth reads per pixel on top of the PCF lookups and is
    // a common GPU watchdog trigger on older Adreno drivers. The map shadow
    // and its soft PCF remain enabled.
    glUniform1i(g.recv.quality, highLoad ? 0 : pr.quality);
    glUniform1i(g.recv.contact, highLoad ? 0 : pr.contact);
    glUniform1i(g.recv.debug, pr.debug);
    glDrawArrays(GL_TRIANGLES, 0, 3);

    // ---- restore
    glActiveTexture(GL_TEXTURE2); glBindTexture(GL_TEXTURE_2D, static_cast<GLuint>(tex2)); glBindSampler(2, static_cast<GLuint>(smp2));
    glActiveTexture(GL_TEXTURE1); glBindTexture(GL_TEXTURE_2D, static_cast<GLuint>(tex1)); glBindSampler(1, static_cast<GLuint>(smp1));
    glActiveTexture(GL_TEXTURE0); glBindSampler(0, static_cast<GLuint>(smp0));
    glActiveTexture(static_cast<GLenum>(activeTex));
    glBindBuffer(GL_ARRAY_BUFFER, static_cast<GLuint>(arrayBuffer));
    glDepthRangef(depthRange[0], depthRange[1]);

    const GLenum err = glGetError();
    Drain();
    if (err != GL_NO_ERROR && g.errorLogBudget > 0) {
        --g.errorLogBudget;
        if (g_log) g_log("[SunShadow] pass finished with GL error 0x%x", static_cast<int>(err));
    }
    if (!g.firstLogged && g_log) {
        g.firstLogged = true;
        g_log("[SunShadow] first pass: %dx%d viewport, %d vertices/draw (stride %d), cascades r=%.0fm/%.0fm texel=%.1fcm/%.1fcm casters=%d",
              f.width, f.height, vertices, stride, static_cast<double>(cas[0].radius), static_cast<double>(cas[1].radius),
              static_cast<double>(cas[0].texel * 100.0f), static_cast<double>(cas[1].texel * 100.0f), g_stats.casterDraws);
    }
    return true;     // drawn; a stray GL error is logged above but does not retire the pass
}

} // namespace ScreenShadow
