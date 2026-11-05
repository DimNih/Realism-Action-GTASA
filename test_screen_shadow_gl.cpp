// Host-side check of the screen-built sun shadows, run on Mesa's software GLES 3 (llvmpipe).
// Not part of the Android shared library. It renders a synthetic GTA-like street (ground, buildings,
// a pole, a pedestrian, a car, a thin wall, a tree), runs the REAL ScreenShadow::Render() on the
// resulting depth buffer and compares every sun-facing pixel with an analytic ray-traced ground truth.
//
//   g++ -std=c++17 -O2 test_screen_shadow_gl.cpp screen_shadow_gl.cpp -o test_screen_shadow_gl -lEGL -lGLESv2
//   EGL_PLATFORM=surfaceless LIBGL_ALWAYS_SOFTWARE=1 ./test_screen_shadow_gl [out_dir] [quick]
#include "screen_shadow_gl.h"
#include "screen_shadow_core.h"

#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GLES3/gl3.h>

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

static void Log(const char* fmt, ...) {
    va_list ap; va_start(ap, fmt); std::vprintf(fmt, ap); va_end(ap); std::printf("\n");
}

// ------------------------------------------------------------------------------------------------
// scene description (world units = metres, z up)
// ------------------------------------------------------------------------------------------------
struct Prim {
    int type;            // 0 box, 1 cylinder (axis z), 2 sphere
    double c[3];         // centre
    double h[3];         // box half extents | cylinder radius,radius,halfHeight | sphere radius,..
    double yaw;          // radians, about z
    int id;
    float col[3];
};
static std::vector<Prim> g_scene;

static void BuildScene() {
    g_scene.clear();
    auto box = [](int id, double x, double y, double z, double hx, double hy, double hz, double yawDeg, float r, float g, float b) {
        g_scene.push_back({0, {x, y, z}, {hx, hy, hz}, yawDeg * M_PI / 180.0, id, {r, g, b}});
    };
    auto cyl = [](int id, double x, double y, double z, double r, double hh, float cr, float cg, float cb) {
        g_scene.push_back({1, {x, y, z}, {r, r, hh}, 0.0, id, {cr, cg, cb}});
    };
    auto sph = [](int id, double x, double y, double z, double r, float cr, float cg, float cb) {
        g_scene.push_back({2, {x, y, z}, {r, r, r}, 0.0, id, {cr, cg, cb}});
    };
    box(1, 14, 28, 6, 6, 5, 6, 0, 0.75f, 0.70f, 0.62f);                 // big building
    box(2, -16, 24, 5, 4, 8, 5, 20, 0.70f, 0.72f, 0.78f);                // L-building arm A
    box(3, -12, 34, 5, 9, 3, 5, 20, 0.70f, 0.72f, 0.78f);                // L-building arm B
    cyl(4, -3.5, 12, 3.5, 0.15, 3.5, 0.55f, 0.55f, 0.58f);               // lamp pole
    cyl(5, 2.0, 8, 0.9, 0.25, 0.9, 0.85f, 0.55f, 0.45f);                 // pedestrian
    box(6, -2.5, 15, 0.7, 0.9, 2.1, 0.7, 30, 0.60f, 0.15f, 0.15f);       // car
    box(7, 7, 18, 1.5, 6, 0.12, 1.5, 0, 0.80f, 0.80f, 0.75f);            // thin wall
    cyl(8, 6, 8, 2.0, 0.2, 2.0, 0.45f, 0.30f, 0.15f);                    // tree trunk
    sph(9, 6, 8, 5.0, 2.2, 0.20f, 0.55f, 0.20f);                         // tree canopy
}

// ------------------------------------------------------------------------------------------------
// ground-truth ray casting
// ------------------------------------------------------------------------------------------------
static bool RayBox(const double o[3], const double d[3], const Prim& p) {
    const double cy = std::cos(-p.yaw), sy = std::sin(-p.yaw);
    const double ro[3] = {o[0] - p.c[0], o[1] - p.c[1], o[2] - p.c[2]};
    const double lo[3] = {cy * ro[0] - sy * ro[1], sy * ro[0] + cy * ro[1], ro[2]};
    const double ld[3] = {cy * d[0] - sy * d[1], sy * d[0] + cy * d[1], d[2]};
    double t0 = 0.0, t1 = 1.0e9;
    for (int a = 0; a < 3; ++a) {
        if (std::fabs(ld[a]) < 1.0e-12) { if (std::fabs(lo[a]) > p.h[a]) return false; continue; }
        double ta = (-p.h[a] - lo[a]) / ld[a], tb = (p.h[a] - lo[a]) / ld[a];
        if (ta > tb) std::swap(ta, tb);
        t0 = std::max(t0, ta); t1 = std::min(t1, tb);
        if (t0 > t1) return false;
    }
    return true;
}
static bool RayCyl(const double o[3], const double d[3], const Prim& p) {
    const double ox = o[0] - p.c[0], oy = o[1] - p.c[1], oz = o[2] - p.c[2], r = p.h[0], hh = p.h[2];
    double tmin = 0.0, tmax = 1.0e9;
    // slab in z
    if (std::fabs(d[2]) < 1.0e-12) { if (std::fabs(oz) > hh) return false; }
    else { double ta = (-hh - oz) / d[2], tb = (hh - oz) / d[2]; if (ta > tb) std::swap(ta, tb); tmin = std::max(tmin, ta); tmax = std::min(tmax, tb); if (tmin > tmax) return false; }
    const double a = d[0] * d[0] + d[1] * d[1], b = 2 * (ox * d[0] + oy * d[1]), c = ox * ox + oy * oy - r * r;
    if (a < 1.0e-12) { if (c > 0) return false; return true; }
    const double disc = b * b - 4 * a * c; if (disc < 0) return false;
    const double s = std::sqrt(disc); double ta = (-b - s) / (2 * a), tb = (-b + s) / (2 * a);
    tmin = std::max(tmin, ta); tmax = std::min(tmax, tb);
    return tmin <= tmax;
}
static bool RaySphere(const double o[3], const double d[3], const Prim& p) {
    const double ox = o[0] - p.c[0], oy = o[1] - p.c[1], oz = o[2] - p.c[2];
    const double b = ox * d[0] + oy * d[1] + oz * d[2], c = ox * ox + oy * oy + oz * oz - p.h[0] * p.h[0];
    const double disc = b * b - c; if (disc < 0) return false;
    const double s = std::sqrt(disc); return (-b + s) >= 0.0;
}
static int Occluded(const double o[3], const double d[3], int skipId) {
    for (const Prim& p : g_scene) {
        if (p.id == skipId) continue;       // every primitive is convex: it cannot shadow its own lit face
        bool hit = p.type == 0 ? RayBox(o, d, p) : (p.type == 1 ? RayCyl(o, d, p) : RaySphere(o, d, p));
        if (hit) return p.id;
    }
    return -1;
}

// ------------------------------------------------------------------------------------------------
// mesh generation
// ------------------------------------------------------------------------------------------------
struct Vtx { float p[3], n[3], c[3], id; };
static std::vector<Vtx> g_mesh;
static void Tri(const Vtx& a, const Vtx& b, const Vtx& c) { g_mesh.push_back(a); g_mesh.push_back(b); g_mesh.push_back(c); }
static void Quad(Vtx a, Vtx b, Vtx c, Vtx d) { Tri(a, b, c); Tri(a, c, d); }

static void AddBoxMesh(const Prim& pr, const double cam[3]) {
    const double cy = std::cos(pr.yaw), sy = std::sin(pr.yaw);
    auto V = [&](double lx, double ly, double lz, double nx, double ny, double nz) {
        Vtx v; const double wx = cy * lx - sy * ly, wy = sy * lx + cy * ly;
        v.p[0] = float(pr.c[0] + wx - cam[0]); v.p[1] = float(pr.c[1] + wy - cam[1]); v.p[2] = float(pr.c[2] + lz - cam[2]);
        v.n[0] = float(cy * nx - sy * ny); v.n[1] = float(sy * nx + cy * ny); v.n[2] = float(nz);
        v.c[0] = pr.col[0]; v.c[1] = pr.col[1]; v.c[2] = pr.col[2]; v.id = float(pr.id); return v; };
    const double x = pr.h[0], y = pr.h[1], z = pr.h[2];
    Quad(V(x,-y,-z,1,0,0), V(x,y,-z,1,0,0), V(x,y,z,1,0,0), V(x,-y,z,1,0,0));
    Quad(V(-x,y,-z,-1,0,0), V(-x,-y,-z,-1,0,0), V(-x,-y,z,-1,0,0), V(-x,y,z,-1,0,0));
    Quad(V(x,y,-z,0,1,0), V(-x,y,-z,0,1,0), V(-x,y,z,0,1,0), V(x,y,z,0,1,0));
    Quad(V(-x,-y,-z,0,-1,0), V(x,-y,-z,0,-1,0), V(x,-y,z,0,-1,0), V(-x,-y,z,0,-1,0));
    Quad(V(-x,-y,z,0,0,1), V(x,-y,z,0,0,1), V(x,y,z,0,0,1), V(-x,y,z,0,0,1));
    Quad(V(-x,y,-z,0,0,-1), V(x,y,-z,0,0,-1), V(x,-y,-z,0,0,-1), V(-x,-y,-z,0,0,-1));
}
static void AddCylMesh(const Prim& pr, const double cam[3]) {
    const int N = 24; const double r = pr.h[0], hh = pr.h[2];
    auto V = [&](double lx, double ly, double lz, double nx, double ny, double nz) {
        Vtx v; v.p[0] = float(pr.c[0] + lx - cam[0]); v.p[1] = float(pr.c[1] + ly - cam[1]); v.p[2] = float(pr.c[2] + lz - cam[2]);
        v.n[0] = float(nx); v.n[1] = float(ny); v.n[2] = float(nz);
        v.c[0] = pr.col[0]; v.c[1] = pr.col[1]; v.c[2] = pr.col[2]; v.id = float(pr.id); return v; };
    for (int i = 0; i < N; ++i) {
        const double a0 = 2 * M_PI * i / N, a1 = 2 * M_PI * (i + 1) / N;
        const double c0 = std::cos(a0), s0 = std::sin(a0), c1 = std::cos(a1), s1 = std::sin(a1);
        Quad(V(r*c0,r*s0,-hh,c0,s0,0), V(r*c1,r*s1,-hh,c1,s1,0), V(r*c1,r*s1,hh,c1,s1,0), V(r*c0,r*s0,hh,c0,s0,0));
        Tri(V(0,0,hh,0,0,1), V(r*c0,r*s0,hh,0,0,1), V(r*c1,r*s1,hh,0,0,1));
        Tri(V(0,0,-hh,0,0,-1), V(r*c1,r*s1,-hh,0,0,-1), V(r*c0,r*s0,-hh,0,0,-1));
    }
}
static void AddSphereMesh(const Prim& pr, const double cam[3]) {
    const int LA = 14, LO = 24; const double r = pr.h[0];
    auto V = [&](double th, double ph) {
        const double nx = std::sin(th) * std::cos(ph), ny = std::sin(th) * std::sin(ph), nz = std::cos(th);
        Vtx v; v.p[0] = float(pr.c[0] + r * nx - cam[0]); v.p[1] = float(pr.c[1] + r * ny - cam[1]); v.p[2] = float(pr.c[2] + r * nz - cam[2]);
        v.n[0] = float(nx); v.n[1] = float(ny); v.n[2] = float(nz);
        v.c[0] = pr.col[0]; v.c[1] = pr.col[1]; v.c[2] = pr.col[2]; v.id = float(pr.id); return v; };
    for (int i = 0; i < LA; ++i)
        for (int j = 0; j < LO; ++j) {
            const double t0 = M_PI * i / LA, t1 = M_PI * (i + 1) / LA, p0 = 2 * M_PI * j / LO, p1 = 2 * M_PI * (j + 1) / LO;
            Quad(V(t0, p0), V(t1, p0), V(t1, p1), V(t0, p1));
        }
}
// Registers every primitive as a geometry caster (what the game side does with DFF triangles).
static void RegisterCasters() {
    ScreenShadow::CasterClear();
    for (const Prim& p : g_scene) {
        g_mesh.clear();
        const double c[3] = {p.c[0], p.c[1], p.c[2]};
        if (p.type == 0) AddBoxMesh(p, c); else if (p.type == 1) AddCylMesh(p, c); else AddSphereMesh(p, c);
        std::vector<float> v; std::vector<uint32_t> idx;
        for (size_t i = 0; i < g_mesh.size(); ++i) { v.push_back(g_mesh[i].p[0]); v.push_back(g_mesh[i].p[1]); v.push_back(g_mesh[i].p[2]); idx.push_back(uint32_t(i)); }
        ScreenShadow::CasterSet(uint32_t(p.id), c, v.data(), int(g_mesh.size()), idx.data(), int(idx.size()));
    }
    g_mesh.clear();
}

static void BuildMesh(const double cam[3]) {
    g_mesh.clear();
    // ground: 0 id
    auto G = [&](double x, double y) { Vtx v; v.p[0] = float(x - cam[0]); v.p[1] = float(y - cam[1]); v.p[2] = float(0.0 - cam[2]);
        v.n[0] = 0; v.n[1] = 0; v.n[2] = 1; v.c[0] = 0.55f; v.c[1] = 0.55f; v.c[2] = 0.52f; v.id = 0; return v; };
    const double S = 150, ox = 2437.3, oy = -1782.1;
    Quad(G(ox-S,oy-S+10), G(ox+S,oy-S+10), G(ox+S,oy+S), G(ox-S,oy+S));
    for (const Prim& p : g_scene) { if (p.type == 0) AddBoxMesh(p, cam); else if (p.type == 1) AddCylMesh(p, cam); else AddSphereMesh(p, cam); }
}

// ------------------------------------------------------------------------------------------------
// GL
// ------------------------------------------------------------------------------------------------
static const char* kSceneVs = R"(#version 300 es
precision highp float;
layout(location=0) in vec3 aPos; layout(location=1) in vec3 aNrm; layout(location=2) in vec3 aCol; layout(location=3) in float aId;
uniform mat4 uVP;
out vec3 vPos; out vec3 vNrm; out vec3 vCol; out float vId;
void main(){ vPos=aPos; vNrm=aNrm; vCol=aCol; vId=aId; gl_Position=uVP*vec4(aPos,1.0); }
)";
static const char* kSceneFs = R"(#version 300 es
precision highp float;
in vec3 vPos; in vec3 vNrm; in vec3 vCol; in float vId;
layout(location=0) out vec4 oColor; layout(location=1) out vec4 oPos; layout(location=2) out vec4 oNrm;
void main(){ oColor=vec4(vCol*0.9,1.0); oPos=vec4(vPos,vId); oNrm=vec4(normalize(vNrm)*0.5+0.5,1.0); }
)";

static GLuint Prog(const char* vs, const char* fs) {
    auto C = [](GLenum t, const char* s) { GLuint sh = glCreateShader(t); glShaderSource(sh, 1, &s, nullptr); glCompileShader(sh);
        GLint ok; glGetShaderiv(sh, GL_COMPILE_STATUS, &ok); if (!ok) { char l[1024]; glGetShaderInfoLog(sh, 1024, nullptr, l); std::printf("shader error: %s\n", l); std::exit(2); } return sh; };
    GLuint p = glCreateProgram(); glAttachShader(p, C(GL_VERTEX_SHADER, vs)); glAttachShader(p, C(GL_FRAGMENT_SHADER, fs)); glLinkProgram(p); return p;
}

static void Perspective(float fovY, float aspect, float n, float f, float* m) {
    std::memset(m, 0, 64); const float t = 1.0f / std::tan(fovY * 0.5f);
    m[0] = t / aspect; m[5] = t; m[10] = -(f + n) / (f - n); m[11] = -1.0f; m[14] = -2.0f * f * n / (f - n);
}
static void LookAtRot(const double eye[3], const double tgt[3], float* m) {   // rotation only (camera-relative)
    double f[3] = {tgt[0]-eye[0], tgt[1]-eye[1], tgt[2]-eye[2]}; double l = std::sqrt(f[0]*f[0]+f[1]*f[1]+f[2]*f[2]); for (double& v : f) v /= l;
    double up[3] = {0,0,1};
    double s[3] = {f[1]*up[2]-f[2]*up[1], f[2]*up[0]-f[0]*up[2], f[0]*up[1]-f[1]*up[0]}; l = std::sqrt(s[0]*s[0]+s[1]*s[1]+s[2]*s[2]); for (double& v : s) v /= l;
    double u[3] = {s[1]*f[2]-s[2]*f[1], s[2]*f[0]-s[0]*f[2], s[0]*f[1]-s[1]*f[0]};
    std::memset(m, 0, 64);
    m[0]=float(s[0]); m[4]=float(s[1]); m[8]=float(s[2]);
    m[1]=float(u[0]); m[5]=float(u[1]); m[9]=float(u[2]);
    m[2]=float(-f[0]); m[6]=float(-f[1]); m[10]=float(-f[2]); m[15]=1.0f;
}
static bool Inverse(const float* m, float* inv) {
    double a[4][8]{}; for (int r = 0; r < 4; ++r) { for (int c = 0; c < 4; ++c) a[r][c] = m[c*4+r]; a[r][r+4] = 1; }
    for (int c = 0; c < 4; ++c) { int p = c; for (int r = c+1; r < 4; ++r) if (std::fabs(a[r][c]) > std::fabs(a[p][c])) p = r;
        if (std::fabs(a[p][c]) < 1e-14) return false; if (p != c) for (int k = 0; k < 8; ++k) std::swap(a[p][k], a[c][k]);
        double d = a[c][c]; for (double& v : a[c]) v /= d; for (int r = 0; r < 4; ++r) if (r != c) { double f = a[r][c]; for (int k = 0; k < 8; ++k) a[r][k] -= f * a[c][k]; } }
    for (int r = 0; r < 4; ++r) for (int c = 0; c < 4; ++c) inv[c*4+r] = float(a[r][c+4]); return true;
}

static void WritePPM(const char* path, const std::vector<unsigned char>& rgb, int w, int h) {
    FILE* f = std::fopen(path, "wb"); if (!f) return; std::fprintf(f, "P6\n%d %d\n255\n", w, h);
    std::fwrite(rgb.data(), 1, rgb.size(), f); std::fclose(f);
}

static long g_occ[16], g_occLeak[16]; static long g_bin[8]; static long g_binAll[8];
struct Result { double leakGround = 0, falseGround = 0, leakObj = 0, falseObj = 0; long gtShadowGround = 0, gtLitGround = 0, gtShadowObj = 0, gtLitObj = 0; };

int main(int argc, char** argv) {
    const char* outDir = argc > 1 ? argv[1] : nullptr;
    const bool quick = argc > 2 && std::strcmp(argv[2], "quick") == 0;
    auto getPlat = (PFNEGLGETPLATFORMDISPLAYEXTPROC)eglGetProcAddress("eglGetPlatformDisplayEXT");
    EGLDisplay dpy = getPlat ? getPlat(EGL_PLATFORM_SURFACELESS_MESA, EGL_DEFAULT_DISPLAY, nullptr) : eglGetDisplay(EGL_DEFAULT_DISPLAY);
    EGLint maj, min; if (!eglInitialize(dpy, &maj, &min)) { std::printf("EGL init failed\n"); return 2; }
    eglBindAPI(EGL_OPENGL_ES_API);
    EGLint ca[] = {EGL_RENDERABLE_TYPE, EGL_OPENGL_ES3_BIT, EGL_SURFACE_TYPE, EGL_PBUFFER_BIT, EGL_NONE}; EGLConfig cfg; EGLint nc;
    eglChooseConfig(dpy, ca, &cfg, 1, &nc); EGLint pb[] = {EGL_WIDTH, 16, EGL_HEIGHT, 16, EGL_NONE};
    EGLSurface surf = eglCreatePbufferSurface(dpy, cfg, pb); EGLint cx[] = {EGL_CONTEXT_CLIENT_VERSION, 3, EGL_NONE};
    EGLContext ctx = eglCreateContext(dpy, cfg, EGL_NO_CONTEXT, cx); eglMakeCurrent(dpy, surf, surf, ctx);
    std::printf("GL: %s | %s\n", glGetString(GL_VERSION), glGetString(GL_RENDERER));

    const int W = quick ? 640 : 1280, H = quick ? 360 : 720;
    BuildScene();
    const GLuint scene = Prog(kSceneVs, kSceneFs);
    GLuint tex[3], depth, fbo; glGenTextures(3, tex); glGenTextures(1, &depth); glGenFramebuffers(1, &fbo);
    auto alloc = [&](GLuint t, GLint ifmt, GLenum fmt, GLenum type) { glBindTexture(GL_TEXTURE_2D, t); glTexImage2D(GL_TEXTURE_2D, 0, ifmt, W, H, 0, fmt, type, nullptr);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST); glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE); glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE); };
    alloc(tex[0], GL_RGBA8, GL_RGBA, GL_UNSIGNED_BYTE);
    alloc(tex[1], GL_RGBA32F, GL_RGBA, GL_FLOAT);
    alloc(tex[2], GL_RGBA8, GL_RGBA, GL_UNSIGNED_BYTE);
    const int depthBits = 24;
    alloc(depth, GL_DEPTH_COMPONENT24, GL_DEPTH_COMPONENT, GL_UNSIGNED_INT);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);
    for (int i = 0; i < 3; ++i) glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0 + i, GL_TEXTURE_2D, tex[i], 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, depth, 0);
    const GLenum all3[3] = {GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1, GL_COLOR_ATTACHMENT2};
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) { std::printf("scene FBO incomplete\n"); return 2; }

    GLuint vao, vbo; glGenVertexArrays(1, &vao); glGenBuffers(1, &vbo);

    // far world offset: exercises the camera-relative path + texel snapping at GTA-like coordinates
    const double kOffset[3] = {2437.3, -1782.1, 11.0};
    struct Cam { const char* name; double eye[3]; double tgt[3]; };
    const Cam cams[] = {
        {"street", {0, -6, 2.2}, {2, 14, 1.8}},
        {"high",   {-8, -4, 9.0}, {2, 16, 1.0}},
        // Camera looks AWAY from the buildings: every caster is off screen, only their shadows are in view.
        {"away",   {-2, 4, 2.2}, {-2, -30, 1.5}},
    };
    const bool useCasters = std::getenv("SS_CASTERS") != nullptr && std::atoi(std::getenv("SS_CASTERS")) != 0;
    const double azs[] = {0, 45, 90, 135, 180, 225, 270, 315};
    const double els[] = {28, 50};
    struct Variant { const char* name; float thickness; int density; };
    const Variant variants[] = {{"thick1.00 d3", 1.0f, 3}};

    int totalFail = 0;
    Variant override_{"env", 1.0f, 3};
    const bool useEnv = std::getenv("SS_THICK") != nullptr;
    if (useEnv) override_.thickness = float(std::atof(std::getenv("SS_THICK")));
    for (const Variant& varIn : variants) {
        const Variant& var = useEnv ? override_ : varIn;
        if (quick && var.density != 3) continue;
        std::printf("\n=== %s ===\n", var.name);
        std::printf("%-7s %4s %4s | %-8s ground: leak%% false%% | objects: leak%% false%%\n", "cam", "az", "el", "");
        double sumLeakG = 0, sumFalseG = 0, sumLeakO = 0, sumFalseO = 0; int runs = 0;
        for (const Cam& cm : cams) {
            const bool away = std::strcmp(cm.name, "away") == 0;
            const double awayEls[] = {12, 20};
            for (double el : (away ? awayEls : els)) for (double az : azs) {
                if (quick && (az != 0 && az != 90 && az != 180 && az != 270)) continue;
                if (away && !(az == 0 || az == 45 || az == 315)) continue;
                double eye[3] = {cm.eye[0] + kOffset[0], cm.eye[1] + kOffset[1], cm.eye[2] + kOffset[2]};
                double tgt[3] = {cm.tgt[0] + kOffset[0], cm.tgt[1] + kOffset[1], cm.tgt[2] + kOffset[2]};
                // scene lives in world space = local + offset; shift prims once per run
                std::vector<Prim> local = g_scene;
                for (Prim& p : g_scene) { p.c[0] += kOffset[0]; p.c[1] += kOffset[1]; p.c[2] += kOffset[2]; }
                // ground sits at z = kOffset[2] ... keep ground at world z = kOffset[2]
                if (useCasters) RegisterCasters();
                BuildMesh(eye);
                // fix ground height: BuildMesh used z=0; move it to the offset plane
                for (size_t i = 0; i < 6; ++i) g_mesh[i].p[2] = float(kOffset[2] - eye[2]);

                const double ang = az * M_PI / 180.0, e = el * M_PI / 180.0;
                // toSun azimuth measured from +y (camera forward) clockwise: 0 = sun in front
                float toSun[3] = {float(std::sin(ang) * std::cos(e)), float(std::cos(ang) * std::cos(e)), float(std::sin(e))};

                float proj[16], viewRel[16], vp[16], invVp[16];
                Perspective(60.0f * 3.14159265f / 180.0f, float(W) / H, 0.3f, 600.0f, proj);
                LookAtRot(eye, tgt, viewRel);
                ShadowMap::Mul4(proj, viewRel, vp);
                if (!Inverse(vp, invVp)) return 2;

                glBindFramebuffer(GL_FRAMEBUFFER, fbo);
                glDrawBuffers(3, all3);
                glViewport(0, 0, W, H);
                glEnable(GL_DEPTH_TEST); glDepthFunc(GL_LESS); glDepthMask(GL_TRUE); glDisable(GL_BLEND); glDisable(GL_CULL_FACE); glDisable(GL_SCISSOR_TEST);
                glColorMask(1, 1, 1, 1);
                const float clr[4] = {0.55f, 0.70f, 0.95f, 1.0f}, zero4[4] = {0, 0, 0, 0}, one = 1.0f;
                glClearBufferfv(GL_COLOR, 0, clr); glClearBufferfv(GL_COLOR, 1, zero4); glClearBufferfv(GL_COLOR, 2, zero4); glClearBufferfv(GL_DEPTH, 0, &one);
                glUseProgram(scene);
                glUniformMatrix4fv(glGetUniformLocation(scene, "uVP"), 1, GL_FALSE, vp);
                glBindVertexArray(vao); glBindBuffer(GL_ARRAY_BUFFER, vbo);
                glBufferData(GL_ARRAY_BUFFER, g_mesh.size() * sizeof(Vtx), g_mesh.data(), GL_STREAM_DRAW);
                for (int i = 0; i < 4; ++i) glEnableVertexAttribArray(i);
                glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vtx), (void*)offsetof(Vtx, p));
                glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vtx), (void*)offsetof(Vtx, n));
                glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(Vtx), (void*)offsetof(Vtx, c));
                glVertexAttribPointer(3, 1, GL_FLOAT, GL_FALSE, sizeof(Vtx), (void*)offsetof(Vtx, id));
                glDrawArrays(GL_TRIANGLES, 0, GLsizei(g_mesh.size()));
                for (int i = 0; i < 4; ++i) glDisableVertexAttribArray(i);

                // read the G-buffers before shading
                std::vector<float> pos(W * H * 4); std::vector<unsigned char> nrm(W * H * 4), color(W * H * 4);
                glReadBuffer(GL_COLOR_ATTACHMENT1); glReadPixels(0, 0, W, H, GL_RGBA, GL_FLOAT, pos.data());
                glReadBuffer(GL_COLOR_ATTACHMENT2); glReadPixels(0, 0, W, H, GL_RGBA, GL_UNSIGNED_BYTE, nrm.data());
                glReadBuffer(GL_COLOR_ATTACHMENT0); glReadPixels(0, 0, W, H, GL_RGBA, GL_UNSIGNED_BYTE, color.data());

                // ---- the module under test: debug=1 (raw map visibility, no contact, no self shading)
                ScreenShadow::Frame fr; fr.depthTex = depth; fr.width = W; fr.height = H; fr.targetFbo = fbo;
                std::memcpy(fr.proj, proj, 64); std::memcpy(fr.vp, vp, 64); std::memcpy(fr.invVp, invVp, 64);
                for (int k = 0; k < 3; ++k) { fr.camWorld[k] = eye[k]; fr.toSun[k] = toSun[k]; }
                fr.sunStrength = 1.0f; fr.depthBits = depthBits;
                ScreenShadow::Params pr; pr.thickness = var.thickness; pr.density = var.density; pr.debug = 1; pr.contact = 0; pr.range = 70.0f; pr.casters = useCasters ? 1 : 0;
                const GLenum only0[3] = {GL_COLOR_ATTACHMENT0, GL_NONE, GL_NONE};
                glDrawBuffers(3, only0);
                static bool logged = false;
                if (!ScreenShadow::Render(fr, pr, logged ? nullptr : Log)) { std::printf("Render failed\n"); return 2; }
                logged = true;
                std::vector<unsigned char> vis(W * H * 4);
                glReadBuffer(GL_COLOR_ATTACHMENT0); glReadPixels(0, 0, W, H, GL_RGBA, GL_UNSIGNED_BYTE, vis.data());

                // ---- compare against the analytic ground truth
                Result res; std::vector<unsigned char> cmp(W * H * 3, 0);
                for (int y = 0; y < H; ++y) for (int x = 0; x < W; ++x) {
                    const int i = (y * W + x);
                    const float id = pos[i * 4 + 3]; const bool hit = nrm[i * 4 + 3] > 0 && (pos[i*4] != 0 || pos[i*4+1] != 0 || pos[i*4+2] != 0);
                    unsigned char* o = &cmp[((H - 1 - y) * W + x) * 3];
                    if (!hit) { o[0] = 140; o[1] = 180; o[2] = 240; continue; }
                    const double n[3] = {nrm[i*4]/255.0*2-1, nrm[i*4+1]/255.0*2-1, nrm[i*4+2]/255.0*2-1};
                    const double ndl = n[0]*toSun[0] + n[1]*toSun[1] + n[2]*toSun[2];
                    const double org[3] = {pos[i*4] + eye[0] + n[0]*0.03, pos[i*4+1] + eye[1] + n[1]*0.03, pos[i*4+2] + eye[2] + n[2]*0.03};
                    const double dir[3] = {toSun[0], toSun[1], toSun[2]};
                    const int occId = Occluded(org, dir, int(id + 0.5f) == 0 ? -1 : int(id + 0.5f)); const bool gtShadow = occId >= 0;
                    const bool pipeShadow = vis[i * 4] < 128;
                    const bool ground = id < 0.5f;
                    const bool facing = ndl > 0.2;
                    unsigned char r, g, b;
                    if (!facing) { r = g = b = 60; }
                    else if (gtShadow && pipeShadow) { r = 20; g = 20; b = 20; }
                    else if (gtShadow && !pipeShadow) { r = 255; g = 40; b = 40; }
                    else if (!gtShadow && pipeShadow) { r = 40; g = 80; b = 255; }
                    else { r = 235; g = 235; b = 200; }
                    o[0] = r; o[1] = g; o[2] = b;
                    if (!facing) continue;
                    if (ground) { if (gtShadow) { ++g_occ[occId]; if (!pipeShadow) ++g_occLeak[occId]; ++res.gtShadowGround; const double dd = std::sqrt(pos[i*4]*pos[i*4]+pos[i*4+1]*pos[i*4+1]+pos[i*4+2]*pos[i*4+2]); int bi = std::min(7, int(dd / 10)); ++g_binAll[bi]; if (!pipeShadow) { res.leakGround += 1; ++g_bin[bi]; } } else { ++res.gtLitGround; if (pipeShadow) res.falseGround += 1; } }
                    else { if (gtShadow) { ++res.gtShadowObj; if (!pipeShadow) res.leakObj += 1; } else { ++res.gtLitObj; if (pipeShadow) res.falseObj += 1; } }
                }
                auto pct = [](double a, long n) { return n > 0 ? 100.0 * a / n : 0.0; };
                const double lg = pct(res.leakGround, res.gtShadowGround), fg = pct(res.falseGround, res.gtLitGround);
                const double lo = pct(res.leakObj, res.gtShadowObj), fo = pct(res.falseObj, res.gtLitObj);
                std::printf("%-7s %4.0f %4.0f | %-8s        %5.1f %5.1f |          %5.1f %5.1f   (gt shadow px: ground %ld obj %ld)\n",
                            cm.name, az, el, "", lg, fg, lo, fo, res.gtShadowGround, res.gtShadowObj);
                sumLeakG += lg; sumFalseG += fg; sumLeakO += lo; sumFalseO += fo; ++runs;

                if (outDir && (useEnv || (var.thickness > 0.9f && var.thickness < 1.1f)) && var.density == 3 && (az == 0 || az == 90 || az == 180 || az == 270) && (el == 28 || (away && el == 12 && az == 0))) {
                    char path[512];
                    std::snprintf(path, sizeof(path), "%s/cmp_%s_az%03.0f_el%02.0f.ppm", outDir, cm.name, az, el); WritePPM(path, cmp, W, H);
                    // final shaded image with the production settings
                    ScreenShadow::Params fin; fin.thickness = var.thickness; fin.density = var.density; fin.debug = 0; fin.contact = 1; fin.range = 70.0f; fin.casters = useCasters ? 1 : 0;
                    // restore unshadowed colour first
                    glDrawBuffers(3, all3);
                    glEnable(GL_DEPTH_TEST); glDepthMask(GL_TRUE); glColorMask(1, 1, 1, 1); glDisable(GL_BLEND); glViewport(0, 0, W, H);
                    glUseProgram(scene); glClearBufferfv(GL_COLOR, 0, clr); glClearBufferfv(GL_DEPTH, 0, &one);
                    glBindVertexArray(vao); glBindBuffer(GL_ARRAY_BUFFER, vbo);
                    for (int k = 0; k < 4; ++k) glEnableVertexAttribArray(k);
                    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vtx), (void*)offsetof(Vtx, p));
                    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vtx), (void*)offsetof(Vtx, n));
                    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(Vtx), (void*)offsetof(Vtx, c));
                    glVertexAttribPointer(3, 1, GL_FLOAT, GL_FALSE, sizeof(Vtx), (void*)offsetof(Vtx, id));
                    glEnable(GL_DEPTH_TEST); glDepthMask(GL_TRUE); glColorMask(1, 1, 1, 1); glDisable(GL_BLEND);
                    glDrawArrays(GL_TRIANGLES, 0, GLsizei(g_mesh.size()));
                    for (int k = 0; k < 4; ++k) glDisableVertexAttribArray(k);
                    glDrawBuffers(3, only0);
                    { std::vector<unsigned char> pre(W * H * 4); glReadBuffer(GL_COLOR_ATTACHMENT0); glReadPixels(0, 0, W, H, GL_RGBA, GL_UNSIGNED_BYTE, pre.data());
                      std::vector<unsigned char> rgb0(W * H * 3);
                      for (int y = 0; y < H; ++y) for (int x = 0; x < W; ++x) for (int c = 0; c < 3; ++c) rgb0[((H-1-y)*W + x)*3 + c] = pre[(y*W + x)*4 + c];
                      std::snprintf(path, sizeof(path), "%s/pre_%s_az%03.0f_el%02.0f.ppm", outDir, cm.name, az, el); WritePPM(path, rgb0, W, H); }
                    ScreenShadow::Render(fr, fin, nullptr);
                    std::vector<unsigned char> shaded(W * H * 4); glReadPixels(0, 0, W, H, GL_RGBA, GL_UNSIGNED_BYTE, shaded.data());
                    std::vector<unsigned char> rgb(W * H * 3);
                    for (int y = 0; y < H; ++y) for (int x = 0; x < W; ++x) for (int c = 0; c < 3; ++c) rgb[((H-1-y)*W + x)*3 + c] = shaded[(y*W + x)*4 + c];
                    std::snprintf(path, sizeof(path), "%s/final_%s_az%03.0f_el%02.0f.ppm", outDir, cm.name, az, el); WritePPM(path, rgb, W, H);
                }
                g_scene = local;
            }
        }
        for (int o = 1; o < 10; ++o) std::printf("   ground shadow cast by prim %d: leak %5.1f%% of %ld\n", o, g_occ[o] ? 100.0*g_occLeak[o]/g_occ[o] : 0.0, g_occ[o]);
        std::memset(g_occ,0,sizeof(g_occ)); std::memset(g_occLeak,0,sizeof(g_occLeak));
        for (int b = 0; b < 8; ++b) std::printf("   ground leak by distance %2d-%2dm: %5.1f%% of %ld\n", b*10, b*10+10, g_binAll[b] ? 100.0*g_bin[b]/g_binAll[b] : 0.0, g_binAll[b]);
        std::memset(g_bin,0,sizeof(g_bin)); std::memset(g_binAll,0,sizeof(g_binAll));
        std::printf("--- mean over %d runs: ground leak %.1f%% false %.1f%% | objects leak %.1f%% false %.1f%%\n", runs,
                    sumLeakG / runs, sumFalseG / runs, sumLeakO / runs, sumFalseO / runs);
    }
    return totalFail;
}
