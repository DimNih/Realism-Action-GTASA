#pragma once

#include <GLES3/gl3.h>

#include <cstdint>

// Sun shadows for EVERYTHING the camera can see (buildings, props, vegetation, vehicles, peds),
// receiving on EVERYTHING the camera can see (ground, walls, cars, peds), with no geometry replay.
//
//  1. The scene depth the light pass already copies is re-projected into two sun-space depth maps
//     (cascades): one GL_POINTS vertex per depth-grid cell, positions rebuilt from the depth, drawn
//     with the sun's orthographic matrix. Whatever the camera sees becomes a shadow caster, with its
//     real shape, including animated peds, alpha-tested leaves and anything a mod adds.
//  2. A full-screen receiver pass reads the depth again, rebuilds each pixel, compares it with the
//     sun maps (contact-hardening PCF), adds short screen-space contact shadows and a
//     surface-faces-away term, and multiplies the scene colour. Because it is per visible pixel it
//     can never bleed through walls, cars or characters: it lands exactly on the surface drawn.
//
// GL contract: the caller owns/restores blend, depth/stencil/scissor/cull state, colour+depth masks,
// viewport, program, VAO, framebuffer bindings and texture unit 0 (+sampler 0). Render() itself saves
// and restores texture units 1-2 and the depth range, which are the other things it touches.
namespace ScreenShadow {

using LogFn = void (*)(const char*, ...);

struct Params {
    float strength = 1.0f;      // 0..1.5 overall shadow darkness
    float range = 70.0f;        // metres from the camera that receive shadows
    float soft = 1.0f;          // penumbra scale
    float ambient = 0.42f;      // light kept inside a full shadow (0 = black, 1 = none)
    float selfShade = 0.65f;    // darkening of surfaces that face away from the sun
    float thickness = 1.0f;     // assumed solid depth behind each visible surface, metres
    int mapSize = 1536;         // texels per cascade side
    int density = 3;            // depth-grid stride used to build the maps (1 = every pixel)
    int contact = 1;            // screen-space contact shadows
    int quality = 1;            // 0 = fixed PCF, 1 = contact-hardening PCSS
    int debug = 0;              // 1 = map visibility, 2 = cascades, 3 = normals, 4 = contact only
    int casters = 1;            // 1 = also draw the registered geometry casters (see CasterSet)
    int splat = 1;              // 1 = also build the maps from the visible depth (sparse grid: noisy on thin/moving things)
};

struct Frame {
    GLuint depthTex = 0;        // copy of the scene depth: NEAREST, compare mode NONE
    int width = 0, height = 0;  // size of depthTex (== scene viewport size)
    int vpX = 0, vpY = 0;       // origin of the scene viewport inside the target framebuffer
    GLuint targetFbo = 0;       // framebuffer the shadow multiplier is drawn into
    float proj[16] = {};        // projection (column-major)
    float vp[16] = {};          // projection * viewRel
    float invVp[16] = {};       // inverse of vp: clip -> camera-relative world
    double camWorld[3] = {};    // camera position in the world
    float toSun[3] = {0, 0, 1}; // unit vector toward the sun
    float sunStrength = 1.0f;   // 0..1 (dawn/dusk/rain already folded in)
    int depthBits = 24;         // precision of depthTex, used to bound the bias
};

struct Stats {
    int pointsPerDraw = 0;
    int splatTexels = 0;
    float radius0 = 0, radius1 = 0, texel0 = 0, texel1 = 0;
    int mapSize = 0;
    int depthFormat = -1;
    int casterDraws = 0;        // geometry caster draw calls issued last frame (both cascades)
    int casterTris = 0;         // triangles those draws contained
};

bool Render(const Frame& frame, const Params& params, LogFn log);

// ---- Geometry casters -------------------------------------------------------------------------
// The depth-built maps only know what the camera sees right now, so a building or pole that is off
// screen (or whose far side is hidden) casts nothing and shadows pop in/out while the camera moves.
// Registered casters are real model triangles (taken from the DFF on the game thread) that are drawn
// straight into both sun maps, so their shadows depend only on the world and the sun.
//   key      : any stable id for the caster (re-using a key replaces the mesh)
//   origin   : world position the vertices are relative to (double: GTA coordinates reach thousands)
//   verts    : vertexCount * 3 floats relative to `origin`
//   indices  : indexCount (multiple of 3) triangle indices
// Thread safe: CasterSet/Remove/Clear may be called from the game thread, Render() runs on the GL thread.
void CasterSet(uint32_t key, const double origin[3], const float* verts, int vertexCount, const uint32_t* indices, int indexCount);
void CasterRemove(uint32_t key);
void CasterClear();
int CasterCount();
bool IsReady();
void Reset();                   // forget GL objects (context was lost)
const Stats& GetStats();

} // namespace ScreenShadow
