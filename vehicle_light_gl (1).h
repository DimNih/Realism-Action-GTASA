#pragma once

namespace VehicleLightGL {

// Twelve entries cover eight headlamps plus a small rear-glow budget.
constexpr int kMaxLights = 12;
constexpr int kVec4PerLight = 9;
using LogFn = void (*)(const char*, ...);

// Nine consecutive vec4 values are uploaded directly to the GLSL uniform array.
struct Light {
    float posRange[4] = {};  // camera-relative xyz (filled by the GL side), influence radius
    float fwdKind[4] = {};   // normalized direction xyz, 1 for spot / 0 for point (rear glow)
    float rightYaw[4] = {};  // orthonormal lamp right xyz, cosine of OUTER cone angle (informational)
    float upGain[4] = {};    // orthonormal lamp up xyz, energy multiplier (night factor * lamp fade)
    float boxC[4] = {};      // unused by shader, w = cosine of INNER cone angle (informational)
    float boxE[4] = {};      // bbox half extents xyz (scissor helper), w reserved
    float color[4] = {};     // linear RGB, w = volumetric (air scattering) weight, 0 for point lights
    // Source-vehicle box expressed in THIS lamp's local frame (right, forward, up), metres.
    // The shader never lets the source vehicle shadow, light or hide its own lamp.
    float selfBoxXY[4] = {}; // xMin, xMax, yMin, yMax
    float selfBoxZ[4] = {};  // zMin, zMax, reserved, reserved
};

// Runtime tunables. Defaults are used when VehicleLight.ini does not exist.
// VehicleLight.ini lives next to VehicleLight.log (AML android data path) and is
// re-read every few seconds, so values can be tuned without restarting the game.
//   intensity=0.72  surface light strength
//   volumetric=0    optional air-beam strength (0 disables the extra pass)
//   range=32        headlight reach in metres
//   albedo=1        1: light multiplies the surface colour, 0: plain additive
//   shadow=1        screen-space occlusion march
//   rear=1          red tail-light glow
//   bloom=0.30      distance headlamp glare (sun-like, soft). 0 disables
//   white=0.50      white lift: whiter road pool (vehiclelights128-like) + whiter beam core. 0 disables
//   bounce=1.00     close-range / upper-facade reflection (light spilling on walls and buildings)
//   debug=0         1: world grid + cone footprint overlay
struct Settings {
    float intensity = 0.72f;
    float volumetric = 0.0f;
    float range = 32.0f;
    int albedo = 1;
    int shadow = 1;
    int rear = 1;
    float bloom = 0.30f;
    float white = 0.50f;
    float bounce = 1.0f;
    int debug = 0;
};
Settings& GetSettings();

// Install EGL/GLES entry-point hooks. Safe to retry if the libraries are not ready yet.
void Install(LogFn log);
// Publish the lights of the current frame. `cameraWorld` is the game camera position (xyz).
void Publish(const Light* lights, int count, const float* cameraWorld);
void Render();
bool IsInstalled();
bool IsActive();

// Full-screen triangle. Fragment shader reconstructs position from the copied scene depth.
constexpr const char* kVertexShader = R"GLSL(#version 300 es
precision highp float;
void main() {
    vec2 p = vec2(float((gl_VertexID << 1) & 2), float(gl_VertexID & 2));
    gl_Position = vec4(p * 2.0 - 1.0, 0.0, 1.0);
}
)GLSL";

// uMode 0 : surface lighting, blended as dst * (1 + L) so the surface keeps its own colour.
// uMode 1 : additive volumetric headlight shafts.
// uMode 2 : debug overlay.
// uMode 3 : sun-like distance glare around camera-facing headlamps (additive).
// uMode 4 : white lift (additive): whiter road pool + whiter beam core, independent of albedo.
constexpr const char* kFragmentShader = R"GLSL(#version 300 es
precision highp float;
precision highp int;
layout(location = 0) out vec4 outColor;
uniform sampler2D uDepth;
uniform mat4 uInvVP;
uniform mat4 uVP;
uniform vec2 uInvRes;
uniform vec2 uOrigin;
uniform vec3 uCam;
uniform int uCount;
uniform int uMode;
uniform int uShadow;
uniform float uGain;
uniform float uVolGain;
uniform float uBloomGain;
uniform float uMaxDist;
uniform float uWhiteGain;
uniform float uBounceGain;
uniform vec4 uL[108];

vec3 reconstruct(vec2 uv, float depth) {
    vec4 clip = vec4(uv * 2.0 - 1.0, depth * 2.0 - 1.0, 1.0);
    vec4 p = uInvVP * clip;
    return p.xyz / (abs(p.w) < 1.0e-6 ? 1.0e-6 : p.w);
}

// True when world point q (camera-relative) lies inside the source vehicle of light b.
bool insideSelf(vec3 q, int b, float margin) {
    vec4 pr = uL[b];
    vec4 fk = uL[b + 1];
    vec4 ry = uL[b + 2];
    vec4 ug = uL[b + 3];
    vec4 b0 = uL[b + 7];
    vec4 b1 = uL[b + 8];
    vec3 d = q - pr.xyz;
    float x = dot(d, ry.xyz);
    float y = dot(d, fk.xyz);
    float z = dot(d, ug.xyz);
    return x > b0.x - margin && x < b0.y + margin &&
           y > b0.z - margin && y < b0.w + margin &&
           z > b1.x - margin && z < b1.y + margin;
}

// Screen-space occlusion between a receiver and a light. The SOURCE VEHICLE is never an
// occluder: with the camera behind the car its body covers the lamp on screen and used to
// darken everything behind it (ghost silhouette) and wipe out lighting next to walls.
float occlusion(vec3 start, vec3 lightPos, int steps, int b) {
    vec3 delta = lightPos - start;
    float visible = 1.0;
    for (int s = 1; s <= 4; ++s) {
        if (s > steps) break;
        float t = float(s) / float(steps + 1);
        vec3 q = start + delta * t;
        vec4 clip = uVP * vec4(q, 1.0);
        if (clip.w <= 0.05) continue;
        vec2 uv = clip.xy / clip.w * 0.5 + 0.5;
        if (any(lessThan(uv, vec2(0.003))) || any(greaterThan(uv, vec2(0.997)))) continue;
        float sceneDepth = texture(uDepth, uv).r;
        if (sceneDepth >= 0.999999) continue;
        vec3 sp = reconstruct(uv, sceneDepth);
        if (insideSelf(sp, b, 0.12)) continue;                 // own vehicle: transparent
        if (distance(sp, lightPos) < 1.4) continue;            // geometry hugging the lamp (driver, bumper, rider)
        float sceneDist = length(sp);
        float rayDist = length(q);
        float diff = rayDist - sceneDist;                      // > 0: ray point is behind the visible surface
        float bias = 0.20 + 0.012 * rayDist;
        float thick = 5.0 + 0.05 * rayDist;
        float blocked = smoothstep(bias, bias * 2.2, diff) * (1.0 - smoothstep(thick, thick * 1.6, diff));
        visible *= 1.0 - 0.82 * blocked;
    }
    return visible;
}

// Elliptical low-beam shape: wide horizontally, shorter below, long upward lobe so that
// facades and upper floors still receive light. Returns 0..1; hot = bright core weight,
// upF = how far above the lamp axis the receiver is (0..1).
float beamShape(vec3 dirFromLight, vec3 F, vec3 R, vec3 U, out float hot, out float upF) {
    hot = 0.0;
    upF = 0.0;
    float f = dot(dirFromLight, F);
    if (f <= 0.03) return 0.0;
    float inv = 1.0 / f;
    float th = dot(dirFromLight, R) * inv;
    float tv = dot(dirFromLight, U) * inv;
    float vScale = tv >= 0.0 ? 2.1 : 1.35;
    float e = length(vec2(th / 1.18, tv / vScale));
    hot = exp(-e * e * 6.0);
    upF = smoothstep(0.10, 0.80, max(tv, 0.0));
    return 1.0 - smoothstep(0.18, 1.0, e);
}

// Strong near the lamp, clearly fainter far away.
float distanceAtten(float d, float range) {
    float window = 1.0 - smoothstep(range * 0.45, range, d);
    float falloff = 1.0 / (0.36 + 0.06 * d + 0.020 * d * d);
    return window * falloff;
}

vec3 surfaceNormal(vec2 uv, float depth, vec3 p) {
    vec2 texel = uInvRes * 1.5;      // wider baseline: less depth-quantisation noise (blocky patches)
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
    // Roads and walls are flat in this game: snapping removes the stair-step patches
    // that depth quantisation otherwise paints into the light.
    float gs = smoothstep(0.70, 0.90, normal.z);
    normal = normalize(mix(normal, vec3(0.0, 0.0, 1.0), gs * 0.9));
    float ws = 1.0 - smoothstep(0.18, 0.38, abs(normal.z));
    normal = normalize(vec3(normal.xy, normal.z * (1.0 - ws * 0.8)));
    return normal;
}

// Accumulates the lighting of every lamp on one receiver.
//   mulL   : multiplies the scene colour (dst * (1 + mulL))
//   whiteL : additive white lift (whiter pool / beam core)
void surfaceLight(vec3 p, vec3 normal, bool wantWhite, out vec3 mulL, out vec3 whiteL) {
    mulL = vec3(0.0);
    whiteL = vec3(0.0);
    vec3 viewDir = normalize(-p);
    float groundRecv = smoothstep(0.30, 0.88, normal.z);
    float facadeRecv = 1.0 - smoothstep(0.35, 0.80, abs(normal.z));
    for (int i = 0; i < 12; ++i) {
        if (i >= uCount) break;
        int b = i * 9;
        vec4 pr = uL[b];
        vec4 fk = uL[b + 1];
        vec4 ry = uL[b + 2];
        vec4 ug = uL[b + 3];
        vec4 col = uL[b + 6];
        vec4 b1 = uL[b + 8];
        if (ug.w <= 0.001) continue;
        vec3 toLight = pr.xyz - p;
        float d = length(toLight);
        if (d <= 0.05 || d >= pr.w) continue;
        if (insideSelf(p, b, 0.06)) continue;                  // never light the own body
        vec3 L = toLight / d;
        float ndl = dot(normal, L);

        if (fk.w < 0.5) {
            // Rear glow: red, backward hemisphere only, short range.
            if (wantWhite) continue;
            float back = smoothstep(-0.10, 0.60, dot(-L, fk.xyz));
            float wrapR = clamp((ndl + 0.5) / 1.5, 0.0, 1.0);
            float att = (1.0 - smoothstep(pr.w * 0.5, pr.w, d)) / (0.5 + 0.35 * d + 0.12 * d * d);
            float pw = att * back * wrapR * ug.w * smoothstep(0.02, 0.25, d);
            if (pw < 0.003) continue;
            mulL += col.rgb * pw;
            continue;
        }

        vec3 F = fk.xyz;
        vec3 R = ry.xyz;
        vec3 U = ug.xyz;
        float hot, upF;
        float shape = beamShape(-L, F, R, U, hot, upF);
        float wrap = clamp((ndl + 0.55) / 1.55, 0.0, 1.0);
        float atten = distanceAtten(d, pr.w) * smoothstep(0.02, 0.18, d);
        float recvGain = mix(0.80, 1.10, groundRecv);
        float direct = atten * shape * (0.55 + 0.65 * hot) * mix(1.0, 0.55, upF) * wrap * recvGain;

        // --- reflection / spill -------------------------------------------------------------
        // (a) near field: a lamp touching a wall still lights the wall and its surroundings
        // (b) road-pool bounce: light that hit the road ahead climbs facades and upper floors
        float fa = dot(-L, F);
        float hemi = smoothstep(-0.30, 0.45, fa);
        float nearB = hemi * (1.0 / (1.0 + 0.28 * d * d)) * (1.0 - smoothstep(5.0, 9.5, d)) *
                      clamp(ndl * 0.5 + 0.5, 0.0, 1.0);
        vec3 Fh = normalize(vec3(F.xy, 0.0) + vec3(1.0e-4, 0.0, 0.0));
        float gh = clamp(-b1.x, 0.3, 1.6);
        vec3 pc = pr.xyz + Fh * 7.0;
        pc.z -= gh;
        vec3 toC = pc - p;
        float dC = max(length(toC), 0.05);
        float ahead = dot(p - pr.xyz, Fh);
        float side = abs(dot(p - pr.xyz, R));
        float poolB = facadeRecv * clamp(dot(normal, toC / dC) * 0.65 + 0.35, 0.0, 1.0) *
                      (1.0 / (1.0 + 0.05 * dC * dC)) * (1.0 - smoothstep(10.0, 17.0, dC)) *
                      smoothstep(-1.0, 2.5, ahead) * (1.0 - smoothstep(6.0, 14.0, side));
        float bounce = (nearB * 0.30 + poolB * 0.34) * uBounceGain;

        if (direct + bounce < 0.004) continue;
        float vis = 1.0;
        if (uShadow != 0) {
            float sm = smoothstep(1.0, 3.0, d);
            if (sm > 0.0) vis = mix(1.0, occlusion(p + normal * 0.08, pr.xyz, 3, b), sm);
        }
        float bounceVis = mix(1.0, vis, 0.6);

        // Core is whiter, edges slightly warmer: not a flat tint.
        vec3 tint = mix(vec3(1.0, 0.93, 0.82), col.rgb, 0.35 + 0.65 * hot);
        mulL += tint * ug.w * direct * vis;
        mulL += vec3(1.0, 0.93, 0.82) * ug.w * bounce * bounceVis;

        if (wantWhite) {
            vec3 rel = p - pr.xyz;
            float ly = dot(rel, F);
            float lx = dot(rel, R);
            float halfW = 0.9 + 0.13 * max(ly, 0.0);
            float lat = exp(-(lx * lx) / (halfW * halfW));
            float along = smoothstep(0.6, 3.0, ly) * (1.0 - smoothstep(13.0, 26.0, ly));
            float pool = lat * along;
            float w = direct * vis * (0.20 + 0.90 * groundRecv * pool);
            whiteL += vec3(1.0, 0.995, 0.98) * ug.w * w;
        } else {
            vec3 halfDir = normalize(L + viewDir);
            float spec = pow(max(dot(normal, halfDir), 0.0), 34.0);
            mulL += col.rgb * spec * atten * shape * recvGain * 0.025 * ug.w;
        }
    }
}

void main() {
    vec2 uv = (gl_FragCoord.xy - uOrigin) * uInvRes;
    float depth = texture(uDepth, uv).r;
    bool isSky = depth >= 0.999999;

    if (uMode == 0 || uMode == 4) {
        if (isSky) discard;
        vec3 p = reconstruct(uv, depth);
        vec3 normal = surfaceNormal(uv, depth, p);
        vec3 mulL, whiteL;
        surfaceLight(p, normal, uMode == 4, mulL, whiteL);
        if (uMode == 0) {
            vec3 illum = mulL * uGain;
            // Soft shoulder instead of a hard clamp: near surfaces get clearly brighter
            // without turning into a flat clipped white disc.
            illum = 1.35 * (1.0 - exp(-illum / 1.35));
            if (max(illum.r, max(illum.g, illum.b)) < 0.003) discard;
            outColor = vec4(illum, 0.0);
        } else {
            vec3 w = whiteL * uWhiteGain;
            w = 0.34 * (1.0 - exp(-w / 0.34));
            if (max(w.r, max(w.g, w.b)) < 0.002) discard;
            outColor = vec4(w, 0.0);
        }
        return;
    }

    // Camera is the origin of this space; the ray runs through the pixel.
    vec3 farPoint = reconstruct(uv, isSky ? 0.99999 : depth);
    float rayLen = isSky ? uMaxDist : min(length(farPoint), uMaxDist);
    vec3 dir = normalize(farPoint);

    if (uMode == 2) {
        vec3 world = (isSky ? dir * 80.0 : farPoint) + uCam;
        vec3 g = abs(fract(world / 2.0 - 0.5) - 0.5);
        float line = (g.x < 0.025 || g.y < 0.025 || (g.z < 0.025 && !isSky)) ? 0.35 : 0.0;
        float lit = 0.0;
        if (!isSky) {
            for (int i = 0; i < 12; ++i) {
                if (i >= uCount) break;
                int b = i * 9;
                vec4 pr = uL[b]; vec4 fk = uL[b + 1]; vec4 ry = uL[b + 2]; vec4 ug = uL[b + 3];
                vec3 v = farPoint - pr.xyz;
                float d = length(v);
                if (d > pr.w || d < 0.1 || fk.w < 0.5) continue;
                float hot, upF;
                lit = max(lit, beamShape(v / d, fk.xyz, ry.xyz, ug.xyz, hot, upF));
            }
        }
        outColor = vec4(line * vec3(0.2, 1.0, 0.3) + vec3(lit * 0.55, 0.0, 0.0), 0.0);
        return;
    }

    // uMode == 3: sun-like glare. Only lamps whose beam points at the camera glare, strongest on
    // the axis. Soft core + halo + faint horizontal streak. Visibility is multi-tap against the
    // scene depth in linear distance, so buildings, cars and corners hide it while the source
    // vehicle itself never does.
    if (uMode == 3) {
        vec3 bloom = vec3(0.0);
        float aspect = uInvRes.y / max(uInvRes.x, 1.0e-6);     // width / height
        for (int i = 0; i < 12; ++i) {
            if (i >= uCount) break;
            int b = i * 9;
            vec4 pr = uL[b];
            vec4 fk = uL[b + 1];
            vec4 ry = uL[b + 2];
            vec4 ug = uL[b + 3];
            vec4 col = uL[b + 6];
            if (fk.w < 0.5 || ug.w <= 0.001) continue;

            float dist = length(pr.xyz);
            if (dist < 2.0) continue;
            vec3 toCam = -pr.xyz / dist;
            float hot, upF;
            float beam = beamShape(toCam, fk.xyz, ry.xyz, ug.xyz, hot, upF);
            float glare = beam * (0.35 + 0.65 * hot);
            float distGain = smoothstep(3.0, 10.0, dist) * (1.0 - smoothstep(70.0, 100.0, dist)) *
                             mix(0.75, 1.0, smoothstep(8.0, 40.0, dist));
            if (glare <= 0.002 || distGain <= 0.002) continue;

            vec4 clip = uVP * vec4(pr.xyz, 1.0);
            if (clip.w <= 0.05) continue;
            vec2 sUv = clip.xy / clip.w * 0.5 + 0.5;
            if (any(lessThan(sUv, vec2(-0.10))) || any(greaterThan(sUv, vec2(1.10)))) continue;

            vec2 delta = (uv - sUv) * vec2(aspect, 1.0);
            float rad = 0.026 + 0.026 * smoothstep(4.0, 45.0, dist);
            float r = length(delta);
            if (r > rad * 7.0) continue;

            float rn = r / rad;
            float core = exp(-rn * rn * 2.2);
            float halo = pow(1.0 / (1.0 + rn * rn * 0.5), 1.25) * (1.0 - smoothstep(3.5, 7.0, rn));
            float streak = exp(-(delta.y * delta.y) / (rad * rad * 0.06)) *
                           exp(-abs(delta.x) / (rad * 5.0));
            float profile = core * 0.70 + halo * 0.55 + streak * 0.14;
            if (profile < 0.004) continue;

            // Visibility: 5 taps around the source in linear distance.
            float tol = 0.7 + 0.02 * dist;
            float vis = 0.0;
            vec2 off = vec2(rad * 0.35 / max(aspect, 0.1), rad * 0.35);
            for (int k = 0; k < 5; ++k) {
                vec2 o = k == 0 ? vec2(0.0) : (k == 1 ? vec2(off.x, 0.0) : (k == 2 ? vec2(-off.x, 0.0) :
                         (k == 3 ? vec2(0.0, off.y) : vec2(0.0, -off.y))));
                vec2 tuv = sUv + o;
                if (any(lessThan(tuv, vec2(0.0))) || any(greaterThan(tuv, vec2(1.0)))) { vis += 1.0; continue; }
                float sd = texture(uDepth, tuv).r;
                if (sd >= 0.999999) { vis += 1.0; continue; }
                vec3 sp = reconstruct(tuv, sd);
                if (insideSelf(sp, b, 0.20)) { vis += 1.0; continue; }
                float diff = length(sp) - dist;                  // < 0: something is in front of the lamp
                vis += smoothstep(-1.4 * tol, -0.4 * tol, diff);
            }
            vis *= 0.2;
            vec2 edge = smoothstep(vec2(-0.10), vec2(0.0), sUv) * (1.0 - smoothstep(vec2(1.0), vec2(1.10), sUv));
            float power = profile * glare * distGain * vis * edge.x * edge.y * ug.w;
            bloom += mix(col.rgb, vec3(1.0), 0.35) * power;
        }
        bloom *= uBloomGain;
        if (max(bloom.r, max(bloom.g, bloom.b)) < 0.0015) discard;
        outColor = vec4(min(bloom, vec3(0.55)), 0.0);
        return;
    }

    // uMode == 1: volumetric headlight shafts.
    float jitter = fract(52.9829189 * fract(0.06711056 * gl_FragCoord.x + 0.00583715 * gl_FragCoord.y));
    const int kSteps = 5;
    float stepLen = rayLen / float(kSteps);
    vec3 acc = vec3(0.0);
    for (int s = 0; s < kSteps; ++s) {
        float t = (float(s) + jitter) * stepLen;
        vec3 q = dir * t;
        for (int i = 0; i < 12; ++i) {
            if (i >= uCount) break;
            int b = i * 9;
            vec4 col = uL[b + 6];
            if (col.w <= 0.0) continue;
            vec4 pr = uL[b];
            vec4 fk = uL[b + 1];
            vec4 ry = uL[b + 2];
            vec4 ug = uL[b + 3];
            vec3 fromLight = q - pr.xyz;
            float d = length(fromLight);
            if (d < 0.6 || d > pr.w) continue;
            vec3 ld = fromLight / d;
            float hot, upF;
            float cone = beamShape(ld, fk.xyz, ry.xyz, ug.xyz, hot, upF);
            if (cone <= 0.0) continue;
            float cosView = dot(ld, -dir);
            const float g = 0.45;
            float phase = (1.0 - g * g) / pow(1.0 + g * g - 2.0 * g * cosView, 1.5);
            float atten = distanceAtten(d, pr.w) * smoothstep(0.6, 3.0, d);
            float w = cone * cone * atten * phase * ug.w * ug.w * col.w;
            if (w < 0.0004) continue;
            if (insideSelf(q, b, 0.1)) continue;
            if (uShadow != 0) w *= occlusion(q, pr.xyz, 1, b);
            acc += col.rgb * w * stepLen;
        }
    }
    acc *= 0.035 * uVolGain;
    if (max(acc.r, max(acc.g, acc.b)) < 0.0008) discard;
    outColor = vec4(min(acc, vec3(0.45)), 0.0);
}
)GLSL";

} // namespace VehicleLightGL
