// Host regression test for collision-clipped Hann splats: ground z=0, wall y=9, pole y=14.
#include "vehicle_light_beam_core.h"
#include <cstdio>
using namespace BeamCore;

static bool hasWall = true, hasPole = false;
static constexpr float WALL = 9.0f;

static bool Cast(const V3& a, const V3& b, V3& p, V3& n) {
    float best = 2.0f;
    V3 bestNormal;
    const V3 d = b - a;
    if (d.z < -1e-6f) {
        const float t = -a.z / d.z;
        if (t >= 0.0f && t <= 1.0f) { best = t; bestNormal = V3(0, 0, 1); }
    }
    if (hasWall && d.y > 1e-6f) {
        const float t = (WALL - a.y) / d.y;
        const V3 q = a + d * t;
        if (t >= 0.0f && t <= 1.0f && t < best && q.x > -8.0f && q.x < 8.0f && q.z >= 0.0f) {
            best = t;
            bestNormal = V3(0, -1, 0);
        }
    }
    if (hasPole && d.y > 1e-6f) {
        const float t = (14.0f - a.y) / d.y;
        const V3 q = a + d * t;
        if (t >= 0.0f && t <= 1.0f && t < best && std::fabs(q.x) < 0.15f && q.z >= 0.0f) {
            best = t;
            bestNormal = V3(0, -1, 0);
        }
    }
    if (best > 1.0f) return false;
    p = a + d * best;
    n = bestNormal;
    return true;
}

int main() {
    Frame vehicle;
    vehicle.origin = V3(0, 0, 0.5f);
    vehicle.right = V3(1, 0, 0);
    vehicle.fwd = V3(0, 1, 0);
    vehicle.up = V3(0, 0, 1);
    Frame beam = vehicle;
    beam.origin = V3(0, 2.1f, 0.7f);
    BeamSpec spec;
    int failures = 0;
    struct Scenario { const char* name; bool wall; bool pole; };
    const Scenario scenarios[] = {{"wall", true, false}, {"open", false, false}, {"pole", false, true}};

    for (const Scenario& scenario : scenarios) {
        hasWall = scenario.wall;
        hasPole = scenario.pole;
        Beam beamData;
        const int hits = BuildBeam(spec, beam, vehicle, 1.0f, Cast, beamData);
        for (int i = 0; i < kVerts; ++i) beamData.current[i] = beamData.target[i];

        float maxCornerY = -1e9f, maxZ = 0.0f, behindPole = 0.0f;
        int splats = 0, wallSplats = 0, groundSplats = 0;
        constexpr int width = 400, height = 300;
        static float image[height][width];
        static float kernelSum[height][width];
        for (auto& row : image) for (float& value : row) value = 0.0f;
        for (auto& row : kernelSum) for (float& value : row) value = 0.0f;

        for (int i = 0; i < kVerts; ++i) {
            if (!beamData.placed[i] || !beamData.hit[i]) continue;
            ++splats;
            Quad quad;
            SplatQuad(beamData, i, vehicle, quad);
            const bool wall = std::fabs(beamData.normal[i].z) < 0.5f;
            (wall ? wallSplats : groundSplats)++;
            for (int corner = 0; corner < 4; ++corner) {
                maxZ = std::max(maxZ, quad.p[corner].z);
                if (std::fabs(quad.p[corner].x) < 7.0f)
                    maxCornerY = std::max(maxCornerY, quad.p[corner].y);
                if (scenario.pole && wall)
                    behindPole = std::max(behindPole, quad.p[corner].y - 14.0f);
            }
            if (wall) continue;

            float x0 = 1e9f, x1 = -1e9f, y0 = 1e9f, y1 = -1e9f;
            for (const V3& point : quad.p) {
                x0 = std::min(x0, point.x); x1 = std::max(x1, point.x);
                y0 = std::min(y0, point.y); y1 = std::max(y1, point.y);
            }
            const V3 center = PointToWorld(beamData.local[i], vehicle);
            const V3 tangent = DirToWorld(beamData.t1[i], vehicle);
            const V3 bitangent = DirToWorld(beamData.t2[i], vehicle);
            const float front = beamData.ext[i][0], back = beamData.ext[i][1];
            const float left = beamData.ext[i][2], right = beamData.ext[i][3];
            const float alongExtent = std::max(front, back), acrossExtent = std::max(left, right);
            for (int py = 0; py < height; ++py) for (int px = 0; px < width; ++px) {
                const float wx = -10.0f + (px + 0.5f) * 20.0f / width;
                const float wy = (height - py - 0.5f) * 30.0f / height;
                if (wx < x0 || wx > x1 || wy < y0 || wy > y1) continue;
                const V3 delta = V3(wx, wy, 0) - center;
                const float along = Dot(delta, tangent), across = Dot(delta, bitangent);
                if (along > front || along < -back || across > right || across < -left) continue;
                const float u = 0.5f + 0.5f * across / acrossExtent;
                const float v = 0.5f + 0.5f * along / alongExtent;
                const float weight = SoftKernel(u, v);
                image[py][px] += weight * beamData.current[i];
                kernelSum[py][px] += weight;
            }
        }

        float minimumKernel = 1e9f, maximumKernel = 0.0f;
        for (int py = 0; py < height; ++py) for (int px = 0; px < width; ++px) {
            const float wx = -10.0f + (px + 0.5f) * 20.0f / width;
            const float wy = (height - py - 0.5f) * 30.0f / height;
            if (std::fabs(wx) < 0.45f * (wy - 2.1f) && wy > 5.0f && wy < 20.0f &&
                (!scenario.wall || wy < 8.2f) && (!scenario.pole || std::fabs(wx) > 1.8f)) {
                minimumKernel = std::min(minimumKernel, kernelSum[py][px]);
                maximumKernel = std::max(maximumKernel, kernelSum[py][px]);
            }
        }
        std::printf("%-5s hits=%d splats=%d (ground %d, wall %d) maxCornerY=%.2f maxZ=%.2f kernelSum min=%.2f max=%.2f\n",
                    scenario.name, hits, splats, groundSplats, wallSplats, maxCornerY, maxZ,
                    minimumKernel, maximumKernel);
        if (scenario.wall && maxCornerY > WALL + 0.45f) { std::printf("FAIL: splat crosses wall\n"); ++failures; }
        if (scenario.wall && (wallSplats == 0 || maxZ < 1.0f)) { std::printf("FAIL: beam did not climb wall\n"); ++failures; }
        if (!scenario.wall && !scenario.pole && groundSplats < 120) { std::printf("FAIL: ground coverage is too small\n"); ++failures; }
        if (scenario.pole && behindPole > 0.25f) { std::printf("FAIL: splat passes behind pole\n"); ++failures; }
        if (scenario.pole && wallSplats < 1) { std::printf("FAIL: pole was not hit\n"); ++failures; }
        if (minimumKernel < 0.60f || maximumKernel > 1.9f) { std::printf("FAIL: kernel coverage is uneven\n"); ++failures; }

        char filename[64];
        std::snprintf(filename, sizeof(filename), "splat_%s.pgm", scenario.name);
        if (FILE* output = std::fopen(filename, "wb")) {
            std::fprintf(output, "P5\n%d %d\n255\n", width, height);
            for (int y = 0; y < height; ++y) for (int x = 0; x < width; ++x)
                std::fputc(static_cast<int>(Clamp01(image[y][x]) * 255.0f), output);
            std::fclose(output);
        }
    }
    std::printf(failures ? "TEST FAILED\n" : "ALL OK\n");
    return failures;
}
