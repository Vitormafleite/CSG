#pragma once

// Small STL files for the mesh tests, written on demand under build/.

#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

namespace fixtures {

inline void writeAsciiStl(const std::string& path, const std::vector<glm::vec3>& tris)
{
    FILE* f = std::fopen(path.c_str(), "w");
    std::fprintf(f, "solid test\n");
    for (size_t i = 0; i + 2 < tris.size(); i += 3)
    {
        std::fprintf(f, " facet normal 0 0 0\n  outer loop\n");
        for (int k = 0; k < 3; ++k)
            std::fprintf(f, "   vertex %.9g %.9g %.9g\n", tris[i + k].x, tris[i + k].y, tris[i + k].z);
        std::fprintf(f, "  endloop\n endfacet\n");
    }
    std::fprintf(f, "endsolid test\n");
    std::fclose(f);
}

// Torus around Y with radii R (ring) and r (tube), outward winding.
inline std::string torus(float R = 2.0f, float r = 1.0f, int nu = 96, int nv = 48)
{
    const float pi = glm::pi<float>();
    auto pt = [&](int i, int j) {
        float u = 2.0f * pi * float(i % nu) / nu, v = 2.0f * pi * float(j % nv) / nv;
        return glm::vec3((R + r * std::cos(v)) * std::cos(u), r * std::sin(v), (R + r * std::cos(v)) * std::sin(u));
    };
    std::vector<glm::vec3> tris;
    for (int i = 0; i < nu; ++i)
        for (int j = 0; j < nv; ++j)
        {
            glm::vec3 a = pt(i, j), b = pt(i + 1, j), c = pt(i + 1, j + 1), d = pt(i, j + 1);
            tris.insert(tris.end(), { a, c, b, a, d, c });
        }
    // Make the winding outward if it came out inward.
    glm::vec3 n = glm::cross(tris[1] - tris[0], tris[2] - tris[0]);
    glm::vec3 centroid = (tris[0] + tris[1] + tris[2]) / 3.0f;
    glm::vec3 tubeCenter = glm::normalize(glm::vec3(centroid.x, 0.0f, centroid.z)) * R;
    if (glm::dot(n, centroid - tubeCenter) < 0.0f)
        for (size_t i = 0; i < tris.size(); i += 3)
            std::swap(tris[i + 1], tris[i + 2]);
    std::string path = "build/test_torus.stl";
    writeAsciiStl(path, tris);
    return path;
}

// Unit cube [-0.5, 0.5]^3 as 12 triangles, outward winding.
inline std::string cube()
{
    std::vector<glm::vec3> tris;
    for (int axis = 0; axis < 3; ++axis)
        for (float s : { -0.5f, 0.5f })
        {
            int i = (axis + 1) % 3, j = (axis + 2) % 3;
            glm::vec3 c[4];
            const float corners[4][2] = { { -0.5f, -0.5f }, { 0.5f, -0.5f }, { 0.5f, 0.5f }, { -0.5f, 0.5f } };
            for (int k = 0; k < 4; ++k)
            {
                c[k][axis] = s;
                c[k][i] = corners[k][0];
                c[k][j] = corners[k][1];
            }
            if (s > 0.0f)
                tris.insert(tris.end(), { c[0], c[1], c[2], c[0], c[2], c[3] });
            else
                tris.insert(tris.end(), { c[0], c[2], c[1], c[0], c[3], c[2] });
        }
    std::string path = "build/test_cube.stl";
    writeAsciiStl(path, tris);
    return path;
}

inline const char* kDragon = "model/dragon_with_pearl.stl";

} // namespace fixtures
