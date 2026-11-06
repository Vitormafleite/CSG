// Property tests for every primitive type with random size, rotation and position.

#include "csg/Analysis.h"
#include "csg/Primitive.h"
#include "csg/Smc.h"

#include "Check.h"

#include <random>
#include <string>

using namespace csg;

namespace {

std::mt19937 g_rng(12345);

float uniform(float a, float b) { return std::uniform_real_distribution<float>(a, b)(g_rng); }

glm::vec3 randomVec(float a, float b) { return glm::vec3(uniform(a, b), uniform(a, b), uniform(a, b)); }

glm::quat randomRotation()
{
    std::normal_distribution<float> n;
    return glm::normalize(glm::quat(n(g_rng), n(g_rng), n(g_rng), n(g_rng)));
}

Primitive randomPrim(PrimType type)
{
    Primitive p;
    p.type = type;
    p.position = randomVec(-2.0f, 2.0f);
    p.rotation = randomRotation();
    p.size = randomVec(0.3f, 2.0f);
    return p;
}

float scaleOf(const Primitive& p) { return std::max(p.size.x, std::max(p.size.y, p.size.z)); }

bool inside(const Aabb& box, const glm::vec3& p, float tol)
{
    return glm::all(glm::greaterThanEqual(p, box.min - tol)) && glm::all(glm::lessThanEqual(p, box.max + tol));
}

Program single(const Primitive& p)
{
    Program prog;
    prog.prims.push_back(p);
    prog.ops.push_back({ OpCode::Push, 0 });
    return prog;
}

void testType(PrimType type)
{
    const std::string name = primTypeName(type);
    const int trials = 3000;

    {
        // Ray interval endpoints lie on the surface, the inside is inside, the outside is outside.
        Property prop(name + ": ray interval matches SDF");
        for (int i = 0; i < trials; ++i)
        {
            Primitive p = randomPrim(type);
            float tol = 2e-4f * scaleOf(p) + 1e-5f;
            glm::vec3 o = p.position + randomVec(-5.0f, 5.0f);
            glm::vec3 d = glm::normalize(randomVec(-1.0f, 1.0f)) * uniform(0.5f, 2.0f);
            // Some axis-aligned rays in local space to hit the degenerate branches
            if (i % 5 == 0)
            {
                glm::vec3 axis(0.0f);
                axis[i % 3] = (i % 2) ? 1.0f : -1.0f;
                d = p.rotation * axis;
                o = p.position + p.rotation * glm::vec3(uniform(-0.5f, 0.5f), uniform(-0.5f, 0.5f), uniform(-0.5f, 0.5f)) * scaleOf(p) - d * 10.0f;
            }

            float t0, t1;
            bool ok = true;
            double err = 0.0;
            if (p.intersect(o, d, t0, t1))
            {
                float e0 = std::fabs(p.sdf(o + d * t0)), e1 = std::fabs(p.sdf(o + d * t1));
                err = std::max(e0, e1);
                ok &= e0 < tol && e1 < tol && t0 <= t1;
                for (int k = 1; k < 8 && t1 - t0 > 1e-3f; ++k)
                    ok &= p.sdf(o + d * (t0 + (t1 - t0) * k / 8.0f)) < tol;
                float dl = glm::length(d);
                ok &= p.sdf(o + d * (t0 - 0.01f / dl)) > -tol;
                ok &= p.sdf(o + d * (t1 + 0.01f / dl)) > -tol;
            }
            else
            {
                for (int k = 0; k <= 200; ++k)
                {
                    float s = p.sdf(o + d * (-10.0f + 20.0f * k / 200.0f));
                    ok &= s > -tol;
                    if (s <= -tol)
                        err = std::max(err, double(-s));
                }
            }
            prop.record(ok, err);
        }
    }

    {
        // Surface samples are on the surface and in the bounds; the normal is outward:
        // stepping delta along it moves exactly delta away from a convex solid.
        Property prop(name + ": surface points, normals, bounds");
        for (int i = 0; i < trials; ++i)
        {
            Primitive p = randomPrim(type);
            float tol = 2e-4f * scaleOf(p) + 1e-5f;
            glm::vec3 x = p.surfacePoint(uniform(0, 1), uniform(0, 1), uniform(0, 1));
            glm::vec3 n = p.normal(x);
            float delta = 0.01f * scaleOf(p);
            float stepped = p.sdf(x + n * delta);
            double err = std::fabs(stepped - delta) / delta;
            bool ok = std::fabs(p.sdf(x)) < tol && std::fabs(glm::length(n) - 1.0f) < 1e-4f && err < 0.05 &&
                      inside(p.bounds(), x, tol);
            prop.record(ok, err);
        }
    }

    {
        Property prop(name + ": wireframe edges on surface");
        for (int i = 0; i < 300; ++i)
        {
            Primitive p = randomPrim(type);
            float tol = 2e-4f * scaleOf(p) + 1e-5f;
            for (const Curve& c : p.edges())
                for (int k = 0; k <= 16; ++k)
                {
                    glm::vec3 x = c.point(k / 16.0f);
                    double err = std::fabs(p.sdf(x));
                    prop.record(err < tol && inside(p.bounds(), x, tol), err);
                }
        }
    }

    {
        Property prop(name + ": volume and area vs formulas");
        for (int i = 0; i < 12; ++i)
        {
            Primitive p = randomPrim(type);
            Program prog = single(p);
            double ev = std::fabs(treeVolume(prog, 256) - p.volume()) / p.volume();
            double ea = std::fabs(treeArea(prog, 20000) - p.area()) / p.area();
            prop.record(ev < 0.01 && ea < 0.01, std::max(ev, ea));
        }
    }
}

} // namespace

void runPrimitiveTests()
{
    std::printf("-- primitives\n");
    for (PrimType t : { PrimType::Block, PrimType::Sphere, PrimType::Cylinder, PrimType::Cone })
        testType(t);
}
