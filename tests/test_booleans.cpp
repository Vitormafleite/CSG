// Set operations on trees: random cross-checks between independent methods,
// set identities, and the classic degenerate cases (shared and flush faces).

#include "csg/Analysis.h"
#include "csg/Smc.h"

#include "Check.h"

#include <glm/gtc/constants.hpp>

#include <random>
#include <string>

using namespace csg;

namespace {

std::mt19937 g_rng(777);

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
    p.position = randomVec(-0.6f, 0.6f);
    p.rotation = randomRotation();
    p.size = randomVec(0.6f, 1.8f);
    return p;
}

Primitive prim(PrimType type, glm::vec3 size, glm::vec3 pos = glm::vec3(0.0f), glm::quat rot = glm::quat(1, 0, 0, 0))
{
    Primitive p;
    p.type = type;
    p.size = size;
    p.position = pos;
    p.rotation = rot;
    return p;
}

Program single(const Primitive& a)
{
    Program prog;
    prog.prims = { a };
    prog.ops = { { OpCode::Push, 0 } };
    return prog;
}

Program binary(const Primitive& a, const Primitive& b, OpCode op)
{
    Program prog;
    prog.prims = { a, b };
    prog.ops = { { OpCode::Push, 0 }, { OpCode::Push, 1 }, { op, -1 } };
    return prog;
}

// Independent volume estimate: fraction of random points with negative tree SDF.
double monteCarloVolume(const Program& prog, int samples)
{
    Aabb box = prog.bounds();
    glm::vec3 ext = box.max - box.min;
    int in = 0;
    for (int i = 0; i < samples; ++i)
    {
        glm::vec3 p = box.min + ext * glm::vec3(uniform(0, 1), uniform(0, 1), uniform(0, 1));
        if (treeSdf(prog, p) < 0.0f)
            ++in;
    }
    return double(ext.x) * ext.y * ext.z * in / samples;
}

const PrimType kTypes[] = { PrimType::Block, PrimType::Sphere, PrimType::Cylinder, PrimType::Cone };
const OpCode kOps[] = { OpCode::Union, OpCode::Intersection, OpCode::Difference };
const char* kOpNames[] = { "union", "intersection", "difference" };

void testRandomPairs()
{
    Property mc("pairs: ray volume == Monte Carlo volume");
    Property identV("pairs: V(AuB)+V(AnB) == V(A)+V(B)");
    Property identD("pairs: V(A-B) == V(A)-V(AnB)");
    Property identA("pairs: S(AuB)+S(AnB) == S(A)+S(B)");
    Property identAD("pairs: S(A-B)+S(B-A) == S(A)+S(B)");

    for (PrimType ta : kTypes)
        for (PrimType tb : kTypes)
            for (int rep = 0; rep < 3; ++rep)
            {
                Primitive a = randomPrim(ta), b = randomPrim(tb);
                double va = treeVolume(single(a), 192), vb = treeVolume(single(b), 192);
                double vu = treeVolume(binary(a, b, OpCode::Union), 192);
                double vi = treeVolume(binary(a, b, OpCode::Intersection), 192);
                double vd = treeVolume(binary(a, b, OpCode::Difference), 192);

                for (int k = 0; k < 3; ++k)
                {
                    Program prog = binary(a, b, kOps[k]);
                    double v = k == 0 ? vu : k == 1 ? vi : vd;
                    Aabb box = prog.bounds();
                    glm::vec3 e = box.max - box.min;
                    double err = std::fabs(monteCarloVolume(prog, 200000) - v) / (double(e.x) * e.y * e.z);
                    mc.record(err < 0.006, err); // ~4 standard errors of the estimate
                }

                double scale = va + vb;
                double e1 = std::fabs(vu + vi - va - vb) / scale;
                double e2 = std::fabs(vd - (va - vi)) / scale;
                identV.record(e1 < 0.003, e1);
                identD.record(e2 < 0.003, e2);

                double sa = a.area(), sb = b.area();
                double su = treeArea(binary(a, b, OpCode::Union), 20000);
                double si = treeArea(binary(a, b, OpCode::Intersection), 20000);
                double sab = treeArea(binary(a, b, OpCode::Difference), 20000);
                double sba = treeArea(binary(b, a, OpCode::Difference), 20000);
                double e3 = std::fabs(su + si - sa - sb) / (sa + sb);
                double e4 = std::fabs(sab + sba - sa - sb) / (sa + sb);
                identA.record(e3 < 0.01, e3);
                identAD.record(e4 < 0.01, e4);
            }
}

// Random 4-primitive trees: the ray spans (interval SMC) must agree with point
// classification by the tree SDF everywhere along the ray.
void testRaySmcMatchesPointSmc()
{
    Property prop("trees: ray SMC agrees with point SMC");
    for (int i = 0; i < 2000; ++i)
    {
        Program prog;
        for (int k = 0; k < 4; ++k)
            prog.prims.push_back(randomPrim(kTypes[g_rng() % 4]));
        // ((p0 op p1) op (p2 op p3)) or (((p0 op p1) op p2) op p3)
        auto op = [&] { return Op{ kOps[g_rng() % 3], -1 }; };
        if (i % 2)
            prog.ops = { { OpCode::Push, 0 }, { OpCode::Push, 1 }, op(), { OpCode::Push, 2 }, { OpCode::Push, 3 }, op(), op() };
        else
            prog.ops = { { OpCode::Push, 0 }, { OpCode::Push, 1 }, op(), { OpCode::Push, 2 }, op(), { OpCode::Push, 3 }, op() };

        glm::vec3 o = randomVec(-4.0f, 4.0f);
        glm::vec3 d = glm::normalize(randomVec(-1.0f, 1.0f));
        SpanList spans = classifyRay(prog, o, d);

        bool ok = true;
        double worst = 0.0;
        const float tol = 1e-3f;
        for (int k = 0; k <= 400; ++k)
        {
            float t = -8.0f + 16.0f * k / 400.0f;
            bool inSpan = false, nearEnd = false;
            for (const Span& s : spans)
            {
                inSpan |= t > s.t0 && t < s.t1;
                nearEnd |= std::fabs(t - s.t0) < 2e-3f || std::fabs(t - s.t1) < 2e-3f;
            }
            if (nearEnd)
                continue;
            float sdf = treeSdf(prog, o + d * t);
            if (inSpan ? sdf > tol : sdf < -tol)
            {
                ok = false;
                worst = std::max(worst, double(std::fabs(sdf)));
            }
        }
        prop.record(ok, worst);
    }
}

void testDegenerateCases()
{
    const double pi = glm::pi<double>();
    const glm::vec3 one(1.0f);
    Primitive cube = prim(PrimType::Block, one);

    // Two unit cubes sharing a face: the shared face is interior, not boundary.
    Primitive right = prim(PrimType::Block, one, glm::vec3(1, 0, 0));
    checkNear("touching cubes union volume", treeVolume(binary(cube, right, OpCode::Union)), 2.0, 0.005);
    checkNear("touching cubes union area", treeArea(binary(cube, right, OpCode::Union)), 10.0, 0.01);
    CHECK(treeVolume(binary(cube, right, OpCode::Intersection)) < 1e-6);
    checkNear("touching cubes intersection area", treeArea(binary(cube, right, OpCode::Intersection)) + 1.0, 1.0, 0.01);
    checkNear("touching cubes difference volume", treeVolume(binary(cube, right, OpCode::Difference)), 1.0, 0.005);
    checkNear("touching cubes difference area", treeArea(binary(cube, right, OpCode::Difference)), 6.0, 0.01);

    // Identical solids: coincident surfaces must count once.
    checkNear("cube u same cube volume", treeVolume(binary(cube, cube, OpCode::Union)), 1.0, 0.005);
    checkNear("cube u same cube area", treeArea(binary(cube, cube, OpCode::Union)), 6.0, 0.01);
    checkNear("cube n same cube area", treeArea(binary(cube, cube, OpCode::Intersection)), 6.0, 0.01);
    CHECK(treeVolume(binary(cube, cube, OpCode::Difference)) < 1e-6);
    checkNear("cube - same cube area", treeArea(binary(cube, cube, OpCode::Difference)) + 1.0, 1.0, 0.01);

    // Flush cut: removing the right half of a 2x2x2 block, sharing four faces.
    Primitive big = prim(PrimType::Block, glm::vec3(2.0f));
    Primitive half = prim(PrimType::Block, glm::vec3(1, 2, 2), glm::vec3(0.5f, 0, 0));
    checkNear("flush cut volume", treeVolume(binary(big, half, OpCode::Difference)), 4.0, 0.005);
    checkNear("flush cut area", treeArea(binary(big, half, OpCode::Difference)), 16.0, 0.01);
    checkNear("flush half intersection area", treeArea(binary(big, half, OpCode::Intersection)), 16.0, 0.01);
    checkNear("flush half union area", treeArea(binary(big, half, OpCode::Union)), 24.0, 0.01);

    // Tube: inner cylinder caps lie in the outer caps' planes.
    double R = 1.0, r = 0.5, h = 2.0;
    Primitive outer = prim(PrimType::Cylinder, glm::vec3(R, h, 0));
    Primitive inner = prim(PrimType::Cylinder, glm::vec3(r, h, 0));
    checkNear("tube volume", treeVolume(binary(outer, inner, OpCode::Difference)), pi * (R * R - r * r) * h, 0.01);
    checkNear("tube area", treeArea(binary(outer, inner, OpCode::Difference)),
              2 * pi * R * h + 2 * pi * r * h + 2 * pi * (R * R - r * r), 0.01);

    // Cone sitting flush on a block (base on the top face).
    Primitive slab = prim(PrimType::Block, glm::vec3(2, 1, 2));
    Primitive cone = prim(PrimType::Cone, glm::vec3(0.8f, 1.0f, 0), glm::vec3(0, 1.0f, 0));
    double coneArea = pi * 0.8 * std::sqrt(0.64 + 1.0);
    checkNear("cone on block union volume", treeVolume(binary(slab, cone, OpCode::Union)), 4.0 + pi * 0.64 / 3.0, 0.01);
    // the base disc covers part of the block's top face
    checkNear("cone on block union area", treeArea(binary(slab, cone, OpCode::Union)), 16.0 - pi * 0.64 + coneArea, 0.01);

    // Ray SMC through the flush cut: one IN span from x=-1 to x=0.
    SpanList spans = classifyRay(binary(big, half, OpCode::Difference), glm::vec3(-5, 0.3f, 0.2f), glm::vec3(1, 0, 0));
    CHECK(spans.size() == 1 && near(spans[0].t0, 4.0, 1e-5) && near(spans[0].t1, 5.0, 1e-5));
    // ...and along the shared top face plane (ON the boundary), nothing is solid.
    spans = classifyRay(binary(big, half, OpCode::Difference), glm::vec3(-5, 1.0f, 0.2f), glm::vec3(1, 0, 0));
    CHECK(spans.size() <= 1);
}

} // namespace

void runBooleanTests()
{
    std::printf("-- set operations\n");
    testRandomPairs();
    testRaySmcMatchesPointSmc();
    testDegenerateCases();
}
