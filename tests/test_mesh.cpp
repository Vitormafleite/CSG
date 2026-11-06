// Mesh primitive: STL loading, solid classification (winding of crossings),
// SMC consistency, analysis, and set operations with the analytic primitives.

#include "csg/Analysis.h"
#include "csg/Mesh.h"
#include "csg/Smc.h"

#include "Check.h"
#include "MeshFixtures.h"

#include <fstream>
#include <random>

using namespace csg;

namespace {

std::mt19937 g_rng(99);
float uniform(float a, float b) { return std::uniform_real_distribution<float>(a, b)(g_rng); }
glm::vec3 randomVec(float a, float b) { return glm::vec3(uniform(a, b), uniform(a, b), uniform(a, b)); }
glm::quat randomRotation()
{
    std::normal_distribution<float> n;
    return glm::normalize(glm::quat(n(g_rng), n(g_rng), n(g_rng), n(g_rng)));
}

std::shared_ptr<const Mesh> load(const std::string& path)
{
    std::string error;
    auto mesh = Mesh::load(path, error);
    if (!mesh)
        std::printf("FAIL load %s: %s\n", path.c_str(), error.c_str());
    CHECK(mesh != nullptr);
    return mesh;
}

Primitive meshPrim(const std::shared_ptr<const Mesh>& mesh, float scale, glm::vec3 pos = glm::vec3(0.0f),
                   glm::quat rot = glm::quat(1, 0, 0, 0))
{
    Primitive p;
    p.type = PrimType::Mesh;
    p.mesh = mesh;
    p.size = glm::vec3(scale, 0.0f, 0.0f);
    p.position = pos;
    p.rotation = rot;
    return p;
}

Program single(const Primitive& p)
{
    Program prog;
    prog.prims = { p };
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

// Ray SMC and point SMC agree: span midpoints are inside, gaps are outside.
void checkRayVsPoint(const std::string& name, const Program& prog, float radius, int rays)
{
    Property prop(name + ": ray SMC = point SMC");
    for (int r = 0; r < rays; ++r)
    {
        glm::vec3 o = glm::normalize(randomVec(-1, 1)) * radius * 2.0f;
        glm::vec3 target = randomVec(-0.5f, 0.5f) * radius;
        glm::vec3 d = glm::normalize(target - o);
        SpanList spans = classifyRay(prog, o, d);
        float prev = -1e30f;
        bool ok = true;
        for (const Span& s : spans)
        {
            if (s.t1 - s.t0 > 1e-3f)
                ok &= treeSdf(prog, o + d * (0.5f * (s.t0 + s.t1))) < 0.0f;
            if (prev > -1e29f && s.t0 - prev > 1e-3f)
                ok &= treeSdf(prog, o + d * (0.5f * (prev + s.t0))) > 0.0f;
            prev = s.t1;
        }
        prop.record(ok);
    }
}

void testLoading()
{
    std::printf("-- mesh loading\n");
    auto torus = load(fixtures::torus());
    auto cube = load(fixtures::cube());
    if (!torus || !cube)
        return;
    CHECK(torus->triangleCount() == 96 * 48 * 2);
    CHECK(torus->closed());
    CHECK(cube->closed());
    CHECK(torus->volume() > 0.0);
    // Normalized: largest extent 1. The torus is 6 x 2 x 6.
    checkNear("torus half extent x", torus->halfExtent().x, 0.5, 1e-5);
    checkNear("torus half extent y", torus->halfExtent().y, 1.0 / 6.0, 1e-5);
    // Polyhedral volume close to 2 pi^2 R r^2 (R = 2, r = 1), in file units.
    double s3 = std::pow(double(torus->fileScale()), 3.0);
    checkNear("torus volume vs 2 pi^2 R r^2", torus->volume() / s3, 2.0 * glm::pi<double>() * glm::pi<double>() * 2.0, 0.01);
    checkNear("cube volume", cube->volume(), 1.0, 1e-6);
    checkNear("cube area", cube->area(), 6.0, 1e-6);
    CHECK(load("build/test_torus.stl") == torus); // cached while in use

    // A hole in the surface is reported.
    {
        std::ofstream f("build/test_open.stl");
        f << "solid open\nfacet normal 0 0 1\nouter loop\nvertex 0 0 0\nvertex 1 0 0\nvertex 0 1 0\nendloop\nendfacet\nendsolid\n";
    }
    auto open = load("build/test_open.stl");
    CHECK(open && !open->closed() && open->openEdges() == 3);

    std::string error;
    CHECK(!Mesh::load("build/does_not_exist.stl", error) && !error.empty());
}

void testClassification()
{
    std::printf("-- mesh classification\n");
    auto torus = load(fixtures::torus());
    auto cube = load(fixtures::cube());
    if (!torus || !cube)
        return;

    // Torus (scaled to 6 units across): the hole is OUT, the tube is IN.
    Primitive t = meshPrim(torus, 6.0f);
    CHECK(t.sdf(glm::vec3(0.0f)) > 0.9f);              // center of the hole, 1 from the tube
    CHECK(t.sdf(glm::vec3(2.0f, 0.0f, 0.0f)) < -0.9f); // tube center
    CHECK(t.sdf(glm::vec3(0.0f, 5.0f, 0.0f)) > 3.9f);

    // A line along X through the ring crosses the solid twice.
    SpanList spans = classifyRay(single(t), glm::vec3(-10, 0, 0), glm::vec3(1, 0, 0));
    CHECK(spans.size() == 2);
    if (spans.size() == 2)
    {
        checkNear("torus span 1 length", spans[0].t1 - spans[0].t0, 2.0, 0.01);
        checkNear("torus span 2 start", spans[1].t0, 11.0, 0.01);
    }

    // Lines exactly through shared edges and vertices of the 12-triangle cube
    // must cross it once: no gap, no double count.
    Primitive c = meshPrim(cube, 1.0f);
    Property edges("cube: lines through edges/vertices");
    const float coords[] = { -0.5f, -0.25f, 0.0f, 0.25f, 0.5f };
    for (int axis = 0; axis < 3; ++axis)
        for (float a : coords)
            for (float b : coords)
            {
                glm::vec3 o(0.0f), d(0.0f);
                o[axis] = -3.0f;
                d[axis] = 1.0f;
                o[(axis + 1) % 3] = a * 0.999f; // just inside the faces, along their diagonals and edges
                o[(axis + 2) % 3] = b * 0.999f;
                SpanList s = classifyRay(single(c), o, d);
                bool ok = s.size() == 1 && near(s[0].t0, 2.5, 1e-4) && near(s[0].t1, 3.5, 1e-4);
                edges.record(ok);
            }
    // Diagonal lines through the triangle diagonals and the cube's corners.
    for (int i = 0; i < 50; ++i)
    {
        glm::vec3 d = glm::normalize(glm::vec3(1.0f, uniform(-1, 1), uniform(-1, 1)));
        glm::vec3 o = -d * 3.0f;
        SpanList s = classifyRay(single(c), o, d);
        edges.record(s.size() == 1);
    }
    SpanList corner = classifyRay(single(c), glm::vec3(-1.0f), glm::normalize(glm::vec3(1.0f)));
    edges.record(corner.size() == 1 && near(corner[0].t1 - corner[0].t0, std::sqrt(3.0), 1e-3));

    checkRayVsPoint("torus", single(meshPrim(torus, 2.0f, glm::vec3(0.3f, -0.2f, 0.1f), randomRotation())), 1.0f, 400);
}

void testAnalysis()
{
    std::printf("-- mesh analysis\n");
    auto torus = load(fixtures::torus());
    if (!torus)
        return;
    Primitive t = meshPrim(torus, 3.0f, glm::vec3(0.4f, 0.1f, -0.3f), randomRotation());
    checkNear("torus primitive volume (scaled)", t.volume(), torus->volume() * 27.0, 1e-5);
    checkNear("torus treeVolume = exact", treeVolume(single(t), 256), t.volume(), 0.005);
    checkNear("torus treeArea = exact", treeArea(single(t), 20000), t.area(), 0.01);

    // Set operations with an analytic primitive. The torus tube has radius 0.5
    // (scale 3: 6 units -> 3), ring radius 1. A big block cutting it in half.
    Primitive half;
    half.type = PrimType::Block;
    half.size = glm::vec3(4.0f, 4.0f, 2.0f);
    half.position = glm::vec3(0.0f, 0.0f, 1.0f);
    Primitive t0 = meshPrim(torus, 3.0f);
    double v = t0.volume();
    checkNear("torus n half-space block", treeVolume(binary(t0, half, OpCode::Intersection), 256), v * 0.5, 0.01);
    checkNear("torus - half-space block", treeVolume(binary(t0, half, OpCode::Difference), 256), v * 0.5, 0.01);
    checkNear("torus u half-space block", treeVolume(binary(t0, half, OpCode::Union), 256),
              half.volume() + v * 0.5, 0.01);

    Primitive sphere;
    sphere.type = PrimType::Sphere;
    sphere.size = glm::vec3(0.8f);
    sphere.position = glm::vec3(1.0f, 0.0f, 0.0f);
    Program diff = binary(t0, sphere, OpCode::Difference);
    Program inter = binary(t0, sphere, OpCode::Intersection);
    checkNear("V(T-S) + V(TnS) = V(T)", treeVolume(diff, 256) + treeVolume(inter, 256), v, 0.005);
    checkRayVsPoint("torus - sphere", diff, 1.6f, 300);
}

void testDragon()
{
    std::FILE* f = std::fopen(fixtures::kDragon, "rb");
    if (!f)
    {
        std::printf("skip dragon (no %s)\n", fixtures::kDragon);
        return;
    }
    std::fclose(f);
    std::printf("-- dragon\n");
    auto dragon = load(fixtures::kDragon);
    if (!dragon)
        return;
    CHECK(dragon->triangleCount() == 999634);
    CHECK(dragon->closed());
    double s3 = std::pow(double(dragon->fileScale()), 3.0);
    checkNear("dragon volume (file units)", dragon->volume() / s3, 402992.886, 1e-5);
    Primitive d = meshPrim(dragon, 2.0f, glm::vec3(0.0f), randomRotation());
    checkNear("dragon treeVolume = exact", treeVolume(single(d), 256), d.volume(), 0.005);
    checkRayVsPoint("dragon", single(d), 1.0f, 300);
}

} // namespace

void runMeshTests()
{
    testLoading();
    testClassification();
    testAnalysis();
    testDragon();
}
