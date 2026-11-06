#include "csg/Analysis.h"
#include "csg/Scene.h"
#include "csg/Smc.h"

#include <glm/gtc/constants.hpp>

#include "Check.h"

#include <cmath>
#include <cstdio>

using namespace csg;

static Primitive prim(PrimType type, glm::vec3 size, glm::vec3 pos = glm::vec3(0.0f))
{
    Primitive p;
    p.type = type;
    p.size = size;
    p.position = pos;
    return p;
}

static Program single(const Primitive& p)
{
    Program prog;
    prog.prims.push_back(p);
    prog.ops.push_back({ OpCode::Push, 0 });
    return prog;
}

static Program binary(const Primitive& a, const Primitive& b, OpCode op)
{
    Program prog;
    prog.prims = { a, b };
    prog.ops = { { OpCode::Push, 0 }, { OpCode::Push, 1 }, { op, -1 } };
    return prog;
}

static void testSpans()
{
    SpanList a = { { 0.0f, 2.0f, 1, 1 } };
    SpanList b = { { 1.0f, 3.0f, 2, 2 } };

    SpanList u = combine(a, b, OpCode::Union);
    CHECK(u.size() == 1 && u[0].t0 == 0.0f && u[0].t1 == 3.0f);

    SpanList i = combine(a, b, OpCode::Intersection);
    CHECK(i.size() == 1 && i[0].t0 == 1.0f && i[0].t1 == 2.0f && i[0].p0 == 2 && i[0].p1 == 1);

    SpanList d = combine(a, b, OpCode::Difference);
    CHECK(d.size() == 1 && d[0].t0 == 0.0f && d[0].t1 == 1.0f && d[0].p1 == -2);

    // B splits A in two
    SpanList hole = combine({ { 0.0f, 3.0f, 1, 1 } }, { { 1.0f, 2.0f, 2, 2 } }, OpCode::Difference);
    CHECK(hole.size() == 2);

    // Touching spans: union merges, intersection is empty (regularized)
    SpanList touchU = combine({ { 0.0f, 1.0f, 1, 1 } }, { { 1.0f, 2.0f, 2, 2 } }, OpCode::Union);
    CHECK(touchU.size() == 1 && touchU[0].t1 == 2.0f);
    SpanList touchI = combine({ { 0.0f, 1.0f, 1, 1 } }, { { 1.0f, 2.0f, 2, 2 } }, OpCode::Intersection);
    CHECK(touchI.empty());
}

static void testIntersections()
{
    const glm::vec3 o(0.0f, 0.0f, -5.0f), d(0.0f, 0.0f, 1.0f);
    float t0, t1;

    CHECK(prim(PrimType::Sphere, glm::vec3(1.0f)).intersect(o, d, t0, t1) && near(t0, 4.0, 1e-5) && near(t1, 6.0, 1e-5));
    CHECK(prim(PrimType::Block, glm::vec3(2.0f)).intersect(o, d, t0, t1) && near(t0, 4.0, 1e-5) && near(t1, 6.0, 1e-5));
    CHECK(prim(PrimType::Cylinder, glm::vec3(1.0f, 2.0f, 0.0f)).intersect(o, d, t0, t1) && near(t0, 4.0, 1e-5));
    // Cone r=1 h=2: at y=0 (mid height) the radius is 0.5
    CHECK(prim(PrimType::Cone, glm::vec3(1.0f, 2.0f, 0.0f)).intersect(o, d, t0, t1) && near(t0, 4.5, 1e-5) && near(t1, 5.5, 1e-5));
    // Along the cone axis: base to apex
    CHECK(prim(PrimType::Cone, glm::vec3(1.0f, 2.0f, 0.0f)).intersect(glm::vec3(0, -5, 0), glm::vec3(0, 1, 0), t0, t1) &&
          near(t0, 4.0, 1e-4) && near(t1, 6.0, 1e-4));
    // Miss above the apex
    CHECK(!prim(PrimType::Cone, glm::vec3(1.0f, 2.0f, 0.0f)).intersect(glm::vec3(0, 1.5f, -5), d, t0, t1));
}

static void testPointClassification()
{
    Program prog = binary(prim(PrimType::Block, glm::vec3(2.0f)), prim(PrimType::Sphere, glm::vec3(0.5f)), OpCode::Difference);
    float eps = onTolerance(prog);
    CHECK(classifyPoint(prog, glm::vec3(0.8f, 0, 0), eps) == Membership::In);
    CHECK(classifyPoint(prog, glm::vec3(0.0f), eps) == Membership::Out);
    CHECK(classifyPoint(prog, glm::vec3(0.5f, 0, 0), eps) == Membership::On);
    CHECK(classifyPoint(prog, glm::vec3(1.0f, 0, 0), eps) == Membership::On);
    CHECK(classifyPoint(prog, glm::vec3(2.0f, 0, 0), eps) == Membership::Out);

    // Edge SMC: a block edge against (block ∪ sphere covering half of that edge).
    Program u = binary(prim(PrimType::Block, glm::vec3(2.0f)), prim(PrimType::Sphere, glm::vec3(1.5f), glm::vec3(1, 1, 1)), OpCode::Union);
    Curve edge;
    edge.a = glm::vec3(-1, 1, 1);
    edge.b = glm::vec3(1, 1, 1);
    auto pieces = classifyCurve(u, edge, onTolerance(u));
    CHECK(pieces.size() == 2 && pieces[0].membership == Membership::On && pieces[1].membership == Membership::In);
    if (pieces.size() == 2)
        checkNear("edge ON/IN split at s=0.25", pieces[0].s1, 0.25, 1e-2);
}

static void testAnalysis()
{
    const double pi = glm::pi<double>();
    Primitive s = prim(PrimType::Sphere, glm::vec3(1.0f));
    Primitive b = prim(PrimType::Block, glm::vec3(1.0f, 2.0f, 3.0f));
    Primitive c = prim(PrimType::Cylinder, glm::vec3(0.5f, 2.0f, 0.0f));
    Primitive k = prim(PrimType::Cone, glm::vec3(1.0f, 2.0f, 0.0f));

    checkNear("sphere volume", treeVolume(single(s)), 4.0 / 3.0 * pi, 0.01);
    checkNear("block volume", treeVolume(single(b)), 6.0, 0.01);
    checkNear("cylinder volume", treeVolume(single(c)), pi * 0.25 * 2.0, 0.01);
    checkNear("cone volume", treeVolume(single(k)), pi * 2.0 / 3.0, 0.01);

    checkNear("sphere area", treeArea(single(s)), 4.0 * pi, 0.01);
    checkNear("block area", treeArea(single(b)), 22.0, 0.01);

    Primitive s2 = prim(PrimType::Sphere, glm::vec3(1.0f), glm::vec3(3, 0, 0));
    checkNear("disjoint union volume", treeVolume(binary(s, s2, OpCode::Union)), 8.0 / 3.0 * pi, 0.01);
    CHECK(treeVolume(binary(s, s2, OpCode::Intersection)) < 1e-6);

    // Two unit spheres with centers 1 apart: lens volume = 5π/12
    Primitive s3 = prim(PrimType::Sphere, glm::vec3(1.0f), glm::vec3(1, 0, 0));
    checkNear("sphere lens volume", treeVolume(binary(s, s3, OpCode::Intersection)), 5.0 * pi / 12.0, 0.01);
    checkNear("sphere union volume", treeVolume(binary(s, s3, OpCode::Union)), 8.0 / 3.0 * pi - 5.0 * pi / 12.0, 0.01);
    // Lens area: two spherical caps of height 1/2 → 2 * 2π r h = 2π
    checkNear("sphere lens area", treeArea(binary(s, s3, OpCode::Intersection)), 2.0 * pi, 0.01);

    // Block minus centered sphere: volume and area both add up
    Primitive cube = prim(PrimType::Block, glm::vec3(3.0f));
    checkNear("cube minus sphere volume", treeVolume(binary(cube, s, OpCode::Difference)), 27.0 - 4.0 / 3.0 * pi, 0.01);
    checkNear("cube minus sphere area", treeArea(binary(cube, s, OpCode::Difference)), 54.0 + 4.0 * pi, 0.01);
}

static void testScene()
{
    Scene scene;
    scene.loadDemo();
    CHECK(scene.roots.size() == 2);
    Program p = scene.displayProgram(0.0f);
    CHECK(p.prims.size() == 6);
    CHECK(p.maxStackDepth() <= 8);

    // Global translation moves every primitive of the tree
    Node* tree = scene.roots[0].get();
    double v0 = treeVolume(scene.program(tree, 0.0f), 128);
    scene.translate(tree, glm::vec3(5, 0, 0));
    Aabb box = scene.bounds(tree, 0.0f);
    checkNear("translated tree center x", box.center().x, 5.0, 1e-3);
    checkNear("volume invariant under translation", treeVolume(scene.program(tree, 0.0f), 128), v0, 0.01);

    // Scaling by parameters scales volume by f^3
    scene.scale(tree, 2.0f, box.center());
    checkNear("volume after scale x2", treeVolume(scene.program(tree, 0.0f), 128), v0 * 8.0, 0.02);

    // Keyframes: the cone moves over time
    Node* cone = scene.roots[1].get();
    CHECK(cone->position.animated());
    checkNear("cone y at mid time", cone->position.value(scene.duration * 0.5f).y, 1.0, 1e-4);
    checkNear("cone y at quarter time", cone->position.value(scene.duration * 0.25f).y, 0.5, 1e-4);

    // Hypervolume of a static tree = volume * duration
    Scene s;
    Node* sphere = s.addPrimitive(PrimType::Sphere);
    double v = treeVolume(s.program(sphere, 0.0f), 128);
    checkNear("static hypervolume", treeHypervolume(s, sphere, 0.0f, 2.0f, 4, 128), 2.0 * v, 1e-3);

    // Limited life: a primitive outside its interval contributes nothing
    sphere->limitedLife = true;
    sphere->visibleFrom = 1.0f;
    sphere->visibleTo = 2.0f;
    CHECK(s.program(sphere, 0.5f).prims.empty());
    CHECK(!s.program(sphere, 1.5f).prims.empty());
}

void runPrimitiveTests();
void runBooleanTests();

int main()
{
    runPrimitiveTests();
    runBooleanTests();
    testSpans();
    testIntersections();
    testPointClassification();
    testAnalysis();
    testScene();
    if (g_failures)
        std::printf("\n%d check(s) failed\n", g_failures);
    else
        std::printf("\nall checks passed\n");
    return g_failures ? 1 : 0;
}
