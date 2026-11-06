// Save/load of the .txt scene format: round trips and error reporting.

#include "csg/Mesh.h"
#include "csg/SceneIO.h"

#include "Check.h"
#include "MeshFixtures.h"

#include <glm/gtc/constants.hpp>

#include <filesystem>

using namespace csg;

namespace {

bool sameProgram(const Program& a, const Program& b)
{
    if (a.prims.size() != b.prims.size() || a.ops.size() != b.ops.size())
        return false;
    for (size_t i = 0; i < a.ops.size(); ++i)
        if (a.ops[i].code != b.ops[i].code || a.ops[i].prim != b.ops[i].prim)
            return false;
    for (size_t i = 0; i < a.prims.size(); ++i)
    {
        const Primitive &p = a.prims[i], &q = b.prims[i];
        if (p.type != q.type || p.position != q.position || p.size != q.size || p.color != q.color ||
            p.reflectivity != q.reflectivity || p.mesh != q.mesh ||
            std::fabs(std::fabs(glm::dot(p.rotation, q.rotation)) - 1.0f) > 1e-6f)
            return false;
    }
    return true;
}

void testRoundTrip()
{
    std::printf("-- save/load\n");
    Scene scene;
    scene.loadDemo();
    std::string error;
    Node* torus = scene.addMesh(fixtures::torus(), error);
    CHECK(torus != nullptr);
    if (!torus)
        return;
    torus->name = "Torus \"quoted\" \\ name";
    torus->limitedLife = true;
    torus->visibleFrom = 0.5f;
    torus->visibleTo = 3.25f;
    torus->reflectivity = 0.4f;
    torus->position.set(2.0f, glm::vec3(1.0f / 3.0f, -2.0f, 0.1f), true);
    torus->rotation.set(4.0f, glm::angleAxis(0.7f, glm::normalize(glm::vec3(1, 2, 3))), true);
    Node* cyl = scene.addPrimitive(PrimType::Cylinder);
    scene.combine(torus->id, cyl->id, NodeType::Difference);
    scene.duration = 7.5f;

    std::string text = saveSceneText(scene, "build");
    Scene loaded;
    bool ok = loadSceneText(loaded, text, "build", error);
    if (!ok)
        std::printf("FAIL load: %s\n", error.c_str());
    CHECK(ok);
    CHECK(loaded.roots.size() == scene.roots.size());
    CHECK(loaded.duration == 7.5f);
    // Saving again gives the same text, and every frame evaluates the same.
    CHECK(saveSceneText(loaded, "build") == text);
    bool same = true;
    for (float t : { 0.0f, 0.7f, 1.0f, 2.5f, 3.0f, 4.0f, 7.5f })
        same &= sameProgram(scene.displayProgram(t), loaded.displayProgram(t));
    CHECK(same);
    Node* root = loaded.roots.back().get();
    CHECK(root->type == NodeType::Difference && root->a->name == "Torus \"quoted\" \\ name");
    CHECK(root->a->mesh && root->a->limitedLife && root->a->visibleTo == 3.25f);
    CHECK(loaded.find(root->id) == root);

    // Through a file in another folder: the mesh path is stored relative to it.
    std::filesystem::remove_all("build/io_test");
    ok = saveScene(scene, "build/io_test/scene.txt", error); // folder missing: fails cleanly
    CHECK(!ok && !error.empty());
    std::filesystem::create_directories("build/io_test");
    CHECK(saveScene(scene, "build/io_test/scene.txt", error));
    Scene fromFile;
    ok = loadScene(fromFile, "build/io_test/scene.txt", error);
    if (!ok)
        std::printf("FAIL load file: %s\n", error.c_str());
    CHECK(ok && saveSceneText(fromFile, "build") == text);
    std::printf("ok   round trip (%zu bytes)\n", text.size());
}

void expectError(const char* what, const std::string& text, const char* fragment)
{
    Scene scene;
    scene.loadDemo();
    size_t before = scene.roots.size();
    std::string error;
    bool ok = loadSceneText(scene, text, ".", error);
    bool good = !ok && error.find(fragment) != std::string::npos && scene.roots.size() == before;
    std::printf("%s error: %-36s -> %s\n", good ? "ok  " : "FAIL", what, error.c_str());
    if (!good)
        ++g_failures;
}

void testErrors()
{
    expectError("no header", "tree\nend\n", "expected 'CSG'");
    expectError("bad version", "CSG 2\n", "version");
    expectError("unknown type", "CSG 1\ntree\n primitive pyramid \"p\"\n end\nend\n", "line 3: unknown primitive type");
    expectError("one-child operation", "CSG 1\ntree\n union \"u\"\n  primitive block \"a\"\n  end\n end\nend\n", "line 6");
    expectError("bad number", "CSG 1\ntree\n primitive block \"a\"\n  size 0 1 x 1\n end\nend\n", "line 4: not a number");
    expectError("missing end", "CSG 1\ntree\n primitive block \"a\"\n", "missing 'end'");
    expectError("unterminated name", "CSG 1\ntree\n primitive block \"a\n", "line 3: unterminated");
    expectError("mesh file missing", "CSG 1\ntree\n primitive mesh \"m\"\n  file \"nope.stl\"\n end\nend\n", "cannot open");

    // Comments, blank lines and keys out of order are accepted.
    Scene scene;
    std::string error;
    bool ok = loadSceneText(scene,
                            "# a comment\nCSG 1   # version\n\nduration 2\ntree\n  primitive sphere \"s\"\n"
                            "    position 1  0 1 0\n    position 0  0 0 0\n  end\nend\n",
                            ".", error);
    CHECK(ok && scene.roots.size() == 1 && scene.roots[0]->position.keys().size() == 2);
    CHECK(ok && scene.roots[0]->position.value(0.5f) == glm::vec3(0.0f, 0.5f, 0.0f));
}

} // namespace

void runIoTests()
{
    testRoundTrip();
    testErrors();
}
