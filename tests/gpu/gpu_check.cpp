// Renders scenes with the GPU ray caster in hit-buffer mode and compares every
// pixel against the CPU classification (csg::classifyRay). Needs a display.

#include "render/Gl.h"

#include <GLFW/glfw3.h>

#include "csg/Scene.h"
#include "csg/Smc.h"
#include "render/RayRenderer.h"

#include "../Check.h"

#include <random>
#include <string>
#include <unordered_set>
#include <vector>

using namespace csg;

namespace {

constexpr int kWidth = 320, kHeight = 240;

std::mt19937 g_rng(4242);
float uniform(float a, float b) { return std::uniform_real_distribution<float>(a, b)(g_rng); }
glm::vec3 randomVec(float a, float b) { return glm::vec3(uniform(a, b), uniform(a, b), uniform(a, b)); }

glm::quat randomRotation()
{
    std::normal_distribution<float> n;
    return glm::normalize(glm::quat(n(g_rng), n(g_rng), n(g_rng), n(g_rng)));
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

Primitive randomPrim(PrimType type)
{
    return prim(type, randomVec(0.6f, 1.8f), randomVec(-0.6f, 0.6f), randomRotation());
}

Program binary(const Primitive& a, const Primitive& b, OpCode op)
{
    Program prog;
    prog.prims = { a, b };
    prog.ops = { { OpCode::Push, 0 }, { OpCode::Push, 1 }, { op, -1 } };
    return prog;
}

struct Target
{
    GLuint fbo = 0, tex = 0;

    void create()
    {
        glGenTextures(1, &tex);
        glBindTexture(GL_TEXTURE_2D, tex);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, kWidth, kHeight, 0, GL_RGBA, GL_FLOAT, nullptr);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glGenFramebuffers(1, &fbo);
        glBindFramebuffer(GL_FRAMEBUFFER, fbo);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, tex, 0);
    }
};

// Renders the program from a few camera angles and compares with the CPU.
void check(const std::string& name, const Program& program, render::RayRenderer& renderer, Target& target)
{
    if (!renderer.upload(program, {}))
    {
        std::printf("FAIL %-44s program exceeds GPU limits\n", name.c_str());
        ++g_failures;
        return;
    }

    Aabb box = program.bounds();
    render::Camera camera;
    camera.target = box.empty() ? glm::vec3(0.0f) : box.center();
    camera.distance = box.empty() ? 5.0f : glm::length(box.max - box.min) * 1.3f + 0.5f;

    long pixels = 0, coverage = 0, depth = 0, ids = 0;
    std::string firstMismatch;
    double worstDepth = 0.0;
    std::vector<float> buf(kWidth * kHeight * 4);
    const float aspect = float(kWidth) / float(kHeight);

    for (int view = 0; view < 4; ++view)
    {
        camera.yaw = 0.4f + 1.7f * view;
        camera.pitch = view % 2 ? -0.5f : 0.45f;

        render::RaySettings settings;
        settings.mode = render::RenderMode::HitBuffer;
        glBindFramebuffer(GL_FRAMEBUFFER, target.fbo);
        glViewport(0, 0, kWidth, kHeight);
        renderer.draw(camera, kWidth, kHeight, settings);
        glReadPixels(0, 0, kWidth, kHeight, GL_RGBA, GL_FLOAT, buf.data());

        for (int y = 0; y < kHeight; ++y)
            for (int x = 0; x < kWidth; ++x)
            {
                const float* px = &buf[(y * kWidth + x) * 4];
                bool gpuHit = px[0] >= 0.0f;

                glm::vec2 ndc((x + 0.5f) / kWidth * 2.0f - 1.0f, (y + 0.5f) / kHeight * 2.0f - 1.0f);
                glm::vec3 o, d;
                camera.ray(ndc, aspect, o, d);
                bool cpuHit = false;
                float cpuT = 0.0f;
                int cpuId = 0;
                SpanList spans = classifyRay(program, o, d);
                auto describe = [&] {
                    if (!firstMismatch.empty())
                        return;
                    char buf[512];
                    int n = std::snprintf(buf, sizeof(buf), "    view %d px (%d,%d): gpu t=%.5f id=%d | cpu spans:", view, x, y,
                                          px[0], int(std::lround(px[1])));
                    for (const Span& s : spans)
                        n += std::snprintf(buf + n, sizeof(buf) - n, " [%.5f %.5f](%d,%d)", s.t0, s.t1, s.p0, s.p1);
                    firstMismatch = buf;
                };
                for (const Span& s : spans)
                    if (s.t0 > 0.0f)
                    {
                        cpuHit = true;
                        cpuT = s.t0;
                        cpuId = s.p0;
                        break;
                    }

                ++pixels;
                if (gpuHit != cpuHit)
                {
                    ++coverage;
                    describe();
                    continue;
                }
                if (!gpuHit)
                    continue;
                double err = std::fabs(px[0] - cpuT) / (1.0 + cpuT);
                worstDepth = std::max(worstDepth, err);
                if (err > 1e-3)
                {
                    ++depth;
                    describe();
                }
                int gpuId = int(std::lround(px[1]));
                if (gpuId != cpuId)
                {
                    // Acceptable only if both surfaces pass through the hit (coincident faces).
                    glm::vec3 hit = o + d * cpuT;
                    int gi = std::abs(gpuId) - 1;
                    bool coincident = gi >= 0 && gi < int(program.prims.size()) &&
                                      std::fabs(program.prims[gi].sdf(hit)) < 1e-3f;
                    if (!coincident)
                    {
                        ++ids;
                        describe();
                    }
                }
            }
    }

    // Allow a handful of grazing pixels (silhouettes, tangent slivers near the
    // span epsilon) where float rounding differs between CPU and GPU.
    bool ok = coverage + depth + ids <= pixels / 2000;
    std::printf("%s %-44s coverage %ld, depth %ld, id %ld of %ld px (worst depth %.1e)\n", ok ? "ok  " : "FAIL",
                name.c_str(), coverage, depth, ids, pixels, worstDepth);
    if (!ok)
        ++g_failures;
    if (!ok && !firstMismatch.empty())
        std::printf("%s\n", firstMismatch.c_str());
}

} // namespace

int main()
{
    if (!glfwInit())
        return 1;
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    GLFWwindow* window = glfwCreateWindow(64, 64, "gpu_check", nullptr, nullptr);
    if (!window || (glfwMakeContextCurrent(window), !gladLoadGL(glfwGetProcAddress)))
    {
        std::printf("cannot create an OpenGL 3.3 context\n");
        return 1;
    }
    std::printf("GPU: %s\n", (const char*)glGetString(GL_RENDERER));

    render::RayRenderer renderer;
    if (!renderer.init())
    {
        std::printf("shader error:\n%s\n", renderer.error().c_str());
        return 1;
    }
    Target target;
    target.create();

    const PrimType types[] = { PrimType::Block, PrimType::Sphere, PrimType::Cylinder, PrimType::Cone };
    const OpCode ops[] = { OpCode::Union, OpCode::Intersection, OpCode::Difference };
    const char* opNames[] = { "u", "n", "-" };

    std::printf("-- single primitives\n");
    for (PrimType t : types)
        for (int i = 0; i < 3; ++i)
        {
            Program p;
            p.prims = { randomPrim(t) };
            p.ops = { { OpCode::Push, 0 } };
            check(std::string(primTypeName(t)) + " #" + std::to_string(i), p, renderer, target);
        }

    std::printf("-- pairs\n");
    for (PrimType ta : types)
        for (PrimType tb : types)
            for (int k = 0; k < 3; ++k)
                check(std::string(primTypeName(ta)) + " " + opNames[k] + " " + primTypeName(tb),
                      binary(randomPrim(ta), randomPrim(tb), ops[k]), renderer, target);

    std::printf("-- degenerate\n");
    const glm::vec3 one(1.0f);
    check("touching cubes u", binary(prim(PrimType::Block, one), prim(PrimType::Block, one, { 1, 0, 0 }), OpCode::Union), renderer, target);
    check("cube - same cube (empty)", binary(prim(PrimType::Block, one), prim(PrimType::Block, one), OpCode::Difference), renderer, target);
    check("cube u same cube", binary(prim(PrimType::Block, one), prim(PrimType::Block, one), OpCode::Union), renderer, target);
    check("flush cut", binary(prim(PrimType::Block, glm::vec3(2)), prim(PrimType::Block, { 1, 2, 2 }, { 0.5f, 0, 0 }), OpCode::Difference), renderer, target);
    check("tube", binary(prim(PrimType::Cylinder, { 1, 2, 0 }), prim(PrimType::Cylinder, { 0.5f, 2, 0 }), OpCode::Difference), renderer, target);
    check("cone on block", binary(prim(PrimType::Block, { 2, 1, 2 }), prim(PrimType::Cone, { 0.8f, 1, 0 }, { 0, 1, 0 }), OpCode::Union), renderer, target);

    std::printf("-- trees\n");
    Scene scene;
    scene.loadDemo();
    check("demo scene t=0", scene.displayProgram(0.0f), renderer, target);
    check("demo scene t=1.3", scene.displayProgram(1.3f), renderer, target);
    for (int i = 0; i < 10; ++i)
    {
        Program p;
        for (int k = 0; k < 5; ++k)
            p.prims.push_back(randomPrim(types[g_rng() % 4]));
        auto op = [&] { return Op{ ops[g_rng() % 3], -1 }; };
        p.ops = { { OpCode::Push, 0 }, { OpCode::Push, 1 }, op(), { OpCode::Push, 2 }, { OpCode::Push, 3 }, op(), op(), { OpCode::Push, 4 }, op() };
        check("random 5-primitive tree #" + std::to_string(i), p, renderer, target);
    }

    renderer.release();
    glfwDestroyWindow(window);
    glfwTerminate();
    std::printf(g_failures ? "\n%d check(s) failed\n" : "\nall GPU checks passed\n", g_failures);
    return g_failures ? 1 : 0;
}
