#include "App.h"

#include "render/Gl.h"

#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include "csg/Analysis.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <thread>

namespace {

void errorCallback(int error, const char* description)
{
    std::fprintf(stderr, "GLFW error %d: %s\n", error, description);
}

const glm::vec3 kColorIn(0.25f, 0.85f, 0.35f);
const glm::vec3 kColorOn(1.0f, 0.85f, 0.2f);
const glm::vec3 kColorOut(0.9f, 0.25f, 0.25f);

glm::vec3 membershipColor(csg::Membership m)
{
    switch (m)
    {
    case csg::Membership::In: return kColorIn;
    case csg::Membership::On: return kColorOn;
    default: return kColorOut;
    }
}

} // namespace

bool App::init(const char* scenePath)
{
    glfwSetErrorCallback(errorCallback);
    if (!glfwInit())
        return false;

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    float scale = ImGui_ImplGlfw_GetContentScaleForMonitor(glfwGetPrimaryMonitor());
    m_window = glfwCreateWindow(int(1440 * scale), int(860 * scale), "CSG Modeler", nullptr, nullptr);
    if (!m_window)
        return false;

    glfwMakeContextCurrent(m_window);
    glfwSwapInterval(1);
    if (!gladLoadGL(glfwGetProcAddress))
    {
        std::fprintf(stderr, "Failed to load OpenGL\n");
        return false;
    }

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.IniFilename = nullptr; // fixed layout
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    ImGui::StyleColorsDark();
    ImGuiStyle& style = ImGui::GetStyle();
    style.ScaleAllSizes(scale);
    style.FontScaleDpi = scale;
    style.WindowRounding = 0.0f;

    ImGui_ImplGlfw_InitForOpenGL(m_window, true);
    ImGui_ImplOpenGL3_Init("#version 330 core");

    if (!m_ray.init())
        std::fprintf(stderr, "%s\n", m_ray.error().c_str());
    if (!m_wire.init())
        std::fprintf(stderr, "line shader failed\n");

    m_scene.loadDemo();
    if (scenePath)
        openScene(scenePath);
    return true;
}

void App::shutdown()
{
    m_ray.release();
    m_wire.release();
    m_viewport.release();
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(m_window);
    glfwTerminate();
}

void App::run()
{
    double last = glfwGetTime();
    while (!glfwWindowShouldClose(m_window))
    {
        glfwPollEvents();
        double now = glfwGetTime();
        update(float(now - last));
        last = now;

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        drawUi();
        ImGui::Render();

        // The viewport texture is drawn by ImGui below, so render it first.
        renderViewport();

        int fbw, fbh;
        glfwGetFramebufferSize(m_window, &fbw, &fbh);
        glViewport(0, 0, fbw, fbh);
        glClearColor(0.08f, 0.08f, 0.09f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(m_window);
    }
}

void App::update(float dt)
{
    if (m_playing)
    {
        m_scene.time += dt * m_speed;
        if (m_scene.time > m_scene.duration)
        {
            if (m_loop)
                m_scene.time = 0.0f;
            else
            {
                m_scene.time = m_scene.duration;
                m_playing = false;
            }
        }
    }

    // Drop selections whose nodes no longer exist.
    m_selection.erase(std::remove_if(m_selection.begin(), m_selection.end(),
                                     [&](int id) { return m_scene.find(id) == nullptr; }),
                      m_selection.end());

    if (m_analysis.live)
        computeAnalysis(false);
}

// ---------------------------------------------------------------- selection

csg::Node* App::primary() const
{
    return m_selection.empty() ? nullptr : m_scene.find(m_selection.back());
}

csg::Node* App::primaryRoot() const
{
    return m_selection.empty() ? nullptr : m_scene.rootOf(m_selection.back());
}

bool App::isSelected(int id) const
{
    return std::find(m_selection.begin(), m_selection.end(), id) != m_selection.end();
}

void App::select(int id, bool additive)
{
    if (!additive)
    {
        m_selection = { id };
        return;
    }
    auto it = std::find(m_selection.begin(), m_selection.end(), id);
    if (it != m_selection.end())
        m_selection.erase(it);
    else
        m_selection.push_back(id);
}

std::unordered_set<int> App::highlightedNodes() const
{
    std::unordered_set<int> ids;
    for (int id : m_selection)
        csg::Scene::forEachPrimitive(static_cast<const csg::Node*>(m_scene.find(id)),
                                     [&](const csg::Node& n) { ids.insert(n.id); });
    return ids;
}

void App::pick(float mouseX, float mouseY, float x0, float y0, bool additive, bool wholeTree)
{
    glm::vec2 ndc((mouseX - x0) / m_viewportWidth * 2.0f - 1.0f, 1.0f - (mouseY - y0) / m_viewportHeight * 2.0f);
    glm::vec3 origin, dir;
    m_camera.ray(ndc, float(m_viewportWidth) / float(m_viewportHeight), origin, dir);

    csg::Program program = m_scene.displayProgram(m_scene.time);
    for (const csg::Span& span : csg::classifyRay(program, origin, dir))
    {
        if (span.t0 <= 0.0f)
            continue;
        int nodeId = program.prims[std::abs(span.p0) - 1].nodeId;
        if (wholeTree)
            nodeId = m_scene.rootOf(nodeId)->id;
        select(nodeId, additive);
        return;
    }
    if (!additive)
        m_selection.clear();
}

void App::frameSelection()
{
    csg::Aabb box;
    if (csg::Node* node = primary())
        box = m_scene.bounds(node, m_scene.time);
    else
        box = m_scene.displayProgram(m_scene.time).bounds();
    if (box.empty())
        return;
    m_camera.target = box.center();
    m_camera.distance = std::max(1.0f, glm::length(box.max - box.min) * 1.4f);
}

// ---------------------------------------------------------------- rendering

// Everything the viewport image depends on, as raw bytes.
std::string App::frameKey(const csg::Program& program, const std::unordered_set<int>& highlight) const
{
    std::string key;
    auto put = [&](const auto& value) { key.append(reinterpret_cast<const char*>(&value), sizeof(value)); };
    put(m_camera.target);
    put(m_camera.yaw);
    put(m_camera.pitch);
    put(m_camera.distance);
    put(m_camera.fovY);
    put(m_viewportWidth);
    put(m_viewportHeight);
    put(m_settings.mode);
    put(m_settings.bounces);
    put(m_settings.lightDir);
    put(m_settings.ground);
    put(m_settings.groundY);
    put(m_settings.smcAlpha);
    put(m_wireOverlay);
    put(m_probe.enabled);
    put(m_probe.a);
    put(m_probe.b);
    for (const csg::Primitive& p : program.prims)
    {
        put(p.type);
        put(p.position);
        put(p.rotation);
        put(p.size);
        put(p.color);
        put(p.reflectivity);
        put(p.nodeId);
        put(p.mesh.get());
        put(bool(highlight.count(p.nodeId)));
    }
    for (const csg::Op& op : program.ops)
    {
        put(op.code);
        put(op.prim);
    }
    return key;
}

void App::renderViewport()
{
    // SMC view classifies against the selected tree only.
    csg::Program program;
    const csg::Node* smcTree = primaryRoot();
    if (m_settings.mode == render::RenderMode::Smc && smcTree)
        program = m_scene.program(smcTree, m_scene.time);
    else
        program = m_scene.displayProgram(m_scene.time);
    std::unordered_set<int> highlight = highlightedNodes();

    // Skip the frame when nothing changed; render at reduced resolution while
    // things keep changing, then once at full resolution when they settle.
    double now = glfwGetTime();
    std::string key = frameKey(program, highlight);
    if (key != m_frameKey)
    {
        m_frameKey = key;
        m_lastChange = now;
    }
    bool moving = m_resolution.adaptive && now - m_lastChange < 0.3;
    float scale = m_resolution.still * (moving ? m_resolution.moving : 1.0f);
    int width = std::max(1, int(m_viewportWidth * scale));
    int height = std::max(1, int(m_viewportHeight * scale));
    if (!m_forceRender && key == m_renderedKey && width == m_renderedWidth && height == m_renderedHeight)
        return;
    m_forceRender = false;
    m_renderedKey = key;
    m_renderedWidth = width;
    m_renderedHeight = height;
    auto start = std::chrono::steady_clock::now();

    m_viewport.resize(width, height);
    m_viewport.bind();
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    m_gpuLimitHit = !m_ray.upload(program, highlight);
    m_ray.draw(m_camera, width, height, m_settings);

    m_wire.clear();
    if (m_settings.mode == render::RenderMode::Wireframe || m_wireOverlay)
        buildWireframe();

    if (m_probe.enabled)
    {
        m_probe.spans = csg::SpanList{};
        if (const csg::Node* tree = probeTree())
        {
            csg::Program prog = m_scene.program(tree, m_scene.time);
            glm::vec3 d = m_probe.b - m_probe.a;
            for (const csg::Span& s : csg::classifyRay(prog, m_probe.a, d))
            {
                csg::Span c = s;
                c.t0 = std::max(c.t0, 0.0f);
                c.t1 = std::min(c.t1, 1.0f);
                if (c.t0 < c.t1)
                    m_probe.spans.push_back(c);
            }

            float t = 0.0f;
            float markerSize = 0.04f * glm::length(d) / 5.0f + 0.02f;
            for (const csg::Span& s : m_probe.spans)
            {
                if (s.t0 > t)
                    m_wire.line(m_probe.a + d * t, m_probe.a + d * s.t0, kColorOut);
                m_wire.line(m_probe.a + d * s.t0, m_probe.a + d * s.t1, kColorIn);
                m_wire.marker(m_probe.a + d * s.t0, markerSize, kColorOn);
                m_wire.marker(m_probe.a + d * s.t1, markerSize, kColorOn);
                t = s.t1;
            }
            if (t < 1.0f)
                m_wire.line(m_probe.a + d * t, m_probe.b, kColorOut);
        }
    }

    float aspect = float(m_viewportWidth) / float(m_viewportHeight);
    m_wire.draw(m_camera.projection(aspect) * m_camera.view());
    render::Framebuffer::unbind();

    glFinish(); // so the timing below measures the GPU work
    m_renderMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
}

// Edge SMC: every primitive edge of each tree classified against that tree.
void App::buildWireframe()
{
    std::vector<csg::Program> programs;
    std::vector<float> key;
    for (const auto& root : m_scene.roots)
    {
        programs.push_back(m_scene.program(root.get(), m_scene.time));
        for (const csg::Primitive& p : programs.back().prims)
            key.insert(key.end(), { float(int(p.type)), p.position.x, p.position.y, p.position.z, p.rotation.w,
                                    p.rotation.x, p.rotation.y, p.rotation.z, p.size.x, p.size.y, p.size.z });
        for (const csg::Op& op : programs.back().ops)
            key.push_back(float(int(op.code)));
        key.push_back(-1.0f); // tree separator
    }

    if (key != m_edgeKey)
    {
        m_edgeKey = std::move(key);

        struct Job
        {
            const csg::Program* program;
            float eps;
            csg::Curve curve;
        };
        std::vector<Job> jobs;
        for (const csg::Program& program : programs)
        {
            float eps = csg::onTolerance(program);
            for (const csg::Primitive& prim : program.prims)
                for (const csg::Curve& curve : prim.edges())
                    jobs.push_back({ &program, eps, curve });
        }

        std::vector<std::vector<csg::CurvePiece>> results(jobs.size());
        unsigned threads = std::max(1u, std::thread::hardware_concurrency());
        std::vector<std::thread> pool;
        for (unsigned w = 0; w < threads; ++w)
            pool.emplace_back([&, w] {
                for (size_t i = w; i < jobs.size(); i += threads)
                    results[i] = csg::classifyCurve(*jobs[i].program, jobs[i].curve, jobs[i].eps, jobs[i].curve.samples);
            });
        for (auto& t : pool)
            t.join();

        m_edgePieces.clear();
        for (size_t i = 0; i < jobs.size(); ++i)
            for (const csg::CurvePiece& piece : results[i])
                m_edgePieces.push_back({ jobs[i].curve, piece.s0, piece.s1, piece.membership });
    }

    for (const EdgePiece& e : m_edgePieces)
        m_wire.curve(e.curve, e.s0, e.s1, membershipColor(e.membership));
}

const csg::Node* App::probeTree() const
{
    if (const csg::Node* root = primaryRoot())
        return root;
    return m_scene.roots.empty() ? nullptr : m_scene.roots.front().get();
}

void App::computeAnalysis(bool withHypervolume)
{
    const csg::Node* tree = primary();
    if (!tree)
    {
        m_analysis.nodeId = -1;
        return;
    }
    auto start = std::chrono::steady_clock::now();
    csg::Program program = m_scene.program(tree, m_scene.time);
    if (tree->id != m_analysis.nodeId)
        m_analysis.hypervolume = -1.0;
    m_analysis.nodeId = tree->id;
    m_analysis.time = m_scene.time;
    m_analysis.volume = csg::treeVolume(program, m_analysis.volumeRes);
    m_analysis.area = csg::treeArea(program, m_analysis.areaSamples);
    if (withHypervolume)
        m_analysis.hypervolume = csg::treeHypervolume(m_scene, tree, 0.0f, m_scene.duration, m_analysis.hyperSteps,
                                                      std::min(m_analysis.volumeRes, 128));
    auto end = std::chrono::steady_clock::now();
    m_analysis.milliseconds = std::chrono::duration<double, std::milli>(end - start).count();
}
