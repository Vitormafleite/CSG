#pragma once

#include "csg/Scene.h"
#include "render/Camera.h"
#include "render/Framebuffer.h"
#include "render/RayRenderer.h"
#include "render/WireRenderer.h"

#include <string>
#include <unordered_set>
#include <vector>

struct GLFWwindow;

class App
{
public:
    bool init();
    void run();
    void shutdown();

private:
    GLFWwindow* m_window = nullptr;

    csg::Scene m_scene;
    std::vector<int> m_selection; // ordered; the last one is the primary selection

    render::Camera m_camera;
    render::Framebuffer m_viewport;
    render::RayRenderer m_ray;
    render::WireRenderer m_wire;
    render::RaySettings m_settings;
    bool m_wireOverlay = false;

    // Edge SMC is costly, so it is cached until the evaluated scene changes.
    struct EdgePiece
    {
        csg::Curve curve;
        float s0, s1;
        csg::Membership membership;
    };
    std::vector<EdgePiece> m_edgePieces;
    std::vector<float> m_edgeKey;
    bool m_gpuLimitHit = false;
    int m_viewportWidth = 1, m_viewportHeight = 1;

    // Timeline
    bool m_playing = false;
    bool m_loop = true;
    float m_speed = 1.0f;

    // Geometric operation inputs
    glm::vec3 m_translate{ 0.5f, 0.0f, 0.0f };
    glm::vec3 m_rotateAxis{ 0.0f, 1.0f, 0.0f };
    float m_rotateDegrees = 45.0f;
    float m_scaleFactor = 1.25f;

    struct Analysis
    {
        int volumeRes = 192;
        int areaSamples = 20000;
        int hyperSteps = 24;
        bool live = false;
        int nodeId = -1;
        float time = 0.0f;
        double volume = -1.0, area = -1.0, hypervolume = -1.0;
        double milliseconds = 0.0;
    } m_analysis;

    struct Probe
    {
        bool enabled = false;
        glm::vec3 a{ -2.5f, 0.3f, 0.2f };
        glm::vec3 b{ 2.5f, 0.3f, 0.2f };
        csg::SpanList spans; // IN spans along a + t (b - a), t in [0, 1]
    } m_probe;

    bool m_showImGuiDemo = false;

    // Selection
    csg::Node* primary() const;
    csg::Node* primaryRoot() const;
    bool isSelected(int id) const;
    void select(int id, bool additive);
    std::unordered_set<int> highlightedNodes() const;
    void pick(float mouseX, float mouseY, float x0, float y0, bool additive, bool wholeTree);
    void frameSelection();

    // Frame
    void update(float dt);
    void renderViewport();
    void buildWireframe();
    void computeAnalysis(bool withHypervolume);
    const csg::Node* probeTree() const;

    // UI (AppUi.cpp)
    void drawUi();
    void drawMenuBar();
    void drawTreePanel();
    void drawOutlinerNode(csg::Node* node, const char* role);
    void drawPropertiesPanel();
    void drawAnalysisPanel();
    void drawTimeline();
    void drawViewport();
};
