#pragma once

#include "csg/Smc.h"
#include "render/Camera.h"
#include "render/Shader.h"

#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

namespace render {

// Values shared with shaders/csg.frag (uMode).
enum class RenderMode : int
{
    RayCast = 0,
    RayTrace = 1,
    Smc = 2,
    Wireframe = 3,
    HitBuffer = 4, // (t, surface id) of the first hit, for checking the GPU against the CPU
};

struct RaySettings
{
    RenderMode mode = RenderMode::RayCast;
    int bounces = 3;
    glm::vec3 lightDir = glm::normalize(glm::vec3(0.5f, 1.0f, 0.3f));
    bool ground = true;
    float groundY = -1.01f;
    float smcAlpha = 0.3f;
};

// GPU ray casting / ray tracing of a CSG program. The program is uploaded to
// a float texture and evaluated per pixel with the same span algebra as
// csg::classifyRay.
class RayRenderer
{
public:
    static constexpr int kMaxPrims = 64;
    static constexpr int kMaxOps = 256;
    static constexpr int kMaxStack = 8; // STACK in csg.frag
    static constexpr int kMaxMeshPrims = 8; // MAXMESH in csg.frag

    ~RayRenderer() { release(); }

    bool init();
    // Frees GL objects; must run while the GL context is alive.
    void release();
    bool reload();
    const std::string& error() const { return m_error; }

    // Returns false (and draws nothing) when the program exceeds the GPU limits.
    bool upload(const csg::Program& program, const std::unordered_set<int>& highlightedNodes);
    // Why the last upload failed.
    const std::string& limitError() const { return m_limitError; }
    void draw(const Camera& camera, int width, int height, const RaySettings& settings);

private:
    // csg.frag variants: one per render mode (RENDER_MODE), without and with
    // mesh support (MESH_SUPPORT). Each is compiled the first time it is used,
    // so a program only inlines the code paths of one mode.
    static constexpr int kModes = 5;
    Shader m_variants[kModes][2];
    bool m_tried[kModes][2] = {};
    bool m_useMesh = false;
    std::string m_error;

    Shader* variant(RenderMode mode, bool mesh);
    GLuint m_vao = 0;
    GLuint m_data = 0;
    int m_primCount = 0;
    glm::vec3 m_boundsMin{ 0.0f }, m_boundsMax{ -1.0f };
    int m_opCount = 0;
    std::vector<float> m_buffer;
    std::string m_limitError;

    // Meshes resident in the texture buffers (kept until a new one is needed).
    struct MeshSlot
    {
        std::shared_ptr<const csg::Mesh> mesh;
        int nodeBase, triBase;
    };
    std::vector<MeshSlot> m_meshes;
    GLuint m_nodeBuf = 0, m_nodeTex = 0, m_triBuf = 0, m_triTex = 0;

    bool uploadMeshes(const csg::Program& program);
    const MeshSlot* slot(const csg::Mesh* mesh) const;
};

} // namespace render
