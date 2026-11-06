#include "render/RayRenderer.h"

#include "csg/Mesh.h"

#include <algorithm>
#include <cstdio>

namespace render {

namespace {
constexpr int kTexWidth = 256; // 4 texels per primitive in row 0, 1 texel per op in row 1
}

void RayRenderer::release()
{
    if (m_data)
        glDeleteTextures(1, &m_data);
    if (m_vao)
        glDeleteVertexArrays(1, &m_vao);
    GLuint textures[] = { m_nodeTex, m_triTex };
    GLuint buffers[] = { m_nodeBuf, m_triBuf };
    if (m_nodeTex)
    {
        glDeleteTextures(2, textures);
        glDeleteBuffers(2, buffers);
    }
    m_data = m_vao = 0;
    for (auto& modes : m_variants)
        for (Shader& s : modes)
            s.release();
    for (auto& modes : m_tried)
        modes[0] = modes[1] = false;
    m_nodeBuf = m_nodeTex = m_triBuf = m_triTex = 0;
    m_meshes.clear();
}

bool RayRenderer::init()
{
    glGenVertexArrays(1, &m_vao);
    glGenTextures(1, &m_data);
    glBindTexture(GL_TEXTURE_2D, m_data);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, kTexWidth, 2, 0, GL_RGBA, GL_FLOAT, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    m_buffer.assign(kTexWidth * 2 * 4, 0.0f);

    // Mesh texture buffers, with a dummy texel until a mesh is uploaded.
    glGenBuffers(1, &m_nodeBuf);
    glGenBuffers(1, &m_triBuf);
    glGenTextures(1, &m_nodeTex);
    glGenTextures(1, &m_triTex);
    const float zero[4] = {};
    for (auto [buf, tex] : { std::pair{ m_nodeBuf, m_nodeTex }, std::pair{ m_triBuf, m_triTex } })
    {
        glBindBuffer(GL_TEXTURE_BUFFER, buf);
        glBufferData(GL_TEXTURE_BUFFER, sizeof(zero), zero, GL_STATIC_DRAW);
        glBindTexture(GL_TEXTURE_BUFFER, tex);
        glTexBuffer(GL_TEXTURE_BUFFER, GL_RGBA32F, buf);
    }
    glBindBuffer(GL_TEXTURE_BUFFER, 0);
    return reload();
}

namespace {
std::string defines(int mode, bool mesh)
{
    return "#define RENDER_MODE " + std::to_string(mode) + "\n#define MESH_SUPPORT " + (mesh ? "1" : "0") + "\n";
}
} // namespace

Shader* RayRenderer::variant(RenderMode mode, bool mesh)
{
    int m = mode == RenderMode::Wireframe ? int(RenderMode::RayCast) : int(mode); // wireframe: background only
    Shader& s = m_variants[m][mesh];
    if (!m_tried[m][mesh])
    {
        m_tried[m][mesh] = true;
        if (mesh)
            std::fprintf(stderr, "Compiling the mesh shader for render mode %d...\n", m);
        if (!s.load("fullscreen.vert", "csg.frag", defines(m, mesh)))
        {
            m_error = s.error();
            std::fprintf(stderr, "%s\n", m_error.c_str());
        }
    }
    return s.valid() ? &s : nullptr;
}

bool RayRenderer::reload()
{
    // Recompile the variants already in use; on failure the old program stays.
    m_error.clear();
    bool ok = true;
    for (int m = 0; m < kModes; ++m)
        for (int mesh = 0; mesh < 2; ++mesh)
            if (m_tried[m][mesh] && !m_variants[m][mesh].load("fullscreen.vert", "csg.frag", defines(m, mesh)))
            {
                m_error = m_variants[m][mesh].error();
                ok = false;
            }
    if (!m_tried[0][0])
        ok &= variant(RenderMode::RayCast, false) != nullptr;
    return ok;
}

const RayRenderer::MeshSlot* RayRenderer::slot(const csg::Mesh* mesh) const
{
    for (const MeshSlot& s : m_meshes)
        if (s.mesh.get() == mesh)
            return &s;
    return nullptr;
}

bool RayRenderer::uploadMeshes(const csg::Program& program)
{
    std::vector<std::shared_ptr<const csg::Mesh>> wanted;
    bool missing = false;
    for (const csg::Primitive& p : program.prims)
        if (p.mesh && std::find(wanted.begin(), wanted.end(), p.mesh) == wanted.end())
        {
            wanted.push_back(p.mesh);
            missing |= slot(p.mesh.get()) == nullptr;
        }
    if (!missing)
        return true;

    // Keep the resident meshes that are still needed, add the new ones.
    std::vector<std::shared_ptr<const csg::Mesh>> meshes;
    for (const MeshSlot& s : m_meshes)
        if (s.mesh.use_count() > 1) // still referenced by the scene
            meshes.push_back(s.mesh);
    for (const auto& m : wanted)
        if (std::find(meshes.begin(), meshes.end(), m) == meshes.end())
            meshes.push_back(m);

    size_t nodeTexels = 0, triTexels = 0;
    for (const auto& m : meshes)
    {
        nodeTexels += m->nodes().size() * 2;
        triTexels += m->triangleCount() * 3;
    }
    GLint maxTexels = 0;
    glGetIntegerv(GL_MAX_TEXTURE_BUFFER_SIZE, &maxTexels);
    if (nodeTexels > size_t(maxTexels) || triTexels > size_t(maxTexels))
    {
        m_limitError = "Mesh too large for this GPU (" + std::to_string(triTexels) + " texels, limit " +
                       std::to_string(maxTexels) + ")";
        return false;
    }

    std::vector<float> nodes, tris;
    nodes.reserve(nodeTexels * 4);
    tris.reserve(triTexels * 4);
    m_meshes.clear();
    for (const auto& m : meshes)
    {
        m_meshes.push_back({ m, int(nodes.size() / 8), int(tris.size() / 12) });
        for (const csg::BvhNode& n : m->nodes())
            nodes.insert(nodes.end(), { n.min.x, n.min.y, n.min.z, float(n.leftOrFirst), n.max.x, n.max.y, n.max.z,
                                        float(n.count) });
        const auto& v = m->vertices();
        const auto& nr = m->normals();
        for (size_t i = 0; i < nr.size(); ++i)
        {
            glm::vec3 v0 = v[i * 3], e1 = v[i * 3 + 1] - v0, e2 = v[i * 3 + 2] - v0;
            tris.insert(tris.end(), { v0.x, v0.y, v0.z, nr[i].x, e1.x, e1.y, e1.z, nr[i].y, e2.x, e2.y, e2.z, nr[i].z });
        }
    }
    glBindBuffer(GL_TEXTURE_BUFFER, m_nodeBuf);
    glBufferData(GL_TEXTURE_BUFFER, GLsizeiptr(nodes.size() * sizeof(float)), nodes.data(), GL_STATIC_DRAW);
    glBindBuffer(GL_TEXTURE_BUFFER, m_triBuf);
    glBufferData(GL_TEXTURE_BUFFER, GLsizeiptr(tris.size() * sizeof(float)), tris.data(), GL_STATIC_DRAW);
    glBindBuffer(GL_TEXTURE_BUFFER, 0);
    return true;
}

bool RayRenderer::upload(const csg::Program& program, const std::unordered_set<int>& highlightedNodes)
{
    m_limitError.clear();
    if (int(program.prims.size()) > kMaxPrims || int(program.ops.size()) > kMaxOps ||
        program.maxStackDepth() > kMaxStack)
    {
        m_limitError = "Scene exceeds GPU limits (64 primitives, 256 ops, depth 8)";
        m_primCount = m_opCount = 0;
        return false;
    }
    int meshPrims = 0;
    for (const csg::Primitive& p : program.prims)
        meshPrims += p.type == csg::PrimType::Mesh;
    m_useMesh = meshPrims > 0;
    if (meshPrims > kMaxMeshPrims)
    {
        m_limitError = "Scene exceeds GPU limits (8 mesh primitives)";
        m_primCount = m_opCount = 0;
        return false;
    }
    // Slightly padded program bounds: rays that miss them skip the evaluation.
    csg::Aabb box = program.bounds();
    float pad = box.empty() ? 0.0f : 1e-4f * glm::length(box.max - box.min) + 1e-5f;
    m_boundsMin = box.min - pad;
    m_boundsMax = box.max + pad;
    if (!uploadMeshes(program))
    {
        m_primCount = m_opCount = 0;
        return false;
    }

    std::fill(m_buffer.begin(), m_buffer.end(), 0.0f);
    auto texel = [&](int x, int y) { return &m_buffer[(y * kTexWidth + x) * 4]; };

    for (size_t i = 0; i < program.prims.size(); ++i)
    {
        const csg::Primitive& p = program.prims[i];
        float* t0 = texel(int(i) * 4 + 0, 0);
        float* t1 = texel(int(i) * 4 + 1, 0);
        float* t2 = texel(int(i) * 4 + 2, 0);
        float* t3 = texel(int(i) * 4 + 3, 0);
        t0[0] = p.position.x; t0[1] = p.position.y; t0[2] = p.position.z; t0[3] = float(int(p.type));
        t1[0] = p.rotation.x; t1[1] = p.rotation.y; t1[2] = p.rotation.z; t1[3] = p.rotation.w;
        t2[0] = p.size.x; t2[1] = p.size.y; t2[2] = p.size.z; t2[3] = p.reflectivity;
        if (p.type == csg::PrimType::Mesh)
        {
            const MeshSlot* s = p.mesh ? slot(p.mesh.get()) : nullptr;
            t2[1] = s ? float(s->nodeBase) : -1.0f;
            t2[2] = s ? float(s->triBase) : -1.0f;
        }
        t3[0] = p.color.r; t3[1] = p.color.g; t3[2] = p.color.b;
        t3[3] = highlightedNodes.count(p.nodeId) ? 1.0f : 0.0f;
    }
    for (size_t i = 0; i < program.ops.size(); ++i)
    {
        float* t = texel(int(i), 1);
        t[0] = float(int(program.ops[i].code));
        t[1] = float(program.ops[i].prim);
    }

    glBindTexture(GL_TEXTURE_2D, m_data);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, kTexWidth, 2, GL_RGBA, GL_FLOAT, m_buffer.data());
    m_primCount = int(program.prims.size());
    m_opCount = int(program.ops.size());
    return true;
}

void RayRenderer::draw(const Camera& camera, int width, int height, const RaySettings& settings)
{
    Shader* variantShader = variant(settings.mode, m_useMesh);
    if (!variantShader)
        return;
    Shader& shader = *variantShader;
    float aspect = float(width) / float(height);

    shader.use();
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, m_data);
    shader.set("uData", 0);
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_BUFFER, m_nodeTex);
    shader.set("uMeshNodes", 1);
    glActiveTexture(GL_TEXTURE2);
    glBindTexture(GL_TEXTURE_BUFFER, m_triTex);
    shader.set("uMeshTris", 2);
    glActiveTexture(GL_TEXTURE0);
    shader.set("uPrimCount", m_primCount);
    shader.set("uOpCount", m_opCount);
    shader.set("uMode", int(settings.mode));
    shader.set("uBounces", settings.bounces);
    shader.set("uLightDir", glm::normalize(settings.lightDir));
    shader.set("uGround", settings.ground ? 1 : 0);
    shader.set("uGroundY", settings.groundY);
    shader.set("uSmcAlpha", settings.smcAlpha);
    shader.set("uEye", camera.eye());
    shader.set("uInvViewProj", glm::inverse(camera.projection(aspect) * camera.view()));
    shader.set("uResolution", glm::vec2(width, height));
    shader.set("uBoundsMin", m_boundsMin);
    shader.set("uBoundsMax", m_boundsMax);

    glDisable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);
    glBindVertexArray(m_vao);
    glDrawArrays(GL_TRIANGLES, 0, 3);
    glBindVertexArray(0);
}

} // namespace render
