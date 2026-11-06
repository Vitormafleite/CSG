#include "render/RayRenderer.h"

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
    m_data = m_vao = 0;
    m_shader.release();
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
    return reload();
}

bool RayRenderer::reload()
{
    return m_shader.load("fullscreen.vert", "csg.frag");
}

bool RayRenderer::upload(const csg::Program& program, const std::unordered_set<int>& highlightedNodes)
{
    if (int(program.prims.size()) > kMaxPrims || int(program.ops.size()) > kMaxOps ||
        program.maxStackDepth() > kMaxStack)
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
    if (!m_shader.valid())
        return;
    float aspect = float(width) / float(height);

    m_shader.use();
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, m_data);
    m_shader.set("uData", 0);
    m_shader.set("uPrimCount", m_primCount);
    m_shader.set("uOpCount", m_opCount);
    m_shader.set("uMode", int(settings.mode));
    m_shader.set("uBounces", settings.bounces);
    m_shader.set("uLightDir", glm::normalize(settings.lightDir));
    m_shader.set("uGround", settings.ground ? 1 : 0);
    m_shader.set("uGroundY", settings.groundY);
    m_shader.set("uSmcAlpha", settings.smcAlpha);
    m_shader.set("uEye", camera.eye());
    m_shader.set("uInvViewProj", glm::inverse(camera.projection(aspect) * camera.view()));
    m_shader.set("uResolution", glm::vec2(width, height));

    glDisable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);
    glBindVertexArray(m_vao);
    glDrawArrays(GL_TRIANGLES, 0, 3);
    glBindVertexArray(0);
}

} // namespace render
