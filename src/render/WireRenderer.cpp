#include "render/WireRenderer.h"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace render {

void WireRenderer::release()
{
    if (m_vbo)
        glDeleteBuffers(1, &m_vbo);
    if (m_vao)
        glDeleteVertexArrays(1, &m_vao);
    m_vbo = m_vao = 0;
    m_shader.release();
}

bool WireRenderer::init()
{
    glGenVertexArrays(1, &m_vao);
    glGenBuffers(1, &m_vbo);
    glBindVertexArray(m_vao);
    glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, pos));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, color));
    glBindVertexArray(0);
    return reload();
}

bool WireRenderer::reload()
{
    return m_shader.load("line.vert", "line.frag");
}

void WireRenderer::line(const glm::vec3& a, const glm::vec3& b, const glm::vec3& color)
{
    m_vertices.push_back({ a, color });
    m_vertices.push_back({ b, color });
}

void WireRenderer::curve(const csg::Curve& c, float s0, float s1, const glm::vec3& color)
{
    int segments = c.circle ? std::max(1, int(std::ceil((s1 - s0) * 96.0f))) : 1;
    glm::vec3 prev = c.point(s0);
    for (int i = 1; i <= segments; ++i)
    {
        glm::vec3 p = c.point(s0 + (s1 - s0) * float(i) / segments);
        line(prev, p, color);
        prev = p;
    }
}

void WireRenderer::marker(const glm::vec3& p, float size, const glm::vec3& color)
{
    for (int axis = 0; axis < 3; ++axis)
    {
        glm::vec3 d(0.0f);
        d[axis] = size;
        line(p - d, p + d, color);
    }
}

void WireRenderer::draw(const glm::mat4& viewProj)
{
    if (m_vertices.empty() || !m_shader.valid())
        return;
    glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
    glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(m_vertices.size() * sizeof(Vertex)), m_vertices.data(), GL_STREAM_DRAW);

    m_shader.use();
    m_shader.set("uViewProj", viewProj);
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glBindVertexArray(m_vao);
    glDrawArrays(GL_LINES, 0, GLsizei(m_vertices.size()));
    glBindVertexArray(0);
    glDisable(GL_BLEND);
}

} // namespace render
