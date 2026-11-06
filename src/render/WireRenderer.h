#pragma once

#include "csg/Primitive.h"
#include "render/Shader.h"

#include <vector>

namespace render {

// Colored line batches: SMC-classified wireframe edges, the probe ray, markers.
class WireRenderer
{
public:
    ~WireRenderer() { release(); }

    bool init();
    bool reload();
    // Frees GL objects; must run while the GL context is alive.
    void release();
    void clear() { m_vertices.clear(); }

    void line(const glm::vec3& a, const glm::vec3& b, const glm::vec3& color);
    // Part [s0, s1] of a curve, tessellated.
    void curve(const csg::Curve& c, float s0, float s1, const glm::vec3& color);
    // Small 3D cross.
    void marker(const glm::vec3& p, float size, const glm::vec3& color);

    void draw(const glm::mat4& viewProj);

private:
    struct Vertex
    {
        glm::vec3 pos;
        glm::vec3 color;
    };

    Shader m_shader;
    GLuint m_vao = 0, m_vbo = 0;
    std::vector<Vertex> m_vertices;
};

} // namespace render
