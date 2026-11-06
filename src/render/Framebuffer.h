#pragma once

#include "render/Gl.h"

namespace render {

// Offscreen color target shown inside the ImGui viewport window.
class Framebuffer
{
public:
    ~Framebuffer() { release(); }
    // Frees GL objects; must run while the GL context is alive.
    void release();

    // (Re)allocates storage when the size changes. The texture id stays the same.
    void resize(int width, int height);
    void bind() const;
    static void unbind();

    GLuint texture() const { return m_color; }
    int width() const { return m_width; }
    int height() const { return m_height; }

private:
    GLuint m_fbo = 0, m_color = 0;
    int m_width = 0, m_height = 0;
};

} // namespace render
