#include "render/Framebuffer.h"

namespace render {

void Framebuffer::release()
{
    if (m_color)
        glDeleteTextures(1, &m_color);
    if (m_fbo)
        glDeleteFramebuffers(1, &m_fbo);
    m_color = m_fbo = 0;
    m_width = m_height = 0;
}

void Framebuffer::resize(int width, int height)
{
    if (width < 1 || height < 1 || (width == m_width && height == m_height))
        return;
    m_width = width;
    m_height = height;

    if (!m_fbo)
    {
        glGenFramebuffers(1, &m_fbo);
        glGenTextures(1, &m_color);
    }

    glBindTexture(GL_TEXTURE_2D, m_color);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    glBindFramebuffer(GL_FRAMEBUFFER, m_fbo);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, m_color, 0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void Framebuffer::bind() const
{
    glBindFramebuffer(GL_FRAMEBUFFER, m_fbo);
    glViewport(0, 0, m_width, m_height);
}

void Framebuffer::unbind()
{
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

} // namespace render
