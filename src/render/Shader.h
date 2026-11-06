#pragma once

#include "render/Gl.h"

#include <glm/glm.hpp>

#include <string>

namespace render {

// Finds a file in the shaders/ directory next to the executable or the working directory.
std::string shaderPath(const std::string& name);

class Shader
{
public:
    Shader() = default;
    ~Shader() { release(); }
    Shader(const Shader&) = delete;
    Shader& operator=(const Shader&) = delete;

    // Compiles and links; on failure keeps the previous program and returns false.
    bool load(const std::string& vertexFile, const std::string& fragmentFile);
    const std::string& error() const { return m_error; }
    bool valid() const { return m_program != 0; }
    // Frees GL objects; must run while the GL context is alive.
    void release();

    void use() const;
    GLint location(const char* name) const;
    void set(const char* name, int v) const;
    void set(const char* name, float v) const;
    void set(const char* name, const glm::vec2& v) const;
    void set(const char* name, const glm::vec3& v) const;
    void set(const char* name, const glm::mat4& m) const;

private:
    GLuint m_program = 0;
    std::string m_error;
};

} // namespace render
