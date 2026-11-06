#include "render/Shader.h"

#include <glm/gtc/type_ptr.hpp>

#include <fstream>
#include <sstream>
#include <unistd.h>

namespace render {

namespace {

std::string executableDir()
{
    char buf[4096];
    ssize_t n = readlink("/proc/self/exe", buf, sizeof(buf) - 1);
    if (n <= 0)
        return ".";
    std::string path(buf, size_t(n));
    return path.substr(0, path.find_last_of('/'));
}

bool readFile(const std::string& path, std::string& out)
{
    std::ifstream in(path);
    if (!in)
        return false;
    std::stringstream ss;
    ss << in.rdbuf();
    out = ss.str();
    return true;
}

GLuint compile(GLenum type, const std::string& source, const std::string& file, std::string& error)
{
    GLuint shader = glCreateShader(type);
    const char* src = source.c_str();
    glShaderSource(shader, 1, &src, nullptr);
    glCompileShader(shader);
    GLint ok = 0;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &ok);
    if (!ok)
    {
        char log[4096];
        glGetShaderInfoLog(shader, sizeof(log), nullptr, log);
        error = file + ":\n" + log;
        glDeleteShader(shader);
        return 0;
    }
    return shader;
}

} // namespace

std::string shaderPath(const std::string& name)
{
    const std::string exe = executableDir();
    for (const std::string& dir : { exe + "/shaders", exe + "/../shaders", std::string("shaders") })
    {
        std::ifstream probe(dir + "/" + name);
        if (probe)
            return dir + "/" + name;
    }
    return "shaders/" + name;
}

void Shader::release()
{
    if (m_program)
        glDeleteProgram(m_program);
    m_program = 0;
}

bool Shader::load(const std::string& vertexFile, const std::string& fragmentFile)
{
    std::string vs, fs;
    if (!readFile(shaderPath(vertexFile), vs) || !readFile(shaderPath(fragmentFile), fs))
    {
        m_error = "cannot read " + vertexFile + " / " + fragmentFile;
        return false;
    }

    GLuint v = compile(GL_VERTEX_SHADER, vs, vertexFile, m_error);
    if (!v)
        return false;
    GLuint f = compile(GL_FRAGMENT_SHADER, fs, fragmentFile, m_error);
    if (!f)
    {
        glDeleteShader(v);
        return false;
    }

    GLuint program = glCreateProgram();
    glAttachShader(program, v);
    glAttachShader(program, f);
    glLinkProgram(program);
    glDeleteShader(v);
    glDeleteShader(f);

    GLint ok = 0;
    glGetProgramiv(program, GL_LINK_STATUS, &ok);
    if (!ok)
    {
        char log[4096];
        glGetProgramInfoLog(program, sizeof(log), nullptr, log);
        m_error = std::string("link:\n") + log;
        glDeleteProgram(program);
        return false;
    }

    if (m_program)
        glDeleteProgram(m_program);
    m_program = program;
    m_error.clear();
    return true;
}

void Shader::use() const { glUseProgram(m_program); }
GLint Shader::location(const char* name) const { return glGetUniformLocation(m_program, name); }
void Shader::set(const char* name, int v) const { glUniform1i(location(name), v); }
void Shader::set(const char* name, float v) const { glUniform1f(location(name), v); }
void Shader::set(const char* name, const glm::vec2& v) const { glUniform2fv(location(name), 1, glm::value_ptr(v)); }
void Shader::set(const char* name, const glm::vec3& v) const { glUniform3fv(location(name), 1, glm::value_ptr(v)); }
void Shader::set(const char* name, const glm::mat4& m) const { glUniformMatrix4fv(location(name), 1, GL_FALSE, glm::value_ptr(m)); }

} // namespace render
