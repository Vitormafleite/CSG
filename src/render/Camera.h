#pragma once

#include <glm/glm.hpp>

namespace render {

// Orbit camera around a target point.
struct Camera
{
    glm::vec3 target{ 0.0f };
    float yaw = 0.7f;
    float pitch = 0.45f;
    float distance = 7.0f;
    float fovY = glm::radians(45.0f);

    glm::vec3 eye() const;
    glm::mat4 view() const;
    glm::mat4 projection(float aspect) const;

    void orbit(float dx, float dy);
    void pan(float dx, float dy);
    void zoom(float steps);

    // World-space ray through a point given in normalized device coordinates.
    void ray(const glm::vec2& ndc, float aspect, glm::vec3& origin, glm::vec3& dir) const;
};

} // namespace render
