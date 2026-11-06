#include "render/Camera.h"

#include <glm/gtc/matrix_transform.hpp>

#include <algorithm>
#include <cmath>

namespace render {

glm::vec3 Camera::eye() const
{
    glm::vec3 dir(std::cos(pitch) * std::sin(yaw), std::sin(pitch), std::cos(pitch) * std::cos(yaw));
    return target + distance * dir;
}

glm::mat4 Camera::view() const
{
    return glm::lookAt(eye(), target, glm::vec3(0.0f, 1.0f, 0.0f));
}

glm::mat4 Camera::projection(float aspect) const
{
    return glm::perspective(fovY, aspect, 0.05f, 500.0f);
}

void Camera::orbit(float dx, float dy)
{
    yaw -= dx * 0.008f;
    pitch = std::clamp(pitch + dy * 0.008f, -1.55f, 1.55f);
}

void Camera::pan(float dx, float dy)
{
    glm::mat4 v = view();
    glm::vec3 right(v[0][0], v[1][0], v[2][0]);
    glm::vec3 up(v[0][1], v[1][1], v[2][1]);
    float scale = distance * 0.0015f;
    target += (-dx * right + dy * up) * scale;
}

void Camera::zoom(float steps)
{
    distance = std::clamp(distance * std::pow(0.88f, steps), 0.5f, 200.0f);
}

void Camera::ray(const glm::vec2& ndc, float aspect, glm::vec3& origin, glm::vec3& dir) const
{
    glm::mat4 inv = glm::inverse(projection(aspect) * view());
    glm::vec4 far = inv * glm::vec4(ndc, 1.0f, 1.0f);
    origin = eye();
    dir = glm::normalize(glm::vec3(far) / far.w - origin);
}

} // namespace render
