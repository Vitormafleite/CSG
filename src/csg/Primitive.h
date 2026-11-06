#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <memory>
#include <vector>

namespace csg {

class Mesh;

// Primitive types. The numeric values are shared with shaders/csg.frag.
enum class PrimType : int { Block = 0, Sphere = 1, Cylinder = 2, Cone = 3, Mesh = 4 };

const char* primTypeName(PrimType type);

// Axis-aligned bounding box in world space.
struct Aabb
{
    glm::vec3 min{ 1e30f };
    glm::vec3 max{ -1e30f };

    void expand(const Aabb& other);
    bool empty() const { return min.x > max.x; }
    glm::vec3 center() const { return (min + max) * 0.5f; }
};

// Parametric wireframe edge: a line segment or a full circle.
struct Curve
{
    bool circle = false;
    glm::vec3 a{ 0.0f }, b{ 0.0f };       // line endpoints
    glm::vec3 center{ 0.0f }, u{ 0.0f }, v{ 0.0f }; // circle frame
    float radius = 0.0f;
    int samples = 64; // edge SMC samples (1 for mesh contour chords: only their ends are on the surface)

    glm::vec3 point(float s) const; // s in [0, 1]
};

// A primitive evaluated at some instant: rigid placement plus size parameters.
//
// Size parameters (scale is given by them, not by a matrix):
//   Block:    size = (width, height, depth)
//   Sphere:   size.x = radius
//   Cylinder: size.x = radius, size.y = height  (axis Y, centered)
//   Cone:     size.x = base radius, size.y = height (axis Y, base at -h/2, apex at +h/2)
//   Mesh:     size.x = largest extent (uniform scale of the normalized mesh)
struct Primitive
{
    PrimType type = PrimType::Block;
    glm::vec3 position{ 0.0f };
    glm::quat rotation{ 1.0f, 0.0f, 0.0f, 0.0f };
    glm::vec3 size{ 1.0f };
    glm::vec3 color{ 0.8f };
    float reflectivity = 0.0f;
    int nodeId = -1;
    std::shared_ptr<const Mesh> mesh; // Mesh only

    // Mesh local space: rigid placement, then the uniform scale.
    glm::vec3 toMesh(const glm::vec3& p) const { return toLocal(p) / size.x; }

    glm::vec3 toLocal(const glm::vec3& p) const;
    glm::vec3 dirToLocal(const glm::vec3& d) const;

    // Ray/line against the solid: the parameter interval [t0, t1] inside it.
    // The direction need not be normalized (transforms are rigid, so t is preserved).
    // A mesh may be non-convex: this gives the hull of its spans; spans() gives all of them.
    bool intersect(const glm::vec3& o, const glm::vec3& d, float& t0, float& t1) const;
    // All IN spans (t0, t1) of the line, sorted.
    void spans(const glm::vec3& o, const glm::vec3& d, std::vector<glm::vec2>& out) const;

    // Signed distance (negative inside). Used for point membership classification.
    float sdf(const glm::vec3& p) const;

    // Outward unit normal at a surface point (world space).
    glm::vec3 normal(const glm::vec3& p) const;

    float volume() const;
    float area() const;

    // Uniform point on the surface from three numbers in [0, 1).
    glm::vec3 surfacePoint(float r0, float r1, float r2) const;

    Aabb bounds() const;
    std::vector<Curve> edges() const;
};

} // namespace csg
