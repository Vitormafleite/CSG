#pragma once

#include "csg/Primitive.h"
#include "csg/Track.h"

#include <memory>
#include <string>

namespace csg {

enum class NodeType { Primitive, Union, Intersection, Difference };

const char* nodeTypeName(NodeType type);

// CSG tree node: a primitive leaf or a set operation on two subtrees.
//
// Modeling is global: every primitive stores its own world placement, and
// geometric operations on a tree are pushed down to all of its primitives.
// Placement and size are keyframed tracks, evaluated at a given time.
struct Node
{
    int id = 0;
    NodeType type = NodeType::Primitive;
    std::string name;

    // Primitive data
    PrimType prim = PrimType::Block;
    Track<glm::vec3> position{ glm::vec3(0.0f) };
    Track<glm::quat> rotation{ glm::quat(1.0f, 0.0f, 0.0f, 0.0f) };
    Track<glm::vec3> size{ glm::vec3(1.0f) };
    glm::vec3 color{ 0.8f };
    float reflectivity = 0.0f;
    bool limitedLife = false; // exists only during [visibleFrom, visibleTo]
    float visibleFrom = 0.0f;
    float visibleTo = 1.0f;
    std::string meshPath;                 // Mesh: the STL file
    std::shared_ptr<const Mesh> mesh;     // Mesh: loaded geometry (shared by nodes with the same file)

    // Operation children (A op B)
    std::unique_ptr<Node> a, b;

    bool isPrimitive() const { return type == NodeType::Primitive; }
    bool visibleAt(float t) const { return !limitedLife || (t >= visibleFrom && t <= visibleTo); }
    Primitive evaluate(float t) const;
};

} // namespace csg
