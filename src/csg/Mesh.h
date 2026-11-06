#pragma once

#include "csg/Primitive.h"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace csg {

// Node of the mesh BVH. Interior nodes (count == 0) have their children at
// leftOrFirst and leftOrFirst + 1; leaves hold triangles [leftOrFirst, leftOrFirst + count).
// The layout is mirrored in shaders/csg.frag (2 texels per node).
struct BvhNode
{
    glm::vec3 min;
    uint32_t leftOrFirst;
    glm::vec3 max;
    uint32_t count;
};

// One crossing of a line with the mesh surface.
struct MeshHit
{
    float t;
    int tri;
    bool entering; // the line enters the solid here (direction against the outward normal)
};

// A closed triangle mesh (B-rep) used as a solid primitive: the solid is the
// region the surface encloses. Loaded from STL, normalized to local space
// (centered on its bounding box, largest extent 1) and indexed by a BVH.
//
// Line classification: every crossing along the line is found, sorted, and
// a winding count (+1 entering, -1 leaving) gives the IN spans where it is > 0.
class Mesh
{
public:
    // Loads (or returns the cached copy of) an STL file, binary or ASCII.
    static std::shared_ptr<const Mesh> load(const std::string& path, std::string& error);

    const std::string& path() const { return m_path; }
    size_t triangleCount() const { return m_normals.size(); }

    // Local space data, triangles in BVH order.
    const std::vector<glm::vec3>& vertices() const { return m_vertices; } // 3 per triangle
    const std::vector<glm::vec3>& normals() const { return m_normals; }   // outward face normals
    const std::vector<BvhNode>& nodes() const { return m_nodes; }
    glm::vec3 halfExtent() const { return m_halfExtent; }
    // Scale applied to the file's coordinates to normalize them.
    float fileScale() const { return m_fileScale; }

    double volume() const { return m_volume; } // local units (divergence theorem)
    double area() const { return m_area; }
    // Watertightness: edges not shared by exactly two triangles, and edges used
    // twice in the same direction (inconsistent orientation).
    int openEdges() const { return m_openEdges; }
    int flippedEdges() const { return m_flippedEdges; }
    bool closed() const { return m_openEdges == 0 && m_flippedEdges == 0; }

    // All crossings of the line o + t*d (local space), sorted by t.
    void hits(const glm::vec3& o, const glm::vec3& d, std::vector<MeshHit>& out) const;
    // IN spans (t0, t1) of the line from the winding count.
    void spans(const glm::vec3& o, const glm::vec3& d, std::vector<glm::vec2>& out) const;

    // Unsigned distance to the surface and the closest triangle.
    float distance(const glm::vec3& p, int& tri) const;
    bool inside(const glm::vec3& p) const;
    float sdf(const glm::vec3& p) const;
    glm::vec3 normalAt(const glm::vec3& p) const;

    // Uniform point on the surface (local space).
    glm::vec3 surfacePoint(float r0, float r1, float r2) const;

    // Wireframe: polylines where axis-aligned planes slice the surface (local space).
    const std::vector<Curve>& contours() const { return m_contours; }

private:
    std::string m_path;
    std::vector<glm::vec3> m_vertices;
    std::vector<glm::vec3> m_normals;
    std::vector<BvhNode> m_nodes;
    std::vector<double> m_areaCdf;
    std::vector<Curve> m_contours;
    glm::vec3 m_halfExtent{ 0.5f };
    float m_fileScale = 1.0f;
    double m_volume = 0.0, m_area = 0.0;
    int m_openEdges = 0, m_flippedEdges = 0;

    bool readStl(const std::string& path, std::string& error);
    void normalize();
    void checkTopology();
    void buildBvh();
    void buildContours();
};

} // namespace csg
