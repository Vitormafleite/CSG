#pragma once

#include "csg/Node.h"
#include "csg/Smc.h"

#include <functional>
#include <memory>
#include <vector>

namespace csg {

// The model: a forest of CSG trees plus the current time.
class Scene
{
public:
    std::vector<std::unique_ptr<Node>> roots;

    float time = 0.0f;
    float duration = 5.0f;
    bool autoKey = false; // edits create keyframes at the current time

    // Tree generators
    Node* addPrimitive(PrimType type);
    // Combines two root trees into a new root (A op B). Returns null if invalid.
    Node* combine(int idA, int idB, NodeType op);
    // Splits an operation root back into its two subtrees.
    bool ungroup(int id);
    bool removeRoot(int id);
    void clear();

    Node* find(int id) const;
    Node* rootOf(int id) const;
    bool isRoot(int id) const;

    // Geometric operations on a whole tree (global modeling): applied to every
    // primitive of the subtree, keyed at the current time if animating.
    void translate(Node* tree, const glm::vec3& delta);
    void rotate(Node* tree, const glm::quat& q, const glm::vec3& pivot);
    void scale(Node* tree, float factor, const glm::vec3& pivot); // scales size parameters
    void keyAll(Node* tree);          // keys the current values at the current time
    void removeKeys(Node* tree);      // removes keys at the current time
    void clearAnimation(Node* tree);  // makes the tree static at its current pose

    // Snapshot of a tree at time t, flattened for SMC.
    Program program(const Node* tree, float t) const;
    // All trees combined by union (what the viewport displays).
    Program displayProgram(float t) const;

    Aabb bounds(const Node* tree, float t) const;
    static void forEachPrimitive(Node* tree, const std::function<void(Node&)>& fn);
    static void forEachPrimitive(const Node* tree, const std::function<void(const Node&)>& fn);

    void loadDemo();

private:
    int m_nextId = 1;
    int m_counters[4] = {};

    std::unique_ptr<Node> takeRoot(int id);
    static void flatten(const Node* node, float t, Program& out);
};

} // namespace csg
