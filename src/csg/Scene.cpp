#include "csg/Scene.h"

#include "csg/Mesh.h"

#include <glm/gtc/constants.hpp>

#include <algorithm>
#include <filesystem>
#include <string>

namespace csg {

namespace {

const glm::vec3 kPalette[] = {
    { 0.90f, 0.45f, 0.35f }, { 0.35f, 0.65f, 0.90f }, { 0.55f, 0.85f, 0.45f },
    { 0.95f, 0.80f, 0.35f }, { 0.75f, 0.50f, 0.90f }, { 0.40f, 0.85f, 0.80f },
};

OpCode opCodeFor(NodeType type)
{
    switch (type)
    {
    case NodeType::Union: return OpCode::Union;
    case NodeType::Intersection: return OpCode::Intersection;
    default: return OpCode::Difference;
    }
}

Node* findIn(Node* node, int id)
{
    if (!node)
        return nullptr;
    if (node->id == id)
        return node;
    if (Node* n = findIn(node->a.get(), id))
        return n;
    return findIn(node->b.get(), id);
}

} // namespace

const char* nodeTypeName(NodeType type)
{
    switch (type)
    {
    case NodeType::Primitive: return "Primitive";
    case NodeType::Union: return "Union";
    case NodeType::Intersection: return "Intersection";
    case NodeType::Difference: return "Difference";
    }
    return "?";
}

Primitive Node::evaluate(float t) const
{
    Primitive p;
    p.type = prim;
    p.position = position.value(t);
    p.rotation = glm::normalize(rotation.value(t));
    p.size = glm::max(size.value(t), glm::vec3(1e-3f));
    p.color = color;
    p.reflectivity = reflectivity;
    p.nodeId = id;
    p.mesh = mesh;
    return p;
}

Node* Scene::addPrimitive(PrimType type)
{
    auto node = std::make_unique<Node>();
    node->id = m_nextId++;
    node->type = NodeType::Primitive;
    node->prim = type;
    int n = ++m_counters[int(type)];
    node->name = std::string(primTypeName(type)) + " " + std::to_string(n);
    node->color = kPalette[(node->id - 1) % (sizeof(kPalette) / sizeof(kPalette[0]))];

    switch (type)
    {
    case PrimType::Block: node->size = Track<glm::vec3>(glm::vec3(1.0f)); break;
    case PrimType::Sphere: node->size = Track<glm::vec3>(glm::vec3(0.6f, 0.0f, 0.0f)); break;
    case PrimType::Cylinder: node->size = Track<glm::vec3>(glm::vec3(0.4f, 1.2f, 0.0f)); break;
    case PrimType::Cone: node->size = Track<glm::vec3>(glm::vec3(0.5f, 1.2f, 0.0f)); break;
    case PrimType::Mesh: node->size = Track<glm::vec3>(glm::vec3(2.0f, 0.0f, 0.0f)); break;
    }

    node->visibleFrom = 0.0f;
    node->visibleTo = duration;
    roots.push_back(std::move(node));
    return roots.back().get();
}

Node* Scene::addMesh(const std::string& path, std::string& error)
{
    std::shared_ptr<const Mesh> mesh = Mesh::load(path, error);
    if (!mesh)
        return nullptr;
    Node* node = addPrimitive(PrimType::Mesh);
    node->meshPath = path;
    node->mesh = std::move(mesh);
    node->name = std::filesystem::path(path).stem().string() + " " + std::to_string(m_counters[int(PrimType::Mesh)]);
    node->color = glm::vec3(0.80f, 0.62f, 0.30f);
    return node;
}

void Scene::assignIds(Node* node)
{
    if (!node)
        return;
    node->id = m_nextId++;
    if (node->isPrimitive())
        ++m_counters[int(node->prim)];
    assignIds(node->a.get());
    assignIds(node->b.get());
}

void Scene::adopt(std::vector<std::unique_ptr<Node>> trees)
{
    clear();
    roots = std::move(trees);
    for (auto& root : roots)
        assignIds(root.get());
}

std::unique_ptr<Node> Scene::takeRoot(int id)
{
    auto it = std::find_if(roots.begin(), roots.end(), [id](const auto& r) { return r->id == id; });
    if (it == roots.end())
        return nullptr;
    std::unique_ptr<Node> node = std::move(*it);
    roots.erase(it);
    return node;
}

Node* Scene::combine(int idA, int idB, NodeType op)
{
    if (op == NodeType::Primitive || idA == idB || !isRoot(idA) || !isRoot(idB))
        return nullptr;

    // Keep the new tree where A was in the list.
    size_t index = std::find_if(roots.begin(), roots.end(), [idA](const auto& r) { return r->id == idA; }) - roots.begin();

    auto node = std::make_unique<Node>();
    node->id = m_nextId++;
    node->type = op;
    node->a = takeRoot(idA);
    node->b = takeRoot(idB);
    node->name = std::string(nodeTypeName(op)) + " " + std::to_string(node->id);

    index = std::min(index, roots.size());
    roots.insert(roots.begin() + index, std::move(node));
    return roots[index].get();
}

bool Scene::ungroup(int id)
{
    auto it = std::find_if(roots.begin(), roots.end(), [id](const auto& r) { return r->id == id; });
    if (it == roots.end() || (*it)->isPrimitive())
        return false;
    std::unique_ptr<Node> a = std::move((*it)->a);
    std::unique_ptr<Node> b = std::move((*it)->b);
    *it = std::move(a);
    roots.insert(it + 1, std::move(b));
    return true;
}

bool Scene::removeRoot(int id)
{
    return takeRoot(id) != nullptr;
}

void Scene::clear()
{
    roots.clear();
    m_nextId = 1;
    std::fill(std::begin(m_counters), std::end(m_counters), 0);
}

Node* Scene::find(int id) const
{
    for (const auto& root : roots)
        if (Node* n = findIn(root.get(), id))
            return n;
    return nullptr;
}

Node* Scene::rootOf(int id) const
{
    for (const auto& root : roots)
        if (findIn(root.get(), id))
            return root.get();
    return nullptr;
}

bool Scene::isRoot(int id) const
{
    return std::any_of(roots.begin(), roots.end(), [id](const auto& r) { return r->id == id; });
}

void Scene::forEachPrimitive(Node* tree, const std::function<void(Node&)>& fn)
{
    if (!tree)
        return;
    if (tree->isPrimitive())
    {
        fn(*tree);
        return;
    }
    forEachPrimitive(tree->a.get(), fn);
    forEachPrimitive(tree->b.get(), fn);
}

void Scene::forEachPrimitive(const Node* tree, const std::function<void(const Node&)>& fn)
{
    forEachPrimitive(const_cast<Node*>(tree), [&](Node& n) { fn(n); });
}

void Scene::translate(Node* tree, const glm::vec3& delta)
{
    forEachPrimitive(tree, [&](Node& n) { n.position.set(time, n.position.value(time) + delta, autoKey); });
}

void Scene::rotate(Node* tree, const glm::quat& q, const glm::vec3& pivot)
{
    forEachPrimitive(tree, [&](Node& n) {
        n.position.set(time, pivot + q * (n.position.value(time) - pivot), autoKey);
        n.rotation.set(time, glm::normalize(q * n.rotation.value(time)), autoKey);
    });
}

void Scene::scale(Node* tree, float factor, const glm::vec3& pivot)
{
    forEachPrimitive(tree, [&](Node& n) {
        n.position.set(time, pivot + factor * (n.position.value(time) - pivot), autoKey);
        n.size.set(time, n.size.value(time) * factor, autoKey);
    });
}

void Scene::keyAll(Node* tree)
{
    forEachPrimitive(tree, [&](Node& n) {
        n.position.set(time, n.position.value(time), true);
        n.rotation.set(time, n.rotation.value(time), true);
        n.size.set(time, n.size.value(time), true);
    });
}

void Scene::removeKeys(Node* tree)
{
    forEachPrimitive(tree, [&](Node& n) {
        n.position.removeKeyAt(time);
        n.rotation.removeKeyAt(time);
        n.size.removeKeyAt(time);
    });
}

void Scene::clearAnimation(Node* tree)
{
    forEachPrimitive(tree, [&](Node& n) {
        n.position.clearKeys(time);
        n.rotation.clearKeys(time);
        n.size.clearKeys(time);
    });
}

void Scene::flatten(const Node* node, float t, Program& out)
{
    if (node->isPrimitive())
    {
        if (node->visibleAt(t))
        {
            out.ops.push_back({ OpCode::Push, int(out.prims.size()) });
            out.prims.push_back(node->evaluate(t));
        }
        else
        {
            out.ops.push_back({ OpCode::Empty, -1 });
        }
        return;
    }
    flatten(node->a.get(), t, out);
    flatten(node->b.get(), t, out);
    out.ops.push_back({ opCodeFor(node->type), -1 });
}

Program Scene::program(const Node* tree, float t) const
{
    Program p;
    if (tree)
        flatten(tree, t, p);
    return p;
}

Program Scene::displayProgram(float t) const
{
    Program p;
    for (size_t i = 0; i < roots.size(); ++i)
    {
        flatten(roots[i].get(), t, p);
        if (i > 0)
            p.ops.push_back({ OpCode::Union, -1 });
    }
    return p;
}

Aabb Scene::bounds(const Node* tree, float t) const
{
    Aabb box;
    forEachPrimitive(tree, [&](const Node& n) { box.expand(n.evaluate(t).bounds()); });
    return box;
}

void Scene::loadDemo()
{
    clear();
    const float halfPi = glm::half_pi<float>();

    // (Block ∩ Sphere) − (Cylinder X ∪ Cylinder Y ∪ Cylinder Z)
    Node* block = addPrimitive(PrimType::Block);
    block->size = Track<glm::vec3>(glm::vec3(1.5f));
    Node* sphere = addPrimitive(PrimType::Sphere);
    sphere->size = Track<glm::vec3>(glm::vec3(1.0f, 0.0f, 0.0f));
    Node* core = combine(block->id, sphere->id, NodeType::Intersection);

    Node* cyl[3];
    for (int i = 0; i < 3; ++i)
    {
        cyl[i] = addPrimitive(PrimType::Cylinder);
        cyl[i]->size = Track<glm::vec3>(glm::vec3(0.45f, 2.0f, 0.0f));
        cyl[i]->color = glm::vec3(0.35f, 0.65f, 0.90f);
    }
    cyl[0]->rotation = Track<glm::quat>(glm::angleAxis(halfPi, glm::vec3(0, 0, 1)));
    cyl[2]->rotation = Track<glm::quat>(glm::angleAxis(halfPi, glm::vec3(1, 0, 0)));
    Node* cross = combine(cyl[0]->id, cyl[1]->id, NodeType::Union);
    cross = combine(cross->id, cyl[2]->id, NodeType::Union);
    combine(core->id, cross->id, NodeType::Difference);

    // An animated cone: bobs up and spins over the timeline.
    Node* cone = addPrimitive(PrimType::Cone);
    cone->color = glm::vec3(0.95f, 0.80f, 0.35f);
    cone->position = Track<glm::vec3>(glm::vec3(2.2f, 0.0f, 0.0f));
    cone->position.set(duration * 0.5f, glm::vec3(2.2f, 1.0f, 0.0f), true);
    cone->position.set(duration, glm::vec3(2.2f, 0.0f, 0.0f), true);
    cone->rotation.set(duration * 0.5f, glm::angleAxis(glm::pi<float>() * 0.5f, glm::vec3(0, 0, 1)), true);
    cone->rotation.set(duration, glm::quat(1, 0, 0, 0), true);
}

} // namespace csg
