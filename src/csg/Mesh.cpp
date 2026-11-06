#include "csg/Mesh.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <map>
#include <mutex>
#include <sstream>
#include <unordered_map>

namespace csg {

namespace {

constexpr float kInf = 1e30f;
constexpr int kLeafSize = 4;
constexpr int kBins = 16;

// Line/triangle tolerances, in local units (the mesh spans 1 unit). They are
// mirrored in shaders/csg.frag.
constexpr float kBaryEps = 2e-4f;  // barycentric slack, so no line slips between two triangles
constexpr float kMergeLen = 1e-5f; // crossings closer than this along the line are the same point

struct Box
{
    glm::vec3 min{ kInf }, max{ -kInf };
    void grow(const glm::vec3& p)
    {
        min = glm::min(min, p);
        max = glm::max(max, p);
    }
    void grow(const Box& b)
    {
        min = glm::min(min, b.min);
        max = glm::max(max, b.max);
    }
    float area() const
    {
        glm::vec3 e = max - min;
        return e.x < 0.0f ? 0.0f : 2.0f * (e.x * e.y + e.y * e.z + e.z * e.x);
    }
};

// Line against an AABB over the whole line (t unbounded).
bool lineBox(const glm::vec3& o, const glm::vec3& invD, const glm::vec3& bmin, const glm::vec3& bmax, float& t0, float& t1)
{
    glm::vec3 ta = (bmin - o) * invD;
    glm::vec3 tb = (bmax - o) * invD;
    glm::vec3 lo = glm::min(ta, tb), hi = glm::max(ta, tb);
    t0 = std::max(std::max(lo.x, lo.y), lo.z);
    t1 = std::min(std::min(hi.x, hi.y), hi.z);
    return t0 <= t1;
}

glm::vec3 safeInverse(const glm::vec3& d)
{
    glm::vec3 inv;
    for (int i = 0; i < 3; ++i)
        inv[i] = std::fabs(d[i]) < 1e-20f ? (d[i] < 0.0f ? -1e30f : 1e30f) : 1.0f / d[i];
    return inv;
}

float boxDistance2(const glm::vec3& p, const BvhNode& n)
{
    glm::vec3 q = glm::max(glm::max(n.min - p, p - n.max), glm::vec3(0.0f));
    return glm::dot(q, q);
}

// Closest point on triangle abc to p (Ericson, Real-Time Collision Detection 5.1.5).
glm::vec3 closestOnTriangle(const glm::vec3& p, const glm::vec3& a, const glm::vec3& b, const glm::vec3& c)
{
    glm::vec3 ab = b - a, ac = c - a, ap = p - a;
    float d1 = glm::dot(ab, ap), d2 = glm::dot(ac, ap);
    if (d1 <= 0.0f && d2 <= 0.0f)
        return a;
    glm::vec3 bp = p - b;
    float d3 = glm::dot(ab, bp), d4 = glm::dot(ac, bp);
    if (d3 >= 0.0f && d4 <= d3)
        return b;
    float vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f)
        return a + ab * (d1 / (d1 - d3));
    glm::vec3 cp = p - c;
    float d5 = glm::dot(ab, cp), d6 = glm::dot(ac, cp);
    if (d6 >= 0.0f && d5 <= d6)
        return c;
    float vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f)
        return a + ac * (d2 / (d2 - d6));
    float va = d3 * d6 - d5 * d4;
    if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f)
        return b + (c - b) * ((d4 - d3) / ((d4 - d3) + (d5 - d6)));
    float denom = 1.0f / (va + vb + vc);
    return a + ab * (vb * denom) + ac * (vc * denom);
}

uint64_t pointKey(const glm::vec3& p)
{
    // Exact position key (bit pattern of x, y, z folded to 64 bits).
    uint32_t b[3];
    std::memcpy(b, &p.x, 12);
    uint64_t h = b[0];
    h = h * 0x9E3779B97F4A7C15ull ^ b[1];
    h = h * 0x9E3779B97F4A7C15ull ^ b[2];
    return h;
}

} // namespace

// ---------------------------------------------------------------- loading

std::shared_ptr<const Mesh> Mesh::load(const std::string& path, std::string& error)
{
    static std::mutex mutex;
    static std::map<std::string, std::weak_ptr<const Mesh>> cache;

    std::error_code ec;
    std::string key = std::filesystem::weakly_canonical(path, ec).string();
    if (ec)
        key = path;

    std::lock_guard<std::mutex> lock(mutex);
    if (auto it = cache.find(key); it != cache.end())
        if (auto mesh = it->second.lock())
            return mesh;

    auto mesh = std::make_shared<Mesh>();
    mesh->m_path = path;
    if (!mesh->readStl(path, error))
        return nullptr;
    if (mesh->m_normals.empty())
    {
        error = "no triangles in " + path;
        return nullptr;
    }
    mesh->normalize();
    mesh->checkTopology();
    mesh->buildBvh();
    mesh->buildContours();
    cache[key] = mesh;
    return mesh;
}

bool Mesh::readStl(const std::string& path, std::string& error)
{
    std::ifstream file(path, std::ios::binary);
    if (!file)
    {
        error = "cannot open " + path;
        return false;
    }
    std::string data((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

    // Binary STL: 80-byte header, uint32 count, 50 bytes per triangle. Some
    // binary files also start with "solid", so the size decides.
    if (data.size() >= 84)
    {
        uint32_t count;
        std::memcpy(&count, data.data() + 80, 4);
        if (data.size() == 84 + size_t(count) * 50)
        {
            m_vertices.resize(size_t(count) * 3);
            for (size_t i = 0; i < count; ++i)
                std::memcpy(&m_vertices[i * 3], data.data() + 84 + i * 50 + 12, 36);
            m_normals.resize(count);
            return true;
        }
    }

    // ASCII STL: "vertex x y z" lines, three per facet.
    if (data.compare(0, 5, "solid") != 0)
    {
        error = "not an STL file: " + path;
        return false;
    }
    std::istringstream in(data);
    std::string word;
    while (in >> word)
    {
        if (word != "vertex")
            continue;
        glm::vec3 v;
        if (!(in >> v.x >> v.y >> v.z))
        {
            error = "malformed vertex in " + path;
            return false;
        }
        m_vertices.push_back(v);
    }
    if (m_vertices.size() % 3 != 0)
    {
        error = "incomplete facet in " + path;
        return false;
    }
    m_normals.resize(m_vertices.size() / 3);
    return true;
}

void Mesh::normalize()
{
    Box box;
    for (const glm::vec3& v : m_vertices)
        box.grow(v);
    glm::vec3 center = 0.5f * (box.min + box.max);
    glm::vec3 ext = box.max - box.min;
    float largest = std::max(std::max(ext.x, ext.y), std::max(ext.z, 1e-30f));
    m_fileScale = 1.0f / largest;
    for (glm::vec3& v : m_vertices)
        v = (v - center) * m_fileScale;
    m_halfExtent = 0.5f * ext * m_fileScale;

    // Normals from the winding (STL normals are often missing or wrong),
    // volume by the divergence theorem, area, and the area CDF for sampling.
    m_volume = m_area = 0.0;
    m_areaCdf.resize(m_normals.size());
    for (size_t i = 0; i < m_normals.size(); ++i)
    {
        glm::dvec3 a(m_vertices[i * 3]), b(m_vertices[i * 3 + 1]), c(m_vertices[i * 3 + 2]);
        glm::dvec3 n = glm::cross(b - a, c - a);
        double len = glm::length(n);
        m_normals[i] = len > 0.0 ? glm::vec3(n / len) : glm::vec3(0.0f);
        m_volume += glm::dot(a, glm::cross(b, c)) / 6.0;
        m_area += 0.5 * len;
        m_areaCdf[i] = m_area;
    }
}

void Mesh::checkTopology()
{
    // Weld vertices by exact position, then count edge uses.
    size_t nv = m_vertices.size();
    std::vector<uint32_t> order(nv), id(nv);
    for (size_t i = 0; i < nv; ++i)
        order[i] = uint32_t(i);
    auto less = [&](uint32_t a, uint32_t b) {
        const glm::vec3 &p = m_vertices[a], &q = m_vertices[b];
        if (p.x != q.x) return p.x < q.x;
        if (p.y != q.y) return p.y < q.y;
        return p.z < q.z;
    };
    std::sort(order.begin(), order.end(), less);
    uint32_t next = 0;
    for (size_t i = 0; i < nv; ++i)
    {
        if (i > 0 && m_vertices[order[i]] != m_vertices[order[i - 1]])
            ++next;
        id[order[i]] = next;
    }

    std::vector<uint64_t> directed, undirected;
    directed.reserve(nv);
    undirected.reserve(nv);
    for (size_t t = 0; t < m_normals.size(); ++t)
        for (int e = 0; e < 3; ++e)
        {
            uint64_t a = id[t * 3 + e], b = id[t * 3 + (e + 1) % 3];
            directed.push_back(a << 32 | b);
            undirected.push_back(std::min(a, b) << 32 | std::max(a, b));
        }
    std::sort(directed.begin(), directed.end());
    std::sort(undirected.begin(), undirected.end());

    m_openEdges = m_flippedEdges = 0;
    for (size_t i = 1; i < directed.size(); ++i)
        m_flippedEdges += directed[i] == directed[i - 1];
    for (size_t i = 0; i < undirected.size();)
    {
        size_t j = i;
        while (j < undirected.size() && undirected[j] == undirected[i])
            ++j;
        m_openEdges += (j - i) != 2;
        i = j;
    }
}

// ---------------------------------------------------------------- BVH (binned SAH)

void Mesh::buildBvh()
{
    const size_t n = m_normals.size();
    std::vector<Box> boxes(n);
    std::vector<glm::vec3> centroids(n);
    std::vector<uint32_t> index(n);
    for (size_t i = 0; i < n; ++i)
    {
        for (int k = 0; k < 3; ++k)
            boxes[i].grow(m_vertices[i * 3 + k]);
        centroids[i] = 0.5f * (boxes[i].min + boxes[i].max);
        index[i] = uint32_t(i);
    }

    m_nodes.clear();
    m_nodes.reserve(2 * n / kLeafSize + 1);
    m_nodes.push_back({});

    struct Task
    {
        uint32_t node, first, count;
    };
    std::vector<Task> stack{ { 0, 0, uint32_t(n) } };
    while (!stack.empty())
    {
        Task task = stack.back();
        stack.pop_back();

        Box bounds, cbounds;
        for (uint32_t i = task.first; i < task.first + task.count; ++i)
        {
            bounds.grow(boxes[index[i]]);
            cbounds.grow(centroids[index[i]]);
        }
        BvhNode& node = m_nodes[task.node];
        node.min = bounds.min;
        node.max = bounds.max;
        node.leftOrFirst = task.first;
        node.count = task.count;
        if (task.count <= uint32_t(kLeafSize))
            continue;

        // Best split over all three axes.
        float bestCost = kInf;
        int bestAxis = -1, bestBin = -1;
        for (int axis = 0; axis < 3; ++axis)
        {
            float lo = cbounds.min[axis], hi = cbounds.max[axis];
            if (hi - lo < 1e-12f)
                continue;
            Box binBox[kBins];
            int binCount[kBins] = {};
            float scale = kBins / (hi - lo);
            for (uint32_t i = task.first; i < task.first + task.count; ++i)
            {
                int b = std::min(kBins - 1, int((centroids[index[i]][axis] - lo) * scale));
                binBox[b].grow(boxes[index[i]]);
                ++binCount[b];
            }
            float leftArea[kBins - 1];
            int leftCount[kBins - 1];
            Box acc;
            int cnt = 0;
            for (int b = 0; b < kBins - 1; ++b)
            {
                acc.grow(binBox[b]);
                cnt += binCount[b];
                leftArea[b] = acc.area();
                leftCount[b] = cnt;
            }
            acc = Box();
            cnt = 0;
            for (int b = kBins - 1; b > 0; --b)
            {
                acc.grow(binBox[b]);
                cnt += binCount[b];
                float cost = leftCount[b - 1] * leftArea[b - 1] + cnt * acc.area();
                if (leftCount[b - 1] > 0 && cnt > 0 && cost < bestCost)
                {
                    bestCost = cost;
                    bestAxis = axis;
                    bestBin = b;
                }
            }
        }

        uint32_t mid;
        if (bestAxis < 0)
        {
            // All centroids coincide: split by count.
            mid = task.first + task.count / 2;
        }
        else
        {
            float lo = cbounds.min[bestAxis], scale = kBins / (cbounds.max[bestAxis] - lo);
            auto it = std::partition(index.begin() + task.first, index.begin() + task.first + task.count, [&](uint32_t t) {
                return std::min(kBins - 1, int((centroids[t][bestAxis] - lo) * scale)) < bestBin;
            });
            mid = uint32_t(it - index.begin());
        }

        uint32_t left = uint32_t(m_nodes.size());
        m_nodes.push_back({});
        m_nodes.push_back({});
        BvhNode& parent = m_nodes[task.node]; // re-fetch: push_back may reallocate
        parent.leftOrFirst = left;
        parent.count = 0;
        stack.push_back({ left, task.first, mid - task.first });
        stack.push_back({ left + 1, mid, task.first + task.count - mid });
    }

    // Reorder the triangles to BVH order.
    std::vector<glm::vec3> vertices(m_vertices.size());
    std::vector<glm::vec3> normals(n);
    std::vector<double> cdf(n);
    double acc = 0.0;
    for (size_t i = 0; i < n; ++i)
    {
        uint32_t t = index[i];
        for (int k = 0; k < 3; ++k)
            vertices[i * 3 + k] = m_vertices[t * 3 + k];
        normals[i] = m_normals[t];
        acc += m_areaCdf[t] - (t > 0 ? m_areaCdf[t - 1] : 0.0);
        cdf[i] = acc;
    }
    m_vertices = std::move(vertices);
    m_normals = std::move(normals);
    m_areaCdf = std::move(cdf);
}

// ---------------------------------------------------------------- queries

void Mesh::hits(const glm::vec3& worldO, const glm::vec3& d, std::vector<MeshHit>& out) const
{
    out.clear();
    if (m_nodes.empty())
        return;
    glm::vec3 invD = safeInverse(d);
    float r0, r1;
    if (!lineBox(worldO, invD, m_nodes[0].min, m_nodes[0].max, r0, r1))
        return;
    // Work from the box entry, so the triangle tests stay precise for far origins.
    const float shift = r0;
    const glm::vec3 o = worldO + d * shift;

    std::vector<MeshHit> raw;
    uint32_t stack[64];
    int sp = 0;
    stack[sp++] = 0;
    while (sp > 0)
    {
        const BvhNode& node = m_nodes[stack[--sp]];
        float t0, t1;
        if (!lineBox(o, invD, node.min, node.max, t0, t1))
            continue;
        if (node.count == 0)
        {
            if (sp + 2 > 64)
                continue;
            stack[sp++] = node.leftOrFirst;
            stack[sp++] = node.leftOrFirst + 1;
            continue;
        }
        for (uint32_t i = node.leftOrFirst; i < node.leftOrFirst + node.count; ++i)
        {
            // Möller-Trumbore with a little barycentric slack.
            const glm::vec3& v0 = m_vertices[i * 3];
            glm::vec3 e1 = m_vertices[i * 3 + 1] - v0, e2 = m_vertices[i * 3 + 2] - v0;
            glm::vec3 p = glm::cross(d, e2);
            float det = glm::dot(e1, p);
            if (std::fabs(det) < 1e-30f)
                continue;
            float inv = 1.0f / det;
            glm::vec3 s = o - v0;
            float u = glm::dot(s, p) * inv;
            if (u < -kBaryEps || u > 1.0f + kBaryEps)
                continue;
            glm::vec3 q = glm::cross(s, e1);
            float v = glm::dot(d, q) * inv;
            if (v < -kBaryEps || u + v > 1.0f + kBaryEps)
                continue;
            raw.push_back({ glm::dot(e2, q) * inv + shift, int(i), det > 0.0f });
        }
    }

    std::sort(raw.begin(), raw.end(), [](const MeshHit& a, const MeshHit& b) { return a.t < b.t; });

    // Clean up crossings at one point: duplicates from triangles sharing an
    // edge or vertex merge, and an enter/leave pair (a grazing touch) cancels.
    const float merge = kMergeLen / std::max(glm::length(d), 1e-30f);
    for (const MeshHit& h : raw)
    {
        if (!out.empty() && h.t - out.back().t <= merge)
        {
            if (out.back().entering != h.entering)
                out.pop_back();
            continue;
        }
        out.push_back(h);
    }
}

void Mesh::spans(const glm::vec3& o, const glm::vec3& d, std::vector<glm::vec2>& out) const
{
    out.clear();
    std::vector<MeshHit> list;
    hits(o, d, list);
    int winding = 0;
    float start = 0.0f;
    for (const MeshHit& h : list)
    {
        if (h.entering)
        {
            if (winding++ == 0)
                start = h.t;
        }
        else if (winding > 0 && --winding == 0)
            out.emplace_back(start, h.t);
    }
}

float Mesh::distance(const glm::vec3& p, int& tri) const
{
    tri = -1;
    if (m_nodes.empty())
        return kInf;
    float best = kInf;
    uint32_t stack[64];
    int sp = 0;
    stack[sp++] = 0;
    while (sp > 0)
    {
        const BvhNode& node = m_nodes[stack[--sp]];
        if (boxDistance2(p, node) >= best)
            continue;
        if (node.count == 0)
        {
            uint32_t a = node.leftOrFirst, b = a + 1;
            float da = boxDistance2(p, m_nodes[a]), db = boxDistance2(p, m_nodes[b]);
            if (da < db)
                std::swap(a, b), std::swap(da, db);
            // Push the farther child first, so the nearer one is visited first.
            if (da < best && sp < 64)
                stack[sp++] = a;
            if (db < best && sp < 64)
                stack[sp++] = b;
            continue;
        }
        for (uint32_t i = node.leftOrFirst; i < node.leftOrFirst + node.count; ++i)
        {
            glm::vec3 c = closestOnTriangle(p, m_vertices[i * 3], m_vertices[i * 3 + 1], m_vertices[i * 3 + 2]);
            float d2 = glm::dot(c - p, c - p);
            if (d2 < best)
            {
                best = d2;
                tri = int(i);
            }
        }
    }
    return std::sqrt(best);
}

bool Mesh::inside(const glm::vec3& p) const
{
    const BvhNode& root = m_nodes[0];
    if (glm::any(glm::lessThan(p, root.min)) || glm::any(glm::greaterThan(p, root.max)))
        return false;
    // Winding along a fixed, axis-skewed direction.
    std::vector<glm::vec2> list;
    spans(p, glm::vec3(0.5773f, 0.5778f, 0.5769f), list);
    for (const glm::vec2& s : list)
        if (s.x < 0.0f && s.y > 0.0f)
            return true;
    return false;
}

float Mesh::sdf(const glm::vec3& p) const
{
    int tri;
    float d = distance(p, tri);
    return inside(p) ? -d : d;
}

glm::vec3 Mesh::normalAt(const glm::vec3& p) const
{
    int tri;
    distance(p, tri);
    return tri >= 0 ? m_normals[tri] : glm::vec3(0.0f, 1.0f, 0.0f);
}

glm::vec3 Mesh::surfacePoint(float r0, float r1, float r2) const
{
    double target = double(r0) * m_area;
    size_t i = std::upper_bound(m_areaCdf.begin(), m_areaCdf.end(), target) - m_areaCdf.begin();
    i = std::min(i, m_areaCdf.size() - 1);
    float a = std::sqrt(r1);
    const glm::vec3* v = &m_vertices[i * 3];
    return (1.0f - a) * v[0] + a * (1.0f - r2) * v[1] + a * r2 * v[2];
}

// ---------------------------------------------------------------- wireframe contours

namespace {

// Douglas-Peucker on a polyline.
void simplify(const std::vector<glm::vec3>& pts, size_t a, size_t b, float tol, std::vector<char>& keep)
{
    if (b <= a + 1)
        return;
    glm::vec3 ab = pts[b] - pts[a];
    float len2 = glm::dot(ab, ab);
    float worst = -1.0f;
    size_t at = a;
    for (size_t i = a + 1; i < b; ++i)
    {
        glm::vec3 ap = pts[i] - pts[a];
        float t = len2 > 0.0f ? glm::clamp(glm::dot(ap, ab) / len2, 0.0f, 1.0f) : 0.0f;
        glm::vec3 diff = ap - ab * t;
        float d2 = glm::dot(diff, diff);
        if (d2 > worst)
        {
            worst = d2;
            at = i;
        }
    }
    if (worst > tol * tol)
    {
        keep[at] = 1;
        simplify(pts, a, at, tol, keep);
        simplify(pts, at, b, tol, keep);
    }
}

} // namespace

void Mesh::buildContours()
{
    constexpr int kSlices = 5;     // planes per axis
    constexpr float kTol = 0.0015f; // simplification tolerance (local units)
    m_contours.clear();

    for (int axis = 0; axis < 3; ++axis)
        for (int k = 1; k <= kSlices; ++k)
        {
            float level = -m_halfExtent[axis] + 2.0f * m_halfExtent[axis] * k / (kSlices + 1);

            // Plane/triangle segments. Edge points are computed from the edge in a
            // canonical vertex order, so neighbours produce bit-identical points.
            std::vector<glm::vec3> segs;
            auto edgePoint = [&](glm::vec3 p, glm::vec3 q) {
                if (q.x < p.x || (q.x == p.x && (q.y < p.y || (q.y == p.y && q.z < p.z))))
                    std::swap(p, q);
                float f = (level - p[axis]) / (q[axis] - p[axis]);
                return p + (q - p) * f;
            };
            for (size_t t = 0; t < m_normals.size(); ++t)
            {
                const glm::vec3* v = &m_vertices[t * 3];
                bool above[3];
                for (int i = 0; i < 3; ++i)
                    above[i] = v[i][axis] >= level;
                if (above[0] == above[1] && above[1] == above[2])
                    continue;
                glm::vec3 pts[2];
                int n = 0;
                for (int i = 0; i < 3; ++i)
                    if (above[i] != above[(i + 1) % 3])
                        pts[n++] = edgePoint(v[i], v[(i + 1) % 3]);
                segs.push_back(pts[0]);
                segs.push_back(pts[1]);
            }

            // Chain the segments into polylines.
            std::unordered_multimap<uint64_t, size_t> ends;
            for (size_t i = 0; i < segs.size(); ++i)
                ends.emplace(pointKey(segs[i]), i);
            std::vector<char> used(segs.size() / 2, 0);
            auto partner = [&](size_t endIndex) -> long {
                auto range = ends.equal_range(pointKey(segs[endIndex]));
                for (auto it = range.first; it != range.second; ++it)
                    if (it->second / 2 != endIndex / 2 && !used[it->second / 2] && segs[it->second] == segs[endIndex])
                        return long(it->second);
                return -1;
            };
            for (size_t s = 0; s < used.size(); ++s)
            {
                if (used[s])
                    continue;
                used[s] = 1;
                std::vector<glm::vec3> line{ segs[s * 2], segs[s * 2 + 1] };
                // Extend forward from the second end, then backward from the first.
                for (int dir = 0; dir < 2; ++dir)
                {
                    size_t tip = dir == 0 ? s * 2 + 1 : s * 2;
                    for (;;)
                    {
                        long e = partner(tip);
                        if (e < 0)
                            break;
                        used[size_t(e) / 2] = 1;
                        size_t other = size_t(e) ^ 1;
                        if (dir == 0)
                            line.push_back(segs[other]);
                        else
                            line.insert(line.begin(), segs[other]);
                        tip = other;
                    }
                }

                std::vector<char> keep(line.size(), 0);
                keep.front() = keep.back() = 1;
                if (line.size() > 2 && line.front() == line.back())
                    keep[line.size() / 2] = 1; // closed loop: split it in two
                size_t a = 0;
                for (size_t i = 1; i < line.size(); ++i)
                    if (keep[i])
                    {
                        simplify(line, a, i, kTol, keep);
                        a = i;
                    }
                glm::vec3 prev = line.front();
                for (size_t i = 1; i < line.size(); ++i)
                {
                    if (!keep[i])
                        continue;
                    Curve c;
                    c.a = prev;
                    c.b = line[i];
                    // Only the ends lie exactly on the surface (chords cut corners),
                    // so edge SMC classifies at the ends and bisects between them.
                    c.samples = 1;
                    m_contours.push_back(c);
                    prev = line[i];
                }
            }
        }
}

} // namespace csg
