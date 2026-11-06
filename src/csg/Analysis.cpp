#include "csg/Analysis.h"

#include <algorithm>
#include <cmath>
#include <thread>
#include <vector>

namespace csg {

namespace {

// Low-discrepancy sequence for even surface sampling.
float halton(int index, int base)
{
    float f = 1.0f, r = 0.0f;
    while (index > 0)
    {
        f /= float(base);
        r += f * float(index % base);
        index /= base;
    }
    return r;
}

template <class Fn>
double parallelSum(int count, Fn fn)
{
    unsigned threads = std::max(1u, std::thread::hardware_concurrency());
    std::vector<double> partial(threads, 0.0);
    std::vector<std::thread> pool;
    for (unsigned w = 0; w < threads; ++w)
        pool.emplace_back([&, w] {
            for (int i = int(w); i < count; i += int(threads))
                partial[w] += fn(i);
        });
    for (auto& t : pool)
        t.join();
    double sum = 0.0;
    for (double p : partial)
        sum += p;
    return sum;
}

} // namespace

double treeVolume(const Program& program, int res)
{
    if (program.empty() || program.prims.empty())
        return 0.0;
    Aabb box = program.bounds();
    glm::vec3 ext = box.max - box.min;
    double cellX = ext.x / res, cellY = ext.y / res;
    glm::vec3 dir(0.0f, 0.0f, 1.0f);

    double length = parallelSum(res, [&](int row) {
        double sum = 0.0;
        float y = box.min.y + (row + 0.5f) * float(cellY);
        for (int col = 0; col < res; ++col)
        {
            float x = box.min.x + (col + 0.5f) * float(cellX);
            glm::vec3 origin(x, y, box.min.z - 1.0f);
            for (const Span& s : classifyRay(program, origin, dir))
                sum += double(s.t1) - double(s.t0);
        }
        return sum;
    });
    return length * cellX * cellY;
}

double treeArea(const Program& program, int samplesPerPrim)
{
    if (program.prims.empty())
        return 0.0;
    const float eps = onTolerance(program);
    const float delta = 10.0f * eps;

    return parallelSum(int(program.prims.size()), [&](int i) {
        const Primitive& prim = program.prims[i];
        int on = 0;
        for (int k = 1; k <= samplesPerPrim; ++k)
        {
            glm::vec3 p = prim.surfacePoint(halton(k, 2), halton(k, 3), halton(k, 5));
            if (std::fabs(treeSdf(program, p)) > eps)
                continue;

            // A coincident surface patch (same plane or sphere, normals aligned)
            // is counted by the first primitive that has it.
            glm::vec3 n = prim.normal(p);
            bool counted = false;
            for (int j = 0; j < i && !counted; ++j)
                counted = std::fabs(program.prims[j].sdf(p)) <= eps &&
                          std::fabs(glm::dot(n, program.prims[j].normal(p))) > 0.999f;
            if (counted)
                continue;

            // True boundary of the regularized solid: inside on one side, outside on the other.
            if (crossesBoundary(program, i, p, delta, eps))
                ++on;
        }
        return double(prim.area()) * on / samplesPerPrim;
    });
}

double treeHypervolume(const Scene& scene, const Node* tree, float t0, float t1, int steps, int res)
{
    if (!tree || steps < 1 || t1 <= t0)
        return 0.0;
    double dt = double(t1 - t0) / steps, sum = 0.0;
    for (int i = 0; i <= steps; ++i)
    {
        double v = treeVolume(scene.program(tree, float(t0 + i * dt)), res);
        sum += (i == 0 || i == steps) ? 0.5 * v : v;
    }
    return sum * dt;
}

} // namespace csg
