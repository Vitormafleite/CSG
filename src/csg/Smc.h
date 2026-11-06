#pragma once

#include "csg/Primitive.h"

#include <vector>

namespace csg {

// Set membership classification (SMC) against a CSG tree.
//
// A tree evaluated at some instant is flattened into a postfix Program. Its
// layout is mirrored in shaders/csg.frag, which runs the same span algebra
// on the GPU.

enum class OpCode : int { Push = 0, Union = 1, Intersection = 2, Difference = 3, Empty = 4 };

struct Op
{
    OpCode code;
    int prim = -1; // index into Program::prims for Push
};

struct Program
{
    std::vector<Primitive> prims;
    std::vector<Op> ops;

    bool empty() const { return ops.empty(); }
    int maxStackDepth() const;
    Aabb bounds() const;
};

// One IN span of a line o + t*d. Each end records the surface that bounds
// it as (primIndex + 1), negated when the surface normal must be flipped
// (it comes from a subtracted solid).
struct Span
{
    float t0, t1;
    int p0, p1;
};

using SpanList = std::vector<Span>;

// Regularized set operation on two sorted span lists.
SpanList combine(const SpanList& a, const SpanList& b, OpCode op);

// Ray/line SMC: the IN spans of the line against the whole tree. Between
// them the line is OUT, and the span ends are ON the boundary.
SpanList classifyRay(const Program& program, const glm::vec3& o, const glm::vec3& d);

enum class Membership { In, On, Out };

// Signed distance bound of the tree (min/max combination of primitive SDFs).
// Its sign is exact, so it gives point membership classification.
float treeSdf(const Program& program, const glm::vec3& p);
// Regularized: points on interior or coincident faces are IN or OUT, not ON.
Membership classifyPoint(const Program& program, const glm::vec3& p, float eps);

// True when the tree's membership differs on the two sides of primitive
// primIndex's surface at p (p lies on that surface), probing at most delta away.
bool crossesBoundary(const Program& program, int primIndex, const glm::vec3& p, float delta, float eps);

struct CurvePiece
{
    float s0, s1;
    Membership membership;
};

// Edge SMC: splits a wireframe curve into IN, ON and OUT pieces against the tree.
std::vector<CurvePiece> classifyCurve(const Program& program, const Curve& curve, float eps, int samples = 64);

// Tolerance for ON classification scaled to the program's size.
float onTolerance(const Program& program);

} // namespace csg
