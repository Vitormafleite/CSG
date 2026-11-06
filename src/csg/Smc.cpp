#include "csg/Smc.h"

#include <algorithm>
#include <cmath>

namespace csg {

namespace {

constexpr float kInf = 1e30f;
constexpr float kSpanEps = 1e-5f;

bool apply(OpCode op, bool inA, bool inB)
{
    switch (op)
    {
    case OpCode::Union: return inA || inB;
    case OpCode::Intersection: return inA && inB;
    case OpCode::Difference: return inA && !inB;
    default: return false;
    }
}

float endT(const SpanList& list, size_t k) { return k % 2 == 0 ? list[k / 2].t0 : list[k / 2].t1; }
int endP(const SpanList& list, size_t k) { return k % 2 == 0 ? list[k / 2].p0 : list[k / 2].p1; }

} // namespace

int Program::maxStackDepth() const
{
    int depth = 0, maxDepth = 0;
    for (const Op& op : ops)
    {
        if (op.code == OpCode::Push || op.code == OpCode::Empty)
            maxDepth = std::max(maxDepth, ++depth);
        else
            --depth;
    }
    return maxDepth;
}

Aabb Program::bounds() const
{
    Aabb box;
    for (const Primitive& p : prims)
        box.expand(p.bounds());
    return box;
}

SpanList combine(const SpanList& a, const SpanList& b, OpCode op)
{
    SpanList out;
    const size_t na = a.size() * 2, nb = b.size() * 2;
    size_t ia = 0, ib = 0;
    bool inA = false, inB = false, inR = false;
    Span cur{};

    // Sweep the endpoints of both lists in order, tracking membership.
    while (ia < na || ib < nb)
    {
        float ta = ia < na ? endT(a, ia) : kInf;
        float tb = ib < nb ? endT(b, ib) : kInf;
        float t;
        int prim;
        if (ta <= tb)
        {
            t = ta;
            inA = ia % 2 == 0;
            prim = endP(a, ia++);
        }
        else
        {
            t = tb;
            inB = ib % 2 == 0;
            prim = endP(b, ib++);
            if (op == OpCode::Difference)
                prim = -prim;
        }

        bool r = apply(op, inA, inB);
        if (r && !inR)
        {
            cur.t0 = t;
            cur.p0 = prim;
        }
        else if (!r && inR)
        {
            cur.t1 = t;
            cur.p1 = prim;
            // Regularize: drop degenerate spans and merge touching ones.
            if (!out.empty() && cur.t0 - out.back().t1 <= kSpanEps)
            {
                out.back().t1 = cur.t1;
                out.back().p1 = cur.p1;
            }
            else if (cur.t1 - cur.t0 > kSpanEps)
                out.push_back(cur);
        }
        inR = r;
    }
    return out;
}

SpanList classifyRay(const Program& program, const glm::vec3& o, const glm::vec3& d)
{
    std::vector<SpanList> stack;
    for (const Op& op : program.ops)
    {
        switch (op.code)
        {
        case OpCode::Push:
        {
            SpanList list;
            std::vector<glm::vec2> spans;
            program.prims[op.prim].spans(o, d, spans);
            for (const glm::vec2& s : spans)
                list.push_back({ s.x, s.y, op.prim + 1, op.prim + 1 });
            stack.push_back(std::move(list));
            break;
        }
        case OpCode::Empty:
            stack.emplace_back();
            break;
        default:
        {
            SpanList b = std::move(stack.back());
            stack.pop_back();
            stack.back() = combine(stack.back(), b, op.code);
            break;
        }
        }
    }
    return stack.empty() ? SpanList{} : std::move(stack.back());
}

float treeSdf(const Program& program, const glm::vec3& p)
{
    std::vector<float> stack;
    for (const Op& op : program.ops)
    {
        switch (op.code)
        {
        case OpCode::Push: stack.push_back(program.prims[op.prim].sdf(p)); break;
        case OpCode::Empty: stack.push_back(kInf); break;
        default:
        {
            float b = stack.back();
            stack.pop_back();
            float& a = stack.back();
            if (op.code == OpCode::Union)
                a = std::min(a, b);
            else if (op.code == OpCode::Intersection)
                a = std::max(a, b);
            else
                a = std::max(a, -b);
            break;
        }
        }
    }
    return stack.empty() ? kInf : stack.back();
}

bool crossesBoundary(const Program& program, int primIndex, const glm::vec3& p, float delta, float eps)
{
    const Primitive& prim = program.prims[primIndex];

    // Probe no farther than the clearance to the other surfaces (their SDFs are
    // exact distances), so shallow crossings are not stepped over. Surfaces
    // within eps are coincident and get the full step, which regularizes them.
    float step = delta;
    for (size_t j = 0; j < program.prims.size(); ++j)
    {
        float clearance = std::fabs(program.prims[j].sdf(p));
        if (int(j) != primIndex && clearance > eps)
            step = std::min(step, 0.5f * clearance);
    }

    glm::vec3 n = prim.normal(p);
    // Shrink the inward step until it stays inside the primitive (thin wedges at sharp edges).
    float in = step;
    for (int i = 0; i < 12 && prim.sdf(p - n * in) >= 0.0f; ++i)
        in *= 0.5f;
    bool outside = treeSdf(program, p + n * step) < 0.0f;
    bool inside = treeSdf(program, p - n * in) < 0.0f;
    return outside != inside;
}

Membership classifyPoint(const Program& program, const glm::vec3& p, float eps)
{
    float d = treeSdf(program, p);
    if (d < -eps)
        return Membership::In;
    if (d > eps)
        return Membership::Out;

    // Near the boundary the SDF alone cannot tell a true boundary point from one
    // on coincident or interior faces (two blocks sharing a face, a flush cut).
    // Regularize: the point is ON only if its neighborhood has both IN and OUT.
    const float delta = 10.0f * eps;
    for (size_t i = 0; i < program.prims.size(); ++i)
        if (std::fabs(program.prims[i].sdf(p)) <= eps && crossesBoundary(program, int(i), p, delta, eps))
            return Membership::On;

    static const glm::vec3 kDirs[] = {
        { 1, 0, 0 }, { -1, 0, 0 }, { 0, 1, 0 }, { 0, -1, 0 }, { 0, 0, 1 }, { 0, 0, -1 },
        { 0.577f, 0.577f, 0.577f }, { -0.577f, 0.577f, 0.577f }, { 0.577f, -0.577f, 0.577f },
        { 0.577f, 0.577f, -0.577f }, { -0.577f, -0.577f, 0.577f }, { -0.577f, 0.577f, -0.577f },
        { 0.577f, -0.577f, -0.577f }, { -0.577f, -0.577f, -0.577f },
    };
    bool anyIn = false, anyOut = false;
    for (const glm::vec3& dir : kDirs)
    {
        if (treeSdf(program, p + dir * delta) < 0.0f)
            anyIn = true;
        else
            anyOut = true;
        if (anyIn && anyOut)
            return Membership::On;
    }
    return anyIn ? Membership::In : Membership::Out;
}

std::vector<CurvePiece> classifyCurve(const Program& program, const Curve& curve, float eps, int samples)
{
    std::vector<CurvePiece> pieces;
    auto add = [&](float s0, float s1, Membership m) {
        if (s1 <= s0)
            return;
        if (!pieces.empty() && pieces.back().membership == m)
            pieces.back().s1 = s1;
        else
            pieces.push_back({ s0, s1, m });
    };

    float prevS = 0.0f;
    Membership prev = classifyPoint(program, curve.point(0.0f), eps);
    for (int i = 1; i <= samples; ++i)
    {
        float s = float(i) / samples;
        Membership cur = classifyPoint(program, curve.point(s), eps);
        if (cur == prev)
        {
            add(prevS, s, cur);
        }
        else
        {
            // Bisect for the transition point.
            float lo = prevS, hi = s;
            for (int it = 0; it < 16; ++it)
            {
                float mid = 0.5f * (lo + hi);
                if (classifyPoint(program, curve.point(mid), eps) == prev)
                    lo = mid;
                else
                    hi = mid;
            }
            add(prevS, lo, prev);
            add(lo, s, cur);
        }
        prevS = s;
        prev = cur;
    }
    return pieces;
}

float onTolerance(const Program& program)
{
    Aabb box = program.bounds();
    if (box.empty())
        return 1e-4f;
    return std::max(1e-5f, 1e-4f * glm::length(box.max - box.min));
}

} // namespace csg
