#include "csg/Primitive.h"

#include "csg/Mesh.h"

#include <glm/gtc/constants.hpp>

#include <algorithm>
#include <cmath>

namespace csg {

namespace {

constexpr float kInf = 1e30f;
constexpr float kPi = glm::pi<float>();

// Clips [t0, t1] to the slab lo <= o + t*d <= hi along one axis.
bool clipSlab(float o, float d, float lo, float hi, float& t0, float& t1)
{
    if (std::fabs(d) < 1e-12f)
        return o >= lo && o <= hi;
    float ta = (lo - o) / d;
    float tb = (hi - o) / d;
    if (ta > tb)
        std::swap(ta, tb);
    t0 = std::max(t0, ta);
    t1 = std::min(t1, tb);
    return t0 <= t1;
}

float dot2(const glm::vec2& v) { return glm::dot(v, v); }

} // namespace

const char* primTypeName(PrimType type)
{
    switch (type)
    {
    case PrimType::Block: return "Block";
    case PrimType::Sphere: return "Sphere";
    case PrimType::Cylinder: return "Cylinder";
    case PrimType::Cone: return "Cone";
    case PrimType::Mesh: return "Mesh";
    }
    return "?";
}

void Aabb::expand(const Aabb& other)
{
    min = glm::min(min, other.min);
    max = glm::max(max, other.max);
}

glm::vec3 Curve::point(float s) const
{
    if (!circle)
        return glm::mix(a, b, s);
    float angle = 2.0f * kPi * s;
    return center + radius * (std::cos(angle) * u + std::sin(angle) * v);
}

glm::vec3 Primitive::toLocal(const glm::vec3& p) const
{
    return glm::conjugate(rotation) * (p - position);
}

glm::vec3 Primitive::dirToLocal(const glm::vec3& d) const
{
    return glm::conjugate(rotation) * d;
}

void Primitive::spans(const glm::vec3& o, const glm::vec3& d, std::vector<glm::vec2>& out) const
{
    out.clear();
    if (type == PrimType::Mesh)
    {
        if (mesh)
            mesh->spans(toMesh(o), dirToLocal(d) / size.x, out);
        return;
    }
    float t0, t1;
    if (intersect(o, d, t0, t1))
        out.emplace_back(t0, t1);
}

bool Primitive::intersect(const glm::vec3& worldO, const glm::vec3& worldD, float& t0, float& t1) const
{
    if (type == PrimType::Mesh)
    {
        std::vector<glm::vec2> list;
        spans(worldO, worldD, list);
        if (list.empty())
            return false;
        t0 = list.front().x;
        t1 = list.back().y;
        return true;
    }

    glm::vec3 o = toLocal(worldO);
    glm::vec3 d = dirToLocal(worldD);
    t0 = -kInf;
    t1 = kInf;

    switch (type)
    {
    case PrimType::Block:
    {
        glm::vec3 h = size * 0.5f;
        return clipSlab(o.x, d.x, -h.x, h.x, t0, t1) && clipSlab(o.y, d.y, -h.y, h.y, t0, t1) &&
               clipSlab(o.z, d.z, -h.z, h.z, t0, t1);
    }
    case PrimType::Sphere:
    {
        float r = size.x;
        float a = glm::dot(d, d);
        float b = glm::dot(o, d);
        float c = glm::dot(o, o) - r * r;
        float disc = b * b - a * c;
        if (a < 1e-12f || disc < 0.0f)
            return false;
        float s = std::sqrt(disc);
        t0 = (-b - s) / a;
        t1 = (-b + s) / a;
        return true;
    }
    case PrimType::Cylinder:
    {
        float r = size.x;
        float hh = size.y * 0.5f;
        float a = d.x * d.x + d.z * d.z;
        float b = o.x * d.x + o.z * d.z;
        float c = o.x * o.x + o.z * o.z - r * r;
        if (a < 1e-12f)
        {
            if (c > 0.0f)
                return false;
        }
        else
        {
            float disc = b * b - a * c;
            if (disc < 0.0f)
                return false;
            float s = std::sqrt(disc);
            t0 = (-b - s) / a;
            t1 = (-b + s) / a;
        }
        return clipSlab(o.y, d.y, -hh, hh, t0, t1);
    }
    case PrimType::Cone:
    {
        // q(t) = rho^2 - k^2 (apexY - y)^2 <= 0, intersected with the slab |y| <= h/2.
        float r = size.x;
        float h = size.y;
        float hh = h * 0.5f;
        float k2 = (r / h) * (r / h);
        float w = hh - o.y;
        float dw = -d.y;
        float a = d.x * d.x + d.z * d.z - k2 * dw * dw;
        float b = o.x * d.x + o.z * d.z - k2 * w * dw; // half of the linear coefficient
        float c = o.x * o.x + o.z * o.z - k2 * w * w;

        float s0 = -kInf, s1 = kInf;
        if (!clipSlab(o.y, d.y, -hh, hh, s0, s1))
            return false;

        if (std::fabs(a) < 1e-12f)
        {
            if (std::fabs(b) < 1e-12f)
            {
                if (c > 0.0f)
                    return false;
                t0 = s0;
                t1 = s1;
                return true;
            }
            float ts = -c / (2.0f * b);
            if (b > 0.0f)
                t1 = ts;
            else
                t0 = ts;
            t0 = std::max(t0, s0);
            t1 = std::min(t1, s1);
            return t0 <= t1;
        }

        float disc = b * b - a * c;
        if (a > 0.0f)
        {
            if (disc < 0.0f)
                return false;
            float s = std::sqrt(disc);
            t0 = std::max((-b - s) / a, s0);
            t1 = std::min((-b + s) / a, s1);
            return t0 <= t1;
        }

        // a < 0: inside region is outside the roots.
        if (disc < 0.0f)
        {
            t0 = s0;
            t1 = s1;
            return true;
        }
        float s = std::sqrt(disc);
        float r1 = (-b + s) / a; // a < 0, so this is the smaller root
        float r2 = (-b - s) / a;
        float a0 = s0, a1 = std::min(r1, s1);
        float b0 = std::max(r2, s0), b1 = s1;
        bool hasA = a0 <= a1, hasB = b0 <= b1;
        if (hasA && (!hasB || a1 - a0 >= b1 - b0))
        {
            t0 = a0;
            t1 = a1;
            return true;
        }
        if (hasB)
        {
            t0 = b0;
            t1 = b1;
            return true;
        }
        return false;
    }
    case PrimType::Mesh:
        break; // handled above
    }
    return false;
}

float Primitive::sdf(const glm::vec3& worldP) const
{
    glm::vec3 p = toLocal(worldP);
    switch (type)
    {
    case PrimType::Block:
    {
        glm::vec3 q = glm::abs(p) - size * 0.5f;
        return glm::length(glm::max(q, 0.0f)) + std::min(std::max(q.x, std::max(q.y, q.z)), 0.0f);
    }
    case PrimType::Sphere:
        return glm::length(p) - size.x;
    case PrimType::Cylinder:
    {
        glm::vec2 d = glm::abs(glm::vec2(glm::length(glm::vec2(p.x, p.z)), p.y)) - glm::vec2(size.x, size.y * 0.5f);
        return std::min(std::max(d.x, d.y), 0.0f) + glm::length(glm::max(d, 0.0f));
    }
    case PrimType::Cone:
    {
        // Capped cone (Inigo Quilez), bottom radius r1, top radius 0, half height hh.
        float hh = size.y * 0.5f;
        float r1 = size.x;
        glm::vec2 q(glm::length(glm::vec2(p.x, p.z)), p.y);
        glm::vec2 k1(0.0f, hh);
        glm::vec2 k2(-r1, 2.0f * hh);
        glm::vec2 ca(q.x - std::min(q.x, q.y < 0.0f ? r1 : 0.0f), std::fabs(q.y) - hh);
        glm::vec2 cb = q - k1 + k2 * glm::clamp(glm::dot(k1 - q, k2) / dot2(k2), 0.0f, 1.0f);
        float s = (cb.x < 0.0f && ca.y < 0.0f) ? -1.0f : 1.0f;
        return s * std::sqrt(std::min(dot2(ca), dot2(cb)));
    }
    case PrimType::Mesh:
        return mesh ? mesh->sdf(p / size.x) * size.x : kInf;
    }
    return kInf;
}

glm::vec3 Primitive::normal(const glm::vec3& worldP) const
{
    glm::vec3 p = toLocal(worldP);
    glm::vec3 n(0.0f, 1.0f, 0.0f);

    switch (type)
    {
    case PrimType::Block:
    {
        glm::vec3 q = glm::abs(p) / (size * 0.5f);
        if (q.x >= q.y && q.x >= q.z)
            n = glm::vec3(p.x > 0.0f ? 1.0f : -1.0f, 0.0f, 0.0f);
        else if (q.y >= q.z)
            n = glm::vec3(0.0f, p.y > 0.0f ? 1.0f : -1.0f, 0.0f);
        else
            n = glm::vec3(0.0f, 0.0f, p.z > 0.0f ? 1.0f : -1.0f);
        break;
    }
    case PrimType::Sphere:
        n = glm::normalize(p);
        break;
    case PrimType::Cylinder:
    {
        float rho = glm::length(glm::vec2(p.x, p.z));
        float dSide = std::fabs(rho - size.x);
        float dCap = std::fabs(std::fabs(p.y) - size.y * 0.5f);
        if (dCap < dSide || rho < 1e-6f)
            n = glm::vec3(0.0f, p.y > 0.0f ? 1.0f : -1.0f, 0.0f);
        else
            n = glm::vec3(p.x / rho, 0.0f, p.z / rho);
        break;
    }
    case PrimType::Cone:
    {
        float hh = size.y * 0.5f;
        float k = size.x / size.y;
        float rho = glm::length(glm::vec2(p.x, p.z));
        float dCap = std::fabs(p.y + hh);
        float dSide = std::fabs(rho - k * (hh - p.y)) / std::sqrt(1.0f + k * k);
        if (dCap < dSide)
            n = glm::vec3(0.0f, -1.0f, 0.0f);
        else if (rho > 1e-6f)
            n = glm::normalize(glm::vec3(p.x / rho, k, p.z / rho));
        break;
    }
    case PrimType::Mesh:
        if (mesh)
            n = mesh->normalAt(p / size.x);
        break;
    }
    return rotation * n;
}

float Primitive::volume() const
{
    switch (type)
    {
    case PrimType::Block: return size.x * size.y * size.z;
    case PrimType::Sphere: return 4.0f / 3.0f * kPi * size.x * size.x * size.x;
    case PrimType::Cylinder: return kPi * size.x * size.x * size.y;
    case PrimType::Cone: return kPi * size.x * size.x * size.y / 3.0f;
    case PrimType::Mesh: return mesh ? float(mesh->volume()) * size.x * size.x * size.x : 0.0f;
    }
    return 0.0f;
}

float Primitive::area() const
{
    float r = size.x, h = size.y;
    switch (type)
    {
    case PrimType::Block: return 2.0f * (size.x * size.y + size.y * size.z + size.x * size.z);
    case PrimType::Sphere: return 4.0f * kPi * r * r;
    case PrimType::Cylinder: return 2.0f * kPi * r * h + 2.0f * kPi * r * r;
    case PrimType::Cone: return kPi * r * (r + std::sqrt(r * r + h * h));
    case PrimType::Mesh: return mesh ? float(mesh->area()) * r * r : 0.0f;
    }
    return 0.0f;
}

glm::vec3 Primitive::surfacePoint(float r0, float r1, float r2) const
{
    glm::vec3 p(0.0f);
    float r = size.x, h = size.y, hh = size.y * 0.5f;

    switch (type)
    {
    case PrimType::Block:
    {
        glm::vec3 e = size * 0.5f;
        float axy = size.x * size.y, ayz = size.y * size.z, axz = size.x * size.z;
        float pick = r0 * 2.0f * (axy + ayz + axz);
        float a = 2.0f * r1 - 1.0f, b = 2.0f * r2 - 1.0f;
        if (pick < 2.0f * axy)
            p = glm::vec3(a * e.x, b * e.y, pick < axy ? e.z : -e.z);
        else if (pick < 2.0f * (axy + ayz))
            p = glm::vec3(pick < 2.0f * axy + ayz ? e.x : -e.x, a * e.y, b * e.z);
        else
            p = glm::vec3(a * e.x, pick < 2.0f * (axy + ayz) + axz ? e.y : -e.y, b * e.z);
        break;
    }
    case PrimType::Sphere:
    {
        float y = 1.0f - 2.0f * r1;
        float rr = std::sqrt(std::max(0.0f, 1.0f - y * y));
        float phi = 2.0f * kPi * r2;
        p = r * glm::vec3(rr * std::cos(phi), y, rr * std::sin(phi));
        break;
    }
    case PrimType::Cylinder:
    {
        float side = 2.0f * kPi * r * h, cap = kPi * r * r;
        float pick = r0 * (side + 2.0f * cap);
        if (pick < side)
        {
            float phi = 2.0f * kPi * r1;
            p = glm::vec3(r * std::cos(phi), (r2 - 0.5f) * h, r * std::sin(phi));
        }
        else
        {
            float rho = r * std::sqrt(r1), phi = 2.0f * kPi * r2;
            p = glm::vec3(rho * std::cos(phi), pick < side + cap ? hh : -hh, rho * std::sin(phi));
        }
        break;
    }
    case PrimType::Cone:
    {
        float side = kPi * r * std::sqrt(r * r + h * h), base = kPi * r * r;
        float phi = 2.0f * kPi * r2;
        if (r0 * (side + base) < side)
        {
            float f = std::sqrt(r1); // fraction of the way from apex to base
            p = glm::vec3(f * r * std::cos(phi), hh - f * h, f * r * std::sin(phi));
        }
        else
        {
            float rho = r * std::sqrt(r1);
            p = glm::vec3(rho * std::cos(phi), -hh, rho * std::sin(phi));
        }
        break;
    }
    case PrimType::Mesh:
        if (mesh)
            p = r * mesh->surfacePoint(r0, r1, r2);
        break;
    }
    return position + rotation * p;
}

Aabb Primitive::bounds() const
{
    glm::vec3 e;
    switch (type)
    {
    case PrimType::Block: e = size * 0.5f; break;
    case PrimType::Sphere: e = glm::vec3(size.x); break;
    case PrimType::Mesh: e = mesh ? mesh->halfExtent() * size.x : glm::vec3(0.0f); break;
    default: e = glm::vec3(size.x, size.y * 0.5f, size.x); break;
    }
    glm::mat3 m = glm::mat3_cast(rotation);
    glm::vec3 we(0.0f);
    for (int c = 0; c < 3; ++c)
        we += glm::abs(m[c]) * e[c];
    if (type == PrimType::Sphere)
        we = e;
    return Aabb{ position - we, position + we };
}

std::vector<Curve> Primitive::edges() const
{
    std::vector<Curve> out;
    auto toWorld = [&](const glm::vec3& p) { return position + rotation * p; };
    auto line = [&](const glm::vec3& a, const glm::vec3& b) {
        Curve c;
        c.a = toWorld(a);
        c.b = toWorld(b);
        out.push_back(c);
    };
    auto circle = [&](const glm::vec3& center, const glm::vec3& u, const glm::vec3& v, float radius) {
        Curve c;
        c.circle = true;
        c.center = toWorld(center);
        c.u = rotation * u;
        c.v = rotation * v;
        c.radius = radius;
        out.push_back(c);
    };

    const glm::vec3 X(1, 0, 0), Y(0, 1, 0), Z(0, 0, 1);
    float r = size.x, hh = size.y * 0.5f;

    switch (type)
    {
    case PrimType::Block:
    {
        glm::vec3 e = size * 0.5f;
        for (int axis = 0; axis < 3; ++axis)
        {
            int i = (axis + 1) % 3, j = (axis + 2) % 3;
            for (float si : { -1.0f, 1.0f })
                for (float sj : { -1.0f, 1.0f })
                {
                    glm::vec3 a(0.0f), b(0.0f);
                    a[axis] = -e[axis];
                    b[axis] = e[axis];
                    a[i] = b[i] = si * e[i];
                    a[j] = b[j] = sj * e[j];
                    line(a, b);
                }
        }
        break;
    }
    case PrimType::Sphere:
    {
        circle(glm::vec3(0.0f), X, Z, r);
        for (float lat : { -0.5f, 0.5f })
            circle(glm::vec3(0.0f, r * lat, 0.0f), X, Z, r * std::sqrt(1.0f - lat * lat));
        for (int m = 0; m < 3; ++m)
        {
            float a = kPi * m / 3.0f;
            circle(glm::vec3(0.0f), glm::vec3(std::cos(a), 0.0f, std::sin(a)), Y, r);
        }
        break;
    }
    case PrimType::Cylinder:
        circle(glm::vec3(0.0f, -hh, 0.0f), X, Z, r);
        circle(glm::vec3(0.0f, hh, 0.0f), X, Z, r);
        for (int g = 0; g < 4; ++g)
        {
            float a = kPi * 0.5f * g;
            glm::vec3 rim(r * std::cos(a), 0.0f, r * std::sin(a));
            line(rim - Y * hh, rim + Y * hh);
        }
        break;
    case PrimType::Cone:
        circle(glm::vec3(0.0f, -hh, 0.0f), X, Z, r);
        for (int g = 0; g < 4; ++g)
        {
            float a = kPi * 0.5f * g;
            line(glm::vec3(r * std::cos(a), -hh, r * std::sin(a)), glm::vec3(0.0f, hh, 0.0f));
        }
        break;
    case PrimType::Mesh:
        if (mesh)
            for (const Curve& c : mesh->contours())
            {
                line(c.a * r, c.b * r);
                out.back().samples = c.samples;
            }
        break;
    }
    return out;
}

} // namespace csg
