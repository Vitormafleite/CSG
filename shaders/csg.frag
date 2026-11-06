#version 330 core

// GPU ray casting of a CSG tree with set membership classification (SMC).
//
// This mirrors the C++ core: primitive intersections from src/csg/Primitive.cpp
// and the span algebra of csg::combine / csg::classifyRay from src/csg/Smc.cpp.
// Keep both in sync.
//
// Data texture layout (src/render/RayRenderer.cpp):
//   row 0, 4 texels per primitive: (pos, type) (rotation quat) (size, reflectivity) (color, highlight)
//          for a mesh, size = (scale, BVH node base, triangle base)
//   row 1, 1 texel per postfix op: (opcode, primitive index)
// Mesh buffers (src/csg/Mesh.h), indices relative to each mesh's base:
//   uMeshNodes, 2 texels per BVH node: (min, leftOrFirst) (max, count)
//   uMeshTris,  3 texels per triangle: (v0, nx) (e1, ny) (e2, nz)

// Built in two variants (RayRenderer): without meshes, and with MESH_SUPPORT 1,
// compiled only once a scene has a mesh. All loops over mesh data have
// dynamic bounds, so drivers cannot try to unroll them.
#ifndef MESH_SUPPORT
#define MESH_SUPPORT 0
#endif
// One program per render mode (uMode), so the tree evaluation is inlined
// once per program: 0 ray casting (also draws the wireframe background),
// 1 ray tracing, 2 SMC surfaces, 4 hit buffer.
#ifndef RENDER_MODE
#define RENDER_MODE 0
#endif

out vec4 fragColor;

uniform sampler2D uData;
#if MESH_SUPPORT
uniform samplerBuffer uMeshNodes;
uniform samplerBuffer uMeshTris;
#endif
uniform int uPrimCount;
uniform int uOpCount;
uniform int uMode;      // as RENDER_MODE, plus 3 wireframe (background only)
uniform int uBounces;
uniform int uGround;
uniform float uGroundY;
uniform float uSmcAlpha;
uniform vec3 uLightDir;
uniform vec3 uEye;
uniform mat4 uInvViewProj;
uniform vec2 uResolution;
uniform vec3 uBoundsMin; // bounds of the whole program: rays that miss it see nothing
uniform vec3 uBoundsMax;

const int STACK = 8;     // RayRenderer::kMaxStack
#if MESH_SUPPORT
const int MAXS = 16;     // max spans per list (a mesh can give many)
#else
const int MAXS = 8;
#endif
const float INF = 1e30;
const float SPAN_EPS = 1e-5;

// Span ends store a surface id: sign * (prim + 1 + ID_STRIDE * (triangle + 1)),
// negative = flipped normal, triangle = -1 for analytic primitives.
const int ID_STRIDE = 128; // > RayRenderer::kMaxPrims

const int OP_PUSH = 0, OP_UNION = 1, OP_INTER = 2, OP_DIFF = 3, OP_EMPTY = 4;
const int BLOCK = 0, SPHERE = 1, CYLINDER = 2, CONE = 3, MESH = 4;

#if MESH_SUPPORT
// Mesh line classification (mirrors csg::Mesh::hits / spans).
const int MAXHITS = 48;
const int MAXMESH = 8;   // mesh primitives per program (RayRenderer::kMaxMeshPrims)
// Window evaluation: the tree is evaluated on [wLo, wHi) only. A mesh's
// winding count at wLo is carried over from the previous window (wCarry, in
// the order the meshes appear in the program).
int wCarry[MAXMESH];
const float MESH_BARY_EPS = 2e-4;
const float MESH_MERGE = 1e-5;
float mT[MAXHITS];
int mTri[MAXHITS];
bool mIn[MAXHITS];
int mN;
#endif

const vec3 COLOR_IN = vec3(0.25, 0.85, 0.35);
const vec3 COLOR_ON = vec3(1.0, 0.85, 0.2);
const vec3 COLOR_OUT = vec3(0.9, 0.25, 0.25);

// Span list stack: list s occupies indices [s*MAXS, s*MAXS + sN[s]).
// Span ends store the bounding surface as (prim + 1), negative = flipped normal.
float sT0[STACK * MAXS];
float sT1[STACK * MAXS];
int sP0[STACK * MAXS];
int sP1[STACK * MAXS];
int sN[STACK];

struct Prim
{
    vec3 pos;
    int type;
    vec4 rot;
    vec3 size;
    float refl;
    vec3 color;
    bool highlight;
};

vec4 texel(int x, int y) { return texelFetch(uData, ivec2(x, y), 0); }

vec3 qrot(vec4 q, vec3 v) { return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v); }
vec3 qinvrot(vec4 q, vec3 v) { return qrot(vec4(-q.xyz, q.w), v); }

Prim loadPrim(int i)
{
    vec4 a = texel(i * 4, 0), b = texel(i * 4 + 1, 0), c = texel(i * 4 + 2, 0), d = texel(i * 4 + 3, 0);
    Prim p;
    p.pos = a.xyz;
    p.type = int(a.w + 0.5);
    p.rot = b;
    p.size = c.xyz;
    p.refl = c.w;
    p.color = d.rgb;
    p.highlight = d.w > 0.5;
    return p;
}

// ---------------------------------------------------------------- meshes
#if MESH_SUPPORT

bool lineBox(vec3 o, vec3 invD, vec3 bmin, vec3 bmax, out float t0, out float t1)
{
    vec3 ta = (bmin - o) * invD, tb = (bmax - o) * invD;
    vec3 lo = min(ta, tb), hi = max(ta, tb);
    t0 = max(max(lo.x, lo.y), lo.z);
    t1 = min(min(hi.x, hi.y), hi.z);
    return t0 <= t1;
}

// Every crossing of the line wo + t*wd with the mesh within [tlo, thi], sorted
// and cleaned up like csg::Mesh::hits, into mT / mTri / mIn.
void meshHits(Prim p, vec3 wo, vec3 wd, float tlo, float thi)
{
    mN = 0;
    int nodeBase = int(p.size.y + 0.5), triBase = int(p.size.z + 0.5);
    if (p.size.y < 0.0) return; // mesh not uploaded
    vec3 o = qinvrot(p.rot, wo - p.pos) / p.size.x;
    vec3 d = qinvrot(p.rot, wd) / p.size.x;
    vec3 invD;
    for (int i = 0; i < 3; ++i)
        invD[i] = abs(d[i]) < 1e-20 ? (d[i] < 0.0 ? -1e30 : 1e30) : 1.0 / d[i];

    float r0, r1;
    if (!lineBox(o, invD, texelFetch(uMeshNodes, nodeBase * 2).xyz, texelFetch(uMeshNodes, nodeBase * 2 + 1).xyz, r0, r1))
        return;
    float shift = r0;
    o += d * shift;
    float lo = tlo - shift, hi = thi - shift;

    int stack[64];
    int sp = 0;
    stack[sp++] = 0;
    while (sp > 0) // terminates: each interior node is pushed once
    {
        int ni = (nodeBase + stack[--sp]) * 2;
        vec4 a = texelFetch(uMeshNodes, ni), b = texelFetch(uMeshNodes, ni + 1);
        float t0, t1;
        if (!lineBox(o, invD, a.xyz, b.xyz, t0, t1) || t1 < lo || t0 > hi)
            continue;
        int first = int(a.w + 0.5), count = int(b.w + 0.5);
        if (count == 0)
        {
            if (sp + 2 <= 64)
            {
                stack[sp++] = first;
                stack[sp++] = first + 1;
            }
            continue;
        }
        for (int k = 0; k < count; ++k)
        {
            int ti = triBase + first + k;
            vec4 v0 = texelFetch(uMeshTris, ti * 3), e1 = texelFetch(uMeshTris, ti * 3 + 1), e2 = texelFetch(uMeshTris, ti * 3 + 2);
            vec3 pv = cross(d, e2.xyz);
            float det = dot(e1.xyz, pv);
            if (abs(det) < 1e-30) continue;
            float inv = 1.0 / det;
            vec3 sv = o - v0.xyz;
            float u = dot(sv, pv) * inv;
            if (u < -MESH_BARY_EPS || u > 1.0 + MESH_BARY_EPS) continue;
            vec3 qv = cross(sv, e1.xyz);
            float v = dot(d, qv) * inv;
            if (v < -MESH_BARY_EPS || u + v > 1.0 + MESH_BARY_EPS) continue;
            float t = dot(e2.xyz, qv) * inv;
            if (t < lo || t >= hi) continue; // half-open, so windows never share a crossing

            // insertion sort; when full, the farthest crossing is dropped
            int j = min(mN, MAXHITS - 1);
            if (mN == MAXHITS && t >= mT[MAXHITS - 1] - shift) continue;
            while (j > 0 && mT[j - 1] > t + shift)
            {
                mT[j] = mT[j - 1]; mTri[j] = mTri[j - 1]; mIn[j] = mIn[j - 1];
                --j;
            }
            mT[j] = t + shift; mTri[j] = ti; mIn[j] = det > 0.0;
            mN = min(mN + 1, MAXHITS);
        }
    }

    // Merge duplicates at one point, cancel grazing enter/leave pairs.
    float merge = MESH_MERGE / max(length(d), 1e-30);
    int n = 0;
    for (int i = 0; i < mN; ++i)
    {
        if (n > 0 && mT[i] - mT[n - 1] <= merge)
        {
            if (mIn[n - 1] != mIn[i]) --n;
            continue;
        }
        mT[n] = mT[i]; mTri[n] = mTri[i]; mIn[n] = mIn[i];
        ++n;
    }
    mN = n;
}

vec3 meshTriNormal(Prim p, int ti)
{
    vec3 n = vec3(texelFetch(uMeshTris, ti * 3).w, texelFetch(uMeshTris, ti * 3 + 1).w, texelFetch(uMeshTris, ti * 3 + 2).w);
    return qrot(p.rot, n);
}

#endif

// ---------------------------------------------------------------- primitives

bool clipSlab(float o, float d, float lo, float hi, inout float t0, inout float t1)
{
    if (abs(d) < 1e-12)
        return o >= lo && o <= hi;
    float ta = (lo - o) / d, tb = (hi - o) / d;
    if (ta > tb) { float s = ta; ta = tb; tb = s; }
    t0 = max(t0, ta);
    t1 = min(t1, tb);
    return t0 <= t1;
}

bool intersectPrim(Prim p, vec3 wo, vec3 wd, out float t0, out float t1)
{
    vec3 o = qinvrot(p.rot, wo - p.pos);
    vec3 d = qinvrot(p.rot, wd);
    t0 = -INF;
    t1 = INF;

    if (p.type == BLOCK)
    {
        vec3 h = p.size * 0.5;
        if (!clipSlab(o.x, d.x, -h.x, h.x, t0, t1)) return false;
        if (!clipSlab(o.y, d.y, -h.y, h.y, t0, t1)) return false;
        return clipSlab(o.z, d.z, -h.z, h.z, t0, t1);
    }
    if (p.type == SPHERE)
    {
        float r = p.size.x;
        float a = dot(d, d), b = dot(o, d), c = dot(o, o) - r * r;
        float disc = b * b - a * c;
        if (a < 1e-12 || disc < 0.0) return false;
        float s = sqrt(disc);
        t0 = (-b - s) / a;
        t1 = (-b + s) / a;
        return true;
    }
    if (p.type == CYLINDER)
    {
        float r = p.size.x, hh = p.size.y * 0.5;
        float a = d.x * d.x + d.z * d.z;
        float b = o.x * d.x + o.z * d.z;
        float c = o.x * o.x + o.z * o.z - r * r;
        if (a < 1e-12)
        {
            if (c > 0.0) return false;
        }
        else
        {
            float disc = b * b - a * c;
            if (disc < 0.0) return false;
            float s = sqrt(disc);
            t0 = (-b - s) / a;
            t1 = (-b + s) / a;
        }
        return clipSlab(o.y, d.y, -hh, hh, t0, t1);
    }

    // CONE: rho^2 - k^2 (apexY - y)^2 <= 0 within the slab |y| <= h/2
    float r = p.size.x, h = p.size.y, hh = h * 0.5;
    float k2 = (r / h) * (r / h);
    float w = hh - o.y, dw = -d.y;
    float a = d.x * d.x + d.z * d.z - k2 * dw * dw;
    float b = o.x * d.x + o.z * d.z - k2 * w * dw;
    float c = o.x * o.x + o.z * o.z - k2 * w * w;

    float s0 = -INF, s1 = INF;
    if (!clipSlab(o.y, d.y, -hh, hh, s0, s1)) return false;

    if (abs(a) < 1e-12)
    {
        if (abs(b) < 1e-12)
        {
            if (c > 0.0) return false;
            t0 = s0; t1 = s1;
            return true;
        }
        float ts = -c / (2.0 * b);
        if (b > 0.0) t1 = ts; else t0 = ts;
        t0 = max(t0, s0);
        t1 = min(t1, s1);
        return t0 <= t1;
    }

    float disc = b * b - a * c;
    if (a > 0.0)
    {
        if (disc < 0.0) return false;
        float s = sqrt(disc);
        t0 = max((-b - s) / a, s0);
        t1 = min((-b + s) / a, s1);
        return t0 <= t1;
    }
    if (disc < 0.0)
    {
        t0 = s0; t1 = s1;
        return true;
    }
    float s = sqrt(disc);
    float r1 = (-b + s) / a, r2 = (-b - s) / a;
    float a0 = s0, a1 = min(r1, s1);
    float b0 = max(r2, s0), b1 = s1;
    bool hasA = a0 <= a1, hasB = b0 <= b1;
    if (hasA && (!hasB || a1 - a0 >= b1 - b0)) { t0 = a0; t1 = a1; return true; }
    if (hasB) { t0 = b0; t1 = b1; return true; }
    return false;
}

vec3 primNormal(Prim p, vec3 wp)
{
    vec3 q = qinvrot(p.rot, wp - p.pos);
    vec3 n = vec3(0.0, 1.0, 0.0);
    if (p.type == BLOCK)
    {
        vec3 a = abs(q) / (p.size * 0.5);
        if (a.x >= a.y && a.x >= a.z) n = vec3(sign(q.x), 0.0, 0.0);
        else if (a.y >= a.z) n = vec3(0.0, sign(q.y), 0.0);
        else n = vec3(0.0, 0.0, sign(q.z));
    }
    else if (p.type == SPHERE)
    {
        n = normalize(q);
    }
    else if (p.type == CYLINDER)
    {
        float rho = length(q.xz);
        float dSide = abs(rho - p.size.x);
        float dCap = abs(abs(q.y) - p.size.y * 0.5);
        if (dCap < dSide || rho < 1e-6) n = vec3(0.0, sign(q.y), 0.0);
        else n = vec3(q.x / rho, 0.0, q.z / rho);
    }
    else
    {
        float hh = p.size.y * 0.5, k = p.size.x / p.size.y;
        float rho = length(q.xz);
        float dCap = abs(q.y + hh);
        float dSide = abs(rho - k * (hh - q.y)) / sqrt(1.0 + k * k);
        if (dCap < dSide) n = vec3(0.0, -1.0, 0.0);
        else if (rho > 1e-6) n = normalize(vec3(q.x / rho, k, q.z / rho));
    }
    return qrot(p.rot, n);
}

// ---------------------------------------------------------------- span algebra (SMC)

float endT(int base, int k) { return (k % 2 == 0) ? sT0[base + k / 2] : sT1[base + k / 2]; }
int endP(int base, int k) { return (k % 2 == 0) ? sP0[base + k / 2] : sP1[base + k / 2]; }

// Combines the two lists on top of the stack into list sp-2 (see csg::combine).
void combineTop(int op, int sp)
{
    float oT0[MAXS], oT1[MAXS];
    int oP0[MAXS], oP1[MAXS];

    int la = sp - 2, lb = sp - 1;
    int baseA = la * MAXS, baseB = lb * MAXS;
    int na = sN[la] * 2, nb = sN[lb] * 2;
    int ia = 0, ib = 0, n = 0;
    bool inA = false, inB = false, inR = false;
    float curT0 = 0.0;
    int curP0 = 0;

    while (ia < na || ib < nb) // one list end per iteration
    {
        float ta = ia < na ? endT(baseA, ia) : INF;
        float tb = ib < nb ? endT(baseB, ib) : INF;
        float t;
        int prim;
        if (ta <= tb)
        {
            t = ta;
            inA = (ia % 2) == 0;
            prim = endP(baseA, ia);
            ++ia;
        }
        else
        {
            t = tb;
            inB = (ib % 2) == 0;
            prim = endP(baseB, ib);
            if (op == OP_DIFF) prim = -prim;
            ++ib;
        }

        bool r = op == OP_UNION ? (inA || inB) : op == OP_INTER ? (inA && inB) : (inA && !inB);
        if (r && !inR)
        {
            curT0 = t;
            curP0 = prim;
        }
        else if (!r && inR)
        {
            if (n > 0 && curT0 - oT1[n - 1] <= SPAN_EPS)
            {
                oT1[n - 1] = t;
                oP1[n - 1] = prim;
            }
            else if (t - curT0 > SPAN_EPS && n < MAXS)
            {
                oT0[n] = curT0; oT1[n] = t;
                oP0[n] = curP0; oP1[n] = prim;
                ++n;
            }
        }
        inR = r;
    }

    for (int k = 0; k < MAXS; ++k)
    {
        if (k >= n) break;
        sT0[baseA + k] = oT0[k]; sT1[baseA + k] = oT1[k];
        sP0[baseA + k] = oP0[k]; sP1[baseA + k] = oP1[k];
    }
    sN[la] = n;
}

// Ray SMC against the whole tree; the IN spans end up in list 0.
float wLo = -INF, wHi = INF;
#ifndef WINDOW_ENDS
#define WINDOWS 4
#define WINDOW_ENDS 0.1, 0.3, 0.6, 1.0
#endif
const float WINDOW_END[WINDOWS] = float[](WINDOW_ENDS);

// The result is exact on [wLo, wHi): mesh spans are cut at the window ends
// (analytic ones need no cut, set membership is pointwise).
void evaluate(vec3 ro, vec3 rd)
{
    int sp = 0;
    sN[0] = 0;
#if MESH_SUPPORT
    int mi = 0;
#endif
    for (int i = 0; i < uOpCount; ++i)
    {
        vec4 op = texel(i, 1);
        int code = int(op.x + 0.5);
        if (code == OP_PUSH || code == OP_EMPTY)
        {
            int base = sp * MAXS;
            sN[sp] = 0;
            float t0, t1;
            if (code == OP_PUSH)
            {
                int pi = int(op.y + 0.5);
                Prim pr = loadPrim(pi);
#if MESH_SUPPORT
                if (pr.type == MESH)
                {
                    // IN spans from the winding count of the crossings, starting
                    // from the count carried into this window.
                    meshHits(pr, ro, rd, wLo, wHi);
                    int winding = mi < MAXMESH ? wCarry[mi] : 0, n = 0, startTri = -1;
                    float start = wLo;
                    for (int h = 0; h < mN; ++h)
                    {
                        if (mIn[h])
                        {
                            if (winding++ == 0) { start = mT[h]; startTri = mTri[h]; }
                        }
                        else if (winding > 0 && --winding == 0 && n < MAXS)
                        {
                            sT0[base + n] = start; sT1[base + n] = mT[h];
                            sP0[base + n] = pi + 1 + ID_STRIDE * (startTri + 1);
                            sP1[base + n] = pi + 1 + ID_STRIDE * (mTri[h] + 1);
                            ++n;
                        }
                    }
                    if (winding > 0 && n < MAXS) // still inside at the window end
                    {
                        sT0[base + n] = start; sT1[base + n] = wHi;
                        sP0[base + n] = pi + 1 + ID_STRIDE * (startTri + 1);
                        sP1[base + n] = pi + 1;
                        ++n;
                    }
                    if (mi < MAXMESH) wCarry[mi] = winding;
                    ++mi;
                    sN[sp] = n;
                }
                else
#endif
                if (intersectPrim(pr, ro, rd, t0, t1))
                {
                    sT0[base] = t0; sT1[base] = t1;
                    sP0[base] = pi + 1; sP1[base] = pi + 1;
                    sN[sp] = 1;
                }
            }
            ++sp;
        }
        else
        {
            combineTop(code, sp);
            --sp;
        }
    }
}

int primOf(int id) { return abs(id) % ID_STRIDE - 1; }
int triOf(int id) { return abs(id) / ID_STRIDE - 1; }

bool rayBox(vec3 o, vec3 d, vec3 bmin, vec3 bmax, out float t0, out float t1)
{
    vec3 inv = 1.0 / d; // +-inf for zero components is fine here
    vec3 ta = (bmin - o) * inv, tb = (bmax - o) * inv;
    vec3 lo = min(ta, tb), hi = max(ta, tb);
    t0 = max(max(lo.x, lo.y), lo.z);
    t1 = min(min(hi.x, hi.y), hi.z);
    return t0 <= t1;
}

// Evaluates the tree along the ray window by window and stops at the first
// window with an answer, so the mesh traversals stop near the first hit.
//   shadow = false: the first entry into the solid with t > tmin (t, id)
//   shadow = true:  whether any part of the solid lies beyond tmin
// Without meshes a single window covers the whole line.
bool march(vec3 ro, vec3 rd, float tmin, bool shadow, out float t, out int id)
{
    t = INF;
    id = 0;
    float ba, bb;
    if (!rayBox(ro, rd, uBoundsMin, uBoundsMax, ba, bb) || bb <= tmin)
        return false;
#if MESH_SUPPORT
    for (int m = 0; m < MAXMESH; ++m) wCarry[m] = 0;
    // Window ends at growing fractions of the bounds, the last one open.
    float start = max(ba, tmin), len = bb - start;
    for (int w = 0; w < WINDOWS; ++w)
    {
        wLo = w == 0 ? -INF : wHi;
        wHi = w == WINDOWS - 1 ? INF : start + len * WINDOW_END[w];
#else
    {
        wLo = -INF;
        wHi = INF;
#endif
        evaluate(ro, rd);
        float lo = max(tmin, wLo);
        for (int k = 0; k < sN[0]; ++k)
        {
            if (shadow)
            {
                if (sT1[k] > lo && sT0[k] < wHi) return true;
            }
            else if (sT0[k] > lo && sT0[k] < wHi) // an entry at wLo is a carried span, not a surface
            {
                t = sT0[k];
                id = sP0[k];
                return true;
            }
        }
    }
    return false;
}

// ---------------------------------------------------------------- shading

vec3 background(vec3 rd)
{
    float f = 0.5 * (rd.y + 1.0);
    return mix(vec3(0.13, 0.14, 0.17), vec3(0.30, 0.34, 0.40), f);
}

vec3 baseColor(Prim p)
{
    return p.highlight ? mix(p.color, vec3(1.0, 0.62, 0.15), 0.45) : p.color;
}

vec3 surfaceNormal(Prim p, int id, vec3 pos, vec3 rd)
{
#if MESH_SUPPORT
    vec3 n = normalize(p.type == MESH ? meshTriNormal(p, triOf(id)) : primNormal(p, pos));
#else
    vec3 n = normalize(primNormal(p, pos));
#endif
    if (id < 0) n = -n; // boundary from a subtracted solid
    if (dot(n, rd) > 0.0) n = -n;
    return n;
}

#if RENDER_MODE == 0
vec3 rayCast(vec3 ro, vec3 rd)
{
    float t;
    int id;
    if (!march(ro, rd, 0.0, false, t, id))
        return background(rd);
    vec3 p = ro + rd * t;
    Prim pr = loadPrim(primOf(id));
    vec3 n = surfaceNormal(pr, id, p, rd);
    float key = max(dot(n, normalize(uLightDir)), 0.0);
    float head = max(dot(n, -rd), 0.0);
    return baseColor(pr) * (0.12 + 0.55 * key + 0.4 * head);
}
#endif

#if RENDER_MODE == 1
float groundHit(vec3 ro, vec3 rd)
{
    if (uGround == 0 || rd.y > -1e-6) return INF;
    float t = (uGroundY - ro.y) / rd.y;
    return t > 1e-3 ? t : INF;
}

vec3 groundColor(vec3 p)
{
    vec2 cell = floor(p.xz);
    float checker = mod(cell.x + cell.y, 2.0);
    return mix(vec3(0.42), vec3(0.55), checker);
}

// Whitted-style: per bounce a primary ray, then a shadow ray from its hit.
// The two alternate in one loop, so the tree is evaluated at a single call site.
vec3 rayTrace(vec3 ro, vec3 rd)
{
    vec3 color = vec3(0.0);
    vec3 weight = vec3(1.0);
    vec3 L = normalize(uLightDir);

    vec3 p = vec3(0.0), n = vec3(0.0, 1.0, 0.0), base = vec3(0.0);
    float refl = 0.0, tg = INF;
    bool onGround = false;
    for (int step = 0; step < 2 * uBounces + 2; ++step)
    {
        bool shadow = (step % 2) == 1;
        float t;
        int id;
        bool found = march(shadow ? p + n * 1e-3 : ro, shadow ? L : rd, 1e-3, shadow, t, id);

        if (!shadow)
        {
            bool hit = found;
            tg = groundHit(ro, rd);
            if (!hit && tg >= INF)
            {
                color += weight * background(rd);
                break;
            }
            onGround = !(hit && t < tg);
            if (!onGround)
            {
                p = ro + rd * t;
                Prim pr = loadPrim(primOf(id));
                n = surfaceNormal(pr, id, p, rd);
                base = baseColor(pr);
                refl = pr.refl;
            }
            else
            {
                p = ro + rd * tg;
                n = vec3(0.0, 1.0, 0.0);
                base = groundColor(p);
                refl = 0.1;
            }
            continue;
        }

        float lit = found ? 0.0 : 1.0;
        float diff = max(dot(n, L), 0.0) * lit;
        float spec = pow(max(dot(n, normalize(L - rd)), 0.0), 48.0) * lit;
        vec3 local = base * (0.15 + 0.85 * diff) + vec3(0.4) * spec;
        if (onGround)
            local = mix(local, background(rd), clamp(tg / 60.0, 0.0, 1.0)); // fade the ground

        color += weight * (1.0 - refl) * local;
        weight *= refl * mix(vec3(1.0), base, 0.3);
        if (refl < 0.01) break;

        ro = p + n * 1e-3;
        rd = reflect(rd, n);
    }
    return color;
}
#endif

#if RENDER_MODE == 2
// Every primitive surface along the ray, classified against the tree:
// IN (inside the solid), ON (part of its boundary), OUT (cut away).
vec3 smcView(vec3 ro, vec3 rd)
{
#if MESH_SUPPORT
    const int MAXH = 48;
#else
    const int MAXH = 32;
#endif
    float hT[MAXH];
    vec3 hC[MAXH];
    float hA[MAXH];
    int count = 0;

#if MESH_SUPPORT
    for (int m = 0; m < MAXMESH; ++m) wCarry[m] = 0;
#endif
    wLo = -INF;
    wHi = INF;
    evaluate(ro, rd); // every span is needed; they stay in list 0

    for (int i = 0; i < uPrimCount; ++i)
    {
        Prim pr = loadPrim(i);
        float t0, t1;
        int surfaces = 2;
#if MESH_SUPPORT
        if (pr.type == MESH)
        {
            meshHits(pr, ro, rd, 1e-4, INF);
            surfaces = mN;
        }
        else
#endif
        if (!intersectPrim(pr, ro, rd, t0, t1))
            continue;

        for (int e = 0; e < surfaces; ++e)
        {
#if MESH_SUPPORT
            float t = pr.type == MESH ? mT[e] : e == 0 ? t0 : t1;
#else
            float t = e == 0 ? t0 : t1;
#endif
            if (t <= 1e-4 || count >= MAXH) continue;

            float eps = 1e-4 * (1.0 + t);
            bool on = false, inside = false;
            for (int k = 0; k < sN[0]; ++k)
            {
                if (abs(t - sT0[k]) < eps || abs(t - sT1[k]) < eps) on = true;
                else if (t > sT0[k] && t < sT1[k]) inside = true;
            }

#if MESH_SUPPORT
            vec3 n = normalize(pr.type == MESH ? meshTriNormal(pr, mTri[e]) : primNormal(pr, ro + rd * t));
#else
            vec3 n = normalize(primNormal(pr, ro + rd * t));
#endif
            float shade = 0.35 + 0.65 * abs(dot(n, rd));
            vec3 c = (on ? COLOR_ON : inside ? COLOR_IN : COLOR_OUT) * shade;
            float a = on ? 0.9 : uSmcAlpha;
            if (pr.highlight) c = mix(c, vec3(1.0), 0.15);

            // insertion sort by distance
            int j = count;
            while (j > 0 && hT[j - 1] > t)
            {
                hT[j] = hT[j - 1]; hC[j] = hC[j - 1]; hA[j] = hA[j - 1];
                --j;
            }
            hT[j] = t; hC[j] = c; hA[j] = a;
            ++count;
        }
    }

    vec3 color = vec3(0.0);
    float transmit = 1.0;
    for (int k = 0; k < count; ++k)
    {
        color += transmit * hA[k] * hC[k];
        transmit *= 1.0 - hA[k];
    }
    return color + transmit * background(rd);
}
#endif

void main()
{
    vec2 ndc = gl_FragCoord.xy / uResolution * 2.0 - 1.0;
    vec4 far = uInvViewProj * vec4(ndc, 1.0, 1.0);
    vec3 ro = uEye;
    vec3 rd = normalize(far.xyz / far.w - uEye);

#if RENDER_MODE == 4
    // Raw first hit for tests/gpu_check.cpp: (t, signed primitive id), or t = -1 on a miss.
    float t;
    int id;
    fragColor = vec4(-1.0, 0.0, 0.0, 1.0);
    if (uOpCount > 0)
    {
        if (march(ro, rd, 0.0, false, t, id))
            fragColor = vec4(t, float(id < 0 ? -(primOf(id) + 1) : primOf(id) + 1), 0.0, 1.0);
    }
#else
    vec3 color;
    if (uOpCount == 0 || uMode == 3)
        color = background(rd);
    else
#if RENDER_MODE == 1
        color = rayTrace(ro, rd);
#elif RENDER_MODE == 2
        color = smcView(ro, rd);
#else
        color = rayCast(ro, rd);
#endif
    fragColor = vec4(color, 1.0);
#endif
}
