#version 330 core

// GPU ray casting of a CSG tree with set membership classification (SMC).
//
// This mirrors the C++ core: primitive intersections from src/csg/Primitive.cpp
// and the span algebra of csg::combine / csg::classifyRay from src/csg/Smc.cpp.
// Keep both in sync.
//
// Data texture layout (src/render/RayRenderer.cpp):
//   row 0, 4 texels per primitive: (pos, type) (rotation quat) (size, reflectivity) (color, highlight)
//   row 1, 1 texel per postfix op: (opcode, primitive index)

out vec4 fragColor;

uniform sampler2D uData;
uniform int uPrimCount;
uniform int uOpCount;
uniform int uMode;      // 0 ray casting, 1 ray tracing, 2 SMC surfaces, 3 wireframe (background only), 4 hit buffer
uniform int uBounces;
uniform int uGround;
uniform float uGroundY;
uniform float uSmcAlpha;
uniform vec3 uLightDir;
uniform vec3 uEye;
uniform mat4 uInvViewProj;
uniform vec2 uResolution;

const int STACK = 8;     // RayRenderer::kMaxStack
const int MAXS = 8;      // max spans per list
const float INF = 1e30;
const float SPAN_EPS = 1e-5;

const int OP_PUSH = 0, OP_UNION = 1, OP_INTER = 2, OP_DIFF = 3, OP_EMPTY = 4;
const int BLOCK = 0, SPHERE = 1, CYLINDER = 2, CONE = 3;

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

    for (int it = 0; it < 4 * MAXS; ++it)
    {
        if (ia >= na && ib >= nb) break;
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
void evaluate(vec3 ro, vec3 rd)
{
    int sp = 0;
    sN[0] = 0;
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
                if (intersectPrim(loadPrim(pi), ro, rd, t0, t1))
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

// First entry into the solid in front of tmin.
bool firstHit(vec3 ro, vec3 rd, float tmin, out float t, out int id)
{
    t = INF;
    id = 0;
    evaluate(ro, rd);
    for (int k = 0; k < MAXS; ++k)
    {
        if (k >= sN[0]) break;
        if (sT0[k] > tmin)
        {
            t = sT0[k];
            id = sP0[k];
            return true;
        }
    }
    return false;
}

bool occluded(vec3 ro, vec3 rd)
{
    evaluate(ro, rd);
    for (int k = 0; k < MAXS; ++k)
    {
        if (k >= sN[0]) break;
        if (sT1[k] > 1e-3) return true;
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
    vec3 n = normalize(primNormal(p, pos));
    if (id < 0) n = -n; // boundary from a subtracted solid
    if (dot(n, rd) > 0.0) n = -n;
    return n;
}

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

vec3 rayCast(vec3 ro, vec3 rd)
{
    float t;
    int id;
    if (!firstHit(ro, rd, 0.0, t, id))
        return background(rd);
    vec3 p = ro + rd * t;
    Prim pr = loadPrim(abs(id) - 1);
    vec3 n = surfaceNormal(pr, id, p, rd);
    float key = max(dot(n, normalize(uLightDir)), 0.0);
    float head = max(dot(n, -rd), 0.0);
    return baseColor(pr) * (0.12 + 0.55 * key + 0.4 * head);
}

vec3 rayTrace(vec3 ro, vec3 rd)
{
    vec3 color = vec3(0.0);
    vec3 weight = vec3(1.0);
    vec3 L = normalize(uLightDir);

    for (int bounce = 0; bounce <= uBounces; ++bounce)
    {
        float t;
        int id;
        bool hit = firstHit(ro, rd, 1e-3, t, id);
        float tg = groundHit(ro, rd);
        if (!hit && tg >= INF)
        {
            color += weight * background(rd);
            break;
        }

        vec3 p, n, base;
        float refl;
        if (hit && t < tg)
        {
            p = ro + rd * t;
            Prim pr = loadPrim(abs(id) - 1);
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

        float lit = occluded(p + n * 1e-3, L) ? 0.0 : 1.0;
        float diff = max(dot(n, L), 0.0) * lit;
        float spec = pow(max(dot(n, normalize(L - rd)), 0.0), 48.0) * lit;
        vec3 local = base * (0.15 + 0.85 * diff) + vec3(0.4) * spec;
        if (!(hit && t < tg))
            local = mix(local, background(rd), clamp(tg / 60.0, 0.0, 1.0)); // fade the ground

        color += weight * (1.0 - refl) * local;
        weight *= refl * mix(vec3(1.0), base, 0.3);
        if (refl < 0.01) break;

        ro = p + n * 1e-3;
        rd = reflect(rd, n);
    }
    return color;
}

// Every primitive surface along the ray, classified against the tree:
// IN (inside the solid), ON (part of its boundary), OUT (cut away).
vec3 smcView(vec3 ro, vec3 rd)
{
    const int MAXH = 32;
    float hT[MAXH];
    vec3 hC[MAXH];
    float hA[MAXH];
    int count = 0;

    evaluate(ro, rd); // result spans stay in list 0

    for (int i = 0; i < uPrimCount; ++i)
    {
        Prim pr = loadPrim(i);
        float t0, t1;
        if (!intersectPrim(pr, ro, rd, t0, t1)) continue;

        for (int e = 0; e < 2; ++e)
        {
            float t = e == 0 ? t0 : t1;
            if (t <= 1e-4 || count >= MAXH) continue;

            float eps = 1e-4 * (1.0 + t);
            bool on = false, inside = false;
            for (int k = 0; k < MAXS; ++k)
            {
                if (k >= sN[0]) break;
                if (abs(t - sT0[k]) < eps || abs(t - sT1[k]) < eps) on = true;
                else if (t > sT0[k] && t < sT1[k]) inside = true;
            }

            vec3 n = normalize(primNormal(pr, ro + rd * t));
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
    for (int k = 0; k < MAXH; ++k)
    {
        if (k >= count) break;
        color += transmit * hA[k] * hC[k];
        transmit *= 1.0 - hA[k];
    }
    return color + transmit * background(rd);
}

void main()
{
    vec2 ndc = gl_FragCoord.xy / uResolution * 2.0 - 1.0;
    vec4 far = uInvViewProj * vec4(ndc, 1.0, 1.0);
    vec3 ro = uEye;
    vec3 rd = normalize(far.xyz / far.w - uEye);

    if (uMode == 4)
    {
        // Raw first hit for tests/gpu_check.cpp: (t, signed surface id), or t = -1 on a miss.
        float t;
        int id;
        fragColor = uOpCount > 0 && firstHit(ro, rd, 0.0, t, id) ? vec4(t, float(id), 0.0, 1.0) : vec4(-1.0, 0.0, 0.0, 1.0);
        return;
    }

    vec3 color;
    if (uOpCount == 0 || uMode == 3)
        color = background(rd);
    else if (uMode == 1)
        color = rayTrace(ro, rd);
    else if (uMode == 2)
        color = smcView(ro, rd);
    else
        color = rayCast(ro, rd);

    fragColor = vec4(color, 1.0);
}
