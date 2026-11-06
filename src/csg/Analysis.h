#pragma once

#include "csg/Scene.h"
#include "csg/Smc.h"

namespace csg {

// Analysis procedures on trees (not primitives): both run SMC against the tree.

// Volume by ray casting: a res x res grid of parallel rays through the tree's
// bounds, summing the length of the IN spans times the cell area.
double treeVolume(const Program& program, int res = 256);

// Surface area: points are sampled uniformly over every primitive's surface;
// those classified ON the tree's boundary count towards it.
double treeArea(const Program& program, int samplesPerPrim = 20000);

// 4D measure: integral of the volume over [t0, t1] (trapezoid rule).
double treeHypervolume(const Scene& scene, const Node* tree, float t0, float t1, int steps = 32, int res = 128);

} // namespace csg
