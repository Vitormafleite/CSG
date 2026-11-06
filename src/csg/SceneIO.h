#pragma once

#include "csg/Scene.h"

#include <string>

namespace csg {

// Text (.txt) save format for CSG trees. One "tree ... end" block per root;
// every node is a block closed by "end"; operation nodes hold their two
// subtrees, A first (order matters for difference). '#' starts a comment.
//
//   CSG 1
//   duration 5
//   tree
//     difference "Body"
//       primitive block "Block 1"
//         color 0.8 0.8 0.8
//         reflectivity 0
//         life 0 1                  # optional: exists only in [from, to]
//         position 0  0 0 0         # time  x y z    (one line per key)
//         rotation 0  1 0 0 0       # time  w x y z  (unit quaternion)
//         size     0  1 1 1         # time  size parameters (see Primitive.h)
//       end
//       primitive mesh "dragon 1"
//         file "dragon.stl"         # relative to the .txt file's folder
//         ...
//       end
//     end
//   end
//
// Primitive types: block sphere cylinder cone mesh.
// Operations: union intersection difference.

std::string saveSceneText(const Scene& scene, const std::string& baseDir = ".");
bool saveScene(const Scene& scene, const std::string& path, std::string& error);

// On failure the scene is left unchanged and error names the line.
bool loadSceneText(Scene& scene, const std::string& text, const std::string& baseDir, std::string& error);
bool loadScene(Scene& scene, const std::string& path, std::string& error);

} // namespace csg
