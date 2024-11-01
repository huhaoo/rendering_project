#pragma once

// default float type, F in [float, double]
typedef float F;
constexpr F                   eps=1e-6;
constexpr F                   inf=1e30;
constexpr int          block_size=256;
// Max number of triangles in a bvh leaf node
constexpr int       bvh_leaf_size=4;
constexpr unsigned      rand_seed=123465789;
// Max depth of bvh tree
constexpr int       bvh_max_depth=31;