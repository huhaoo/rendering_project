#pragma once

#include"geometry.hpp"
#include"config.h"
#include"bvh.hpp"

#include<cuda_runtime.h>
#include<vector>

std::vector<std::pair<int,F> > intersect_ray_triangle(std::vector<ray> r, std::vector<triangle> t, std::vector<bvh_node> b);
