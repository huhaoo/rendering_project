#pragma once

#include"config.h"
#include"geometry.hpp"

#include<vector>
#include<algorithm>
#include<random>
#include<cuda_runtime.h>

/*
 * Sequential bvh: store the index of sons and the index of the triangles in the leaf node. That is, we assume the array of triangles and bvh nodes are fixed.
 * Root is always 0.
 */
class bvh_node
{
public:
	int s[2];
	vec a,b;
	int l,r;
	bvh_node(triangle *p,int L,int R):s{-1,-1},l(L),r(R)
	{
		a=b=p[l].a;
		for(int i=l;i<r;i++)
		{
			a=a.pwmin(p[i].a).pwmin(p[i].b).pwmin(p[i].c);
			b=b.pwmax(p[i].a).pwmax(p[i].b).pwmax(p[i].c);
		}
	}
};
int constructure(triangle *p, int l, int r, std::vector<bvh_node> &b);
__host__ __device__ void intersect_bvh(const ray &r, const bvh_node *b, const triangle *t, int &id, F &d);