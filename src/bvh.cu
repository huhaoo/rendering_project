#include"config.h"
#include"geometry.hpp"
#include"bvh.hpp"

#include<vector>
#include<algorithm>
#include<random>
#include<math.h>

__host__ int constructure(triangle *p, int l, int r, std::vector<bvh_node> &b)
{
	static std::mt19937 Rand(rand_seed);
	int i=b.size(); b.push_back(bvh_node(p,l,r));
	if(r-l<=bvh_leaf_size) return i;
	int m=(l+r)/2; int type=rand()%3;
	switch(type)
	{
	case 0: std::nth_element(p+l,p+m,p+r,[&](const triangle &a,const triangle &b){ return a.a.x<b.a.x; }); break;
	case 1: std::nth_element(p+l,p+m,p+r,[&](const triangle &a,const triangle &b){ return a.a.y<b.a.y; }); break;
	case 2: std::nth_element(p+l,p+m,p+r,[&](const triangle &a,const triangle &b){ return a.a.z<b.a.z; }); break;
	}
	b[i].s[0]=constructure(p,l,m,b);
	b[i].s[1]=constructure(p,m,r,b);
	return i;
}
__host__ __device__ void intersect_bvh(ray &r, bvh_node *b, const triangle *t, int &id, F &d)
{
	int stake[bvh_max_depth]; int top; stake[top=0]=0;
	while(top>=0)
	{
		int i=stake[top];
		if(b[i].s[0]==-1)
		{
			for(int j=b[i].l;j<b[i].r;j++)
			{
				F d0=intersect(r,t[j]);
				if(d0<d){ d=d0; id=j; }
			}
			top--; continue;
		}
		F tl=0,tr=inf;
		if(abs(r.d.x)<eps){ if(r.o.x<b[i].a.x-eps||r.o.x>b[i].b.x+eps){ top--; continue; } }
		else if(r.d.x>0){ tl=max(tl,(b[i].a.x-r.o.x)/r.d.x); tr=min(tr,(b[i].b.x-r.o.x)/r.d.x); }
		else{ tl=max(tl,(b[i].b.x-r.o.x)/r.d.x); tr=min(tr,(b[i].a.x-r.o.x)/r.d.x); }
		if(abs(r.d.y)<eps){ if(r.o.y<b[i].a.y-eps||r.o.y>b[i].b.y+eps){ top--; continue; } }
		else if(r.d.y>0){ tl=max(tl,(b[i].a.y-r.o.y)/r.d.y); tr=min(tr,(b[i].b.y-r.o.y)/r.d.y); }
		else{ tl=max(tl,(b[i].b.y-r.o.y)/r.d.y); tr=min(tr,(b[i].a.y-r.o.y)/r.d.y); }
		if(abs(r.d.z)<eps){ if(r.o.z<b[i].a.z-eps||r.o.z>b[i].b.z+eps){ top--; continue; } }
		else if(r.d.z>0){ tl=max(tl,(b[i].a.z-r.o.z)/r.d.z); tr=min(tr,(b[i].b.z-r.o.z)/r.d.z); }
		else{ tl=max(tl,(b[i].b.z-r.o.z)/r.d.z); tr=min(tr,(b[i].a.z-r.o.z)/r.d.z); }
		if(tl-eps>tr){ top--; continue; }
		stake[top]=b[i].s[0]; stake[++top]=b[i].s[1];
	}
}