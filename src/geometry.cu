#include"geometry.hpp"
#include"config.h"
#include<math.h>

constexpr F INF=inf*1.1;
template<class T> __host__ __device__ T intersect_base(const ray_base<T>& r,const triangle_base<T>& t)
{
	typedef vec_base<T> vec;
	vec n=t.n(); T rn=r.d.dot(n);
	if(abs(rn)<eps) return INF;
	T d=(t.a-r.o).dot(n)/rn;
	if(d<eps) return INF;
	vec p=r.o+r.d*d;
	T s1=(t.a-p).cross(t.b-p).len();
	T s2=(t.b-p).cross(t.c-p).len();
	T s3=(t.c-p).cross(t.a-p).len();
	T s=(t.a-t.b).cross(t.b-t.c).len();
	if(abs(s1+s2+s3-s)<eps) return d;
	else return INF;
}

template<class T> __host__ __device__ T intersect_base(const ray_base<T>& r,const sphere_base<T>& s)
{
	typedef vec_base<T> vec;
	vec O=s.o-r.o;
	T x=r.d.dot(O);
	T y=sqrt(O.dot(O)-x*x);
	if(y>s.r-eps) return INF;
	T X=sqrt(s.r*s.r-y*y);
	if(x<-X+eps) return INF;
	if(x<X) return x+X;
	return x-X;
}

__host__ __device__ F intersect(const ray& r,const triangle& t){ return intersect_base(r,t); }
__host__ __device__ F intersect(const ray& r,const sphere& s){ return intersect_base(r,s); }