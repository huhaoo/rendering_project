#pragma once

#include<cuda_runtime.h>
#include<math.h>
#include<stdlib.h>
#include<assert.h>
#include"config.h"
#include<algorithm>

template<class T> class vec_base
{
public:
	T x,y,z;
	__host__ __device__ vec_base():x(0),y(0),z(0){}
	__host__ __device__ vec_base(T X,T Y,T Z):x(X),y(Y),z(Z){}
	__host__ __device__ vec_base(const vec_base<T>& v):x(v.x),y(v.y),z(v.z){}

	__host__ __device__ T len()const{ return sqrt(x*x+y*y+z*z); }
	__host__ __device__ vec_base<T> normalize()const{ T l=len(); return vec_base<T>(x/l,y/l,z/l); }
	__host__ __device__ vec_base<T> cross(const vec_base<T>& v)const{ return vec_base<T>(y*v.z-z*v.y,z*v.x-x*v.z,x*v.y-y*v.x); }
	__host__ __device__ T dot(const vec_base<T>& v)const{ return x*v.x+y*v.y+z*v.z; }
	__host__ __device__ vec_base<T> operator+(const vec_base<T>& v)const{ return vec_base<T>(x+v.x,y+v.y,z+v.z); }
	__host__ __device__ vec_base<T> operator-(const vec_base<T>& v)const{ return vec_base<T>(x-v.x,y-v.y,z-v.z); }
	__host__ __device__ vec_base<T> operator*(const vec_base<T>& v)const{ return cross(v); }
	__host__ __device__ vec_base<T> operator*(T t)const{ return vec_base<T>(x*t,y*t,z*t); }
	__host__ __device__ vec_base<T> operator/(T t)const{ return vec_base<T>(x/t,y/t,z/t); }
	__host__ __device__ vec_base<T> operator-()const{ return vec_base<T>(-x,-y,-z); }
	__host__ __device__ bool operator==(const vec_base<T>& v)const{ return abs(x-v.x)<eps && abs(y-v.y)<eps && abs(z-v.z)<eps; }
	__host__ __device__ bool operator!=(const vec_base<T>& v)const{ return !(*this==v); }
	// pointwise multiplication
	__host__ __device__ vec_base<T> pwmul(const vec_base<T>& v)const{ return vec_base<T>(x*v.x,y*v.y,z*v.z); }
	// pointwise division
	__host__ __device__ vec_base<T> pwdiv(const vec_base<T>& v)const{ return vec_base<T>(x/v.x,y/v.y,z/v.z); }
	__host__ __device__ vec_base<T> pwmin(const vec_base<T>& v)const{ return vec_base<T>(min(x,v.x),min(y,v.y),min(z,v.z)); }
	__host__ __device__ vec_base<T> pwmax(const vec_base<T>& v)const{ return vec_base<T>(max(x,v.x),max(y,v.y),max(z,v.z)); }
};

template<class T> class triangle_base
{
public:
	vec_base<T> a,b,c;
	__host__ __device__ triangle_base():a(),b(),c(){}
	__host__ __device__ triangle_base(const vec_base<T>& A,const vec_base<T>& B,const vec_base<T>& C):a(A),b(B),c(C){}
	__host__ __device__ triangle_base(const triangle_base<T>& t):a(t.a),b(t.b),c(t.c){}
	// normal vector
	__host__ __device__ vec_base<T> n()const{ return (b-a).cross(c-a).normalize(); }
};

template<class T> class ray_base
{
public:
	vec_base<T> o,d;
	// __host__ __device__ ray_base():o(),d(){} // Invalid
	__host__ __device__ ray_base(const vec_base<T>& O,const vec_base<T>& D):o(O){ assert(D.len()>eps); d=D.normalize(); }
	__host__ __device__ ray_base(const ray_base<T>& r):o(r.o),d(r.d){}
};

template<class T> class sphere_base
{
public:
	vec_base<T> o;
	T r;
	__host__ __device__ sphere_base():o(),r(0){}
	__host__ __device__ sphere_base(const vec_base<T>& O,T R):o(O),r(R){}
	__host__ __device__ sphere_base(const sphere_base<T>& s):o(s.o),r(s.r){}
};

/*
 * return >inf                                                         if no intersection
 * return the distance from the ray origin to the intersection point   otherwise
 */
template<class T> __host__ __device__ T intersect_base(const ray_base<T>& r,const triangle_base<T>& t);

/*
 * return >inf                                                         if no intersection
 * return the distance from the ray origin to the intersection point   otherwise
 */
template<class T> __host__ __device__ T intersect_base(const ray_base<T>& r,const sphere_base<T>& s);

typedef vec_base<F> vec;
typedef triangle_base<F> triangle;
typedef ray_base<F> ray;
typedef sphere_base<F> sphere;
__host__ __device__ F intersect(const ray& r,const triangle& t);
__host__ __device__ F intersect(const ray& r,const sphere& s);
