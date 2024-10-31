#pragma once

// #include<cuda_runtime.h>
#include<math.h>

#define __host__
#define __device__

__host__ __device__ template<class T> class vec_base
{
	static T eps=1e-6;
public:
	void set_eps(T e){ eps=e; }
	T x,y,z;
	vec_base():x(0),y(0),z(0){}
	vec_base(T X,T Y,T Z):x(X),y(Y),z(Z){}
	vec_base(const vec_base<T>& v):x(v.x),y(v.y),z(v.z){}
	T len(){ return sqrt(x*x+y*y+z*z); }
	vec_base<T> normalize(){ T l=len(); return vec_base<T>(x/l,y/l,z/l); }
	vec_base<T> cross(const vec_base<T>& v){ return vec_base<T>(y*v.z-z*v.y,z*v.x-x*v.z,x*v.y-y*v.x); }
	T dot(const vec_base<T>& v){ return x*v.x+y*v.y+z*v.z; }
};
__host__ __device__ template<class T> vec_base<T> operator+(const vec_base<T>& a,const vec_base<T>& b){ return vec_base<T>(a.x+b.x,a.y+b.y,a.z+b.z); }
__host__ __device__ template<class T> vec_base<T> operator-(const vec_base<T>& a,const vec_base<T>& b){ return vec_base<T>(a.x-b.x,a.y-b.y,a.z-b.z); }
__host__ __device__ template<class T> vec_base<T> operator*(const vec_base<T>& a,const vec_base<T>& b){ return vec_base<T>(a.x*b.x,a.y*b.y,a.z*b.z); }
__host__ __device__ template<class T> vec_base<T> operator/(const vec_base<T>& a,const vec_base<T>& b){ return vec_base<T>(a.x/b.x,a.y/b.y,a.z/b.z); }
__host__ __device__ template<class T> vec_base<T> operator*(const vec_base<T>& a,const T& b){ return vec_base<T>(a.x*b,a.y*b,a.z*b); }
__host__ __device__ template<class T> vec_base<T> operator/(const vec_base<T>& a,const T& b){ return vec_base<T>(a.x/b,a.y/b,a.z/b); }
__host__ __device__ template<class T> vec_base<T> operator*(const T& a,const vec_base<T>& b){ return vec_base<T>(a*b.x,a*b.y,a*b.z); }
__host__ __device__ template<class T> vec_base<T> operator/(const T& a,const vec_base<T>& b){ return vec_base<T>(a/b.x,a/b.y,a/b.z); }
__host__ __device__ template<class T> vec_base<T> operator-(const vec_base<T>& a){ return vec_base<T>(-a.x,-a.y,-a.z); }
__host__ __device__ template<class T> bool operator==(const vec_base<T>& a,const vec_base<T>& b){ return fabs(a.x-b.x)<vec_base<T>::eps&&fabs(a.y-b.y)<vec_base<T>::eps&&fabs(a.z-b.z)<vec_base<T>::eps; }

typedef vec_base<float> vecf;
typedef vec_base<double> vec;