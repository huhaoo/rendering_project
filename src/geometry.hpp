#pragma once

#include<cuda_runtime.h>
#include<math.h>
#include<stdlib.h>
#include<assert.h>

const double eps=1e-6;
template<class T> class vec_base
{
public:
	T x,y,z;
	__host__ __device__ vec_base():x(0),y(0),z(0){}
	__host__ __device__ vec_base(T X,T Y,T Z):x(X),y(Y),z(Z){}
	__host__ __device__ vec_base(const vec_base<T>& v):x(v.x),y(v.y),z(v.z){}

	__host__ __device__ T len(){ return sqrt(x*x+y*y+z*z); }
	__host__ __device__ vec_base<T> normalize(){ T l=len(); return vec_base<T>(x/l,y/l,z/l); }
	__host__ __device__ vec_base<T> cross(const vec_base<T>& v){ return vec_base<T>(y*v.z-z*v.y,z*v.x-x*v.z,x*v.y-y*v.x); }
	__host__ __device__ T dot(const vec_base<T>& v){ return x*v.x+y*v.y+z*v.z; }
	__host__ __device__ vec_base<T> operator+(const vec_base<T>& v){ return vec_base<T>(x+v.x,y+v.y,z+v.z); }
	__host__ __device__ vec_base<T> operator-(const vec_base<T>& v){ return vec_base<T>(x-v.x,y-v.y,z-v.z); }
	__host__ __device__ vec_base<T> operator*(const vec_base<T>& v){ return cross(v); }
	__host__ __device__ vec_base<T> operator*(T t){ return vec_base<T>(x*t,y*t,z*t); }
	__host__ __device__ vec_base<T> operator/(T t){ return vec_base<T>(x/t,y/t,z/t); }
	__host__ __device__ vec_base<T> operator-(){ return vec_base<T>(-x,-y,-z); }
	__host__ __device__ bool operator==(const vec_base<T>& v){ return fabs(x-v.x)<eps && fabs(y-v.y)<eps && fabs(z-v.z)<eps; }
	__host__ __device__ bool operator!=(const vec_base<T>& v){ return !(*this==v); }
	// pointwise multiplication
	__host__ __device__ vec_base<T> pwmul(const vec_base<T>& v){ return vec_base<T>(x*v.x,y*v.y,z*v.z); }
	// pointwise division
	__host__ __device__ vec_base<T> pwdiv(const vec_base<T>& v){ return vec_base<T>(x/v.x,y/v.y,z/v.z); }
};

template<class T> class triangle_base
{
public:
	vec_base<T> a,b,c;
	__host__ __device__ triangle_base():a(),b(),c(){}
	__host__ __device__ triangle_base(const vec_base<T>& A,const vec_base<T>& B,const vec_base<T>& C):a(A),b(B),c(C){}
	__host__ __device__ triangle_base(const triangle_base<T>& t):a(t.a),b(t.b),c(t.c){}
	// normal vector
	__host__ __device__ vec_base<T> n(){ return (b-a).cross(c-a).normalize(); }
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
 * return -1                                                           if no intersection
 * return the distance from the ray origin to the intersection point   otherwise
 */
__host__ __device__ template<class T> T intersect_base(const ray_base<T>& r,const triangle_base<T>& t)
{
	typedef vec_base<T> vec;
	vec n=t.n(); T rn=r.d.dot(n);
	if(fabs(rn)<eps) return -1;
	T d=(t.a-r.o).dot(n)/rn;
	if(d<eps) return -1;
	vec p=r.o+r.d*d;
	T s1=(t.a-p).cross(t.b-p).len();
	T s2=(t.b-p).cross(t.c-p).len();
	T s3=(t.c-p).cross(t.a-p).len();
	T s=(t.a-t.b).cross(t.b-t.c).len();
	if(fabs(s1+s2+s3-s)<eps) return d;
	else return -1;
}

/*
 * return -1                                                           if no intersection
 * return the distance from the ray origin to the intersection point   otherwise
 */
__host__ __device__ template<class T> T intersect_base(const ray_base<T>& r,const sphere_base<T>& s)
{
	typedef vec_base<T> vec;
	vec O=s.o-r.o;
	T x=r.d.dot(O);
	T y=sqrt(O.dot(O)-x*x);
	if(y>s.r-eps) return -1;
	T X=sqrt(s.r*s.r-y*y);
	if(x<-X+eps) return -1;
	if(x<X) return x+X;
	return x-X;
}

typedef vec_base<float> vecf;
typedef vec_base<double> vec;
typedef vec_base<long double> vecl;
typedef triangle_base<float> trianglef;
typedef triangle_base<double> triangle;
typedef triangle_base<long double> trianglel;
typedef ray_base<float> rayf;
typedef ray_base<double> ray;
typedef ray_base<long double> rayl;
typedef sphere_base<float> spheref;
typedef sphere_base<double> sphere;
typedef sphere_base<long double> spherel;
__host__ __device__ float intersect(const rayf& r,const trianglef& t){ return intersect_base(r,t); }
__host__ __device__ double intersect(const ray& r,const triangle& t){ return intersect_base(r,t); }
__host__ __device__ long double intersect(const rayl& r,const trianglel& t){ return intersect_base(r,t); }
__host__ __device__ float intersect(const rayf& r,const spheref& s){ return intersect_base(r,s); }
__host__ __device__ double intersect(const ray& r,const sphere& s){ return intersect_base(r,s); }
__host__ __device__ long double intersect(const rayl& r,const spherel& s){ return intersect_base(r,s); }