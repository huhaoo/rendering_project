#include"geometry.hpp"
#include"config.h"
#include"bvh.hpp"
#include"intersect.hpp"

#include<vector>

#define call(f,n) f<<<(n+block_size-1)/block_size,block_size>>>
__global__ void init(int* id, F* d, int n)
{
    int i=blockIdx.x*blockDim.x+threadIdx.x;
    if(i>=n) return;
    id[i]=-1; d[i]=inf;
}
__global__ void intesect_bvh_core(ray *r, triangle *t, bvh_node *b,int n,int *id,F *d)
{
    int i=blockIdx.x*blockDim.x+threadIdx.x;
    if(i>=n) return;
    intersect_bvh(r[i],b,t,id[i],d[i]);
}

std::vector<std::pair<int,F> > intersect_ray_triangle(std::vector<ray> r, std::vector<triangle> t, std::vector<bvh_node> b)
{
    int n=r.size(); int m=t.size(); int k=b.size();
#ifdef use_cuda
    ray* R; triangle* T; bvh_node* B;
    int *id;
    F *d;
    cudaMalloc(&R,  n*sizeof(ray));
    cudaMalloc(&T,  m*sizeof(triangle));
    cudaMalloc(&B,  k*sizeof(bvh_node));
    cudaMalloc(&id, n*sizeof(int));
    cudaMalloc(&d,  n*sizeof(F));
    cudaMemcpy(T, t.data(), m*sizeof(triangle), cudaMemcpyHostToDevice);
    cudaMemcpy(R, r.data(), n*sizeof(vec),      cudaMemcpyHostToDevice);
    cudaMemcpy(B, b.data(), k*sizeof(bvh_node), cudaMemcpyHostToDevice);
    call(init,n)(id,d,n);
    call(intesect_bvh_core,n)(R,T,B,n,id,d);
    std::vector<std::pair<int,F> > result;
    int *Id=new int[n]; F *D=new F[n];
    cudaMemcpy(Id, id, n*sizeof(int), cudaMemcpyDeviceToHost);
    cudaMemcpy(D,  d,  n*sizeof(F),   cudaMemcpyDeviceToHost);
    for(int i=0;i<n;i++) result.push_back(std::make_pair(D[i],Id[i]));
    cudaFree(R); cudaFree(T); cudaFree(B); cudaFree(id); cudaFree(d); delete[] Id; delete[] D;
    return result;
#else
    std::vector<std::pair<int,F> > result;
    for(int i=0;i<n;i++)
    {
        int id; F d=inf;
        intersect_bvh(r[i],b.data(),t.data(),id,d);
        result.push_back(std::make_pair(id,d));
    }
    return result;
#endif
}
