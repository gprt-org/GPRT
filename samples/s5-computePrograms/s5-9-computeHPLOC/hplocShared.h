#include "gprt.h"

// A 32 byte BVH2 node structure
struct BVH2Node {
    // [32b xlo] [32b ylo] [32b zlo] [1b - isInner] [7b - L Geom ID] [24b - L Prim ID]
    float4 aabbMinAndL;
    // [32b xhi] [32b yhi] [32b zhi] [1b - isInner] [7b - R Geom ID] [24b - R Prim ID]
    float4 aabbMaxAndR;

    #if defined(__SLANG_COMPILER__)
    __init() {
        aabbMinAndL = float4(+FLT_MAX);
        aabbMaxAndR = float4(-FLT_MAX);
    }

    __init(uint2 leftCluster, uint2 rightCluster, float2x3 new_bounds) {
        aabbMinAndL.xyz = new_bounds[0];
        aabbMaxAndR.xyz = new_bounds[1];
        uint32_t leftIndex  = (leftCluster.x  & 0x00FFFFFFu) | ((leftCluster.y  & 0x000000FFu) << 24u);
        uint32_t rightIndex = (rightCluster.x & 0x00FFFFFFu) | ((rightCluster.y & 0x000000FFu) << 24u);
        aabbMinAndL.w = asfloat(leftIndex);
        aabbMaxAndR.w = asfloat(rightIndex);
    }
    #endif

    bool isChildLeaf(int childIndex) {
        #if defined(__SLANG_COMPILER__)
        uint32_t index = (childIndex == 0) ? asuint(aabbMinAndL.w) : asuint(aabbMaxAndR.w);
        #else
        uint32_t index = (childIndex == 0) ? *(uint32_t *)&aabbMinAndL.w : *(uint32_t*)&aabbMaxAndR.w;
        #endif
        uint32_t top8Bits = (index >> 24);
        if (top8Bits == 255) return false;
        return true;
    }

    uint32_t getChildPrimID(int childIndex) {
        #if defined(__SLANG_COMPILER__)
        uint32_t index = (childIndex == 0) ? asuint(aabbMinAndL.w) : asuint(aabbMaxAndR.w);
        #else
        uint32_t index = (childIndex == 0) ? *(uint32_t*)&aabbMinAndL.w : *(uint32_t*)&aabbMaxAndR.w;
        #endif
        return index & 0x00FFFFFF;
    }

    uint32_t getChildGeomID(int childIndex) {
        #if defined(__SLANG_COMPILER__)
        uint32_t index = (childIndex == 0) ? asuint(aabbMinAndL.w) : asuint(aabbMaxAndR.w);
        #else
        uint32_t index = (childIndex == 0) ? *(uint32_t*)&aabbMinAndL.w : *(uint32_t*)&aabbMaxAndR.w;
        #endif
        return (index >> 24) & 0x000000FF;
    }

    uint32_t getChildCluster(int childIndex) {
        #if defined(__SLANG_COMPILER__)
        uint32_t index = (childIndex == 0) ? asuint(aabbMinAndL.w) : asuint(aabbMaxAndR.w);
        #else
        uint32_t index = (childIndex == 0) ? *(uint32_t*)&aabbMinAndL.w : *(uint32_t*)&aabbMaxAndR.w;
        #endif
        return index;
    }
};

struct HPLOCParams {
    int N;              // Number of primitives across all referenced geometries
    int M;              // Number of geometries
    BVH2Node* BVH2;  // BVH2 nodes, excluding leaves ((N - 1) x 32 bytes)
    // gprt::Buffer BVH8;  // BVH8 nodes (ceil((N x 2 - 1) / 8) x 80 bytes)
    // gprt::Buffer BVH8P; // BVH8 parents
    // gprt::Buffer BVH8L; // BVH8 leaves (N x 64 bytes)
    // gprt::Buffer BVH8LP; // BVH8 leaves (N x 64 bytes)
    uint2* I;     // Cluster indices reordered by space filling curve codes (N x 4 bytes)
    // gprt::Buffer C;     // Space filling curve codes, sorted in ascending order (N x 8 bytes)
    uint64_t* C; // Space filling curve codes, sorted in ascending order (N x 8 bytes)
    DescriptorHandle<RWStructuredBuffer<uint32_t>> AC; // Some atomic counters for allocation and work scheduling (5 x 4 bytes)

    // BVH2 parent IDs. initialized to -1. (N x 4 bytes)
    DescriptorHandle<RWStructuredBuffer<uint32_t>> pID; 

    // gprt::Buffer indexPairs; // BVH2 -> BVH8 pairs (N x 8bytes)
    uint64_t* indexPairs; // Might need to be a descriptor handle...
    
    // For geometry
    // gprt::Buffer rootBounds; 
    DescriptorHandle<RWStructuredBuffer<uint32_t>> rootBounds; // Two float3's storing aabb of trimesh (using uint due to atomic limitations...)
    
    // Each is count "M".
    uint32_t* primPrefix; // An exclusive prefix sum of the prim counts in each geometry
    uint3** triangles; // Triangle index buffers, one handle per geometry
    float3** vertices;  // Triangle vertex buffers, one handle per geometry
};