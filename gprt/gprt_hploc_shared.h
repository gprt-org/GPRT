// MIT License

// Copyright (c) 2023 Nathan V. Morrical

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#pragma once

#include "gprt.h"

// Assuming at most 128 geometries for now, but this can be increased if needed.
// (Might be able to use the full 255... just need to test it out...)
#ifndef GEOM_ID_BVH2 
#define GEOM_ID_BVH2 255
#endif

#ifndef INVALID_ID
#define INVALID_ID UINT32_MAX
#endif

struct HPLOCParams {
    int N;              // Number of primitives across all referenced geometries
    gprt::Buffer BVH2;  // BVH2 nodes, excluding leaves ((N - 1) x 32 bytes)
    gprt::Buffer BVH8;  // BVH8 nodes (ceil((N x 2 - 1) / 8) x 80 bytes)
    gprt::Buffer BVH8L; // BVH8 leaves (N x 64 bytes)
    gprt::Buffer AC;    // Some atomic counters for allocation and work scheduling (5 x 4 bytes)
    gprt::Buffer I;     // Cluster indices reordered by space filling curve codes (N x 4 bytes)
    gprt::Buffer C;     // Space filling curve codes, sorted in ascending order (N x 8 bytes)
    gprt::Buffer pID;   // BVH2 parent IDs. initialized to -1. (N x 4 bytes)

    gprt::Buffer indexPairs; // BVH2 -> BVH8 pairs (N x 8bytes)
    
    // For geometry
    gprt::Buffer rootBounds; // Two float3's storing aabb of trimesh
    gprt::Buffer primPrefix; // An exclusive prefix sum of the prim counts in each geometry
    gprt::Buffer triangles; // Triangle index buffers, one handle per geometry
    gprt::Buffer vertices;  // Triangle vertex buffers, one handle per geometry
};

// A 32 byte BVH2 node structure
struct BVH2Node {
    // [32b xlo] [32b ylo] [32b zlo] [1b - isInner] [7b - L Geom ID] [24b - L Prim ID]
    float4 aabbMinAndL;
    // [32b xhi] [32b yhi] [32b zhi] [1b - isInner] [7b - R Geom ID] [24b - R Prim ID]
    float4 aabbMaxAndR;

    #if defined(__SLANG_COMPILER__)
    __init(int2 leftCluster, int2 rightCluster, float2x3 new_bounds) {
        aabbMinAndL.xyz = new_bounds[0];
        aabbMaxAndR.xyz = new_bounds[1];
        uint32_t leftIndex  = (leftCluster.x  & 0x00FFFFFF) | ((leftCluster.y  & 0x000000FF) << 24);
        uint32_t rightIndex = (rightCluster.x & 0x00FFFFFF) | ((rightCluster.y & 0x000000FF) << 24);
        aabbMinAndL.w = asfloat(leftIndex);
        aabbMaxAndR.w = asfloat(rightIndex);
    }
    #endif

    float2x3 getBounds() {
        #if defined(__SLANG_COMPILER__)
        return float2x3(aabbMinAndL.xyz, aabbMaxAndR.xyz);
        #else
        float2x3 bounds;
        bounds[0] = aabbMinAndL.xyz();
        bounds[1] = aabbMaxAndR.xyz();
        return bounds;
        #endif
    }

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

#if defined(__SLANG_COMPILER__)
// Extracts the 8 exponent bits from the floating point number that conservatively bound it, avoiding infinity.
uint floatToExponent(float num) {
    uint bits = asuint(num);
    uint exponentBits = (bits >> 23) & 0xFF; // Mask to get only the exponent bits
    // Avoiding 255, since that's infinity.
    return uint(max(min(exponentBits + 1, 254u), 2u));
}

// Convert an exponent back to a floating-point scale factor
float exponentToFloat(uint exponent) {
    uint bits = exponent << 23;
    return asfloat(bits);
}

float3 exponentToFloat(uint3 exponent) {
    uint3 bits = exponent << 23;
    return asfloat(bits);
}

// Returns an 8-bit mask field to indicate which of the children are internal nodes.
uint getIMask(uint assignedChildren[8], uint numLeaves) {
    uint iMask = 0;
    for (int i = 0; i < 8; ++i) {
        if (assignedChildren[i] != INVALID_ID) {
            uint relativeIndex = assignedChildren[i];
            if (relativeIndex >= numLeaves) {
                iMask |= (1 << i);
            }
        }
    }
    return iMask;
}

// could probably optimize a bit if needed...
uint8_t getOctant(float3 dir) {
    uint8_t octant = uint8_t(((dir.x < 0 ? 1 : 0) << 2) | ((dir.y < 0 ? 1 : 0) << 1) | ((dir.z < 0 ? 1 : 0) << 0));
    return octant;
}

#endif

struct BVH8Triangle {
    float3 v[3];
    uint32_t clusterID;
};


struct BVH8Leaf { BVH8Triangle tri; };

// 80 bytes
// Following from here: https://users.aalto.fi/~laines9/publications/ylitie2017hpg_paper.pdf
struct BVH8Node
{
    // To interpret "meta" field:
    //   Empty child slot: field set to 00000000
    //   Internal node: high 3 bits 001 while low 5 bits store child slot index + 24. (values range in 24-31)
    //   Leaf node: high 3 bits store number of triangles using unary encoding, and low 5 bits store the
    //              index of first triangle relative to the triangle base index (ranging in 0...23)

    // [32b origin x] [32b origin y] [32b origin z] [8b extent x] [8b extent y] [8b extent z] [8b inner node mask]
    float3 origin; uint extx : 8; uint exty : 8; uint extz : 8; uint imask : 8;

    // [32b child node base index] [32b primitive base index]
    uint childNodeBaseIndex; uint primitiveBaseIndex;

    // [8b child 0] [8b child 1] [8b child 2] [8b child 3] [8b child 4] [8b child 5] [8b child 6] [8b child 7]
    uint64_t meta8;
    uint64_t qlox8;
    uint64_t qloy8;
    uint64_t qloz8;
    uint64_t qhix8;
    uint64_t qhiy8;
    uint64_t qhiz8;

    float3 getScale() {
        #if defined(__SLANG_COMPILER__)
        uint3 bits = uint3((extx + 127) << 23, (exty + 127) << 23, (extz + 127) << 23);
        return asfloat(bits);
        // return exponentToFloat(uint3(extx, exty, extz));
        #else
        uint3 bits = uint3((extx + 127) << 23, (exty + 127) << 23, (extz + 127) << 23);
        return *((float3 *)&bits);
        #endif
    }

    #if defined(__SLANG_COMPILER__)
    property float3 scale {
        get {
            return exponentToFloat(uint3(extx, exty, extz));
        }
        set {
            extx = floatToExponent(newValue.x);
            exty = floatToExponent(newValue.y);
            extz = floatToExponent(newValue.z);
        }
    }

    property float2x3 bounds {
        get {
            float3 aabbMin = origin;
            float3 aabbMax = aabbMin + scale;
            return float2x3(aabbMin, aabbMax);
        }
    }

    uint8_t3[2] getChild(uint slotIndex) {
        uint64_t shift = slotIndex << 3;

        // First get low bits
        uint8_t3[2] qaabb;
        qaabb[0].x = uint8_t(qlox8 >> shift);
        qaabb[0].y = uint8_t(qloy8 >> shift);
        qaabb[0].z = uint8_t(qloz8 >> shift);
        // Then get high bits
        qaabb[1].x = uint8_t(qhix8 >> shift);
        qaabb[1].y = uint8_t(qhiy8 >> shift);
        qaabb[1].z = uint8_t(qhiz8 >> shift);
        return qaabb;
    }

    [mutating]
    void setMeta(int slotIndex, uint8_t meta) {
        uint64_t shift = slotIndex << 3;
        meta8 = (meta8 & ~(0xFFull << shift)) | (uint64_t(meta) << shift);
    }

    [mutating]
    void setIMask(int slotIndex, bool value) {
        imask = (imask & ~(1 << slotIndex)) | (uint8_t(value) << slotIndex);
    };

    #endif

    bool hasInnerNode(bool debug = false) {
        uint64_t maskLower5 = 0x1F1F1F1F1F1F1F1Full; // Mask to isolate lower 5 bits from each byte
        uint64_t subtract24 = 0x1818181818181818ull; // Subtract 24 from each byte
        uint64_t upperLimit = 0x0707070707070707ull; // Upper limit for check (31 - 24 = 7)

        // Apply mask, subtract 24, check if any are less than or equal to 7
        uint64_t result = (meta8 & maskLower5) - subtract24;
        return ((result - upperLimit) & ~result & 0x8080808080808080) != 0;
    }

    bool getIMask(int slotIndex) {
        return (imask & (1 << slotIndex)) != 0;
    }

    uint8_t getMeta(uint slotIndex) {
        uint64_t shift = slotIndex << 3;
        return uint8_t(meta8 >> shift);
    }

    // Mainly for testing
    float2x3 getChildBounds(uint slotIndex) {
        uint64_t shift = slotIndex << 3;

        uint3 qaabb[2];

        // First get low bits
        qaabb[0].x = uint8_t(qlox8 >> shift);
        qaabb[0].y = uint8_t(qloy8 >> shift);
        qaabb[0].z = uint8_t(qloz8 >> shift);
        
        // Then get high bits
        qaabb[1].x = uint8_t(qhix8 >> shift);
        qaabb[1].y = uint8_t(qhiy8 >> shift);
        qaabb[1].z = uint8_t(qhiz8 >> shift);

        float3 scale = getScale();
        float2x3 bounds;
        bounds[0] = origin + (float3(qaabb[0]) / 255.f) * scale;
        bounds[1] = origin + (float3(qaabb[1]) / 255.f) * scale;
        return bounds;
    }


    // // Returns the number of non-empty child slots
    // uint getSize() {
    //     uint size = 0;
    //     for (int i = 0; i < 8; ++i) {
    //         uint child = uint(meta >> (i * 8ull)) & 0xFF;
    //         if (child != 0) size++;
    //     }
    //     return size;
    // }
};

struct TriangleRecord {
    gprt::Buffer BVH8L;
    gprt::Buffer triangles; // Triangle index buffers, one handle per geometry
    gprt::Buffer vertices;  // Triangle vertex buffers, one handle per geometry
};

struct TraverseRecord {
    uint N; // number of primitives
    gprt::Buffer BVH2; // BVH2 nodes (N-1 x 64 bytes)
    gprt::Buffer BVH8N; // BVH8 nodes (ceil((N x 2 - 1) / 8) x 80 bytes)
    gprt::Buffer BVH8L; // BVH8 leaves

    // One buffer per geometry
    gprt::Buffer primBuffers;
    gprt::Buffer vertBuffers;

    // An offset for the given instance to the SBT of callables
    // for nearest neighbor programs
    uint32_t InstanceContributionToHitGroupIndex;
};