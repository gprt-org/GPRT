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

#define PRIMITIVE_POSTPONING 0
#define DYNAMIC_FETCH 0

#include "gprt_nnq.h"

using namespace gprt;

inline float dot2(float3 v ) { return dot(v,v); }

inline float max3(float3 v) {
    return max(max(v.x, v.y), v.z);
}

inline float min3(float3 v) {
    return min(min(v.x, v.y), v.z);
}

inline float sum3(float3 v) {
    return v.x + v.y + v.z;
}

// note, returns squared distance
float udTriangle(in float3 v1, in float3 v2, in float3 v3, in float3 p)
{
    float3 v21 = v2 - v1; float3 p1 = p - v1;
    float3 v32 = v3 - v2; float3 p2 = p - v2;
    float3 v13 = v1 - v3; float3 p3 = p - v3;
    float3 nor = cross(v21, v13);

    return ((sign(dot(cross(v21, nor), p1)) +
                     sign(dot(cross(v32, nor), p2)) +
                     sign(dot(cross(v13, nor), p3)) < 2.0)
                    ?
                  min(min(
                              dot2(v21 * clamp(dot(v21, p1) / dot2(v21), 0.0, 1.0) - p1),
                              dot2(v32 * clamp(dot(v32, p2) / dot2(v32), 0.0, 1.0) - p2)),
                          dot2(v13 * clamp(dot(v13, p3) / dot2(v13), 0.0, 1.0) - p3))
                    :
                  dot(nor, p1) * dot(nor, p1) / dot2(nor));
}

/**
 * Computes the square of the distance from a point to an AABB.
 * If the point is contained in the rectangle then the distance is zero.
 */
float computeSquaredMinDistanceToAABB(float3 origin, float3 aabbMin, float3 aabbMax) {
    // For each dimension, determine which side that "p" is on.
    // Then, compute distance to that side. If the point is inside the box,
    // then the distance is zero.
    float3 d = select((origin < aabbMin), origin - aabbMin,
                      select((origin > aabbMax), origin - aabbMax,
                             float3(0.f, 0.f, 0.f))
    );
    // Sums the squares of the distances.
    return dot(d, d);
}

/**
 * Computes the squared minimum distance between a point and a triangle.
 * This function is not sensitive to the handedness of the coordinate system.
 * @param[in] vertices Positions of the three vertices.
 * @param[in] p Coordinates of the point.
 * @return Squared minimum distance between p and the triangle.
 */
float computeSquaredMinDistanceToTriangle(const float3 vertices[3], const float3 p)
{
    // Project p onto the plane of the triangle (the result is independent of triangle winding).
    const float3 n = normalize(cross(vertices[1] - vertices[0], vertices[2] - vertices[0]));
    const float projDistance = dot(n, (p - vertices[0]));
    const float3 pProj = p - projDistance * n;

    // Edge tests to compute signed distance to each edge.
    // Positive result means the projected point is "inside" the edge.
    // With flipped winding, the edges are flipped but n is also flipped so it still works.
    const float3 edges[3] = {
        normalize(vertices[1] - vertices[0]),
        normalize(vertices[2] - vertices[1]),
        normalize(vertices[0] - vertices[2]),
    };
    float sqrPlanarDistance = FLT_MAX;
    uint insideMask = 0u;
    [unroll]
    for (uint i = 0u; i < 3u; ++i)
    {
        const float3 edgeN = cross(n, edges[i]);
        const float edgeProjDistance = dot(edgeN, pProj - vertices[i]);
        if (edgeProjDistance >= 0.0f)
        {
            insideMask |= 1u << i;
        }
        else
        {
            const float3 vec = pProj - vertices[i];
            sqrPlanarDistance = min(edgeProjDistance * edgeProjDistance, sqrPlanarDistance);
        }
    }
    if (insideMask == 0x7)
    {
        sqrPlanarDistance = 0.0f;
    }

    // If only one edge is considering the point as inside, then the projected point
    // is closest to a triangle corner (the vertex opposite of that edge).
    else if (insideMask == 1u << 0u)
    {
        sqrPlanarDistance = dot(pProj - vertices[2], pProj - vertices[2]);
    }
    else if (insideMask == 1u << 1u)
    {
        sqrPlanarDistance = dot(pProj - vertices[0], pProj - vertices[0]);
    }
    else if (insideMask == 1u << 2u)
    {
        sqrPlanarDistance = dot(pProj - vertices[1], pProj - vertices[1]);
    }

    return projDistance * projDistance + sqrPlanarDistance;
}

// from real time collision detection
float findClosestPointTriangle(float3 pa, float3 pb, float3 pc, float3 x, out float3 p, out float2 t)
{
    // source: real time collision detection
    // check if x in vertex region outside pa
    float3 ab = pb - pa;
    float3 ac = pc - pa;
    float3 ax = x - pa;
    float d1 = dot(ab, ax);
    float d2 = dot(ac, ax);
    if (d1 <= 0.0 && d2 <= 0.0)
    {
        // barycentric coordinates (1, 0, 0)
        t = float2(1.0, 0.0);
        p = pa;
        return length(x - p);
    }

    // check if x in vertex region outside pb
    float3 bx = x - pb;
    float d3 = dot(ab, bx);
    float d4 = dot(ac, bx);
    if (d3 >= 0.0 && d4 <= d3)
    {
        // barycentric coordinates (0, 1, 0)
        t = float2(0.0, 1.0);
        p = pb;
        return length(x - p);
    }

    // check if x in vertex region outside pc
    float3 cx = x - pc;
    float d5 = dot(ab, cx);
    float d6 = dot(ac, cx);
    if (d6 >= 0.0 && d5 <= d6)
    {
        // barycentric coordinates (0, 0, 1)
        t = float2(0.0, 0.0);
        p = pc;
        return length(x - p);
    }

    // check if x in edge region of ab, if so return projection of x onto ab
    float vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0)
    {
        // barycentric coordinates (1 - v, v, 0)
        float v = d1 / (d1 - d3);
        t = float2(1.0 - v, v);
        p = pa + ab * v;
        return length(x - p);
    }

    // check if x in edge region of ac, if so return projection of x onto ac
    float vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0)
    {
        // barycentric coordinates (1 - w, 0, w)
        float w = d2 / (d2 - d6);
        t = float2(1.0 - w, 0.0);
        p = pa + ac * w;
        return length(x - p);
    }

    // check if x in edge region of bc, if so return projection of x onto bc
    float va = d3 * d6 - d5 * d4;
    if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0)
    {
        // barycentric coordinates (0, 1 - w, w)
        float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        t = float2(0.0, 1.0 - w);
        p = pb + (pc - pb) * w;
        return length(x - p);
    }

    // x inside face region. Compute p through its barycentric coordinates (u, v, w)
    float denom = 1.0 / (va + vb + vc);
    float v = vb * denom;
    float w = vc * denom;
    t = float2(1.0 - v - w, v);
    p = pa + ab * v + ac * w; //= u*a + v*b + w*c, u = va*denom = 1.0f - v - w
    return length(x - p);
}

// from real time collision detection
float findClosestPointTriangleSquared(float3 pa, float3 pb, float3 pc, float3 x, out float3 p, out float2 t)
{
    // source: real time collision detection
    // check if x in vertex region outside pa
    float3 ab = pb - pa;
    float3 ac = pc - pa;
    float3 ax = x - pa;
    float d1 = dot(ab, ax);
    float d2 = dot(ac, ax);
    if (d1 <= 0.0 && d2 <= 0.0)
    {
        // barycentric coordinates (1, 0, 0)
        t = float2(1.0, 0.0);
        p = pa;
        return dot2(x - p);
    }

    // check if x in vertex region outside pb
    float3 bx = x - pb;
    float d3 = dot(ab, bx);
    float d4 = dot(ac, bx);
    if (d3 >= 0.0 && d4 <= d3)
    {
        // barycentric coordinates (0, 1, 0)
        t = float2(0.0, 1.0);
        p = pb;
        return dot2(x - p);
    }

    // check if x in vertex region outside pc
    float3 cx = x - pc;
    float d5 = dot(ab, cx);
    float d6 = dot(ac, cx);
    if (d6 >= 0.0 && d5 <= d6)
    {
        // barycentric coordinates (0, 0, 1)
        t = float2(0.0, 0.0);
        p = pc;
        return dot2(x - p);
    }

    // check if x in edge region of ab, if so return projection of x onto ab
    float vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0)
    {
        // barycentric coordinates (1 - v, v, 0)
        float v = d1 / (d1 - d3);
        t = float2(1.0 - v, v);
        p = pa + ab * v;
        return dot2(x - p);
    }

    // check if x in edge region of ac, if so return projection of x onto ac
    float vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0)
    {
        // barycentric coordinates (1 - w, 0, w)
        float w = d2 / (d2 - d6);
        t = float2(1.0 - w, 0.0);
        p = pa + ac * w;
        return dot2(x - p);
    }

    // check if x in edge region of bc, if so return projection of x onto bc
    float va = d3 * d6 - d5 * d4;
    if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0)
    {
        // barycentric coordinates (0, 1 - w, w)
        float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        t = float2(0.0, 1.0 - w);
        p = pb + (pc - pb) * w;
        return dot2(x - p);
    }

    // x inside face region. Compute p through its barycentric coordinates (u, v, w)
    float denom = 1.0 / (va + vb + vc);
    float v = vb * denom;
    float w = vc * denom;
    t = float2(1.0 - v - w, v);
    p = pa + ab * v + ac * w; //= u*a + v*b + w*c, u = va*denom = 1.0f - v - w
    return dot2(x - p);
}

// Calcs signed distance to a box of semi-size "rad" centered at
// the origin. It works in 2D, 3D, 4D any number of dimensions by
// replacing the vec data types.
float boxDistance(in float3 p, in float3 aabbmin, float3 aabbmax)
{
    float3 cen = .5 * (aabbmin + aabbmax);
    float3 rad = .5 * (aabbmax - aabbmin);
    p = p - cen;

    float3 d = abs(p) - rad;
    return dot2(max(d, 0.0));// + min(max(d.x, max(d.y, d.z)), 0.0);
}

float sqAABB( in float3 p, in float3 minCorner, in float3 maxCorner ) 
{
    float3 size = (maxCorner - minCorner) * 0.5;
    float3 center = minCorner + size;
    float3 d = max(abs(p - center) - size, 0.0);
    return dot(d, d);
}

inline float sdBox(float3 origin, float3 aabbMin, float3 aabbMax) {
    // For each dimension...
    // Determine which side that "p" is on.
    // Then, add the L2 distance to that side to our total
    float3 d = select((origin < aabbMin), origin - aabbMin,
                      select((origin > aabbMax), origin - aabbMax,
                             float3(0.f, 0.f, 0.f)));
    return dot2(d);
}

inline float sdBox(float3 origin, float2x3 bounds) {
    // For each dimension...
    // Determine which side that "p" is on.
    // Then, add the L2 distance to that side to our total
    float3 d = select((origin < bounds[0]), origin - bounds[0],
                      select((origin > bounds[1]), origin - bounds[1],
                             float3(0.f, 0.f, 0.f)));
    return sqrt(dot(d, d));
}

bool rayBox(float3 o, float3 d, float2 trange, float3 lo, float3 hi, out float tc, out float tf) {
    float3 tlo = (lo - o) / d;
    float3 thi = (hi - o) / d;
    float3 tc3 = min(tlo, thi);
    float3 tf3 = max(tlo, thi);
    tc = max3(tc3);
    tf = min3(tf3);
    return (max(tc, trange.x) < min(tf, trange.y));
}

// // Meant to mimic a ray-slab test.
// bool maxDistTest(float3 o, float3 d, float2 trange, float3 lo, float3 hi, out float tc) {
//     float3 tlo = (lo - o) / d;
//     float3 thi = (hi - o) / d;
//     float3 tc3 = min(tlo, thi);
//     float3 tf3 = max(tlo, thi);
//     tc = min3(tc3);
//     return trange.x < tc && tc < trange.y;
// }

// tf = min3(tf3);
// bool maxDistTest(float3 o, float3 d, float2 trange, float3 lo, float3 hi, out float tc) {
//     float3 tlo = (lo - o) / d;
//     float3 thi = (hi - o) / d;
//     float3 tc3 = min(tlo, thi);
//     float3 tf3 = max(tlo, thi);
//     float tc = max3(tf3);
//     return trange.x < tc && tc < trange.y;
// }

inline bool minDistTest(float3 o, float3 d, float2 trange, float3 lo, float3 hi, out float tc) {
    float3 tlo = (lo - o) / d;
    float3 thi = (hi - o) / d;
    float3 tc3 = min(tlo, thi);
    float3 tf3 = max(tlo, thi);
    float3 d3 = select(0.0 < tc3, tc3,
                    select(0.0 > tf3, tf3,
                        float3(0.f, 0.f, 0.f)));
    tc = d3.x * d3.x + d3.y * d3.y + d3.z * d3.z;
    return trange.x <= tc && tc <= trange.y;
}

//------------------------------------------------------------------------
struct TraversalState {
    uint sp; // stack pointer
    
    StackEntry nodeGroup;
    StackEntry primGroup;

    RWByteAddressBuffer BVH8N;
    RWByteAddressBuffer BVH8L;

    RWByteAddressBuffer BVH8NP;
    RWByteAddressBuffer BVH8LP;

    // note, repurposing tmax for search radius, and ignoring tmin.
    float3 origin; float tmin;
    float3 direction; float tmax;

    float2 triUV;

    int enablePostponing;

    // debug data...
    uint debug;
    uint debugHits;
    float debugTime;

    __init() {
    }
};

//------------------------------------------------------------------------
BVH2Node fetchBVH2Node(RWByteAddressBuffer BVH2N, int node_index) {
    //return BVH2N.Load<BVH2Node>(node_index * sizeof(BVH2Node));

    float4 n[2];
    n[0] = BVH2N.LoadAligned<float4>(sizeof(float4) * (node_index * 2 + 0));
    n[1] = BVH2N.LoadAligned<float4>(sizeof(float4) * (node_index * 2 + 1));
    BVH2Node node = reinterpret<BVH2Node>(n);
    return node;
}

BVH8Node fetchBVH8Node(RWByteAddressBuffer BVH8N, int node_index) {
    float4 n[8];
    n[0] = BVH8N.LoadAligned<float4>(sizeof(float4) * (node_index * 8 + 0));
    n[1] = BVH8N.LoadAligned<float4>(sizeof(float4) * (node_index * 8 + 1));
    n[2] = BVH8N.LoadAligned<float4>(sizeof(float4) * (node_index * 8 + 2));
    n[3] = BVH8N.LoadAligned<float4>(sizeof(float4) * (node_index * 8 + 3));
    n[4] = BVH8N.LoadAligned<float4>(sizeof(float4) * (node_index * 8 + 4));
    n[5] = BVH8N.LoadAligned<float4>(sizeof(float4) * (node_index * 8 + 5));
    n[6] = BVH8N.LoadAligned<float4>(sizeof(float4) * (node_index * 8 + 6));
    n[7] = BVH8N.LoadAligned<float4>(sizeof(float4) * (node_index * 8 + 7));
    BVH8Node node = reinterpret<BVH8Node>(n);
    return node;
};

BVH8Triangle fetchTriangle(RWByteAddressBuffer BVH8L, int tri_index) {
    float4 n[3];
    n[0] = BVH8L.LoadAligned<float4>(sizeof(float4) * (tri_index * 3 + 0));
    n[1] = BVH8L.LoadAligned<float4>(sizeof(float4) * (tri_index * 3 + 1));
    n[2] = BVH8L.LoadAligned<float4>(sizeof(float4) * (tri_index * 3 + 2));
    BVH8Triangle tri = reinterpret<BVH8Triangle>(n);
    return tri;
}

//------------------------------------------------------------------------
struct StackEntry {
    // Indicates whether the group is a primitive group or a node group.
    uint32_t isPrimGroup : 1;

    // A base index common to all items in the queue.
    uint32_t baseIdx : 31;
    
    // A queue of the relative indices offset from the base index.
    uint32_t relIdxQueue : 24; // 3 bits per slot, 8 slots

    // A mask indicating which items of the queue still need processing
    uint32_t hits: 8; // 1 bit per slot, 8 slots
};

static const int LOCAL_STACK_SIZE = 16;

inline void pushNodeGroup(inout StackEntry localStack[LOCAL_STACK_SIZE], inout TraversalState tstate)
{
    localStack[tstate.sp] = tstate.nodeGroup;
    tstate.sp = tstate.sp + 1;
}

inline void pushPrimGroup(inout StackEntry localStack[LOCAL_STACK_SIZE], inout TraversalState tstate)
{
    localStack[tstate.sp] = tstate.primGroup;
    tstate.sp = tstate.sp + 1;
}

inline StackEntry stackPop(inout StackEntry localStack[LOCAL_STACK_SIZE], inout TraversalState tstate)
{
    tstate.sp--;
    StackEntry entry = localStack[tstate.sp];
    return entry;
}

// inline
float4 OpConvertUToF(uint32_t4 u) {
    return spirv_asm {
        result : $$float4 = OpConvertUToF $u;
    };
}

float3 OpConvertUToF(uint32_t3 u) {
    return spirv_asm {
        result : $$float3 = OpConvertUToF $u;
    };
}

#define ENABLE_BITFIELDS

uint bfe(uint value, int offset, int bits)
{
    __target_switch
    {
    // case glsl: __intrinsic_asm "bitfieldExtract";
    // case spirv: return spirv_asm {
    //     result:$$uint = OpBitFieldUExtract $value $offset $bits
    // };
    default:
        return (value >> offset) & ((1u << bits) - 1);
    }
}

uint3 bfe(uint3 value, int offset, int bits)
{
    __target_switch
    {
    // case glsl: __intrinsic_asm "bitfieldExtract";
    // case spirv: return spirv_asm {
    //     result:$$uint3 = OpBitFieldUExtract $value $offset $bits
    // };
    default:
        uint3 result;
        [ForceUnroll]
        for (int i = 0; i < 3; ++i)
        {
            result[i] = bfe(value[i], offset, bits);
        }
        return result;
    }
}

uint4 bfe(uint4 value, int offset, int bits)
{
    __target_switch
    {
    // case glsl: __intrinsic_asm "bitfieldExtract";
    // case spirv: return spirv_asm {
    //     result:$$uint4 = OpBitFieldUExtract $value $offset $bits
    // };
    default:
        uint4 result;
        [ForceUnroll]
        for (int i = 0; i < 4; ++i)
        {
            result[i] = bfe(value[i], offset, bits);
        }
        return result;
    }
}

uint bfi(uint base, uint insert, int offset, int bits)
{
    __target_switch
    {
    // case glsl: __intrinsic_asm "bitfieldInsert";
    // case spirv: return spirv_asm {
    //     result:$$uint = OpBitFieldInsert $base $insert $offset $bits
    // };
    default:
        uint clearMask = ~(((1u << bits) - 1u) << offset);
        uint clearedBase = base & clearMask;
        uint maskedInsert = (insert & ((1u << bits) - 1u)) << offset;
        return clearedBase | maskedInsert;
    }
}

uint4 bfi(uint4 base, uint4 insert, int offset, int bits)
{
    __target_switch
    {
    // case glsl: __intrinsic_asm "bitfieldInsert";
    // case spirv: return spirv_asm {
    //     result:$$uint4 = OpBitFieldInsert $base $insert $offset $bits
    // };
    default:
        uint4 result;
        [ForceUnroll]
        for (int i = 0; i < 4; ++i)
        {
            result[i] = bfi(base[i], insert[i], offset, bits);
        }
        return result;
    }
}

float unpackUnorm1x8(uint p)
{
    const uint byteMask = 0xff;
    return float(p & byteMask) / 255.0;
}

float4 unpackUnorm4x8(uint p)
{
    __target_switch
    {
    case glsl: __intrinsic_asm "unpackUnorm4x8";
    case spirv: return spirv_asm {
        result:$$float4 = OpExtInst glsl450 UnpackUnorm4x8 $p
    };
    default:
        return float4(
            unpackUnorm1x8(p),
            unpackUnorm1x8(p >> 8),
            unpackUnorm1x8(p >> 16),
            unpackUnorm1x8(p >> 24));
    }
}

float intersectBox(float3 scale, float3 anear, float3 afar, float3 aabbMin, float3 aabbMax) {
    float3 tc3 = mad(scale, aabbMin, anear);
    float3 tf3 = mad(scale, aabbMax, afar);
    float3 d3 = clamp(0.0, tc3, tf3);
    return dot(d3, d3);
}

void computeStackEntries(float2x4 sortedDistsAndMeta, float currentTMax, uint nodeBaseIdx, uint primBaseIdx, out StackEntry nodeGroup, out StackEntry primGroup) {
    // Indicates which queue slots contain hits.
    uint32_t leafHits = 0;
    uint32_t nodeHits = 0;

    // The queue of relative indices, 3 bits each and 24b total.
    uint32_t queue = 0;

    [unroll]
    for (int i = 0; i < 2; ++i) {
        // A float containing the hit distance, with the item's 4-bit metadata in the low bits.
        // If the item is "empty", sortedDistsAndMeta is FLT_MAX.
        let meta4 = asuint(sortedDistsAndMeta[i]) & 0x0000000F;
        let hit4 = (sortedDistsAndMeta[i] < currentTMax);
        let leaf4 = bool4(meta4 & 0b1000);
        
        // Mark which slots in the queue belong to a leaf or an inner node.
        let leafHits4 = uint4(hit4 &&  leaf4) << (uint4(0, 1, 2, 3) + (i << 2));
        let nodeHits4 = uint4(hit4 && !leaf4) << (uint4(0, 1, 2, 3) + (i << 2));
        leafHits |= leafHits4.x | leafHits4.y | leafHits4.z | leafHits4.w;
        nodeHits |= nodeHits4.x | nodeHits4.y | nodeHits4.z | nodeHits4.w;
        
        // Store the relative index in the priority queue.
        // note, queueSlot*3 is at most 8*3 = 24 bits. 
        let relIdx4 = (meta4 & 0b0111) << (uint4(0, 3, 6, 9) + (i * 12));
        queue |= relIdx4.x | relIdx4.y | relIdx4.z | relIdx4.w;
    }

    // Store entries into traversal state
    nodeGroup.baseIdx = nodeBaseIdx;
    primGroup.baseIdx = primBaseIdx;
    nodeGroup.isPrimGroup = 0;
    primGroup.isPrimGroup = 1;
    nodeGroup.relIdxQueue = queue;
    primGroup.relIdxQueue = queue;
    nodeGroup.hits = nodeHits;
    primGroup.hits = leafHits;
}

//------------------------------------------------------------------------

void intersectNode(BVH8Node node, float3 origin, float currentTMax, out StackEntry nodeGroup, out StackEntry primGroup)
{    
    let pos = asfloat(node.posScaleSMask.xyz);
    let packedScale = node.posScaleSMask.w;
    let meta8 = node.idxAndMeta.zw;
    let byteShift = uint4(0, 8, 16, 24);
    let scaleAndMask = uint8_t4(uint4(packedScale) >> byteShift);
    let scale = asfloat(uint3(scaleAndMask.xyz) << 23); // Shift 8-bit scale to floating-point exponent bits.
    let smask = scaleAndMask.w;
    let an = pos - origin;
    let af = pos - origin;

    float2x4 dists;    
    [unroll]
    for (uint i = 0; i < 2; ++i) {
        let qlo = uint3(node.lox[i], node.loy[i], node.loz[i]);
        let qhi = uint3(node.hix[i], node.hiy[i], node.hiz[i]);

        [unroll]
        for (uint j = 0; j < 4; ++j) {
            let tc3 = mad(scale, bfe(qlo, (j << 3), 8), an);
            let tf3 = mad(scale, bfe(qhi, (j << 3), 8), af);
            let d3 = clamp(0.0, tc3, tf3);
            dists[i][j] = dot(d3, d3);
        }

        let off = i << 2;
        let used4 = uint4(bfe(smask, off + 0, 1), bfe(smask, off + 1, 1), bfe(smask, off + 2, 1), bfe(smask, off + 3, 1));
        let meta4 = uint4(bfe( meta8[i], 0, 4),   bfe( meta8[i], 8, 4),   bfe( meta8[i], 16, 4),  bfe( meta8[i], 24, 4));
        dists[i] = select(bool4(used4), asfloat((asuint(dists[i] - FLT_EPSILON) & 0xFFFFFFF0) | meta4), FLT_MAX);

        SortNetwork123(dists[i]);
    }
    SortNetwork4567(dists[0], dists[1]);
    computeStackEntries(dists, currentTMax, node.idxAndMeta.x, node.idxAndMeta.y, nodeGroup, primGroup);
}

//------------------------------------------------------------------------
inline void traverseBVH8(inout TraversalState tstate) {
    StackEntry localStack[LOCAL_STACK_SIZE];
    tstate.sp = 0;

    // Root
    tstate.nodeGroup.isPrimGroup = 0;
    tstate.nodeGroup.baseIdx = 0;
    tstate.nodeGroup.hits = 0b00000001;
    tstate.nodeGroup.relIdxQueue = 0;

    // bool enablePostponing = false;

    // Intersect root node
    {
        let node = fetchBVH8Node(tstate.BVH8N, 0);
        intersectNode(node, tstate.origin, tstate.tmax, tstate.nodeGroup, tstate.primGroup);
    }

    while (true)
    {
        // int ntrav = countbits( WaveActiveBallot( true ).x );
        // if (tstate.debug) printf("Ntrav %d\n", ntrav);
        
        // handle leaves
        if (tstate.primGroup.hits > 0)
        {
            do
            {
                // if(enablePostponing && countbits( WaveActiveBallot( true ).x ) * 2 < ntrav )  // Is SIMD utilization too low?
                //     break;                                                                    // Postpone primitive intersections

                tstate.debugHits++;

                let slot = firstbithigh( tstate.primGroup.hits );
                tstate.primGroup.hits = bfi(tstate.primGroup.hits, 0, slot, 1); // Clear it from list

                // Reverse traversal priority
                let relIdx = int(tstate.primGroup.relIdxQueue >> (slot * 3)) & 0b111;
                let absIdx = tstate.primGroup.baseIdx + relIdx;
                
                let tri = fetchTriangle(tstate.BVH8L, absIdx);
                float2 bc; float3 pclosest;
                let dist = findClosestPointTriangleSquared(tri.v0, tri.v1, tri.v2, tstate.origin, pclosest, bc);
                if (dist < tstate.tmax) {
                    tstate.tmax = dist;
                }
            } while (tstate.primGroup.hits > 0);

            // if ( tstate.primGroup.hits > 0 )
            //     pushPrimGroup( localStack, tstate );
        }

        // stack pop?
        if ( tstate.nodeGroup.hits == 0 )
        {
            if( tstate.sp == 0 ) break; // empty stack ==> done            
            tstate.nodeGroup = stackPop( localStack, tstate );
        }

        // if current group is actually a prim group...
        if (tstate.nodeGroup.isPrimGroup == 1)
        {
            tstate.primGroup = tstate.nodeGroup;
            
            // clear to indicate that work has moved into prim group
            tstate.nodeGroup.hits = 0;
        }

        // else current group is a node group
        else
        {
            let slot = firstbithigh( tstate.nodeGroup.hits );
            tstate.nodeGroup.hits = bfi(tstate.nodeGroup.hits, 0, slot, 1); // Clear it from list

            // If nodeGroup still contains hits to test, push it to stack.
            if ( tstate.nodeGroup.hits > 0 ) {
                pushNodeGroup( localStack, tstate );
            }
            
            // Reverse traversal priority
            let relIdx = int(tstate.nodeGroup.relIdxQueue >> (slot * 3)) & 0b111;
            let absIdx = tstate.nodeGroup.baseIdx + relIdx;

            // if (tstate.debug && nodeIdx == 7) {
            //     printf("----------------------debugging how we got to 7----------------------\n");
            //     printf("bitIdxBefore %d, hitsBefore %d, hitsAfter %d, bitIdxAfter %d, tstate.nodegrp_ord %d, slotIdx %d, relativeIdx %d, tstate.nodegrp_idx %d\n",
            //             bitIdxBefore   , hitsBefore   , hits        , bitIdx        , tstate.nodegrp_ord   , slotIdx   , relativeIdx,    tstate.nodegrp_idx);

            // }

            let node = fetchBVH8Node(tstate.BVH8N, absIdx);
            intersectNode(node, tstate.origin, tstate.tmax, tstate.nodeGroup, tstate.primGroup );
        }
    }

    // if (tstate.debug) {
    //     uint parent = tstate.BVH8LP.Load<uint>(finalTri * sizeof(uint));
    //     printf("final tri: %d parent %d\n", finalTri, parent);
    // }
}

//------------------------------------------------------------------------
[shader("callable")]
void TraverseBVH8(uniform TraverseRecord record, inout TraversePayload travParams) {
    TraversalState tstate = TraversalState();
    tstate.BVH8N = buffers[record.BVH8N.index];
    tstate.BVH8L = buffers[record.BVH8L.index];
    tstate.BVH8NP = buffers[record.BVH8NP.index];
    tstate.BVH8LP = buffers[record.BVH8LP.index];
    tstate.origin = travParams.OriginAndTMax.xyz;
    tstate.direction = float3(1.0, 1.0, 1.0);
    tstate.tmax = travParams.OriginAndTMax.w;
    tstate.debug = bool(travParams.QueryFlags & NN_FLAG_DEBUG);
    // if (tstate.debug) {
    //     printf("----------------------\n");
    // }
    tstate.debugHits = 0;
    tstate.debugTime = travParams.time;
    traverseBVH8(tstate);
    travParams.hitPrims = tstate.debugHits;
    travParams.OriginAndTMax.w = sqrt(tstate.tmax);
}

// Function to swap elements based on condition


void SortNetwork123(inout float4 arr0) {
    // Level 1
    arr0.xy = select((arr0.x < arr0.y).xx, arr0.yx, arr0.xy);
    arr0.zw = select((arr0.z < arr0.w).xx, arr0.wz, arr0.zw);  
    
    // Level 2
    arr0.xz = select((arr0.x < arr0.z).xx, arr0.zx, arr0.xz);
    arr0.yw = select((arr0.y < arr0.w).xx, arr0.wy, arr0.yw);

    // Level 3
    arr0.yz = select((arr0.y < arr0.z).xx, arr0.zy, arr0.yz);
}

float2 Swap(float2 xy) {
    return select((xy.x < xy).xx, xy.yx, xy);
    // if (ia < ib) {
    //     float tmp = ia;
    //     ia = ib;
    //     ib = tmp;
    // }
}

void SortNetwork4567(inout float4 arr0, inout float4 arr1) {
    // Assuming layers 1, 2 and 3 are already sorted
    
    // Level 4 and 5
    float4 arr01xy = float4(arr0.x, arr0.y, arr1.x, arr1.y);
    arr01xy = select((arr01xy.xy < arr01xy.zw).xyxy, arr01xy.zwxy, arr01xy.xyzw);
    arr01xy.yz = select((arr01xy.y < arr01xy.z).xx, arr01xy.zy, arr01xy.yz);
    arr0.xy = arr01xy.xy;
    
    float4 arr01zw = float4(arr0.z, arr0.w, arr1.z, arr1.w);
    arr01zw = select((arr01zw.xy < arr01zw.zw).xyxy, arr01zw.zwxy, arr01zw.xyzw);
    arr01zw.yz = select((arr01zw.y < arr01zw.z).xx, arr01zw.zy, arr01zw.yz);
    arr1.zw = arr01zw.zw;

    // Level 6
    float4 arr01zxwy = float4(arr01zw.x, arr01xy.z, arr01zw.y, arr01xy.w);
    arr01zxwy = select((arr01zxwy.xz < arr01zxwy.yw).xxyy, arr01zxwy.yxwz, arr01zxwy.xyzw);
    arr0.z = arr01zxwy.x;
    arr1.y = arr01zxwy.w;
    
    // Level 7
    float2 arr01wx = float2(arr01zxwy.z, arr01zxwy.y);
    arr01wx = select((arr01wx.x < arr01wx.y).xx, arr01wx.yx, arr01wx);
    arr0.w = arr01wx.x;
    arr1.x = arr01wx.y;
}

// // Sorting network for an array of 8 elements
// inline void SortWithNetwork(inout float4 arr0, inout float4 arr1) {
//     // Level 1
//     arr0.xy = select((arr0.x < arr0.y).xx, arr0.yx, arr0.xy);
//     arr0.zw = select((arr0.z < arr0.w).xx, arr0.wz, arr0.zw);    
//     arr1.xy = select((arr1.x < arr1.y).xx, arr1.yx, arr1.xy);
//     arr1.zw = select((arr1.z < arr1.w).xx, arr1.wz, arr1.zw);

//     // Level 2
//     arr0.xz = select((arr0.x < arr0.z).xx, arr0.zx, arr0.xz);
//     arr0.yw = select((arr0.y < arr0.w).xx, arr0.wy, arr0.yw);
//     arr1.xz = select((arr1.x < arr1.z).xx, arr1.zx, arr1.xz);
//     arr1.yw = select((arr1.y < arr1.w).xx, arr1.wy, arr1.yw);

//     // Level 3
//     arr0.yz = select((arr0.y < arr0.z).xx, arr0.zy, arr0.yz);
//     arr1.yz = select((arr1.y < arr1.z).xx, arr1.zy, arr1.yz);

//     // Level 4
//     Swap(arr0.x, arr1.x);
//     Swap(arr0.y, arr1.y);
//     Swap(arr0.z, arr1.z);
//     Swap(arr0.w, arr1.w);

//     // Level 5
//     Swap(arr0.y, arr1.x);
//     Swap(arr0.w, arr1.z);

//     // Level 6
//     Swap(arr0.z, arr1.x);
//     Swap(arr0.w, arr1.y);

//     // Level 7
//     Swap(arr0.w, arr1.x);
// }

struct IntersectTriangleParams {
    uint GeometryIndex : 8;
    uint PrimitiveIndex : 24;
    uint InstanceIndex : 22; // could be more...
    uint QueryFlags : 10;
    float4 ObjectOriginAndTMax;
};

void OpLine(uint32_t line, uint32_t column)
{
    spirv_asm {
        OpLine $(24) $line $column;
    };
}

// Discontinue any source-level location information that might be active from a previous 
// OpLine instruction,
void OpNoLine() {
    spirv_asm {
        OpNoLine;
    };
}

// Built-in nearest triangle program
[shader("callable")]
void IntersectTriangle(uniform TriangleRecord record, inout IntersectTriangleParams params) {
    // bool debug = bool(params.QueryFlags & NN_FLAG_DEBUG);
    RWByteAddressBuffer triangles = buffers[record.triangles.index];
    RWByteAddressBuffer vertices = buffers[record.vertices.index];
    
    int3 tri = triangles.LoadAligned<int3>(params.PrimitiveIndex * sizeof(int3));
    float3 pa = vertices.LoadAligned<float3>(tri.x * sizeof(float3));
    float3 pb = vertices.LoadAligned<float3>(tri.y * sizeof(float3));
    float3 pc = vertices.LoadAligned<float3>(tri.z * sizeof(float3));
    float3 x = params.ObjectOriginAndTMax.xyz;

    float3 p; float2 bc;
    float dist = findClosestPointTriangleSquared(pa, pb, pc, x, p, bc);
    
    // if (debug) {
    //     printf("Tri %d, dist %f\n", params.PrimitiveIndex, dist);
    //     printf("Tri verts %f %f %f %f %f %f %f %f %f\n", pa.x, pa.y, pa.z, pb.x, pb.y, pb.z, pc.x, pc.y, pc.z);
    //     printf("Query origin %f %f %f\n", x.x, x.y, x.z);
    // }
    // Report the "hit".
    params.ObjectOriginAndTMax.w = dist;
}

// Bottleneck is LGSB loading BVH2Node. 
// Active threads per warp is ~6 out of 32. 
[shader("callable")]
void TraverseBVH2(uniform TraverseRecord record, inout TraversePayload travParams) {
    bool debug = bool(travParams.QueryFlags & NN_FLAG_DEBUG);
    float tmax = travParams.OriginAndTMax.w * travParams.OriginAndTMax.w;
    float3 orig = travParams.OriginAndTMax.xyz;
    float3 dir = float3(1.0, 1.0, 1.0);

    RWByteAddressBuffer BVH2 = buffers[record.BVH2.index];
    travParams.hitNodes = 0;
    travParams.hitPrims = 0;

    uint localStack[64];
    int stackptr = 0;

    localStack[stackptr++] = uint(255 << 24); // pushing root node
    
    do {
        uint cluster = localStack[--stackptr];

        // Handle inner nodes
        if ((cluster >> 24) == 255) {
            travParams.hitNodes++;

            // Bottleneck by ~70%, long scoreboard stall here 
            BVH2Node node = fetchBVH2Node(BVH2, getClusterIndex(cluster));
            uint2 LR = uint2(node.getChildCluster(0), node.getChildCluster(1));
            bool2 LRIsLeaf = ((LR >> 24) != 255);
            
            float2 dists;
            for (int i = 0; i < 2; ++i) {
                if (LRIsLeaf[i]) { dists[i] = 0.0;} // if leaf, trivially accept
                else { // if node, compute distance
                    BVH2Node child = fetchBVH2Node(BVH2, getClusterIndex(LR[i]));
                    minDistTest(orig, dir, float2(0.0, 1e38f), child.getBounds()[0], child.getBounds()[1], dists[i]);
                }
            }
            
            // Farthest first, then closest. (We want to pop closest first)
            uint2 order = dists[0] <= dists[1] ? uint2(1, 0) : uint2(0, 1);
            if (dists[order[0]] < tmax) localStack[stackptr++] = LR[order[0]];
            if (dists[order[1]] < tmax) localStack[stackptr++] = LR[order[1]];
        }
        // Handle leaves
        else {
            travParams.hitPrims++;

            // System generated index of geometry in BLAS
            uint GeometryContributionToHitGroupIndex = getClusterMeta(cluster);

            IntersectTriangleParams params;
            params.GeometryIndex = GeometryContributionToHitGroupIndex;
            params.PrimitiveIndex = getClusterIndex(cluster);
            params.InstanceIndex = 0; // todo...
            params.QueryFlags = travParams.QueryFlags;
            params.ObjectOriginAndTMax = tmax;
            params.ObjectOriginAndTMax.xyz = orig;

            // (already accounts for StartAddress and StrideInBytes from DispatchRays())
            uint CallableIndex = (travParams.QueryContributionToHitGroupIndex +
                                  travParams.MultiplierForGeometryContributionToShaderIndex * GeometryContributionToHitGroupIndex)
                                  + record.InstanceContributionToHitGroupIndex; // Slightly different, every tree adds "2" here to account for traversal callables.

            CallShader(CallableIndex, params);

            // If we found a hit, update the traversal parameters
            if (params.ObjectOriginAndTMax.w < tmax) {
                tmax = params.ObjectOriginAndTMax.w;
            }
        }
    } while (stackptr > 0);

    travParams.OriginAndTMax.w = sqrt(tmax);
}

[shader("callable")]
void TraverseLinear(uniform TraverseRecord record, inout TraversePayload travParams) {
    bool debug = bool(travParams.QueryFlags & NN_FLAG_DEBUG);
    RWByteAddressBuffer BVH8L = buffers[record.BVH8L.index];

    // if (debug) printf("Hello from TraverseLinear\n");


    int N = record.N;
    for (int i = 0; i < N; ++i) {
        #if 0
        // System generated index of geometry in BLAS
        uint GeometryContributionToHitGroupIndex = 0; // assuming 1 geometry for now 

        IntersectTriangleParams params;
        params.GeometryIndex = GeometryContributionToHitGroupIndex;
        params.PrimitiveIndex = i & 0x00FFFFFF;
        params.InstanceIndex = 0; // todo...
        params.QueryFlags = travParams.QueryFlags;
        params.ObjectOriginAndTMax = travParams.OriginAndTMax;

        // (already accounts for StartAddress and StrideInBytes from DispatchRays())
        uint CallableIndex = (travParams.QueryContributionToHitGroupIndex +
                              travParams.MultiplierForGeometryContributionToShaderIndex * GeometryContributionToHitGroupIndex)
                                  + record.InstanceContributionToHitGroupIndex; // Slightly different, every tree adds "2" here to account for traversal callables.

        CallShader(CallableIndex, params);

        // If we found a hit, update the traversal parameters
        if (params.ObjectOriginAndTMax.w < travParams.OriginAndTMax.w) {
            //NearestNeighbor = params.PrimitiveIndex;
            travParams.OriginAndTMax = params.ObjectOriginAndTMax;
        }
        #else
        BVH8Triangle tri = BVH8L.Load<BVH8Triangle>(i * sizeof(BVH8Triangle));
        
        float3 p; float2 bc;
        float d = findClosestPointTriangle(tri.v0, tri.v1, tri.v2, travParams.OriginAndTMax.xyz, p, bc);
        if (d < travParams.OriginAndTMax.w) {
            travParams.OriginAndTMax.w = d;
        }
        #endif
    }
}
