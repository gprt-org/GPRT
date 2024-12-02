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
inline BVH2Node fetchBVH2Node(RWByteAddressBuffer BVH2N, int node_index) {
    float4 n[2];
    n[0] = BVH2N.LoadAligned<float4>(sizeof(float4) * (node_index * 2 + 0));
    n[1] = BVH2N.LoadAligned<float4>(sizeof(float4) * (node_index * 2 + 1));
    BVH2Node node = reinterpret<BVH2Node>(n);
    return node;
}

BVH8Triangle fetchTriangle(float4* BVH8L, int tri_index) {
    return reinterpret<BVH8Triangle>(float4[3](
        BVH8L[tri_index * 3 + 0],
        BVH8L[tri_index * 3 + 1],
        BVH8L[tri_index * 3 + 2]));
}

//------------------------------------------------------------------------


inline bool pushNodeGroup(inout StackEntry localStack[LOCAL_STACK_SIZE], inout TraversalState tstate)
{
    localStack[tstate.sp] = tstate.nodeGroup;
    tstate.sp = tstate.sp + 1;
    if (tstate.sp == LOCAL_STACK_SIZE) return false;
    return true;
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

[ForceInline]
inline uint bfe(uint value, int offset, int bits)
{
    return spirv_asm {
        result:$$uint = OpBitFieldUExtract $value $offset $bits
    };
}

[ForceInline]
inline uint3 bfe(uint3 value, int offset, int bits)
{
    return spirv_asm {
        result:$$uint3 = OpBitFieldUExtract $value $offset $bits
    };
}

[ForceInline]
inline uint bfi(uint base, uint insert, int offset, int bits)
{
    return spirv_asm {
        result:$$uint = OpBitFieldInsert $base $insert $offset $bits
    };
}

//------------------------------------------------------------------------

#define USE_DOT

[__unsafeForceInlineEarly]
inline void intersectNode(inout TraversalState tstate, const uint node_index, const uint64_t BVH8N)
{
    const uint *baseuPtr = ((uint *)(BVH8N)) + (32 * node_index);
    const uint8_t4 *baseu8Ptr = (uint8_t4 *)baseuPtr;
    const float3 *basef3Ptr = (float3 *)baseuPtr;
    const float *basefPtr = (float *)baseuPtr;

    const float3 pos = *basef3Ptr;
    const uint8_t4 psam = baseu8Ptr[3];

    tstate.nodeGroup.baseIdx = baseuPtr[4];
    tstate.primGroup.baseIdx = baseuPtr[5];

    let meta8 : vector<uint, 2> = uint2(baseuPtr[6], baseuPtr[7]);
    let scale = uintBitsToFloat(uint3(psam.xyz) << 23); // Shift 8-bit scale to floating-point exponent bits.
    let smask = psam.w;

    let apos = pos - tstate.origin;

    // Seems like the below code needs to be re-written as pure SPIR-V...
    // Otherwise, the compiler generates a ton of SFU ops converting unnecessarily between int8 and FP32...

    let aabbPtr = (baseuPtr + 8);
    float2x4 dists;
    [unroll]
    for (uint i = 0; i < 2; ++i) {
        let lox = *(aabbPtr + (0 | i));
        let loy = *(aabbPtr + (2 | i));
        let loz = *(aabbPtr + (4 | i));
        let hix = *(aabbPtr + (6 | i));
        let hiy = *(aabbPtr + (8 | i));
        let hiz = *(aabbPtr + (10 | i));

        let qlo : vector<uint, 3> = uint3(lox, loy, loz);
        let qhi : vector<uint, 3> = uint3(hix, hiy, hiz);

        [unroll]
        for (uint j = 0; j < 4; ++j) {
            // let used = bfe(smask, off | j, 1);
            let tc3 = mad(scale, bfe(qlo, (j << 3), 8), apos);
            let tf3 = mad(scale, bfe(qhi, (j << 3), 8), apos);
            let p = abs(clamp(0.0, tc3, tf3));
            dists[i][j] = dot(p, p);
        }

        let off : uint = i << 2;
        let used4 = bool4(bfe(smask, off | 0, 1), bfe(smask, off | 1, 1), bfe(smask, off | 2, 1), bfe(smask, off | 3, 1));
        let meta4 = uint4(bfe(meta8[i], 0, 4), bfe(meta8[i], 8, 4), bfe(meta8[i], 16, 4), bfe(meta8[i], 24, 4));
        dists[i] = select(used4, asfloat((asuint(dists[i] - FLT_EPSILON) & 0xFFFFFFF0) | meta4), FLT_MAX);

        SortNetwork123(dists[i]);
    }
    SortNetwork4567(dists[0], dists[1]);

    // Indicates which queue slots contain hits.
    uint32_t leafHits = 0;
    uint32_t nodeHits = 0;

    // The queue of relative indices, 3 bits each and 24b total.
    uint32_t queue = 0;

    [unroll]
    for (int i = 0; i < 2; ++i) {
        // A float containing the hit distance, with the item's 4-bit metadata in the low bits.
        // If the item is "empty", sortedDistsAndMeta is FLT_MAX.
        let meta4 = asuint(dists[i]) & 0x0000000F;
        let hit4 = (dists[i] < tstate.tmax);
        let leaf4 = bool4(meta4 & 0b1000);

        // Mark which slots in the queue belong to a leaf or an inner node.
        let leafHits4 = uint4(hit4 && leaf4) << (uint4(0, 1, 2, 3) + (i << 2));
        let nodeHits4 = uint4(hit4 && !leaf4) << (uint4(0, 1, 2, 3) + (i << 2));
        leafHits |= leafHits4.x | leafHits4.y | leafHits4.z | leafHits4.w;
        nodeHits |= nodeHits4.x | nodeHits4.y | nodeHits4.z | nodeHits4.w;

        // Store the relative index in the priority queue.
        // note, queueSlot*3 is at most 8*3 = 24 bits.
        let relIdx4 = (meta4 & 0b0111) << (uint4(0, 3, 6, 9) + (i * 12));
        queue |= relIdx4.x | relIdx4.y | relIdx4.z | relIdx4.w;
    }

    // Store entries into traversal state
    tstate.nodeGroup.isPrimGroup = 0;
    tstate.primGroup.isPrimGroup = 1;
    tstate.nodeGroup.relIdxQueue = queue;
    tstate.primGroup.relIdxQueue = queue;
    tstate.nodeGroup.hits = nodeHits;
    tstate.primGroup.hits = leafHits;
}

//------------------------------------------------------------------------
[shader("callable")]
void TraverseBVH8(uniform uint64_t BVH8N_ptr, uniform uint64_t BVH8L_ptr, inout TraversalState tstate) {
    // TraversalState tstate;
    // Stack localStack;

    //  // 2*LOCAL_STACK_SIZE (ie, 16) registers
    
        

    // int ntrav = WaveGetLaneCount();
    // int ntraversingPrims = 0;
    // int test = WaveActiveCountBits( true );

    // do {

    // const int maxSteps = 8;

    // StackEntry primStack[8];
    // int primStackPtr = 0;

    // Process inner nodes

    int primHits = 0;
    while (true)
    {
        // infLoopDetect--;
        // if (infLoopDetect < 0) break;

        // int ntrav = countbits( WaveActiveBallot( true ).x );
        // if (tstate.debug) printf("Ntrav %d\n", ntrav);
        // tstate.debugHits = ntrav;
        
        // handle leaves
        if (tstate.primGroup.hits > 0)
        {
            primHits++;
            // if (tstate.scheduleReorder == false) {                
            //     tstate.scheduleReorder = true;
            //     return;
            // }
            // else tstate.scheduleReorder = false;

            // handle leaves
            while (tstate.primGroup.hits > 0)
            {
                // if (countbits(WaveActiveBallot(true).x) * 2 < ntrav) // Is SIMD utilization too low?
                //     tstate.debugHits = 32;

                // tstate.debugHits++;

                let slot = firstbithigh(tstate.primGroup.hits);
                // if (slot < 0) break;
                tstate.primGroup.hits = bitfieldInsert(tstate.primGroup.hits, 0, slot, 1); // Clear it from list

                // Reverse traversal priority
                let relIdx = int(tstate.primGroup.relIdxQueue >> (slot * 3)) & 0b111;
                let absIdx = tstate.primGroup.baseIdx + relIdx;
                
                let BVH8L = (float4*)BVH8L_ptr;
                let tri = reinterpret<BVH8Triangle>(float4[3](
                BVH8L[absIdx * 3 + 0],
                BVH8L[absIdx * 3 + 1],
                BVH8L[absIdx * 3 + 2]));
                // let tri = fetchTriangle(, absIdx);
                float2 bc; float3 pclosest;
                float dist = findClosestPointTriangleSquared(tri.v0, tri.v1, tri.v2, tstate.origin, pclosest, bc);

                #ifndef USE_DOT
                dist = sqrt(dist);
                #endif

                if (dist < tstate.tmax) {
                    tstate.tmax = dist;
                }

                // if (tstate.debugHits == 32) break;
                // if(enablePostponing && ntraversingPrims * 2 < ntrav )  // Is SIMD utilization too low?
                // if(enablePostponing && 2 < ntrav )  // Is SIMD utilization too low?
                //     break;                                             // Postpone primitive intersections
            }

            // Potential pre-emption
            // if (primHits > 10) break; 

            // // Delay processing primitives until later (trying to reduce divergence...)
            // if (tstate.primGroup.hits > 0) {
            //     primStack[primStackPtr] = tstate.primGroup;
            //     primStackPtr = primStackPtr + 1;
            // }

            // // testing...
            // while (tstate.primGroup.hits > 0)
            // {                
            //     // if (countbits(WaveActiveBallot(true).x) * 2 < ntrav) // Is SIMD utilization too low?
            //     //     tstate.debugHits = 32;

            //     // tstate.debugHits++;

            //     let slot = firstbithigh(tstate.primGroup.hits);
            //     // if (slot < 0) break;
            //     tstate.primGroup.hits = bitfieldInsert(tstate.primGroup.hits, 0, slot, 1); // Clear it from list

            //     // Reverse traversal priority
            //     let relIdx = int(tstate.primGroup.relIdxQueue >> (slot * 3)) & 0b111;
            //     let absIdx = tstate.primGroup.baseIdx + relIdx;
                
            //     let BVH8L = (float4*)tstate.BVH8L_ptr;
            //     let tri = reinterpret<BVH8Triangle>(float4[3](
            //     BVH8L[absIdx * 3 + 0],
            //     BVH8L[absIdx * 3 + 1],
            //     BVH8L[absIdx * 3 + 2]));
            //     // let tri = fetchTriangle(, absIdx);
            //     float2 bc; float3 pclosest;
            //     float dist = findClosestPointTriangleSquared(tri.v0, tri.v1, tri.v2, tstate.origin, pclosest, bc);

            //     // #ifndef USE_DOT
            //     // dist = sqrt(dist);
            //     // #endif

            //     if (dist < tstate.tmax) {
            //         tstate.tmax = dist;
            //     }

            //     // if (tstate.debugHits == 32) break;
            //     // if(enablePostponing && ntraversingPrims * 2 < ntrav )  // Is SIMD utilization too low?
            //     // if(enablePostponing && 2 < ntrav )  // Is SIMD utilization too low?
            //     //     break;                                             // Postpone primitive intersections
            // }            
        }

        // stack pop?
        if ( tstate.nodeGroup.hits == 0 )
        {
            if( tstate.sp == 0 ) {
                tstate.doneTraversing = true;
                break; // empty stack ==> done
            }

            {
                tstate.sp--;
                tstate.nodeGroup = tstate.localStack[tstate.sp];
            }
            // tstate.nodeGroup = stackPop(tstate.localStack, tstate );
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
            if (tstate.nodeGroup.hits > 0) {
                tstate.localStack[tstate.sp] = tstate.nodeGroup;
                tstate.sp = tstate.sp + 1;                
                // pushNodeGroup(tstate.localStack, tstate);
            }
            
            // Reverse traversal priority
            let relIdx = int(tstate.nodeGroup.relIdxQueue >> (slot * 3)) & 0b111;
            let absIdx = tstate.nodeGroup.baseIdx + relIdx;

            // if (tstate.debug && nodeIdx == 7) {
            //     printf("----------------------debugging how we got to 7----------------------\n");
            //     printf("bitIdxBefore %d, hitsBefore %d, hitsAfter %d, bitIdxAfter %d, tstate.nodegrp_ord %d, slotIdx %d, relativeIdx %d, tstate.nodegrp_idx %d\n",
            //             bitIdxBefore   , hitsBefore   , hits        , bitIdx        , tstate.nodegrp_ord   , slotIdx   , relativeIdx,    tstate.nodegrp_idx);

            // }
            intersectNode(tstate, absIdx, BVH8N_ptr);
        }
    }

    // // Work through the primitives pushed onto the primitive queue
    // for (int step = maxSteps - 1; step >= 0; --step)
    // {
    //     if( primStackPtr == 0 ) {
    //         break; // empty stack ==> done
    //     }

    //     // pop a primitive to test...
    //     {
    //         primStackPtr--;
    //         tstate.primGroup = primStack[primStackPtr];
    //     }

    //     // handle leaves
    //     while (tstate.primGroup.hits > 0)
    //     {
    //         // if (countbits(WaveActiveBallot(true).x) * 2 < ntrav) // Is SIMD utilization too low?
    //         //     tstate.debugHits = 32;

    //         // tstate.debugHits++;

    //         let slot = firstbithigh(tstate.primGroup.hits);
    //         // if (slot < 0) break;
    //         tstate.primGroup.hits = bitfieldInsert(tstate.primGroup.hits, 0, slot, 1); // Clear it from list

    //         // Reverse traversal priority
    //         let relIdx = int(tstate.primGroup.relIdxQueue >> (slot * 3)) & 0b111;
    //         let absIdx = tstate.primGroup.baseIdx + relIdx;
            
    //         let BVH8L = (float4*)tstate.BVH8L_ptr;
    //         let tri = reinterpret<BVH8Triangle>(float4[3](
    //         BVH8L[absIdx * 3 + 0],
    //         BVH8L[absIdx * 3 + 1],
    //         BVH8L[absIdx * 3 + 2]));
    //         // let tri = fetchTriangle(, absIdx);
    //         float2 bc; float3 pclosest;
    //         float dist = findClosestPointTriangleSquared(tri.v0, tri.v1, tri.v2, tstate.origin, pclosest, bc);

    //         // #ifndef USE_DOT
    //         // dist = sqrt(dist);
    //         // #endif

    //         if (dist < tstate.tmax) {
    //             tstate.tmax = dist;
    //         }

    //         // if (tstate.debugHits == 32) break;
    //         // if(enablePostponing && ntraversingPrims * 2 < ntrav )  // Is SIMD utilization too low?
    //         // if(enablePostponing && 2 < ntrav )  // Is SIMD utilization too low?
    //         //     break;                                             // Postpone primitive intersections
    //     }        
    // }
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

struct IntersectTriangleParams {
    uint GeometryIndex : 8;
    uint PrimitiveIndex : 24;
    uint InstanceIndex : 22; // could be more...
    uint QueryFlags : 10;
    float4 ObjectOriginAndTMax;
};

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
