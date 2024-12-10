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

[ForceInline]
void BitonicSortNetworkPhase0(inout uint2 arr) {
    // Stage 0
    arr.xy = select((arr.x < arr.y).xx, arr.yx, arr.xy);
    // arr.zw = select((arr.z < arr.w).xx, arr.wz, arr.zw);
}

[ForceInline]
void BitonicSortNetworkPhase1(inout uint4 arr) {
    // Stage 0
    arr.xz = select((arr.x < arr.z).xx, arr.zx, arr.xz);
    arr.yw = select((arr.y < arr.w).xx, arr.wy, arr.yw);

    // Stage 1
    arr.yz = select((arr.y < arr.z).xx, arr.zy, arr.yz);
}

[ForceInline]
void BitonicSortNetworkPhase2(inout uint4 arr0, inout uint4 arr1) {
    // Level 4 and 5
    uint4 arr01xy = uint4(arr0.x, arr0.y, arr1.x, arr1.y);
    arr01xy = select((arr01xy.xy < arr01xy.zw).xyxy, arr01xy.zwxy, arr01xy.xyzw);
    arr01xy.yz = select((arr01xy.y < arr01xy.z).xx, arr01xy.zy, arr01xy.yz);
    arr0.xy = arr01xy.xy;

    uint4 arr01zw = uint4(arr0.z, arr0.w, arr1.z, arr1.w);
    arr01zw = select((arr01zw.xy < arr01zw.zw).xyxy, arr01zw.zwxy, arr01zw.xyzw);
    arr01zw.yz = select((arr01zw.y < arr01zw.z).xx, arr01zw.zy, arr01zw.yz);
    arr1.zw = arr01zw.zw;

    // Level 6
    uint4 arr01zxwy = uint4(arr01zw.x, arr01xy.z, arr01zw.y, arr01xy.w);
    arr01zxwy = select((arr01zxwy.xz < arr01zxwy.yw).xxyy, arr01zxwy.yxwz, arr01zxwy.xyzw);
    arr0.z = arr01zxwy.x;
    arr1.y = arr01zxwy.w;

    // Level 7
    uint2 arr01wx = uint2(arr01zxwy.z, arr01zxwy.y);
    arr01wx = select((arr01wx.x < arr01wx.y).xx, arr01wx.yx, arr01wx);
    arr0.w = arr01wx.x;
    arr1.x = arr01wx.y;
}

// Branch free compare and swap operation
inline void cswp(inout uint a, inout uint b) {
    uint tmp = max(a, b);
    b = min(a, b);
    a = tmp;
}

// 19 compare and swap operations
void oddEvenMergeSort8(inout uint4 lo, inout uint4 hi) {
    // For reference: https://hwlang.de/algorithmen/sortieren/networks/oemen.htm

    // Stage 1: Sort pairs
    cswp(lo.x, lo.y); // (0,1)
    cswp(lo.z, lo.w); // (2,3)
    cswp(hi.x, hi.y); // (4,5)
    cswp(hi.z, hi.w); // (6,7)

    // Stage 2: Sort quartets
    cswp(lo.x, lo.z); // (0,2)
    cswp(lo.y, lo.w); // (1,3)
    cswp(hi.x, hi.z); // (4,6)
    cswp(hi.y, hi.w); // (5,7)

    cswp(lo.y, lo.z); // (1,2)
    cswp(hi.y, hi.z); // (5,6)

    // Stage 3: Merge the two sorted halves (odd-even merge)
    cswp(lo.x, hi.x); // (0,4)
    cswp(lo.y, hi.y); // (1,5)
    cswp(lo.z, hi.z); // (2,6)
    cswp(lo.w, hi.w); // (3,7)

    cswp(lo.y, hi.x); // (1,4)
    cswp(lo.w, hi.z); // (3,6)

    cswp(lo.z, hi.x); // (2,4)
    cswp(lo.w, hi.y); // (3,5)

    cswp(lo.z, hi.y); // (2,5)
}

// This function implements a bitonic sorting network for 8 elements.
// Starting from any arrangement, it will produce a descending order sort.
void bitonicSort8(inout uint4 lo, inout uint4 hi) {
    // Step 1: Create bitonic sequences of length 2.
    // We'll do this by making pairs ascend or descend such that combined they form a bitonic sequence.
    // Ascending pairs:
    cswp(lo.x, lo.y); // (0,1) ascending pair
    cswp(lo.z, lo.w); // (2,3) ascending pair
    cswp(hi.x, hi.y); // (4,5) ascending pair
    cswp(hi.z, hi.w); // (6,7) ascending pair

    // To form a bitonic sequence, half should be "flipped".
    // We can flip by swapping indices in comparisons:
    cswp(lo.w, lo.z); // Re-compare (3,2) effectively making (2,3) descending
    cswp(hi.w, hi.z); // Re-compare (7,6) making (6,7) descending

    // Now we have two-element bitonic sequences:
    // (0,1) ascending, (2,3) descending, (4,5) ascending, (6,7) descending

    // Step 2: Merge these pairs into length-4 bitonic sequences.
    // Compare elements across these pairs to form bigger bitonic sequences of length 4.
    cswp(lo.x, lo.z); // (0,2)
    cswp(lo.y, lo.w); // (1,3)
    cswp(hi.x, hi.z); // (4,6)
    cswp(hi.y, hi.w); // (5,7)

    // Now refine these 4-length sequences:
    cswp(lo.y, lo.z); // (1,2)
    cswp(hi.y, hi.z); // (5,6)
    // After this, we have two length-4 bitonic sequences: one in lo (0..3), one in hi (4..7).

    // Step 3: Merge the two length-4 bitonic sequences into a single length-8 sequence.
    cswp(lo.x, hi.x); // (0,4)
    cswp(lo.y, hi.y); // (1,5)
    cswp(lo.z, hi.z); // (2,6)
    cswp(lo.w, hi.w); // (3,7)

    cswp(lo.y, hi.x); // (1,4)
    cswp(lo.w, hi.z); // (3,6)

    cswp(lo.z, hi.x); // (2,4)
    cswp(lo.w, hi.y); // (3,5)
}

// Branch free compare and swap operation
inline void cswp(inout half a, inout half b) {
    half tmp = max(a, b);
    b = min(a, b);
    a = tmp;
}

// 19 compare and swap operations
void oddEvenMergeSort8(inout half4 lo, inout half4 hi) {
    // For reference: https://hwlang.de/algorithmen/sortieren/networks/oemen.htm
    // Stage 1: Sort pairs
    cswp(lo.x, lo.y); // (0,1)
    cswp(lo.z, lo.w); // (2,3)
    cswp(hi.x, hi.y); // (4,5)
    cswp(hi.z, hi.w); // (6,7)

    // Stage 2: Sort quartets
    cswp(lo.x, lo.z); // (0,2)
    cswp(lo.y, lo.w); // (1,3)
    cswp(hi.x, hi.z); // (4,6)
    cswp(hi.y, hi.w); // (5,7)

    cswp(lo.y, lo.z); // (1,2)
    cswp(hi.y, hi.z); // (5,6)

    // Stage 3: Merge the two sorted halves (odd-even merge)
    cswp(lo.x, hi.x); // (0,4)
    cswp(lo.y, hi.y); // (1,5)
    cswp(lo.z, hi.z); // (2,6)
    cswp(lo.w, hi.w); // (3,7)

    cswp(lo.y, hi.x); // (1,4)
    cswp(lo.w, hi.z); // (3,6)

    cswp(lo.z, hi.x); // (2,4)
    cswp(lo.w, hi.y); // (3,5)

    cswp(lo.z, hi.y); // (2,5)
}

//------------------------------------------------------------------------

#define USE_DOT

[ForceInline]
inline T loadAligned<int alignment, T>(T *ptr)
{
    __target_switch
    {
    case spirv:
        return spirv_asm {
            result = OpLoad $$T $ptr Aligned !alignment;
        };
    default:
        return *ptr;
    }
}

[__unsafeForceInlineEarly]
inline void intersectNode(inout TraversalState tstate, const uint node_index, const uint64_t BVH8N)
{
    const float4 *fpPtr = ((float4 *)(BVH8N)) + (8 * node_index);
    const uint4 *uiPtr = ((uint4 *)(BVH8N)) + (8 * node_index);

    let A = loadAligned<16, float4>(fpPtr);
    let B = loadAligned<16, uint4>(uiPtr + 1);
    let pos = A.xyz;
    let scaleAndMask = asuint(A.w);

    tstate.nodeGroup.setBaseIndex(B.x);
    tstate.primGroup.setBaseIndex(B.y);

    let meta8 : vector<uint, 2> = B.zw;
    let expBits = uint3(bfe(scaleAndMask, 0, 8), bfe(scaleAndMask, 8, 8), bfe(scaleAndMask, 16, 8)) << 23;
    let smask = bfe(scaleAndMask, 24, 8);

    let scale = float3(asfloat(expBits.x), asfloat(expBits.y), asfloat(expBits.z)); // Shift 8-bit scale to floating-point exponent bits.
    let apos = pos - tstate.origin;
    
    uint4[3] AABBData;
    AABBData[0] = loadAligned<16, uint4>(uiPtr + 2);
    AABBData[1] = loadAligned<16, uint4>(uiPtr + 3);
    AABBData[2] = loadAligned<16, uint4>(uiPtr + 4);
    let LHD = reinterpret<LoHiData>(AABBData);

    uint2x4 dists;
    [unroll]
    for (uint i = 0; i < 2; ++i) {
        let qlo = uint3(LHD.lox[i], LHD.loy[i], LHD.loz[i]);
        let qhi = uint3(LHD.hix[i], LHD.hiy[i], LHD.hiz[i]);
        [unroll]
        for (uint j = 0; j < 4; ++j) {
            let tc3 = mad(scale, bfe(qlo, (j << 3), 8), apos);
            let tf3 = mad(scale, bfe(qhi, (j << 3), 8), apos);
            let p = abs(clamp(0.0, tc3, tf3));
            dists[i][j] = asuint(dot(p, p));
        }

        let off : uint = i << 2;
        let used4 = uint4(bfe(smask, off | 0, 1), bfe(smask, off | 1, 1), bfe(smask, off | 2, 1), bfe(smask, off | 3, 1));
        let meta4 = uint4(bfe(meta8[i], 0, 4), bfe(meta8[i], 8, 4), bfe(meta8[i], 16, 4), bfe(meta8[i], 24, 4));
        dists[i] = select(bool4(used4), (dists[i] & 0xFFFFFFF0) | meta4, UINT32_MAX);
    }
    oddEvenMergeSort8(dists[0], dists[1]);
    
    // Indicates which queue slots contain hits.
    uint32_t leafHits = 0;
    uint32_t nodeHits = 0;

    // The queue of relative indices, 3 bits each and 24b total.
    uint32_t queue = 0;

    [unroll]
    for (int i = 0; i < 2; ++i) {
        // A float containing the hit distance, with the item's 4-bit metadata in the low bits.
        // If the item is "empty", sortedDistsAndMeta is FLT_MAX.
        let meta4 = dists[i] & 0x0000000F;
        let hit4 = (dists[i] < tstate.itmax);
        let leaf4 = bool4(meta4 & 0b1000);

        // let meta4 = asuint16(dists[i]) & 0x000F;
        // let hit4 = (float4(dists[i]) < tstate.tmax);

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
    tstate.nodeGroup.setPrimGroup(false);
    tstate.primGroup.setPrimGroup(true);
    tstate.nodeGroup.setRelIdxQueue(queue);
    tstate.primGroup.setRelIdxQueue(queue);
    tstate.nodeGroup.setHits(nodeHits);
    tstate.primGroup.setHits(leafHits);
}

//------------------------------------------------------------------------
[shader("callable")]
void TraverseBVH8(uniform uint64_t BVH8N_ptr, uniform uint64_t BVH8L_ptr, inout TraversalState tstate) {
    while (true)
    {
        tstate.debugHits++;
        // handle leaves
        if (tstate.primGroup.getHits() > 0)
        {
            while (tstate.primGroup.getHits() > 0)
            {
                let slot = firstbithigh(tstate.primGroup.getHits());
                tstate.primGroup.setHits(bfi(tstate.primGroup.getHits(), 0, slot, 1)); // Clear it from list

                // Reverse traversal priority
                let relIdx = int(tstate.primGroup.getRelIdxQueue() >> (slot * 3)) & 0b111;
                let absIdx = tstate.primGroup.getBaseIndex() + relIdx;
                
                let BVH8L = (float4*)BVH8L_ptr;
                let tri = reinterpret<BVH8Triangle>(float4[3](
                    loadAligned<16, float4>(BVH8L + (absIdx * 3 + 0)),
                    loadAligned<16, float4>(BVH8L + (absIdx * 3 + 1)),
                    loadAligned<16, float4>(BVH8L + (absIdx * 3 + 2))));
                float2 bc; float3 pclosest;
                float dist = findClosestPointTriangleSquared(tri.v0, tri.v1, tri.v2, tstate.origin, pclosest, bc);

                #ifndef USE_DOT
                dist = sqrt(dist);
                #endif

                if (dist < tstate.tmax) {
                    tstate.tmax = dist;
                    tstate.itmax = asuint(dist);
                }
            }      
        }

        // stack pop?
        if ( tstate.nodeGroup.getHits() == 0 )
        {
            if( tstate.sp == 0 ) {
                tstate.doneTraversing = true;
                break; // empty stack ==> done
            }

            {
                tstate.sp--;
                tstate.nodeGroup = tstate.localStack[tstate.sp];
            }
        }

        // if current group is actually a prim group...
        if (tstate.nodeGroup.isPrimGroup() == 1)
        {
            tstate.primGroup = tstate.nodeGroup;
            
            // clear to indicate that work has moved into prim group
            tstate.nodeGroup.setHits(0);
        }

        // else current group is a node group
        else
        {
            let slot = firstbithigh( tstate.nodeGroup.getHits() );
            tstate.nodeGroup.setHits(bfi(tstate.nodeGroup.getHits(), 0, slot, 1)); // Clear it from list

            // If nodeGroup still contains hits to test, push it to stack.
            if (tstate.nodeGroup.getHits() > 0) {
                tstate.localStack[tstate.sp] = tstate.nodeGroup;
                tstate.sp = tstate.sp + 1;                
                // pushNodeGroup(tstate.localStack, tstate);
            }
            
            // Reverse traversal priority
            let relIdx = int(tstate.nodeGroup.getRelIdxQueue() >> (slot * 3)) & 0b111;
            let absIdx = tstate.nodeGroup.getBaseIndex() + relIdx;

            // if (tstate.debug && nodeIdx == 7) {
            //     printf("----------------------debugging how we got to 7----------------------\n");
            //     printf("bitIdxBefore %d, hitsBefore %d, hitsAfter %d, bitIdxAfter %d, tstate.nodegrp_ord %d, slotIdx %d, relativeIdx %d, tstate.nodegrp_idx %d\n",
            //             bitIdxBefore   , hitsBefore   , hits        , bitIdx        , tstate.nodegrp_ord   , slotIdx   , relativeIdx,    tstate.nodegrp_idx);

            // }
            intersectNode(tstate, absIdx, BVH8N_ptr);
        }
    }
}
