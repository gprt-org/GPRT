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

#include "gprt_nnq.h"

using namespace gprt;

float dot2(float3 v ) { return dot(v,v); }


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


inline float sdBox(float3 origin, float3 aabbMin, float3 aabbMax) {
    // For each dimension...
    // Determine which side that "p" is on.
    // Then, add the L2 distance to that side to our total
    float3 d = select((origin < aabbMin), origin - aabbMin,
                      select((origin > aabbMax), origin - aabbMax,
                             float3(0.f, 0.f, 0.f)));
    return sqrt(dot(d, d));
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




// uint8_t octant = 0;
// octant |= (dir.x < 0) ? uint8_t(1) : uint8_t(0);
// octant |= (dir.y < 0) ? uint8_t(2) : uint8_t(0);
// octant |= (dir.z < 0) ? uint8_t(4) : uint8_t(0);
// return octant;

uint8_t getOrder(uint8_t i, uint8_t octant) {
    return i ^ octant;
}

interface IGroup {
    bool representsNodeGroup();
    bool isEmptySet();
};

struct PrimGroup : IGroup {
    // 3 bits per child slot for a total of 8.
    // The number of bits set indicates the total number of primitives in the group.
    uint primHits : 24;
    uint isNodeGroup : 8;
    uint primitiveBaseIndex : 29;
    uint octant : 3;
    
    __init() {
        primHits = 0;
        isNodeGroup = 0;
        primitiveBaseIndex = 0;
        octant = 0;
    }

    bool isEmptySet() {
        return (primHits == 0);
    }

    bool representsNodeGroup() {
        // test to see if bits 24-31 are set
        return (isNodeGroup != 0);
    }

    [mutating]
    uint popNextPrimitive(bool debug = false) {
        // Find the index of the highest set bit in the primHits field of the primitive group entry.
        // clear that bit to remove the primitive.
        int bitIndex = firstbithigh(primHits);
        primHits &= ~(1 << bitIndex);

        // The index of each set bit in "primHits" directly
        // indicates the relative index of the primitive.
        return primitiveBaseIndex + bitIndex;

        // uint relativeIndex = countbits(primHits);
        // return primitiveBaseIndex + relativeIndex;

        // Each leaf stores up to 3 prims per byte. This determines which byte the bit belongs to.
        // uint slotIndex = bitIndex % 3;
        
        // The corresponding child slot index is found by reversing the traversal priority computation:
        //(bitIndex) ^ (7 - octant);
        
        
        
        // uint relativeIndex = countbits(imask & ~(-1 << slotIndex));
        // uint8_t prim = extractByte(primHits, bitIndex);
        
        // // if (debug) {
        // //     this.print();
        // //     printf("slot index: %d, relativeIndex %d\n", slotIndex, relativeIndex);
        // // }
        // return relativeIndex;
    }
};

struct NodeGroup : IGroup {
    uint imask : 8;
    uint pad : 16;
    uint hits : 8;
    uint childNodeBaseIndex: 29;
    uint octant : 3;

    __init() {
        imask = 0;
        pad = 0;
        hits = 0;
        childNodeBaseIndex = 0;
        octant = 0;
    }

    // For initializing the root node
    __init(uint childNodeBaseIndex, uint8_t imask, uint8_t hits, uint8_t octant) {        
        this.imask = imask;
        this.hits = hits;
        this.childNodeBaseIndex = childNodeBaseIndex;
        this.pad = 0;
        this.octant = octant;
    }
    
    bool isEmptySet() {
        return (hits == 0);
    }

    bool representsNodeGroup() {
        // test to see if bits 24-31 are set
        return (hits != 0);
    }

    void print() {
        printf("NodeGroup: imask %d, hits %d, childNodeBaseIndex %d, octant %d\n", int(imask), int(hits), int(childNodeBaseIndex), int(octant));
    }

    // The code below nearly identically matches the reference implementation here: 
    // https://github.com/AlanIWBFT/CWBVH/blob/master/src/TraversalKernelCWBVH.cu#L99
    // This implementation uses __bfind, which is equivalent to firstbithigh.
    [mutating]
    uint popClosestNode(bool debug = false) {
        // Find the index of the highest set bit in the hits field of the node group entry.
        // clear that bit to remove the node.
        int bitIndex = firstbithigh(hits);
        hits &= ~(1 << bitIndex);

        // The corresponding child slot index is found by subtracting the bias of 24 (already
        //  accounted for by the bit field) and reversing the traversal priority computation:
        uint slotIndex = (bitIndex) ^ (7 - octant);
        uint relativeIndex = countbits(imask & ~(0xFFFFFFFF << slotIndex));        
        return childNodeBaseIndex + relativeIndex;

        
        
        // for 130 case, expecting slot indices 1 and 7...

        // The relative index of the node is then obtained by computing the number of neighboring nodes
        // stored in the lower child slots. (Note, the !(-1 << slotIndex) is a mask that hides the higher bits.)
        // uint8_t imaskReversed = 0;
        // for (int i = 0; i < 8; ++i) {
        //     imaskReversed |= ((imask >> i) & 1) << (7 - i);
        // }

        // I would expect "slotIndex" to be 1 and 7 for the below case, and yet it is not.
        // Either imask is wrong, or the slotIndex computation is wrong.

        // round imask     hitsBefore  hitsAfter  bitIndex  octant octantInv   slotIndex  imaskMask  maskedIMask  relativeIndex 
        //     0  10000010  00110000    00010000          5       3         4          1         1           0          0
        //      1  10000010  00010000    00000000         4       3         4          0         0           0          0
        //
        //                                                5       3         4          1
        //                                                3       3         4          7 
        
        
        
        // if (debug) {
        //     // printf("imask %d, hitsBefore %d, hitsAfter %d, bitIndex %d, octant %d, slotIndex: %d, imaskMask %d, maskedIMask 0x%x, relativeIndex %d\n", 
        //     //         imask,    hitsBefore,    hits,         bitIndex,    octant,    slotIndex,     imaskMask,    maskedIMask,      relativeIndex);
        // }
        
        
    }
};

struct Group : IGroup {
    uint data0;
    uint data1;

    __init() { 
        data0 = 0; 
        data1 = 0;
    }
    bool representsNodeGroup() {
        return bool((data0 >> 24) != 0);
    }

    bool isEmptySet() {
        return data1 == 0;
    }
}

struct GroupStack {
    Group items[32];
    uint size;

    __init() {
        size = 0;
    }

    [mutating]
    bool push(Group G) {
        if (size >= 32) return false;
        items[size++] = G;
        return true;
    }

    [mutating]
    Group pop() {
        size--;
        return items[size];
    }
}

/**
 * \brief A universal stack data structure.
 * \tparam T The type of item to store in the stack.
 * \tparam N The maximum number of items the stack can hold.
 */
struct Stack<T, let N : int>
{
    T items[N];
    int stackptr;

    [mutating]
    void clear() { stackptr = 0; }

    uint size() { return stackptr; }

    __init() {
        stackptr = 0;
    }

    // Conditionally pushes an item onto the stack (by value).
    // If the stack is full, the function returns false.
    [mutating]
    bool push(T item) {
        if (stackptr >= N) return false;
        items[stackptr] = item;
        stackptr++;
        return true;
    }

    // Pops an item off the stack (by value)
    // If the stack is empty, the function returns false.
    [mutating]
    bool pop(inout T item) {
        if (stackptr == 0) return false;
        --stackptr;
        item = items[stackptr];        
        return true;
    }
}

struct BVH2Stack {
    // [32b child0, 32b child1]
    Stack<uint64_t, 64> stack;

    __init() {
        stack = Stack<uint64_t, 64>();
    }

    [mutating]
    bool push(int child0, int child1) {
        int64_t item = 0;
        item |= int64_t(child0) << 32ull;
        item |= int64_t(child1);
        return stack.push(item);
    }

    [mutating]
    bool pop(inout int child0, inout int child1) {
        int64_t item = 0;
        if (!stack.pop(item)) return false;
        child0 = int(item >> 32ull);
        child1 = int(item);
        return true;
    }

    uint size() { return stack.size(); }
};

// Function to swap elements based on condition
void Swap(inout uint ia, inout uint ib) {
    float a = asfloat(ia);
    float b = asfloat(ib);
    float t = min(a, b);
    b = max(a, b);
    a = t;
    ia = asuint(a);
    ib = asuint(b);
}

// Sorting network for an array of 8 elements
void SortWithNetwork(inout uint arr[8]) {
    // Level 1
    Swap(arr[0], arr[1]);
    Swap(arr[2], arr[3]);
    Swap(arr[4], arr[5]);
    Swap(arr[6], arr[7]);

    // Level 2
    Swap(arr[0], arr[2]);
    Swap(arr[1], arr[3]);
    Swap(arr[4], arr[6]);
    Swap(arr[5], arr[7]);

    // Level 3
    Swap(arr[1], arr[2]);
    Swap(arr[5], arr[6]);

    // Level 4
    Swap(arr[0], arr[4]);
    Swap(arr[1], arr[5]);
    Swap(arr[2], arr[6]);
    Swap(arr[3], arr[7]);

    // Level 5
    Swap(arr[1], arr[4]);
    Swap(arr[3], arr[6]);

    // Level 6
    Swap(arr[2], arr[4]);
    Swap(arr[3], arr[5]);

    // Level 7
    Swap(arr[3], arr[4]);
}

// __device__ __inline__ uint sign_extend_s8x4(uint i) { uint v; asm("prmt.b32 %0, %1, 0x0, 0x0000BA98;" : "=r"(v) : "r"(i)); return v; }
// Combine the sign-extended bytes 
// (this could probably be done more efficiently, but this logic is a bit more fool proof)

uint32_t sign_extend_s8x4(uint32_t word) {
    uint32_t result = (uint32_t(uint8_t(word >>  7) * 0xFF) << 0)
                    | (uint32_t(uint8_t(word >> 15) * 0xFF) << 8)
                    | (uint32_t(uint8_t(word >> 23) * 0xFF) << 16)
                    | (uint32_t(uint8_t(word >> 31) * 0xFF) << 24);
    return result;
}

uint64_t sign_extend_s8x8(uint64_t word) {
    // Combine the sign-extended bytes
    uint64_t result = (uint64_t(uint8_t(word >> 7ull) * 0xFF) << 0ull)
                    | (uint64_t(uint8_t(word >> 15ull) * 0xFF) << 8ull)
                    | (uint64_t(uint8_t(word >> 23ull) * 0xFF) << 16ull)
                    | (uint64_t(uint8_t(word >> 31ull) * 0xFF) << 24ull)
                    | (uint64_t(uint8_t(word >> 39ull) * 0xFF) << 32ull)
                    | (uint64_t(uint8_t(word >> 47ull) * 0xFF) << 40ull)
                    | (uint64_t(uint8_t(word >> 55ull) * 0xFF) << 48ull)
                    | (uint64_t(uint8_t(word >> 63ull) * 0xFF) << 56ull);
    return result;
}

uint64_t replicate_sign_bits(uint64_t word) {
    // Step-by-step spread the sign bit to all other bits in each byte
    uint64_t spread_signs = (word & 0x8080808080808080ull) >> 7;
    spread_signs |= spread_signs << 1;
    spread_signs |= spread_signs << 2;
    spread_signs |= spread_signs << 4;
    return spread_signs;
}

uint extract_byte32(uint i, uint n) { 
    return (i >> (n * 8)) & 0xFF; 
}

uint8_t extract_byte64(uint64_t bits, uint byteIndex) {
    return uint8_t((bits >> (8 * byteIndex)) & 0xFF);
};

// Todo, remove...
uint8_t extractByte(uint64_t bits, uint byteIndex) {
    return uint8_t((bits >> (8 * byteIndex)) & 0xFF);
};

// This function decompresses the boxes for node "n" and tests for intersection against them.
// This function is also responsible for traversal order computations and forming "Groups" which define traversal stack entries.
void intersectChildren(
    RWByteAddressBuffer BVH8N, RWByteAddressBuffer BVH8L, uint n, float4 q, inout NodeGroup Gn, inout PrimGroup Gp, bool debug) 
{
    BVH8Node node = BVH8N.Load<BVH8Node>(n * sizeof(BVH8Node));

    
    // if (debug) printf("imask %d, meta8 0x%lx\n", node.imask, node.meta);

    float3 parentScale = node.scale;
    float3 parentAABBMin = node.origin;
    float3 parentAABBMax = node.origin + parentScale;
    float3 parentCenter = (parentAABBMin + parentAABBMax) * 0.5f;

    // Determine the octant of the query point relative to the node's origin.
    float3 dir = parentCenter - q.xyz;
    uint8_t queryOctant = getOctant(dir);
    
    // Order children based on octant
    // Each "meta" stores a unary "count" of items stored in a slot in the top 3 bits, then the relative index in the lower 5 bits.
    //   For inner nodes, a value of "24" is added to the relative index, which means the 4th and 5th bits becomes set to 1. 
    //   For primitive nodes, the relative index is stored directly, meaning either the 4th or 5th bit will be set, but never both.
    uint64_t meta8 = node.meta8;

    // Invert the octant to get the traversal priority
    uint64_t octinv8 = uint64_t(7 - queryOctant) * 0x0101010101010101ull;
    
    // Each byte here will be either 0 or 1, indicating if the slot is an inner node or a leaf.
    // The shift by 1 tests of both 4th and 5th bits are set, which indicates that the slot is an inner node.
    uint64_t is_inner8 = (meta8 & (meta8 << 1ull)) & 0x1010101010101010ull;
    
    // If an inner node is set, replicating the sign bit here (for each byte in isolation) produces a mask
    // of 1s or 0s for each byte, masking out primitive slots and preserving inner slots. 
    uint64_t inner_mask8 = replicate_sign_bits(is_inner8 << 3ull);

    // We enable or disable octant reordering based on the inner mask.
    // If the node is inner, xoring does the following (simplified to quadrants, but same idea):
    //     octant   : [00] [01] [10] [11]
    //     order[0] : [00] [01] [10] [11]
    //     order[1] : [01] [00] [11] [10]
    //     order[2] : [10] [11] [00] [01]
    //     order[3] : [11] [10] [01] [00]
    // So, we preserve all the original "inner node" relative offsets, but these become reordered
    // Eg if the octant was "01", we could reorder to traverse 01 first.
    // We use the inverse of the octant since we want to traverse the closer octant first, rather than the
    // one furthest along the same direction.
    // This ordering is reversed when we pop the closest child off the group.
    uint64_t bitIndex8 = (meta8 ^ (octinv8 & inner_mask8)) & 0x1F1F1F1F1F1F1F1Full;
    
    // Now we'll compute the hit mask for our stack entry. The top 8 bits of this hit mask 
    // represent the inner node hits, while the bottom 24 bits represent the primitive hits.
    // (BVH8 nodes can store up to 24 primitives, with 1-3 primitives per slot.)
    uint32_t hitMask = 0;

    // Child bits here extracts the top 3 bits for each child slot. 
    uint64_t childTop3Bits8 = (meta8 >> 5ull) & 0x0707070707070707ull;


    // Traverse through slots in the node in their original layout
    uint8_t hits = 0; // mask of hits, ordered by original slots. 
    [unroll]
    for (int slotIndex = 0; slotIndex < 8; ++slotIndex) {
        // for now, doing this test naively for the moment...
        uint8_t meta = node.getMeta(slotIndex);
        if (meta == 0) continue;
        
        float2x3 childBounds = node.getChildBounds(slotIndex);

        // Compute the minimum distance from the query to this child node
        float distance = sdBox(q.xyz, childBounds);
        
        // Test to see if the box intersects the current query sphere
        bool intersected = true;//distance < q.w;
        hits |= (uint8_t(intersected) << slotIndex);
    }



    


    // if (debug) {
    //     for (int i = 0; i < 8; ++i) {
    //         uint8_t is_inner = extractByte(is_inner8, i);
    //         uint8_t innerMask = extractByte(inner_mask8, i);
    //         if (is_inner != 0 && innerMask != 0xFF) {
    //             printf("Error, inner node found but inner mask is not 0xFF\n");
    //         }

    //         if (is_inner == 0 && innerMask != 0) {
    //             printf("Error, leaf node found but inner mask is not 0\n");
    //         }
    //     }
    // }

    // if (debug) {
    //     printf("Node %d Meta8 %lx IsInner8 %lx InnerMask8 %lx\n", n, meta8, is_inner8, inner_mask8);
    // }

    for (uint8_t childIndex = 0; childIndex < 8; ++childIndex) {
        //uint slotIndex = getOrder(childIndex, 7-queryOctant);
        bool intersected = (hits & (1 << childIndex)) != 0;
        if (intersected) {
            // If it's an empty slot, childBits will be 0.
            // If it's an internal node, childBits will be 1.
            // If it's a primitive, then child bits will be from 001 to 111 (meaning, 1 to 3)
            uint8_t childTop3Bits = extractByte(childTop3Bits8, childIndex);

            // If the slot stores an inner node, bitIndex is the reordered index of the inner node in the node array.
            // If the slot stores a primitive, bitIndex is the relative primitive index,
            //   with the count in the upper 3 bits removed.
            uint8_t bitIndex = extractByte(bitIndex8, childIndex);
            hitMask |= (uint32_t(childTop3Bits) << (bitIndex));
        }
    }

    Gn.childNodeBaseIndex = node.childNodeBaseIndex;
    Gn.hits = (hitMask >> 24) & 0xFF;
    Gn.imask = node.imask;
    Gn.octant = queryOctant;

    Gp.octant = queryOctant;
    Gp.primitiveBaseIndex = node.primitiveBaseIndex;
    Gp.isNodeGroup = 0;
    Gp.primHits = hitMask & 0x00FFFFFF;

    // Currently not passing...
    bool testNodeEntry = true;
    if (testNodeEntry && debug) {
        NodeGroup testG = Gn;
        uint numInner = countbits(node.imask);
        uint numInGroup = 0;
        while (testG.isEmptySet() == false) {
            testG.popClosestNode();
            numInGroup++;
        }
        if (numInner != numInGroup) {
            printf("numInner != numInGroup\n");
            printf("Inner nodes %d, found %d\n", numInner, numInGroup);
        }

        testG = Gn;
        int on[8] = int[8](0, 0, 0, 0, 0, 0, 0, 0);
        for (int i = 0; i < 8; ++i) {
            if (node.getIMask(i)) on[i] = 1;
        }
        
        int fn[8] = int[8](0, 0, 0, 0, 0, 0, 0, 0);
        while (testG.isEmptySet() == false) {
            uint nodeIndex = testG.popClosestNode();
            uint relativeIndex = nodeIndex - node.childNodeBaseIndex;
            fn[relativeIndex] += 1;

            if (fn[relativeIndex] > 1) {
                printf("Error, found duplicate node index %d\n", relativeIndex);
                // How did we arrive at the hits mask here? it seems to be wrong...

                NodeGroup testG2 = Gn;
                while (testG2.isEmptySet() == false) {
                    testG2.popClosestNode(true); // now that we've caught a bug, we'll print out the nodes.
                }
            }
        }

        // // See if all nodes were found in the node stack
        // bool allFound = true;
        // for (int i = 0; i < 8; ++i) {
        //     if (on[i] == fn[i]) continue;
        //     allFound = false;
        // }

        // if (!allFound) {
        //     printf("Not all nodes found!\n");
        //     printf("Nodes at %d %d %d %d %d %d %d %d\n", on[0], on[1], on[2], on[3], on[4], on[5], on[6], on[7]);
        //     printf("Found nodes %d %d %d %d %d %d %d %d\n", fn[0], fn[1], fn[2], fn[3], fn[4], fn[5], fn[6], fn[7]);
        // }
    }

    // This seems to pass...
    // Note, assumes intersections are always true for now. 
    bool testPrimEntry = false;
    if (testPrimEntry && debug) {
        PrimGroup testG = Gp;

        int prims[8] = int[8](0, 0, 0, 0, 0, 0, 0, 0);
        bool onePrimIsLeaf = false;
        for (int i = 0; i < 8; ++i) {
            uint8_t meta = node.getMeta(i);
            if (meta == 0) continue;
            if (node.getIMask(i)) continue;
            
            int bitIndex = meta & 0x1F;
            prims[bitIndex] = 1;
            onePrimIsLeaf = true;
        }
        int op[8] = prims;

        if (onePrimIsLeaf && Gp.isEmptySet()) {
            printf("Error, found leaf prims but Gp is empty!\n");
        }

        int[8] fp = int[8](0, 0, 0, 0, 0, 0, 0, 0);
        while (testG.isEmptySet() == false) {
            uint primIndex = testG.popNextPrimitive(debug);
            uint relativeIndex = primIndex - node.primitiveBaseIndex;
            if (relativeIndex >= 8) {
                printf("Error, relative index is out of bounds. %d\n", relativeIndex);
            }
            else {
                fp[relativeIndex] = 1;
            }
        }

        // See if all primitives were found in the primitive stack
        bool allFound = true;
        for (int i = 0; i < 8; ++i) {
            if (prims[i] == fp[i]) continue;
            allFound = false;
            break;
        }
        
        if (!allFound) {
            printf("Not all prims found!\n");
            printf("Prims are %d %d %d %d %d %d %d %d\n", op[0], op[1], op[2], op[3], op[4], op[5], op[6], op[7]);
            printf("Found prims %d %d %d %d %d %d %d %d\n", fp[0], fp[1], fp[2], fp[3], fp[4], fp[5], fp[6], fp[7]);
        }
    }

    // if (debug && n == 2) {
        // printf("Parent node %d, Gn is Node? %d Gp is node? %d G is node? %d \n", n, 
        //     bool(Gn.representsNodeGroup()), bool(Gp.representsNodeGroup()), 
        //     bool(reinterpret<Group>(Gn).representsNodeGroup())
        // );

        // printf("Node %d HitMask 0x%x\n", n, hitMask);
        // printf("Hit mask %x\n", hitMask);
        // printf("NodeEntry {imask %d, hits %d, baseIndex %d, octant %d}\n", int(Gn.imask), int(Gn.hits), int(Gn.childNodeBaseIndex), int(Gn.octant));
        // printf("PrimEntry {primHits %d, isNodeGroup %d, baseIndex %d, octant %d}\n", int(Gp.primHits), int(Gp.isNodeGroup), int(Gp.primitiveBaseIndex), int(Gp.octant));
    // }

    

    //return debugBoxes;

    

    // // Top 8 bits of this are set. 
    // uint64_t hitMask = 0; 
    // if (debug) {
    //     printf("Octant %d, Meta8: 0x%lx, octinv8 0x%lx, is_inner8 0x%lx, inner_mask8 0x%lx, bit_index8 0x%lx\n", octant, meta8, octinv8, is_inner8, inner_mask8, bit_index8);
        
    //     for (int i = 0; i < 8; ++i) {
    //         //if (intersected)
    //         {
    //             // 0001 1001 0001 1011 0001 1101 0001 1000 0001 1110 0001 1111 0001 1100 0001 1010
    //             uint8_t child_bits = extractByte(child_bits8, i);
    //             uint8_t bit_index = extractByte(bit_index8, i);
    //             hitMask |= (uint64_t(child_bits) << uint64_t(bit_index));
    //         }
    //     }
    //     printf("ChildBits 0x%lx, bit_index8 %lx, HitMask 0x%lx\n", child_bits8, bit_index8, hitMask);
    //     // printf("Item %d child_bits %d bit_index %d\n", i, int(child_bits), int(bit_index));
    // }

    // bit_index8 now indicates directly where bits should be set in the "hits" and "triangle hits" fields in 
    // the node and triangle group entries.
    


    #if 1
    if (debug) {
        // // Experimenting...
        // BVH8Node node = load<BVH8Node>(BVH8N, 300);
        // uint64_t meta8 = node.meta;
        
        
        
        // for (uint i = 0; i < 8; ++i) {
        //     uint8_t meta = node.getMeta(i);
        //     // uint8_t is_inner = (meta & (meta << 1)) & 0x10;
        //     // uint8_t inner_mask = ;
        //     // printf("Meta %d: %d, is_inner %d, is_inner_shifted %d, inner_mask %d\n", i, 
        //     //     uint( uint8_t(meta)), 
        //     //     uint( uint8_t(is_inner8 >> (8 * i))), 
        //     //     uint( uint8_t(is_inner8_shifted >> (8 * i))), 
        //     //     uint( uint8_t(inner_mask8 >> (8ull * i))));
        // }
    
    }
    

    #else
    // Just a standard approach, sort from near to far.
    uint iMax = asuint(FLT_MAX) & 0xFFFFFFF0;
    uint[8] dists = uint[8](iMax, iMax, iMax, iMax, iMax, iMax, iMax, iMax);

    float3 o = node.origin + q.Origin;
    float3 scale = node.scale;
    for (int i = 0; i < 8; ++i) {
        let meta = node.getMeta(i);
        if (meta == 0) continue; // empty node
        
        let qchild = node.getChild(i);
        float2x3 child = float2x3(
            (float3(qchild[0]) / 256.f) * scale,
            (float3(qchild[1]) / 256.f) * scale,
        );
        float d = sdBox(o, child[0], child[1]);
        dists[i] = (asuint(d) & 0xFFFFFFF0) | i;
    }
    
    SortWithNetwork(dists);

    uint8_t nodeHits = 0;
    uint8_t primHits = 0;

    for (int i = 0; i < 8; ++i) {
        int index = dists[i] & 0x7;
        float d = asfloat(dists[i] & 0xFFFFFFF0);
        if (d > q.tMax) continue; // missed
        let meta = node.getMeta(i);
        if (meta == 0) continue;

        // item is inner node
        if ((meta >> 7) == 1) {

        }

    }

    #endif


    // float3 c = (bounds[0] + bounds[1]) * .5;
    // uint oct = getOctant(c - q.Origin);

    // for (each child that corresponds to an internal node...) {
    //     uint octinv4 = (7 - oct) * 0x01010101;

    //     uint traversalPriority = (slot_index ^ (7 - oct));

    // }

    // if (NG.isEmpty()) G = Group();
    // else G = reinterpret<Group>(NG);

    // if (PG.primHits == 0) Gp = PrimGroup();
    // else Gp = PG;


}

// bool ratioOfActiveThreadsIsTooLow() {
//     return false;
// }

// uint64_t sign_extend_s8x8(uint64_t word)
// {
//     // Create a mask that has the high bit of each byte set if the original high bit was set
//     uint64_t mask = (word & 0x8080808080808080ull) * 0x1FEull;

//     // XOR the mask with the original word and subtract the mask
//     uint64_t extended = (word ^ mask) - mask;

//     return extended;
// }




// // Mask to keep only the lowest bits of each byte
// spread_signs &= 0x0101010101010101ull;
// spread_signs *= 0xFF; // Spread across each byte




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
    RWByteAddressBuffer triangles = buffers[record.triangles.index];
    RWByteAddressBuffer vertices = buffers[record.vertices.index];
    
    int3 tri = triangles.LoadAligned<int3>(params.PrimitiveIndex * sizeof(int3));
    float3 pa = vertices.LoadAligned<float3>(tri.x * sizeof(float3));
    float3 pb = vertices.LoadAligned<float3>(tri.y * sizeof(float3));
    float3 pc = vertices.LoadAligned<float3>(tri.z * sizeof(float3));
    float3 x = params.ObjectOriginAndTMax.xyz;

    float3 p; float2 bc;
    float dist = findClosestPointTriangle(pa, pb, pc, x, p, bc);
    
    // Report the "hit".
    if (dist < params.ObjectOriginAndTMax.w) {
        params.ObjectOriginAndTMax.w = dist;
    }
}


// Bottleneck is LGSB loading BVH2Node. 
// Active threads per warp is ~6 out of 32. 
[shader("callable")]
void TraverseBVH2(uniform TraverseRecord record, inout TraversePayload travParams) {
    bool debug = bool(travParams.QueryFlags & NN_FLAG_DEBUG);
    RWByteAddressBuffer BVH2 = buffers[record.BVH2.index];
    uint NearestNeighbor = UINT32_MAX;
    var stack = Stack<uint2, 64>();
    stack.push(uint2((255 << 24) | 0, UINT32_MAX)); // pushing root node
    travParams.hitNodes = 0;
    travParams.hitPrims = 0;
    do {
        uint2 group = uint2(UINT32_MAX);
        stack.pop(group);
        if (group.x == UINT32_MAX) continue; // done traversing this group

        // pop item from group
        uint index = group.x;
        group = uint2(group.y, UINT32_MAX);
        stack.push(group);

        // If we're traversing an inner node
        if ((index >> 24) == 255) {
            travParams.hitNodes++;

            // Bottleneck by ~70%, long scoreboard stall here 
            BVH2Node node = BVH2.LoadAligned<BVH2Node>((index & 0x00FFFFFF) * sizeof(BVH2Node)); 
            float dParent = sdBox(travParams.OriginAndTMax.xyz, node.getBounds()[0], node.getBounds()[1]);
            if (dParent > travParams.OriginAndTMax.w) continue; // cull away subtree

            uint2 LR = uint2(node.getChildCluster(0), node.getChildCluster(1));
            bool2 LRIsLeaf = (LR >> 24) != 255;

            if (all(LRIsLeaf)) { stack.push(LR); }
            else if (LRIsLeaf.x) { stack.push(LR.xy); }
            else if (LRIsLeaf.y) { stack.push(LR.yx); }
            else {
                // Prioritize by AABB distance
                uint2 LRAddr = LR & 0x00FFFFFF;
                BVH2Node LNode = BVH2.LoadAligned<BVH2Node>(LRAddr.x * sizeof(BVH2Node));
                BVH2Node RNode = BVH2.LoadAligned<BVH2Node>(LRAddr.y * sizeof(BVH2Node));
                float2 dLR = float2(
                    sdBox(travParams.OriginAndTMax.xyz, LNode.getBounds()[0], LNode.getBounds()[1]),
                    sdBox(travParams.OriginAndTMax.xyz, RNode.getBounds()[0], RNode.getBounds()[1])
                );
                LR = select(dLR > float2(travParams.OriginAndTMax.w), uint2(UINT32_MAX), LR);
                if (all(LR == uint2(UINT32_MAX))) continue;
                stack.push((dLR.x < dLR.y) ? LR.xy : LR.yx);
            }
        }
        // else we're traversing a leaf
        else {
            travParams.hitPrims++;

            // System generated index of geometry in BLAS
            uint GeometryContributionToHitGroupIndex = (index >> 24);

            IntersectTriangleParams params;
            params.GeometryIndex = GeometryContributionToHitGroupIndex;
            params.PrimitiveIndex = index & 0x00FFFFFF;
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
                NearestNeighbor = params.PrimitiveIndex;
                travParams.OriginAndTMax = params.ObjectOriginAndTMax;
            }
        }
    } while (stack.size() > 0);

    if (debug) {
        printf("Nearest is %d with distance %f\n", NearestNeighbor, travParams.OriginAndTMax.w);
    }
}




int __bfind(uint x) {

    return firstbithigh(x);
}

int __popc(uint x) {
    return countbits(x);
}

#define PRIMITIVE_POSTPONING 1
#define DYNAMIC_FETCH 0

[shader("callable")]
void TraverseBVH8(uniform TraverseRecord record, inout TraversePayload travParams) {
    bool debug = bool(travParams.QueryFlags & NN_FLAG_DEBUG);
    uint debugHits = 0;

    var traversalStack = Stack<uint3, 16>(); // (for now, storing active octant in z)
    RWByteAddressBuffer BVH8N = buffers[record.BVH8N.index];
    RWByteAddressBuffer BVH8L = buffers[record.BVH8L.index];

    float3 orig = travParams.OriginAndTMax.xyz;
    
    // Note, tmax is squared to avoid square root operations for computing closest distances
	float tmin = 0.0, tmax = travParams.OriginAndTMax.w * travParams.OriginAndTMax.w;
	
    // float idirx, idiry, idirz;
	// uint octinv;

    // float4 root_n1 = BVH8N.LoadAligned<float4>(sizeof(float4) * (0 * 5 + 1));

    float debugTime = travParams.time;
    uint s = int(debugTime * 2.f) % 8;
    float3 dir = float3(
                    (((s >> 2) & 1) == 1) ? -1.0f : 1.0f,
					(((s >> 1) & 1) == 1) ? -1.0f : 1.0f,
					(((s >> 0) & 1) == 1) ? -1.0f : 1.0f);
    // float3 dir = float3(1.0, 1.0, 1.0);
    uint octinv = 7 - getOctant(dir);
    
	uint3 nodeGroup = uint3(0, 0b10000000000000000000000000000000, octinv);
	uint3 primGroup = uint3(0);
	int hitAddr = -1;
	float2 triangleuv;

    do {        
        if (nodeGroup.y > 0x00FFFFFF)
        {
            const uint hits = nodeGroup.y;
            const uint imask = nodeGroup.y;
            const uint child_bit_index = __bfind(hits);
            const uint child_node_base_index = nodeGroup.x;
            
            // (modified, pull the inversed octant from the stack)
            // const uint nodeGroupInvOctOrder = nodeGroup.z;

            uint yBefore = nodeGroup.y;
            nodeGroup.y &= ~(1 << child_bit_index);

            if (nodeGroup.y > 0x00FFFFFF)
            {
                traversalStack.push(nodeGroup);
            }

            const uint slot_index = (child_bit_index - 24) ^ octinv;
            const uint relative_index = __popc(imask & ~(0xFFFFFFFF << slot_index));
            const uint child_node_index = child_node_base_index + relative_index;

            // These represent the different rows of the current BVH8 node
            float4 n0, n1, n2, n3, n4;
            n0 = BVH8N.LoadAligned<float4>(sizeof(float4) * (child_node_index * 5 + 0));
            n1 = BVH8N.LoadAligned<float4>(sizeof(float4) * (child_node_index * 5 + 1));
            n2 = BVH8N.LoadAligned<float4>(sizeof(float4) * (child_node_index * 5 + 2));
            n3 = BVH8N.LoadAligned<float4>(sizeof(float4) * (child_node_index * 5 + 3));
            n4 = BVH8N.LoadAligned<float4>(sizeof(float4) * (child_node_index * 5 + 4));

            float3 p = n0.xyz;
            
            uint expAndIMaskBits = asuint(n0.w);

            uint3 e;
            e.x = (expAndIMaskBits >>  0) & 0xFF;
            e.y = (expAndIMaskBits >>  8) & 0xFF;
            e.z = (expAndIMaskBits >> 16) & 0xFF;
            uint localImask = (expAndIMaskBits >> 24) & 0xFF;
                            
            // base offsets
            nodeGroup.x = asuint(n1.x);
            primGroup.x = asuint(n1.y);
            primGroup.y = 0;

            // Octant from a ray traveling towards the center of the wide node.
            const float3 parent_scale = asfloat((e + 127) << 23);
            const float3 center = p + parent_scale * 0.5f;
            // const float3 dir = center - orig;
            // const uint localOctant = getOctant(dir);
            // const uint octinv = 7 - localOctant;
            // nodeGroup.z = nodeGroupInvOctOrder; //7 - localOctant;
            // primGroup.z = nodeGroupInvOctOrder; //7 - localOctant;

            uint hitmask = 0;

            // Adapting slab test to compute a closest distance query. 
            // Original code for a ray below. Primary difference is that idir is (1,1,1)
            {
                // const float adjusted_idirx = uint_as_float((e.x + 127) << 23) * idirx;
                // const float adjusted_idiry = uint_as_float((e.y + 127) << 23) * idiry;
                // const float adjusted_idirz = uint_as_float((e.z + 127) << 23) * idirz;
                // const float origx = -(orig.x - p.x) * idirx;
                // const float origy = -(orig.y - p.y) * idiry;
                // const float origz = -(orig.z - p.z) * idirz;
            }

            // Think of sign of "direction" as a traversal ordering mechanism. 
            // For octant traversal ordering, we simply look at the signs of dir. 
            float dirx = dir.x;
            float diry = dir.y;
            float dirz = dir.z;

            // Think of "inverse direction" as a scale factor, plus a reflection about the parent node origin.
            //
            // For ray queries, this "inverse direction" factor does two things:
            //  1. it scales the boxes inversely by the ray direction's length.
            //  2. it flips the planes of the box so that "lo" is what the ray hits first, and "hi" is what the ray hits last. 
            // 
            // But for a closest point query, we want a different behavior
            //  1. because non-uniform scales do not preserve Euclidean distances.
            //  2. because inverted boxes return the "maximum distance" to the origin rather than the "minimum".
            //
            // Still, we can repurpose parts of this slab test, by:
            //  1. decomposing the instance transform into a "scale" component and a "rigid" component.
            //  2. only transforming the query origin by the inverse of the rigid component. 
            //  3. then applying the "non-rigid" scale to the box bounds, 
            //      as well as the mirror flip of the planes by the ray direction by extracting the sign bits. 
            float idirx = 1.0f / dir.x;
            float idiry = 1.0f / dir.y;
            float idirz = 1.0f / dir.z;
            float adjusted_idirx = asfloat((e.x + 127) << 23) * idirx;
            float adjusted_idiry = asfloat((e.y + 127) << 23) * idiry;
            float adjusted_idirz = asfloat((e.z + 127) << 23) * idirz;

            // "idir" here flips the query origin, which is important for octant reordering, so here we do actually need a signed inverse dir here.
            // The magnitude of this inverse direction must be decoupled from the ray direction's magnitude, and should instead come from the instance transform.
            float origx = -(orig.x - p.x) * idirx;
            float origy = -(orig.y - p.y) * idiry;
            float origz = -(orig.z - p.z) * idirz;
            
            // Octant reordering by Ylitie/Karras/Liane
            /*var*/ /*const */ let octinv8 = octinv * 0x0101010101010101ull;
            /*var*/ /*const */ let meta8 = uint64_t(uint64_t(asuint(n1.w)) << 32) | uint64_t(asuint(n1.z));
            /*var*/ /*const */ let is_inner8 = (meta8 & (meta8 << 1)) & 0x1010101010101010ull;
            /*var*/ /*const */ let inner_mask8 = sign_extend_s8x8(is_inner8 << 3);
            /*var*/ /*const */ let bit_index8 = (meta8 ^ (octinv8 & inner_mask8)) & 0x1F1F1F1F1F1F1F1Full;
            /*var*/ /*const */ let child_bits8 = (meta8 >> 5) & 0x0707070707070707ull;
            
            // Potential micro-optimization: use PRMT to do the selection here, as described by the paper.
            
            // Note! "dir" here points from query origin to the centroid of the parent, and changes from 
            // one multinode to the next.
            // (Edit: running into issues with that, as negative directions flip the planes so lo is hi and hi is lo)
            
            // Recycling this piece of the ray-slab test for distance queries.
            // Note that "adjusted_idir x/y/z" is 1.0 for distance queries, 
            // making it "rigid" and "identity". 
            /*var*/ /*const */ let qLoHix = asuint((dirx < 0) ? float4(n3.zw, n2.xy) : float4(n2.xy, n3.zw));
            /*var*/ /*const */ let qLoHiy = asuint((diry < 0) ? float4(n4.xy, n2.zw) : float4(n2.zw, n4.xy));
            /*var*/ /*const */ let qLoHiz = asuint((dirz < 0) ? float4(n4.zw, n3.xy) : float4(n3.xy, n4.zw));
            /*var*/ /*const */ let tminx = float4x2((qLoHix.xy >>  0) & 0xFF, (qLoHix.xy >>  8) & 0xFF, (qLoHix.xy >> 16) & 0xFF, (qLoHix.xy >> 24) & 0xFF) * adjusted_idirx + origx;
            /*var*/ /*const */ let tminy = float4x2((qLoHiy.xy >>  0) & 0xFF, (qLoHiy.xy >>  8) & 0xFF, (qLoHiy.xy >> 16) & 0xFF, (qLoHiy.xy >> 24) & 0xFF) * adjusted_idiry + origy;
            /*var*/ /*const */ let tminz = float4x2((qLoHiz.xy >>  0) & 0xFF, (qLoHiz.xy >>  8) & 0xFF, (qLoHiz.xy >> 16) & 0xFF, (qLoHiz.xy >> 24) & 0xFF) * adjusted_idirz + origz;
            /*var*/ /*const */ let tmaxx = float4x2((qLoHix.zw >>  0) & 0xFF, (qLoHix.zw >>  8) & 0xFF, (qLoHix.zw >> 16) & 0xFF, (qLoHix.zw >> 24) & 0xFF) * adjusted_idirx + origx;
            /*var*/ /*const */ let tmaxy = float4x2((qLoHiy.zw >>  0) & 0xFF, (qLoHiy.zw >>  8) & 0xFF, (qLoHiy.zw >> 16) & 0xFF, (qLoHiy.zw >> 24) & 0xFF) * adjusted_idiry + origy;
            /*var*/ /*const */ let tmaxz = float4x2((qLoHiz.zw >>  0) & 0xFF, (qLoHiz.zw >>  8) & 0xFF, (qLoHiz.zw >> 16) & 0xFF, (qLoHiz.zw >> 24) & 0xFF) * adjusted_idirz + origz;

            [unroll]
            for (int childIndex = 0; childIndex < 8; childIndex++)
            {
                const uint child_bits = extract_byte64(child_bits8, childIndex);
                const uint bit_index = extract_byte64(bit_index8, childIndex);
                if (child_bits == 0) continue; // empty slot
                uint r = childIndex & 3;
                uint c = childIndex >> 2;

                // // Use VMIN, VMAX to compute the slabs
                // const float cmin = fmaxf(fmax_fmax(tminx[childIndex], tminy[childIndex], tminz[childIndex]), tmin);
                // const float cmax = fminf(fmin_fmin(tmaxx[childIndex], tmaxy[childIndex], tmaxz[childIndex]), tmax);
                // bool intersected = cmin <= cmax;

                // Changes from RT max of mins / min of maxes test explanated below:
                {
                    // For a distance query, we want the closest distance from the slabs to the origin. 
                    // So we no longer take the max of the mins and the min of the maxes.
                    // (Previous logic for rays commented out below.)
                       
                    
                    // Rather, we want to solve this equation: minDist(P, R) = \sum_{1}^{3} (r_i + (-p_i))^2.
                    //    where r_i = clamp(p_i, min_i, max_i)

                    // We can repurpose tmin_i and tmax_i from the ray-slab test with some simple math:
                    
                        // At the moment, tmin_i = min_i * 1.f + (-p_i) and tmax_i = max_i * 1.f + (-p_i).
                        // 
                        //       { min_i if p_i < min_i
                        // r_i = { max_i if p_i > max_i
                        //       { p_i otherwise       
                        // 
                        // and so...
                        //
                        //                { min_i + (-p_i) if p_i + (-p_i) < min_i + (-p_i)
                        // r_i + (-p_i) = { max_i + (-p_i) if p_i + (-p_i) > max_i + (-p_i)
                        //                { p_i + (-p_i) otherwise       
                        // 
                        // Which simplifies to...
                        //
                        //                { tmin_i if 0.0 < tmin_i
                        // r_i + (-p_i) = { tmax_i if 0.0 > tmax_i
                        //                { 0.0 otherwise       
                }

                // In short, we selectively add squares of tmin and tmax values to the excess.
                // For cases where a box is degenerate, this would theoretically return the maximum distance
                // In that case, if the box is still within the current "tmax", then it's intersected.
                
                // Note that ray direction flips the sign of the tmin and tmax values, so we need to flip the sign of the 
                // plane assure we get the intended minimum distance. 
                bool findMinDist = true; // If true, finds the minimum distance. If false, finds the farthest distance. 
                float ldistx = (findMinDist) ? min(tminx[r][c], tmaxx[r][c]) : max(tminx[r][c], tmaxx[r][c]);
                float rdistx = (findMinDist) ? max(tminx[r][c], tmaxx[r][c]) : min(tminx[r][c], tmaxx[r][c]);
                float ldisty = (findMinDist) ? min(tminy[r][c], tmaxy[r][c]) : max(tminy[r][c], tmaxy[r][c]);
                float rdisty = (findMinDist) ? max(tminy[r][c], tmaxy[r][c]) : min(tminy[r][c], tmaxy[r][c]);
                float ldistz = (findMinDist) ? min(tminz[r][c], tmaxz[r][c]) : max(tminz[r][c], tmaxz[r][c]);
                float rdistz = (findMinDist) ? max(tminz[r][c], tmaxz[r][c]) : min(tminz[r][c], tmaxz[r][c]);

                float excess = 0.0;
                if (0.0 < ldistx) excess += ldistx * ldistx;
                if (0.0 > rdistx) excess += rdistx * rdistx;
                if (0.0 < ldisty) excess += ldisty * ldisty;
                if (0.0 > rdisty) excess += rdisty * rdisty;
                if (0.0 < ldistz) excess += ldistz * ldistz;
                if (0.0 > rdistz) excess += rdistz * rdistz;

                // Intersection is true if distance is less than the current closest distance.
                bool intersected = (excess <= tmax);

                // Potential micro-optimization: use VSHL to implement this part, as described by the paper
                if (intersected) hitmask |= child_bits << bit_index;
            }
        

            // (in C++:)
            // nodeGroup.y = (hitmask & 0xFF000000) | (*((byte*)&n0.w + 3)); // <- highest order byte of n0.w is the imask 
            nodeGroup.y = (hitmask & 0xFF000000) | localImask;
            primGroup.y = hitmask & 0x00FFFFFF;
        }
        else
        {
            primGroup = nodeGroup;
            nodeGroup = uint3(0, 0, nodeGroup.z);
        }
        
        #if PRIMITIVE_POSTPONING
        const int totalThreads = WaveActiveCountBits(true);
        #endif

        while (primGroup.y != 0)
        {
            #if PRIMITIVE_POSTPONING
            const float Rt = 0.2;
            const int threshold = int(totalThreads * Rt);
            const int numActiveThreads = WaveActiveCountBits(true);
            if (numActiveThreads < threshold)
            {
                // if (debug) {
                //     printf("Postponing %d\n", primGroup.y);
                // }
                traversalStack.push(primGroup);
                break;
            }
            #endif
            
            uint leafIndex = __bfind(primGroup.y);
            uint leafAddr = primGroup.x + leafIndex;

            primGroup.y &= ~(1 << leafIndex);

            // if (debug && (primGroup.x == 66)) {
            //     printf("hit %d leafAddr %d (base %d, index %d, next index %d)\n", debugHits, leafAddr, primGroup.x, leafIndex, __bfind(primGroup.y));
            // }
            debugHits++;

            // if (debug) {
            //     printf("Iter %d\n prim %d primGroup.y 0x%x", iter, leafAddr, primGroup.y);
            // }

            // int triAddr = triangleGroup.x * 3 + triangleIndex * 3;


            BVH8Leaf leaf = BVH8L.Load<BVH8Leaf>(leafAddr * sizeof(BVH8Leaf));
            BVH8Triangle tri = leaf.tri;

            float3 p; float2 bc;
            float dist = findClosestPointTriangleSquared(tri.v[0], tri.v[1], tri.v[2], orig, p, bc);
            if (dist < tmax) {
                tmax = dist;
                // Update the nearest neighbor here
            }
        }

        if (nodeGroup.y <= 0x00FFFFFF)
        {
            if (traversalStack.stackptr > 0)
            {
                traversalStack.pop(nodeGroup);
            }
            else
            {
                break;
            }
        }
        
        #if DYNAMIC_FETCH
        // todo...
        #endif
    } while(true);

    travParams.hitPrims = debugHits;
    travParams.OriginAndTMax.w = sqrt(tmax);
}


[shader("callable")]
void TraverseBVH8_old(uniform TraverseRecord record, inout TraversePayload travParams) {
    bool debug = bool(travParams.QueryFlags & NN_FLAG_DEBUG);
    uint debugHits = 0;

    var traversalStack = Stack<uint3, 16>(); // (for now, storing active octant in z)
    RWByteAddressBuffer BVH8N = buffers[record.BVH8N.index];
    RWByteAddressBuffer BVH8L = buffers[record.BVH8L.index];

    float3 orig = travParams.OriginAndTMax.xyz;
    
    // Note, tmax is squared to avoid square root operations for computing closest distances
	float tmin = 0.0, tmax = travParams.OriginAndTMax.w * travParams.OriginAndTMax.w;
	
    // float idirx, idiry, idirz;
	// uint octinv;

    // float4 root_n1 = BVH8N.LoadAligned<float4>(sizeof(float4) * (0 * 5 + 1));

	uint3 nodeGroup = uint3(0, 0b10000000000000000000000000000000, 0);
	uint3 primGroup = uint3(0);
	int hitAddr = -1;
	float2 triangleuv;
    //uint octinv = 0;

    uint iter = 0;

    uint node14visits = 0;

    do {
        iter++;
        
        if (nodeGroup.y > 0x00FFFFFF)
        {
            const uint hits = nodeGroup.y;
            const uint imask = nodeGroup.y;
            const uint child_bit_index = __bfind(hits);
            const uint child_node_base_index = nodeGroup.x;
            
            // if (debug) {
            //     printf("Iter %d\n node %d nodeGroup.y 0x%x", iter, child_node_base_index, nodeGroup.y);
            // }

            // (modified, pull the inversed octant from the stack)
            const uint octinv = nodeGroup.z;

            uint yBefore = nodeGroup.y;
            nodeGroup.y &= ~(1 << child_bit_index);

            if (nodeGroup.y > 0x00FFFFFF)
            {
                traversalStack.push(nodeGroup);


                // if (debug && nodeGroup.x == 14 && nodeGroup.y == 16777376) {
                //     node14visits++;
                //     // Adding entry (%d, %d) to stack. imask %d. 
                //     // printf("Visits %d\n", node14visits);

                //     // for (int i = 0; i < traversalStack.stackptr; ++i) {
                //     //     printf("\t Stack[%2d] = (%d, %d)\n", i, traversalStack.items[i].x, traversalStack.items[i].y);
                //     // }
                //     // printf("Y before %d Y After %d\n", yBefore, nodeGroup.y);

                //     // for (int i = 0; i < traversalStack.stackptr; ++i) {
                //     //     if (all(traversalStack.items[i] == nodeGroup)) {
                //     //         printf("Error, found duplicate node group\n");
                //     //     }

                //     // }

                    
                // }

            }

            {
                const uint slot_index = (child_bit_index - 24) ^ octinv;
                const uint octinv4 = octinv * 0x01010101u;
                const uint relative_index = __popc(imask & ~(0xFFFFFFFF << slot_index));
                const uint child_node_index = child_node_base_index + relative_index;

                // These represent the different rows of the current BVH8 node
                float4 n0, n1, n2, n3, n4;
                n0 = BVH8N.LoadAligned<float4>(sizeof(float4) * (child_node_index * 5 + 0));
                n1 = BVH8N.LoadAligned<float4>(sizeof(float4) * (child_node_index * 5 + 1));
                n2 = BVH8N.LoadAligned<float4>(sizeof(float4) * (child_node_index * 5 + 2));
                n3 = BVH8N.LoadAligned<float4>(sizeof(float4) * (child_node_index * 5 + 3));
                n4 = BVH8N.LoadAligned<float4>(sizeof(float4) * (child_node_index * 5 + 4));

                float3 p = n0.xyz;
                
                uint expAndIMaskBits = asuint(n0.w);

                uint3 e;
                e.x = (expAndIMaskBits >>  0) & 0xFF;
                e.y = (expAndIMaskBits >>  8) & 0xFF;
                e.z = (expAndIMaskBits >> 16) & 0xFF;
                // e.x = *((char*)&n0.w + 0);
                // e.y = *((char*)&n0.w + 1);
                // e.z = *((char*)&n0.w + 2);
                uint localImask = (expAndIMaskBits >> 24) & 0xFF;
                                
                // base offsets
                nodeGroup.x = asuint(n1.x);
                primGroup.x = asuint(n1.y);
                primGroup.y = 0;

                // Octant from a ray traveling towards the center of the wide node.
                const float3 parent_scale = asfloat((e + 127) << 23);
                const float3 center = p + parent_scale * 0.5f;
                const float3 dir = center - orig;
                // const uint localOctant = getOctant(dir);
                // const uint octinv = 7 - localOctant;
                nodeGroup.z = octinv; //7 - localOctant;
                primGroup.z = octinv; //7 - localOctant;

                uint hitmask = 0;

                // Adapting slab test to compute a closest distance query. 
                // Original code for a ray below. Primary difference is that idir is (1,1,1)
                {
                    // const float adjusted_idirx = uint_as_float((e.x + 127) << 23) * idirx;
                    // const float adjusted_idiry = uint_as_float((e.y + 127) << 23) * idiry;
                    // const float adjusted_idirz = uint_as_float((e.z + 127) << 23) * idirz;
                    // const float origx = -(orig.x - p.x) * idirx;
                    // const float origy = -(orig.y - p.y) * idiry;
                    // const float origz = -(orig.z - p.z) * idirz;
                }

                // Think of "idir" as a scale factor that query applies to the child bounds. 
                // That scale factor "amplifies" the scale that the parent wide node applies to the child bounds.
                // For ray queries, that "idir" factor is the inverse of the ray direction.
                // But for a closest point query, this scale factor must be 1.

                // If the current instance has a transform applied, I recommend decomposing the transform into
                // a rigid component (rotation and translation) and a scale component.
                // Then, apply the scale component to the multinode bounds and the inverse of the rigid component to the query origin.
                //   This way, idir always stays "rigid" and "1" (so Euclidean distances are preserved). 
                float dirx = 1.0f;
                float diry = 1.0f;
                float dirz = 1.0f;
                float idirx = 1.0f;
                float idiry = 1.0f;
                float idirz = 1.0f;
                const float adjusted_idirx = asfloat((e.x + 127) << 23) * idirx;
                const float adjusted_idiry = asfloat((e.y + 127) << 23) * idiry;
                const float adjusted_idirz = asfloat((e.z + 127) << 23) * idirz;
                // (Like for rays, we take the negative of the origin, so tests below become FMA operations)
                const float origx = -(orig.x - p.x) * idirx;
                const float origy = -(orig.y - p.y) * idiry;
                const float origz = -(orig.z - p.z) * idirz;

                // First four slots
                {
                    // First 4
                    const uint meta4 = asuint(n1.z);
                    const uint is_inner4 = (meta4 & (meta4 << 1)) & 0x10101010;
                    const uint inner_mask4 = sign_extend_s8x4(is_inner4 << 3);
                    const uint bit_index4 = (meta4 ^ (octinv4 & inner_mask4)) & 0x1F1F1F1F;
                    const uint child_bits4 = (meta4 >> 5) & 0x07070707;

                    // Potential micro-optimization: use PRMT to do the selection here, as described by the paper.
                    // Note! "dir" here points from query origin to the centroid of the parent, and changes from 
                    // one multinode to the next.
                    // (Edit: running into issues with that, as negative directions flip the planes so lo is hi and hi is lo)
                    uint swizzledLox = (dirx < 0) ? asuint(n3.z) : asuint(n2.x);
                    uint swizzledHix = (dirx < 0) ? asuint(n2.x) : asuint(n3.z);

                    uint swizzledLoy = (diry < 0) ? asuint(n4.x) : asuint(n2.z);
                    uint swizzledHiy = (diry < 0) ? asuint(n2.z) : asuint(n4.x);

                    uint swizzledLoz = (dirz < 0) ? asuint(n4.z) : asuint(n3.x);
                    uint swizzledHiz = (dirz < 0) ? asuint(n3.x) : asuint(n4.z);
                    
                    float tminx[4];
                    float tminy[4];
                    float tminz[4];
                    float tmaxx[4];
                    float tmaxy[4];
                    float tmaxz[4];

                    // debugging... 
                    uint4 xLo = uint4(
                        ((swizzledLox >>  0) & 0xFF),
                        ((swizzledLox >>  8) & 0xFF),
                        ((swizzledLox >> 16) & 0xFF),
                        ((swizzledLox >> 24) & 0xFF)
                    );
                    uint4 xHi = uint4(
                        ((swizzledHix >>  0) & 0xFF),
                        ((swizzledHix >>  8) & 0xFF),
                        ((swizzledHix >> 16) & 0xFF),
                        ((swizzledHix >> 24) & 0xFF)
                    );
                    bool4 empty = (xLo > xHi);

                    // Recycling this piece of the ray-slab test for distance queries.
                    // Note that "adjusted_idir x/y/z" is 1.0 for distance queries, 
                    // making it "rigid" and "identity". 
                    tminx[0] = ((swizzledLox >>  0) & 0xFF) * adjusted_idirx + origx;
                    tminx[1] = ((swizzledLox >>  8) & 0xFF) * adjusted_idirx + origx;
                    tminx[2] = ((swizzledLox >> 16) & 0xFF) * adjusted_idirx + origx;
                    tminx[3] = ((swizzledLox >> 24) & 0xFF) * adjusted_idirx + origx;

                    tminy[0] = ((swizzledLoy >>  0) & 0xFF) * adjusted_idiry + origy;
                    tminy[1] = ((swizzledLoy >>  8) & 0xFF) * adjusted_idiry + origy;
                    tminy[2] = ((swizzledLoy >> 16) & 0xFF) * adjusted_idiry + origy;
                    tminy[3] = ((swizzledLoy >> 24) & 0xFF) * adjusted_idiry + origy;

                    tminz[0] = ((swizzledLoz >>  0) & 0xFF) * adjusted_idirz + origz;
                    tminz[1] = ((swizzledLoz >>  8) & 0xFF) * adjusted_idirz + origz;
                    tminz[2] = ((swizzledLoz >> 16) & 0xFF) * adjusted_idirz + origz;
                    tminz[3] = ((swizzledLoz >> 24) & 0xFF) * adjusted_idirz + origz;

                    tmaxx[0] = ((swizzledHix >>  0) & 0xFF) * adjusted_idirx + origx;
                    tmaxx[1] = ((swizzledHix >>  8) & 0xFF) * adjusted_idirx + origx;
                    tmaxx[2] = ((swizzledHix >> 16) & 0xFF) * adjusted_idirx + origx;
                    tmaxx[3] = ((swizzledHix >> 24) & 0xFF) * adjusted_idirx + origx;

                    tmaxy[0] = ((swizzledHiy >>  0) & 0xFF) * adjusted_idiry + origy;
                    tmaxy[1] = ((swizzledHiy >>  8) & 0xFF) * adjusted_idiry + origy;
                    tmaxy[2] = ((swizzledHiy >> 16) & 0xFF) * adjusted_idiry + origy;
                    tmaxy[3] = ((swizzledHiy >> 24) & 0xFF) * adjusted_idiry + origy;

                    tmaxz[0] = ((swizzledHiz >>  0) & 0xFF) * adjusted_idirz + origz;
                    tmaxz[1] = ((swizzledHiz >>  8) & 0xFF) * adjusted_idirz + origz;
                    tmaxz[2] = ((swizzledHiz >> 16) & 0xFF) * adjusted_idirz + origz;
                    tmaxz[3] = ((swizzledHiz >> 24) & 0xFF) * adjusted_idirz + origz;
                    
                    for (int childIndex = 0; childIndex < 4; childIndex++)
                    {
                        // For a distance query, we want the closest distance from the slabs to the origin. 
                        // So we no longer take the max of the mins and the min of the maxes.
                        // (Previous logic for rays commented out below.)
                        {
                            // // Use VMIN, VMAX to compute the slabs
                            // const float cmin = fmaxf(fmax_fmax(tminx[childIndex], tminy[childIndex], tminz[childIndex]), tmin);
                            // const float cmax = fminf(fmin_fmin(tmaxx[childIndex], tmaxy[childIndex], tmaxz[childIndex]), tmax);
                            // bool intersected = cmin <= cmax;
                        }

                        // Rather, we want to solve this equation: minDist(P, R) = \sum_{1}^{3} (r_i + (-p_i))^2.
                        //    where r_i = clamp(p_i, min_i, max_i)

                        // We can repurpose tmin_i and tmax_i from the ray-slab test with some simple math:
                        {
                            // At the moment, tmin_i = min_i * 1.f + (-p_i) and tmax_i = max_i * 1.f + (-p_i).
                            // 
                            //       { min_i if p_i < min_i
                            // r_i = { max_i if p_i > max_i
                            //       { p_i otherwise       
                            // 
                            // and so...
                            //
                            //                { min_i + (-p_i) if p_i + (-p_i) < min_i + (-p_i)
                            // r_i + (-p_i) = { max_i + (-p_i) if p_i + (-p_i) > max_i + (-p_i)
                            //                { p_i + (-p_i) otherwise       
                            // 
                            // Which simplifies to...
                            //
                            //                { tmin_i if 0.0 < tmin_i
                            // r_i + (-p_i) = { tmax_i if 0.0 > tmax_i
                            //                { 0.0 otherwise       
                        }
                        
                        // In short, we instead selectively add squares of tmin and tmax values to the excess.
                        float excess = 0.0;
                        if (0.0 < tminx[childIndex]) excess += tminx[childIndex] * tminx[childIndex];
                        if (0.0 > tmaxx[childIndex]) excess += tmaxx[childIndex] * tmaxx[childIndex];
                        if (0.0 < tminy[childIndex]) excess += tminy[childIndex] * tminy[childIndex];
                        if (0.0 > tmaxy[childIndex]) excess += tmaxy[childIndex] * tmaxy[childIndex];
                        if (0.0 < tminz[childIndex]) excess += tminz[childIndex] * tminz[childIndex];
                        if (0.0 > tmaxz[childIndex]) excess += tmaxz[childIndex] * tmaxz[childIndex];
                        
                        // Intersection is true if distance is less than the current closest distance.
                        bool intersected = (!empty[childIndex]) && (excess <= tmax);

                        // Potential micro-optimization: use VSHL to implement this part, as described by the paper
                        if (intersected)
                        {
                            const uint child_bits = extract_byte32(child_bits4, childIndex);
                            const uint bit_index = extract_byte32(bit_index4, childIndex);
                            hitmask |= child_bits << bit_index;
                        }
                    }
                }

                // Second four slots
                {
                    // Second 4
                    const uint meta4 = asuint(n1.w);
                    const uint is_inner4 = (meta4 & (meta4 << 1)) & 0x10101010;
                    const uint inner_mask4 = sign_extend_s8x4(is_inner4 << 3);
                    const uint bit_index4 = (meta4 ^ (octinv4 & inner_mask4)) & 0x1F1F1F1F;
                    const uint child_bits4 = (meta4 >> 5) & 0x07070707;

                    // Potential micro-optimization: use PRMT to do the selection here, as described by the paper
                    // Note! "dir" here points from query origin to the centroid of the parent, and changes from 
                    // one multinode to the next.
                    // (see edit in first part)
                    uint swizzledLox = (dirx < 0) ? asuint(n3.w) : asuint(n2.y);
                    uint swizzledHix = (dirx < 0) ? asuint(n2.y) : asuint(n3.w);

                    uint swizzledLoy = (diry < 0) ? asuint(n4.y) : asuint(n2.w);
                    uint swizzledHiy = (diry < 0) ? asuint(n2.w) : asuint(n4.y);

                    uint swizzledLoz = (dirz < 0) ? asuint(n4.w) : asuint(n3.y);
                    uint swizzledHiz = (dirz < 0) ? asuint(n3.y) : asuint(n4.w);

                    float tminx[4];
                    float tminy[4];
                    float tminz[4];
                    float tmaxx[4];
                    float tmaxy[4];
                    float tmaxz[4];

                    // debugging... 
                    uint4 xLo = uint4(
                        ((swizzledLox >>  0) & 0xFF),
                        ((swizzledLox >>  8) & 0xFF),
                        ((swizzledLox >> 16) & 0xFF),
                        ((swizzledLox >> 24) & 0xFF)
                    );
                    uint4 xHi = uint4(
                        ((swizzledHix >>  0) & 0xFF),
                        ((swizzledHix >>  8) & 0xFF),
                        ((swizzledHix >> 16) & 0xFF),
                        ((swizzledHix >> 24) & 0xFF)
                    );
                    bool4 empty = (xLo > xHi);

                    // Recycling this piece of the ray-slab test for distance queries.
                    // Note that "adjusted_idir x/y/z" is 1.0 for distance queries, 
                    // making it "rigid" and "identity". 
                    tminx[0] = ((swizzledLox >>  0) & 0xFF) * adjusted_idirx + origx;
                    tminx[1] = ((swizzledLox >>  8) & 0xFF) * adjusted_idirx + origx;
                    tminx[2] = ((swizzledLox >> 16) & 0xFF) * adjusted_idirx + origx;
                    tminx[3] = ((swizzledLox >> 24) & 0xFF) * adjusted_idirx + origx;

                    tminy[0] = ((swizzledLoy >>  0) & 0xFF) * adjusted_idiry + origy;
                    tminy[1] = ((swizzledLoy >>  8) & 0xFF) * adjusted_idiry + origy;
                    tminy[2] = ((swizzledLoy >> 16) & 0xFF) * adjusted_idiry + origy;
                    tminy[3] = ((swizzledLoy >> 24) & 0xFF) * adjusted_idiry + origy;

                    tminz[0] = ((swizzledLoz >>  0) & 0xFF) * adjusted_idirz + origz;
                    tminz[1] = ((swizzledLoz >>  8) & 0xFF) * adjusted_idirz + origz;
                    tminz[2] = ((swizzledLoz >> 16) & 0xFF) * adjusted_idirz + origz;
                    tminz[3] = ((swizzledLoz >> 24) & 0xFF) * adjusted_idirz + origz;

                    tmaxx[0] = ((swizzledHix >>  0) & 0xFF) * adjusted_idirx + origx;
                    tmaxx[1] = ((swizzledHix >>  8) & 0xFF) * adjusted_idirx + origx;
                    tmaxx[2] = ((swizzledHix >> 16) & 0xFF) * adjusted_idirx + origx;
                    tmaxx[3] = ((swizzledHix >> 24) & 0xFF) * adjusted_idirx + origx;

                    tmaxy[0] = ((swizzledHiy >>  0) & 0xFF) * adjusted_idiry + origy;
                    tmaxy[1] = ((swizzledHiy >>  8) & 0xFF) * adjusted_idiry + origy;
                    tmaxy[2] = ((swizzledHiy >> 16) & 0xFF) * adjusted_idiry + origy;
                    tmaxy[3] = ((swizzledHiy >> 24) & 0xFF) * adjusted_idiry + origy;

                    tmaxz[0] = ((swizzledHiz >>  0) & 0xFF) * adjusted_idirz + origz;
                    tmaxz[1] = ((swizzledHiz >>  8) & 0xFF) * adjusted_idirz + origz;
                    tmaxz[2] = ((swizzledHiz >> 16) & 0xFF) * adjusted_idirz + origz;
                    tmaxz[3] = ((swizzledHiz >> 24) & 0xFF) * adjusted_idirz + origz;

                    for (int childIndex = 0; childIndex < 4; childIndex++)
                    {
                        // For a distance query, we want the closest distance from the slabs to the origin. 
                        // So we no longer take the max of the mins and the min of the maxes.
                        // (Previous logic for rays commented out below.)
                        {
                            // // Use VMIN, VMAX to compute the slabs
                            // const float cmin = fmaxf(fmax_fmax(tminx[childIndex], tminy[childIndex], tminz[childIndex]), tmin);
                            // const float cmax = fminf(fmin_fmin(tmaxx[childIndex], tmaxy[childIndex], tmaxz[childIndex]), tmax);
                            // bool intersected = cmin <= cmax;
                        }

                        // Rather, we want to solve this equation: minDist(P, R) = \sum_{1}^{3} (r_i + (-p_i))^2.
                        //    where r_i = clamp(p_i, min_i, max_i)

                        // We can repurpose tmin_i and tmax_i from the ray-slab test with some simple math:
                        {
                            // At the moment, tmin_i = min_i * 1.f + (-p_i) and tmax_i = max_i * 1.f + (-p_i).
                            // 
                            //       { min_i if p_i < min_i
                            // r_i = { max_i if p_i > max_i
                            //       { p_i otherwise       
                            // 
                            // and so...
                            //
                            //                { min_i + (-p_i) if p_i + (-p_i) < min_i + (-p_i)
                            // r_i + (-p_i) = { max_i + (-p_i) if p_i + (-p_i) > max_i + (-p_i)
                            //                { p_i + (-p_i) otherwise       
                            // 
                            // Which simplifies to...
                            //
                            //                { tmin_i if 0.0 < tmin_i
                            // r_i + (-p_i) = { tmax_i if 0.0 > tmax_i
                            //                { 0.0 otherwise       
                        }
                        
                        // In short, we instead selectively add squares of tmin and tmax values to the excess.
                        float excess = 0.0;
                        if (0.0 < tminx[childIndex]) excess += tminx[childIndex] * tminx[childIndex];
                        if (0.0 > tmaxx[childIndex]) excess += tmaxx[childIndex] * tmaxx[childIndex];
                        if (0.0 < tminy[childIndex]) excess += tminy[childIndex] * tminy[childIndex];
                        if (0.0 > tmaxy[childIndex]) excess += tmaxy[childIndex] * tmaxy[childIndex];
                        if (0.0 < tminz[childIndex]) excess += tminz[childIndex] * tminz[childIndex];
                        if (0.0 > tmaxz[childIndex]) excess += tmaxz[childIndex] * tmaxz[childIndex];

                        // Intersection is true if distance is less than the current closest distance.
                        bool intersected = (!empty[childIndex]) && (excess <= tmax);

                        // Potential micro-optimization: use VSHL to implement this part, as described by the paper
                        if (intersected)
                        {
                            const uint child_bits = extract_byte32(child_bits4, childIndex);
                            const uint bit_index = extract_byte32(bit_index4, childIndex);
                            hitmask |= child_bits << bit_index;
                        }
                    }
                }

                // (in C++:)
                // nodeGroup.y = (hitmask & 0xFF000000) | (*((byte*)&n0.w + 3)); // <- highest order byte of n0.w is the imask 
                nodeGroup.y = (hitmask & 0xFF000000) | localImask;
                primGroup.y = hitmask & 0x00FFFFFF;

                // if (debug) {
                //     printf("Node %d hitmask 0x%x\n", child_node_index, hitmask);
                // }
            }
        }
        else
        {
            primGroup = nodeGroup;
            nodeGroup = uint3(0, 0, nodeGroup.z);
        }
        
        #if PRIMITIVE_POSTPONING
        const int totalThreads = WaveActiveCountBits(true);
        #endif

        while (primGroup.y != 0)
        {
            #if PRIMITIVE_POSTPONING
            const float Rt = 0.2;
            const int threshold = int(totalThreads * Rt);
            const int numActiveThreads = WaveActiveCountBits(true);
            if (numActiveThreads < threshold)
            {
                // if (debug) {
                //     printf("Postponing %d\n", primGroup.y);
                // }
                traversalStack.push(primGroup);
                break;
            }
            #endif
            
            uint leafIndex = __bfind(primGroup.y);
            uint leafAddr = primGroup.x + leafIndex;

            primGroup.y &= ~(1 << leafIndex);

            // if (debug && (primGroup.x == 66)) {
            //     printf("hit %d leafAddr %d (base %d, index %d, next index %d)\n", debugHits, leafAddr, primGroup.x, leafIndex, __bfind(primGroup.y));
            // }
            debugHits++;

            // if (debug) {
            //     printf("Iter %d\n prim %d primGroup.y 0x%x", iter, leafAddr, primGroup.y);
            // }

            // int triAddr = triangleGroup.x * 3 + triangleIndex * 3;


            BVH8Leaf leaf = BVH8L.Load<BVH8Leaf>(leafAddr * sizeof(BVH8Leaf));
            BVH8Triangle tri = leaf.tri;

            float3 p; float2 bc;
            float dist = findClosestPointTriangleSquared(tri.v[0], tri.v[1], tri.v[2], orig, p, bc);
            if (dist < tmax) {
                tmax = dist;
                // Update the nearest neighbor here
            }
        }

        if (nodeGroup.y <= 0x00FFFFFF)
        {
            if (traversalStack.stackptr > 0)
            {
                traversalStack.pop(nodeGroup);
            }
            else
            {
                break;
            }
        }
        
        #if DYNAMIC_FETCH
        // todo...
        #endif
    } while(true);

    // if (debug) {
    //     printf("Hits: %d\n", debugHits);
    // }

    travParams.OriginAndTMax.w = sqrt(tmax);
}

[shader("callable")]
void TraverseBVH8Old1(uniform TraverseRecord record, inout TraversePayload travParams) {
    bool debug = bool(travParams.QueryFlags & NN_FLAG_DEBUG);
    float debugTime = travParams.time;

    bool showAABB = bool(int(debugTime) % 2);

    RWByteAddressBuffer BVH8N = buffers[record.BVH8N.index];
    RWByteAddressBuffer BVH8L = buffers[record.BVH8L.index];

    // Separating out inner nodes from primitives
    Group G = reinterpret<Group>(NodeGroup(/*root*/ 0, /*imask*/ 1 << 7, /*hits*/ 1 << 7, /*octant*/ 0));
    PrimGroup Gp = PrimGroup();
    NodeGroup Gn = NodeGroup();
    var stack = GroupStack();

    int iters = 0;

    bool primFound = false;

    // loop
    do {
        if (G.representsNodeGroup())
        {
            NodeGroup N = reinterpret<NodeGroup>(G);
            uint n = N.popClosestNode();
            if (!N.isEmptySet()) stack.push(reinterpret<Group>(N));
            // By design, G either only contains nodes or only contains primitives.
            intersectChildren(BVH8N, BVH8L, n, travParams.OriginAndTMax, /*hit nodes*/ Gn, /*hit prims*/ Gp, debug);
            G = reinterpret<Group>(Gn);
        }

        // At this point we handle the primitive group "Gp"...
        else { // G represents a primitive group (potentially pushed below)
            Gp = reinterpret<PrimGroup>(G);
            G = Group();
        }

        while (!Gp.isEmptySet()) {
            // todo
            // if (ratioOfActiveThreadsIsTooLow()) {
            //     stack.push(Gt);
            //     break;
            // }
            uint primAddr = Gp.popNextPrimitive(debug);
            uint pidx = primAddr - Gp.primitiveBaseIndex;
            // if (debug) {
            //     printf("Primitive %d\n", primAddr);
            // }
            BVH8Leaf leaf = BVH8L.Load<BVH8Leaf>(primAddr * sizeof(BVH8Leaf));
            
            BVH8Triangle tri = leaf.tri;

            // if (debug) {

            //     float3 aabbMin = min(min(tri.v[0], tri.v[1]), tri.v[2]);
            //     float3 aabbMax = max(max(tri.v[0], tri.v[1]), tri.v[2]);
                
            //     if (any(aabbMin < debugBoxes[pidx][0]) || any(aabbMax > debugBoxes[pidx][1])) {
            //         printf("Primitive %d AABB does not fit in parent box\n", primAddr);
            //     }
            // }

            float3 p; float2 bc;
            float dist = findClosestPointTriangle(tri.v[0], tri.v[1], tri.v[2], travParams.OriginAndTMax.xyz, p, bc);
            if (dist < travParams.OriginAndTMax.w) {
                travParams.OriginAndTMax.w = dist;
                primFound = true;
                // Update the nearest neighbor here
            }

            
            // t = GetNextTriangle(Gt);
            // Gt = Gt \ t;
            // IntersectTriangle(t, q);
        }

        //if (primFound) return; // temp

        if (G.isEmptySet()) {
            if (stack.size == 0) {
                break;
            }
            G = stack.pop();
        }

        iters++;
        if (iters > 100000) {
            printf("Iters exceeded 100000\n");
            break; // temp
        }
    } while(true);
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
        BVH8Leaf leaf = BVH8L.Load<BVH8Leaf>(i * sizeof(BVH8Leaf));
        BVH8Triangle tri = leaf.tri;

        float3 p; float2 bc;
        float d = findClosestPointTriangle(tri.v[0], tri.v[1], tri.v[2], travParams.OriginAndTMax.xyz, p, bc);
        if (d < travParams.OriginAndTMax.w) {
            travParams.OriginAndTMax.w = d;
        }
        #endif
    }
}

// struct StackItem {
//     uint32_t _key;
//     float _val;
//     int key() { return _key;}
//     float value() { return _val;}

//     static StackItem Create(uint key, float value) {
//         StackItem pair;
//         pair._key = key;
//         pair._val = value;
//         return pair;
//     }

//     static StackItem Create() {
//         StackItem pair;
//         pair._key = -1;
//         pair._val = FLT_MAX;
//         return pair;
//     }
// };

// struct Stack<let N : int> {
//     StackItem items[N];

//     [mutating]
//     void clear() {
//         for (int i = 0; i < N; ++i) {
//             items[i] = StackItem::Create();
//         }
//     };

//     [mutating]
//     void insert(StackItem newItem) {
//         if (items[0].value() > newItem.value()) {
//             items[0] = newItem;
//             // [unroll]
//             for (int i = 1; i < N; ++i) {
//                 if (items[i - 1].value() >= items[i].value()) break;
//                 StackItem tmp = items[i - 1]; items[i - 1] = items[i]; items[i] = tmp;
//             }
//         }
//     }

//     static Stack<N> Create<let N : int>() {
//         Stack<N> list;
//         list.clear();
//         return list;
//     };

//     int key(int index) {
//         return items[N - 1 - index].key();
//     };

//     float value(int index) {
//         return items[N - 1 - index].value();
//     };
// };

// Stack<B> intersectAndSortChildren<let B : int> (
//   float3 queryOrigin, inout float closestDistance,
//   int start, int maxClusters,
//   bool useAABBs, in gprt::Buffer aabbs,
//   bool useOOBBs, in gprt::Buffer oobbs, bool debug = false)
// {
//   Stack<B> H;
//   H.clear();

//   for (int child = 0; child < B; ++child) {
//     uint32_t index = start + child;
//     if (index >= maxClusters) {
//       break;
//     }

//     float minDist = 0.f;
//     float maxDist = 1e38f;

//     // Upward Culling: skip superclusters father than current closest primitive
//     if (useAABBs) {
//       float dist = getAABBDist(queryOrigin, aabbs, index);
//       minDist = max(minDist, dist);
//       if (minDist > closestDistance) continue;
//     }

//     if (useOOBBs) {
//       float dist = getOOBBDist(queryOrigin, oobbs, index);
//       minDist = max(minDist, dist);
//       if (minDist > closestDistance) continue;
//     }

//     H.insert(StackItem::Create(index, minDist));
//   }

//   return H;
// }

// void TraverseLeaf<let B : int> (NodeRecord record, inout TraversePayload payload, int index) {
//     int numPrims = record.numPrims;

//     // At this point, it appears that the fastest thing to do is just
//     // let traversal proceed linearly at the leaves to drive upward culling.
//     for (uint32_t primID = 0; primID < B; ++primID) {
//         uint32_t itemID = index * B + primID;
//         if (itemID >= numPrims) continue;

//         float3 p = payload.Origin;

//         float3x4 tri = gprt::load<float3x4>(record.triangleLists, itemID);

//         float3 ba = tri[1].xyz - tri[0].xyz; float3 pa = p - tri[0].xyz;
//         float3 cb = tri[2].xyz - tri[1].xyz; float3 pb = p - tri[1].xyz;
//         float3 ac = tri[0].xyz - tri[2].xyz; float3 pc = p - tri[2].xyz;
//         float3 nor = cross(ba, ac);

//         float distToEdgeA = sqrt(_dot2(ba * clamp(dot(ba, pa) / _dot2(ba), 0.0f, 1.0f) - pa));
//         float distToEdgeB = sqrt(_dot2(cb * clamp(dot(cb, pb) / _dot2(cb), 0.0f, 1.0f) - pb));
//         float distToEdgeC = sqrt(_dot2(ac * clamp(dot(ac, pc) / _dot2(ac), 0.0f, 1.0f) - pc));
        
//         float distToPlane = sqrt(dot(nor, pa) * dot(nor, pa) / _dot2(nor));
//         float distToEdges = min(min(distToEdgeA, distToEdgeB), distToEdgeC);

//         // inside/outside test
//         bool inside = (sign(dot(cross(ba, nor), pa)) +
//                            sign(dot(cross(cb, nor), pb)) +
//                            sign(dot(cross(ac, nor), pc)) < 2.0f);

//         float dist = inside ? distToEdges : distToPlane;

//         // Primitive farther than furthest
//         if (dist > payload.closestDistance) continue;

//         // Newly found closest primitive
//         payload.closestDistance = dist;
//         payload.closestPrimitive = itemID;
//     }
// }

// void TraverseTree<let N : int, let B : int>(NodeRecord record, inout TraversePayload payload, int parentIndex)
// {
//     int start = parentIndex * B;
//     Stack<B> stack = intersectAndSortChildren<B>(payload.Origin, payload.closestDistance,
//                                                  start, record.numClusters[N - 1],
//                                                  true/*useAABBs*/, record.aabbs[N - 1],
//                                                  true/*useOOBBs*/, record.oobbs[N - 1]);
//     for (int i = 0; i < B; ++i) {
//         int childIndex = stack.key(i);
//         float mindist = stack.value(i);
//         if (childIndex == -1 || mindist > payload.closestDistance) return;
//         if (N > 1) TraverseTree<N - 1, B>(record, payload, childIndex);
//         else TraverseLeaf<B>(record, payload, childIndex);
//     }
// }

// [shader("callable")]
// void TraverseNode1(uniform NodeRecord record, inout TraversePayload payload) { TraverseTree<1, BRANCHING_FACTOR>(record, payload, 0);}

// [shader("callable")]
// void TraverseNode2(uniform NodeRecord record, inout TraversePayload payload) { TraverseTree<2, BRANCHING_FACTOR>(record, payload, 0);}

// [shader("callable")]
// void TraverseNode3(uniform NodeRecord record, inout TraversePayload payload) { TraverseTree<3, BRANCHING_FACTOR>(record, payload, 0);}

// [shader("callable")]
// void TraverseNode4(uniform NodeRecord record, inout TraversePayload payload) { TraverseTree<4, BRANCHING_FACTOR>(record, payload, 0);}

// [shader("callable")]
// void TraverseNode5(uniform NodeRecord record, inout TraversePayload payload) { TraverseTree<5, BRANCHING_FACTOR>(record, payload, 0);}

// [shader("callable")]
// void TraverseNode6(uniform NodeRecord record, inout TraversePayload payload) { TraverseTree<6, BRANCHING_FACTOR>(record, payload, 0);}

// [shader("callable")]
// void TraverseNode7(uniform NodeRecord record, inout TraversePayload payload) { TraverseTree<7, BRANCHING_FACTOR>(record, payload, 0);}

// [shader("callable")]
// void TraverseNode8(uniform NodeRecord record, inout TraversePayload payload) { TraverseTree<8, BRANCHING_FACTOR>(record, payload, 0);}

// [shader("callable")]
// void TraverseNode9(uniform NodeRecord record, inout TraversePayload payload) { TraverseTree<9, BRANCHING_FACTOR>(record, payload, 0);}

// [shader("callable")]
// void TraverseNode10(uniform NodeRecord record, inout TraversePayload payload) {TraverseTree<10, BRANCHING_FACTOR>(record, payload, 0);}