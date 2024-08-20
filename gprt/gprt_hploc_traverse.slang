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
        items[stackptr] = item;
        stackptr++;
        return true;
    }

    [mutating]
    T pop() {
        --stackptr;
        return items[stackptr];
    }

    // // Pops an item off the stack (by value)
    // // If the stack is empty, the function returns false.
    // [mutating]
    // bool pop(inout T item) {
    //     if (stackptr == 0) return false;
    //     --stackptr;
    //     item = items[stackptr];        
    //     return true;
    // }
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
        if (stack.stackptr == 0) return false;
        item = stack.pop();
        child0 = int(item >> 32ull);
        child1 = int(item);
        return true;
    }

    uint size() { return stack.size(); }
};

// Function to swap elements based on condition
__inline__ void Swap(inout StackEntry ia, inout StackEntry ib) {
    if (ia.dist > ib.dist) {
        StackEntry tmp = ia;
        ia = ib;
        ib = tmp;
    }
}

// Sorting network for an array of 8 elements
__inline__ void SortWithNetwork(inout StackEntry arr[8]) {
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

struct StackEntry {
    uint index;
    float dist;

    __init(uint _index, float _dist) {
        index = _index;
        dist = _dist;
    }
}

void BubbleSort<let ARRAY_SIZE : int>(inout StackEntry arr[ARRAY_SIZE]) {
    for (int i = 0; i < ARRAY_SIZE - 1; ++i)
    {
        for (int j = 0; j < ARRAY_SIZE - i - 1; j++)
        {
            if (arr[j].dist > arr[j + 1].dist) {
                StackEntry tmp = arr[j];
                arr[j] = arr[j + 1];
                arr[j + 1] = tmp;
            }
        }
    }
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
        uint2 group = stack.pop();
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

BVH8Node fetchNode(RWByteAddressBuffer BVH8N, int node_index) {
    return BVH8N.Load<BVH8Node>(node_index * sizeof(BVH8Node));

    // float4 n[5];
    // n[0] = BVH8N.LoadAligned<float4>(sizeof(float4) * (node_index * 5 + 0));
    // n[1] = BVH8N.LoadAligned<float4>(sizeof(float4) * (node_index * 5 + 1));
    // n[2] = BVH8N.LoadAligned<float4>(sizeof(float4) * (node_index * 5 + 2));
    // n[3] = BVH8N.LoadAligned<float4>(sizeof(float4) * (node_index * 5 + 3));
    // n[4] = BVH8N.LoadAligned<float4>(sizeof(float4) * (node_index * 5 + 4));
    // return reinterpret<BVH8Node>(n);
};

struct StackEntryYlitie
{
    uint32_t firstIdx;
    uint32_t hits;
    uint32_t octant;

    __init(uint32_t firstIdx, uint32_t hits, uint32_t octant) {
        this.firstIdx = firstIdx;
        this.hits = hits;
        this.octant = octant;
    }
};



struct TraversalState {
    uint8_t oct;
    uint dupoct;

    Stack<StackEntry, 8> primGroup;
    Stack<StackEntry, 8> nodeGroup;

    // note, repurposing tmax for search radius, and ignoring tmin.
    float3 origin; float tmin;
    float3 direction; float tmax;

    float2 triUV;

    int enablePostponing;

    // debug data...
    uint debug;
    uint debugHits;
    float debugTime;
    uint showBox;

    __init() {
    }
};

struct TraversalStateYlitie {
    uint8_t oct;
    uint dupoct;

    StackEntryYlitie nodeGroup;
    StackEntryYlitie primGroup;

    // note, repurposing tmax for search radius, and ignoring tmin. 
    float3 origin; float tmin;
    float3 direction; float tmax;
    
    float2 triUV;

    int enablePostponing;

    // debug data...
    uint debug;
    uint debugHits;
    float debugTime;
    uint showBox;

    __init() {
        
    }
};

float3[2] getNodeBounds(BVH8Node node) {
    float3 pos = node.header.pos;
    uint3 packedScale = node.header.getScale();
    float3 scale = asfloat(packedScale << 23);
    
    float3[2] bounds;
    bounds[0] = pos;
    bounds[1] = pos + scale * 255.0f;
    return bounds;

    // // Max. relative error in triangle test. Compensate in node test by increasing bbox size.
    // float watertightNodeEps = (5.0f * exp2(-24.0f));

    // // Min. absolute error introduced by the triangle test.
    // float watertightNodeEpsMin = exp2(-75.f);

    // float3 relMin = pos;
    // float3 relMax = relMin + scale * 255.0f;



    // float coordBound = max(fmaxof3f(abs(relMin)), fmaxof3f(abs(relMax)));
    // float Xerr = __fmaf_rn(watertightNodeEps, coordBound, watertightNodeEpsMin); // (should be fmaf_ru...)

    // float3 Xerr3;
    // Xerr3.x = asfloat(asint(Xerr) | (asint(1.f) & 0x80000000u)); // Pick up sign from invDir
    // Xerr3.y = asfloat(asint(Xerr) | (asint(1.f) & 0x80000000u));
    // Xerr3.z = asfloat(asint(Xerr) | (asint(1.f) & 0x80000000u));
    // float3 anear = relMin - Xerr3;
    // float3 afar = relMin + Xerr3;

    // float2x3 bounds;
    // bounds[0] = anear + relMin;
    // bounds[1] = afar + relMax;
    // return bounds;
}



void intersectNode(BVH8Node node, uint nodeIdx, inout TraversalState tstate)
{
    tstate.nodeGroup = Stack<StackEntry, 8>();
    tstate.primGroup = Stack<StackEntry, 8>();

    [ForceUnroll]
    for (int i = 0; i < 8; ++i) {
        tstate.nodeGroup.items[i].index = -1;
        tstate.nodeGroup.items[i].dist = FLT_MAX;
    }

    float3 pos = node.header.pos;
    uint3 packedScale = node.header.getScale();
    uint32_t hits = 0;

    // Shift 8-bit scale to floating-point exponent bits.
    float3 scale = asfloat(packedScale << 23);
    float3 orig = tstate.origin;
    float3 dir = tstate.direction;

    float3 invDir = rcp(dir);

    // Max. relative error in triangle test. Compensate in node test by increasing bbox size.
    float watertightNodeEps = (5.0f * exp2(-24.0f));

    // Min. absolute error introduced by the triangle test.
    float watertightNodeEpsMin = exp2(-75.f);

    // Compute nudged bbox corners
    float3 relMin = pos - orig;
    float3 relMax = relMin + scale * 255.0f;
    float coordBound = max(fmaxof3f(abs(relMin)), fmaxof3f(abs(relMax)));
    float Xerr = __fmaf_rn(watertightNodeEps, coordBound, watertightNodeEpsMin); // (should be fmaf_ru...)

    float3 Xerr3;
    Xerr3.x = asfloat(asint(Xerr) | (asint(invDir.x) & 0x80000000u)); // Pick up sign from invDir
    Xerr3.y = asfloat(asint(Xerr) | (asint(invDir.y) & 0x80000000u));
    Xerr3.z = asfloat(asint(Xerr) | (asint(invDir.z) & 0x80000000u));
    float3 anear = relMin - Xerr3;
    float3 afar = relMin + Xerr3;

    uint numPrevInner = 0;

    [unroll]
    for (int j = 0; j < 2; j++)
    {
        uint8_t4 lox = extract_bytes32(node.lox[j]);
        uint8_t4 loy = extract_bytes32(node.loy[j]);
        uint8_t4 loz = extract_bytes32(node.loz[j]);
        uint8_t4 hix = extract_bytes32(node.hix[j]);
        uint8_t4 hiy = extract_bytes32(node.hiy[j]);
        uint8_t4 hiz = extract_bytes32(node.hiz[j]);

        // Select tnear, tfar by ray sign for x and y. These replace 2 vmnmx per box.
        uint8_t4 nearx = select(dir.x >= 0.f, lox, hix);
        uint8_t4 farx = select(dir.x >= 0.f, hix, lox);
        uint8_t4 neary = select(dir.y >= 0.f, loy, hiy);
        uint8_t4 fary = select(dir.y >= 0.f, hiy, loy);
        uint8_t4 nearz = select(dir.z >= 0.f, loz, hiz);
        uint8_t4 farz = select(dir.z >= 0.f, hiz, loz);

        [unroll]
        for (int i = 0; i < 4; i++) {
            if (node.header.isEmpty(j * 4 + i)) continue;

            float3 tnear, tfar;
            tnear.x = anear.x + scale.x * float(nearx[i]); 
            tnear.y = anear.y + scale.y * float(neary[i]); 
            tnear.z = anear.z + scale.z * float(nearz[i]); 
            tfar.x = afar.x + scale.x * float(farx[i]);    
            tfar.y = afar.y + scale.y * float(fary[i]);    
            tfar.z = afar.z + scale.z * float(farz[i]);    

            float3 tlo, thi;
            tlo.x = __fmul_rd(tnear.x, invDir.x);
            tlo.y = __fmul_rd(tnear.y, invDir.y);
            tlo.z = __fmul_rd(tnear.z, invDir.z);
            thi.x = __fmul_ru(tfar.x, invDir.x);
            thi.y = __fmul_ru(tfar.y, invDir.y);
            thi.z = __fmul_ru(tfar.z, invDir.z);

            // Normally we would intersect t-ranges here...
            // float tmin = max(max(tlo.x, tstate.tmin), max(tlo.y, tlo.z));
            // float tmax = min(min(thi.x, tstate.tmax), min(thi.y, thi.z));
            // bool intersects = (tmin <= tmax);

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

            // For a closest point query we instead want to solve the following...
            float3 ldist = min(tlo, thi);
            float3 rdist = max(tlo, thi);

            float3 excess = 0.0;
            excess += select(float3(0.0) < ldist, ldist * ldist, float3(0.f));
            excess += select(float3(0.0) > rdist, rdist * rdist, float3(0.f));
            float dist = excess.x + excess.y + excess.z;
            float centerDist = dot2((ldist + rdist) * .5);

            bool intersects = (dist <= tstate.tmax);
            if (intersects) {
                if (node.header.isLeaf(j * 4 + i)) {
                    tstate.primGroup.push(
                        StackEntry(node.header.firstPrimIdx + node.header.getLeafRemapOfs(j * 4 + i), dist));
                }
                else {
                    int relativeIndex = numPrevInner;
                    tstate.nodeGroup.push(StackEntry(node.header.firstNodeIdx + relativeIndex, dist));
                }
            }
            if (node.header.isInner(j * 4 + i)) numPrevInner++;
        }
    }

    // sort node stack
    SortWithNetwork(tstate.nodeGroup.items);
}

[shader("callable")]
void TraverseBVH8(uniform TraverseRecord record, inout TraversePayload travParams) {
    RWByteAddressBuffer BVH8N = buffers[record.BVH8N.index];
    RWByteAddressBuffer BVH8L = buffers[record.BVH8L.index];

    var traversalStack = Stack<Stack<StackEntry, 8>, 32>(); // clear trav stack
    var tstate = TraversalState();

    tstate.primGroup = Stack<StackEntry, 8>();
    tstate.nodeGroup = Stack<StackEntry, 8>();
    tstate.nodeGroup.push(StackEntry(0, FLT_MAX));
    traversalStack.push(tstate.nodeGroup);

    tstate.origin = travParams.OriginAndTMax.xyz;
    tstate.direction = float3(1.0, 1.0, 1.0);

    // Note, tmax is squared to avoid square root operations for computing closest distances
    tstate.tmax = travParams.OriginAndTMax.w * travParams.OriginAndTMax.w;

    // Debug data...
    tstate.debug = bool(travParams.QueryFlags & NN_FLAG_DEBUG);
    tstate.debugHits = 0;
    tstate.debugTime = travParams.time;
    tstate.showBox = bool(int(tstate.debugTime * 2.f) % 2);

    do {
        // Handle leaves
        while (tstate.primGroup.size() > 0) {
            uint idx = tstate.primGroup.pop().index;

            tstate.debugHits++;

            BVH8Triangle tri = BVH8L.Load<BVH8Triangle>(idx * sizeof(BVH8Triangle));

            float3 p; float2 bc;
            float dist = findClosestPointTriangleSquared(tri.v0, tri.v1, tri.v2, tstate.origin, p, bc);

            if (dist < tstate.tmax) {
                tstate.tmax = dist;
                // Update the nearest neighbor here
            }
        }

        // if we have inner node hits
        if (tstate.nodeGroup.size() > 0)
        {
            StackEntry item = tstate.nodeGroup.pop();
            uint idx = item.index;
            float dist = item.dist;
            if (tstate.nodeGroup.size() > 0)
                traversalStack.push(tstate.nodeGroup);

            BVH8Node node = fetchNode(BVH8N, idx);
            intersectNode(node, idx, tstate);
        }

        // Stack pop?
        if (tstate.nodeGroup.size() == 0)
        {
            // Empty stack -> done
            if (traversalStack.stackptr <= 0) break;

            // Pop the stack
            tstate.nodeGroup = traversalStack.pop();
        }
    } while (traversalStack.stackptr > 0);

    // if (tstate.debug) printf("num prim hits %d\n", tstate.debugHits);

    travParams.hitPrims = tstate.debugHits;
    travParams.OriginAndTMax.w = sqrt(tstate.tmax);
}

void intersectNodeYlitie(BVH8Node node, uint nodeIdx, inout TraversalStateYlitie tstate)
{
    float3 pos = node.header.pos;
    uint3 packedScale = node.header.getScale();
    tstate.nodeGroup.firstIdx = node.header.firstNodeIdx;
    tstate.primGroup.firstIdx = node.header.firstPrimIdx;
    uint32_t hits = 0;

    // Shift 8-bit scale to floating-point exponent bits.
    float3 scale = asfloat(packedScale << 23);
    float3 orig = tstate.origin;
    float3 dir = tstate.direction;

    float3 invDir = rcp(dir);

    // Max. relative error in triangle test. Compensate in node test by increasing bbox size.
    float watertightNodeEps = (5.0f * exp2(-24.0f));

    // Min. absolute error introduced by the triangle test.
    float watertightNodeEpsMin = exp2(-75.f);

    // Compute nudged bbox corners
    float3 relMin = pos - orig;
    float3 relMax = relMin + scale * 255.0f;
    float coordBound = max(fmaxof3f(abs(relMin)), fmaxof3f(abs(relMax)));
    float Xerr = __fmaf_rn(watertightNodeEps, coordBound, watertightNodeEpsMin); // (should be fmaf_ru...)

    float3 Xerr3;
    Xerr3.x = asfloat(asint(Xerr) | (asint(invDir.x) & 0x80000000u)); // Pick up sign from invDir
    Xerr3.y = asfloat(asint(Xerr) | (asint(invDir.y) & 0x80000000u));
    Xerr3.z = asfloat(asint(Xerr) | (asint(invDir.z) & 0x80000000u));
    float3 anear = relMin - Xerr3;
    float3 afar = relMin + Xerr3;

    // Todo, run some tests in this loop to verify that the AABBs are entirely inside the parent, and that none are degenerate...
    int debugNodeHits[8];
    int debugPrimHits[8];
    int numNodes = 0;
    int numLeaves = 0;
    int numEmpty = 0;
    //[ForceUnroll]
    for (int i = 0; i < 8; i++) {
        debugNodeHits[i] = -1;
        debugPrimHits[i] = -1;

        if (node.header.isLeaf(i)) numLeaves++;
    }

    //[ForceUnroll]
    for (int j = 0; j < 2; j++)
    {
        uint8_t4 lox = extract_bytes32(node.lox[j]);
        uint8_t4 loy = extract_bytes32(node.loy[j]);
        uint8_t4 loz = extract_bytes32(node.loz[j]);
        uint8_t4 hix = extract_bytes32(node.hix[j]);
        uint8_t4 hiy = extract_bytes32(node.hiy[j]);
        uint8_t4 hiz = extract_bytes32(node.hiz[j]);

        // Select tnear, tfar by ray sign for x and y. These replace 2 vmnmx per box.
        uint8_t4 nearx = select(dir.x >= 0.f, lox, hix);
        uint8_t4 farx  = select(dir.x >= 0.f, hix, lox);
        uint8_t4 neary = select(dir.y >= 0.f, loy, hiy);
        uint8_t4 fary  = select(dir.y >= 0.f, hiy, loy);
        uint8_t4 nearz = select(dir.z >= 0.f, loz, hiz);
        uint8_t4 farz  = select(dir.z >= 0.f, hiz, loz);

        // Leaves insert numtris bits to dynamic hitmask; inner nodes insert 1 and empty nodes 0.
        uint32_t ofs = node.header.meta[j];
        uint32_t countsMask = (ofs >> 5) & 0x07070707; // top 3 bits counting items in each slot. For inner its 1, for tri leaves its 1-3 (unary). empty nodes is 0.
        uint32_t innermask = (ofs & (ofs << 1));    // all inner nodes have bit 5,4 (16,8). Leaves may have one set but not both.
        innermask = vsignExtend4(innermask << 3);   // sbyte sign marks inner nodes, extend sign to all bits.
        ofs ^= (tstate.dupoct & innermask); // compute traversal priority for inner node children only.

        

        // const uint octinv4 = (7 - tstate.oct) * 0x01010101u;
        // const uint meta4 = node.header.meta[j];
        // const uint is_inner4 = (meta4 & (meta4 << 1)) & 0x10101010;
        // const uint inner_mask4 = sign_extend_s8x4(is_inner4 << 3);
        // const uint bit_index4 = (meta4 ^ (octinv4 & inner_mask4)) & 0x1F1F1F1F;
        // const uint child_bits4 = (meta4 >> 5) & 0x07070707;

        //[ForceUnroll]
        for (int i = 0; i < 4; i++) {
            if (node.header.isEmpty(j * 4 + i)) continue;

            float3 tnear, tfar;
            tnear.x = anear.x + scale.x * float(nearx[i]); //bfe(nearx, (i % 4) * 8, 8);
            tnear.y = anear.y + scale.y * float(neary[i]); //bfe(neary, (i % 4) * 8, 8);
            tnear.z = anear.z + scale.z * float(nearz[i]); //bfe(nearz, (i % 4) * 8, 8);
            tfar.x  = afar.x  + scale.x * float(farx[i]);  //bfe(farx, (i % 4) * 8, 8);
            tfar.y  = afar.y  + scale.y * float(fary[i]);  //bfe(fary, (i % 4) * 8, 8);
            tfar.z  = afar.z  + scale.z * float(farz[i]);  //bfe(farz, (i % 4) * 8, 8);

            float3 tlo, thi;
            tlo.x = __fmul_rd(tnear.x, invDir.x);
            tlo.y = __fmul_rd(tnear.y, invDir.y);
            tlo.z = __fmul_rd(tnear.z, invDir.z);
            thi.x = __fmul_ru(tfar.x, invDir.x);
            thi.y = __fmul_ru(tfar.y, invDir.y);
            thi.z = __fmul_ru(tfar.z, invDir.z);

            // Normally we would intersect t-ranges here...
            // float tmin = max(max(tlo.x, tstate.tmin), max(tlo.y, tlo.z));
            // float tmax = min(min(thi.x, tstate.tmax), min(thi.y, thi.z));
            // bool intersects = (tmin <= tmax);

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

            // For a closest point query we instead want to solve the following...
            float3 ldist = min(tlo, thi);
            float3 rdist = max(tlo, thi);

            float3 excess = 0.0;
            excess += select(float3(0.0) < ldist, ldist * ldist, float3(0.f));
            excess += select(float3(0.0) > rdist, rdist * rdist, float3(0.f));
            float dist = excess.x + excess.y + excess.z;

            // Intersection is true if distance is less than the current closest distance.
            // There seems to be some bug with intersection tests for inner nodes... leaves appear to work fine...
            // Perhaps something to do with octant reordering, but I'm not sure why...
            bool intersects = (dist * (.5 + .5 * sin(tstate.debugTime)) <= tstate.tmax);// || node.header.isLeaf(j * 4 + i);

            if (intersects ) {
                // if (node.header.isLeaf(j * 4 + i)) {
                //     if (node.header.getLeafRemapOfs(j * 4 + i) >= 8) {
                //         printf("Error, relative leaf addr out of range!\n");
                //     }
                //     debugPrimHits[node.header.getLeafRemapOfs(j * 4 + i)] = 1;
                // }
                // else {
                //     debugNodeHits[j * 4 + i] = j * 4 + i;
                // }

                const uint top3countbits = extract_byte32(countsMask, i) & 0b111;
                // for leaves, ofs is the inner node mask. for nodes, it's slot index ^ oct
                const uint offset = extract_byte32(ofs, i) & 0b11111;
                hits |= (top3countbits << offset);
            }
        }
    }

    // Extract inner, leaf node hits
    let leafHits = hits & 0b00000000000000000000000011111111;
    let innerHits = (hits & 0b11111111000000000000000000000000) | node.header.getIMask(); // Move valid inner node mask to low byte.
    tstate.nodeGroup.hits = innerHits;
    tstate.primGroup.hits = leafHits;

    // Test to confirm that stack entries are correct
    // if (tstate.debug) {
    //     StackEntry nodeGroup = tstate.nodeGroup;
    //     StackEntry primGroup = tstate.primGroup;
    //     // printf("--------------\n");
    //     // printf("Num nodes %d numLeaves %d numEmpty %d\n", numNodes, numLeaves, numEmpty);

    //     // for (int i = 0; i < 8; ++i) {
    //     //     if (debugNodeHits[i] != -1 && debugNodeHits[i] != i) {
    //     //         printf("Debug node hits %d has incorrectly defined value %d\n", i, debugNodeHits[i]);
    //     //     }
    //     // }

    //     while (nodeGroup.hits > 0x00FFFFFF)
    //     {
    //         int bitIdx = firstbithigh(nodeGroup.hits);

    //         nodeGroup.hits &= nodeGroup.hits & ~(1 << bitIdx);

    //         // Find index of the child slot by reversing traversal priority computation and removing bias of 24.
    //         uint slotIdx = (bitIdx ^ tstate.dupoct) & 0x7;
    //         // int slotIdx = (bitIdx - 24) ^ (7 - tstate.oct);

    //         if (debugNodeHits[slotIdx] != slotIdx) {
    //             printf("Error, node %d recovered slot Idx %d but should be %d!\n", nodeIdx, slotIdx, debugNodeHits[slotIdx]);
    //         }

    //         int bruteForceRelativeIdx = 0;
    //         uint8_t imask = extract_byte32(nodeGroup.hits, 0);
    //         for (int i = 0; i < slotIdx; ++i) {
    //             if ((imask & (1 << i)) != 0) 
    //                 bruteForceRelativeIdx++;
    //         }

    //         // Compute number of internal siblings nodes in memory before this node using imask in low byte of nodeGroup.hits.
    //         int relativeIdx = __popc(nodeGroup.hits & ~(0xFFFFFFFF << slotIdx));
    //         // int relativeIdx = __popc(tstate.nodeGroup.hits >> slotIdx);

    //         if (relativeIdx != bruteForceRelativeIdx) {
    //             printf("Error, node %d slot %d recovered relative index %d but actual relative index should be %d. Imask %d\n", nodeIdx, slotIdx, relativeIdx, bruteForceRelativeIdx, imask);
    //         }

    //         // else {
    //         //     printf("Correct, node %d slot %d recovered relative index %d and actual relative index should be %d. Imask %d\n", nodeIdx, slotIdx, relativeIdx, bruteForceRelativeIdx, imask);
    //         // }      
    //     }

    //     int foundPrims = 0;

    //     while (primGroup.hits != 0) {
    //         foundPrims++;
    //         uint idx = firstbithigh(primGroup.hits);
    //         primGroup.hits &= ~(1 << idx); // clear the prim from the hits list

    //         if (debugPrimHits[idx] != 1) {
    //             printf("Error, recovered relative index %d but leaf not hit!\n", idx);
    //         }

    //         idx += primGroup.firstIdx;
    //         // ... potentially more testing here...
    //     }

    //     if (foundPrims != numLeaves) {
    //         printf("Error, not all primitives found! expected %d, actual %d\n", numLeaves, foundPrims);
    //     }
    // }
}

[shader("callable")]
void TraverseBVH8Ylitie(uniform TraverseRecord record, inout TraversePayload travParams) {
    RWByteAddressBuffer BVH8N = buffers[record.BVH8N.index];
    RWByteAddressBuffer BVH8L = buffers[record.BVH8L.index];

    var traversalStack = Stack<StackEntryYlitie, 32>(); // clear trav stack
    var tstate = TraversalStateYlitie();

    tstate.nodeGroup = StackEntryYlitie(0, 0, 0);
    tstate.primGroup = StackEntryYlitie(0, 0, 0);

    tstate.enablePostponing = PRIMITIVE_POSTPONING;

    tstate.origin = travParams.OriginAndTMax.xyz;
    tstate.direction = float3(1.0, 1.0, 1.0);
    
    // Note, tmax is squared to avoid square root operations for computing closest distances
    tstate.tmax = travParams.OriginAndTMax.w * travParams.OriginAndTMax.w;

    // Debug data...
    tstate.debug = bool(travParams.QueryFlags & NN_FLAG_DEBUG);
    tstate.debugHits = 0;
    tstate.debugTime = travParams.time;
    tstate.showBox = bool(int(tstate.debugTime * 2.f) % 2);

    tstate.oct = getOctant(tstate.direction);
    tstate.dupoct = duplicateByte(tstate.oct);

    float tfar = 10.f;

    // Intersect root node
    {
        BVH8Node node = fetchNode(BVH8N, 0);
        intersectNodeYlitie(node, 0, tstate);
    }

    bool firstLeaf = true;
    uint lastNode = 0;

    uint numHits = 0;
    uint maxNumHits = 1024;//int(tstate.debugTime) % 10;

    do {
        // Handle leaves
        // Seems like this is giving me undefined behavior unfortunately...
        // const int numTraversing = WaveActiveCountBits(true);
        if (tstate.primGroup.hits != 0)
        {
            bool bottomLevel = true;

            if (bottomLevel) {
                while (tstate.primGroup.hits != 0) {
                    uint idx = firstbithigh(tstate.primGroup.hits);
                    tstate.primGroup.hits &= (~(1 << idx)); // clear the prim from the hits list
                    idx += tstate.primGroup.firstIdx;


                    // if (tstate.enablePostponing == 1 && 
                    //     WaveActiveCountBits(true) * 4 < numTraversing) // Is SIMD utilization too low?
                    //         break;                                     // Postpone primitive testing

                    // if (tstate.debug && tstate.primGroup.firstIdx != node.header.firstPrimIdx) {
                    //     printf("Error, mismatching base prim index!");
                    // }

                    tstate.debugHits++;

                    // if (tstate.debug && firstLeaf) {
                    //     printf("Leaf hits 0x%x\n", tstate.primGroup.hits);
                    // }

                    // Select triangle
                    
                    // if (tstate.debug && firstLeaf) {
                    //     printf("Rel idx is %d, base idx is %d parentNode should be %d and is %d\n", idx, tstate.primGroup.firstIdx, t);
                    // }



                    BVH8Triangle tri = BVH8L.Load<BVH8Triangle>(idx * sizeof(BVH8Triangle));

                    

                    BVH8Node node = fetchNode(BVH8N, tri.debug);
                    float3[2] parentBounds = getNodeBounds(node);

                    // float dist = computeSquaredMinDistanceToAABB(tstate.origin, parentBounds[0], parentBounds[1]);


                    if (tstate.debug == 1) {
                        float3[2] triaabb;
                        triaabb[0] = min(min(tri.v0, tri.v1), tri.v2);
                        triaabb[1] = max(max(tri.v0, tri.v1), tri.v2);

                        if (any(triaabb[0] < parentBounds[0]) || any(triaabb[1] > parentBounds[1]))
                            printf("Error, tri outside parent bounds\n");


                        // for (int i = 0; i < traversalStack.stackptr; ++i) {
                        //     uint parentNodeIdx = traversalStack.debugItems[i];
                        //     BVH8Node ancestor = fetchNode(BVH8N, parentNodeIdx);


                        // }


                        // printf("Expected parent %d and is %d\n", tri.debug, traversalStack.debugItems[traversalStack.stackptr - 1]);
                    }

                    float3 p; float2 bc;
                    float dist = findClosestPointTriangleSquared(tri.v0, tri.v1, tri.v2, tstate.origin, p, bc);
                    
                    if (dist < tstate.tmax) {
                        // if (tstate.debug) 
                        //     printf("New closest, %d, dist %f, debug %d\n", tri.userTriangleID, dist, tri.debug);
                        tstate.tmax = dist;
                        // Update the nearest neighbor here
                        numHits++;

                        if (numHits > maxNumHits) break;
                    }

                }
                if (numHits > maxNumHits) break;

                // If there are more prims to test, push back to stack
                // if (tstate.primGroup.hits != 0)
                //     traversalStack.push(tstate.primGroup);

                firstLeaf = false;
            }
            
            else // todo, top-level leaf transition
            {
            }
        }

        // Stack pop?
        if (tstate.nodeGroup.hits <= 0x00FFFFFF)
        {
            // Empty stack -> done
            if (traversalStack.stackptr <= 0) break;
            
            // Todo, handle any transitions here back to a top level traveral...
            // if (traversalStack.stackptr == BLASEntryPoint) ...

            // Pop the stack
            tstate.nodeGroup = traversalStack.pop();
        }

        // if we have inner node hits
        if (tstate.nodeGroup.hits > 0x00FFFFFF)
        {
            int bitIdx = firstbithigh(tstate.nodeGroup.hits);
            tstate.nodeGroup.hits &= tstate.nodeGroup.hits & ~(1 << bitIdx);

            // Find index of the child slot by reversing traversal priority computation and removing bias of 24.
            uint slotIdx = (bitIdx ^ tstate.dupoct) & 0x7;
            // int slotIdx = (bitIdx - 24) ^ (7 - tstate.oct);

            // Compute number of internal siblings nodes in memory before this node using imask in low byte of nodeGroup.hits.
            int relativeIdx = __popc(tstate.nodeGroup.hits & ~(0xFFFFFFFF << slotIdx));

            // int bruteForceRelativeIdx = 0;
            // uint8_t imask = extract_byte32(tstate.nodeGroup.hits, 0);
            // for (int i = 0; i < slotIdx; ++i) {
            //     if ((imask & (1 << i)) != 0)
            //         bruteForceRelativeIdx++;
            // }
            // int relativeIdx = __popc(tstate.nodeGroup.hits >> slotIdx);

            BVH8Node node = fetchNode(BVH8N, tstate.nodeGroup.firstIdx + relativeIdx);

            // If nodeGroup still contains hits to test, push it to stack.
            if (tstate.nodeGroup.hits > 0x00FFFFFF) {
                traversalStack.push(tstate.nodeGroup);
            }
            
            intersectNodeYlitie(node, tstate.nodeGroup.firstIdx + relativeIdx, tstate);
        }
        
        // Only leaf node hits
        else
        {
            tstate.primGroup = tstate.nodeGroup;
            tstate.nodeGroup = StackEntryYlitie(0, 0, 0);
        }

        #if DYNAMIC_FETCH
        // todo...
        #endif
    } while(true);

    // if (tstate.debug) printf("num prim hits %d\n", tstate.debugHits);

    travParams.hitPrims = tstate.debugHits;
    travParams.OriginAndTMax.w = sqrt(tstate.tmax);
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
