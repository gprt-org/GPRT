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

#include "gprt_knn_shared.h"

#include "svd.slangh"

// space filling curve routines
#include "sfc.slang"

float dot2(float3 v) {return dot(v, v);}

float3x3 outerProduct(float3 a, float3 b) {
    return float3x3(a * b.x, a * b.y, a * b.z);
}

// outer product of a vector with itself
float3x3 outer2(float3 a) {
    return outerProduct(a, a);
}

int getNumNodesInLevel(int level, int numPrimitives) {
    int numNodes = numPrimitives;
    for (int i = 0; i <= level; ++i) {
        numNodes = (numNodes + (BRANCHING_FACTOR - 1)) / BRANCHING_FACTOR;
    }
    return numNodes;
}

/**
 * @brief Returns the number of primitives in a given node in our "complete" tree
 * @param level What level is the node in the tree? 0 is the first level, 1 is the second, etc
 * @param index What node are we asking about relative to the given level?
 * @param numPrimitives How many primitives in total are there in the tree?
 */
int getPrimsInNode(
    int level,
    int index,
    int numPrimitives
) {
    int numNodesInLevel = getNumNodesInLevel(level, numPrimitives);
    // Theoretical maximum primitives in a node at this level
    int maxPrimsInLevel = int(pow(BRANCHING_FACTOR, level + 1));
    // Account for when primitive counts don't exactly match a multiple of the branching factor
    return (index == (numNodesInLevel - 1)) ? (numPrimitives % maxPrimsInLevel) : maxPrimsInLevel;
}

/**
 * @brief Returns the parent node ID for the given primitive and parent level
 * @param level What level is the parent node in the tree? 0 is the first level, 1 is the second, etc
 * @param index What primitive are we asking about?
 */
int getPrimParentNode(
    int level,
    int primIndex
) {
    int parentIndex = primIndex;
    for (int i = 0; i <= level; ++i) {
        parentIndex = parentIndex / BRANCHING_FACTOR;
    }
    return parentIndex;
}

[shader("compute")]
[numthreads(32, 1, 1)]
void ComputeTriangleRootBounds(uint3 DispatchThreadID: SV_DispatchThreadID, uniform ComputeTriangleRootBoundsParams params) 
{
    int primID = DispatchThreadID.x;
    if (primID >= params.numPrims) return;

    uint3 tri = gprt::load<uint3>(params.triangles, primID);
    float3 p1, p2, p3;
    p1 = gprt::load<float3>(params.vertices, tri.x);
    if (isnan(p1.x)) return;
    p2 = gprt::load<float3>(params.vertices, tri.y);
    p3 = gprt::load<float3>(params.vertices, tri.z);
    float3 c = (p1 + p2 + p3) / 3.f;

    // temporarily cache the center point here for computing codes later
    gprt::store<float3>(params.centers, primID, c);

    float3 aabbMin = min(p1, min(p2, p3));
    float3 aabbMax = max(p1, max(p2, p3));

    // Compute world bounding box
    gprt::atomicMin32f(params.rootBounds, 0, aabbMin);
    gprt::atomicMax32f(params.rootBounds, 1, aabbMax);
}

[shader("compute")]
[numthreads(32, 1, 1)]
void ComputeTriangleCodes(uint3 DispatchThreadID: SV_DispatchThreadID, uniform ComputeTriangleCodesParams params)
{
    int primID = DispatchThreadID.x;
    if (primID >= params.numPrims) return;

    // Transform center into 0-1 range
    float3 c = gprt::load<float3>(params.centers, primID);
    float3 aabbMin = gprt::load<float3>(params.rootBounds, 0);
    float3 aabbMax = gprt::load<float3>(params.rootBounds, 1);
    c = (c - aabbMin) / (aabbMax - aabbMin);
    uint32_t code = hilbert_encode(c);
    gprt::store<uint64_t>(params.hilbertCodes, primID, uint64_t(code));
    gprt::store<uint64_t>(params.primIDs, primID, primID);
}

[shader("compute")]
[numthreads(32, 1, 1)]
void ExpandTriangles(uint3 DispatchThreadID: SV_DispatchThreadID, uniform ExpandTrianglesParams params)
{
    int primID = DispatchThreadID.x;
    if (primID >= params.numPrims) return;
    uint32_t primAddr = uint32_t(gprt::load<uint64_t>(params.primIDs, primID));
    // load unsorted address
    int3 indices = gprt::load<int3>(params.triangles, primAddr);
    float3x4 tri;
    tri[0] = float4(gprt::load<float3>(params.vertices, indices.x), gprt::load<uint8_t>(params.vertCounts, indices.x));
    tri[1] = float4(gprt::load<float3>(params.vertices, indices.y), gprt::load<uint8_t>(params.vertCounts, indices.y));
    tri[2] = float4(gprt::load<float3>(params.vertices, indices.z), gprt::load<uint8_t>(params.vertCounts, indices.z));
    gprt::store<float3x4>(params.trianglesExp, primID, tri);
}

[shader("compute")]
[numthreads(32, 1, 1)]
void ComputeTriangleAABBsAndCenters(uint3 DispatchThreadID: SV_DispatchThreadID, uniform ComputeTriangleAABBsAndCentersParams params) 
{
    int NID = DispatchThreadID.x;
    int numNClusters = getNumNodesInLevel(params.level, params.numPrims);
    if (NID >= numNClusters) return;

    float3 aabb[2] = { float3(1e38f, 1e38f, 1e38f), -float3(1e38f, 1e38f, 1e38f) };
    float3 center = float3(0, 0, 0);
    int total = 0;    
    for (uint32_t pid = 0; pid < BRANCHING_FACTOR; ++pid) {
        uint32_t primID = NID * BRANCHING_FACTOR + pid;
        if (primID >= params.numPrims) continue;
        float3x4 tri = gprt::load<float3x4>(params.trianglesExp, primID);
        // if (any(isnan(tri))) continue; // slang oddly generates invalid boolean mat type here...
        if (any(isnan(tri[0])) || any(isnan(tri[1])) || any(isnan(tri[2]))) continue;
        for (int i = 0; i < 3; ++i) {
            aabb[0] = min(aabb[0], tri[i].xyz);
            aabb[1] = max(aabb[1], tri[i].xyz);
            center += tri[i].xyz;
            total += 1;
        }
    }
    center /= total;

    // Store AABB
    gprt::store<float3[2]>(params.aabbs, NID, aabb);

    // // find the vertex closest to center
    // float3 closest = float3(1e38f, 1e38f, 1e38f);
    // float closestDist = 1e38f;
    // for (uint32_t pid = 0; pid < BRANCHING_FACTOR; ++pid) {
    //     uint32_t primID = NID * BRANCHING_FACTOR + pid;
    //     if (primID >= params.numPrims) continue;
    //     float3 a, b, c;
    //     a = gprt::load<float4>(params.trianglesExp, primID * 3 + 0).xyz;
    //     if (isnan(a.x)) continue;
    //     b = gprt::load<float4>(params.trianglesExp, primID * 3 + 1).xyz;
    //     c = gprt::load<float4>(params.trianglesExp, primID * 3 + 2).xyz;
    //     float3 closestA = abs(a - center);
    //     float3 closestB = abs(b - center);
    //     float3 closestC = abs(c - center);
    //     float distA = dot(closestA, closestA);
    //     float distB = dot(closestB, closestB);
    //     float distC = dot(closestC, closestC);
    //     if (distA < closestDist) { closestDist = distA; closest = a; }
    //     if (distB < closestDist) { closestDist = distB; closest = b; }
    //     if (distC < closestDist) { closestDist = distC; closest = c; }
    // }

    // Store center of vertices
    gprt::store<float3>(params.centers, NID, center);
}

bool isInvalid(float3 v0, float3 v1, float3 v2) {
    return any(isnan(v0)) || any(isnan(v1)) || any(isnan(v2));
}

bool isDegenerate(float3 v0, float3 v1, float3 v2) {
    float3 v0v1 = v1 - v0;
    float3 v0v2 = v2 - v0;
    float3 crossProd = cross(v0v1, v0v2);
    return length(crossProd) < 1e-9f;
}

// index of highest bit or -1 if none set
int findLeadingOne(uint v) {
    return (v == 0) ? -1 : firstbithigh(v);
}

uint64_t WaveMaskBitXor64(uint64_t val, int laneMask) {
    int lo = int(val);
    int hi = int(val >> 32ull);
    hi = WaveMaskBitXor(hi, laneMask);
    lo = WaveMaskBitXor(lo, laneMask);
    
    uint64_t result = (uint64_t(hi) << 32ull) | uint64_t(lo);
    return result;
}

float ud2float_rd(uint32_t value) {
    // taking an unsigned integer and rounding the converted float down
    return float(value);
}

float ull2float_ru(uint64_t value) {
    float result = float(value & 0xFFFFFFFF);     // Convert the lower 32 bits
    result += float(value >> 32) * 4294967296.0f; // Convert the upper 32 bits and scale back up

    // Since we want to ensure that the float is not less than the original uint64, we check and adjust
    if (uint64_t(result) < value) {
        result = nextafter(result, FLT_MAX);
    }

    return result;
}

[shader("compute")]
[numthreads(32, 1, 1)]
void MortonTriangleSplitterInit(uint3 DispatchThreadID: SV_DispatchThreadID, uniform MortonTriangleSplitterParams p)
{
    uint64_t priority = 0;

    uint32_t primIdx = DispatchThreadID.x;

    // If out of bounds, initiate split primitive indices as invalid and then terminate
    if (primIdx >= p.maxInputTris) {
        if (primIdx < p.maxOutputPrims) {
            gprt::store<int>(p.outPrimIndices, -1);
        }
        return;
    }

    // Init outPrimIndices buffer to hold the indices of the primitives that will be split
    gprt::store<int>(p.outPrimIndices, primIdx, primIdx);
    if (p.splitFactor == 0.f) return; // done here if not splitting

    // Fetch triangle
    uint3 tri = gprt::load<uint3>(p.triangles, primIdx);
    float3 v0, v1, v2;
    v0 = gprt::load<float3>(p.vertices, tri.x);
    v1 = gprt::load<float3>(p.vertices, tri.y);
    v2 = gprt::load<float3>(p.vertices, tri.z);
    
    // Dont split degenerate tris or tris with NaN vertex values
    if (!isDegenerate(v0, v1, v2) && !isInvalid(v0, v1, v2)) {
        float3 lo = min(min(v0, v1), v2);
        float3 hi = max(max(v0, v1), v2);
        float3 di = hi - lo;

        // Compute transform from world space model AABB to 20 bit signed unorm.
        float3 rlo = gprt::load<float3>(p.rootBounds, 0);
        float3 rhi = gprt::load<float3>(p.rootBounds, 1);
        float3 rdiag = rhi - rlo;
        float longestAxis = max(rdiag.x, max(rdiag.y, rdiag.z));
        float scale = float(1 << 20) / longestAxis;
        float3 origin = float3(1 << 19) - (rlo + rhi) * (scale * 0.5f);

        // Find most important median split plane and its importance
        float epsA = 1.0 - p.splitEps;
        float epsB = p.splitEps;
        int3 ilo = int3((lo * epsA + hi * epsB) * scale + origin);
        int3 ihi = max(int3((lo * epsB + hi * epsA) * scale + origin) - 1, ilo);
        int importance = findLeadingOne((ilo.x ^ ihi.x) | (ilo.y ^ ihi.y) | (ilo.z ^ ihi.z));

        // Calculate ideal surface area assuming infinite splits
        // (equation 5 from the paper)
        float3 d1 = v1 - v0, d2 = v2 - v0;
        float3 n = abs(cross(d1, d2));
        float idealArea = n.x + n.y + n.z;

        // Calculate the actual surface area of the aabb
        float actualArea = 2.f * (di.x * di.y + di.x * di.z + di.y * di.z);

        // Evaluate split priority (equation 6 from the paper)
        float areaPriority = log2((actualArea - idealArea) / (6.0 * longestAxis * longestAxis));
        float splitPriority = p.Log2X * float(importance) - p.Log2X;
        float fpriority = exp2((splitPriority + areaPriority) * p.Y);

        priority = uint64_t(float(0xFFFFFFFFu) * saturate(fpriority));
    }
    
    // Store priority
    gprt::store<uint64_t>(p.outPriorities, primIdx, priority);

    // Accumulate total priority
    priority = WaveActiveSum(priority);

    if (WaveGetLaneIndex() == 0)
        gprt::atomicAdd(p.totalPriority, 0, priority);
}

// Just run this with one thread
[shader("compute")]
[numthreads(1, 1, 1)]
void MortonTriangleSplitterUpdateState(uint3 DispatchThreadID: SV_DispatchThreadID, uniform MortonTriangleSplitterParams p)
{
    // Current iterator state
    MortonTriangleSplitterConstants c = gprt::load<MortonTriangleSplitterConstants>(p.globalConstants, 0);

    // Before first iteration we'll set curScaleInterval to a lower bound that's calculated by the totalPriority.
    int maxPrimsAfterSplit = p.maxInputTris + int(p.maxInputTris * p.splitFactor);
    maxPrimsAfterSplit = min(maxPrimsAfterSplit, p.maxOutputPrims);
    int maxSplits = maxPrimsAfterSplit - p.maxInputTris;

    if (c.curScaleFactor == 0.0) {
        float lowerBound = (c.totalPriority == 0ull) ? 0 : floor(max(nextafter(float(maxSplits), -FLT_MAX), 0.0f) / nextafter(float(c.totalPriority), FLT_MAX));
        c.curScaleFactor = lowerBound;

        // Update iterator state
        gprt::store<MortonTriangleSplitterConstants>(p.globalConstants, 0, c);
        return;
    }
    
        
    // After first round, get tight lower and upper bounds
    if (c.curScaleFactor == 0.0) {
        float lowerBound = c.curScaleInterval;
        int lowerSplits = c.totalSplits1;
        if (c.totalSplits3 <= maxSplits) {
            lowerBound = c.curScaleInterval * 3.0;
            lowerSplits = c.totalSplits3;
        }
        else if (c.totalSplits2 <= maxSplits) {
            lowerBound = c.curScaleInterval * 2.0;
            lowerSplits = c.totalSplits2;
        }

        float upperBound = lowerBound * max(1.0, float(maxSplits) / float(lowerSplits));
        if (c.totalSplits2 > maxSplits) {
            upperBound = min(upperBound, c.curScaleInterval * 2.0);
        }
        else if (c.totalSplits3 > maxSplits) {
            upperBound = min(upperBound, c.curScaleInterval * 3.0);
        }

        c.curScaleFactor = lowerBound;
        c.curScaleInterval = (upperBound - lowerBound) * .25;

        // clear total splits for next round
        c.totalSplits1 = 0;
        c.totalSplits2 = 0;
        c.totalSplits3 = 0;

        // Update iterator state
        gprt::store<MortonTriangleSplitterConstants>(p.globalConstants, 0, c);
        return;
    }
    
    // After remaining rounds we chose the correct subinterval and keep iterating
    if (c.totalSplits3 <= maxSplits)
        c.curScaleFactor = c.curScaleFactor + c.curScaleInterval * 3.0;
    else if (c.totalSplits2 <= maxSplits)
        c.curScaleFactor = c.curScaleFactor + c.curScaleInterval * 2.0;
    else
        c.curScaleFactor = c.curScaleFactor + c.curScaleInterval * 1.0;
    c.curScaleInterval = c.curScaleInterval * 0.25;

    // clear total splits for next round
    c.totalSplits1 = 0;
    c.totalSplits2 = 0;
    c.totalSplits3 = 0;

    // Update iterator state
    gprt::store<MortonTriangleSplitterConstants>(p.globalConstants, 0, c);
}

[shader("compute")]
[numthreads(32, 1, 1)]
void MortonTriangleSplitterIter(uint3 DispatchThreadID: SV_DispatchThreadID, uniform MortonTriangleSplitterParams params)
{
}

[shader("compute")]
[numthreads(32, 1, 1)]
void MortonTriangleSplitter(uint3 DispatchThreadID: SV_DispatchThreadID, uniform MortonTriangleSplitterParams params)
{    

}

[shader("compute")]
[numthreads(32, 1, 1)]
void ComputeAABBsAndCenters(uint3 DispatchThreadID: SV_DispatchThreadID, uniform ComputeAABBsAndCentersParams params)
{
    int PID = DispatchThreadID.x;
    int numParents = getNumNodesInLevel(params.level, params.numPrims);
    int numChildren = getNumNodesInLevel(params.level-1, params.numPrims);
    if (PID >= numParents) return;
    float3 nAabb[2] = { float3(1e38f), float3(-1e38f) };
    float3 nCenter = float3(0, 0, 0);
    int total = 0;
    for (uint32_t i = 0; i < BRANCHING_FACTOR; ++i) {
        uint32_t CID = BRANCHING_FACTOR * PID + i;
        if (CID >= numChildren) continue;
        float3 mAabb[2] = gprt::load<float3[2]>(params.childAABBs, CID);
        if (any(mAabb[1] < mAabb[0])) continue; // invalid cluster
        nAabb[0] = min(mAabb[0], nAabb[0]);
        nAabb[1] = max(mAabb[1], nAabb[1]);
        uint32_t pplM = getPrimsInNode(params.level - 1, CID, params.numPrims);
        float3 mCenter = gprt::load<float3>(params.childCenters, CID);
        nCenter += mCenter * pplM;
        total += pplM;
    }
    nCenter /= total;

    // Store AABB
    gprt::store<float3[2]>(params.parentAABBs, PID, nAabb);

    // Store center of vertices
    gprt::store<float3>(params.parentCenters, PID, nCenter);
}

[shader("compute")]
[numthreads(32, 1, 1)]
void ComputeTriangleOBBCovariances(uint3 DispatchThreadID: SV_DispatchThreadID, uniform ComputeTriangleOBBCovariancesParams params)
{
    int PID = DispatchThreadID.x;
    int numParents = getNumNodesInLevel(params.level, params.numPrims);
    if (PID >= numParents) return;
    float3 center = gprt::load<float3>(params.parentCenters, PID);
    int total = 0;    
    float3x3 covariance = float3x3(0,0,0,0,0,0,0,0,0);
    for (uint32_t i = 0; i < BRANCHING_FACTOR; ++i) {
        uint32_t primID = PID * BRANCHING_FACTOR + i;
        if (primID >= params.numPrims) continue;
        float3 a, b, c;
        a = gprt::load<float4>(params.trianglesExp, primID * 3 + 0).xyz;
        if (isnan(a.x)) continue;
        b = gprt::load<float4>(params.trianglesExp, primID * 3 + 1).xyz;
        c = gprt::load<float4>(params.trianglesExp, primID * 3 + 2).xyz;
        covariance += outer2(a - center) + outer2(b - center) + outer2(c - center);
        total = total + 3;
    }
    covariance /= total;

    // Temporarily store covariance in OBB memory
    gprt::store<float3x3>(params.parentOBBs, PID, covariance); 
}

// Using law of total covariance: https://en.wikipedia.org/wiki/Law_of_total_covariance
[shader("compute")]
[numthreads(32, 1, 1)]
void ComputeOBBCovariances(uint3 DispatchThreadID: SV_DispatchThreadID, uniform ComputeOBBCovariancesParams params)
{
    int PID = DispatchThreadID.x;
    int numParents = getNumNodesInLevel(params.level, params.numPrims);
    int numChildren = getNumNodesInLevel(params.level-1, params.numPrims);
    if (PID >= numParents) return;
    // Compute expected value of combined partial covariances
    // Also load average points and compute average of those
    int totalPrims = 0;
    int totalM = 0;
    float3 mAvgCenter = float3(0,0,0);
    float3 mCenters[BRANCHING_FACTOR];
    float3x3 nCovarianceLH = float3x3(0,0,0,0,0,0,0,0,0);
    for (int i = 0; i < BRANCHING_FACTOR; ++i) {
        int CID = PID * BRANCHING_FACTOR + i;
        if (CID >= numChildren) break;
        int ppM = getPrimsInNode(params.level-1, CID, params.numPrims);
        mCenters[i] = gprt::load<float3>(params.childCenters, CID);
        float3x3 mCovariance = gprt::load<float3x3>(params.childOBBs, CID);
        mAvgCenter += mCenters[i];
        nCovarianceLH += mCovariance * ppM;
        totalPrims += ppM;
        totalM += 1;
    }
    nCovarianceLH /= totalPrims;
    mAvgCenter /= totalM;
    // Compute covariance over partial averages
    float3x3 nCovarianceRH = float3x3(0,0,0,0,0,0,0,0,0);
    for (int i = 0; i < totalM; ++i)
        nCovarianceRH += outer2(mCenters[i] - mAvgCenter);
    nCovarianceRH /= totalM;
    // Law of total covariance
    float3x3 nCovariance = nCovarianceLH + nCovarianceRH;
    gprt::store<float3x3>(params.parentOBBs, PID, nCovariance);
}

[shader("compute")]
[numthreads(32, 1, 1)]
void ComputeOBBAngles(uint3 DispatchThreadID: SV_DispatchThreadID, uniform ComputeOBBAnglesParams params)
{
    int CID = DispatchThreadID.x;
    int numNClusters = getNumNodesInLevel(params.level, params.numPrims);
    if (CID >= numNClusters) return;
    float3x3 covariance = gprt::load<float3x3>(params.OBBs, CID);
    float3 obbEul = mat3_to_eul(symmetric_eigenanalysis(covariance));
    gprt::store<float3>(params.OBBs, CID * 3 + 0,    float3(1e38f, 1e38f, 1e38f));
    gprt::store<float3>(params.OBBs, CID * 3 + 1, -float3(1e38f, 1e38f, 1e38f));
    gprt::store<float3>(params.OBBs, CID * 3 + 2, obbEul);
}

// Run this "BRANCHING_FACTOR" times, sweeping pc.iteration from 0 to BRANCHING_FACTOR-1
[shader("compute")]
[numthreads(32, 1, 1)]
void ComputeTriangleOBBBounds(uint3 DispatchThreadID: SV_DispatchThreadID, uniform ComputeTriangleOBBBoundsParams params)
{
    int N0ID = DispatchThreadID.x * params.stride + params.iteration;
    int numNClusters = getNumNodesInLevel(0, params.numPrims);
    if (N0ID >= numNClusters) return;

    // load triangles into registers
    int numTris = 0;
    float3x4 tris[BRANCHING_FACTOR];
    for (int i = 0; i < BRANCHING_FACTOR; ++i) {
        int primID = N0ID * BRANCHING_FACTOR + i;
        if (primID >= params.numPrims) continue;
        tris[i] = gprt::load<float3x4>(params.trianglesExp, primID);
        numTris++;
    }

    for (int level = 0; level < params.numLevels; ++level) {
        int parentID = getPrimParentNode(level, N0ID * BRANCHING_FACTOR);
        gprt::Buffer oobbs = params.OBBHierarchy[level];

        float3 obbEul = gprt::load<float3>(oobbs, parentID * 3 + 2);
        float3x3 obbRot = eul_to_mat3(obbEul);
        float3 obbMin =    float3(1e38f, 1e38f, 1e38f);
        float3 obbMax = -float3(1e38f, 1e38f, 1e38f);
        for (int i = 0; i < numTris; ++i) {
            // Rotate leaf vertices into the leaf OBB
            for (int j = 0; j < 3; ++j) {
                float3 V = mul(obbRot, tris[i][j].xyz);
                obbMin = min(obbMin, V);
                obbMax = max(obbMax, V);
            }
        }
        if (level == 0) {
            // Store OBB bounds for current leaf node
            gprt::store<float3>(oobbs, N0ID * 3 + 0, obbMin);
            gprt::store<float3>(oobbs, N0ID * 3 + 1, obbMax);
        }
        else {
            // Atomically merge into our parent. 
            gprt::atomicMin32f(oobbs, parentID * 3 + 0, obbMin);
            gprt::atomicMax32f(oobbs, parentID * 3 + 1, obbMax);
        }
    }
}
