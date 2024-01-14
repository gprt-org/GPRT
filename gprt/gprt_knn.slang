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

#include "gprt_knn.h"

#include "svd.slangh"

bool getPointBounds(gprt::Buffer points, uint primID, out float3 aabbMin, out float3 aabbMax)
{
    float3 p = gprt::load<float3>(points, primID);
    if (isnan(p.x)) { 
        aabbMin = float3(1e38f, 1e38f, 1e38f);
        aabbMax = -float3(1e38f, 1e38f, 1e38f);
        return false;
    }
    aabbMin = p;
    aabbMax = p;
    return true;
}

bool getEdgeBounds(gprt::Buffer edges, gprt::Buffer points, uint primID, out float3 aabbMin, out float3 aabbMax)
{
    uint2 edge = gprt::load<uint2>(edges, primID);
    float3 p1 = gprt::load<float3>(points, edge.x);
    if (isnan(p1.x)) {
        aabbMin = float3(1e38f, 1e38f, 1e38f);
        aabbMax = -float3(1e38f, 1e38f, 1e38f);
        return false;
    }
    float3 p2 = gprt::load<float3>(points, edge.y);
    aabbMin = float3(min(p1.x,p2.x),min(p1.y,p2.y),min(p1.z,p2.z));
    aabbMax = float3(max(p1.x,p2.x),max(p1.y,p2.y),max(p1.z,p2.z));
    return true;
}

bool getTriangleBounds(gprt::Buffer triangles, gprt::Buffer points, uint primID, out float3 aabbMin, out float3 aabbMax)
{
    uint3 tri = gprt::load<uint3>(triangles, primID);
    float3 p1 = gprt::load<float3>(points, tri.x);
    if (isnan(p1.x)) {
        aabbMin = float3(1e38f, 1e38f, 1e38f);
        aabbMax = -float3(1e38f, 1e38f, 1e38f);
        return false;
    }
    float3 p2 = gprt::load<float3>(points, tri.y);
    float3 p3 = gprt::load<float3>(points, tri.z);
    aabbMin = float3(min(p1.x,min(p2.x,p3.x)),min(p1.y,min(p2.y,p3.y)),min(p1.z,min(p2.z,p3.z)));
    aabbMax = float3(max(p1.x,max(p2.x,p3.x)),max(p1.y,max(p2.y,p3.y)),max(p1.z,max(p2.z,p3.z)));
    return true;
}

bool getPointCentroid(gprt::Buffer points, uint primID, out float3 c) {
    float3 p1 = gprt::load<float3>(points, primID);
    if (isnan(p1.x)) { 
        c = float3(1e38f, 1e38f, 1e38f);
        return false;
    }
    c = p1;
    return true;
}

bool getEdgeCentroid(gprt::Buffer edges, gprt::Buffer points, uint primID, out float3 c) 
{
    uint2 edge = gprt::load<uint2>(edges, primID);
    float3 p1 = gprt::load<float3>(points, edge.x);
    if (isnan(p1.x)) {
        c = float3(1e38f, 1e38f, 1e38f);
        return false;
    }
    float3 p2 = gprt::load<float3>(points, edge.y);
    c = (p1 + p2) / 2.f;
    return true;
}

bool getTriangleCentroid(gprt::Buffer triangles, gprt::Buffer points, uint primID, out float3 c) 
{
    uint3 tri = gprt::load<uint3>(triangles, primID);
    float3 p1 = gprt::load<float3>(points, tri.x);
    if (isnan(p1.x)) {
        c = float3(1e38f, 1e38f, 1e38f);
        return false;
    }
    float3 p2 = gprt::load<float3>(points, tri.y);
    float3 p3 = gprt::load<float3>(points, tri.z);
    c = (p1 + p2 + p3) / 3.f;
    return true; 
}

bool getTriangleCentroidAndDiagonal(gprt::Buffer triangles, gprt::Buffer points, uint primID, out float3 c, out float d) 
{
    d = 0.f;
    uint3 tri = gprt::load<uint3>(triangles, primID);
    float3 p1 = gprt::load<float3>(points, tri.x);
    if (isnan(p1.x)) {
        c = float3(1e38f, 1e38f, 1e38f);
        return false;
    }
    float3 p2 = gprt::load<float3>(points, tri.y);
    float3 p3 = gprt::load<float3>(points, tri.z);
    c = (p1 + p2 + p3) / 3.f;

    float3 aabbMin = min(min(p1, p2), p3);
    float3 aabbMax = max(max(p1, p2), p3);
    float3 diagonal = aabbMax - aabbMin;
    d = length(diagonal);
    return true; 
}

float dot2(float3 v) {return dot(v, v);}
void swap(inout float3 a, inout float3 b) { float3 tmp = a; a = b; b = tmp;}
void swap(inout float a, inout float b) { float tmp = a; a = b; b = tmp;}
float4 makeMinimumBoundingSphere(in float3 p1, in float3 p2, in float3 p3) {
    float4 s;

    // Calculate relative distances
    float A = length(p1 - p2);
    float B = length(p2 - p3);
    float C = length(p3 - p1);

    // Re-orient triangle (make A longest side)
    float3 a = p3, b = p1, c = p2;
    if (B < C) swap(B, C), swap(b, c); 
    if (A < B) swap(A, B), swap(a, b); 

    // If obtuse, just use longest diameter, otherwise circumscribe
    if ((B*B) + (C*C) <= (A*A)) {
        s.w = A / 2.f;
        s.xyz = (b + c) / 2.f;
    } else {
        // http://en.wikipedia.org/wiki/Circumscribed_circle
        float cos_a = (B*B + C*C - A*A) / (B*C*2);
        s.w = A / (sqrt(1 - cos_a*cos_a)*2.f);
        float3 alpha = a - c, beta = b - c;
        s.xyz = cross((beta * dot2(alpha) - alpha * dot2(beta)), (cross(alpha, beta))) /
            ((dot2(cross(alpha, beta))) * 2.f) + c;
    }
    return s;
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
    uint32_t code = hilbert_encode3D(c.x, c.y, c.z);
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
    int3 tri = gprt::load<int3>(params.triangles, primAddr);
    float3 a = gprt::load<float3>(params.vertices, tri.x);
    float3 b = gprt::load<float3>(params.vertices, tri.y);
    float3 c = gprt::load<float3>(params.vertices, tri.z);
    gprt::store<float3>(params.trianglesExp, primID * 3 + 0, a);
    gprt::store<float3>(params.trianglesExp, primID * 3 + 1, b);
    gprt::store<float3>(params.trianglesExp, primID * 3 + 2, c);
}

[shader("compute")]
[numthreads(32, 1, 1)]
void ComputeTriangleAABBsAndCenters(uint3 DispatchThreadID: SV_DispatchThreadID, uniform ComputeTriangleAABBsAndCentersParams params) 
{
    int NID = DispatchThreadID.x;
    int numNClusters = getNumNodesInLevel(params.level, params.numPrims);
    if (NID >= numNClusters) return;

    float3 aabbMin =    float3(1e38f, 1e38f, 1e38f);
    float3 aabbMax = -float3(1e38f, 1e38f, 1e38f);
    float3 center = float3(0, 0, 0);
    int total = 0;    
    for (uint32_t pid = 0; pid < BRANCHING_FACTOR; ++pid) {
        uint32_t primID = NID * BRANCHING_FACTOR + pid;
        if (primID >= params.numPrims) continue;
        float3 a, b, c;
        a = gprt::load<float3>(params.trianglesExp, primID * 3 + 0);
        if (isnan(a.x)) continue;
        b = gprt::load<float3>(params.trianglesExp, primID * 3 + 1);
        c = gprt::load<float3>(params.trianglesExp, primID * 3 + 2);
        aabbMin = min(aabbMin, min(min(a, b), c));
        aabbMax = max(aabbMax, max(max(a, b), c));
        center += (a + b + c);
        total += 3;
    }
    center /= total;

    // Store AABB
    gprt::store<float3>(params.aabbs, NID * 2 + 0, aabbMin);
    gprt::store<float3>(params.aabbs, NID * 2 + 1, aabbMax);

    // Store center of vertices
    gprt::store<float3>(params.centers, NID, center);
}

[shader("compute")]
[numthreads(32, 1, 1)]
void ComputeAABBsAndCenters(uint3 DispatchThreadID: SV_DispatchThreadID, uniform ComputeAABBsAndCentersParams params)
{
    int PID = DispatchThreadID.x;
    int numParents = getNumNodesInLevel(params.level, params.numPrims);
    int numChildren = getNumNodesInLevel(params.level-1, params.numPrims);
    if (PID >= numParents) return;    
    float3 nAabbMin = float3(1e38f, 1e38f, 1e38f); 
    float3 nAabbMax = -float3(1e38f, 1e38f, 1e38f);
    float3 nCenter = float3(0, 0, 0);
    int total = 0;
    for (uint32_t i = 0; i < BRANCHING_FACTOR; ++i) {
        uint32_t CID = BRANCHING_FACTOR * PID + i;
        if (CID >= numChildren) continue;
        float3 mAabbMin = gprt::load<float3>(params.childAABBs, CID * 2 + 0);
        float3 mAabbMax = gprt::load<float3>(params.childAABBs, CID * 2 + 1);
        if (any(mAabbMax < mAabbMin)) continue; // invalid cluster
        nAabbMin = min(mAabbMin, nAabbMin);
        nAabbMax = max(mAabbMax, nAabbMax);
        uint32_t pplM = getPrimsInNode(params.level - 1, CID, params.numPrims);
        float3 mCenter = gprt::load<float3>(params.childCenters, CID);
        nCenter += mCenter * pplM;
        total += pplM;
    }
    nCenter /= total;

    // Store AABB
    gprt::store<float3>(params.parentAABBs, PID * 2 + 0, float3(nAabbMin));
    gprt::store<float3>(params.parentAABBs, PID * 2 + 1, float3(nAabbMax));

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
        a = gprt::load<float3>(params.trianglesExp, primID * 3 + 0);
        if (isnan(a.x)) continue;
        b = gprt::load<float3>(params.trianglesExp, primID * 3 + 1);
        c = gprt::load<float3>(params.trianglesExp, primID * 3 + 2);
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

    int numVerts = 0;
    float3 v[BRANCHING_FACTOR * 3];
    for (int i = 0; i < BRANCHING_FACTOR; ++i) {
        int primID = N0ID * BRANCHING_FACTOR + i;
        if (primID >= params.numPrims) continue;
        // load triangles into registers
        v[i * 3 + 0] = gprt::load<float3>(params.trianglesExp, primID * 3 + 0);
        v[i * 3 + 1] = gprt::load<float3>(params.trianglesExp, primID * 3 + 1);
        v[i * 3 + 2] = gprt::load<float3>(params.trianglesExp, primID * 3 + 2);
        numVerts += 3;
    }

    [unroll] // oddly, if I don't unroll, I crash when max levels is 1... Todo, test to see if this is still true with slang
    for (int level = 0; level < params.numLevels; ++level) {
        int parentID = getPrimParentNode(level, N0ID * BRANCHING_FACTOR);
        gprt::Buffer oobbs = params.OBBHierarchy[level];

        float3 obbEul = gprt::load<float3>(oobbs, parentID * 3 + 2);
        float3x3 obbRot = eul_to_mat3(obbEul);
        float3 obbMin =    float3(1e38f, 1e38f, 1e38f);
        float3 obbMax = -float3(1e38f, 1e38f, 1e38f);
        for (int i = 0; i < numVerts; ++i) {
            // Rotate leaf vertices into the leaf OBB
            float3 V = mul(obbRot, v[i]);
            obbMin = min(obbMin, V);
            obbMax = max(obbMax, V);
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
