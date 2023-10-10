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

#include "svd.hlsli"

[[vk::push_constant]] gprt::NNConstants pc;

bool getPointBounds(gprt::Buffer points, uint primID, out float3 aabbMin, out float3 aabbMax)
{
  float3 p = gprt::load<float3>(points, primID);
  if (isnan(p.x)) return false;
  aabbMin = p;
  aabbMax = p;
  return true;
}

bool getEdgeBounds(gprt::Buffer edges, gprt::Buffer points, uint primID, out float3 aabbMin, out float3 aabbMax)
{
  uint2 edge = gprt::load<uint2>(edges, primID);
  float3 p1 = gprt::load<float3>(points, edge.x);
  if (isnan(p1.x)) return false;
  float3 p2 = gprt::load<float3>(points, edge.y);
  aabbMin = float3(min(p1.x,p2.x),min(p1.y,p2.y),min(p1.z,p2.z));
  aabbMax = float3(max(p1.x,p2.x),max(p1.y,p2.y),max(p1.z,p2.z));
  return true;
}

bool getTriangleBounds(gprt::Buffer triangles, gprt::Buffer points, uint primID, out float3 aabbMin, out float3 aabbMax)
{
  uint3 tri = gprt::load<uint3>(triangles, primID);
  float3 p1 = gprt::load<float3>(points, tri.x);
  if (isnan(p1.x)) return false;
  float3 p2 = gprt::load<float3>(points, tri.y);
  float3 p3 = gprt::load<float3>(points, tri.z);
  aabbMin = float3(min(p1.x,min(p2.x,p3.x)),min(p1.y,min(p2.y,p3.y)),min(p1.z,min(p2.z,p3.z)));
  aabbMax = float3(max(p1.x,max(p2.x,p3.x)),max(p1.y,max(p2.y,p3.y)),max(p1.z,max(p2.z,p3.z)));
  return true;
}

bool getPointCentroid(gprt::Buffer points, uint primID, out float3 c) {
  float3 p1 = gprt::load<float3>(points, primID);
  if (isnan(p1.x)) return false;
  c = p1;
  return true;
}

bool getEdgeCentroid(gprt::Buffer edges, gprt::Buffer points, uint primID, out float3 c) 
{
  uint2 edge = gprt::load<uint2>(edges, primID);
  float3 p1 = gprt::load<float3>(points, edge.x);
  if (isnan(p1.x)) return false;
  float3 p2 = gprt::load<float3>(points, edge.y);
  c = (p1 + p2) / 2.f;
  return true;
}

bool getTriangleCentroid(gprt::Buffer triangles, gprt::Buffer points, uint primID, out float3 c) 
{
  uint3 tri = gprt::load<uint3>(triangles, primID);
  float3 p1 = gprt::load<float3>(points, tri.x);
  if (isnan(p1.x)) return false;
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
  if (isnan(p1.x)) return false;
  float3 p2 = gprt::load<float3>(points, tri.y);
  float3 p3 = gprt::load<float3>(points, tri.z);
  c = (p1 + p2 + p3) / 3.f;

  float3 aabbMin = min(min(p1, p2), p3);
  float3 aabbMax = max(max(p1, p2), p3);
  float3 diagonal = aabbMax - aabbMin;
  d = length(diagonal);
  return true; 
}

int getNumNodesInLevel(int level, int numPrimitives) {
  // For level -1, the number of "nodes" is just the number of primitives
  int numNodes = numPrimitives;
  // When level is 0 or higher, we repeatedly divide the number of primitives per
  // node, rounding up. 
  for (int i = 0; i < (level + 1); ++i) {
    numNodes = (numNodes + (BRANCHING_FACTOR - 1)) / BRANCHING_FACTOR;
  }
  return numNodes;
}

/**
 * @brief Returns the number of primitives in a given node in our "complete" tree 
 * @param level What level is the node in the tree? -1 refers to primitives per leaf, 0 is the first level, 1 is the second, etc
 * @param index What node are we asking about relative to the given level?
 * @param numPrimitives How many primitives in total are there in the tree?
 */
int getPrimsInNode(
  int level,   
  int index,    
  int numPrimitives
) {
  int numNodesInLevel = getNumNodesInLevel(level, numPrimitives);
  // Theoretical maximum primitives in the level
  int maxPrimsInLevel = pow(BRANCHING_FACTOR, level + 1);
  // Account for when primitive counts don't exactly match a multiple of the branching factor
  return (index == (numNodesInLevel - 1)) ? (numPrimitives % maxPrimsInLevel) : maxPrimsInLevel;
}

/**
 * @brief Returns the parent node ID for the given primitive and level 
 * @param level What level is the node in the tree? -1 refers to primitives per leaf, 0 is the first level, 1 is the second, etc
 * @param index What primitive are we asking about?
 */
int getParentNode(
  int level,   
  int index
) {
  // if the index is -1, then the parent index is just ourselves
  int parentIndex = index;
  // When level is 0 or higher, we repeatedly divide the number of primitives per
  // node to determine our parent index.
  for (int i = 0; i < (level + 1); ++i) {
    parentIndex = parentIndex / BRANCHING_FACTOR;
  }
  return parentIndex;
}

typedef gprt::NNAccel NNAccel;

GPRT_COMPUTE_PROGRAM(ComputePointBounds, (NNAccel, record), (1,1,1)) {
  int primID = DispatchThreadID.x;
  if (primID >= record.numPrims) return;
  float3 aabbMin, aabbMax;
  if (!getPointBounds(record.points, primID, aabbMin, aabbMax)) return;
  gprt::atomicMin32f(record.aabb, 0, aabbMin.x);
  gprt::atomicMin32f(record.aabb, 1, aabbMin.y);
  gprt::atomicMin32f(record.aabb, 2, aabbMin.z);
  gprt::atomicMax32f(record.aabb, 3, aabbMax.x);
  gprt::atomicMax32f(record.aabb, 4, aabbMax.y);
  gprt::atomicMax32f(record.aabb, 5, aabbMax.z);
}

GPRT_COMPUTE_PROGRAM(ComputeEdgeBounds, (NNAccel, record), (1,1,1)) {
  int primID = DispatchThreadID.x;
  if (primID >= record.numPrims) return;
  float3 aabbMin, aabbMax;
  if (!getEdgeBounds(record.edges, record.points, primID, aabbMin, aabbMax)) return;
  gprt::atomicMin32f(record.aabb, 0, aabbMin.x);
  gprt::atomicMin32f(record.aabb, 1, aabbMin.y);
  gprt::atomicMin32f(record.aabb, 2, aabbMin.z);
  gprt::atomicMax32f(record.aabb, 3, aabbMax.x);
  gprt::atomicMax32f(record.aabb, 4, aabbMax.y);
  gprt::atomicMax32f(record.aabb, 5, aabbMax.z);
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

GPRT_COMPUTE_PROGRAM(ComputePointHilbertCodes, (NNAccel, record), (1, 1, 1)) {
  // int primID = DispatchThreadID.x;
  // if (primID >= record.numPrims) return;
  // float3 c;
  // if (!getPointCentroid(record.points, primID, c)) {
  //   gprt::store<uint>(record.codes, primID, -1);
  //   return;
  // }
  // float3 aabbMin = gprt::load<float3>(record.aabb, 0);
  // float3 aabbMax = gprt::load<float3>(record.aabb, 1);
  // c = (c - aabbMin) / (aabbMax - aabbMin);
  
  // uint64_t code = hilbert64_encode3D(c.x, c.y, c.z);
  // gprt::store<uint64_t>(record.codes, primID, code);
  // gprt::store<uint64_t>(record.ids, primID, primID);
}

GPRT_COMPUTE_PROGRAM(ComputePointMortonCodes, (NNAccel, record), (1, 1, 1)) {
  // int primID = DispatchThreadID.x;
  // if (primID >= record.numPrims) return;
  // float3 c;
  // if (!getPointCentroid(record.points, primID, c)) {
  //   gprt::store<uint>(record.codes, primID, -1);
  //   return;
  // }
  // float3 aabbMin = gprt::load<float3>(record.aabb, 0);
  // float3 aabbMax = gprt::load<float3>(record.aabb, 1);
  // c = (c - aabbMin) / (aabbMax - aabbMin);

  // uint64_t code = morton64_encode3D(c.x, c.y, c.z);
  // gprt::store<uint64_t>(record.codes, primID, code);
  // gprt::store<uint64_t>(record.ids, primID, primID);
}

GPRT_COMPUTE_PROGRAM(ComputeEdgeHilbertCodes, (NNAccel, record), (1, 1, 1)) {
  // int primID = DispatchThreadID.x;
  // if (primID >= record.numPrims) return;
  // float3 c;
  // if (!getEdgeCentroid(record.edges, record.points, primID, c)) {
  //   gprt::store<uint>(record.codes, primID, -1);
  //   return;
  // }

  // float3 aabbMin = gprt::load<float3>(record.aabb, 0);
  // float3 aabbMax = gprt::load<float3>(record.aabb, 1);

  // c = (c - aabbMin) / (aabbMax - aabbMin);

  // uint64_t code = hilbert64_encode3D(c.x, c.y, c.z);
  // gprt::store<uint64_t>(record.codes, primID, code);
  // gprt::store<uint64_t>(record.ids, primID, primID);
}

GPRT_COMPUTE_PROGRAM(ComputeEdgeMortonCodes, (NNAccel, record), (1, 1, 1)) {
  // int primID = DispatchThreadID.x;
  // if (primID >= record.numPrims) return;
  // float3 c;
  // if (!getEdgeCentroid(record.edges, record.points, primID, c)) {
  //   gprt::store<uint>(record.codes, primID, -1);
  //   return;
  // }

  // float3 aabbMin = gprt::load<float3>(record.aabb, 0);
  // float3 aabbMax = gprt::load<float3>(record.aabb, 1);

  // c = (c - aabbMin) / (aabbMax - aabbMin);

  // uint64_t code = morton64_encode3D(c.x, c.y, c.z);
  // gprt::store<uint64_t>(record.codes, primID, code);
  // gprt::store<uint64_t>(record.ids, primID, primID);
}

GPRT_COMPUTE_PROGRAM(ComputeTriangleRootBounds, (NNAccel, record), (32,1,1)) {
  int primID = DispatchThreadID.x;
  if (primID >= pc.numPrims) return;

  uint3 tri = gprt::load<uint3>(pc.indices, primID);
  float3 p1, p2, p3;
  p1 = gprt::load<float3>(pc.points, tri.x);
  if (isnan(p1.x)) return;
  p2 = gprt::load<float3>(pc.points, tri.y);
  p3 = gprt::load<float3>(pc.points, tri.z);
  float3 c = (p1 + p2 + p3) / 3.f;

  // temporarily cache the center point here for computing codes later
  gprt::store<float3>(pc.buffer1, primID, c);

  float3 aabbMin = min(p1, min(p2, p3));
  float3 aabbMax = max(p1, max(p2, p3));

  // Compute world bounding box
  gprt::atomicMin32f(pc.buffer2, 0, aabbMin);
  gprt::atomicMax32f(pc.buffer2, 1, aabbMax);
}

GPRT_COMPUTE_PROGRAM(ComputeTriangleCodes, (NNAccel, record), (32, 1, 1)) {
  int primID = DispatchThreadID.x;
  if (primID >= pc.numPrims) return;

  // Transform center into 0-1 range
  float3 c = gprt::load<float3>(pc.buffer1, primID);
  float3 aabbMin = gprt::load<float3>(pc.buffer2, 0);
  float3 aabbMax = gprt::load<float3>(pc.buffer2, 1);
  c = (c - aabbMin) / (aabbMax - aabbMin);
  uint64_t code = hilbert64_encode3D(c.x, c.y, c.z);
  gprt::store<uint64_t>(pc.buffer3, primID, code);
  gprt::store<uint64_t>(pc.buffer4, primID, primID);
}

GPRT_COMPUTE_PROGRAM(ExpandTriangles, (NNAccel, record), (32,1,1)) {
  int primID = DispatchThreadID.x;
  if (primID >= pc.numPrims) return;
  uint32_t primAddr = uint32_t(gprt::load<uint64_t>(pc.buffer1, primID));
  // load unsorted address
  int3 tri = gprt::load<int3>(pc.indices, primAddr);
  float3 a = gprt::load<float3>(pc.points, tri.x);
  float3 b = gprt::load<float3>(pc.points, tri.y);
  float3 c = gprt::load<float3>(pc.points, tri.z);
  gprt::store<float3>(pc.triangles, primID * 3 + 0, a);
  gprt::store<float3>(pc.triangles, primID * 3 + 1, b);
  gprt::store<float3>(pc.triangles, primID * 3 + 2, c);
}

GPRT_COMPUTE_PROGRAM(ComputeTriangleAABBsAndCenters, (NNAccel, record), (32,1,1)) {
  int NID = DispatchThreadID.x;
  int numNClusters = getNumNodesInLevel(pc.level, pc.numPrims);
  if (NID >= numNClusters) return;

  float3 aabbMin =  float3(1e38f, 1e38f, 1e38f);
  float3 aabbMax = -float3(1e38f, 1e38f, 1e38f);
  float3 center = float3(0, 0, 0);
  int total = 0;  
  for (uint32_t pid = 0; pid < BRANCHING_FACTOR; ++pid) {
    uint32_t primID = NID * BRANCHING_FACTOR + pid;
    float3 a, b, c;
    a = gprt::load<float3>(pc.triangles, primID * 3 + 0);
    if (isnan(a.x)) continue;
    b = gprt::load<float3>(pc.triangles, primID * 3 + 1);
    c = gprt::load<float3>(pc.triangles, primID * 3 + 2);
    aabbMin = min(aabbMin, min(min(a, b), c));
    aabbMax = max(aabbMax, max(max(a, b), c));
    center += (a + b + c);
    total += 3;
  }
  center /= total;

  // Store AABB
  gprt::store<float3>(pc.buffer1, NID * 2 + 0, aabbMin);
  gprt::store<float3>(pc.buffer1, NID * 2 + 1, aabbMax);

  // Store center of vertices
  gprt::store<float3>(pc.buffer3, NID, center);
}

GPRT_COMPUTE_PROGRAM(ComputeAABBsAndCenters, (NNAccel, record), (32,1,1)) {
  int NID = DispatchThreadID.x;
  int numNClusters = getNumNodesInLevel(pc.level, pc.numPrims);
  int numMClusters = getNumNodesInLevel(pc.level-1, pc.numPrims);
  if (NID >= numNClusters) return;  
  float3 nAabbMin = float3(1e38f, 1e38f, 1e38f); 
  float3 nAabbMax = -float3(1e38f, 1e38f, 1e38f);
  float3 nCenter = float3(0, 0, 0);
  int total = 0;
  for (uint32_t i = 0; i < BRANCHING_FACTOR; ++i) {
    uint32_t MID = BRANCHING_FACTOR * NID + i;
    if (MID >= numMClusters) continue;
    float3 mAabbMin = gprt::load<float3>(pc.buffer2, MID * 2 + 0);
    float3 mAabbMax = gprt::load<float3>(pc.buffer2, MID * 2 + 1);
    if (any(mAabbMax < mAabbMin)) continue; // invalid cluster
    nAabbMin = min(mAabbMin, nAabbMin);
    nAabbMax = max(mAabbMax, nAabbMax);
    uint32_t pplM = getPrimsInNode(pc.level - 1, MID, pc.numPrims);
    float3 mCenter = gprt::load<float3>(pc.buffer4, MID);
    nCenter += mCenter * pplM;
    total += pplM;
  }
  nCenter /= total;

  // Store AABB
  gprt::store<float3>(pc.buffer1, NID * 2 + 0, float3(nAabbMin));
  gprt::store<float3>(pc.buffer1, NID * 2 + 1, float3(nAabbMax));

  // Store center of vertices
  gprt::store<float3>(pc.buffer3, NID, nCenter);
}

GPRT_COMPUTE_PROGRAM(ComputeTriangleOBBCovariances, (NNAccel, record), (32,1,1)) {
  int NID = DispatchThreadID.x;
  int numNClusters = getNumNodesInLevel(pc.level, pc.numPrims);
  if (NID >= numNClusters) return;
  float3 center = gprt::load<float3>(pc.buffer3, NID);
  int total = 0;  
  float3x3 covariance = float3x3(0,0,0,0,0,0,0,0,0);
  for (uint32_t pid = 0; pid < BRANCHING_FACTOR; ++pid) {
    uint32_t primID = NID * BRANCHING_FACTOR + pid;
    float3 a, b, c;
    a = gprt::load<float3>(pc.triangles, primID * 3 + 0);
    if (isnan(a.x)) continue;
    b = gprt::load<float3>(pc.triangles, primID * 3 + 1);
    c = gprt::load<float3>(pc.triangles, primID * 3 + 2);
    covariance += outer2(a - center) + outer2(b - center) + outer2(c - center);
    total = total + 3;
  }
  covariance /= total;

  // Temporarily store covariance in OBB memory
  gprt::store<float3x3>(pc.buffer1, NID, covariance); 
}

// Using law of total covariance: https://en.wikipedia.org/wiki/Law_of_total_covariance
GPRT_COMPUTE_PROGRAM(ComputeOBBCovariances, (NNAccel, record), (32,1,1)) {
  int NID = DispatchThreadID.x;
  int numNClusters = getNumNodesInLevel(pc.level, pc.numPrims);
  int numMClusters = getNumNodesInLevel(pc.level-1, pc.numPrims);
  if (NID >= numNClusters) return;
  // Compute expected value of combined partial covariances
  // Also load average points and compute average of those
  int totalPrims = 0;
  int totalM = 0;
  float3 mAvgCenter = float3(0,0,0);
  float3 mCenters[BRANCHING_FACTOR];
  float3x3 nCovarianceLH = float3x3(0,0,0,0,0,0,0,0,0);
  for (int i = 0; i < BRANCHING_FACTOR; ++i) {
    int MID = NID * BRANCHING_FACTOR + i;
    if (MID >= numMClusters) break;
    int ppM = getPrimsInNode(pc.level-1, MID, pc.numPrims);
    mCenters[i] = gprt::load<float3>(pc.buffer4, MID);
    float3x3 mCovariance = gprt::load<float3x3>(pc.buffer2, MID);
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
  gprt::store<float3x3>(pc.buffer1, NID, nCovariance);
}

GPRT_COMPUTE_PROGRAM(ComputeOBBAngles, (NNAccel, record), (32,1,1)) {
  int NID = DispatchThreadID.x;
  int numNClusters = getNumNodesInLevel(pc.level, pc.numPrims);
  if (NID >= numNClusters) return;
  float3x3 covariance = gprt::load<float3x3>(pc.buffer1, NID);
  float3 obbEul = mat3_to_eul(symmetric_eigenanalysis(covariance));
  gprt::store<float3>(pc.buffer1, NID * 3 + 0,  float3(1e38f, 1e38f, 1e38f));
  gprt::store<float3>(pc.buffer1, NID * 3 + 1, -float3(1e38f, 1e38f, 1e38f));
  gprt::store<float3>(pc.buffer1, NID * 3 + 2, obbEul);
}

// Run this "BRANCHING_FACTOR" times, sweeping pc.iteration from 0 to BRANCHING_FACTOR-1
GPRT_COMPUTE_PROGRAM(ComputeTriangleOBBBounds, (NNAccel, record), (32,1,1)) {
  int N0ID = DispatchThreadID.x * BRANCHING_FACTOR + pc.iteration;
  if (N0ID >= record.numL0Clusters) return;

  float3 v[BRANCHING_FACTOR * 3];
  for (int i = 0; i < BRANCHING_FACTOR; ++i) {
    int primID = N0ID * BRANCHING_FACTOR + i;
    if (primID >= pc.numPrims) continue;
    // load triangles into registers
    v[i * 3 + 0] = gprt::load<float3>(pc.triangles, primID * 3 + 0);
    v[i * 3 + 1] = gprt::load<float3>(pc.triangles, primID * 3 + 1);
    v[i * 3 + 2] = gprt::load<float3>(pc.triangles, primID * 3 + 2);
  }

  for (int level = 0; level < 7; ++level) {  
    int parentID = getParentNode(level, N0ID * BRANCHING_FACTOR);
    gprt::Buffer obbs;
         if (level == 0) obbs = record.l0obbs;
    else if (level == 1) obbs = record.l1obbs;
    else if (level == 2) obbs = record.l2obbs;
    else if (level == 3) obbs = record.l3obbs;
    else if (level == 4) obbs = record.l4obbs;
    else if (level == 5) obbs = record.l5obbs;
    else if (level == 6) obbs = record.l6obbs;

    float3 obbEul = gprt::load<float3>(obbs, parentID * 3 + 2);
    float3x3 obbRot = eul_to_mat3(obbEul);
    float3 obbMin =  float3(1e38f, 1e38f, 1e38f);
    float3 obbMax = -float3(1e38f, 1e38f, 1e38f);
    for (int i = 0; i < BRANCHING_FACTOR * 3; ++i) {
      if (i >= pc.numPrims * 3) continue;
      // Rotate leaf vertices into the leaf OBB
      float3 V = mul(obbRot, v[i]);
      obbMin = min(obbMin, V);
      obbMax = max(obbMax, V);
    }

    if (level == 0) {
      // Store OBB bounds for current leaf node
      gprt::store<float3>(obbs, N0ID * 3 + 0, obbMin);
      gprt::store<float3>(obbs, N0ID * 3 + 1, obbMax);
    }
    else {
      // Atomically merge into our parent. 
      gprt::atomicMin32f(obbs, parentID * 3 + 0, obbMin);
      gprt::atomicMax32f(obbs, parentID * 3 + 1, obbMax);
    }
  }
}

GPRT_COMPUTE_PROGRAM(ComputePointClusters, (NNAccel, record), (1,1,1)) {
  // int clusterID = DispatchThreadID.x;
  // if (clusterID >= record.numL0Clusters) return;
  // uint32_t numPrims = record.numPrims;
  // uint32_t numL0Clusters = record.numL0Clusters;

  // float3 clusterAabbMin = float3(1e38f, 1e38f, 1e38f); 
  // float3 clusterAabbMax = -float3(1e38f, 1e38f, 1e38f);
  // for (uint32_t i = 0; i < BRANCHING_FACTOR; ++i) {
  //   uint32_t idx = BRANCHING_FACTOR * clusterID + i;
  //   uint32_t primID = uint32_t(gprt::load<uint64_t>(record.ids, idx));
  //   if (primID >= numPrims || primID == -1) continue;
  //   float3 aabbMin, aabbMax;
  //   if (!getPointBounds(record.points, primID, aabbMin, aabbMax)) continue;
  //   clusterAabbMin = min(aabbMin, clusterAabbMin);
  //   clusterAabbMax = max(aabbMax, clusterAabbMax);
  // }

  // gprt::store<float3>(record.l0clusters, 2 * clusterID + 0, clusterAabbMin);
  // gprt::store<float3>(record.l0clusters, 2 * clusterID + 1, clusterAabbMax);
}

GPRT_COMPUTE_PROGRAM(ComputeEdgeClusters, (NNAccel, record), (1,1,1)) {
  // int clusterID = DispatchThreadID.x;
  // if (clusterID >= record.numL0Clusters) return;
  // uint32_t numPrims = record.numPrims;
  // uint32_t numL0Clusters = record.numL0Clusters;

  // float3 clusterAabbMin = float3(1e38f, 1e38f, 1e38f); 
  // float3 clusterAabbMax = -float3(1e38f, 1e38f, 1e38f);
  // for (uint32_t i = 0; i < BRANCHING_FACTOR; ++i) {
  //   uint32_t idx = BRANCHING_FACTOR * clusterID + i;
  //   uint32_t primID = uint32_t(gprt::load<uint64_t>(record.ids, idx));
  //   if (primID >= numPrims || primID == -1) continue;
  //   float3 aabbMin, aabbMax;
  //   if (!getEdgeBounds(record.edges, record.points, primID, aabbMin, aabbMax)) continue;
  //   clusterAabbMin = min(aabbMin, clusterAabbMin);
  //   clusterAabbMax = max(aabbMax, clusterAabbMax);
  // }

  // gprt::store<float3>(record.l0clusters, 2 * clusterID + 0, clusterAabbMin);
  // gprt::store<float3>(record.l0clusters, 2 * clusterID + 1, clusterAabbMax);
}

// GPRT_COMPUTE_PROGRAM(ComputeTriangleClusters, (NNAccel, record), (1,1,1)) {
//   int clusterID = DispatchThreadID.x;
//   if (clusterID >= record.numL0Clusters) return;
//   uint32_t numPrims = record.numPrims;

//   float3 clusterAabbMin = float3(1e38f, 1e38f, 1e38f); 
//   float3 clusterAabbMax = -float3(1e38f, 1e38f, 1e38f);
//   for (uint32_t i = 0; i < BRANCHING_FACTOR; ++i) {
//     uint32_t idx = BRANCHING_FACTOR * clusterID + i;
//     uint32_t primID = uint32_t(gprt::load<uint64_t>(record.codes, idx));
//     if (primID >= numPrims || primID == -1) continue;
//     float3 aabbMin, aabbMax;
//     if (!getTriangleBounds(record.triangles, record.points, primID, aabbMin, aabbMax)) continue;
//     clusterAabbMin = min(aabbMin, clusterAabbMin);
//     clusterAabbMax = max(aabbMax, clusterAabbMax);
//   }

//   gprt::store<float3>(record.l0clusters, 2 * clusterID + 0, float3(clusterAabbMin));
//   gprt::store<float3>(record.l0clusters, 2 * clusterID + 1, float3(clusterAabbMax));
// }

uint clz(uint x) {
  if (firstbithigh(x) == -1) return 32;
  return 31 - firstbithigh(x);
}


int delta(gprt::Buffer morton_codes, uint num_codes, uint i, uint j) {
  // Karras' delta(i,j) function
  // Denotes the length of the longest common
  // prefix between keys k_i and k_j

  // Cf. Figure 4: "for simplicity, we define that
  // delta(i,j) = -1 when j not in [0,n-1]"
  if (j < 0 || j >= num_codes)
    return -1;
  uint i_code = uint(gprt::load<uint64_t>(morton_codes, i));
  uint j_code = uint(gprt::load<uint64_t>(morton_codes, j));
  uint xord = i_code ^ j_code;
  if (xord == 0)
    return clz(i ^ j) + 32;
  else
    return clz(xord);
}

// Find node range that an inner node overlaps
int2 determine_range(
  gprt::Buffer morton_codes,
  int num_codes, int i, inout int split) {

  // Determine direction of the range (+1 or -1)
  int d = delta(morton_codes, num_codes, i, i + 1) >= delta(morton_codes, num_codes, i, i - 1) ? 1 : -1;

  // Compute upper bound for the length of the range
  int delta_min = delta(morton_codes, num_codes, i, i - d);
  int l_max = 2;
  while (delta(morton_codes, num_codes, i, i + l_max * d) > delta_min)
    l_max *= 2;

  // Find the other end using binary search
  int l = 0;
  for (int t = l_max >> 1; t >= 1; t >>= 1)
    if (delta(morton_codes, num_codes, i, i + (l + t) * d) > delta_min)
      l += t;

  int j = i + l * d;

  // Find the split position using binary search
  int delta_node = delta(morton_codes, num_codes, i, j);
  int s = 0;
  float divf = 2.f;
  int t = ceil(l / divf);
  for(; t >= 1; divf *= 2.f, t = ceil(l / divf))
  {
    uint cs = gprt::load<uint>(morton_codes, i + (s + t) * d);
    if (delta(morton_codes, num_codes, i, i + (s + t) * d) > delta_node)
      s += t;
  }

  split = i + s * d + min(d, 0);

  if (d == 1)
    return int2(i, j);
  else
    return int2(j, i);
}

void storeNode(gprt::Buffer buffer, int address, int4 node) {
  gprt::store<int4>(buffer, address, node);
}

void storeNodeLeft(gprt::Buffer buffer, int address, int left) {
  gprt::store<int>(buffer, address * 4 + 0, left);
}

void storeNodeRight(gprt::Buffer buffer, int address, int right) {
  gprt::store<int>(buffer, address * 4 + 1, right);
}

void storeNodeParent(gprt::Buffer buffer, int address, int parent) {
  gprt::store<int>(buffer, address * 4 + 2, parent);
}

void storeNodeLeaf(gprt::Buffer buffer, int address, int leaf) {
  gprt::store<int>(buffer, address * 4 + 3, leaf);
}

int4 loadNode(gprt::Buffer buffer, int address) {
  return gprt::load<int4>(buffer, address);
}

int loadNodeLeft(gprt::Buffer buffer, int address) {
  return gprt::load<int>(buffer, address * 4 + 0);
}

int loadNodeRight(gprt::Buffer buffer, int address) {
  return gprt::load<int>(buffer, address * 4 + 1);
}

int loadNodeParent(gprt::Buffer buffer, int address) {
  return gprt::load<int>(buffer, address * 4 + 2);
}

int loadNodeLeaf(gprt::Buffer buffer, int address) {
  return gprt::load<int>(buffer, address * 4 + 3);
}

// GPRT_COMPUTE_PROGRAM(ComputeAABBCodes, (NNAccel, record), (1, 1, 1)) {
//   int aabbID = DispatchThreadID.x;
//   if (aabbID >= record.numL3Clusters) return;

//   #ifdef ENABLE_OBBS
//   float3 obbMin = gprt::load<float3>(record.l3clusters, aabbID * 3 + 0);
//   float3 obbMax = gprt::load<float3>(record.l3clusters, aabbID * 3 + 1);
//   float3 obbEul = gprt::load<float3>(record.l3clusters, aabbID * 3 + 2);
//   if (any(obbMax < obbMin)) {
//     gprt::store<uint>(record.codes, aabbID, -1);
//     return;
//   }; // invalid cluster
//   float3x3 obbRotInv = transpose(eul_to_mat3(obbEul));
//   float3 aabbMin = float3(1e38f, 1e38f, 1e38f);
//   float3 aabbMax = -float3(1e38f, 1e38f, 1e38f);
//   for (int k = 0; k < 8; ++k) {
//     float3 obbCor = getCorner(obbMin, obbMax, k);
//     obbCor = mul(obbRotInv, obbCor);
//     aabbMin = min(aabbMin, obbCor);
//     aabbMax = max(aabbMax, obbCor);
//   }
//   #else
//   float3 aabbMin = gprt::load<float3>(record.l3clusters, aabbID * 2 + 0);
//   float3 aabbMax = gprt::load<float3>(record.l3clusters, aabbID * 2 + 1);
//   #endif

//   if (isnan(aabbMin.x) || all(aabbMax < aabbMin)) {
//     gprt::store<uint>(record.codes, aabbID, -1);
//     return;
//   }
//   float3 c = (aabbMax + aabbMin) * .5f;
  
//   float3 gAabbMin = gprt::load<float3>(record.aabb, 0);
//   float3 gAabbMax = gprt::load<float3>(record.aabb, 1);
//   c = (c - gAabbMin) / (gAabbMax - gAabbMin);

//   uint64_t code = uint64_t(morton_encode3D(c.x, c.y, c.z));
//   uint64_t lbvhId = uint64_t(aabbID);
  
//   gprt::store<uint64_t>(record.lbvhMortonCodes, aabbID, code);
//   gprt::store<uint64_t>(record.lbvhIds, aabbID, lbvhId);
// }

// GPRT_COMPUTE_PROGRAM(MakeNodes, (NNAccel, record), (1, 1, 1)) {
//   int nodeID = DispatchThreadID.x;
//   if (nodeID >= (record.numL3Clusters * 2 - 1)) return;
//   storeNode(record.lbvhNodes, nodeID, int4(-1,-1,-1,-1));
//   gprt::store<float3>(record.lbvhAabbs, nodeID * 2 + 0,  float3(1e38f, 1e38f, 1e38f));
//   gprt::store<float3>(record.lbvhAabbs, nodeID * 2 + 1, -float3(1e38f, 1e38f, 1e38f));
// }

// GPRT_COMPUTE_PROGRAM(SplitNodes, (NNAccel, record), (1, 1, 1)) {
//   int num_leaves = record.numL3Clusters;
//   int num_codes = record.numL3Clusters;
//   gprt::Buffer morton_codes = record.lbvhMortonCodes;
//   int index = DispatchThreadID.x;
//   int numInner = record.numL3Clusters - 1;

//   if (index < numInner)
//   {
//     // NOTE: This is [first..last], not [first..last)!!
//     int split = -1;
//     int2 range = determine_range(morton_codes, num_codes, index, split);
//     int first = range.x;
//     int last = range.y;

//     int left = split;
//     int right = split + 1;

//     if (left == first)
//     {
//       // left child is leaf
//       // inner[index].left = num_inner + left;
//       storeNodeLeft(record.lbvhNodes, index, numInner + left);
//       // leaves[left].parent = index;
//       storeNodeParent(record.lbvhNodes, numInner + left, index);
//     }
//     else
//     {
//       // left child is inner
//       // inner[index].left = left;
//       storeNodeLeft(record.lbvhNodes, index, left);
//       // inner[left].parent = index;
//       storeNodeParent(record.lbvhNodes, left, index);
//     }

//     if (right == last)
//     {
//       // right child is leaf
//       // inner[index].right = num_inner + right;
//       storeNodeRight(record.lbvhNodes, index, numInner + right);
//       // leaves[right].parent = index;
//       storeNodeParent(record.lbvhNodes, numInner + right, index);
//     }
//     else
//     {
//       // right child is inner
//       // inner[index].right = right;
//       storeNodeRight(record.lbvhNodes, index, right);
//       // inner[right].parent = index;
//       storeNodeParent(record.lbvhNodes, right, index);
//     }
//   }
// }

// GPRT_COMPUTE_PROGRAM(BuildHierarchy, (NNAccel, record), (1, 1, 1)) {
//   int index = DispatchThreadID.x;
//   uint numLeaves = record.numL3Clusters;
//   if (index >= numLeaves) return;

//   uint id = uint32_t(gprt::load<uint64_t>(record.lbvhIds, index));
//   #ifdef ENABLE_OBBS
//   float3 obbMin = gprt::load<float3>(record.l3clusters, id * 3 + 0);
//   float3 obbMax = gprt::load<float3>(record.l3clusters, id * 3 + 1);
//   float3 obbEul = gprt::load<float3>(record.l3clusters, id * 3 + 2);
//   if (any(obbMax < obbMin)) {
//     gprt::store<uint>(record.codes, id, -1);
//     return;
//   }; // invalid cluster
//   float3x3 obbRotInv = transpose(eul_to_mat3(obbEul));
//   float3 aabbMin = float3(1e38f, 1e38f, 1e38f);
//   float3 aabbMax = -float3(1e38f, 1e38f, 1e38f);
//   for (int k = 0; k < 8; ++k) {
//     float3 obbCor = getCorner(obbMin, obbMax, k);
//     obbCor = mul(obbRotInv, obbCor);
//     aabbMin = min(aabbMin, obbCor);
//     aabbMax = max(aabbMax, obbCor);
//   }
//   #else
//   float3 aabbMin = gprt::load<float3>(record.l3clusters, id * 2 + 0);
//   float3 aabbMax = gprt::load<float3>(record.l3clusters, id * 2 + 1);
//   #endif

//   // float3 aabbMin = gprt::load<float3>(record.l3clusters, id * 2 + 0);
//   // float3 aabbMax = gprt::load<float3>(record.l3clusters, id * 2 + 1);
//   // getTriangleBounds(record.triangles, record.positions, id, aabbMin, aabbMax); // Change to AABBs

//   // Leaf's bounding box
//   int leafAddr = index + (record.numL3Clusters - 1);
//   gprt::store<float3>(record.lbvhAabbs, leafAddr * 2 + 0, aabbMin);
//   gprt::store<float3>(record.lbvhAabbs, leafAddr * 2 + 1, aabbMax);

//   // Leaf's object
//   storeNodeLeaf(record.lbvhNodes, leafAddr, id);
  
//   // Atomically combine child bounding boxes and update parents
//   int next = loadNodeParent(record.lbvhNodes, leafAddr);
//   while (next >= 0) {
//     gprt::atomicMin32f(record.lbvhAabbs, next * 6 + 0, aabbMin.x);
//     gprt::atomicMin32f(record.lbvhAabbs, next * 6 + 1, aabbMin.y);
//     gprt::atomicMin32f(record.lbvhAabbs, next * 6 + 2, aabbMin.z);
//     gprt::atomicMax32f(record.lbvhAabbs, next * 6 + 3, aabbMax.x);
//     gprt::atomicMax32f(record.lbvhAabbs, next * 6 + 4, aabbMax.y);
//     gprt::atomicMax32f(record.lbvhAabbs, next * 6 + 5, aabbMax.z);
//     next = loadNodeParent(record.lbvhNodes, next);   
//   }
// }

struct ClosestPointAttributes {
  int minDist;
}; 

GPRT_INTERSECTION_PROGRAM(ClosestNeighborIntersection, (NNAccel, record)) {
  uint leafID = PrimitiveIndex();
  float3 aabbMin = gprt::load<float3>(record.numL4Clusters, 2 * leafID + 0) + record.maxSearchRange;
  float3 aabbMax = gprt::load<float3>(record.numL4Clusters, 2 * leafID + 1) - record.maxSearchRange;
  float3 origin = WorldRayOrigin();

  float minDist = getMinDist(origin, aabbMin, aabbMax);
  if (minDist > pow(record.maxSearchRange, 2)) return;
  
  ClosestPointAttributes attr;
  attr.minDist = minDist;
  ReportHit(0.0f, 0, attr);
}

// void ClosestPrimitiveQuery(
//   float3 queryOrigin, 
//   float3 gaabbMin, 
//   float3 gaabbMax, 
//   int superClusterID, 
//   int numL0Clusters, 
//   int numPrims, 
//   int primType, 
//   float maxSearchRange, 
//   gprt::Buffer clusters,
//   gprt::Buffer primIDs,
//   gprt::Buffer codes,
//   gprt::Buffer vertices,
//   gprt::Buffer indices,
//   inout NNPayload payload
//   )
// {   
  
// }

GPRT_ANY_HIT_PROGRAM(ClosestPointAnyHit, (NNAccel, record), (NNPayload, payload), (ClosestPointAttributes, hitLeaf)) {
  // uint superClusterID = PrimitiveIndex();
  // float3 origin = WorldRayOrigin();

  // // Out of range
  // if (hitSuperCluster.minDist > payload.closestDistance) {
  //   gprt::ignoreHit();
  //   return;
  // }

  // float3 gaabbMin = gprt::load<float3>(record.aabb, 0);
  // float3 gaabbMax = gprt::load<float3>(record.aabb, 1);

  // ClosestPrimitiveQuery(
  //   origin, gaabbMin, gaabbMax, superClusterID, 
  //   record.numL0Clusters, record.numPrims, 0, record.maxSearchRange,
  //   record.l0clusters, record.ids, record.codes, record.points, gprt::Buffer(0,0), payload);

  // gprt::ignoreHit(); // forces traversal to continue to next supercluster
}

GPRT_ANY_HIT_PROGRAM(ClosestEdgeAnyHit, (NNAccel, record), (NNPayload, payload), (ClosestPointAttributes, hitLeaf)) {
  // uint superClusterID = PrimitiveIndex();
  // float3 origin = WorldRayOrigin();

  // // Out of range
  // if (hitSuperCluster.minDist > payload.closestDistance) {
  //   gprt::ignoreHit();
  //   return;
  // }

  // float3 gaabbMin = gprt::load<float3>(record.aabb, 0);
  // float3 gaabbMax = gprt::load<float3>(record.aabb, 1);

  // ClosestPrimitiveQuery(
  //   origin, gaabbMin, gaabbMax, superClusterID, 
  //   record.numL0Clusters, record.numPrims, 1, record.maxSearchRange,
  //   record.l0clusters, record.ids, record.codes, record.points, record.edges, payload);

  // gprt::ignoreHit(); // forces traversal to continue to next supercluster
}

GPRT_ANY_HIT_PROGRAM(ClosestTriangleAnyHit, (NNAccel, record), (NNPayload, payload), (ClosestPointAttributes, hitLeaf)) {
  // uint leafID = PrimitiveIndex();
  // float3 queryOrigin = WorldRayOrigin();

  // // Out of range
  // if (hitLeaf.minDist > payload.closestDistance) {
  //   gprt::ignoreHit();
  //   return;
  // }

  // TraverseLeaf(record, RAY_FLAG_NONE, queryOrigin, 0.f, record.maxSearchRange, leafID, payload, false);
  
  // gprt::ignoreHit(); // forces traversal to continue to next supercluster
}

