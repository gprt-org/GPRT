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

void swap(inout float a, inout float b) { float tmp = a; a = b; b = tmp; }
void swap(inout float3 a, inout float3 b) { float3 tmp = a; a = b; b = tmp; }

GPRT_COMPUTE_PROGRAM(ComputeTriangleBounds, (NNAccel, record), (1,1,1)) {
  int primID = DispatchThreadID.x;
  if (primID >= record.numPrims) return;

  float3 aabbMin =  float3(1e20f, 1e20f, 1e20f);
  float3 aabbMax = -float3(1e20f, 1e20f, 1e20f);
  bool invalid = true;
  uint3 tri = gprt::load<uint3>(record.triangles, primID);
  float3 a, b, c;
  a = gprt::load<float3>(record.points, tri.x);
  if (isnan(a.x)) {
    #ifdef ENABLE_OBBS
    gprt::store<float3>(record.primBounds, primID * 3 + 0, float3(1e20f, 1e20f, 1e20f));
    gprt::store<float3>(record.primBounds, primID * 3 + 1, -float3(1e20f, 1e20f, 1e20f));
    gprt::store<float3>(record.primBounds, primID * 3 + 2, float3(0, 0, 0));
    #else
    gprt::store<float3>(record.primBounds, primID * 2 + 0, float3(1e20f, 1e20f, 1e20f));
    gprt::store<float3>(record.primBounds, primID * 2 + 1, -float3(1e20f, 1e20f, 1e20f));
    #endif
  } else {
    invalid = false;
    b = gprt::load<float3>(record.points, tri.y);
    c = gprt::load<float3>(record.points, tri.z);
    aabbMin = min(min(a, b), c);
    aabbMax = max(max(a, b), c);

    #ifdef ENABLE_OBBS
    float3 points[3] = {a, b, c}; 
    float3 center = (points[0] + points[1] + points[2]) / 3.f;
    float3x3 covariance = float3x3(0,0,0,0,0,0,0,0,0);
    for(int i=0; i < 3 /*number of points*/; i++) {
      float3 p = points[i] - center;
      covariance += outerProduct(p, p);
    }
    covariance /= 3.f; // num points;
    float3x3 rot = svd(covariance).V;
    float3 obbRot = mat3_to_eul(rot);
    float3 obbMin =  float3(1e20f, 1e20f, 1e20f);
    float3 obbMax = -float3(1e20f, 1e20f, 1e20f);
    for(int i=0; i < 3 /*number of points*/; i++) {
      float3 p = mul(rot, points[i] - center) + center;
      obbMin = min(obbMin, p);
      obbMax = max(obbMax, p);
    }
    gprt::store<float3>(record.primBounds, primID * 3 + 0, obbMin);
    gprt::store<float3>(record.primBounds, primID * 3 + 1, obbMax);
    gprt::store<float3>(record.primBounds, primID * 3 + 2, obbRot);
    #else
    gprt::store<float3>(record.primBounds, primID * 2 + 0, aabbMin);
    gprt::store<float3>(record.primBounds, primID * 2 + 1, aabbMax);
    #endif
  }
  if (invalid) return;

  // Compute world bounding box
  gprt::atomicMin32f(record.aabb, 0, aabbMin.x);
  gprt::atomicMin32f(record.aabb, 1, aabbMin.y);
  gprt::atomicMin32f(record.aabb, 2, aabbMin.z);
  gprt::atomicMax32f(record.aabb, 3, aabbMax.x);
  gprt::atomicMax32f(record.aabb, 4, aabbMax.y);
  gprt::atomicMax32f(record.aabb, 5, aabbMax.z);
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

GPRT_COMPUTE_PROGRAM(ComputeTriangleHilbertCodes, (NNAccel, record), (1, 1, 1)) {
  // int primID = DispatchThreadID.x;
  // if (primID >= record.numPrims) return;

  // uint3 tri = gprt::load<uint3>(record.triangles, primID);
  // float3 p1 = gprt::load<float3>(record.points, tri.x);
  // if (isnan(p1.x)) {
  //   gprt::store<uint>(record.codes, primID, -1);
  //   return;
  // }
  // float3 p2 = gprt::load<float3>(record.points, tri.y);
  // float3 p3 = gprt::load<float3>(record.points, tri.z);
  // float3 c = (p1 + p2 + p3) / 3.f;
  // float3 d = float3(
  //   distance(c, p1),
  //   distance(c, p2),
  //   distance(c, p3)
  // );
  // float radius = min(min(d.x, d.y), d.z);

  // float3 aabbMin = gprt::load<float3>(record.aabb, 0);
  // float3 aabbMax = gprt::load<float3>(record.aabb, 1);

  // c = (c - aabbMin) / (aabbMax - aabbMin);

  // uint64_t code = hilbert64_encode3D(c.x, c.y, c.z);
  // uint64_t primitive = uint64_t(primID) | (uint64_t(asuint(radius)) << 32ull);

  // if (primID == 0) {
  //   float3 test = hilbert64_decode3D(code);
  //   printf("Original: %f %f %f, quantized %f %f %f\n", c.x, c.y, c.z, test.x, test.y, test.z);
  // }
  
  // gprt::store<uint64_t>(record.codes, primID, code);
  // gprt::store<uint64_t>(record.ids, primID, primitive);
}

GPRT_COMPUTE_PROGRAM(ComputeTriangleCodes, (NNAccel, record), (1, 1, 1)) {
  int primID = DispatchThreadID.x;
  if (primID >= record.numPrims) return;

  uint3 tri = gprt::load<uint3>(record.triangles, primID);
  float3 p1 = gprt::load<float3>(record.points, tri.x);
  if (isnan(p1.x)) {
    gprt::store<uint>(record.codes, primID, -1);
    return;
  }
  float3 p2 = gprt::load<float3>(record.points, tri.y);
  float3 p3 = gprt::load<float3>(record.points, tri.z);
  float3 c = (p1 + p2 + p3) / 3.f;
  float3 d = float3(
    distance(c, p1),
    distance(c, p2),
    distance(c, p3)
  );
  float3 aabbMin = gprt::load<float3>(record.aabb, 0);
  float3 aabbMax = gprt::load<float3>(record.aabb, 1);
  float diagonal = distance(aabbMax, aabbMin);
  c = (c - aabbMin) / (aabbMax - aabbMin);
  uint64_t code = 0;
  code |= uint64_t(primID); 
  code |= (uint64_t(hilbert_encode3D(c.x, c.y, c.z)) << 32ull);
  gprt::store<uint64_t>(record.codes, primID, code);
}

GPRT_COMPUTE_PROGRAM(ComputeTriangleMortonCodes, (NNAccel, record), (1, 1, 1)) {
  // int primID = DispatchThreadID.x;
  // if (primID >= record.numPrims) return;

  // uint3 tri = gprt::load<uint3>(record.triangles, primID);
  // float3 p1 = gprt::load<float3>(record.points, tri.x);
  // if (isnan(p1.x)) {
  //   gprt::store<uint>(record.codes, primID, -1);
  //   return;
  // }
  // float3 p2 = gprt::load<float3>(record.points, tri.y);
  // float3 p3 = gprt::load<float3>(record.points, tri.z);
  // float3 c = (p1 + p2 + p3) / 3.f;
  // float3 d = float3(
  //   distance(c, p1),
  //   distance(c, p2),
  //   distance(c, p3)
  // );
  // float radius = min(min(d.x, d.y), d.z);

  // float3 aabbMin = gprt::load<float3>(record.aabb, 0);
  // float3 aabbMax = gprt::load<float3>(record.aabb, 1);

  // c = (c - aabbMin) / (aabbMax - aabbMin);

  // uint64_t code = morton64_encode3D(c.x, c.y, c.z);
  // uint64_t primitive = uint64_t(primID) | (uint64_t(asuint(radius)) << 32ull);

  // if (primID == 0) {
  //   float3 test = morton64_decode3D(code);
  //   printf("Original: %f %f %f, quantized %f %f %f\n", c.x, c.y, c.z, test.x, test.y, test.z);
  // }
  
  // gprt::store<uint64_t>(record.codes, primID, code);
  // gprt::store<uint64_t>(record.ids, primID, primitive);
}

GPRT_COMPUTE_PROGRAM(ComputePointClusters, (NNAccel, record), (1,1,1)) {
  // int clusterID = DispatchThreadID.x;
  // if (clusterID >= record.numL0Clusters) return;
  // uint32_t numPrims = record.numPrims;
  // uint32_t numL0Clusters = record.numL0Clusters;

  // float3 clusterAabbMin = float3(1e20f, 1e20f, 1e20f); 
  // float3 clusterAabbMax = -float3(1e20f, 1e20f, 1e20f);
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

  // float3 clusterAabbMin = float3(1e20f, 1e20f, 1e20f); 
  // float3 clusterAabbMax = -float3(1e20f, 1e20f, 1e20f);
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

//   float3 clusterAabbMin = float3(1e20f, 1e20f, 1e20f); 
//   float3 clusterAabbMax = -float3(1e20f, 1e20f, 1e20f);
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

GPRT_COMPUTE_PROGRAM(ComputeL0Clusters, (NNAccel, record), (1,1,1)) {

  int l0ClusterID = DispatchThreadID.x;
  if (l0ClusterID >= record.numL0Clusters) return;

  uint32_t numLeaves = record.numLeaves;
  uint32_t numPrims = record.numPrims;

  #ifdef ENABLE_OBBS_INTERNAL
  // In retrospect, this is doing an insane amount of rotations (especially if multiple prims per leaf)...
  // Wondering if there's a better way to compute this...

  // Load all points into registers and compute center
  float3 pObbMins[BRANCHING_FACTOR * PRIMS_PER_LEAF];
  float3 pObbMaxs[BRANCHING_FACTOR * PRIMS_PER_LEAF];
  float3 pObbRots[BRANCHING_FACTOR * PRIMS_PER_LEAF];
  float3 pObbCens[BRANCHING_FACTOR * PRIMS_PER_LEAF];
  float3 l0Center = float3(0.f, 0.f, 0.f);
  uint32_t numDataPoints = 0;
  for (uint32_t i = 0; i < BRANCHING_FACTOR; ++i) {
    uint32_t leafID = BRANCHING_FACTOR * l0ClusterID + i;
    if (leafID >= numLeaves) continue;
    for (uint32_t j = 0; j < PRIMS_PER_LEAF; ++j) {
      uint32_t primID = leafID * PRIMS_PER_LEAF + j;
      uint32_t primitiveAddress = uint32_t(gprt::load<uint64_t>(record.codes, primID)); 
      float3 obbMin = gprt::load<float3>(record.primBounds, primitiveAddress * 3 + 0);
      float3 obbMax = gprt::load<float3>(record.primBounds, primitiveAddress * 3 + 1);
      float3 obbRot = gprt::load<float3>(record.primBounds, primitiveAddress * 3 + 2);
      float3 obbCen = (obbMin + obbMax) * .5f;
      pObbMins[i * PRIMS_PER_LEAF + j] = obbMin;
      pObbMaxs[i * PRIMS_PER_LEAF + j] = obbMax;
      pObbRots[i * PRIMS_PER_LEAF + j] = obbRot;
      pObbCens[i * PRIMS_PER_LEAF + j] = obbCen;
      l0Center += obbCen;
      numDataPoints++;
    }
  }
  l0Center /= numDataPoints;

  // Compute covariance
  float3x3 covariance = float3x3(0,0,0,0,0,0,0,0,0);
  for (uint32_t i = 0; i < BRANCHING_FACTOR; ++i) {
    uint32_t leafID = BRANCHING_FACTOR * l0ClusterID + i;
    if (leafID >= numLeaves) continue;
    for (uint32_t j = 0; j < PRIMS_PER_LEAF; ++j) {
      float3 obbCen = pObbCens[i * PRIMS_PER_LEAF + j];
      float3 p = obbCen - l0Center;
      covariance += outerProduct(p, p);
    }
  }
  covariance /= (numDataPoints * 8);

  // Compute SVD
  float3x3 l0Rot = svd(covariance).V;
  float3 l0obbRot = mat3_to_eul(l0Rot);

  // Compute OBB
  float3 l0obbMin =  float3(1e20f, 1e20f, 1e20f);
  float3 l0obbMax = -float3(1e20f, 1e20f, 1e20f);
  for (uint32_t i = 0; i < BRANCHING_FACTOR; ++i) {
    uint32_t leafID = BRANCHING_FACTOR * l0ClusterID + i;
    if (leafID >= numLeaves) continue;
    for (uint32_t j = 0; j < PRIMS_PER_LEAF; ++j) {
      float3 obbCen = pObbCens[i * PRIMS_PER_LEAF + j];
      float3 obbMin = pObbMins[i * PRIMS_PER_LEAF + j];
      float3 obbMax = pObbMaxs[i * PRIMS_PER_LEAF + j];
      float3x3 obbRot = transpose(eul_to_mat3(pObbRots[i * PRIMS_PER_LEAF + j]));
      obbMin = mul(obbRot, obbMin - obbCen) + obbCen;
      obbMax = mul(obbRot, obbMax - obbCen) + obbCen;
      obbMin = mul(l0Rot, obbMin - l0Center) + l0Center;
      obbMax = mul(l0Rot, obbMax - l0Center) + l0Center;
      l0obbMin = min(l0obbMin, min(obbMin, obbMax));
      l0obbMax = max(l0obbMax, max(obbMin, obbMax));
    }
  }

  gprt::store<float3>(record.l0clusters, l0ClusterID * 3 + 0, float3(l0obbMin));
  gprt::store<float3>(record.l0clusters, l0ClusterID * 3 + 1, float3(l0obbMax));
  gprt::store<float3>(record.l0clusters, l0ClusterID * 3 + 2, float3(l0obbRot));

  #else

  float3 l0ClusterAabbMin = float3(1e20f, 1e20f, 1e20f); 
  float3 l0ClusterAabbMax = -float3(1e20f, 1e20f, 1e20f);
  for (uint32_t i = 0; i < BRANCHING_FACTOR; ++i) {
    uint32_t leafID = BRANCHING_FACTOR * l0ClusterID + i;
    if (leafID >= numLeaves) continue;
    float3 aabbMin = float3(1e20f, 1e20f, 1e20f);
    float3 aabbMax = -float3(1e20f, 1e20f, 1e20f);
    for (uint32_t j = 0; j < PRIMS_PER_LEAF; ++j) {
      uint32_t primID = leafID * PRIMS_PER_LEAF + j;
      uint32_t primitiveAddress = uint32_t(gprt::load<uint64_t>(record.codes, primID)); 
      #ifdef ENABLE_OBBS
      float3 obbMin = gprt::load<float3>(record.primBounds, primitiveAddress * 3 + 0);
      float3 obbMax = gprt::load<float3>(record.primBounds, primitiveAddress * 3 + 1);
      float3 obbRot = gprt::load<float3>(record.primBounds, primitiveAddress * 3 + 2);
      float3x3 obbRotMat = eul_to_mat3(obbRot);      
      float3 obbC = (obbMin + obbMax) * .5f;
      float3x3 rot = transpose(obbRotMat);
      obbMin = mul(rot, obbMin - obbC) + obbC;
      obbMax = mul(rot, obbMax - obbC) + obbC;
      aabbMin = min(aabbMin, min(obbMin, obbMax));
      aabbMax = max(aabbMax, max(obbMin, obbMax));
      #else
      float3 primAabbMin = gprt::load<float3>(record.primBounds, primitiveAddress * 2 + 0);
      float3 primAabbMax = gprt::load<float3>(record.primBounds, primitiveAddress * 2 + 1);
      aabbMin = min(aabbMin, primAabbMin);
      aabbMax = max(aabbMax, primAabbMax);
      #endif
    }
    if (any(aabbMax < aabbMin)) continue; // invalid cluster
    l0ClusterAabbMin = min(aabbMin, l0ClusterAabbMin);
    l0ClusterAabbMax = max(aabbMax, l0ClusterAabbMax);
  }
  gprt::store<float3>(record.l0clusters, 2 * l0ClusterID + 0, float3(l0ClusterAabbMin));
  gprt::store<float3>(record.l0clusters, 2 * l0ClusterID + 1, float3(l0ClusterAabbMax));
  #endif
}

GPRT_COMPUTE_PROGRAM(ComputeL1Clusters, (NNAccel, record), (1,1,1)) {
  int l1ClusterID = DispatchThreadID.x;
  if (l1ClusterID >= record.numL1Clusters) return;
  uint32_t numL0Clusters = record.numL0Clusters;

  float3 l1ClusterAabbMin = float3(1e20f, 1e20f, 1e20f); 
  float3 l1ClusterAabbMax = -float3(1e20f, 1e20f, 1e20f);
  for (uint32_t i = 0; i < BRANCHING_FACTOR; ++i) {
    uint32_t clusterID = BRANCHING_FACTOR * l1ClusterID + i;
    if (clusterID >= numL0Clusters) continue;
    #ifdef ENABLE_OBBS_INTERNAL
    float3 obbMin = gprt::load<float3>(record.l0clusters, clusterID * 3 + 0);
    float3 obbMax = gprt::load<float3>(record.l0clusters, clusterID * 3 + 1);
    float3 obbRot = gprt::load<float3>(record.l0clusters, clusterID * 3 + 2);
    if (any(obbMax < obbMin)) continue; // invalid cluster
    float3x3 obbRotMat = eul_to_mat3(obbRot);      
    float3 obbC = (obbMin + obbMax) * .5f;
    float3x3 rot = transpose(obbRotMat);
    obbMin = mul(rot, obbMin - obbC) + obbC;
    obbMax = mul(rot, obbMax - obbC) + obbC;
    l1ClusterAabbMin = min(l1ClusterAabbMin, min(obbMin, obbMax));
    l1ClusterAabbMax = max(l1ClusterAabbMax, max(obbMin, obbMax));
    #else
    float3 aabbMin = gprt::load<float3>(record.l0clusters, clusterID * 2 + 0);
    float3 aabbMax = gprt::load<float3>(record.l0clusters, clusterID * 2 + 1);
    if (any(aabbMax < aabbMin)) continue; // invalid cluster
    l1ClusterAabbMin = min(l1ClusterAabbMin, aabbMin);
    l1ClusterAabbMax = max(l1ClusterAabbMax, aabbMax);
    #endif
  }

  gprt::store<float3>(record.l1clusters, 2 * l1ClusterID + 0, float3(l1ClusterAabbMin));
  gprt::store<float3>(record.l1clusters, 2 * l1ClusterID + 1, float3(l1ClusterAabbMax));
}

GPRT_COMPUTE_PROGRAM(ComputeL2Clusters, (NNAccel, record), (1,1,1)) {
  int l2ClusterID = DispatchThreadID.x;
  if (l2ClusterID >= record.numL1Clusters) return;
  uint32_t numL1Clusters = record.numL1Clusters;

  float3 l2ClusterAabbMin = float3(1e20f, 1e20f, 1e20f); 
  float3 l2ClusterAabbMax = -float3(1e20f, 1e20f, 1e20f);
  for (uint32_t i = 0; i < BRANCHING_FACTOR; ++i) {
    uint32_t clusterID = BRANCHING_FACTOR * l2ClusterID + i;
    if (clusterID >= numL1Clusters) continue;
    float3 aabbMin = gprt::load<float3>(record.l1clusters, clusterID * 2 + 0);
    float3 aabbMax = gprt::load<float3>(record.l1clusters, clusterID * 2 + 1);
    if (any(aabbMax < aabbMin)) continue; // invalid cluster
    l2ClusterAabbMin = min(aabbMin, l2ClusterAabbMin);
    l2ClusterAabbMax = max(aabbMax, l2ClusterAabbMax);
  }

  gprt::store<float3>(record.l2clusters, 2 * l2ClusterID + 0, float3(l2ClusterAabbMin));
  gprt::store<float3>(record.l2clusters, 2 * l2ClusterID + 1, float3(l2ClusterAabbMax));
}

GPRT_COMPUTE_PROGRAM(ComputeL3Clusters, (NNAccel, record), (1,1,1)) {
  int leafID = DispatchThreadID.x;
  if (leafID >= record.numL3Clusters) return;
  uint32_t numL2Clusters = record.numL2Clusters;

  float3 leafAabbMin = float3(1e20f, 1e20f, 1e20f); 
  float3 leafAabbMax = -float3(1e20f, 1e20f, 1e20f);
  for (uint32_t i = 0; i < BRANCHING_FACTOR; ++i) {
    uint32_t clusterID = BRANCHING_FACTOR * leafID + i;
    if (clusterID >= numL2Clusters) continue;
    float3 aabbMin = gprt::load<float3>(record.l2clusters, clusterID * 2 + 0);
    float3 aabbMax = gprt::load<float3>(record.l2clusters, clusterID * 2 + 1);
    if (any(aabbMax < aabbMin)) continue; // invalid cluster
    leafAabbMin = min(aabbMin, leafAabbMin);
    leafAabbMax = max(aabbMax, leafAabbMax);
  }

  // Dialate by the max search range // Temporarily disabling
  gprt::store<float3>(record.l3clusters, 2 * leafID + 0, leafAabbMin/* - record.maxSearchRange*/);
  gprt::store<float3>(record.l3clusters, 2 * leafID + 1, leafAabbMax/* + record.maxSearchRange*/);
}

GPRT_COMPUTE_PROGRAM(ComputeL3Treelets, (NNAccel, record), (1,1,1)) {
  int l3ID = DispatchThreadID.x;
  if (l3ID >= record.numL3Clusters) return;

  int numL3Clusters = record.numL3Clusters;
  int numL2Clusters = record.numL2Clusters;

  float3 l3Min = gprt::load<float3>(record.l3clusters, 2 * l3ID + 0);
  float3 l3Max = gprt::load<float3>(record.l3clusters, 2 * l3ID + 1);

  // 1) Compute the treelet

  // Note, doubling to increment exponent by 1. This is important, since 
  // we need all children after being scaled to be smaller than their parent
  // and therefore within a 0-1 scale. We drop the mantissa for compression,
  // so as a consequence we need to make the exponent slightly more conservative.
  float3 exactScale = (l3Max - l3Min) * 2.f; 
  uint3 exponent;
  exponent.x = asuint(exactScale.x) >> 23;
  exponent.y = asuint(exactScale.y) >> 23;
  exponent.z = asuint(exactScale.z) >> 23;
  uint32_t exponent32 = (exponent.z << 16) | 
                        (exponent.y <<  8) | 
                        (exponent.x <<  0);
  float4 l3Treelet;
  l3Treelet.xyz = l3Min;
  l3Treelet.w = asfloat(exponent32);
  gprt::store<float4>(record.treelets, l3ID, l3Treelet);
  
  // Similar quantization scheme as Cline et al, but 
  // using just the exponent for additional compression, 
  // and also restoring precision at every level.

  // Also reversed translation and took reciprical of scale 
  // so that the decompression is a fused multiply and accumulate
  float3 scale = float3(
    asfloat(((exponent32 >>  0) & 255) << 23),
    asfloat(((exponent32 >>  8) & 255) << 23),
    asfloat(((exponent32 >> 16) & 255) << 23)
  ) / 255.f;
  float3 translate = l3Min;

  // 2) Quantize the children
  for (uint32_t i = 0; i < BRANCHING_FACTOR; ++i) {
    uint32_t l2ID = l3ID * BRANCHING_FACTOR + i;
    if (l2ID > numL2Clusters) break;
    float3 l2Min = gprt::load<float3>(record.l2clusters, 2 * l2ID + 0);
    float3 l2Max = gprt::load<float3>(record.l2clusters, 2 * l2ID + 1);
    l2Min = (l2Min - translate) / scale;
    l2Max = (l2Max - translate) / scale;
    uint64_t child = ((uint64_t(l2Min.x) & 255ull) <<  0ull)
                   | ((uint64_t(l2Min.y) & 255ull) <<  8ull)
                   | ((uint64_t(l2Min.z) & 255ull) << 16ull)
                   | ((uint64_t(l2Max.x) & 255ull) << 24ull)
                   | ((uint64_t(l2Max.y) & 255ull) << 32ull)
                   | ((uint64_t(l2Max.z) & 255ull) << 40ull);
    gprt::store<uint64_t>(record.children, l2ID, child);   
  }
}

GPRT_COMPUTE_PROGRAM(ComputeL2Treelets, (NNAccel, record), (1,1,1)) {
  int l2ID = DispatchThreadID.x;
  if (l2ID >= record.numL2Clusters) return;

  int numL1Clusters = record.numL1Clusters;
  int numL2Clusters = record.numL2Clusters;
  int numL3Clusters = record.numL3Clusters;

  float3 l2Min = gprt::load<float3>(record.l2clusters, 2 * l2ID + 0);
  float3 l2Max = gprt::load<float3>(record.l2clusters, 2 * l2ID + 1);
  float3 exactScale = (l2Max - l2Min) * 2.f; 
  uint3 exponent;
  exponent.x = asuint(exactScale.x) >> 23;
  exponent.y = asuint(exactScale.y) >> 23;
  exponent.z = asuint(exactScale.z) >> 23;
  uint32_t exponent32 = (exponent.z << 16) | 
                        (exponent.y <<  8) | 
                        (exponent.x <<  0);
  float4 l2Treelet;
  l2Treelet.xyz = l2Min;
  l2Treelet.w = asfloat(exponent32);
  gprt::store<float4>(record.treelets, numL3Clusters + l2ID, l2Treelet);
  
  float3 scale = float3(
    asfloat(((exponent32 >>  0) & 255) << 23),
    asfloat(((exponent32 >>  8) & 255) << 23),
    asfloat(((exponent32 >> 16) & 255) << 23)
  ) / 255.f;
  float3 translate = l2Min;

  for (uint32_t i = 0; i < BRANCHING_FACTOR; ++i) {
    uint32_t l1ID = l2ID * BRANCHING_FACTOR + i;
    if (l1ID > numL1Clusters) break;
    float3 l1Min = gprt::load<float3>(record.l1clusters, 2 * l1ID + 0);
    float3 l1Max = gprt::load<float3>(record.l1clusters, 2 * l1ID + 1);
    l1Min = (l1Min - translate) / scale;
    l1Max = (l1Max - translate) / scale;
    uint64_t child = ((uint64_t(l1Min.x) & 255ull) <<  0ull)
                   | ((uint64_t(l1Min.y) & 255ull) <<  8ull)
                   | ((uint64_t(l1Min.z) & 255ull) << 16ull)
                   | ((uint64_t(l1Max.x) & 255ull) << 24ull)
                   | ((uint64_t(l1Max.y) & 255ull) << 32ull)
                   | ((uint64_t(l1Max.z) & 255ull) << 40ull);
    gprt::store<uint64_t>(record.children, numL2Clusters + l1ID, child);   
  }
}

GPRT_COMPUTE_PROGRAM(ComputeL1Treelets, (NNAccel, record), (1,1,1)) {
  int l1ID = DispatchThreadID.x;
  if (l1ID >= record.numL1Clusters) return;

  int numL0Clusters = record.numL0Clusters;
  int numL1Clusters = record.numL1Clusters;
  int numL2Clusters = record.numL2Clusters;
  int numL3Clusters = record.numL3Clusters;

  float3 l1Min = gprt::load<float3>(record.l1clusters, 2 * l1ID + 0);
  float3 l1Max = gprt::load<float3>(record.l1clusters, 2 * l1ID + 1);
  float3 exactScale = (l1Max - l1Min) * 2.f; 
  uint3 exponent;
  exponent.x = asuint(exactScale.x) >> 23;
  exponent.y = asuint(exactScale.y) >> 23;
  exponent.z = asuint(exactScale.z) >> 23;
  uint32_t exponent32 = (exponent.z << 16) | 
                        (exponent.y <<  8) | 
                        (exponent.x <<  0);
  float4 l1Treelet;
  l1Treelet.xyz = l1Min;
  l1Treelet.w = asfloat(exponent32);
  gprt::store<float4>(record.treelets, numL3Clusters + numL2Clusters + l1ID, l1Treelet);
  
  float3 scale = float3(
    asfloat(((exponent32 >>  0) & 255) << 23),
    asfloat(((exponent32 >>  8) & 255) << 23),
    asfloat(((exponent32 >> 16) & 255) << 23)
  ) / 255.f;
  float3 translate = l1Min;

  for (uint32_t i = 0; i < BRANCHING_FACTOR; ++i) {
    uint32_t l0ID = l1ID * BRANCHING_FACTOR + i;
    if (l0ID > numL0Clusters) break;
    float3 l0Min = gprt::load<float3>(record.l0clusters, 2 * l0ID + 0);
    float3 l0Max = gprt::load<float3>(record.l0clusters, 2 * l0ID + 1);
    l0Min = (l0Min - translate) / scale;
    l0Max = (l0Max - translate) / scale;
    uint64_t child = ((uint64_t(l0Min.x) & 255ull) <<  0ull)
                   | ((uint64_t(l0Min.y) & 255ull) <<  8ull)
                   | ((uint64_t(l0Min.z) & 255ull) << 16ull)
                   | ((uint64_t(l0Max.x) & 255ull) << 24ull)
                   | ((uint64_t(l0Max.y) & 255ull) << 32ull)
                   | ((uint64_t(l0Max.z) & 255ull) << 40ull);
    gprt::store<uint64_t>(record.children, numL2Clusters + numL1Clusters + l0ID, child);   
  }
}

// for now, just assuming triangles
GPRT_COMPUTE_PROGRAM(ComputeL0Treelets, (NNAccel, record), (1,1,1)) {
  int l0ID = DispatchThreadID.x;
  if (l0ID >= record.numL0Clusters) return;

  int numPrims = record.numPrims;
  int numLeaves = record.numLeaves;
  int numL0Clusters = record.numL0Clusters;
  int numL1Clusters = record.numL1Clusters;
  int numL2Clusters = record.numL2Clusters;
  int numL3Clusters = record.numL3Clusters;

  float3 l0Min = gprt::load<float3>(record.l0clusters, 2 * l0ID + 0);
  float3 l0Max = gprt::load<float3>(record.l0clusters, 2 * l0ID + 1);
  float3 exactScale = (l0Max - l0Min) * 2.f; 
  uint3 exponent;
  exponent.x = asuint(exactScale.x) >> 23;
  exponent.y = asuint(exactScale.y) >> 23;
  exponent.z = asuint(exactScale.z) >> 23;
  uint32_t exponent32 = (exponent.z << 16) | 
                        (exponent.y <<  8) | 
                        (exponent.x <<  0);
  float4 l0Treelet;
  l0Treelet.xyz = l0Min;
  l0Treelet.w = asfloat(exponent32);
  gprt::store<float4>(record.treelets, numL3Clusters + numL2Clusters + numL1Clusters + l0ID, l0Treelet);
  
  float3 scale = float3(
    asfloat(((exponent32 >>  0) & 255) << 23),
    asfloat(((exponent32 >>  8) & 255) << 23),
    asfloat(((exponent32 >> 16) & 255) << 23)
  ) / (255.f);
  float3 translate = l0Min;

  for (uint32_t i = 0; i < BRANCHING_FACTOR; ++i) {
    uint32_t leafID = l0ID * BRANCHING_FACTOR + i;
    if (leafID > numLeaves) break;

    #ifdef ENABLE_OBBS
    uint64_t2 child;
    uint32_t numDataPoints = 0;
    float3 center = float3(0.f, 0.f, 0.f);

    // Get Center
    for (uint32_t j = 0; j < PRIMS_PER_LEAF; ++j) {
      uint32_t primID = leafID * PRIMS_PER_LEAF + j;
      if (primID >= numPrims) continue;

      uint32_t primitiveAddress = uint32_t(gprt::load<uint64_t>(record.codes, primID)); 
      if (primitiveAddress > numPrims) continue;

      bool invalid = true;
      uint3 tri = gprt::load<uint3>(record.triangles, primitiveAddress);
      float3 a, b, c;
      a = gprt::load<float3>(record.points, tri.x);
      if (isnan(a.x)) continue;
      b = gprt::load<float3>(record.points, tri.y);
      c = gprt::load<float3>(record.points, tri.z);
      numDataPoints += 3;
      center += a;
      center += b;
      center += c;
    }
    center /= (numDataPoints);

    // Compute Covariance
    float3x3 covariance = float3x3(0,0,0,0,0,0,0,0,0);
    for (uint32_t j = 0; j < PRIMS_PER_LEAF; ++j) {
      uint32_t primID = leafID * PRIMS_PER_LEAF + j;
      if (primID >= numPrims) continue;

      uint32_t primitiveAddress = uint32_t(gprt::load<uint64_t>(record.codes, primID)); 
      if (primitiveAddress > numPrims) continue;

      bool invalid = true;
      uint3 tri = gprt::load<uint3>(record.triangles, primitiveAddress);
      float3 a, b, c;
      a = gprt::load<float3>(record.points, tri.x);
      if (isnan(a.x)) continue;
      b = gprt::load<float3>(record.points, tri.y);
      c = gprt::load<float3>(record.points, tri.z);

      float3 pa = a - center;
      float3 pb = b - center;
      float3 pc = c - center;
      covariance += outerProduct(pa, pa);
      covariance += outerProduct(pb, pb);
      covariance += outerProduct(pc, pc);
    }
    covariance /= numDataPoints;

    // Compute SVD
    float3x3 rot = svd(covariance).V;
    float3 obbEul = mat3_to_eul(rot);
    
    // Quantize rotation.
    float3 qObbEul = ((obbEul + radians(180)) / radians(360)) * 1048576.f;
    child.y = ((uint64_t(qObbEul.x) & 1048575ull) <<  0ull)
            | ((uint64_t(qObbEul.y) & 1048575ull) << 20ull)
            | ((uint64_t(qObbEul.z) & 1048575ull) << 40ull);

    qObbEul = float3(((child.y >>  0ull) & 1048575ull),
                     ((child.y >> 20ull) & 1048575ull),
                     ((child.y >> 40ull) & 1048575ull));

    obbEul = ((qObbEul / 1048576.f) * radians(360)) - radians(180);
    float3x3 obbRot = eul_to_mat3(obbEul);

    // Compute OBB
    float3 obbMin =  float3(1e20f, 1e20f, 1e20f);
    float3 obbMax = -float3(1e20f, 1e20f, 1e20f);

    for (uint32_t j = 0; j < PRIMS_PER_LEAF; ++j) {
      uint32_t primID = leafID * PRIMS_PER_LEAF + j;
      if (primID >= numPrims) continue;

      uint32_t primitiveAddress = uint32_t(gprt::load<uint64_t>(record.codes, primID)); 
      if (primitiveAddress > numPrims) continue;

      bool invalid = true;
      uint3 tri = gprt::load<uint3>(record.triangles, primitiveAddress);
      float3 a, b, c;
      a = gprt::load<float3>(record.points, tri.x);
      if (isnan(a.x)) continue;
      b = gprt::load<float3>(record.points, tri.y);
      c = gprt::load<float3>(record.points, tri.z);

      float3 pa = mul(obbRot, a - center) + center;
      float3 pb = mul(obbRot, b - center) + center;
      float3 pc = mul(obbRot, c - center) + center;

      obbMin = min(obbMin, pa);
      obbMin = min(obbMin, pb);
      obbMin = min(obbMin, pc);
      obbMax = max(obbMax, pa);
      obbMax = max(obbMax, pb);
      obbMax = max(obbMax, pc);
    }

    // Quantize OBB bounds
    if (any(obbMax < obbMin)) continue;
    obbMin = (obbMin - translate) / scale;
    obbMax = (obbMax - translate) / scale;

    child.x = ((uint64_t(obbMin.x) & 255ull) <<  0ull)
            | ((uint64_t(obbMin.y) & 255ull) <<  8ull)
            | ((uint64_t(obbMin.z) & 255ull) << 16ull)
            | ((uint64_t(obbMax.x) & 255ull) << 24ull)
            | ((uint64_t(obbMax.y) & 255ull) << 32ull)
            | ((uint64_t(obbMax.z) & 255ull) << 40ull);

    gprt::store<uint64_t>(record.children, numL2Clusters + numL1Clusters + numL0Clusters + leafID * 2 + 0, child.x);   
    gprt::store<uint64_t>(record.children, numL2Clusters + numL1Clusters + numL0Clusters + leafID * 2 + 1, child.y);   

    #else
    float3 aabbMin = float3(1e20f, 1e20f, 1e20f);
    float3 aabbMax = -float3(1e20f, 1e20f, 1e20f);

    for (uint32_t j = 0; j < PRIMS_PER_LEAF; ++j) {
      uint32_t primID = leafID * PRIMS_PER_LEAF + j;
      if (primID >= numPrims) continue;

      uint32_t primitiveAddress = uint32_t(gprt::load<uint64_t>(record.codes, primID)); 
      if (primitiveAddress > numPrims) continue;
      
      float3 primAabbMin = gprt::load<float3>(record.primBounds, primitiveAddress * 2 + 0);
      float3 primAabbMax = gprt::load<float3>(record.primBounds, primitiveAddress * 2 + 1);
      aabbMin = min(aabbMin, primAabbMin);
      aabbMax = max(aabbMax, primAabbMax);
    }
    if (any(aabbMax < aabbMin)) continue;

    aabbMin = (aabbMin - translate) / scale;
    aabbMax = (aabbMax - translate) / scale;
    uint64_t child = ((uint64_t(aabbMin.x) & 255ull) <<  0ull)
                  | ((uint64_t(aabbMin.y) & 255ull) <<  8ull)
                  | ((uint64_t(aabbMin.z) & 255ull) << 16ull)
                  | ((uint64_t(aabbMax.x) & 255ull) << 24ull)
                  | ((uint64_t(aabbMax.y) & 255ull) << 32ull)
                  | ((uint64_t(aabbMax.z) & 255ull) << 40ull);
    gprt::store<uint64_t>(record.children, numL2Clusters + numL1Clusters + numL0Clusters + leafID, child);
    #endif
  }
}

    // uint32_t exponent32 = asuint(l3Treelet.w);
    // // float3 scale = float3(
    // //   asfloat(((exponent32 >>  0) & 255) << 23) * 256.f,
    // //   asfloat(((exponent32 >>  8) & 255) << 23) * 256.f,
    // //   asfloat(((exponent32 >> 16) & 255) << 23) * 256.f
    // // );
    // // float3 translate = l3Treelet.xyz;

    // if (l3ID == 0 && i == 0) {
    //   printf("Decoded translate: %f, %f, %f scale: %f %f %f\n", 
    //     translate.x, translate.y, translate.z, 
    //     scale.x, scale.y, scale.z);
    // }

    // float3 lo = float3( (child >>  0ull) & 255, 
    //                     (child >>  8ull) & 255, 
    //                     (child >> 16ull) & 255);
    // float3 hi = float3(((child >> 24ull) & 255) + 1u, 
    //                    ((child >> 32ull) & 255) + 1u, 
    //                    ((child >> 40ull) & 255) + 1u);
    // lo = lo * scale + translate;
    // hi = hi * scale + translate;
    // float3 error = scale * float(1u);

    // if (l3ID == 0 && i == 0) {
    //   printf("Decompressed min: %f %f %f, max %f %f %f\n", 
    //     lo.x, lo.y, lo.z, 
    //     hi.x, hi.y, hi.z);
    // }

    // if (l3ID == 0 && i == 0) {
    //   printf("Error: %f %f %f\n", 
    //     error.x, error.y, error.z);
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

GPRT_COMPUTE_PROGRAM(ComputeAABBCodes, (NNAccel, record), (1, 1, 1)) {
  int aabbID = DispatchThreadID.x;
  if (aabbID >= record.numL3Clusters) return;

  float3 aabbMin = gprt::load<float3>(record.l3clusters, aabbID * 2 + 0);
  float3 aabbMax = gprt::load<float3>(record.l3clusters, aabbID * 2 + 1);
  
  if (isnan(aabbMin.x) || all(aabbMax < aabbMin)) {
    gprt::store<uint>(record.codes, aabbID, -1);
    return;
  }
  float3 c = (aabbMax + aabbMin) * .5f;
  
  float3 gAabbMin = gprt::load<float3>(record.aabb, 0);
  float3 gAabbMax = gprt::load<float3>(record.aabb, 1);
  c = (c - gAabbMin) / (gAabbMax - gAabbMin);

  uint64_t code = uint64_t(morton_encode3D(c.x, c.y, c.z));
  uint64_t lbvhId = uint64_t(aabbID);
  
  gprt::store<uint64_t>(record.lbvhMortonCodes, aabbID, code);
  gprt::store<uint64_t>(record.lbvhIds, aabbID, lbvhId);
}

GPRT_COMPUTE_PROGRAM(MakeNodes, (NNAccel, record), (1, 1, 1)) {
  int nodeID = DispatchThreadID.x;
  if (nodeID >= (record.numL3Clusters * 2 - 1)) return;
  storeNode(record.lbvhNodes, nodeID, int4(-1,-1,-1,-1));
  gprt::store<float3>(record.lbvhAabbs, nodeID * 2 + 0,  float3(1e20f, 1e20f, 1e20f));
  gprt::store<float3>(record.lbvhAabbs, nodeID * 2 + 1, -float3(1e20f, 1e20f, 1e20f));
}

GPRT_COMPUTE_PROGRAM(SplitNodes, (NNAccel, record), (1, 1, 1)) {
  int num_leaves = record.numL3Clusters;
  int num_codes = record.numL3Clusters;
  gprt::Buffer morton_codes = record.lbvhMortonCodes;
  int index = DispatchThreadID.x;
  int numInner = record.numL3Clusters - 1;

  if (index < numInner)
  {
    // NOTE: This is [first..last], not [first..last)!!
    int split = -1;
    int2 range = determine_range(morton_codes, num_codes, index, split);
    int first = range.x;
    int last = range.y;

    int left = split;
    int right = split + 1;

    if (left == first)
    {
      // left child is leaf
      // inner[index].left = num_inner + left;
      storeNodeLeft(record.lbvhNodes, index, numInner + left);
      // leaves[left].parent = index;
      storeNodeParent(record.lbvhNodes, numInner + left, index);
    }
    else
    {
      // left child is inner
      // inner[index].left = left;
      storeNodeLeft(record.lbvhNodes, index, left);
      // inner[left].parent = index;
      storeNodeParent(record.lbvhNodes, left, index);
    }

    if (right == last)
    {
      // right child is leaf
      // inner[index].right = num_inner + right;
      storeNodeRight(record.lbvhNodes, index, numInner + right);
      // leaves[right].parent = index;
      storeNodeParent(record.lbvhNodes, numInner + right, index);
    }
    else
    {
      // right child is inner
      // inner[index].right = right;
      storeNodeRight(record.lbvhNodes, index, right);
      // inner[right].parent = index;
      storeNodeParent(record.lbvhNodes, right, index);
    }
  }
}

GPRT_COMPUTE_PROGRAM(BuildHierarchy, (NNAccel, record), (1, 1, 1)) {
  int index = DispatchThreadID.x;
  uint numLeaves = record.numL3Clusters;
  if (index >= numLeaves) return;

  uint id = uint32_t(gprt::load<uint64_t>(record.lbvhIds, index));
  float3 aabbMin = gprt::load<float3>(record.l3clusters, id * 2 + 0);
  float3 aabbMax = gprt::load<float3>(record.l3clusters, id * 2 + 1);
  // getTriangleBounds(record.triangles, record.positions, id, aabbMin, aabbMax); // Change to AABBs

  // Leaf's bounding box
  int leafAddr = index + (record.numL3Clusters - 1);
  gprt::store<float3>(record.lbvhAabbs, leafAddr * 2 + 0, aabbMin);
  gprt::store<float3>(record.lbvhAabbs, leafAddr * 2 + 1, aabbMax);

  // Leaf's object
  storeNodeLeaf(record.lbvhNodes, leafAddr, id);
  
  // Atomically combine child bounding boxes and update parents
  int next = loadNodeParent(record.lbvhNodes, leafAddr);
  while (next >= 0) {
    gprt::atomicMin32f(record.lbvhAabbs, next * 6 + 0, aabbMin.x);
    gprt::atomicMin32f(record.lbvhAabbs, next * 6 + 1, aabbMin.y);
    gprt::atomicMin32f(record.lbvhAabbs, next * 6 + 2, aabbMin.z);
    gprt::atomicMax32f(record.lbvhAabbs, next * 6 + 3, aabbMax.x);
    gprt::atomicMax32f(record.lbvhAabbs, next * 6 + 4, aabbMax.y);
    gprt::atomicMax32f(record.lbvhAabbs, next * 6 + 5, aabbMax.z);
    next = loadNodeParent(record.lbvhNodes, next);   
  }
}

struct ClosestPointAttributes {
  int minDist;
}; 

GPRT_INTERSECTION_PROGRAM(ClosestNeighborIntersection, (NNAccel, record)) {
  uint leafID = PrimitiveIndex();
  float3 aabbMin = gprt::load<float3>(record.numL3Clusters, 2 * leafID + 0) + record.maxSearchRange;
  float3 aabbMax = gprt::load<float3>(record.numL3Clusters, 2 * leafID + 1) - record.maxSearchRange;
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
  uint leafID = PrimitiveIndex();
  float3 queryOrigin = WorldRayOrigin();

  // Out of range
  if (hitLeaf.minDist > payload.closestDistance) {
    gprt::ignoreHit();
    return;
  }

  TraverseLeaf(record, RAY_FLAG_NONE, queryOrigin, 0.f, record.maxSearchRange, leafID, payload, false);
  
  gprt::ignoreHit(); // forces traversal to continue to next supercluster
}

