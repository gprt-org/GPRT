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

struct KNNAccelData {
  // input
  alignas(4) uint32_t numPrims;
  alignas(4) uint32_t numClusters;
  alignas(4) uint32_t numSuperClusters;
  alignas(4) uint32_t numPrimsPerCluster;
  alignas(4) uint32_t numClustersPerSuperCluster;
  alignas(4) float maxSearchRange;

  alignas(16) gprt::Buffer points; 
  alignas(16) gprt::Buffer edges; 
  alignas(16) gprt::Buffer triangles;

  // Hilbert codes of quantized primitive centroids
  // One uint32_t per primitive
  alignas(16) gprt::Buffer hilbertCodes;

  // Primitive IDs that correspond to sorted hilbert codes. 
  // One uint32_t per primitive
  alignas(16) gprt::Buffer ids;

  // Buffer containing the global AABB. Pair of two floats
  alignas(16) gprt::Buffer aabb;

  // Buffers of AABBs. Each aabb is a pair of float3.
  alignas(16) gprt::Buffer clusters;
  alignas(16) gprt::Buffer superClusters;
};

#ifndef GPRT_DEVICE
#include <limits.h>
#include <stdexcept>
extern GPRTProgram knnDeviceCode;

typedef enum {
    GPRT_KNN_TYPE_POINTS = 1,
    GPRT_KNN_TYPE_EDGES = 2,
    GPRT_KNN_TYPE_TRIANGLES = 3,
} GPRTKNNType;

struct GPRTKNNAccel {
    KNNAccelData handle;
    GPRTModule module;
    
    GPRTKNNType type;

    GPRTBufferOf<float3> points;
    GPRTBufferOf<uint2> edges;
    GPRTBufferOf<uint3> triangles;

    GPRTBufferOf<uint32_t> hilbertCodes;
    GPRTBufferOf<uint32_t> ids;
    GPRTBufferOf<float3> aabb;
    GPRTBufferOf<float3> clusters;
    GPRTBufferOf<float3> superClusters;

    GPRTBufferOf<uint8_t> scratch;

    GPRTComputeOf<KNNAccelData> computePointBounds;
    GPRTComputeOf<KNNAccelData> computeEdgeBounds;
    GPRTComputeOf<KNNAccelData> computeTriangleBounds;
    GPRTComputeOf<KNNAccelData> computePointClusters;
    GPRTComputeOf<KNNAccelData> computeEdgeClusters;
    GPRTComputeOf<KNNAccelData> computeTriangleClusters;
    GPRTComputeOf<KNNAccelData> computeSuperClusters;
    GPRTComputeOf<KNNAccelData> computePointHilbertCodes;
    GPRTComputeOf<KNNAccelData> computeEdgeHilbertCodes;
    GPRTComputeOf<KNNAccelData> computeTriangleHilbertCodes;

    // Set to true to enable validation.
    bool _testing = false;
};

/** 
 * Creates an acceleration structure that supports nearest neighbor queries
 * @param GPRTContext The GPRT context
 * @param type The type of nearest neighbor queries to be supported. 
 *  GPRT_KNN_TYPE_POINTS is for closest point queries. In this case, edges and triangles are expected to be nullptr.
 *  GPRT_KNN_TYPE_EDGES is for closest-point-on-edge queries. In this case, triangles is expected to be nullptr.
 *  GPRT_KNN_TYPE_TRIANGLES is for closest-point-on-triangle queries. In this case, edges is expected to be nullptr.
 * @param points A buffer of positions in space.
 * @param edges A buffer of indices connecting positions to form edges, or null if unused.
 * @param triangles A buffer of indices connecting positions to form triangles, or null if unused. 
 * @param count The number of primitives (either points, edges or triangles depending on the type)
 * @param maxSearchRange The largest possible search radius
*/
inline 
GPRTKNNAccel gprtKNNAccelCreate(
  GPRTContext context,
  GPRTKNNType type,
  GPRTBufferOf<float3> points, 
  GPRTBufferOf<uint2> edges, 
  GPRTBufferOf<uint3> triangles, 
  uint32_t count,
  float maxSearchRange) 
{
  GPRTKNNAccel knnAccel;
  knnAccel.type = type;
  knnAccel.module = gprtModuleCreate(context, knnDeviceCode);

  knnAccel.computePointBounds = gprtComputeCreate<KNNAccelData>(context, knnAccel.module, "ComputePointBounds");
  knnAccel.computePointClusters = gprtComputeCreate<KNNAccelData>(context, knnAccel.module, "ComputePointClusters");
  knnAccel.computePointHilbertCodes = gprtComputeCreate<KNNAccelData>(context, knnAccel.module, "ComputePointHilbertCodes");

  knnAccel.computeEdgeBounds = gprtComputeCreate<KNNAccelData>(context, knnAccel.module, "ComputeEdgeBounds");
  knnAccel.computeEdgeClusters = gprtComputeCreate<KNNAccelData>(context, knnAccel.module, "ComputeEdgeClusters");
  knnAccel.computeEdgeHilbertCodes = gprtComputeCreate<KNNAccelData>(context, knnAccel.module, "ComputeEdgeHilbertCodes");

  knnAccel.computeTriangleBounds = gprtComputeCreate<KNNAccelData>(context, knnAccel.module, "ComputeTriangleBounds");
  knnAccel.computeTriangleClusters = gprtComputeCreate<KNNAccelData>(context, knnAccel.module, "ComputeTriangleClusters");
  knnAccel.computeTriangleHilbertCodes = gprtComputeCreate<KNNAccelData>(context, knnAccel.module, "ComputeTriangleHilbertCodes");

  knnAccel.computeSuperClusters = gprtComputeCreate<KNNAccelData>(context, knnAccel.module, "ComputeSuperClusters");

  knnAccel.handle.numPrims = count;
  knnAccel.handle.numPrimsPerCluster = 16;
  knnAccel.handle.numClustersPerSuperCluster = 16;
  knnAccel.handle.maxSearchRange = maxSearchRange;

  // some logic here for rounding up
  knnAccel.handle.numClusters = (knnAccel.handle.numPrims + (knnAccel.handle.numPrimsPerCluster - 1)) / knnAccel.handle.numPrimsPerCluster;
  knnAccel.handle.numSuperClusters = (knnAccel.handle.numClusters + (knnAccel.handle.numClustersPerSuperCluster - 1)) / knnAccel.handle.numClustersPerSuperCluster;

  // cache these...
  knnAccel.points = points;
  knnAccel.edges = edges;
  knnAccel.triangles = triangles;

  // create these
  knnAccel.hilbertCodes = gprtDeviceBufferCreate<uint32_t>(context, knnAccel.handle.numPrims);
  knnAccel.ids = gprtDeviceBufferCreate<uint32_t>(context, knnAccel.handle.numPrims);
  knnAccel.aabb = gprtDeviceBufferCreate<float3>(context, 2);
  knnAccel.clusters = gprtDeviceBufferCreate<float3>(context, 2 * knnAccel.handle.numClusters);
  knnAccel.superClusters = gprtDeviceBufferCreate<float3>(context, 2 * knnAccel.handle.numSuperClusters);
  
  // to be resized as needed
  knnAccel.scratch = gprtDeviceBufferCreate<uint8_t>(context);

  knnAccel.handle.points = gprtBufferGetHandle(knnAccel.points);
  if (type == GPRT_KNN_TYPE_EDGES)
    knnAccel.handle.edges = gprtBufferGetHandle(knnAccel.edges);
  if (type == GPRT_KNN_TYPE_TRIANGLES)
    knnAccel.handle.triangles = gprtBufferGetHandle(knnAccel.triangles);

  knnAccel.handle.hilbertCodes = gprtBufferGetHandle(knnAccel.hilbertCodes);
  knnAccel.handle.ids = gprtBufferGetHandle(knnAccel.ids);
  knnAccel.handle.aabb = gprtBufferGetHandle(knnAccel.aabb);
  knnAccel.handle.clusters = gprtBufferGetHandle(knnAccel.clusters);
  knnAccel.handle.superClusters = gprtBufferGetHandle(knnAccel.superClusters);

  gprtComputeSetParameters(knnAccel.computePointBounds, &knnAccel.handle);
  gprtComputeSetParameters(knnAccel.computePointClusters, &knnAccel.handle);
  gprtComputeSetParameters(knnAccel.computePointHilbertCodes, &knnAccel.handle);

  gprtComputeSetParameters(knnAccel.computeEdgeBounds, &knnAccel.handle);
  gprtComputeSetParameters(knnAccel.computeEdgeClusters, &knnAccel.handle);
  gprtComputeSetParameters(knnAccel.computeEdgeHilbertCodes, &knnAccel.handle);

  gprtComputeSetParameters(knnAccel.computeTriangleBounds, &knnAccel.handle);
  gprtComputeSetParameters(knnAccel.computeTriangleClusters, &knnAccel.handle);
  gprtComputeSetParameters(knnAccel.computeTriangleHilbertCodes, &knnAccel.handle);
  
  gprtComputeSetParameters(knnAccel.computeSuperClusters, &knnAccel.handle);
  

  gprtBuildShaderBindingTable(context, GPRT_SBT_COMPUTE);

  return knnAccel;
};

// We might change to an API like this in the future...
// inline void gprtPointKNNAccelSetPoints();
// inline void gprtEdgeKNNAccelSetIndices();
// inline void gprtTriangleKNNAccelSetIndices();

inline void gprtKNNAccelBuild(GPRTContext context, GPRTKNNAccel &knnAccel)
{
  typedef uint32_t uint;

  // initialize root AABB
  gprtBufferMap(knnAccel.aabb);
  float3* aabbPtr = gprtBufferGetPointer(knnAccel.aabb);
  aabbPtr[0].x = aabbPtr[0].y = aabbPtr[0].z = std::numeric_limits<float>::max();
  aabbPtr[1].x = aabbPtr[1].y = aabbPtr[1].z = -std::numeric_limits<float>::max();
  gprtBufferUnmap(knnAccel.aabb);

  // Compute the global bounds
  if (knnAccel.type == GPRT_KNN_TYPE_POINTS) {
    gprtComputeLaunch1D(context, knnAccel.computePointBounds, knnAccel.handle.numPrims);
  } else if (knnAccel.type == GPRT_KNN_TYPE_EDGES) {
    gprtComputeLaunch1D(context, knnAccel.computeEdgeBounds, knnAccel.handle.numPrims);
  } else if (knnAccel.type == GPRT_KNN_TYPE_TRIANGLES) {
    gprtComputeLaunch1D(context, knnAccel.computeTriangleBounds, knnAccel.handle.numPrims);
  }

  if (knnAccel._testing) {
    float3 aabbMin = float3(1e20f, 1e20f, 1e20f);
    float3 aabbMax = float3(-1e20f, -1e20f, -1e20f);

    gprtBufferMap(knnAccel.points);
    float3* points = gprtBufferGetPointer(knnAccel.points);
    uint2* edges = nullptr;
    uint3* triangles = nullptr;

    if (knnAccel.type == GPRT_KNN_TYPE_EDGES) {
      gprtBufferMap(knnAccel.edges);
      edges = gprtBufferGetPointer(knnAccel.edges);
    } else if (knnAccel.type == GPRT_KNN_TYPE_TRIANGLES) {
      gprtBufferMap(knnAccel.triangles);
      triangles = gprtBufferGetPointer(knnAccel.triangles);
    }

    for (uint32_t i = 0; i < knnAccel.handle.numPrims; ++i) {
      if (knnAccel.type == GPRT_KNN_TYPE_POINTS) {
        // inactive point
        if (isnan(points[i].x)) continue;
        aabbMin = linalg::min(aabbMin, points[i]);
        aabbMax = linalg::max(aabbMax, points[i]);
      } else if (knnAccel.type == GPRT_KNN_TYPE_EDGES) {
        uint2 edge = edges[i];
        float3 p1 = points[edge.x];
        float3 p2 = points[edge.y];
        // inactive edge
        if (isnan(p1.x)) continue;
        aabbMin = linalg::min(aabbMin, p1);
        aabbMax = linalg::max(aabbMax, p1);
        aabbMin = linalg::min(aabbMin, p2);
        aabbMax = linalg::max(aabbMax, p2);
      } else if (knnAccel.type == GPRT_KNN_TYPE_TRIANGLES) {
        uint3 triangle = triangles[i];
        float3 p1 = points[triangle.x];
        float3 p2 = points[triangle.y];
        float3 p3 = points[triangle.y];
        // inactive triangle
        if (isnan(p1.x)) continue;
        aabbMin = linalg::min(aabbMin, p1);
        aabbMax = linalg::max(aabbMax, p1);
        aabbMin = linalg::min(aabbMin, p2);
        aabbMax = linalg::max(aabbMax, p2);
      }
    }

    gprtBufferUnmap(knnAccel.points);

    if (knnAccel.type == GPRT_KNN_TYPE_EDGES) {
      gprtBufferUnmap(knnAccel.edges);
    } else if (knnAccel.type == GPRT_KNN_TYPE_TRIANGLES) {
      gprtBufferUnmap(knnAccel.triangles);
    }

    gprtBufferMap(knnAccel.aabb);
    float3* deviceAABB = gprtBufferGetPointer(knnAccel.aabb);
    if (!all(equal(deviceAABB[0], aabbMin))) throw std::runtime_error("incorrect minimum bounds");
    if (!all(equal(deviceAABB[1], aabbMax))) throw std::runtime_error("incorrect maximum bounds");
  }

  // Use global bounds to compute hilbert codes
  if (knnAccel.type == GPRT_KNN_TYPE_POINTS) {
    gprtComputeLaunch1D(context, knnAccel.computePointHilbertCodes, knnAccel.handle.numPrims);
  } else if (knnAccel.type == GPRT_KNN_TYPE_EDGES) {
    gprtComputeLaunch1D(context, knnAccel.computeEdgeHilbertCodes, knnAccel.handle.numPrims);
  } else if (knnAccel.type == GPRT_KNN_TYPE_TRIANGLES) {
    gprtComputeLaunch1D(context, knnAccel.computeTriangleHilbertCodes, knnAccel.handle.numPrims);
  }

  // Disabling this test... Some differences with GPU and CPU floats cause slightly different codes
  // every now and then...

  // if (knnAccel._testing) {
  //   /* define the bitmask_t type as an integer of sufficient size */
  //   typedef uint64_t bitmask_t;
  //   /* define the halfmask_t type as an integer of 1/2 the size of bitmask_t */
  //   typedef uint32_t halfmask_t;

  //   /* implementation of the hilbert functions */
  //   #define adjust_rotation(rotation,nDims,bits)                            \
  //   do {                                                                    \
  //         /* rotation = (rotation + 1 + ffs(bits)) % nDims; */              \
  //         bits &= -bits & nd1Ones;                                          \
  //         while (bits)                                                      \
  //           bits >>= 1, ++rotation;                                         \
  //         if ( ++rotation >= nDims )                                        \
  //           rotation -= nDims;                                              \
  //   } while (0)

  //   #define ones(T,k) ((((T)2) << (k-1)) - 1)

  //   #define rdbit(w,k) (((w) >> (k)) & 1)
        
  //   #define rotateRight(arg, nRots, nDims)                                  \
  //   ((((arg) >> (nRots)) | ((arg) << ((nDims)-(nRots)))) & ones(bitmask_t,nDims))

  //   #define rotateLeft(arg, nRots, nDims)                                   \
  //   ((((arg) << (nRots)) | ((arg) >> ((nDims)-(nRots)))) & ones(bitmask_t,nDims))

  //   auto bitTranspose = [&](uint32_t nDims, uint32_t nBits, bitmask_t inCoords) -> bitmask_t
  //   {
  //     uint32_t const nDims1 = nDims-1;
  //     uint32_t inB = nBits;
  //     uint32_t utB;
  //     bitmask_t inFieldEnds = 1;
  //     bitmask_t inMask = ones(bitmask_t,inB);
  //     bitmask_t coords = 0;

  //     while ((utB = inB / 2))
  //       {
  //         uint32_t const shiftAmt = nDims1 * utB;
  //         bitmask_t const utFieldEnds =
  //     inFieldEnds | (inFieldEnds << (shiftAmt+utB));
  //         bitmask_t const utMask =
  //     (utFieldEnds << utB) - utFieldEnds;
  //         bitmask_t utCoords = 0;
  //         uint32_t d;
  //         if (inB & 1)
  //     {
  //       bitmask_t const inFieldStarts = inFieldEnds << (inB-1);
  //       uint32_t oddShift = 2*shiftAmt;
  //       for (d = 0; d < nDims; ++d)
  //         {
  //           bitmask_t temp = inCoords & inMask;
  //           inCoords >>= inB;
  //           coords |= (temp & inFieldStarts) <<	oddShift++;
  //           temp &= ~inFieldStarts;
  //           temp = (temp | (temp << shiftAmt)) & utMask;
  //           utCoords |= temp << (d*utB);
  //         }
  //     }
  //         else
  //     {
  //       for (d = 0; d < nDims; ++d)
  //         {
  //           bitmask_t temp = inCoords & inMask;
  //           inCoords >>= inB;
  //           temp = (temp | (temp << shiftAmt)) & utMask;
  //           utCoords |= temp << (d*utB);
  //         }
  //     }
  //         inCoords = utCoords;
  //         inB = utB;
  //         inFieldEnds = utFieldEnds;
  //         inMask = utMask;
  //       }
  //     coords |= inCoords;
  //     return coords;
  //   };

  //   auto hilbert_c2i = [&](uint32_t nBits, bitmask_t const coord[3]) -> bitmask_t
  //   {
  //     int nDims = 3;
  //     if (nDims > 1)
  //       {
  //         uint32_t const nDimsBits = nDims*nBits;
  //         bitmask_t index;
  //         uint32_t d;
  //         bitmask_t coords = 0;
  //         for (d = nDims; d--; )
  //     {
  //       coords <<= nBits;
  //       coords |= coord[d];
  //     }

  //         if (nBits > 1)
  //     {
  //       halfmask_t const ndOnes = ones(halfmask_t,nDims);
  //       halfmask_t const nd1Ones= ndOnes >> 1; /* for adjust_rotation */
  //       uint32_t b = nDimsBits;
  //       uint32_t rotation = 0;
  //       halfmask_t flipBit = 0;
  //       bitmask_t const nthbits = ones(bitmask_t,nDimsBits) / ndOnes;
  //       coords = bitTranspose(nDims, nBits, coords);
  //       coords ^= coords >> nDims;
  //       index = 0;
  //       do
  //         {
  //           halfmask_t bits = (halfmask_t)((coords >> (b-=nDims)) & ndOnes);
  //           bits = (halfmask_t)rotateRight(flipBit ^ bits, rotation, nDims);
  //           index <<= nDims;
  //           index |= bits;
  //           flipBit = (halfmask_t)1 << rotation;
  //           adjust_rotation(rotation,nDims,bits);
  //         } while (b);
  //       index ^= nthbits >> 1;
  //     }
  //         else
  //     index = coords;
  //         for (d = 1; d < nDimsBits; d *= 2)
  //     index ^= index >> d;
  //         return index;
  //       }
  //     else
  //       return coord[0];
  //   };

  //   auto hilbert_encode3D = [&](float x, float y, float z) -> uint32_t
  //   {
  //     x = x * (float)(1 << 10);
  //     y = y * (float)(1 << 10);
  //     z = z * (float)(1 << 10);
  //     const bitmask_t coord[3] = {bitmask_t(x), bitmask_t(y), bitmask_t(z)};
  //     return uint32_t(hilbert_c2i(10, coord));
  //   };
  
  //   gprtBufferMap(knnAccel.aabb);
  //   float3* deviceAABB = gprtBufferGetPointer(knnAccel.aabb);

  //   gprtBufferMap(knnAccel.hilbertCodes);
  //   uint32_t* hilbertCodes = gprtBufferGetPointer(knnAccel.hilbertCodes);

  //   gprtBufferMap(knnAccel.ids);
  //   uint32_t* ids = gprtBufferGetPointer(knnAccel.ids);
    
  //   gprtBufferMap(knnAccel.points);
  //   float3* points = gprtBufferGetPointer(knnAccel.points);
  //   uint2* edges = nullptr;
  //   uint3* triangles = nullptr;

  //   if (knnAccel.type == GPRT_KNN_TYPE_EDGES) {
  //     gprtBufferMap(knnAccel.edges);
  //     edges = gprtBufferGetPointer(knnAccel.edges);
  //   } else if (knnAccel.type == GPRT_KNN_TYPE_TRIANGLES) {
  //     gprtBufferMap(knnAccel.triangles);
  //     triangles = gprtBufferGetPointer(knnAccel.triangles);
  //   }

  //   for (uint32_t i = 0; i < knnAccel.handle.numPrims; ++i) {
  //     float3 pt;
  //     if (knnAccel.type == GPRT_KNN_TYPE_POINTS) {
  //       // inactive point
  //       if (isnan(points[i].x)) {
  //         if (hilbertCodes[i] != -1) throw std::runtime_error("Incorrect hilbert code!");
  //         continue;
  //       };
  //       pt = points[i];
  //     } else if (knnAccel.type == GPRT_KNN_TYPE_EDGES) {
  //       uint2 edge = edges[i];
  //       float3 p1 = points[edge.x];
  //       float3 p2 = points[edge.y];
  //       // inactive edge
  //       if (isnan(p1.x)) {
  //         if (hilbertCodes[i] != -1) throw std::runtime_error("Incorrect hilbert code!");
  //         continue;
  //       }
  //       pt = (p1 + p2) / 2.f;
  //     } else if (knnAccel.type == GPRT_KNN_TYPE_TRIANGLES) {
  //       uint3 triangle = triangles[i];
  //       float3 p1 = points[triangle.x];
  //       float3 p2 = points[triangle.y];
  //       float3 p3 = points[triangle.y];
  //       // inactive triangle
  //       if (isnan(p1.x)) {
  //         if (hilbertCodes[i] != -1) throw std::runtime_error("Incorrect hilbert code!");
  //         continue;
  //       }
  //       pt = (p1 + p2 + p3) / 3.f;
  //     }
  //     float3 aabbMin = deviceAABB[0];
  //     float3 aabbMax = deviceAABB[1];
  //     pt = (pt - aabbMin) / (aabbMax - aabbMin);
  //     // quantize to 10 bits
  //     pt = linalg::min(linalg::max(pt * 1024.f, float3(0.f, 0.f, 0.f)), float3(1023.f, 1023.f, 1023.f));

  //     uint32_t code = hilbert_encode3D(pt.x, pt.y, pt.z);
  //     if (hilbertCodes[i] != code)  throw std::runtime_error("Incorrect hilbert code!");
  //     if (ids[i] != i) throw std::runtime_error("Incorrect ID!");
  //   }

  //   gprtBufferUnmap(knnAccel.points);
  //   if (knnAccel.type == GPRT_KNN_TYPE_EDGES) {
  //     gprtBufferUnmap(knnAccel.edges);
  //   } else if (knnAccel.type == GPRT_KNN_TYPE_TRIANGLES) {
  //     gprtBufferUnmap(knnAccel.triangles);
  //   }
  //   gprtBufferUnmap(knnAccel.ids);
  //   gprtBufferUnmap(knnAccel.hilbertCodes);
  //   gprtBufferUnmap(knnAccel.aabb);
  // }

  // Sort the primitive references by hilbert codes
  gprtBufferSortPayload(context, knnAccel.hilbertCodes, knnAccel.ids, knnAccel.scratch);

  // Now compute cluster bounding boxes...
  if (knnAccel.type == GPRT_KNN_TYPE_POINTS) {
    gprtComputeLaunch1D(context, knnAccel.computePointClusters, knnAccel.handle.numClusters);
  } else if (knnAccel.type == GPRT_KNN_TYPE_EDGES) {
    gprtComputeLaunch1D(context, knnAccel.computeEdgeClusters, knnAccel.handle.numClusters);
  } else if (knnAccel.type == GPRT_KNN_TYPE_TRIANGLES) {
    gprtComputeLaunch1D(context, knnAccel.computeTriangleClusters, knnAccel.handle.numClusters);
  }

  if (knnAccel._testing) {
    gprtBufferMap(knnAccel.clusters);
    gprtBufferMap(knnAccel.ids);
    float3* clusters = gprtBufferGetPointer(knnAccel.clusters);
    uint32_t* ids = gprtBufferGetPointer(knnAccel.ids);

    gprtBufferMap(knnAccel.points);
    float3* points = gprtBufferGetPointer(knnAccel.points);
    uint2* edges = nullptr;
    uint3* triangles = nullptr;

    if (knnAccel.type == GPRT_KNN_TYPE_EDGES) {
      gprtBufferMap(knnAccel.edges);
      edges = gprtBufferGetPointer(knnAccel.edges);
    } else if (knnAccel.type == GPRT_KNN_TYPE_TRIANGLES) {
      gprtBufferMap(knnAccel.triangles);
      triangles = gprtBufferGetPointer(knnAccel.triangles);
    }

    for (uint32_t i = 0; i < knnAccel.handle.numClusters; ++i) {
      float3 aabbMin = clusters[i * 2 + 0];
      float3 aabbMax = clusters[i * 2 + 1];
      for (uint32_t j = 0; j < knnAccel.handle.numPrimsPerCluster; ++j) {
        uint32_t idx = i * knnAccel.handle.numPrimsPerCluster + j;
        uint32_t primID = ids[idx];
        if (primID >= knnAccel.handle.numPrims || primID == -1) continue;

        if (knnAccel.type == GPRT_KNN_TYPE_POINTS) {
          // inactive point
          if (isnan(points[primID].x)) continue;
          if (any(greater(points[primID], aabbMax)) || any(less(points[primID], aabbMin))) throw std::runtime_error("Primitive out of bounds!");
        } else if (knnAccel.type == GPRT_KNN_TYPE_EDGES) {
          uint2 edge = edges[primID];
          float3 p1 = points[edge.x];
          float3 p2 = points[edge.y];
          // inactive edge
          if (isnan(p1.x)) continue;
          if (any(greater(p1, aabbMax)) || any(less(p1, aabbMin))) throw std::runtime_error("Primitive out of bounds!");
          if (any(greater(p2, aabbMax)) || any(less(p2, aabbMin))) throw std::runtime_error("Primitive out of bounds!");
        } else if (knnAccel.type == GPRT_KNN_TYPE_TRIANGLES) {
          uint3 triangle = triangles[primID];
          float3 p1 = points[triangle.x];
          float3 p2 = points[triangle.y];
          float3 p3 = points[triangle.y];
          // inactive triangle
          if (isnan(p1.x)) continue;
          if (any(greater(p1, aabbMax)) || any(less(p1, aabbMin))) throw std::runtime_error("Primitive out of bounds!");
          if (any(greater(p2, aabbMax)) || any(less(p2, aabbMin))) throw std::runtime_error("Primitive out of bounds!");
          if (any(greater(p3, aabbMax)) || any(less(p3, aabbMin))) throw std::runtime_error("Primitive out of bounds!");
        }
      }
    }
    
    gprtBufferUnmap(knnAccel.points);

    if (knnAccel.type == GPRT_KNN_TYPE_EDGES) {
      gprtBufferUnmap(knnAccel.edges);
    } else if (knnAccel.type == GPRT_KNN_TYPE_TRIANGLES) {
      gprtBufferUnmap(knnAccel.triangles);
    }

    gprtBufferUnmap(knnAccel.clusters);
  }

  // ... and then compute super cluster bounding boxes
  gprtComputeLaunch1D(context, knnAccel.computeSuperClusters, knnAccel.handle.numSuperClusters);

  if (knnAccel._testing) {
    gprtBufferMap(knnAccel.superClusters);
    gprtBufferMap(knnAccel.clusters);
    float3* superClusters = gprtBufferGetPointer(knnAccel.superClusters);
    float3* clusters = gprtBufferGetPointer(knnAccel.clusters);
    
    for (uint32_t i = 0; i < knnAccel.handle.numSuperClusters; ++i) {
      float3 aabbMin = superClusters[i * 2 + 0];
      float3 aabbMax = superClusters[i * 2 + 1];
      for (uint32_t j = 0; j < knnAccel.handle.numClustersPerSuperCluster; ++j) {
        uint32_t idx = i * knnAccel.handle.numClustersPerSuperCluster + j;
        if (idx >= knnAccel.handle.numClusters) continue;

        float3 cAABBMin = clusters[idx * 2 + 0];
        float3 cAABBMax = clusters[idx * 2 + 1];

        // invalid box
        if (any(less(cAABBMax, cAABBMin))) continue;

        if (any(greater(cAABBMax, aabbMax)) || any(less(cAABBMin, aabbMin))) throw std::runtime_error("cluster out of bounds!");
      }
    }
    
    gprtBufferUnmap(knnAccel.clusters);
    gprtBufferUnmap(knnAccel.superClusters);
  }

  // At this point, we want to create an RT core tree over these superclusters.
  // We dialate the bounds of these superclusters by the maximum search radius
  
}

inline void gprtKNNAccelDestroy(GPRTKNNAccel &knnAccel) {
    gprtBufferDestroy(knnAccel.hilbertCodes);
    gprtBufferDestroy(knnAccel.ids);
    gprtBufferDestroy(knnAccel.aabb);
    gprtBufferDestroy(knnAccel.clusters);
    gprtBufferDestroy(knnAccel.superClusters);
    gprtBufferDestroy(knnAccel.scratch);

    gprtComputeDestroy(knnAccel.computePointBounds);
    gprtComputeDestroy(knnAccel.computeEdgeBounds);
    gprtComputeDestroy(knnAccel.computeTriangleBounds);
    gprtComputeDestroy(knnAccel.computePointClusters);
    gprtComputeDestroy(knnAccel.computeEdgeClusters);
    gprtComputeDestroy(knnAccel.computeTriangleClusters);
    gprtComputeDestroy(knnAccel.computeSuperClusters);
    gprtComputeDestroy(knnAccel.computePointHilbertCodes);
    gprtComputeDestroy(knnAccel.computeEdgeHilbertCodes);
    gprtComputeDestroy(knnAccel.computeTriangleHilbertCodes);
    gprtModuleDestroy(knnAccel.module);
}

#endif