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

#define NUM_CLUSTERS_PER_SUPERCLUSTER 8
#define NUM_PRIMS_PER_CLUSTER 8

struct KNNAccelData {
  // input
  alignas(4) uint32_t numPrims;
  alignas(4) uint32_t numClusters;
  alignas(4) uint32_t numSuperClusters;
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

  // Buffers of AABBs. Each AABB here contains clusters, but is also dialated by "maximum search range".
  alignas(16) gprt::Buffer superClusters;

  // An RT core tree
  alignas(16) gprt::Accel accel;
};

float rm(float p, float rp, float rq) {
    if (p <= (rp+rq)/2.f) {
        return rp;
    }
    return rq;
}

float rM(float p, float rp, float rq) {
    if (p >= (rp+rq)/2.f) {
        return rp;
    }
    return rq;
}

// minMaxDist computes the minimum of the maximum distances from p to points
// on r. If r is the bounding box of some geometric objects, then there is
// at least one object contained in r within minMaxDist(p, r) of p.
//
// Implemented per Definition 4 of "Nearest Neighbor Queries" by
// N. Roussopoulos, S. Kelley and F. Vincent, ACM SIGMOD, pages 71-79, 1995.
float getMinMaxDist(float3 origin, float3 aabbMin, float3 aabbMax) {
    float minmaxdist = 1e20f;
    float S = 0.f;
    for (int i = 0; i < 3; ++i) {
        float d = origin[i] - rM(origin[i], aabbMin[i], aabbMax[i]);
        S += d * d;
    }    
    for (int i = 0; i < 3; ++i) {
        float d1 = origin[i] - rM(origin[i], aabbMin[i], aabbMax[i]);
        float d2 = origin[i] - rm(origin[i], aabbMin[i], aabbMax[i]);
        float d = S - d1*d1 + d2*d2;
        if (d < minmaxdist) minmaxdist = d;
    }
    return minmaxdist;
}

// minDist computes the square of the distance from a point to a rectangle.
// If the point is contained in the rectangle then the distance is zero.
//
// Implemented per Definition 2 of "Nearest Neighbor Queries" by
// N. Roussopoulos, S. Kelley and F. Vincent, ACM SIGMOD, pages 71-79, 1995.
float getMinDist(float3 origin, float3 aabbMin, float3 aabbMax) {
    float minDist = 0.0f;
    for (int i = 0; i < 3; ++i) {
        if (origin[i] < aabbMin[i]) {
            float d = origin[i] - aabbMin[i];
            minDist += d * d;
        } else if (origin[i] > aabbMax[i]) {
            float d = origin[i] - aabbMax[i];
            minDist += d * d;
        } 
    }
    return minDist;
}

float dot2(float3 v ) { return dot(v,v); }
float ndot(float2 a, float2 b ) { return a.x*b.x - a.y*b.y; }

float getPointDist2( float3 p, float3 a )
{
  return dot2(a - p);
}

float getEdgeDist2( float3 p, float3 a, float3 b )
{
  float3 pa = p - a, ba = b - a;
  float h = clamp( dot(pa,ba)/dot2(ba), 0.0f, 1.0f );
  return dot2(pa - ba*h);
}

float getTriangleDist2( float3 p, float3 a, float3 b, float3 c )
{
  float3 ba = b - a; float3 pa = p - a;
  float3 cb = c - b; float3 pb = p - b;
  float3 ac = a - c; float3 pc = p - c;
  float3 nor = cross( ba, ac );

  return 
    (sign(dot(cross(ba,nor),pa)) +
     sign(dot(cross(cb,nor),pb)) +
     sign(dot(cross(ac,nor),pc))<2.0f)
     ?
     min( min(
     dot2(ba*clamp(dot(ba,pa)/dot2(ba),0.0f,1.0f)-pa),
     dot2(cb*clamp(dot(cb,pb)/dot2(cb),0.0f,1.0f)-pb) ),
     dot2(ac*clamp(dot(ac,pc)/dot2(ac),0.0f,1.0f)-pc) )
     :
     dot(nor,pa)*dot(nor,pa)/dot2(nor);
}

#ifdef GPRT_DEVICE

// Payload for nearest neighbor queries
struct [raypayload] NNPayload {
  float closestDistance : read(anyhit, caller) : write(anyhit, caller);
  int closestPrimitive : read(anyhit, caller) : write(anyhit, caller);
};

void TraceNN(float3 queryOrigin, in KNNAccelData knnAccel, out int closestPrimitive, out float closestDistance) {
  NNPayload payload;
  payload.closestPrimitive = -1;
  payload.closestDistance = 1e20f;

  RayDesc rayDesc;
  rayDesc.Origin = queryOrigin;
  rayDesc.Direction = float3(1.f, 1.f, 1.f);
  rayDesc.TMin = 0.0f;
  rayDesc.TMax = 0.0f;
  RaytracingAccelerationStructure world = gprt::getAccelHandle(knnAccel.accel);
  TraceRay(world,
           RAY_FLAG_SKIP_CLOSEST_HIT_SHADER | RAY_FLAG_ACCEPT_FIRST_HIT_AND_END_SEARCH,
           0xff,
           0,                       // ray type
           gprt::getNumRayTypes(),  // number of ray types
           0,                       // miss type
           rayDesc,                 // the ray to trace
           payload                  // the payload IO
  );

  closestPrimitive = payload.closestPrimitive;
  closestDistance = payload.closestDistance;
}

#else
#include <limits.h>
#include <stdexcept>
extern GPRTProgram knnDeviceCode;

typedef enum {
    GPRT_KNN_TYPE_POINTS = 1,
    GPRT_KNN_TYPE_EDGES = 2,
    GPRT_KNN_TYPE_TRIANGLES = 3,
} GPRTKNNType;

typedef enum {
    GPRT_KNN_GEOM_KIND_POINTS = 1,
    GPRT_KNN_GEOM_KIND_EDGES = 2,
    GPRT_KNN_GEOM_KIND_TRIANGLES = 3,
} GPRTKNNGeomKind;

typedef enum {
  GPRT_KNN_BUILD_MODE_UNINITIALIZED,
  GPRT_KNN_BUILD_MODE_FAST_TRACE_NO_UPDATE
} GPRTKNNBuildMode;

// Down the road, we might be able to add callables to this type which could serve 
// to replace closest hit / anyhit / intersection programs
struct GPRTKNNGeomType {
  GPRTKNNGeomKind kind;
  size_t recordSize;
};

struct GPRTKNNGeom {
  GPRTKNNGeomType type; 
  uint32_t numPoints;
  GPRTBufferOf<float3> points;
  
  uint32_t numPrims;
  GPRTBufferOf<uint2> edges;
  GPRTBufferOf<uint3> triangles;
};

struct GPRTKNNAccel {
    KNNAccelData handle;
    GPRTModule module;
    
    GPRTKNNType type;
    GPRTKNNGeom* knnGeom;

    GPRTBufferOf<uint32_t> hilbertCodes;
    GPRTBufferOf<uint32_t> ids;
    GPRTBufferOf<float3> aabb;
    GPRTBufferOf<float3> clusters;
    GPRTBufferOf<float3> superClusters;

    GPRTBufferOf<uint8_t> scratch;

    GPRTGeomTypeOf<KNNAccelData> geomType;
    GPRTGeomOf<KNNAccelData> geom;
    GPRTAccel geomAccel;
    GPRTAccel instanceAccel;

    GPRTComputeOf<KNNAccelData> computePointBounds = nullptr;
    GPRTComputeOf<KNNAccelData> computeEdgeBounds = nullptr;
    GPRTComputeOf<KNNAccelData> computeTriangleBounds = nullptr;
    GPRTComputeOf<KNNAccelData> computePointClusters = nullptr;
    GPRTComputeOf<KNNAccelData> computeEdgeClusters = nullptr;
    GPRTComputeOf<KNNAccelData> computeTriangleClusters = nullptr;
    GPRTComputeOf<KNNAccelData> computeSuperClusters = nullptr;
    GPRTComputeOf<KNNAccelData> computePointHilbertCodes = nullptr;
    GPRTComputeOf<KNNAccelData> computeEdgeHilbertCodes = nullptr;
    GPRTComputeOf<KNNAccelData> computeTriangleHilbertCodes = nullptr;

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

inline GPRTKNNGeomType gprtKNNGeomTypeCreate(
  GPRTContext context, 
  GPRTKNNGeomKind kind,
  size_t recordSize = 0
) {
  GPRTKNNGeomType geomType;
  geomType.kind = kind;
  geomType.recordSize = recordSize;
  return geomType;
}

inline void gprtKNNGeomTypeSetClosestNeighborProg(
  GPRTKNNGeomType &type, 
  uint32_t queryType,
  GPRTModule module,
  const char *entrypoint
) {
  // todo
}

inline GPRTKNNGeom gprtKNNGeomCreate(
  GPRTContext context, 
  GPRTKNNGeomType type
) {
  GPRTKNNGeom geom;
  geom.type = type;
  return geom;
}

inline void gprtKNNGeomSetPositions(
  GPRTKNNGeom &geom, 
  GPRTBufferOf<float3> positions,
  size_t numPositions
) {
  geom.points = positions;
  geom.numPoints = numPositions;
}

inline void gprtKNNEdgesSetIndices(
  GPRTKNNGeom &geom, 
  GPRTBufferOf<uint2> edges,
  size_t numEdges
) {
  geom.edges = edges;
  geom.numPrims = numEdges;
}

inline void gprtKNNTrianglesSetIndices(
  GPRTKNNGeom &geom, 
  GPRTBufferOf<uint3> triangles,
  size_t numTriangles
) {
  geom.triangles = triangles;
  geom.numPrims = numTriangles;
}

inline GPRTKNNAccel gprtKNNPointsAccelCreate(
  GPRTContext context,
  size_t numGeometries,
  GPRTKNNGeom* arrayOfChildGeoms
){
  GPRTKNNAccel knnAccel;
  knnAccel.type = GPRT_KNN_TYPE_POINTS;
  knnAccel.module = gprtModuleCreate(context, knnDeviceCode);

  if (numGeometries != 1) throw std::runtime_error("Not yet implemented");

  knnAccel.knnGeom = &arrayOfChildGeoms[0];

  knnAccel.computePointBounds = gprtComputeCreate<KNNAccelData>(context, knnAccel.module, "ComputePointBounds");
  knnAccel.computePointClusters = gprtComputeCreate<KNNAccelData>(context, knnAccel.module, "ComputePointClusters");
  knnAccel.computePointHilbertCodes = gprtComputeCreate<KNNAccelData>(context, knnAccel.module, "ComputePointHilbertCodes");
  knnAccel.computeSuperClusters = gprtComputeCreate<KNNAccelData>(context, knnAccel.module, "ComputeSuperClusters");
  
  knnAccel.geomType = gprtGeomTypeCreate<KNNAccelData>(context, GPRT_AABBS);
  gprtGeomTypeSetIntersectionProg(knnAccel.geomType, 0, knnAccel.module, "ClosestNeighborIntersection");
  gprtGeomTypeSetAnyHitProg(knnAccel.geomType, 0, knnAccel.module, "ClosestPointAnyHit");
  
  knnAccel.geom = gprtGeomCreate<KNNAccelData>(context, knnAccel.geomType);
  knnAccel.geomAccel = gprtAABBAccelCreate(context, 1, &knnAccel.geom);
  knnAccel.instanceAccel = gprtInstanceAccelCreate(context, 1, &knnAccel.geomAccel);

  // some logic here for rounding up
  knnAccel.handle.numPrims = knnAccel.knnGeom->numPoints;
  knnAccel.handle.numClusters = (knnAccel.handle.numPrims + (NUM_PRIMS_PER_CLUSTER - 1)) / NUM_PRIMS_PER_CLUSTER;
  knnAccel.handle.numSuperClusters = (knnAccel.handle.numClusters + (NUM_CLUSTERS_PER_SUPERCLUSTER - 1)) / NUM_CLUSTERS_PER_SUPERCLUSTER;

  // create these
  knnAccel.hilbertCodes = gprtDeviceBufferCreate<uint32_t>(context, knnAccel.handle.numPrims);
  knnAccel.ids = gprtDeviceBufferCreate<uint32_t>(context, knnAccel.handle.numPrims);
  knnAccel.aabb = gprtDeviceBufferCreate<float3>(context, 2);
  knnAccel.clusters = gprtDeviceBufferCreate<float3>(context, 2 * knnAccel.handle.numClusters);
  knnAccel.superClusters = gprtDeviceBufferCreate<float3>(context, 2 * knnAccel.handle.numSuperClusters);
  
  knnAccel.handle.maxSearchRange = 0.f; 

  // to be resized as needed
  knnAccel.scratch = gprtDeviceBufferCreate<uint8_t>(context);

  knnAccel.handle.points = gprtBufferGetHandle(knnAccel.knnGeom->points);

  knnAccel.handle.hilbertCodes = gprtBufferGetHandle(knnAccel.hilbertCodes);
  knnAccel.handle.ids = gprtBufferGetHandle(knnAccel.ids);
  knnAccel.handle.aabb = gprtBufferGetHandle(knnAccel.aabb);
  knnAccel.handle.clusters = gprtBufferGetHandle(knnAccel.clusters);
  knnAccel.handle.superClusters = gprtBufferGetHandle(knnAccel.superClusters);

  knnAccel.handle.accel = gprtAccelGetHandle(knnAccel.instanceAccel);

  gprtComputeSetParameters(knnAccel.computePointBounds, &knnAccel.handle);
  gprtComputeSetParameters(knnAccel.computePointClusters, &knnAccel.handle);
  gprtComputeSetParameters(knnAccel.computePointHilbertCodes, &knnAccel.handle);
  gprtComputeSetParameters(knnAccel.computeSuperClusters, &knnAccel.handle);

  gprtGeomSetParameters(knnAccel.geom, &knnAccel.handle);
  gprtAABBsSetPositions(knnAccel.geom, knnAccel.superClusters, knnAccel.handle.numSuperClusters);
  
  return knnAccel;
};

inline GPRTKNNAccel gprtKNNEdgesAccelCreate(
  GPRTContext context,
  size_t numGeometries,
  GPRTKNNGeom* arrayOfChildGeoms
){
  GPRTKNNAccel knnAccel;
  knnAccel.type = GPRT_KNN_TYPE_EDGES;
  knnAccel.module = gprtModuleCreate(context, knnDeviceCode);

  if (numGeometries != 1) throw std::runtime_error("Not yet implemented");

  knnAccel.knnGeom = &arrayOfChildGeoms[0];

  knnAccel.computeEdgeBounds = gprtComputeCreate<KNNAccelData>(context, knnAccel.module, "ComputeEdgeBounds");
  knnAccel.computeEdgeClusters = gprtComputeCreate<KNNAccelData>(context, knnAccel.module, "ComputeEdgeClusters");
  knnAccel.computeEdgeHilbertCodes = gprtComputeCreate<KNNAccelData>(context, knnAccel.module, "ComputeEdgeHilbertCodes");
  knnAccel.computeSuperClusters = gprtComputeCreate<KNNAccelData>(context, knnAccel.module, "ComputeSuperClusters");
  
  knnAccel.geomType = gprtGeomTypeCreate<KNNAccelData>(context, GPRT_AABBS);
  gprtGeomTypeSetIntersectionProg(knnAccel.geomType, 0, knnAccel.module, "ClosestNeighborIntersection");
  gprtGeomTypeSetAnyHitProg(knnAccel.geomType, 0, knnAccel.module, "ClosestEdgeAnyHit");
  
  knnAccel.geom = gprtGeomCreate<KNNAccelData>(context, knnAccel.geomType);
  knnAccel.geomAccel = gprtAABBAccelCreate(context, 1, &knnAccel.geom);
  knnAccel.instanceAccel = gprtInstanceAccelCreate(context, 1, &knnAccel.geomAccel);

  // some logic here for rounding up
  knnAccel.handle.numPrims = knnAccel.knnGeom->numPrims;
  knnAccel.handle.numClusters = (knnAccel.handle.numPrims + (NUM_PRIMS_PER_CLUSTER - 1)) / NUM_PRIMS_PER_CLUSTER;
  knnAccel.handle.numSuperClusters = (knnAccel.handle.numClusters + (NUM_CLUSTERS_PER_SUPERCLUSTER - 1)) / NUM_CLUSTERS_PER_SUPERCLUSTER;

  // create these
  knnAccel.hilbertCodes = gprtDeviceBufferCreate<uint32_t>(context, knnAccel.handle.numPrims);
  knnAccel.ids = gprtDeviceBufferCreate<uint32_t>(context, knnAccel.handle.numPrims);
  knnAccel.aabb = gprtDeviceBufferCreate<float3>(context, 2);
  knnAccel.clusters = gprtDeviceBufferCreate<float3>(context, 2 * knnAccel.handle.numClusters);
  knnAccel.superClusters = gprtDeviceBufferCreate<float3>(context, 2 * knnAccel.handle.numSuperClusters);
  
  knnAccel.handle.maxSearchRange = 0.f; 

  // to be resized as needed
  knnAccel.scratch = gprtDeviceBufferCreate<uint8_t>(context);

  knnAccel.handle.points = gprtBufferGetHandle(knnAccel.knnGeom->points);
  knnAccel.handle.edges = gprtBufferGetHandle(knnAccel.knnGeom->edges);

  knnAccel.handle.hilbertCodes = gprtBufferGetHandle(knnAccel.hilbertCodes);
  knnAccel.handle.ids = gprtBufferGetHandle(knnAccel.ids);
  knnAccel.handle.aabb = gprtBufferGetHandle(knnAccel.aabb);
  knnAccel.handle.clusters = gprtBufferGetHandle(knnAccel.clusters);
  knnAccel.handle.superClusters = gprtBufferGetHandle(knnAccel.superClusters);

  knnAccel.handle.accel = gprtAccelGetHandle(knnAccel.instanceAccel);

  gprtComputeSetParameters(knnAccel.computeEdgeBounds, &knnAccel.handle);
  gprtComputeSetParameters(knnAccel.computeEdgeClusters, &knnAccel.handle);
  gprtComputeSetParameters(knnAccel.computeEdgeHilbertCodes, &knnAccel.handle);
  gprtComputeSetParameters(knnAccel.computeSuperClusters, &knnAccel.handle);

  gprtGeomSetParameters(knnAccel.geom, &knnAccel.handle);
  gprtAABBsSetPositions(knnAccel.geom, knnAccel.superClusters, knnAccel.handle.numSuperClusters);
  
  return knnAccel;
};

inline GPRTKNNAccel gprtKNNTrianglesAccelCreate(
  GPRTContext context,
  size_t numGeometries,
  GPRTKNNGeom* arrayOfChildGeoms
){
  GPRTKNNAccel knnAccel;
  knnAccel.type = GPRT_KNN_TYPE_TRIANGLES;
  knnAccel.module = gprtModuleCreate(context, knnDeviceCode);

  if (numGeometries != 1) throw std::runtime_error("Not yet implemented");

  knnAccel.knnGeom = &arrayOfChildGeoms[0];

  knnAccel.computeTriangleBounds = gprtComputeCreate<KNNAccelData>(context, knnAccel.module, "ComputeTriangleBounds");
  knnAccel.computeTriangleClusters = gprtComputeCreate<KNNAccelData>(context, knnAccel.module, "ComputeTriangleClusters");
  knnAccel.computeTriangleHilbertCodes = gprtComputeCreate<KNNAccelData>(context, knnAccel.module, "ComputeTriangleHilbertCodes");
  knnAccel.computeSuperClusters = gprtComputeCreate<KNNAccelData>(context, knnAccel.module, "ComputeSuperClusters");
  
  knnAccel.geomType = gprtGeomTypeCreate<KNNAccelData>(context, GPRT_AABBS);
  gprtGeomTypeSetIntersectionProg(knnAccel.geomType, 0, knnAccel.module, "ClosestNeighborIntersection");
  gprtGeomTypeSetAnyHitProg(knnAccel.geomType, 0, knnAccel.module, "ClosestTriangleAnyHit");
  
  knnAccel.geom = gprtGeomCreate<KNNAccelData>(context, knnAccel.geomType);
  knnAccel.geomAccel = gprtAABBAccelCreate(context, 1, &knnAccel.geom);
  knnAccel.instanceAccel = gprtInstanceAccelCreate(context, 1, &knnAccel.geomAccel);

  // some logic here for rounding up
  knnAccel.handle.numPrims = knnAccel.knnGeom->numPrims;
  knnAccel.handle.numClusters = (knnAccel.handle.numPrims + (NUM_PRIMS_PER_CLUSTER - 1)) / NUM_PRIMS_PER_CLUSTER;
  knnAccel.handle.numSuperClusters = (knnAccel.handle.numClusters + (NUM_CLUSTERS_PER_SUPERCLUSTER - 1)) / NUM_CLUSTERS_PER_SUPERCLUSTER;

  // create these
  knnAccel.hilbertCodes = gprtDeviceBufferCreate<uint32_t>(context, knnAccel.handle.numPrims);
  knnAccel.ids = gprtDeviceBufferCreate<uint32_t>(context, knnAccel.handle.numPrims);
  knnAccel.aabb = gprtDeviceBufferCreate<float3>(context, 2);
  knnAccel.clusters = gprtDeviceBufferCreate<float3>(context, 2 * knnAccel.handle.numClusters);
  knnAccel.superClusters = gprtDeviceBufferCreate<float3>(context, 2 * knnAccel.handle.numSuperClusters);
  
  knnAccel.handle.maxSearchRange = 0.f; 

  // to be resized as needed
  knnAccel.scratch = gprtDeviceBufferCreate<uint8_t>(context);

  knnAccel.handle.points = gprtBufferGetHandle(knnAccel.knnGeom->points);
  knnAccel.handle.triangles = gprtBufferGetHandle(knnAccel.knnGeom->triangles);

  knnAccel.handle.hilbertCodes = gprtBufferGetHandle(knnAccel.hilbertCodes);
  knnAccel.handle.ids = gprtBufferGetHandle(knnAccel.ids);
  knnAccel.handle.aabb = gprtBufferGetHandle(knnAccel.aabb);
  knnAccel.handle.clusters = gprtBufferGetHandle(knnAccel.clusters);
  knnAccel.handle.superClusters = gprtBufferGetHandle(knnAccel.superClusters);

  knnAccel.handle.accel = gprtAccelGetHandle(knnAccel.instanceAccel);

  gprtComputeSetParameters(knnAccel.computeTriangleBounds, &knnAccel.handle);
  gprtComputeSetParameters(knnAccel.computeTriangleClusters, &knnAccel.handle);
  gprtComputeSetParameters(knnAccel.computeTriangleHilbertCodes, &knnAccel.handle);
  gprtComputeSetParameters(knnAccel.computeSuperClusters, &knnAccel.handle);

  gprtGeomSetParameters(knnAccel.geom, &knnAccel.handle);
  gprtAABBsSetPositions(knnAccel.geom, knnAccel.superClusters, knnAccel.handle.numSuperClusters);
  
  return knnAccel;
};

inline void gprtKNNAccelSetSearchRange(GPRTKNNAccel &accel, float searchRange) {
  accel.handle.maxSearchRange = searchRange;
};

inline void gprtKNNAccelBuild(GPRTContext context, GPRTKNNAccel &knnAccel, GPRTKNNBuildMode mode)
{
  typedef uint32_t uint;

  gprtBuildShaderBindingTable(context, GPRT_SBT_COMPUTE);

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

  // Use global bounds to compute hilbert codes
  if (knnAccel.type == GPRT_KNN_TYPE_POINTS) {
    gprtComputeLaunch1D(context, knnAccel.computePointHilbertCodes, knnAccel.handle.numPrims);
  } else if (knnAccel.type == GPRT_KNN_TYPE_EDGES) {
    gprtComputeLaunch1D(context, knnAccel.computeEdgeHilbertCodes, knnAccel.handle.numPrims);
  } else if (knnAccel.type == GPRT_KNN_TYPE_TRIANGLES) {
    gprtComputeLaunch1D(context, knnAccel.computeTriangleHilbertCodes, knnAccel.handle.numPrims);
  }

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

  // ... and then compute super cluster bounding boxes
  gprtComputeLaunch1D(context, knnAccel.computeSuperClusters, knnAccel.handle.numSuperClusters);
  
  // Now we can build our RT core tree
  gprtAccelBuild(context, knnAccel.geomAccel, GPRT_BUILD_MODE_FAST_BUILD_NO_UPDATE, false, false);
  gprtAccelBuild(context, knnAccel.instanceAccel, GPRT_BUILD_MODE_FAST_BUILD_NO_UPDATE, false, false);

  // Handle might have changed, so update here
  knnAccel.handle.accel = gprtAccelGetHandle(knnAccel.instanceAccel);
}

inline void gprtKNNAccelDestroy(GPRTKNNAccel &knnAccel) {
    gprtBufferDestroy(knnAccel.hilbertCodes);
    gprtBufferDestroy(knnAccel.ids);
    gprtBufferDestroy(knnAccel.aabb);
    gprtBufferDestroy(knnAccel.clusters);
    gprtBufferDestroy(knnAccel.superClusters);
    gprtBufferDestroy(knnAccel.scratch);

    if (knnAccel.computePointBounds) gprtComputeDestroy(knnAccel.computePointBounds);
    if (knnAccel.computeEdgeBounds) gprtComputeDestroy(knnAccel.computeEdgeBounds);
    if (knnAccel.computeTriangleBounds) gprtComputeDestroy(knnAccel.computeTriangleBounds);
    if (knnAccel.computePointClusters) gprtComputeDestroy(knnAccel.computePointClusters);
    if (knnAccel.computeEdgeClusters) gprtComputeDestroy(knnAccel.computeEdgeClusters);
    if (knnAccel.computeTriangleClusters) gprtComputeDestroy(knnAccel.computeTriangleClusters);
    if (knnAccel.computeSuperClusters) gprtComputeDestroy(knnAccel.computeSuperClusters);
    if (knnAccel.computePointHilbertCodes) gprtComputeDestroy(knnAccel.computePointHilbertCodes);
    if (knnAccel.computeEdgeHilbertCodes) gprtComputeDestroy(knnAccel.computeEdgeHilbertCodes);
    if (knnAccel.computeTriangleHilbertCodes) gprtComputeDestroy(knnAccel.computeTriangleHilbertCodes);

    gprtAccelDestroy(knnAccel.instanceAccel);
    gprtAccelDestroy(knnAccel.geomAccel);
    gprtGeomDestroy(knnAccel.geom);
    gprtGeomTypeDestroy(knnAccel.geomType);
    gprtModuleDestroy(knnAccel.module);
}

#endif