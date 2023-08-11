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

/* define the bitmask_t type as an integer of sufficient size */
typedef uint64_t bitmask_t;
/* define the halfmask_t type as an integer of 1/2 the size of bitmask_t */
typedef uint32_t halfmask_t;

/* implementation of the hilbert functions */
#define adjust_rotation(rotation,nDims,bits)                            \
do {                                                                    \
      /* rotation = (rotation + 1 + ffs(bits)) % nDims; */              \
      bits &= -bits & nd1Ones;                                          \
      while (bits)                                                      \
        bits >>= 1, ++rotation;                                         \
      if ( ++rotation >= nDims )                                        \
        rotation -= nDims;                                              \
} while (0)

#define ones(T,k) ((((T)2) << (k-1)) - 1)

#define rdbit(w,k) (((w) >> (k)) & 1)
     
#define rotateRight(arg, nRots, nDims)                                  \
((((arg) >> (nRots)) | ((arg) << ((nDims)-(nRots)))) & ones(bitmask_t,nDims))

#define rotateLeft(arg, nRots, nDims)                                   \
((((arg) << (nRots)) | ((arg) >> ((nDims)-(nRots)))) & ones(bitmask_t,nDims))

inline bitmask_t
bitTranspose(uint32_t nDims, uint32_t nBits, bitmask_t inCoords)
{
  uint32_t const nDims1 = nDims-1;
  uint32_t inB = nBits;
  uint32_t utB;
  bitmask_t inFieldEnds = 1;
  bitmask_t inMask = ones(bitmask_t,inB);
  bitmask_t coords = 0;

  while ((utB = inB / 2))
    {
      uint32_t const shiftAmt = nDims1 * utB;
      bitmask_t const utFieldEnds =
	inFieldEnds | (inFieldEnds << (shiftAmt+utB));
      bitmask_t const utMask =
	(utFieldEnds << utB) - utFieldEnds;
      bitmask_t utCoords = 0;
      uint32_t d;
      if (inB & 1)
	{
	  bitmask_t const inFieldStarts = inFieldEnds << (inB-1);
	  uint32_t oddShift = 2*shiftAmt;
	  for (d = 0; d < nDims; ++d)
	    {
	      bitmask_t temp = inCoords & inMask;
	      inCoords >>= inB;
	      coords |= (temp & inFieldStarts) <<	oddShift++;
	      temp &= ~inFieldStarts;
	      temp = (temp | (temp << shiftAmt)) & utMask;
	      utCoords |= temp << (d*utB);
	    }
	}
      else
	{
	  for (d = 0; d < nDims; ++d)
	    {
	      bitmask_t temp = inCoords & inMask;
	      inCoords >>= inB;
	      temp = (temp | (temp << shiftAmt)) & utMask;
	      utCoords |= temp << (d*utB);
	    }
	}
      inCoords = utCoords;
      inB = utB;
      inFieldEnds = utFieldEnds;
      inMask = utMask;
    }
  coords |= inCoords;
  return coords;
}


/*****************************************************************
 * hilbert_i2c
 * 
 * Convert an index into a Hilbert curve to a set of coordinates.
 * Inputs:
 *  nDims:      Number of coordinate axes.
 *  nBits:      Number of bits per axis.
 *  index:      The index, contains nDims*nBits bits (so nDims*nBits must be <= 8*sizeof(bitmask_t)).
 * Outputs:
 *  coord:      The list of nDims coordinates, each with nBits bits.
 * Assumptions:
 *      nDims*nBits <= (sizeof index) * (bits_per_byte)
 */

inline void hilbert_i2c(uint32_t nDims, uint32_t nBits, bitmask_t index, bitmask_t coord[])
{
  if (nDims > 1)
    {
      bitmask_t coords;
      halfmask_t const nbOnes = ones(halfmask_t,nBits);
      uint32_t d;

      if (nBits > 1)
	{
	  uint32_t const nDimsBits = nDims*nBits;
	  halfmask_t const ndOnes = ones(halfmask_t,nDims);
	  halfmask_t const nd1Ones= ndOnes >> 1; /* for adjust_rotation */
	  uint32_t b = nDimsBits;
	  uint32_t rotation = 0;
	  halfmask_t flipBit = 0;
	  bitmask_t const nthbits = ones(bitmask_t,nDimsBits) / ndOnes;
	  index ^= (index ^ nthbits) >> 1;
	  coords = 0;
	  do
	    {
	      halfmask_t bits = (halfmask_t)((index >> (b-=nDims)) & ndOnes);
	      coords <<= nDims;
	      coords |= rotateLeft(bits, rotation, nDims) ^ flipBit;
	      flipBit = (halfmask_t)1 << rotation;
	      adjust_rotation(rotation,nDims,bits);
	    } while (b);
	  for (b = nDims; b < nDimsBits; b *= 2)
	    coords ^= coords >> b;
	  coords = bitTranspose(nBits, nDims, coords);
	}
      else
	coords = index ^ (index >> 1);

      for (d = 0; d < nDims; ++d)
	{
	  coord[d] = coords & nbOnes;
	  coords >>= nBits;
	}
    }
  else
    coord[0] = index;
}

/*****************************************************************
 * hilbert_c2i
 * 
 * Convert coordinates of a point on a Hilbert curve to its index.
 * Inputs:
 *  nBits:      Number of bits/coordinate.
 *  coord:      Array of n nBits-bit coordinates.
 * Outputs:
 *  index:      Output index value.  nDims*nBits bits.
 * Assumptions:
 *      nDims*nBits <= (sizeof bitmask_t) * (bits_per_byte)
 */
inline bitmask_t hilbert_c2i(uint32_t nBits, bitmask_t const coord[3])
{
  int nDims = 3;
  if (nDims > 1)
    {
      uint32_t const nDimsBits = nDims*nBits;
      bitmask_t index;
      uint32_t d;
      bitmask_t coords = 0;
      for (d = nDims; d--; )
	{
	  coords <<= nBits;
	  coords |= coord[d];
	}

      if (nBits > 1)
	{
	  halfmask_t const ndOnes = ones(halfmask_t,nDims);
	  halfmask_t const nd1Ones= ndOnes >> 1; /* for adjust_rotation */
	  uint32_t b = nDimsBits;
	  uint32_t rotation = 0;
	  halfmask_t flipBit = 0;
	  bitmask_t const nthbits = ones(bitmask_t,nDimsBits) / ndOnes;
	  coords = bitTranspose(nDims, nBits, coords);
	  coords ^= coords >> nDims;
	  index = 0;
	  do
	    {
	      halfmask_t bits = (halfmask_t)((coords >> (b-=nDims)) & ndOnes);
	      bits = (halfmask_t)rotateRight(flipBit ^ bits, rotation, nDims);
	      index <<= nDims;
	      index |= bits;
	      flipBit = (halfmask_t)1 << rotation;
	      adjust_rotation(rotation,nDims,bits);
	    } while (b);
	  index ^= nthbits >> 1;
	}
      else
	index = coords;
      for (d = 1; d < nDimsBits; d *= 2)
	index ^= index >> d;
      return index;
    }
  else
    return coord[0];
}

inline uint32_t hilbert_encode3D(float x, float y, float z)
{
    x = x * (float)(1 << 10);
    y = y * (float)(1 << 10);
    z = z * (float)(1 << 10);
    const bitmask_t coord[3] = {bitmask_t(x), bitmask_t(y), bitmask_t(z)};
    return uint32_t(hilbert_c2i(10, coord));
}

inline uint64_t hilbert64_encode3D(float x, float y, float z)
{
    x = x * (float)(1 << 20);
    y = y * (float)(1 << 20);
    z = z * (float)(1 << 20);
    const bitmask_t coord[3] = {bitmask_t(x), bitmask_t(y), bitmask_t(z)};
    return hilbert_c2i(20, coord);
}

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

GPRT_COMPUTE_PROGRAM(ComputePointBounds, (KNNAccelData, record), (1,1,1)) {
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

GPRT_COMPUTE_PROGRAM(ComputeEdgeBounds, (KNNAccelData, record), (1,1,1)) {
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

GPRT_COMPUTE_PROGRAM(ComputeTriangleBounds, (KNNAccelData, record), (1,1,1)) {
  int primID = DispatchThreadID.x;
  if (primID >= record.numPrims) return;
  float3 aabbMin, aabbMax;
  if (!getTriangleBounds(record.triangles, record.points, primID, aabbMin, aabbMax)) return;
  gprt::atomicMin32f(record.aabb, 0, aabbMin.x);
  gprt::atomicMin32f(record.aabb, 1, aabbMin.y);
  gprt::atomicMin32f(record.aabb, 2, aabbMin.z);
  gprt::atomicMax32f(record.aabb, 3, aabbMax.x);
  gprt::atomicMax32f(record.aabb, 4, aabbMax.y);
  gprt::atomicMax32f(record.aabb, 5, aabbMax.z);
}

GPRT_COMPUTE_PROGRAM(ComputePointHilbertCodes, (KNNAccelData, record), (1, 1, 1)) {
  int primID = DispatchThreadID.x;
  if (primID >= record.numPrims) return;
  float3 c;
  if (!getPointCentroid(record.points, primID, c)) {
    gprt::store<uint>(record.hilbertCodes, primID, -1);
    return;
  }
  float3 aabbMin = gprt::load<float3>(record.aabb, 0);
  float3 aabbMax = gprt::load<float3>(record.aabb, 1);
  c = (c - aabbMin) / (aabbMax - aabbMin);

  // Quantize to 10 bit
  c = min(max(c * 1024.f, float3(0.f, 0.f, 0.f)), float3(1023.f, 1023.f, 1023.f));
  
  uint code = hilbert_encode3D(c.x, c.y, c.z);
  gprt::store<uint>(record.hilbertCodes, primID, code);
  gprt::store<uint>(record.ids, primID, primID);
}

GPRT_COMPUTE_PROGRAM(ComputeEdgeHilbertCodes, (KNNAccelData, record), (1, 1, 1)) {
  int primID = DispatchThreadID.x;
  if (primID >= record.numPrims) return;
  float3 c;
  if (!getEdgeCentroid(record.edges, record.points, primID, c)) {
    gprt::store<uint>(record.hilbertCodes, primID, -1);
    return;
  }

  float3 aabbMin = gprt::load<float3>(record.aabb, 0);
  float3 aabbMax = gprt::load<float3>(record.aabb, 1);

  c = (c - aabbMin) / (aabbMax - aabbMin);

  // Quantize to 10 bit
  c = min(max(c * 1024.f, float3(0.f, 0.f, 0.f)), float3(1023.f, 1023.f, 1023.f));
  
  uint code = hilbert_encode3D(c.x, c.y, c.z);
  gprt::store<uint>(record.hilbertCodes, primID, code);
  gprt::store<uint>(record.ids, primID, primID);
}

GPRT_COMPUTE_PROGRAM(ComputeTriangleHilbertCodes, (KNNAccelData, record), (1, 1, 1)) {
  int primID = DispatchThreadID.x;
  if (primID >= record.numPrims) return;
  float3 c;
  if (!getTriangleCentroid(record.triangles, record.points, primID, c)) {
    gprt::store<uint>(record.hilbertCodes, primID, -1);
    return;
  }

  float3 aabbMin = gprt::load<float3>(record.aabb, 0);
  float3 aabbMax = gprt::load<float3>(record.aabb, 1);

  c = (c - aabbMin) / (aabbMax - aabbMin);

  // Quantize to 10 bit
  c = min(max(c * 1024.f, float3(0.f, 0.f, 0.f)), float3(1023.f, 1023.f, 1023.f));

  uint code = hilbert_encode3D(c.x, c.y, c.z);
  gprt::store<uint>(record.hilbertCodes, primID, code);
  gprt::store<uint>(record.ids, primID, primID);
}

GPRT_COMPUTE_PROGRAM(ComputePointClusters, (KNNAccelData, record), (1,1,1)) {
  int clusterID = DispatchThreadID.x;
  if (clusterID >= record.numClusters) return;
  uint32_t numPrims = record.numPrims;
  uint32_t numPrimsPerCluster = record.numPrimsPerCluster;
  uint32_t numClusters = record.numClusters;

  float3 clusterAabbMin = float3(1e20f, 1e20f, 1e20f); 
  float3 clusterAabbMax = -float3(1e20f, 1e20f, 1e20f);
  for (uint32_t i = 0; i < numPrimsPerCluster; ++i) {
    uint32_t idx = numPrimsPerCluster * clusterID + i;
    uint32_t primID = gprt::load<uint32_t>(record.ids, idx);
    if (primID >= numPrims || primID == -1) continue;
    float3 aabbMin, aabbMax;
    if (!getPointBounds(record.points, primID, aabbMin, aabbMax)) continue;
    clusterAabbMin = min(aabbMin, clusterAabbMin);
    clusterAabbMax = max(aabbMax, clusterAabbMax);
  }

  gprt::store<float3>(record.clusters, 2 * clusterID + 0, clusterAabbMin);
  gprt::store<float3>(record.clusters, 2 * clusterID + 1, clusterAabbMax);
}

GPRT_COMPUTE_PROGRAM(ComputeEdgeClusters, (KNNAccelData, record), (1,1,1)) {
  int clusterID = DispatchThreadID.x;
  if (clusterID >= record.numClusters) return;
  uint32_t numPrims = record.numPrims;
  uint32_t numPrimsPerCluster = record.numPrimsPerCluster;
  uint32_t numClusters = record.numClusters;

  float3 clusterAabbMin = float3(1e20f, 1e20f, 1e20f); 
  float3 clusterAabbMax = -float3(1e20f, 1e20f, 1e20f);
  for (uint32_t i = 0; i < numPrimsPerCluster; ++i) {
    uint32_t idx = numPrimsPerCluster * clusterID + i;
    uint32_t primID = gprt::load<uint32_t>(record.ids, idx);
    if (primID >= numPrims || primID == -1) continue;
    float3 aabbMin, aabbMax;
    if (!getEdgeBounds(record.edges, record.points, primID, aabbMin, aabbMax)) continue;
    clusterAabbMin = min(aabbMin, clusterAabbMin);
    clusterAabbMax = max(aabbMax, clusterAabbMax);
  }

  gprt::store<float3>(record.clusters, 2 * clusterID + 0, clusterAabbMin);
  gprt::store<float3>(record.clusters, 2 * clusterID + 1, clusterAabbMax);
}

GPRT_COMPUTE_PROGRAM(ComputeTriangleClusters, (KNNAccelData, record), (1,1,1)) {
  int clusterID = DispatchThreadID.x;
  if (clusterID >= record.numClusters) return;
  uint32_t numPrims = record.numPrims;
  uint32_t numPrimsPerCluster = record.numPrimsPerCluster;
  uint32_t numClusters = record.numClusters;

  float3 clusterAabbMin = float3(1e20f, 1e20f, 1e20f); 
  float3 clusterAabbMax = -float3(1e20f, 1e20f, 1e20f);
  for (uint32_t i = 0; i < numPrimsPerCluster; ++i) {
    uint32_t idx = numPrimsPerCluster * clusterID + i;
    uint32_t primID = gprt::load<uint32_t>(record.ids, idx);
    if (primID >= numPrims || primID == -1) continue;
    float3 aabbMin, aabbMax;
    if (!getTriangleBounds(record.triangles, record.points, primID, aabbMin, aabbMax)) continue;
    clusterAabbMin = min(aabbMin, clusterAabbMin);
    clusterAabbMax = max(aabbMax, clusterAabbMax);
  }

  gprt::store<float3>(record.clusters, 2 * clusterID + 0, clusterAabbMin);
  gprt::store<float3>(record.clusters, 2 * clusterID + 1, clusterAabbMax);
}

GPRT_COMPUTE_PROGRAM(ComputeSuperClusters, (KNNAccelData, record), (1,1,1)) {
  int superClusterID = DispatchThreadID.x;
  if (superClusterID >= record.numSuperClusters) return;
  uint32_t numClusters = record.numClusters;
  uint32_t numClustersPerSuperCluster = record.numClustersPerSuperCluster;
  uint32_t numSuperClusters = record.numSuperClusters;

  float3 superClusterAabbMin = float3(1e20f, 1e20f, 1e20f); 
  float3 superClusterAabbMax = -float3(1e20f, 1e20f, 1e20f);
  for (uint32_t i = 0; i < numClustersPerSuperCluster; ++i) {
    uint32_t clusterID = numClustersPerSuperCluster * superClusterID + i;
    if (clusterID >= numClusters) continue;
    float3 aabbMin = gprt::load<float3>(record.clusters, clusterID * 2 + 0);
    float3 aabbMax = gprt::load<float3>(record.clusters, clusterID * 2 + 1);
    if (any(aabbMax < aabbMin)) continue; // invalid cluster
    superClusterAabbMin = min(aabbMin, superClusterAabbMin);
    superClusterAabbMax = max(aabbMax, superClusterAabbMax);
  }

  gprt::store<float3>(record.superClusters, 2 * superClusterID + 0, superClusterAabbMin);
  gprt::store<float3>(record.superClusters, 2 * superClusterID + 1, superClusterAabbMax);
}