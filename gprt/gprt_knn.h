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

// Uniform parameters for entry points. Shared between Slang and C++
struct ComputeTriangleRootBoundsParams {
    int numPrims;
    gprt::Buffer triangles;
    gprt::Buffer vertices;
    gprt::Buffer centers;
    gprt::Buffer rootBounds;
#ifndef __SLANG_COMPILER__
    ComputeTriangleRootBoundsParams(int numPrims, gprt::Buffer triangles, gprt::Buffer vertices, gprt::Buffer centers, gprt::Buffer rootBounds) :
      numPrims(numPrims), triangles(triangles), vertices(vertices), centers(centers), rootBounds(rootBounds) {}
#endif
};

struct ComputeTriangleCodesParams {
    int numPrims;
    gprt::Buffer centers;
    gprt::Buffer rootBounds;
    gprt::Buffer hilbertCodes;
    gprt::Buffer primIDs;
#ifndef __SLANG_COMPILER__
    ComputeTriangleCodesParams(int numPrims, gprt::Buffer centers, gprt::Buffer rootBounds, gprt::Buffer hilbertCodes, gprt::Buffer primIDs) :
      numPrims(numPrims), centers(centers), rootBounds(rootBounds), hilbertCodes(hilbertCodes), primIDs(primIDs) {}
#endif
};

struct ExpandTrianglesParams {
    int numPrims;
    gprt::Buffer primIDs;
    gprt::Buffer triangles;
    gprt::Buffer vertices;
    gprt::Buffer trianglesExp;
#ifndef __SLANG_COMPILER__
    ExpandTrianglesParams(int numPrims, gprt::Buffer primIDs, gprt::Buffer triangles, gprt::Buffer vertices, gprt::Buffer trianglesExp) :
      numPrims(numPrims), primIDs(primIDs), triangles(triangles), vertices(vertices), trianglesExp(trianglesExp) {}
#endif
};

struct ComputeTriangleAABBsAndCentersParams {
    int numPrims;
    int level;
    gprt::Buffer trianglesExp;
    gprt::Buffer aabbs;
    gprt::Buffer centers;
#ifndef __SLANG_COMPILER__
    ComputeTriangleAABBsAndCentersParams(int numPrims, int level, gprt::Buffer trianglesExp, gprt::Buffer aabbs, gprt::Buffer centers) :
      numPrims(numPrims), level(level), trianglesExp(trianglesExp), aabbs(aabbs), centers(centers) {}
#endif
};

struct ComputeAABBsAndCentersParams {
    int numPrims;
    int level;
    gprt::Buffer childAABBs;
    gprt::Buffer childCenters;
    gprt::Buffer parentAABBs;
    gprt::Buffer parentCenters;
#ifndef __SLANG_COMPILER__
    ComputeAABBsAndCentersParams(int numPrims, int level, gprt::Buffer childAABBs, gprt::Buffer childCenters, gprt::Buffer parentAABBs, gprt::Buffer parentCenters) :
      numPrims(numPrims), level(level), childAABBs(childAABBs), childCenters(childCenters), parentAABBs(parentAABBs), parentCenters(parentCenters) {}
#endif
};

struct ComputeTriangleOBBCovariancesParams {
    int numPrims;
    int level;
    gprt::Buffer trianglesExp;
    gprt::Buffer parentCenters;
    gprt::Buffer parentOBBs;
#ifndef __SLANG_COMPILER__
    ComputeTriangleOBBCovariancesParams(int numPrims, int level, gprt::Buffer trianglesExp, gprt::Buffer parentCenters, gprt::Buffer parentOBBs) :
      numPrims(numPrims), level(level), trianglesExp(trianglesExp), parentCenters(parentCenters), parentOBBs(parentOBBs) {}
#endif
};

struct ComputeOBBCovariancesParams {
    int numPrims;
    int level;
    gprt::Buffer childOBBs;
    gprt::Buffer childCenters;
    gprt::Buffer parentOBBs;
    gprt::Buffer parentCenters;
#ifndef __SLANG_COMPILER__
    ComputeOBBCovariancesParams(int numPrims, int level, gprt::Buffer childOBBs, gprt::Buffer childCenters, gprt::Buffer parentOBBs, gprt::Buffer parentCenters) :
      numPrims(numPrims), level(level), childOBBs(childOBBs), childCenters(childCenters), parentOBBs(parentOBBs), parentCenters(parentCenters) {}
#endif
};

struct ComputeOBBAnglesParams {
    int numPrims;
    int level;
    gprt::Buffer OBBs;
#ifndef __SLANG_COMPILER__
    ComputeOBBAnglesParams(int numPrims, int level, gprt::Buffer OBBs) :
      numPrims(numPrims), level(level), OBBs(OBBs) {}
#endif
};

struct ComputeTriangleOBBBoundsParams {
    int numPrims;
    int numLevels;
    int stride;
    int iteration;
    gprt::Buffer trianglesExp;
    gprt::Buffer OBBHierarchy[MAX_LEVELS];
#ifndef __SLANG_COMPILER__
    ComputeTriangleOBBBoundsParams(int numPrims, int numLevels, int stride, int iteration, gprt::Buffer trianglesExp, gprt::Buffer OBBHierarchy[MAX_LEVELS]) :
      numPrims(numPrims), numLevels(numLevels), stride(stride), iteration(iteration), trianglesExp(trianglesExp) {
        for (int i = 0; i < MAX_LEVELS; ++i) {
          this->OBBHierarchy[i] = OBBHierarchy[i];
        }
      }
#endif
};

struct NNStats {
    float primsHit;
    float lHit[MAX_LEVELS];

    float getTotalTouched() {
        float totalTouched = primsHit;
        for (int i = 0; i < MAX_LEVELS; ++i) {
            totalTouched += lHit[i];
        }
        return totalTouched;
    }

    static NNStats Create() {
        NNStats stats;
        stats.primsHit = 0;
        for (int i = 0; i < MAX_LEVELS; ++i) {
            stats.lHit[i] = 0;
        }
        return stats;
    };
};

// Below code is device-code specific
#ifdef __SLANG_COMPILER__

enum NN_FLAG : uint32_t
{
  NN_FLAG_NONE                            = 0x00, 
  NN_FLAG_ACCEPT_FIRST_NEIGHBOR_AND_END_SEARCH = 0x04,
  // Attempts to take the mindist of the bounds rather than the primitives themselves. 
  // If the minimum distance to the bounds is zero, then the true closest distance is reported.
  // Doesn't return a primitive ID. Useful for walk on spheres and signed distance fields
  NN_FLAG_ACCEPT_UNDERESTIMATE_DISTANCE = 0x08,  
};



inline float rm(float p, float rp, float rq) {
    if (p <= (rp+rq)/2.f) {
        return rp;
    }
    return rq;
}

inline float rM(float p, float rp, float rq) {
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
// inline float getMinMaxDist(float3 origin, float3 aabbMin, float3 aabbMax) {
//     float minmaxdist = 1e38f;
//     float S = 0.f;
//     for (int i = 0; i < 3; ++i) {
//         float d = origin[i] - rM(origin[i], aabbMin[i], aabbMax[i]);
//         S += d * d;
//     }    
//     for (int i = 0; i < 3; ++i) {
//         float d1 = origin[i] - rM(origin[i], aabbMin[i], aabbMax[i]);
//         float d2 = origin[i] - rm(origin[i], aabbMin[i], aabbMax[i]);
//         float d = S - d1*d1 + d2*d2;
//         if (d < minmaxdist) minmaxdist = d;
//     }
//     return minmaxdist;
// }

// minDist computes the square of the distance from a point to a rectangle.
// If the point is contained in the rectangle then the distance is zero.
//
// Implemented per Definition 2 of "Nearest Neighbor Queries" by
// N. Roussopoulos, S. Kelley and F. Vincent, ACM SIGMOD, pages 71-79, 1995.
// inline float getMinDist(float3 origin, float3 aabbMin, float3 aabbMax) {
//     float minDist = 0.0f;
//     for (int i = 0; i < 3; ++i) {
//         if (origin[i] < aabbMin[i]) {
//             float d = origin[i] - aabbMin[i];
//             minDist += d * d;
//         } else if (origin[i] > aabbMax[i]) {
//             float d = origin[i] - aabbMax[i];
//             minDist += d * d;
//         } 
//     }
//     return minDist;
// }


// minDist computes the square of the distance from a point to a rectangle.
// If the point is contained in the rectangle then the distance is zero.
//
// Implemented per Definition 2 of "Nearest Neighbor Queries" by
// N. Roussopoulos, S. Kelley and F. Vincent, ACM SIGMOD, pages 71-79, 1995.
inline float getMinDist(float3 origin, float3 aabbMin, float3 aabbMax) {
  // For each dimension...
  // Determine which side that "p" is on.
  // Then, add the L2 distance to that side to our total
  float3 d = select((origin < aabbMin), origin - aabbMin, 
             select((origin > aabbMax), origin - aabbMax, 
            float3(0.f, 0.f, 0.f))
  );
  return sqrt(dot(d, d));
}

// maxDist computes the square of the farthest distance from a point to a rectangle.
inline float getMaxDist(float3 origin, float3 aabbMin, float3 aabbMax) {  
// could be optimized. For now, just worried about correctness.
  float maxDist = 0.0f;
  float3 c = (aabbMin+aabbMax) * .5f;
  for (int i = 0; i < 3; ++i) {
      // if to the left of center
      if (origin[i] < c[i]) {
        // measure distance to right
        float d = origin[i] - aabbMax[i];
        maxDist += d * d;
      } else {
        // measure distance to left
        float d = origin[i] - aabbMin[i];
        maxDist += d * d;
      }
  }
  return sqrt(maxDist);
}


// minMaxDist computes the minimum of the maximum distances from p to points
// on r. If r is the bounding box of some geometric objects, then there is
// at least one object contained in r within minMaxDist(p, r) of p.
//
// Implemented per Definition 4 of "Nearest Neighbor Queries" by
// N. Roussopoulos, S. Kelley and F. Vincent, ACM SIGMOD, pages 71-79, 1995.
inline float getMinMaxDist(float3 origin, float3 aabbMin, float3 aabbMax) {
  // TEMP
  // return getMaxDist(origin, aabbMin, aabbMax);

  // by definition, MinMaxDist(p, r) =
  // min{1<=k<=n}(|pk - rmk|^2 + sum{1<=i<=n, i != k}(|pi - rMi|^2))
  // where rmk means, "distance to side nearest point"
  // and rMk means, "distance to side farthest from point".

  // In other words, for each dimension we take the distance to the closest side, 
  // then add on the distances to the farthest sides of all other walls. 

  // This is where the "MinMax" comes from.

  // below is a linear time algorithm to compute the above.
  float3 c = (aabbMin+aabbMax) * .5f;
  float3 d1 = origin - select(origin >= c, aabbMin, aabbMax);
  float S = dot(d1, d1);
  float3 d2 = origin - select(origin < c, aabbMin, aabbMax);
  float3 d = float3(S, S, S) - d1 * d1 + d2 * d2;
  float dist = min(d.x, min(d.y, d.z));
  return sqrt(dist);
}


float _dot2(float3 v ) { return dot(v,v); }

float getPointDist2( float3 p, float3 a )
{
  return _dot2(a - p);
}

float getEdgeDist2( float3 p, float3 a, float3 b )
{
  float3 pa = p - a, ba = b - a;
  float h = clamp( dot(pa,ba)/_dot2(ba), 0.0f, 1.0f );
  return _dot2(pa - ba*h);
}

// Returns the minimum and maximum distances (squared)
// https://iquilezles.org/articles/triangledistance/
float2 getTriangleDist2( float3 p, float3 a, float3 b, float3 c )
{
  // prepare data
  float3 ba = b - a; float3 pa = p - a;
  float3 cb = c - b; float3 pb = p - b;
  float3 ac = a - c; float3 pc = p - c;
  float3 nor = cross( ba, ac );
   
  float minDist = (
    // inside/outside test 
     sign(dot(cross(ba,nor),pa)) +
     sign(dot(cross(cb,nor),pb)) +
     sign(dot(cross(ac,nor),pc))<2.0f)
     ? // 3 edges
     min( min(
     _dot2(ba*clamp(dot(ba,pa)/_dot2(ba),0.0f,1.0f)-pa),
     _dot2(cb*clamp(dot(cb,pb)/_dot2(cb),0.0f,1.0f)-pb) ),
     _dot2(ac*clamp(dot(ac,pc)/_dot2(ac),0.0f,1.0f)-pc) )
     : // 1 face 
     dot(nor,pa)*dot(nor,pa)/_dot2(nor);

  float maxDist = max(max(_dot2(pa), _dot2(pb)), _dot2(pc));
  return sqrt(float2(minDist, maxDist));
}

// Payload for nearest neighbor queries
struct NNPayload {
  float closestDistance;
  int closestPrimitive;
  #ifdef COLLECT_STATS
  int primsHit;
  int lHit[MAX_LEVELS];
  #endif
};


/* define the bitmask_t type as an integer of sufficient size */
typedef uint64_t bitmask_t;
/* define the halfmask_t type as an integer of 1/2 the size of bitmask_t */
typedef uint32_t halfmask_t;

/* implementation of the hilbert functions */
#define adjust_rotation(rotation,nDims,bits)                            \
do {                                                                    \
      /* rotation = (rotation + 1 + ffs(bits)) % nDims; */              \
      bits &= -bits & nd1Ones;                                          \
      while (bits != 0)                                                 \
        bits >>= 1, ++rotation;                                         \
      if ( (++rotation) >= nDims )                                      \
        rotation -= nDims;                                              \
} while (false)

#define ones(T,k) ((((T)2) << (k-1)) - 1)

#define rdbit(w,k) (((w) >> (k)) & 1)
     
#define rotateRight(arg, nRots, nDims)                                  \
((((arg) >> (nRots)) | ((arg) << ((nDims)-(nRots)))) & ones(bitmask_t,nDims))

#define rotateLeft(arg, nRots, nDims)                                   \
((((arg) << (nRots)) | ((arg) >> ((nDims)-(nRots)))) & ones(bitmask_t,nDims))

inline bitmask_t
bitTranspose(uint32_t nDims, uint32_t nBits, bitmask_t inCoords)
{
  uint32_t nDims1 = nDims - 1;
  uint32_t inB = nBits;
  uint32_t utB;
  bitmask_t inFieldEnds = 1;
  bitmask_t inMask = ones(bitmask_t,inB);
  bitmask_t coords = 0;

  while ((utB = inB / 2) != 0)
  {
    uint32_t shiftAmt = nDims1 * utB;
    bitmask_t utFieldEnds = inFieldEnds | (inFieldEnds << (shiftAmt+utB));
    bitmask_t utMask = (utFieldEnds << utB) - utFieldEnds;
    bitmask_t utCoords = 0;
    uint32_t d;
    if ((inB & 1) != 0)
    {
      bitmask_t inFieldStarts = inFieldEnds << (inB-1);
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
// inline bitmask_t hilbert_c2i(uint32_t nBits, bitmask_t coord[3])
// {
//   int nDims = 3;
//   if (nDims > 1)
//     {
//       uint32_t nDimsBits = nDims*nBits;
//       bitmask_t index;
//       uint32_t d;
//       bitmask_t coords = 0;
//       for (d = nDims; d--; )
// 	{
// 	  coords <<= nBits;
// 	  coords |= coord[d];
// 	}

//       if (nBits > 1)
// 	{
// 	  halfmask_t ndOnes = ones(halfmask_t,nDims);
// 	  halfmask_t nd1Ones= ndOnes >> 1; /* for adjust_rotation */
// 	  uint32_t b = nDimsBits;
// 	  uint32_t rotation = 0;
// 	  halfmask_t flipBit = 0;
// 	  bitmask_t nthbits = ones(bitmask_t,nDimsBits) / ndOnes;
// 	  coords = bitTranspose(nDims, nBits, coords);
// 	  coords ^= coords >> nDims;
// 	  index = 0;
// 	  do
// 	    {
// 	      halfmask_t bits = (halfmask_t)((coords >> (b-=nDims)) & ndOnes);
// 	      bits = (halfmask_t)rotateRight(flipBit ^ bits, rotation, nDims);
// 	      index <<= nDims;
// 	      index |= bits;
// 	      flipBit = (halfmask_t)1 << rotation;
// 	      adjust_rotation(rotation,nDims,bits);
// 	    } while (b);
// 	  index ^= nthbits >> 1;
// 	}
//       else
// 	index = coords;
//       for (d = 1; d < nDimsBits; d *= 2)
// 	index ^= index >> d;
//       return index;
//     }
//   else
//     return coord[0];
// }

inline bitmask_t hilbert_c2i_3d(uint nBits, bitmask_t coord[3])
{
  int nDims = 3;
  
  uint nDimsBits = nDims*nBits;
  bitmask_t index;
  uint d;
  bitmask_t coords = 0;
  for (d = nDims; d > 0; d--)
  {
    coords <<= nBits;
    coords |= coord[d];
  }
  if (nBits > 1)
  {
    halfmask_t ndOnes = ones(halfmask_t,nDims);
    halfmask_t nd1Ones= ndOnes >> 1; /* for adjust_rotation */
    uint b = nDimsBits;
    uint rotation = 0;
    halfmask_t flipBit = 0;
    bitmask_t nthbits = ones(bitmask_t,nDimsBits) / ndOnes;
    coords = bitTranspose(nDims, nBits, coords);
    coords ^= coords >> nDims;
    index = 0;
    do {
        halfmask_t bits = halfmask_t((coords >> (b-=nDims)) & ndOnes);
        bits = halfmask_t(rotateRight(flipBit ^ bits, rotation, nDims));
        index <<= nDims;
        index |= bits;
        flipBit = (halfmask_t)1 << rotation;
        adjust_rotation(rotation,nDims,bits);
    } while (b != 0);
    index ^= nthbits >> 1;
  }
  else
    index = coords;
  for (d = 1; d < nDimsBits; d *= 2)
    index ^= index >> d;
  return index;
}

inline bitmask_t hilbert_c2i_4d(uint nBits, bitmask_t coord[4])
{
  int nDims = 4;
  
  uint nDimsBits = nDims*nBits;
  bitmask_t index;
  uint d;
  bitmask_t coords = 0;
  for (d = nDims; d > 0; d--)
  {
    coords <<= nBits;
    coords |= coord[d];
  }
  if (nBits > 1)
  {
    halfmask_t ndOnes = ones(halfmask_t,nDims);
    halfmask_t nd1Ones= ndOnes >> 1; /* for adjust_rotation */
    uint b = nDimsBits;
    uint rotation = 0;
    halfmask_t flipBit = 0;
    bitmask_t nthbits = ones(bitmask_t,nDimsBits) / ndOnes;
    coords = bitTranspose(nDims, nBits, coords);
    coords ^= coords >> nDims;
    index = 0;
    do {
        halfmask_t bits = halfmask_t((coords >> (b-=nDims)) & ndOnes);
        bits = halfmask_t(rotateRight(flipBit ^ bits, rotation, nDims));
        index <<= nDims;
        index |= bits;
        flipBit = (halfmask_t)1 << rotation;
        adjust_rotation(rotation,nDims,bits);
    } while (b != 0);
    index ^= nthbits >> 1;
  }
  else
    index = coords;
  for (d = 1; d < nDimsBits; d *= 2)
    index ^= index >> d;
  return index;
}

inline void hilbert_i2c_3d(uint nBits, bitmask_t index, out bitmask_t coord[3])
{
  uint nDims = 3;
  
  bitmask_t coords;
  halfmask_t nbOnes = ones(halfmask_t,nBits);
  uint d;

  if (nBits > 1) {
    uint nDimsBits = nDims*nBits;
    halfmask_t ndOnes = ones(halfmask_t,nDims);
    halfmask_t nd1Ones= ndOnes >> 1; /* for adjust_rotation */
    uint b = nDimsBits;
    uint rotation = 0;
    halfmask_t flipBit = 0;
    bitmask_t nthbits = ones(bitmask_t,nDimsBits) / ndOnes;
    index ^= (index ^ nthbits) >> 1;
    coords = 0;
    do
      {
        halfmask_t bits = (halfmask_t)((index >> (b-=nDims)) & ndOnes);
        coords <<= nDims;
        coords |= rotateLeft(bits, rotation, nDims) ^ flipBit;
        flipBit = (halfmask_t)1 << rotation;
        adjust_rotation(rotation,nDims,bits);
      } while (b != 0);
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

inline void hilbert_i2c_4d(uint nBits, bitmask_t index, out bitmask_t coord[4])
{
  uint nDims = 4;
  
  bitmask_t coords;
  halfmask_t nbOnes = ones(halfmask_t,nBits);
  uint d;

  if (nBits > 1) {
    uint nDimsBits = nDims*nBits;
    halfmask_t ndOnes = ones(halfmask_t,nDims);
    halfmask_t nd1Ones= ndOnes >> 1; /* for adjust_rotation */
    uint b = nDimsBits;
    uint rotation = 0;
    halfmask_t flipBit = 0;
    bitmask_t nthbits = ones(bitmask_t,nDimsBits) / ndOnes;
    index ^= (index ^ nthbits) >> 1;
    coords = 0;
    do
      {
        halfmask_t bits = (halfmask_t)((index >> (b-=nDims)) & ndOnes);
        coords <<= nDims;
        coords |= rotateLeft(bits, rotation, nDims) ^ flipBit;
        flipBit = (halfmask_t)1 << rotation;
        adjust_rotation(rotation,nDims,bits);
      } while (b != 0);
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

inline uint32_t hilbert_encode3D(float x, float y, float z)
{
    x = x * (float)(1ul << 10);
    y = y * (float)(1ul << 10);
    z = z * (float)(1ul << 10);
    bitmask_t coord[3];
    coord[0] = bitmask_t(x);
    coord[1] = bitmask_t(y);
    coord[2] = bitmask_t(z);
    return uint32_t(hilbert_c2i_3d(10, coord));
}

inline float3 hilbert_decode3D(uint32_t code)
{
  bitmask_t coord[3];
  hilbert_i2c_3d(10, code, coord);
  float3 c;
  c.x = float(coord[0]) / (float) (1ul << 10);
  c.y = float(coord[1]) / (float) (1ul << 10);
  c.z = float(coord[2]) / (float) (1ul << 10);
  return c;
}

inline uint32_t hilbert_encode4D(float x, float y, float z, float w)
{
    x = x * (float)(1ul << 8);
    y = y * (float)(1ul << 8);
    z = z * (float)(1ul << 8);
    w = w * (float)(1ul << 8);
    bitmask_t coord[4];
    coord[0] = bitmask_t(x);
    coord[1] = bitmask_t(y);
    coord[2] = bitmask_t(z);
    coord[3] = bitmask_t(w);
    return uint32_t(hilbert_c2i_4d(8, coord));
}

inline uint64_t hilbert64_encode3D(float x, float y, float z)
{
    x = x * (float)(1ull << 20);
    y = y * (float)(1ull << 20);
    z = z * (float)(1ull << 20);
    bitmask_t coord[3];
    coord[0] = bitmask_t(x);
    coord[1] = bitmask_t(y);
    coord[2] = bitmask_t(z);
    return hilbert_c2i_3d(20, coord);
}

inline float3 hilbert64_decode3D(uint64_t code)
{
  bitmask_t coord[3];
  hilbert_i2c_3d(20, code, coord);
  float3 c;
  c.x = float(coord[0]) / (float) (1ull << 20);
  c.y = float(coord[1]) / (float) (1ull << 20);
  c.z = float(coord[2]) / (float) (1ull << 20);
  return c;
}

inline uint64_t hilbert64_encode4D(float x, float y, float z, float w)
{
    x = x * (float)(1ull << 16);
    y = y * (float)(1ull << 16);
    z = z * (float)(1ull << 16);
    w = w * (float)(1ull << 16);
    bitmask_t coord[4];
    coord[0] = bitmask_t(x);
    coord[1] = bitmask_t(y);
    coord[2] = bitmask_t(z);
    coord[3] = bitmask_t(w);
    return hilbert_c2i_4d(16, coord);
}

inline float4 hilbert64_decode4D(uint64_t code)
{
  bitmask_t coord[4];
  hilbert_i2c_4d(16, code, coord);
  float4 c;
  c.x = float(coord[0]) / (float) (1ull << 16);
  c.y = float(coord[1]) / (float) (1ull << 16);
  c.z = float(coord[2]) / (float) (1ull << 16);
  c.w = float(coord[3]) / (float) (1ull << 16);
  return c;
}

uint separate_bits(uint n)
{
    n &=                  0x000003FF;// 0xb00000000000000000000001111111111u;
    n = (n ^ (n << 16)) & 0xFF0000FF;// 0xb11111111000000000000000011111111u;
    n = (n ^ (n <<  8)) & 0x0300F00F;// 0xb00000011000000001111000000001111u;
    n = (n ^ (n <<  4)) & 0x030C30C3;// 0xb00000011000011000011000011000011u;
    n = (n ^ (n <<  2)) & 0x09249249;// 0xb00001001001001001001001001001001u;
    return n;
};

uint compact_bits(uint n)
{
  n &=                  0x09249249; // 0xb00001001001001001001001001001001u;
  n = (n ^ (n >>  2)) & 0x030C30C3; // 0xb00000011000011000011000011000011u;
  n = (n ^ (n >>  4)) & 0x0300F00F; // 0xb00000011000000001111000000001111u;
  n = (n ^ (n >>  8)) & 0xFF0000FF; // 0xb11111111000000000000000011111111u;
  n = (n ^ (n >> 16)) & 0x000003FF; // 0xb00000000000000000000001111111111u;
  return n;
};

uint64_t separate_bits_64(uint64_t n)
{
    n &=                  0x00000000003FFFFFULL;//0b0000000000000000000000000000000000000000001111111111111111111111ull;
    n = (n ^ (n << 32)) & 0xFFFF00000000FFFFULL;//0b1111111111111111000000000000000000000000000000001111111111111111ull;
    n = (n ^ (n << 16)) & 0x00FF0000FF0000FFULL;//0b0000000011111111000000000000000011111111000000000000000011111111ull;
    n = (n ^ (n <<  8)) & 0xF00F00F00F00F00FULL;//0b1111000000001111000000001111000000001111000000001111000000001111ull;
    n = (n ^ (n <<  4)) & 0x30C30C30C30C30C3ULL;//0b0011000011000011000011000011000011000011000011000011000011000011ull;
    n = (n ^ (n <<  2)) & 0x9249249249249249ULL;//0b1001001001001001001001001001001001001001001001001001001001001001ull;
    return n;
};

uint64_t compact_bits_64(uint64_t n)
{
  n &=                  0x9249249249249249ULL; // 0b1001001001001001001001001001001001001001001001001001001001001001ull;
  n = (n ^ (n >>  2)) & 0x30C30C30C30C30C3ULL; // 0b0011000011000011000011000011000011000011000011000011000011000011ull;
  n = (n ^ (n >>  4)) & 0xF00F00F00F00F00FULL; // 0b1111000000001111000000001111000000001111000000001111000000001111ull;
  n = (n ^ (n >>  8)) & 0x00FF0000FF0000FFULL; // 0b0000000011111111000000000000000011111111000000000000000011111111ull;
  n = (n ^ (n >> 16)) & 0xFFFF00000000FFFFULL; // 0b1111111111111111000000000000000000000000000000001111111111111111ull;
  n = (n ^ (n >> 32)) & 0x00000000003FFFFFULL; // 0b0000000000000000000000000000000000000000001111111111111111111111ull;
  return n;
};

inline uint morton_encode3D(float x, float y, float z)
{
  x = x * (float)(1ul << 10);
  y = y * (float)(1ul << 10);
  z = z * (float)(1ul << 10);
  return separate_bits(uint(x)) | (separate_bits(uint(y)) << 1) | (separate_bits(uint(z)) << 2); 
}

inline uint morton_encode4D(float x, float y, float z, float w)
{
  x = x * (float)(1ul << 8);
  y = y * (float)(1ul << 8);
  z = z * (float)(1ul << 8);
  w = w * (float)(1ul << 8);
  return separate_bits(uint(x)) | (separate_bits(uint32_t(y)) << 1) | (separate_bits(uint32_t(z)) << 2) | separate_bits(uint32_t(w)) << 3; 
}

inline float3 morton_decode3D(uint32_t index)
{
  float3 c;
  c.x = float(compact_bits(index >> 0));
  c.y = float(compact_bits(index >> 1));
  c.z = float(compact_bits(index >> 2));

  c.x /= (float) (1ul << 10);
  c.y /= (float) (1ul << 10);
  c.z /= (float) (1ul << 10);
  return c;
}

inline float4 morton_decode4D(uint32_t index)
{
  float4 c;
  c.x = float(compact_bits(index >> 0));
  c.y = float(compact_bits(index >> 1));
  c.z = float(compact_bits(index >> 2));
  c.w = float(compact_bits(index >> 3));

  c.x /= (float) (1ul << 8);
  c.y /= (float) (1ul << 8);
  c.z /= (float) (1ul << 8);
  c.w /= (float) (1ul << 8);
  return c;
}

inline uint64_t morton64_encode3D(float x, float y, float z)
{
  x = x * (float)(1ull << 20);
  y = y * (float)(1ull << 20);
  z = z * (float)(1ull << 20);
  return separate_bits_64(uint64_t(x)) | (separate_bits_64(uint64_t(y)) << 1) | (separate_bits_64(uint64_t(z)) << 2); 
}

inline float3 morton64_decode3D(uint64_t index)
{
  float3 c;
  c.x = float(compact_bits_64(index >> 0));
  c.y = float(compact_bits_64(index >> 1));
  c.z = float(compact_bits_64(index >> 2));
  c.x /= (float) (1ull << 20);
  c.y /= (float) (1ull << 20);
  c.z /= (float) (1ull << 20);
  return c;
}

inline uint64_t morton64_encode4D(float x, float y, float z, float w)
{
  x = x * (float)(1ull << 16);
  y = y * (float)(1ull << 16);
  z = z * (float)(1ull << 16);
  w = w * (float)(1ull << 16);
  return separate_bits_64(uint64_t(x)) | (separate_bits_64(uint64_t(y)) << 1) | (separate_bits_64(uint64_t(z)) << 2) | (separate_bits_64(uint64_t(w)) << 3); 
}

inline float4 morton64_decode4D(uint64_t index)
{
  float4 c;
  c.x = float(compact_bits_64(index >> 0));
  c.y = float(compact_bits_64(index >> 1));
  c.z = float(compact_bits_64(index >> 2));
  c.w = float(compact_bits_64(index >> 3));
  c.x /= (float) (1ull << 16);
  c.y /= (float) (1ull << 16);
  c.z /= (float) (1ull << 16);
  c.w /= (float) (1ull << 16);
  return c;
}

float3x3 outerProduct(float3 a, float3 b) {
  return float3x3(a * b.x, a * b.y, a * b.z);
}

// outer product of a vector with itself
float3x3 outer2(float3 a) {
  return outerProduct(a,a);
}

float3 getCorner(float3 aabbMin, float3 aabbMax, int cornerID) {
  int x = ((cornerID & (1 << 0)) != 0) ? 1 : 0;
  int y = ((cornerID & (1 << 1)) != 0) ? 1 : 0;
  int z = ((cornerID & (1 << 2)) != 0) ? 1 : 0;
  return float3(
    (x == 0) ? aabbMin.x : aabbMax.x,
    (y == 0) ? aabbMin.y : aabbMax.y,
    (z == 0) ? aabbMin.z : aabbMax.z
  );
}

float getVolume(float3 aabbMin, float3 aabbMax) {
  float w = aabbMax.x - aabbMin.x;
  float h = aabbMax.y - aabbMin.y;
  float d = aabbMax.z - aabbMin.z;
  return w * h * d;
}

float getSurfaceArea(float3 aabbMin, float3 aabbMax) {
  float w = aabbMax.x - aabbMin.x;
  float h = aabbMax.y - aabbMin.y;
  float d = aabbMax.z - aabbMin.z;
  return 2 * w * h + 2 * w * d + 2 * h * d;
}

#define _EPSILON .00001f
#define _PI 3.14159265358979323846264f


// Assuming matrix is normalized... Returns two euler results, so that we can pick the best one.
void mat3_to_eul2(in float3x3 mat, out float3 eul1, out float3 eul2)
{
  // float cy = hypot(mat[0][0], mat[1][0]);
  float cy = sqrt(mat[0][0] * mat[0][0] + mat[1][0] * mat[1][0]);

  if (cy > 16.0f * _EPSILON) {

    eul1[0] = atan2(mat[2][1], mat[2][2]);
    eul1[1] = atan2(-mat[2][0], cy);
    eul1[2] = atan2(mat[1][0], mat[0][0]);

    eul2[0] = atan2(-mat[2][1], -mat[2][2]);
    eul2[1] = atan2(-mat[2][0], -cy);
    eul2[2] = atan2(-mat[1][0], -mat[0][0]);
  }
  else {
    eul1[0] = atan2(-mat[1][2], mat[1][1]);
    eul1[1] = atan2(-mat[2][0], cy);
    eul1[2] = 0.0f;
    eul2 = eul1;
  }
}

// Assuming matrix is normalized 
float3 mat3_to_eul(in float3x3 mat)
{
  float3 eul1; 
  float3 eul2;

  mat3_to_eul2(mat, eul1, eul2);

  /* return best, which is just the one with lowest values it in */
  if (abs(eul1[0]) + abs(eul1[1]) + abs(eul1[2]) >
      abs(eul2[0]) + abs(eul2[1]) + abs(eul2[2]))
  {
    return eul2;
  }
  else {
    return eul1;
  }
}

float3x3 eul_to_mat3(float3 eul)
{
  float3x3 mat;
  float ci, cj, ch, si, sj, sh, cc, cs, sc, ss;

  sincos(eul[0], si, ci);
  sincos(eul[1], sj, cj);
  sincos(eul[2], sh, ch);
  cc = ci * ch;
  cs = ci * sh;
  sc = si * ch;
  ss = si * sh;

  mat[0][0] = cj * ch;
  mat[0][1] = sj * sc - cs;
  mat[0][2] = sj * cc + ss;
  mat[1][0] = cj * sh;
  mat[1][1] = sj * ss + cc;
  mat[1][2] = sj * cs - sc;
  mat[2][0] = -sj;
  mat[2][1] = cj * si;
  mat[2][2] = cj * ci;
  return mat;
}

// Note, determinant of matrix must be positive for valid results
float4 mat3_to_quat(float3x3 mat)
{
  float4 q;
  
  if (mat[2][2] < 0.0f) {
    if (mat[0][0] > mat[1][1]) {
     float trace = 1.0f + mat[0][0] - mat[1][1] - mat[2][2];
      float s = 2.0f * sqrt(trace);
      if (mat[2][1] < mat[1][2]) {
        /* Ensure W is non-negative for a canonical result. */
        s = -s;
      }
      q[1] = 0.25f * s;
      s = 1.0f / s;
      q[0] = (mat[2][1] - mat[1][2]) * s;
      q[2] = (mat[1][0] + mat[0][1]) * s;
      q[3] = (mat[0][2] + mat[2][0]) * s;
      if (((trace == 1.0f) && (q[0] == 0.0f && q[2] == 0.0f && q[3] == 0.0f))) {
        /* Avoids the need to normalize the degenerate case. */
        q[1] = 1.0f;
      }
    }
    else {
     float trace = 1.0f - mat[0][0] + mat[1][1] - mat[2][2];
      float s = 2.0f * sqrt(trace);
      if (mat[0][2] < mat[2][0]) {
        /* Ensure W is non-negative for a canonical result. */
        s = -s;
      }
      q[2] = 0.25f * s;
      s = 1.0f / s;
      q[0] = (mat[0][2] - mat[2][0]) * s;
      q[1] = (mat[1][0] + mat[0][1]) * s;
      q[3] = (mat[2][1] + mat[1][2]) * s;
      if (((trace == 1.0f) && (q[0] == 0.0f && q[1] == 0.0f && q[3] == 0.0f))) {
        /* Avoids the need to normalize the degenerate case. */
        q[2] = 1.0f;
      }
    }
  }
  else {
    if (mat[0][0] < -mat[1][1]) {
     float trace = 1.0f - mat[0][0] - mat[1][1] + mat[2][2];
      float s = 2.0f * sqrt(trace);
      if (mat[1][0] < mat[0][1]) {
        /* Ensure W is non-negative for a canonical result. */
        s = -s;
      }
      q[3] = 0.25f * s;
      s = 1.0f / s;
      q[0] = (mat[1][0] - mat[0][1]) * s;
      q[1] = (mat[0][2] + mat[2][0]) * s;
      q[2] = (mat[2][1] + mat[1][2]) * s;
      if (((trace == 1.0f) && (q[0] == 0.0f && q[1] == 0.0f && q[2] == 0.0f))) {
        /* Avoids the need to normalize the degenerate case. */
        q[3] = 1.0f;
      }
    }
    else {
      /* NOTE(@campbellbarton): A zero matrix will fall through to this block,
       * needed so a zero scaled matrices to return a quaternion without rotation, see: T101848. */
     float trace = 1.0f + mat[0][0] + mat[1][1] + mat[2][2];
      float s = 2.0f * sqrt(trace);
      q[0] = 0.25f * s;
      s = 1.0f / s;
      q[1] = (mat[2][1] - mat[1][2]) * s;
      q[2] = (mat[0][2] - mat[2][0]) * s;
      q[3] = (mat[1][0] - mat[0][1]) * s;
      if (((trace == 1.0f) && (q[1] == 0.0f && q[2] == 0.0f && q[3] == 0.0f))) {
        /* Avoids the need to normalize the degenerate case. */
        q[0] = 1.0f;
      }
    }
  }
  return q;
}

#define M_SQRT2 1.41421356237309504880 /* sqrt(2) */
float3x3 mat3_from_quat(float4 q) {
  float3x3 m;
  float q0, q1, q2, q3, qda, qdb, qdc, qaa, qab, qac, qbb, qbc, qcc;

  q0 = M_SQRT2 * q[0];
  q1 = M_SQRT2 * q[1];
  q2 = M_SQRT2 * q[2];
  q3 = M_SQRT2 * q[3];

  qda = q0 * q1;
  qdb = q0 * q2;
  qdc = q0 * q3;
  qaa = q1 * q1;
  qab = q1 * q2;
  qac = q1 * q3;
  qbb = q2 * q2;
  qbc = q2 * q3;
  qcc = q3 * q3;

  m[0][0] = (float)(1.0 - qbb - qcc);
  m[1][0] = (float)(qdc + qab);
  m[2][0] = (float)(-qdb + qac);

  m[0][1] = (float)(-qdc + qab);
  m[1][1] = (float)(1.0 - qaa - qcc);
  m[2][1] = (float)(qda + qbc);

  m[0][2] = (float)(qdb + qac);
  m[1][2] = (float)(-qda + qbc);
  m[2][2] = (float)(1.0 - qaa - qbb);
  return m;
}

// Uses a Cayley transform to generate an orthogonal basis.
// The given vector "v" contains rotations in x, y, and z, where 0 to 1 map to 0 to 90 degrees.
float3x3 genOrthoBasis(float3 v) {
  float w = -1.f;

  float3x3 mat;
  mat._11 = w*w + v.x*v.x - v.y*v.y - v.z*v.z;
  mat._21 = 2*(v.x*v.y + w*v.z);
  mat._31 = 2*(v.x*v.z - w*v.y);

  mat._12 = 2*(v.x*v.y - w*v.z);
  mat._22 = w*w - v.x*v.x + v.y*v.y - v.z*v.z;
  mat._32 =  2*(w*v.x + v.y*v.z);

  mat._13 = 2*(w*v.y + v.x*v.z);
  mat._23 = 2*(v.y*v.z - w*v.x);
  mat._33 = w*w - v.x*v.x - v.y*v.y + v.z*v.z;

  mat /= (v.x*v.x + v.y*v.y + v.z*v.z + w*w);
  return mat;
};

struct StackItem {
  uint32_t _key;
  float _val;
  int key() {return _key;}
  float value() {return _val;}
  
  static StackItem Create(uint key, float value) {
    StackItem pair;
    pair._key = key;
    pair._val = value;
    return pair;
  }

  static StackItem Create() {
    StackItem pair;
    pair._key = -1;
    pair._val = FLT_MAX;
    return pair;
  }
};

struct Stack {
  StackItem items[BRANCHING_FACTOR];

  [mutating]
  void clear() {
    for (int i = 0; i < BRANCHING_FACTOR; ++i) {
      items[i] = StackItem::Create();
    }
  };

  [mutating]
  void insert(StackItem newItem) {
    if (items[0].value() > newItem.value()) {
      items[0] = newItem;
      // [unroll]
      for (int i = 1; i < BRANCHING_FACTOR; ++i) {
        if (items[i-1].value() >= items[i].value()) break;
        StackItem tmp = items[i-1]; items[i-1] = items[i]; items[i] = tmp;
      }
    }
  }

  static Stack Create(){
    Stack list;
    list.clear();
    return list;
  };

  int key(int index) {
    return items[BRANCHING_FACTOR - 1 - index].key();
  };

  float value(int index) {
    return items[BRANCHING_FACTOR - 1 - index].value();
  };
};

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

// void insertionSort(inout StackItem list[BRANCHING_FACTOR]) {
//   StackItem item;
//   int i, j;
//   for (i = 1; i < BRANCHING_FACTOR; ++i) {
//     item = list[i];
//     for (j = i - 1; j >= 0 && list[j].key > item.key; j--)
//       list[j + 1] = list[j];
//     list[j + 1] = item;  
//   }
// };

// x is overoptimistic, y is pessimistic
float getAABBDist(float3 p, gprt::Buffer aabbs, uint32_t index) {
  float3 aabbMin = gprt::load<float3>(aabbs, index * 2 + 0);
  float3 aabbMax = gprt::load<float3>(aabbs, index * 2 + 1);
  // return float2(getMinDist(p, aabbMin, aabbMax), getMinMaxDist(p, aabbMin, aabbMax));
  return getMinDist(p, aabbMin, aabbMax);
}

float getOOBBDist(float3 p, gprt::Buffer oobbs, uint32_t index) {
  float2 dists;
  float3 obbMin = gprt::load<float3>(oobbs, index * 3 + 0);
  float3 obbMax = gprt::load<float3>(oobbs, index * 3 + 1);
  float3 obbEul = gprt::load<float3>(oobbs, index * 3 + 2);
  float3x3 obbRot = eul_to_mat3(obbEul);
  float3 pRot = mul(obbRot, p);
  // return float2(getMinDist(pRot, obbMin, obbMax), getMinMaxDist(pRot, obbMin, obbMax));
  return getMinDist(pRot, obbMin, obbMax);
}

void TraverseLeaf(in gprt::NNAccel record, uint32_t leafID, bool useAABBs, bool useOOBBs, uint distanceLevel, float3 queryOrigin, inout NNPayload payload, bool debug) {
  int numPrims = record.numPrims;

  // At this point, it appears that the fastest thing to do is just 
  // let traversal proceed linearly at the leaves to drive upward culling.
  for (uint32_t primID = 0; primID < BRANCHING_FACTOR; ++primID) {
    uint32_t itemID = leafID * BRANCHING_FACTOR + primID;
    if (itemID >= numPrims) continue;

    // minDist = min(minDist, l0MinDist);

    #ifdef COLLECT_STATS
    payload.primsHit++; // Count this as traversing a primitive
    #endif
    float3 p = queryOrigin;

    float3 a = gprt::load<float3>(record.triangleLists, itemID * 3 + 0);
    float3 b = gprt::load<float3>(record.triangleLists, itemID * 3 + 1);

    float3 ba = b - a; float3 pa = p - a;
    float distToEdgeA = sqrt(_dot2(ba*clamp(dot(ba,pa)/_dot2(ba),0.0f,1.0f)-pa));
    // if (distToEdgeA > payload.closestDistance) continue;

    float3 c = gprt::load<float3>(record.triangleLists, itemID * 3 + 2);
    float3 cb = c - b; float3 pb = p - b;

    float distToEdgeB = sqrt(_dot2(cb*clamp(dot(cb,pb)/_dot2(cb),0.0f,1.0f)-pb));
    // if (distToEdgeB > payload.closestDistance) continue;

    float3 ac = a - c; float3 pc = p - c;
    float distToEdgeC = sqrt(_dot2(ac*clamp(dot(ac,pc)/_dot2(ac),0.0f,1.0f)-pc));
    // if (distToEdgeC > payload.closestDistance) continue;

    float3 nor = cross( ba, ac );
    float distToPlane = sqrt(dot(nor,pa)*dot(nor,pa)/_dot2(nor));
    // if (distToPlane > payload.closestDistance) continue;

    float distToEdges = min(min(distToEdgeA, distToEdgeB), distToEdgeC);

    // inside/outside test 
    bool inside = (sign(dot(cross(ba,nor),pa)) +
                  sign(dot(cross(cb,nor),pb)) +
                  sign(dot(cross(ac,nor),pc))<2.0f);

    float dist = inside ? distToEdges : distToPlane;
    
    // Primitive farther than furthest
    if (dist > payload.closestDistance) continue;

    // Newly found closest primitive
    payload.closestDistance = dist;
    payload.closestPrimitive = itemID;
  }
}

Stack intersectAndSortChildren(
  float3 queryOrigin, inout float closestDistance, 
  int start, int maxClusters,
  bool useAABBs, in gprt::Buffer aabbs, 
  bool useOOBBs, in gprt::Buffer oobbs, bool debug = false) 
{
  Stack H;
  H.clear();

  for (int child = 0; child < BRANCHING_FACTOR; ++child) {
    uint32_t index = start + child;
    if (index >= maxClusters) {
      break;
    }

    float minDist = 0.f;
    float maxDist = 1e38f;

    // Upward Culling: skip superclusters father than current closest primitive
    if (useAABBs) {
      float dist = getAABBDist(queryOrigin, aabbs, index);
      minDist = max(minDist, dist);
      if (minDist > closestDistance) continue;
    }
    
    if (useOOBBs) {
      float dist = getOOBBDist(queryOrigin, oobbs, index);
      minDist = max(minDist, dist);
      if (minDist > closestDistance) continue;
    }

    H.insert(StackItem::Create(index, minDist));
  }

  return H;
}

// This method unfortunately seems pretty slow. 
// Our hypothesis is that the "trail" here causes an indirect register read from the stack, which is slow.
void TraverseTreeFullStack(in gprt::NNAccel record, uint distanceLevel, bool useAABBs, bool useOOBBs, float3 queryOrigin, float tmax, inout NNPayload payload, bool debug) {    
  payload.closestDistance = tmax;

  gprt::Buffer aabbs[MAX_LEVELS] = record.aabbs;
  gprt::Buffer oobbs[MAX_LEVELS] = record.oobbs;

  float minDist = 0.f;

  float closestDistAvg = 0.f;
  float closestDistM2 = 0.f; // m2 aggregates the squared distance from the mean
  float closestDistVar = 0.f;
  int totalHits = 0;

  uint32_t numClusters[MAX_LEVELS] = record.numClusters;

  Stack stack[MAX_LEVELS];

  int trail[MAX_LEVELS];
  for (int i = 0; i < MAX_LEVELS; ++i) trail[i] = 0;  

  // Traverse over all levels, starting from the top and going down
  int level = MAX_LEVELS - 1;
  int parentIndex = 0;
  while (level < MAX_LEVELS) {
    // If we're traversing down the tree...
    if (trail[level] == 0) {
      // Intersect and sort the children at this level
      stack[level] = intersectAndSortChildren(queryOrigin, payload.closestDistance, parentIndex * BRANCHING_FACTOR, numClusters[level], useAABBs, aabbs[level], useOOBBs, oobbs[level], debug);
    }

    // Traverse all children from near to far
    [unroll]
    for (int i = 0; i < BRANCHING_FACTOR; ++i) {
      // If recursing up, skip forward to the child we were last on 
      if (i < trail[level]) continue; 
      
      // Fetch child index and distance from the sorted stack
      int childIndex = stack[level].key(trail[level]);
      float minDist = stack[level].value(trail[level]);
      
      // Pop when all children from here on are too far
      if (childIndex == -1 || minDist > payload.closestDistance) {
        trail[level] = BRANCHING_FACTOR;
        break;
      }

      // if at this point we're at the bottom of the tree, traverse the leaf
      if (level == 0) {
        TraverseLeaf(record, childIndex, useAABBs, useOOBBs, distanceLevel, queryOrigin, payload, debug);
        // Advance the trail now that we've processed this child.
        trail[level]++;
      } else {
        // recurse down the tree
        parentIndex = childIndex;
        break;
      }
    }

    if (trail[level] == BRANCHING_FACTOR) {
      // Reset the trail when we're done with this level.
      trail[level] = 0;
      // Then, go back up to the parent level.
      parentIndex /= BRANCHING_FACTOR;
      level++;
      // advance to the next node
      trail[level]++;
    } else {
      // recurse down      
      level--;
    }
  }

  if (payload.closestPrimitive != -1) {
    payload.closestPrimitive = uint32_t(gprt::load<uint64_t>(record.ids, payload.closestPrimitive)); 
  }
}

#define TRAVERSE_TREE_N_LEVELS(N, RECURSION)                                                                         \
void TraverseTree ## N(                                                                                              \
  in gprt::NNAccel record,                                                                                           \
  int parentIndex,                                                                                                   \
  bool useAABBs, bool useOOBBs, int distanceLevel,                                                                   \
  float3 queryOrigin, inout NNPayload payload, bool debug) {                                                         \
  int start = parentIndex * BRANCHING_FACTOR;                                                                        \
  Stack stack = intersectAndSortChildren(queryOrigin, payload.closestDistance,                                        \
                                        start, record.numClusters[N-1],                                              \
                                        useAABBs, record.aabbs[N-1],                                                 \
                                        useOOBBs, record.oobbs[N-1]);                                                \
  for (int i = 0; i < BRANCHING_FACTOR; ++i) {                                                                       \
    int currentIndex = stack.key(i);                                                                                 \
    float mindist = stack.value(i);                                                                                  \
    if (currentIndex == -1 || mindist > payload.closestDistance) return;                               \
    RECURSION(record, currentIndex, useAABBs, useOOBBs, distanceLevel, queryOrigin, payload, debug);                        \
  }                                                                                                                  \
}                                                                                        

#define CAT(a, ...) a ## __VA_ARGS__
#define TRAVERSE_TREE(N) CAT(TraverseTree, N)

#if (MAX_LEVELS >= 1) 
TRAVERSE_TREE_N_LEVELS(1, TraverseLeaf)
#endif
#if (MAX_LEVELS >= 2) 
TRAVERSE_TREE_N_LEVELS(2, TraverseTree1)
#endif
#if (MAX_LEVELS >= 3) 
TRAVERSE_TREE_N_LEVELS(3, TraverseTree2)
#endif
#if (MAX_LEVELS >= 4) 
TRAVERSE_TREE_N_LEVELS(4, TraverseTree3)
#endif
#if (MAX_LEVELS >= 5) 
TRAVERSE_TREE_N_LEVELS(5, TraverseTree4)
#endif
#if (MAX_LEVELS >= 6) 
TRAVERSE_TREE_N_LEVELS(6, TraverseTree5)
#endif
#if (MAX_LEVELS >= 7) 
TRAVERSE_TREE_N_LEVELS(7, TraverseTree6)
#endif
#if (MAX_LEVELS >= 8) 
TRAVERSE_TREE_N_LEVELS(8, TraverseTree7)
#endif
#if (MAX_LEVELS >= 9) 
TRAVERSE_TREE_N_LEVELS(9, TraverseTree8)
#endif
#if (MAX_LEVELS >= 10) 
TRAVERSE_TREE_N_LEVELS(10, TraverseTree9)
#endif

void TraverseTreeRecursive(in gprt::NNAccel record, uint distanceLevel, bool useAABBs, bool useOOBBs, float3 queryOrigin, float tmax, inout NNPayload payload, bool debug) {
  payload.closestDistance = tmax;  
  switch (record.numLevels)
  {
    case  1 :  TraverseTree1(record, 0, useAABBs, useOOBBs, distanceLevel, queryOrigin, payload, debug); break;
    case  2 :  TraverseTree2(record, 0, useAABBs, useOOBBs, distanceLevel, queryOrigin, payload, debug); break;
    case  3 :  TraverseTree3(record, 0, useAABBs, useOOBBs, distanceLevel, queryOrigin, payload, debug); break;
    case  4 :  TraverseTree4(record, 0, useAABBs, useOOBBs, distanceLevel, queryOrigin, payload, debug); break;
    case  5 :  TraverseTree5(record, 0, useAABBs, useOOBBs, distanceLevel, queryOrigin, payload, debug); break;
    case  6 :  TraverseTree6(record, 0, useAABBs, useOOBBs, distanceLevel, queryOrigin, payload, debug); break;
    case  7 :  TraverseTree7(record, 0, useAABBs, useOOBBs, distanceLevel, queryOrigin, payload, debug); break;
    case  8 :  TraverseTree8(record, 0, useAABBs, useOOBBs, distanceLevel, queryOrigin, payload, debug); break;
    case  9 :  TraverseTree9(record, 0, useAABBs, useOOBBs, distanceLevel, queryOrigin, payload, debug); break;
    case 10 : TraverseTree10(record, 0, useAABBs, useOOBBs, distanceLevel, queryOrigin, payload, debug); break;
    default: break;
  }  
  if (payload.closestPrimitive != -1) {
    payload.closestPrimitive = uint32_t(gprt::load<uint64_t>(record.ids, payload.closestPrimitive)); 
  }
}

// void TraverseTree(in gprt::NNAccel record, uint NNFlags, float3 queryOrigin, float tmin, float tmax, inout NNPayload payload, bool debug) {
//   float dist = tmax;
//   uint stack[32];
//   uint stackPtr = 0;
//   stackPtr ++;
//   stack[stackPtr] = 0; // root

//   while (stackPtr > 0)
//   {
//     bool gotoNext = false;
//     int4 node = gprt::load<int4>(record.lbvhNodes, stack[stackPtr]);
//     stackPtr = stackPtr - 1;

//     // while left and right contain children
//     while (node.x != -1 && node.y != -1) {
//       int4 children[2] = { 
//           gprt::load<int4>(record.lbvhNodes, node.x), 
//           gprt::load<int4>(record.lbvhNodes, node.y) 
//       };

//       float3 leftAABB[2] = {
//           gprt::load<float3>(record.lbvhAabbs, 2 * node.x + 0), // + record.maxSearchRange, 
//           gprt::load<float3>(record.lbvhAabbs, 2 * node.x + 1), // - record.maxSearchRange, 
//       };

//       float3 rightAABB[2] = {
//           gprt::load<float3>(record.lbvhAabbs, 2 * node.y + 0), // + record.maxSearchRange, 
//           gprt::load<float3>(record.lbvhAabbs, 2 * node.y + 1), // - record.maxSearchRange, 
//       };

//       float distL = sqrt(_dot2(max(max(leftAABB[0] - queryOrigin, float3(0.f, 0.f, 0.f)), queryOrigin - leftAABB[1])));
//       float distR = sqrt(_dot2(max(max(rightAABB[0] - queryOrigin, float3(0.f, 0.f, 0.f)), queryOrigin - rightAABB[1])));
//       if (distL < dist && distR < dist) {
//         if (distL < distR) {
//           stackPtr++;
//           stack[stackPtr] = node.y;
//           node = children[0];
//         }
//         else {
//           stackPtr++;
//           stack[stackPtr] = node.x;
//           node = children[1];
//         }
//       }
//       else if (distL < dist)
//         node = children[0];
//       else if (distR < dist)
//         node = children[1];
//       else {
//         gotoNext = true;
//         break;
//       }
//     }
//     if (gotoNext) continue;

//     // Traverse leaf
//     int leafID = node.w;
//     TraverseLeaf(record, NNFlags, queryOrigin, tmin, tmax, leafID, payload, debug);
//     dist = min(dist, payload.closestDistance);
//   }
// }

void TraceNN(
  in gprt::NNAccel knnAccel, 
  uint distanceLevel,
  bool useAABBs,
  bool useOOBBs,
  float3 queryOrigin, 
  float tMax, 
  out int closestPrimitive, 
  out float closestDistance, 
  #ifdef COLLECT_STATS
  inout NNStats stats,
  #endif 
  bool debug) 
{
  NNPayload payload;
  payload.closestPrimitive = -1;
  payload.closestDistance = tMax;
  #ifdef COLLECT_STATS
  payload.primsHit = int(stats.primsHit);
  for (int i = 0; i < MAX_LEVELS; ++i) {
    payload.lHit[i] = int(stats.lHit[i]);
  }
  #endif

  if (tMax > 0.f) {
    // #ifndef ENABLE_LBVH_REFERENCE
    // RayDesc rayDesc;
    // rayDesc.Origin = queryOrigin;
    // rayDesc.Direction = float3(1.f, 1.f, 1.f);
    // rayDesc.TMin = 0.0f;
    // rayDesc.TMax = 0.0f;
    // RaytracingAccelerationStructure world = gprt::getAccelHandle(knnAccel.accel);
    // TraceRay(world,
    //         RAY_FLAG_SKIP_CLOSEST_HIT_SHADER | RAY_FLAG_ACCEPT_FIRST_HIT_AND_END_SEARCH,
    //         0xff,
    //         0,                       // ray type
    //         gprt::getNumRayTypes(),  // number of ray types
    //         0,                       // miss type
    //         rayDesc,                 // the ray to trace
    //         payload                  // the payload IO
    // );
    // #else

    TraverseTreeRecursive(knnAccel, distanceLevel, useAABBs, useOOBBs, queryOrigin, tMax, payload, debug);
    // TraverseTreeFullStack(knnAccel, distanceLevel, useAABBs, useOOBBs, queryOrigin, tMax, payload, debug);
    
    // #endif
  }

    // to do... figure out how to determine type, or upload it yourself.

//   // Linear exhaustive points
//   for (uint32_t j = 0; j < knnAccel.numPrims; ++j) {
//     float3 pt = gprt::load<float3>(knnAccel.points, j);
//     float distance = getPointDist2(queryOrigin, pt);
//     if (distance < payload.closestDistance && distance <= pow(knnAccel.maxSearchRange, 2.f)) {
//       payload.closestDistance = distance;
//       payload.closestPrimitive = j;
//     }
//   }

//   // Linear exhaustive triangles
//   for (uint32_t j = 0; j < knnAccel.numPrims; ++j) {
//     uint3 tri = gprt::load<uint3>(knnAccel.triangles, j);
//     float3 a = gprt::load<float3>(knnAccel.points, tri.x);
//     float3 b = gprt::load<float3>(knnAccel.points, tri.y);
//     float3 c = gprt::load<float3>(knnAccel.points, tri.z);
//     float distance = getTriangleDist2(queryOrigin, a, b, c);
//     if (distance < payload.closestDistance && distance <= pow(knnAccel.maxSearchRange, 2.f)) {
//       payload.closestDistance = distance;
//       payload.closestPrimitive = j;
//     }
//   }

  closestPrimitive = payload.closestPrimitive;
  closestDistance = payload.closestDistance;
  #ifdef COLLECT_STATS
  stats.primsHit = payload.primsHit;
  for (int i = 0; i < knnAccel.numLevels; ++i) {
    stats.lHit[i] = payload.lHit[i];
  }
  #endif
}
#endif