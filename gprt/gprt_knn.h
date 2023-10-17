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

#ifndef FLT_MAX 
#define FLT_MAX 3.402823466e+38
#endif
#ifndef FLT_MIN
#define FLT_MIN 1.175494351e-38
#endif

enum NN_FLAG : uint32_t
{
  NN_FLAG_NONE                            = 0x00, 
  NN_FLAG_ACCEPT_FIRST_NEIGHBOR_AND_END_SEARCH = 0x04,
  // Attempts to take the mindist of the bounds rather than the primitives themselves. 
  // If the minimum distance to the bounds is zero, then the true closest distance is reported.
  // Doesn't return a primitive ID. Useful for walk on spheres and signed distance fields
  NN_FLAG_ACCEPT_UNDERESTIMATE_DISTANCE = 0x08,  
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

#ifdef GPRT_DEVICE

// Payload for nearest neighbor queries
struct [raypayload] NNPayload {
  float closestDistance : read(anyhit, caller) : write(anyhit, caller);
  int closestPrimitive : read(anyhit, caller) : write(anyhit, caller);
  #ifdef COLLECT_STATS
  int primsHit : read(anyhit, caller) : write(anyhit, caller);
  int lHit[MAX_LEVELS] : read(anyhit, caller) : write(anyhit, caller);
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
// inline bitmask_t hilbert_c2i(uint32_t nBits, bitmask_t const coord[3])
// {
//   int nDims = 3;
//   if (nDims > 1)
//     {
//       uint32_t const nDimsBits = nDims*nBits;
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
// 	  halfmask_t const ndOnes = ones(halfmask_t,nDims);
// 	  halfmask_t const nd1Ones= ndOnes >> 1; /* for adjust_rotation */
// 	  uint32_t b = nDimsBits;
// 	  uint32_t rotation = 0;
// 	  halfmask_t flipBit = 0;
// 	  bitmask_t const nthbits = ones(bitmask_t,nDimsBits) / ndOnes;
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

inline bitmask_t hilbert_c2i_3d(uint nBits, bitmask_t const coord[3])
{
  int nDims = 3;
  
  uint const nDimsBits = nDims*nBits;
  bitmask_t index;
  uint d;
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
    uint b = nDimsBits;
    uint rotation = 0;
    halfmask_t flipBit = 0;
    bitmask_t const nthbits = ones(bitmask_t,nDimsBits) / ndOnes;
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
    } while (b);
    index ^= nthbits >> 1;
  }
  else
    index = coords;
  for (d = 1; d < nDimsBits; d *= 2)
    index ^= index >> d;
  return index;
}

inline bitmask_t hilbert_c2i_4d(uint nBits, bitmask_t const coord[4])
{
  int nDims = 4;
  
  uint const nDimsBits = nDims*nBits;
  bitmask_t index;
  uint d;
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
    uint b = nDimsBits;
    uint rotation = 0;
    halfmask_t flipBit = 0;
    bitmask_t const nthbits = ones(bitmask_t,nDimsBits) / ndOnes;
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
    } while (b);
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
  halfmask_t const nbOnes = ones(halfmask_t,nBits);
  uint d;

  if (nBits > 1) {
    uint const nDimsBits = nDims*nBits;
    halfmask_t const ndOnes = ones(halfmask_t,nDims);
    halfmask_t const nd1Ones= ndOnes >> 1; /* for adjust_rotation */
    uint b = nDimsBits;
    uint rotation = 0;
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

inline void hilbert_i2c_4d(uint nBits, bitmask_t index, out bitmask_t coord[4])
{
  uint nDims = 4;
  
  bitmask_t coords;
  halfmask_t const nbOnes = ones(halfmask_t,nBits);
  uint d;

  if (nBits > 1) {
    uint const nDimsBits = nDims*nBits;
    halfmask_t const ndOnes = ones(halfmask_t,nDims);
    halfmask_t const nd1Ones= ndOnes >> 1; /* for adjust_rotation */
    uint b = nDimsBits;
    uint rotation = 0;
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
    n &=                  0x00000000003FFFFF;//0b0000000000000000000000000000000000000000001111111111111111111111ull;
    n = (n ^ (n << 32)) & 0xFFFF00000000FFFF;//0b1111111111111111000000000000000000000000000000001111111111111111ull;
    n = (n ^ (n << 16)) & 0x00FF0000FF0000FF;//0b0000000011111111000000000000000011111111000000000000000011111111ull;
    n = (n ^ (n <<  8)) & 0xF00F00F00F00F00F;//0b1111000000001111000000001111000000001111000000001111000000001111ull;
    n = (n ^ (n <<  4)) & 0x30C30C30C30C30C3;//0b0011000011000011000011000011000011000011000011000011000011000011ull;
    n = (n ^ (n <<  2)) & 0x9249249249249249;//0b1001001001001001001001001001001001001001001001001001001001001001ull;
    return n;
};

uint64_t compact_bits_64(uint64_t n)
{
  n &=                  0x9249249249249249; // 0b1001001001001001001001001001001001001001001001001001001001001001ull;
  n = (n ^ (n >>  2)) & 0x30C30C30C30C30C3; // 0b0011000011000011000011000011000011000011000011000011000011000011ull;
  n = (n ^ (n >>  4)) & 0xF00F00F00F00F00F; // 0b1111000000001111000000001111000000001111000000001111000000001111ull;
  n = (n ^ (n >>  8)) & 0x00FF0000FF0000FF; // 0b0000000011111111000000000000000011111111000000000000000011111111ull;
  n = (n ^ (n >> 16)) & 0xFFFF00000000FFFF; // 0b1111111111111111000000000000000000000000000000001111111111111111ull;
  n = (n ^ (n >> 32)) & 0x00000000003FFFFF; // 0b0000000000000000000000000000000000000000001111111111111111111111ull;
  return n;
};

inline uint morton_encode3D(float x, float y, float z)
{
  x = x * (float)(1ul << 10);
  y = y * (float)(1ul << 10);
  z = z * (float)(1ul << 10);
  return separate_bits(x) | (separate_bits(y) << 1) | (separate_bits(z) << 2); 
}

inline uint morton_encode4D(float x, float y, float z, float w)
{
  x = x * (float)(1ul << 8);
  y = y * (float)(1ul << 8);
  z = z * (float)(1ul << 8);
  w = w * (float)(1ul << 8);
  return separate_bits(x) | (separate_bits(y) << 1) | (separate_bits(z) << 2) | separate_bits(w) << 3; 
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
  return separate_bits_64(x) | (separate_bits_64(y) << 1) | (separate_bits_64(z) << 2); 
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
  return separate_bits_64(x) | (separate_bits_64(y) << 1) | (separate_bits_64(z) << 2) | (separate_bits_64(w) << 3); 
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
      const float trace = 1.0f + mat[0][0] - mat[1][1] - mat[2][2];
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
      const float trace = 1.0f - mat[0][0] + mat[1][1] - mat[2][2];
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
      const float trace = 1.0f - mat[0][0] - mat[1][1] + mat[2][2];
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
      const float trace = 1.0f + mat[0][0] + mat[1][1] + mat[2][2];
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


struct Pair {
  uint32_t _key : 16;
  uint32_t _val : 16;
  int key() {return _key;}
  float value() {return float(asfloat16(uint16_t(_val)));}

  static Pair Create(uint key, float value) {
    Pair pair;
    pair._key = uint16_t(key);
    pair._val = asuint16(half(value));
    return pair;
  }
};

struct List {
  Pair items[BRANCHING_FACTOR];
  void clear() {
    for (int i = 0; i < BRANCHING_FACTOR; ++i) {
      items[i]._key = 65535;
      items[i]._val = asuint16(65504.h);
    }
  };

  void insert(Pair newItem) {
    if (items[0].value() > newItem.value()) {
      items[0] = newItem;
      // [unroll]
      for (int i = 1; i < BRANCHING_FACTOR; ++i) {
        if (items[i-1].value() >= items[i].value()) break;
        Pair tmp = items[i-1]; items[i-1] = items[i]; items[i] = tmp;
      }
    }
  }

  static List Create(){
    List list;
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
  // For level -1, the number of "nodes" is just the number of primitives
  int numNodes = (numPrimitives + (PRIMS_PER_LEAF - 1)) / PRIMS_PER_LEAF;
  // When level is 1 or higher, we repeatedly divide the number of primitives per
  // node, rounding up. 
  for (int i = 0; i < level; ++i) {
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
  int maxPrimsInLevel = PRIMS_PER_LEAF * pow(BRANCHING_FACTOR, level);
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
  int parentIndex = primIndex / PRIMS_PER_LEAF;
  // When level is 1 or higher, we repeatedly divide the number of primitives per
  // node to determine our parent index.
  for (int i = 0; i < level; ++i) {
    parentIndex = parentIndex / BRANCHING_FACTOR;
  }
  return parentIndex;
}

// void insertionSort(inout Pair list[BRANCHING_FACTOR]) {
//   Pair item;
//   int i, j;
//   for (int i = 1; i < BRANCHING_FACTOR; ++i) {
//     item = list[i];
//     for (j = i - 1; j >= 0 && list[j].key > item.key; j--)
//       list[j + 1] = list[j];
//     list[j + 1] = item;  
//   }
// };

// x is overoptimistic, y is pessimistic
float2 getAABBDists(float3 p, in gprt::Buffer aabbs, uint32_t index) {
  float2 dists;
  float3 aabbMin = gprt::load<float3>(aabbs, index * 2 + 0);
  float3 aabbMax = gprt::load<float3>(aabbs, index * 2 + 1);
  dists.x = getMinDist(p, aabbMin, aabbMax);
  dists.y = getMinMaxDist(p, aabbMin, aabbMax);
  return dists;
}

float2 getOOBBDists(float3 p, in gprt::Buffer oobbs, uint32_t index) {
  float2 dists;
  float3 obbMin = gprt::load<float3>(oobbs, index * 3 + 0);
  float3 obbMax = gprt::load<float3>(oobbs, index * 3 + 1);
  float3 obbEul = gprt::load<float3>(oobbs, index * 3 + 2);
  float3x3 obbRot = eul_to_mat3(obbEul);
  float3 pRot = mul(obbRot, p);
  dists.x = getMinDist(pRot, obbMin, obbMax);
  dists.y = getMinMaxDist(pRot, obbMin, obbMax);
  return dists;
}

void TraverseLeaf(in gprt::NNAccel record, uint32_t leafID, float3 queryOrigin, float tmax, inout NNPayload payload, bool debug) {
  int numPrims = record.numPrims;

  // At this point, it appears that the fastest thing to do is just 
  // let traversal proceed linearly at the leaves to drive upward culling.
  for (uint32_t primID = 0; primID < PRIMS_PER_LEAF; ++primID) {
    uint32_t itemID = leafID * PRIMS_PER_LEAF + primID;
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

List intersectAndSortChildren(
  float3 queryOrigin, float closestDistance, 
  int start, int maxClusters,
  bool useAABBs, in gprt::Buffer aabbs, 
  bool useOOBBs, in gprt::Buffer oobbs) 
{
  List H;
  H.clear();

  for (int child = 0; child < BRANCHING_FACTOR; ++child) {
    uint32_t index = start + child;
    if (index >= maxClusters) break;

    float minDist = 0.f;

    // Upward Culling: skip superclusters father than current closest primitive
    if (useAABBs) {
      float2 dists = getAABBDists(queryOrigin, aabbs, index);
      minDist = max(minDist, dists.x);
      if (minDist > closestDistance) continue;
    }
    
    if (useOOBBs) {
      float2 dists = getOOBBDists(queryOrigin, oobbs, index);
      minDist = max(minDist, dists.x);
      if (minDist > closestDistance) continue;
    }

    H.insert(Pair::Create(child, minDist));
  }

  return H;
}

// // sorts by value
// List sortList(in List unsortedList) {
//   List sortedList;
//   sortedList.clear();
//   for (int child = 0; child < BRANCHING_FACTOR; ++child) {
//     sortedList.insert(unsortedList.items[child]);
//   }
//   return sortedList;
// }

void TraverseTree(in gprt::NNAccel record, uint distanceLevel, bool useAABBs, bool useOOBBs, float3 queryOrigin, float tmax, inout NNPayload payload, bool debug) {    
  payload.closestDistance = tmax;

  gprt::Buffer aabbs[MAX_LEVELS] = record.aabbs;
  gprt::Buffer oobbs[MAX_LEVELS] = record.oobbs;

  float minDist = 0.f;

  float closestDistAvg = 0.f;
  float closestDistM2 = 0.f; // m2 aggregates the squared distance from the mean
  float closestDistVar = 0.f;
  int totalHits = 0;


  uint32_t numClusters[MAX_LEVELS] = record.numClusters;

  List activeClusters[MAX_LEVELS];

  int fullTrail[MAX_LEVELS];
  for (int i = 0; i < MAX_LEVELS; ++i) fullTrail[i] = 0;


  int trail[MAX_LEVELS];
  for (int i = 0; i < MAX_LEVELS; ++i) trail[i] = 0;

  activeClusters[6] = intersectAndSortChildren(queryOrigin, payload.closestDistance, 0, numClusters[6], useAABBs, aabbs[6], useOOBBs, oobbs[6]);

  // Traverse clusters near to far
  for (trail[6] = 0; trail[6] < BRANCHING_FACTOR; ++trail[6]) {
    int l6Index = activeClusters[6].key(trail[6]);
    float l6MinDist = activeClusters[6].value(trail[6]);
    
    // Pop when nodes here on are culled
    if (l6Index >= BRANCHING_FACTOR) break;
    
    // Break when nodes here on are too far
    if (l6MinDist > payload.closestDistance) break;

    if (distanceLevel == 7) {
      payload.closestDistance = l6MinDist;
      continue;
    }
    
    #ifdef COLLECT_STATS
    payload.lHit[6]++; // Count this as traversing a l6
    #endif

    fullTrail[6] = l6Index;

    activeClusters[5] = intersectAndSortChildren(queryOrigin, payload.closestDistance, fullTrail[6] * BRANCHING_FACTOR, numClusters[5], useAABBs, aabbs[5], useOOBBs, oobbs[5]);

    // Traverse clusters near to far
    for (trail[5] = 0; trail[5] < BRANCHING_FACTOR; ++trail[5]) {
      int l5Index = activeClusters[5].key(trail[5]);
      float l5MinDist = activeClusters[5].value(trail[5]);
      
      // Pop when nodes here on are culled
      if (l5Index >= BRANCHING_FACTOR) break;
      
      // Break when nodes here on are too far
      if (l5MinDist > payload.closestDistance) break;

      if (distanceLevel == 6) {
        payload.closestDistance = l5MinDist;
        continue;
      }

      #ifdef COLLECT_STATS
      payload.lHit[5]++; // Count this as traversing a l5
      #endif

      fullTrail[5] = fullTrail[6] * BRANCHING_FACTOR + l5Index;

      activeClusters[4] = intersectAndSortChildren(queryOrigin, payload.closestDistance, fullTrail[5] * BRANCHING_FACTOR, numClusters[4], useAABBs, aabbs[4], useOOBBs, oobbs[4]);

      // Traverse clusters near to far
      for (trail[4] = 0; trail[4] < BRANCHING_FACTOR; ++trail[4]) {
        int l4Index = activeClusters[4].key(trail[4]);
        float l4MinDist = activeClusters[4].value(trail[4]);
        
        // Pop when nodes here on are culled
        if (l4Index >= BRANCHING_FACTOR) break;
        
        // Break when nodes here on are too far
        if (l4MinDist > payload.closestDistance) break;

        if (distanceLevel == 5) {
          payload.closestDistance = l4MinDist;
          continue;
        }
      
        #ifdef COLLECT_STATS
        payload.lHit[4]++; // Count this as traversing a leaf
        #endif

        fullTrail[4] = fullTrail[5] * BRANCHING_FACTOR + l4Index;

        activeClusters[3] = intersectAndSortChildren(queryOrigin, payload.closestDistance, fullTrail[4] * BRANCHING_FACTOR, numClusters[3], useAABBs, aabbs[3], useOOBBs, oobbs[3]);

        // Traverse clusters near to far
        for (trail[3] = 0; trail[3] < BRANCHING_FACTOR; ++trail[3]) {
          int l3Index = activeClusters[3].key(trail[3]);
          float l3MinDist = activeClusters[3].value(trail[3]);
          
          // Pop when nodes here on are culled
          if (l3Index >= BRANCHING_FACTOR) break;
          
          // Break when nodes here on are too far
          if (l3MinDist > payload.closestDistance) break;

          if (distanceLevel == 4) {
            payload.closestDistance = l3MinDist;
            continue;
          }
          
          #ifdef COLLECT_STATS
          payload.lHit[3]++; // Count this as traversing a l3
          #endif
          
          fullTrail[3] = fullTrail[4] * BRANCHING_FACTOR + l3Index;

          activeClusters[2] = intersectAndSortChildren(queryOrigin, payload.closestDistance, fullTrail[3] * BRANCHING_FACTOR, numClusters[2], useAABBs, aabbs[2], useOOBBs, oobbs[2]);

          // Traverse clusters near to far
          for (trail[2] = 0; trail[2] < BRANCHING_FACTOR; ++trail[2]) {
            int l2Index = activeClusters[2].key(trail[2]);
            float l2MinDist = activeClusters[2].value(trail[2]);
            
            // Pop when nodes here on are culled
            if (l2Index >= BRANCHING_FACTOR) break;
            
            // Break when nodes here on are too far
            if (l2MinDist > payload.closestDistance) break;

            if (distanceLevel == 3) {
              payload.closestDistance = l2MinDist;
              continue;
            }
            
            #ifdef COLLECT_STATS
            payload.lHit[2]++; // Count this as traversing a supercluster
            #endif

            fullTrail[2] = fullTrail[3] * BRANCHING_FACTOR + l2Index;
            activeClusters[1] = intersectAndSortChildren(queryOrigin, payload.closestDistance, fullTrail[2] * BRANCHING_FACTOR, numClusters[1], useAABBs, aabbs[1], useOOBBs, oobbs[1]);

            // Traverse clusters near to far
            for (trail[1] = 0; trail[1] < BRANCHING_FACTOR; ++trail[1]) {
              int l1Index = activeClusters[1].key(trail[1]);
              float l1MinDist = activeClusters[1].value(trail[1]);
              
              // Pop when nodes here on are culled
              if (l1Index >= BRANCHING_FACTOR) break;
              
              // Break when nodes here on are too far
              if (l1MinDist > payload.closestDistance) break;

              if (distanceLevel == 2) {
                payload.closestDistance = l1MinDist;
                continue;
              }
              
              #ifdef COLLECT_STATS
              payload.lHit[1]++; // Count this as traversing a cluster
              #endif

              fullTrail[1] = fullTrail[2] * BRANCHING_FACTOR + l1Index;
              int level = 0;
              
              activeClusters[level] = intersectAndSortChildren(queryOrigin, payload.closestDistance, fullTrail[1] * BRANCHING_FACTOR, numClusters[0], useAABBs, aabbs[0], useOOBBs, oobbs[0]);

              // Traverse all children from near to far
              for (trail[level] = 0; trail[level] < BRANCHING_FACTOR; ++trail[level]) {
                int child = activeClusters[level].key(trail[level]);
                float minDist = activeClusters[level].value(trail[level]);
                
                // Pop when nodes here on are culled
                if (child >= BRANCHING_FACTOR) break;
                
                // Break when nodes here on are too far
                if (minDist > payload.closestDistance) break;

                fullTrail[0] = fullTrail[1] * BRANCHING_FACTOR + child;
                if (fullTrail[0] >= numClusters[level]) break;

                #ifdef COLLECT_STATS
                payload.lHit[level]++; // Count this as a hit
                #endif

                TraverseLeaf(record, fullTrail[0], queryOrigin, tmax, payload, debug);
              }
            }
          }
        }
      }
    }
  }
  
  if (payload.closestPrimitive != -1) {
    payload.closestPrimitive = uint32_t(gprt::load<uint64_t>(record.ids, payload.closestPrimitive)); 
  }

  // if (debug) {
  //   printf("Total Hits %d True Dist: %f Closest Prim %d Average Dist %f Variance %f Std Dev %f\n", 
  //     totalHits, payload.closestDistance, payload.closestPrimitive, closestDistAvg, closestDistVar, sqrt(closestDistVar)
  //   );
  // }
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
  payload.primsHit = stats.primsHit;
  for (int i = 0; i < MAX_LEVELS; ++i) {
    payload.lHit[i] = stats.lHit[i];
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
    TraverseTree(knnAccel, distanceLevel, useAABBs, useOOBBs, queryOrigin, tMax, payload, debug);
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
  for (int i = 0; i < MAX_LEVELS; ++i) {
    stats.lHit[i] = payload.lHit[i];
  }
  #endif
}
#endif