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

#define FLT_MAX 3.402823466e+38
#define FLT_MIN 1.175494351e-38

enum NN_FLAG : uint32_t
{
  NN_FLAG_NONE                            = 0x00, 
  NN_FLAG_ACCEPT_FIRST_NEIGHBOR_AND_END_SEARCH = 0x04,
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
//     float minmaxdist = 1e20f;
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
  return dot(d, d);
}

// minMaxDist computes the minimum of the maximum distances from p to points
// on r. If r is the bounding box of some geometric objects, then there is
// at least one object contained in r within minMaxDist(p, r) of p.
//
// Implemented per Definition 4 of "Nearest Neighbor Queries" by
// N. Roussopoulos, S. Kelley and F. Vincent, ACM SIGMOD, pages 71-79, 1995.
inline float getMinMaxDist(float3 origin, float3 aabbMin, float3 aabbMax) {
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
  float3 d2 = origin - select(origin <= c, aabbMin, aabbMax);
  float3 d = float3(S, S, S) - d1 * d1 + d2 * d2;
  return min(d.x, min(d.y, d.z));
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
  return maxDist;
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
float2 getTriangleDist2( float3 p, float3 a, float3 b, float3 c )
{
  float3 ba = b - a; float3 pa = p - a;
  float3 cb = c - b; float3 pb = p - b;
  float3 ac = a - c; float3 pc = p - c;
  float3 nor = cross( ba, ac );
  float minDist = (sign(dot(cross(ba,nor),pa)) +
     sign(dot(cross(cb,nor),pb)) +
     sign(dot(cross(ac,nor),pc))<2.0f)
     ?
     min( min(
     _dot2(ba*clamp(dot(ba,pa)/_dot2(ba),0.0f,1.0f)-pa),
     _dot2(cb*clamp(dot(cb,pb)/_dot2(cb),0.0f,1.0f)-pb) ),
     _dot2(ac*clamp(dot(ac,pc)/_dot2(ac),0.0f,1.0f)-pc) )
     :
     dot(nor,pa)*dot(nor,pa)/_dot2(nor);

  float maxDist = max(max(_dot2(pa), _dot2(pb)), _dot2(pc));
  return float2(minDist, maxDist);
}

#ifdef GPRT_DEVICE

// Payload for nearest neighbor queries
struct [raypayload] NNPayload {
  float closestDistance : read(anyhit, caller) : write(anyhit, caller);
  int closestPrimitive : read(anyhit, caller) : write(anyhit, caller);
  int4 stats : read(anyhit, caller) : write(anyhit, caller);
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

void TraverseLeaf(in gprt::NNAccel record, uint NNFlags, float3 queryOrigin, float tmin, float tmax, uint leafID, inout NNPayload payload, bool debug) {
  payload.stats.w++; // Count this as traversing a leaf

  #ifdef ENABLE_QUANTIZATION
  // Load treelet
  float4 l3Treelet = gprt::load<float4>(record.treelets, leafID);
  uint32_t l3Exponent32 = asuint(l3Treelet.w);
  float3 l3Translate = l3Treelet.xyz;
  float3 l3Span = float3(
    asfloat(((l3Exponent32 >>  0) & 255) << 23),
    asfloat(((l3Exponent32 >>  8) & 255) << 23),
    asfloat(((l3Exponent32 >> 16) & 255) << 23)
  );
  float3 l3Scale = l3Span / (255.f - 1.f);
  #endif
  
  int numL3Clusters = record.numL3Clusters;
  int numL2Clusters = record.numL2Clusters;
  int numL1Clusters = record.numL1Clusters;
  int numL0Clusters = record.numL0Clusters;
  int numPrims = record.numPrims;
  
  // Initialize active l2 list
  #if defined(ENABLE_QUEUE_QUANTIZATION) && defined(ENABLE_QUANTIZATION)
  //       15-8    7-0
  // bits: [ID] [Distance]
  // Quantizing by the distance to the farthest corner of the treelet. An ID of 255 means "empty".
  float l2QueueTranslate = getMinDist(queryOrigin, l3Treelet.xyz, l3Treelet.xyz + l3Span);
  float l2QueueScale = getMaxDist(queryOrigin, l3Treelet.xyz, l3Treelet.xyz + l3Span) - l2QueueTranslate;
  uint16_t activeL2Clusters[BRANCHING_FACTOR];
  [unroll]
  for (int l2 = 0; l2 < BRANCHING_FACTOR; ++l2) {
    activeL2Clusters[l2] = 255 << 8 | 255;
  }
  #else
  uint32_t activeL2Clusters[BRANCHING_FACTOR];
  float l2ClusterDistances[BRANCHING_FACTOR];
  [unroll]
  for (int l2 = 0; l2 < BRANCHING_FACTOR; ++l2) {
    activeL2Clusters[l2] = -1;
    l2ClusterDistances[l2] = FLT_MAX;
  }
  #endif

  // Insert into active l0 list by distance far to near
  for (int l2 = 0; l2 < BRANCHING_FACTOR; ++l2) {
    float minDistCorrection = 0.f;
    int l2ClusterID = leafID * BRANCHING_FACTOR + l2;
    if (l2ClusterID > numL2Clusters) break;

    #ifdef ENABLE_QUANTIZATION
    uint64_t child = gprt::load<uint64_t>(record.children, l2ClusterID);
    float3 aabbMin = float3(((child >>  0ull) & 255), 
                             ((child >>  8ull) & 255), 
                             ((child >> 16ull) & 255));
    float3 aabbMax = float3(((child >> 24ull) & 255) + 1u, 
                             ((child >> 32ull) & 255) + 1u, 
                             ((child >> 40ull) & 255) + 1u);
    aabbMin = aabbMin * l3Scale + l3Translate;
    aabbMax = aabbMax * l3Scale + l3Translate;
    #else
    uint32_t offset = 2 * record.numL0Clusters + 2 * record.numL1Clusters;
    float3 aabbMin = gprt::load<float3>(record.clusters, offset + l2ClusterID * 2 + 0);
    float3 aabbMax = gprt::load<float3>(record.clusters, offset + l2ClusterID * 2 + 1);
    #endif
    float minDist = getMinDist(queryOrigin, aabbMin, aabbMax);
    float maxDist = getMaxDist(queryOrigin, aabbMin, aabbMax);

    // Range Culling: Farthest corner closer than min range
    if (maxDist <= tmin * tmin) continue;
    
    // Upward culling, skip superclusters outside the search radius
    if (minDist >= tmax * tmax) continue;

    // Upward culling, skip superclusters father than current closest primitive
    if (minDist >= payload.closestDistance) continue;

    // Insertion sort
    #if defined(ENABLE_QUEUE_QUANTIZATION) && defined(ENABLE_QUANTIZATION)
    activeL2Clusters[0] = (uint16_t(l2) << 8) | uint16_t(((minDist - l2QueueTranslate) / l2QueueScale) * 255.f);
    [unroll]
    for (int i = 1; i < BRANCHING_FACTOR; ++i) {
      if ((activeL2Clusters[i-1] & 255) >= (activeL2Clusters[i] & 255)) break;
      uint16_t tmpID = activeL2Clusters[i-1]; activeL2Clusters[i-1] = activeL2Clusters[i]; activeL2Clusters[i] = tmpID;
    }
    #else
    l2ClusterDistances[0] = minDist;
    activeL2Clusters[0] = l2;
    [unroll]
    for (int i = 1; i < BRANCHING_FACTOR; ++i) {
      if (l2ClusterDistances[i-1] >= l2ClusterDistances[i]) break;
      float tmpDist = l2ClusterDistances[i-1]; l2ClusterDistances[i-1] = l2ClusterDistances[i]; l2ClusterDistances[i] = tmpDist;
      uint32_t tmpID = activeL2Clusters[i-1]; activeL2Clusters[i-1] = activeL2Clusters[i]; activeL2Clusters[i] = tmpID;
    }
    #endif

    // Downward culling... truncate closest distance to the pessemistic distance of this cluster
    #ifdef ENABLE_DOWNAWARD_CULLING
    #ifdef ENABLE_QUANTIZATION
    float pessimisticDistance = getMaxDist(queryOrigin, aabbMin, aabbMax);
    #else
    float pessimisticDistance = getMinMaxDist(queryOrigin, aabbMin, aabbMax);
    #endif
    if (pessimisticDistance < payload.closestDistance) payload.closestDistance = pessimisticDistance;
    #endif
  }

  // Traverse clusters near to far
  for (int l2 = 0; l2 < BRANCHING_FACTOR; ++l2) {
    #if defined(ENABLE_QUEUE_QUANTIZATION) && defined(ENABLE_QUANTIZATION)
    uint16_t tmp = activeL2Clusters[BRANCHING_FACTOR - 1 - l2];
    float minDist = ((tmp & 255) / 255.f) * l2QueueScale + l2QueueTranslate;
    int l2Index = (tmp >> 8);
    #else
    float minDist = l2ClusterDistances[BRANCHING_FACTOR - 1 - l2];
    int l2Index = activeL2Clusters[BRANCHING_FACTOR - 1 - l2];
    #endif

    // break out when all superclusters from here on are too far
    if (l2Index >= BRANCHING_FACTOR || minDist >= payload.closestDistance) break;   
    
    payload.stats.w++; // Count this as traversing a l2
    
    int l2ClusterID = leafID * BRANCHING_FACTOR + l2Index;

    #ifdef ENABLE_QUANTIZATION
    // Load treelet
    float4 l2Treelet = gprt::load<float4>(record.treelets, numL3Clusters + l2ClusterID);
    uint32_t l2Exponent32 = asuint(l2Treelet.w);
    float3 l2Translate = l2Treelet.xyz;
    float3 l2Span = float3(
      asfloat(((l2Exponent32 >>  0) & 255) << 23),
      asfloat(((l2Exponent32 >>  8) & 255) << 23),
      asfloat(((l2Exponent32 >> 16) & 255) << 23)
    );
    float3 l2Scale = l2Span / (255.f - 1.f);
    #endif

    // Initialize active l1 list
    #if defined(ENABLE_QUEUE_QUANTIZATION) && defined(ENABLE_QUANTIZATION)
    //       15-8    7-0
    // bits: [ID] [Distance]
    // Quantizing by the distance to the farthest corner of the treelet. An ID of 255 means "empty".
    float l1QueueTranslate = getMinDist(queryOrigin, l2Treelet.xyz, l2Treelet.xyz + l2Span);
    float l1QueueScale = getMaxDist(queryOrigin, l2Treelet.xyz, l2Treelet.xyz + l2Span) - l2QueueTranslate;
    uint16_t activeL1Clusters[BRANCHING_FACTOR];
    [unroll]
    for (int l1 = 0; l1 < BRANCHING_FACTOR; ++l1) {
      activeL1Clusters[l1] = 255 << 8 | 255;
    }
    #else
    uint32_t activeL1Clusters[BRANCHING_FACTOR];
    float l1ClusterDistances[BRANCHING_FACTOR];
    [unroll]
    for (int l1 = 0; l1 < BRANCHING_FACTOR; ++l1) {
      activeL1Clusters[l1] = -1;
      l1ClusterDistances[l1] = FLT_MAX;
    }
    #endif

    // Insert into active l1 list by distance far to near
    for (int l1 = 0; l1 < BRANCHING_FACTOR; ++l1) {
      int l1ClusterID = l2ClusterID * BRANCHING_FACTOR + l1;
      if (l1ClusterID > numL1Clusters) break;
      #ifdef ENABLE_QUANTIZATION
      uint64_t child = gprt::load<uint64_t>(record.children, numL2Clusters + l1ClusterID);
      float3 aabbMin = float3(((child >>  0ull) & 255), 
                              ((child >>  8ull) & 255), 
                              ((child >> 16ull) & 255));
      float3 aabbMax = float3(((child >> 24ull) & 255) + 1u, 
                              ((child >> 32ull) & 255) + 1u, 
                              ((child >> 40ull) & 255) + 1u);
      aabbMin = aabbMin * l2Scale + l2Translate;
      aabbMax = aabbMax * l2Scale + l2Translate;
      #else
      uint32_t offset = 2 * record.numL0Clusters;
      float3 aabbMin = gprt::load<float3>(record.clusters, offset + l1ClusterID * 2 + 0);
      float3 aabbMax = gprt::load<float3>(record.clusters, offset + l1ClusterID * 2 + 1);
      #endif
      float minDist = getMinDist(queryOrigin, aabbMin, aabbMax);
      float maxDist = getMaxDist(queryOrigin, aabbMin, aabbMax);

      // Range Culling: Farthest corner closer than min range
      if (maxDist <= tmin * tmin) continue;
      
      // Upward Culling: Skip superclusters outside the search radius
      if (minDist >= tmax * tmax) continue;

      // Upward Culling: Skip superclusters father than current closest primitive
      if (minDist >= payload.closestDistance) continue;

      // Insertion sort
      #if defined(ENABLE_QUEUE_QUANTIZATION) && defined(ENABLE_QUANTIZATION)
      activeL1Clusters[0] = (uint16_t(l1) << 8) | uint16_t(((minDist - l1QueueTranslate) / l1QueueScale) * 255.f);
      [unroll]
      for (int i = 1; i < BRANCHING_FACTOR; ++i) {
        if ((activeL1Clusters[i-1] & 255) >= (activeL1Clusters[i] & 255)) break;
        uint16_t tmpID = activeL1Clusters[i-1]; activeL1Clusters[i-1] = activeL1Clusters[i]; activeL1Clusters[i] = tmpID;
      }
      #else
      l1ClusterDistances[0] = minDist;
      activeL1Clusters[0] = l1;
      [unroll]
      for (int i = 1; i < BRANCHING_FACTOR; ++i) {
        if (l1ClusterDistances[i-1] >= l1ClusterDistances[i]) break;
        float tmpDist = l1ClusterDistances[i-1]; l1ClusterDistances[i-1] = l1ClusterDistances[i]; l1ClusterDistances[i] = tmpDist;
        uint32_t tmpID = activeL1Clusters[i-1]; activeL1Clusters[i-1] = activeL1Clusters[i]; activeL1Clusters[i] = tmpID;
      }
      #endif

      // Downward culling... truncate closest distance to the pessemistic distance of this cluster
      #ifdef ENABLE_DOWNAWARD_CULLING
      #ifdef ENABLE_QUANTIZATION
      float pessimisticDistance = getMaxDist(queryOrigin, aabbMin, aabbMax);
      #else
      float pessimisticDistance = getMinMaxDist(queryOrigin, aabbMin, aabbMax);
      #endif
      if (pessimisticDistance < payload.closestDistance) payload.closestDistance = pessimisticDistance;
      #endif
    }

    // Traverse clusters near to far
    for (int l1 = 0; l1 < BRANCHING_FACTOR; ++l1) {
      #if defined(ENABLE_QUEUE_QUANTIZATION) && defined(ENABLE_QUANTIZATION)
      uint16_t tmp = activeL1Clusters[BRANCHING_FACTOR - 1 - l1];
      float minDist = ((tmp & 255) / 255.f) * l1QueueScale + l1QueueTranslate;
      int l1Index = (tmp >> 8);
      #else
      float minDist = l1ClusterDistances[BRANCHING_FACTOR - 1 - l1];
      int l1Index = activeL1Clusters[BRANCHING_FACTOR - 1 - l1];
      #endif
      
      // break out when all superclusters from here on are too far
      if (l1Index >= BRANCHING_FACTOR || minDist >= payload.closestDistance) break;   
      
      payload.stats.z++; // Count this as traversing a supercluster

      int l1ClusterID = l2ClusterID * BRANCHING_FACTOR + l1Index;

      #ifdef ENABLE_QUANTIZATION
      // Load treelet
      float4 l1Treelet = gprt::load<float4>(record.treelets, numL3Clusters + numL2Clusters + l1ClusterID);
      uint32_t l1Exponent32 = asuint(l1Treelet.w);
      float3 l1Translate = l1Treelet.xyz;
      float3 l1Span = float3(
        asfloat(((l1Exponent32 >>  0) & 255) << 23),
        asfloat(((l1Exponent32 >>  8) & 255) << 23),
        asfloat(((l1Exponent32 >> 16) & 255) << 23)
      );
      float3 l1Scale = l1Span / (255.f - 1.f);
      #endif

      // Initialize active l0 cluster list
      #if defined(ENABLE_QUEUE_QUANTIZATION) && defined(ENABLE_QUANTIZATION)
      //       15-8    7-0
      // bits: [ID] [Distance]
      // Quantizing by the distance to the farthest corner of the treelet. An ID of 255 means "empty".
      float l0QueueTranslate = getMinDist(queryOrigin, l1Treelet.xyz, l1Treelet.xyz + l1Span);
      float l0QueueScale = getMaxDist(queryOrigin, l1Treelet.xyz, l1Treelet.xyz + l1Span) - l0QueueTranslate;
      uint16_t activeL0Clusters[BRANCHING_FACTOR];
      [unroll]
      for (int l0 = 0; l0 < BRANCHING_FACTOR; ++l0) {
        activeL0Clusters[l0] = 255 << 8 | 255;
      }
      #else
      uint32_t activeL0Clusters[BRANCHING_FACTOR];
      float l0ClusterDistances[BRANCHING_FACTOR];
      [unroll]
      for (int l0 = 0; l0 < BRANCHING_FACTOR; ++l0) {
        activeL0Clusters[l0] = -1;
        l0ClusterDistances[l0] = FLT_MAX;
      }
      #endif

      // Insert into active cluster list by distance far to near
      for (int l0 = 0; l0 < BRANCHING_FACTOR; ++l0) {
        int clusterID = l1ClusterID * BRANCHING_FACTOR + l0;
        if (clusterID > numL0Clusters) break;
        #ifdef ENABLE_QUANTIZATION
        uint64_t child = gprt::load<uint64_t>(record.children, numL2Clusters + numL1Clusters + clusterID);
        float3 aabbMin = float3(((child >>  0ull) & 255), 
                                ((child >>  8ull) & 255), 
                                ((child >> 16ull) & 255));
        float3 aabbMax = float3(((child >> 24ull) & 255) + 1u, 
                                ((child >> 32ull) & 255) + 1u, 
                                ((child >> 40ull) & 255) + 1u);
        aabbMin = aabbMin * l1Scale + l1Translate;
        aabbMax = aabbMax * l1Scale + l1Translate;
        #else
        uint32_t offset = 2 * record.numL0Clusters;
        float3 aabbMin = gprt::load<float3>(record.clusters, clusterID * 2 + 0);
        float3 aabbMax = gprt::load<float3>(record.clusters, clusterID * 2 + 1);
        #endif
        float minDist = getMinDist(queryOrigin, aabbMin, aabbMax);
        float maxDist = getMaxDist(queryOrigin, aabbMin, aabbMax);

        // Range Culling: Farthest corner closer than min range
        if (maxDist <= tmin * tmin) continue;
        
        // Skip clusters outside the search radius
        if (minDist >= tmax * tmax) continue;

        // Skip clusters father than current closest primitive
        if (minDist >= payload.closestDistance) continue;

        // Insertion sort
        #if defined(ENABLE_QUEUE_QUANTIZATION) && defined(ENABLE_QUANTIZATION)
        activeL0Clusters[0] = (uint16_t(l0) << 8) | uint16_t(((minDist - l0QueueTranslate) / l0QueueScale) * 255.f);
        [unroll]
        for (int i = 1; i < BRANCHING_FACTOR; ++i) {
          if ((activeL0Clusters[i-1] & 255) >= (activeL0Clusters[i] & 255)) break;
          uint16_t tmpID = activeL0Clusters[i-1]; activeL0Clusters[i-1] = activeL0Clusters[i]; activeL0Clusters[i] = tmpID;
        }
        #else
        l0ClusterDistances[0] = minDist;
        activeL0Clusters[0] = l0;
        [unroll]
        for (int i = 1; i < BRANCHING_FACTOR; ++i) {
          if (l0ClusterDistances[i-1] >= l0ClusterDistances[i]) break;
          float tmpDist = l0ClusterDistances[i-1]; l0ClusterDistances[i-1] = l0ClusterDistances[i]; l0ClusterDistances[i] = tmpDist;
          uint32_t tmpID = activeL0Clusters[i-1]; activeL0Clusters[i-1] = activeL0Clusters[i]; activeL0Clusters[i] = tmpID;
        }
        #endif

        // Downward culling... truncate closest distance to the pessemistic distance of this cluster
        #ifdef ENABLE_DOWNAWARD_CULLING
        #ifdef ENABLE_QUANTIZATION
        float pessimisticDistance = getMaxDist(queryOrigin, aabbMin, aabbMax);
        #else
        float pessimisticDistance = getMinMaxDist(queryOrigin, aabbMin, aabbMax);
        #endif
        if (pessimisticDistance < payload.closestDistance) payload.closestDistance = pessimisticDistance;
        #endif
      }

      // Traverse clusters near to far
      for (int l0 = 0; l0 < BRANCHING_FACTOR; ++l0) {
        #if defined(ENABLE_QUEUE_QUANTIZATION) && defined(ENABLE_QUANTIZATION)
        uint16_t tmp = activeL0Clusters[BRANCHING_FACTOR - 1 - l0];
        float minDist = ((tmp & 255) / 255.f) * l0QueueScale + l0QueueTranslate;
        int l0Index = (tmp >> 8);
        #else
        float minDist = l0ClusterDistances[BRANCHING_FACTOR - 1 - l0];
        int l0Index = activeL0Clusters[BRANCHING_FACTOR - 1 - l0];
        #endif

        // break out when all clusters from here on are too far
        if (l0Index >= BRANCHING_FACTOR || minDist >= payload.closestDistance) break;

        payload.stats.y++; // Count this as traversing a cluster

        int l0ClusterID = l1ClusterID * BRANCHING_FACTOR + l0Index;
        #ifdef ENABLE_QUANTIZATION
        // Load treelet
        float4 l0Treelet = gprt::load<float4>(record.treelets, numL3Clusters + numL2Clusters + numL1Clusters + l0ClusterID);
        uint32_t l0Exponent32 = asuint(l0Treelet.w);
        float3 l0Translate = l0Treelet.xyz;
        float3 l0Span = float3(
          asfloat(((l0Exponent32 >>  0) & 255) << 23),
          asfloat(((l0Exponent32 >>  8) & 255) << 23),
          asfloat(((l0Exponent32 >> 16) & 255) << 23)
        );
        float3 l0Scale = l0Span / (255.f - 1.f);
        #endif

        // Initialize active primitive list
        #if defined(ENABLE_QUEUE_QUANTIZATION) && defined(ENABLE_QUANTIZATION)
        //       15-8    7-0
        // bits: [ID] [Distance]
        // Quantizing by the distance to the farthest corner of the treelet. An ID of 255 means "empty".
        float primQueueTranslate = getMinDist(queryOrigin, l0Treelet.xyz, l0Treelet.xyz + l0Span);
        float primQueueScale = getMaxDist(queryOrigin, l0Treelet.xyz, l0Treelet.xyz + l0Span) - primQueueTranslate;
        uint16_t activePrimitives[BRANCHING_FACTOR];
        [unroll]
        for (int l0 = 0; l0 < BRANCHING_FACTOR; ++l0) {
          activePrimitives[l0] = 255 << 8 | 255;
        }
        #else
        uint32_t activePrimitives[BRANCHING_FACTOR];
        float primitiveDistances[BRANCHING_FACTOR];
        [unroll]
        for (int pidx = 0; pidx < BRANCHING_FACTOR; ++pidx) {
          activePrimitives[pidx] = -1;
          primitiveDistances[pidx] = FLT_MAX;
        }
        #endif

        // Insert into active primitive list by distance far to near
        for (int pidx = 0; pidx < BRANCHING_FACTOR; ++pidx) {
          uint32_t itemID = l0ClusterID * BRANCHING_FACTOR + pidx;
          if (itemID > numPrims) break;

          #ifdef ENABLE_QUANTIZATION
          #ifdef ENABLE_BOUNDING_BALLS
          uint32_t child = gprt::load<uint32_t>(record.children, 2 * (numL2Clusters + numL1Clusters + numL0Clusters) + itemID);
          float3 position = float3(((child >> 0) & 255), 
                                  ((child >>  8) & 255), 
                                  ((child >> 16) & 255));
          float radius = asfloat(uint32_t((child >> 32)));
          position = position * l0Scale + l0Translate;
          radius = radius * max(l0Scale.x, max(l0Scale.y, l0Scale.z));

          #else
          uint64_t child = gprt::load<uint64_t>(record.children, numL2Clusters + numL1Clusters + numL0Clusters + itemID);
          float3 aabbMin = float3(((child >>  0ull) & 255), 
                                  ((child >>  8ull) & 255), 
                                  ((child >> 16ull) & 255));
          float3 aabbMax = float3(((child >> 24ull) & 255) + 1u, 
                                  ((child >> 32ull) & 255) + 1u, 
                                  ((child >> 40ull) & 255) + 1u);
          aabbMin = aabbMin * l0Scale + l0Translate;
          aabbMax = aabbMax * l0Scale + l0Translate;
          #endif
          #else
          uint32_t primitiveID = uint32_t(gprt::load<uint64_t>(record.codes, itemID)); 
          float3 aabbMin = gprt::load<float3>(record.primAABBs, primitiveID * 2 + 0);
          float3 aabbMax = gprt::load<float3>(record.primAABBs, primitiveID * 2 + 1);
          #endif

          #if defined(ENABLE_QUANTIZATION) && defined(ENABLE_BOUNDING_BALLS) 
          float centerDist = distance(queryOrigin, position);
          float minDist = centerDist - radius;
          float maxDist = centerDist + radius;
          minDist = minDist * minDist;
          maxDist = maxDist * maxDist;
          #else
          float minDist = getMinDist(queryOrigin, aabbMin, aabbMax);
          float maxDist = getMaxDist(queryOrigin, aabbMin, aabbMax);
          #endif

          // Range Culling: Farthest corner closer than min range
          if (maxDist <= tmin * tmin) continue;

          // Upward Culling: Skip primitives outside the search radius
          if (minDist >= tmax * tmax) continue;

          // Upward Culling: Skip clusters father than current closest primitive
          if (minDist >= payload.closestDistance) continue;

          // Insertion sort
          #if defined(ENABLE_QUEUE_QUANTIZATION) && defined(ENABLE_QUANTIZATION)
          activePrimitives[0] = (uint16_t(pidx) << 8) | uint16_t(((minDist - primQueueTranslate) / primQueueScale) * 255.f);
          [unroll]
          for (int i = 1; i < BRANCHING_FACTOR; ++i) {
            if ((activePrimitives[i-1] & 255) >= (activePrimitives[i] & 255)) break;
            uint16_t tmpID = activePrimitives[i-1]; activePrimitives[i-1] = activePrimitives[i]; activePrimitives[i] = tmpID;
          }
          #else
          primitiveDistances[0] = minDist;
          activePrimitives[0] = pidx;
          [unroll]
          for (int i = 1; i < BRANCHING_FACTOR; ++i) {
            if (primitiveDistances[i-1] >= primitiveDistances[i]) break;
            float tmpDist = primitiveDistances[i-1]; primitiveDistances[i-1] = primitiveDistances[i]; primitiveDistances[i] = tmpDist;
            uint32_t tmpID = activePrimitives[i-1]; activePrimitives[i-1] = activePrimitives[i]; activePrimitives[i] = tmpID;
          }
          #endif

          // Downward culling... truncate closest distance to the pessemistic distance of this cluster
          #ifdef ENABLE_DOWNAWARD_CULLING
          #ifdef ENABLE_QUANTIZATION
          float pessimisticDistance = maxDist;
          #else
          float pessimisticDistance = getMinMaxDist(queryOrigin, aabbMin, aabbMax);
          #endif
          if (pessimisticDistance < payload.closestDistance) payload.closestDistance = pessimisticDistance;
          #endif
        }

        // Traverse all primitives in this cluster from near to far
        [unroll]
        for (int pidx = 0; pidx < BRANCHING_FACTOR; ++pidx) {
          #if defined(ENABLE_QUEUE_QUANTIZATION) && defined(ENABLE_QUANTIZATION)
          uint16_t tmp = activePrimitives[BRANCHING_FACTOR - 1 - pidx];
          float minDist = ((tmp & 255) / 255.f) * primQueueScale + primQueueTranslate;
          int idx = (tmp >> 8);
          #else
          float minDist = primitiveDistances[BRANCHING_FACTOR - 1 - pidx];
          int idx = activePrimitives[BRANCHING_FACTOR - 1 - pidx];
          #endif

          // break out when all primitives from here on are too far
          if (idx >= BRANCHING_FACTOR || minDist >= payload.closestDistance) break;

          payload.stats.x++; // Count this as traversing a primitive
          uint32_t itemID = l0ClusterID * BRANCHING_FACTOR + idx;
          uint32_t primitiveID = uint32_t(gprt::load<uint64_t>(record.codes, itemID)); 
          int3 tri = gprt::load<int3>(record.triangles, primitiveID);
          float3 a = gprt::load<float3>(record.points, tri.x);
          float3 b = gprt::load<float3>(record.points, tri.y);
          float3 c = gprt::load<float3>(record.points, tri.z);
          if (isnan(a.x)) continue;
          float2 dists = getTriangleDist2(queryOrigin, a, b, c);

          // Closest primitive distance farther than max range
          if (dists.x > tmax * tmax) continue;

          // Farthest primitive distance closer than min range
          if (dists.y <= tmin * tmin) continue;

          // Primitive intersects tmin sphere
          if (dists.x <= tmin*tmin && dists.y > tmin*tmin){
            dists.x = tmin * tmin;
          }
          
          // Primitive farther than furthest
          if (dists.x >= payload.closestDistance) continue;
          
          // Newly found closest primitive
          payload.closestDistance = dists.x;
          payload.closestPrimitive = primitiveID;

          // If we're accepting the first hit, stop traversal now.
          if ((NNFlags & RAY_FLAG_ACCEPT_FIRST_HIT_AND_END_SEARCH) != 0) return;

          // Stop now if we've found something that hits tmin.
          if (dists.x == tmin * tmin) return;
        }
      }
    }
  }
}

void TraverseTree(in gprt::NNAccel record, uint NNFlags, float3 queryOrigin, float tmin, float tmax, inout NNPayload payload, bool debug) {
  float dist = tmax * tmax;// 1e20f;
  uint stack[32];
  uint stackPtr = 0;
  stackPtr ++;
  stack[stackPtr] = 0; // root

  while (stackPtr > 0)
  {
    bool gotoNext = false;
    int4 node = gprt::load<int4>(record.lbvhNodes, stack[stackPtr]);
    stackPtr = stackPtr - 1;

    // while left and right contain children
    while (node.x != -1 && node.y != -1) {
      int4 children[2] = { 
          gprt::load<int4>(record.lbvhNodes, node.x), 
          gprt::load<int4>(record.lbvhNodes, node.y) 
      };

      float3 leftAABB[2] = {
          gprt::load<float3>(record.lbvhAabbs, 2 * node.x + 0), // + record.maxSearchRange, 
          gprt::load<float3>(record.lbvhAabbs, 2 * node.x + 1), // - record.maxSearchRange, 
      };

      float3 rightAABB[2] = {
          gprt::load<float3>(record.lbvhAabbs, 2 * node.y + 0), // + record.maxSearchRange, 
          gprt::load<float3>(record.lbvhAabbs, 2 * node.y + 1), // - record.maxSearchRange, 
      };

      float distL = _dot2(max(max(leftAABB[0] - queryOrigin, float3(0.f, 0.f, 0.f)), queryOrigin - leftAABB[1]));
      float distR = _dot2(max(max(rightAABB[0] - queryOrigin, float3(0.f, 0.f, 0.f)), queryOrigin - rightAABB[1]));
      if (distL < dist && distR < dist) {
        if (distL < distR) {
          stackPtr++;
          stack[stackPtr] = node.y;
          node = children[0];
        }
        else {
          stackPtr++;
          stack[stackPtr] = node.x;
          node = children[1];
        }
      }
      else if (distL < dist)
        node = children[0];
      else if (distR < dist)
        node = children[1];
      else {
        gotoNext = true;
        break;
      }
    }
    if (gotoNext) continue;

    // Traverse leaf
    uint32_t offset = 2 * record.numL0Clusters + 2 * record.numL1Clusters + 2 * record.numL2Clusters;
    int leafID = node.w;
    float3 aabbMin = gprt::load<float3>(record.clusters, offset + 2 * leafID + 0); // + record.maxSearchRange;
    float3 aabbMax = gprt::load<float3>(record.clusters, offset + 2 * leafID + 1); // - record.maxSearchRange;

    float minDist = getMinDist(queryOrigin, aabbMin, aabbMax);
    if (minDist > dist) continue;

    TraverseLeaf(record, NNFlags, queryOrigin, tmin, tmax, leafID, payload, debug);
    dist = min(dist, payload.closestDistance);
  }
}

void TraceNN(
  in gprt::NNAccel knnAccel, 
  uint NNFlags,
  float3 queryOrigin, 
  float tMin, 
  float tMax, 
  out int closestPrimitive, 
  out float closestDistance, inout int4 stats, bool debug) 
{
  NNPayload payload;
  payload.closestPrimitive = -1;
  payload.closestDistance = tMax * tMax; //knnAccel.maxSearchRange * knnAccel.maxSearchRange;
  payload.stats = stats;

  if (tMax > 0.f) {
    #ifndef ENABLE_LBVH_REFERENCE
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
    #else
    TraverseTree(knnAccel, NNFlags, queryOrigin, tMin, tMax, payload, debug);
    #endif
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
  // payload.stats.w++;
  stats = payload.stats;
}
#endif