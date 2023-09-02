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

GPRT_COMPUTE_PROGRAM(ComputeTriangleBounds, (NNAccel, record), (1,1,1)) {
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
  float radius = min(min(d.x, d.y), d.z);

  float3 aabbMin = gprt::load<float3>(record.aabb, 0);
  float3 aabbMax = gprt::load<float3>(record.aabb, 1);
  float diagonal = distance(aabbMax, aabbMin);

  c = (c - aabbMin) / (aabbMax - aabbMin);

  // uint64_t code = morton64_encode4D(c.x, c.y, c.z, radius / diagonal); 
  // uint64_t code = morton64_encode4D(c.x, c.y, c.z, radius / diagonal); 

  uint64_t code = 0;
  code |= uint64_t(primID); 
  code |= (uint64_t(hilbert_encode3D(c.x, c.y, c.z)) << 32ull);

  if(primID == 0) {
    printf("Prim %d coded ID %d\n",
      primID, uint32_t(code)
    );
  }

  uint64_t primitive = 0;
  primitive |= uint64_t(morton_encode3D(c.x, c.y, c.z));
  primitive |= (uint64_t(asuint(radius)) << 32ull);

  //uint64_t(morton_encode4D(c.x, c.y, c.z, radius / diagonal)) | (uint64_t(hilbert_encode3D(c.x, c.y, c.z)) << 32ull);
  // if (primID == 0) {
  //   float3 test = hilbert64_decode3D(code);
  //   printf("Original: %f %f %f, quantized %f %f %f\n", c.x, c.y, c.z, test.x, test.y, test.z);
  // }
  
  gprt::store<uint64_t>(record.codes, primID, code);
  gprt::store<uint64_t>(record.ids, primID, primitive);
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

GPRT_COMPUTE_PROGRAM(ComputeTriangleClusters, (NNAccel, record), (1,1,1)) {
  int clusterID = DispatchThreadID.x;
  if (clusterID >= record.numL0Clusters) return;
  uint32_t numPrims = record.numPrims;

  float3 clusterAabbMin = float3(1e20f, 1e20f, 1e20f); 
  float3 clusterAabbMax = -float3(1e20f, 1e20f, 1e20f);
  for (uint32_t i = 0; i < BRANCHING_FACTOR; ++i) {
    uint32_t idx = BRANCHING_FACTOR * clusterID + i;
    uint32_t primID = uint32_t(gprt::load<uint64_t>(record.codes, idx));
    if (primID >= numPrims || primID == -1) continue;
    float3 aabbMin, aabbMax;
    if (!getTriangleBounds(record.triangles, record.points, primID, aabbMin, aabbMax)) continue;
    clusterAabbMin = min(aabbMin, clusterAabbMin);
    clusterAabbMax = max(aabbMax, clusterAabbMax);
  }

  gprt::store<half3>(record.l0clusters, 2 * clusterID + 0, half3(clusterAabbMin));
  gprt::store<half3>(record.l0clusters, 2 * clusterID + 1, half3(clusterAabbMax));
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
    float3 aabbMin = gprt::load<half3>(record.l0clusters, clusterID * 2 + 0);
    float3 aabbMax = gprt::load<half3>(record.l0clusters, clusterID * 2 + 1);
    if (any(aabbMax < aabbMin)) continue; // invalid cluster
    l1ClusterAabbMin = min(aabbMin, l1ClusterAabbMin);
    l1ClusterAabbMax = max(aabbMax, l1ClusterAabbMax);
  }

  gprt::store<half3>(record.l1clusters, 2 * l1ClusterID + 0, half3(l1ClusterAabbMin));
  gprt::store<half3>(record.l1clusters, 2 * l1ClusterID + 1, half3(l1ClusterAabbMax));
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
    float3 aabbMin = gprt::load<half3>(record.l1clusters, clusterID * 2 + 0);
    float3 aabbMax = gprt::load<half3>(record.l1clusters, clusterID * 2 + 1);
    if (any(aabbMax < aabbMin)) continue; // invalid cluster
    l2ClusterAabbMin = min(aabbMin, l2ClusterAabbMin);
    l2ClusterAabbMax = max(aabbMax, l2ClusterAabbMax);
  }

  gprt::store<half3>(record.l2clusters, 2 * l2ClusterID + 0, half3(l2ClusterAabbMin));
  gprt::store<half3>(record.l2clusters, 2 * l2ClusterID + 1, half3(l2ClusterAabbMax));
}

GPRT_COMPUTE_PROGRAM(ComputeLeaves, (NNAccel, record), (1,1,1)) {
  int leafID = DispatchThreadID.x;
  if (leafID >= record.numLeaves) return;
  uint32_t numL2Clusters = record.numL2Clusters;

  float3 leafAabbMin = float3(1e20f, 1e20f, 1e20f); 
  float3 leafAabbMax = -float3(1e20f, 1e20f, 1e20f);
  for (uint32_t i = 0; i < BRANCHING_FACTOR; ++i) {
    uint32_t clusterID = BRANCHING_FACTOR * leafID + i;
    if (clusterID >= numL2Clusters) continue;
    float3 aabbMin = gprt::load<half3>(record.l2clusters, clusterID * 2 + 0);
    float3 aabbMax = gprt::load<half3>(record.l2clusters, clusterID * 2 + 1);
    if (any(aabbMax < aabbMin)) continue; // invalid cluster
    leafAabbMin = min(aabbMin, leafAabbMin);
    leafAabbMax = max(aabbMax, leafAabbMax);
  }

  // Dialate by the max search range
  gprt::store<float3>(record.leaves, 2 * leafID + 0, leafAabbMin - record.maxSearchRange);
  gprt::store<float3>(record.leaves, 2 * leafID + 1, leafAabbMax + record.maxSearchRange);
}

struct ClosestPointAttributes {
  int minDist;
}; 

GPRT_INTERSECTION_PROGRAM(ClosestNeighborIntersection, (NNAccel, record)) {
  uint leafID = PrimitiveIndex();
  float3 aabbMin = gprt::load<float3>(record.leaves, 2 * leafID + 0) + record.maxSearchRange;
  float3 aabbMax = gprt::load<float3>(record.leaves, 2 * leafID + 1) - record.maxSearchRange;
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

  payload.stats.w++; // Count this as traversing a leaf

  int numL2Clusters = record.numL2Clusters;
  int numL1Clusters = record.numL1Clusters;
  int numL0Clusters = record.numL0Clusters;
  int numPrims = record.numPrims;
  float maxSearchRange = record.maxSearchRange;
  float3 gaabbMin = gprt::load<float3>(record.aabb, 0);
  float3 gaabbMax = gprt::load<float3>(record.aabb, 1);
  float gdiag = distance(gaabbMax, gaabbMin);


  // Initialize active l0 list
  uint16_t activeL2Clusters[BRANCHING_FACTOR];
  half l2ClusterDistances[BRANCHING_FACTOR];
  [unroll]
  for (int l2 = 0; l2 < BRANCHING_FACTOR; ++l2) {
    activeL2Clusters[l2] = -1;
    l2ClusterDistances[l2] = 65504.f;
  }

  // Insert into active l0 list by distance far to near
  for (int l2 = 0; l2 < BRANCHING_FACTOR; ++l2) {
    int l2ClusterID = leafID * BRANCHING_FACTOR + l2;
    if (l2ClusterID > numL0Clusters) break;
    float3 aabbMin = gprt::load<half3>(record.l2clusters, l2ClusterID * 2 + 0);
    float3 aabbMax = gprt::load<half3>(record.l2clusters, l2ClusterID * 2 + 1);
    float minDist = getMinDist(queryOrigin, aabbMin, aabbMax);
    
    // Skip superclusters outside the search radius
    if (minDist >= maxSearchRange * maxSearchRange) continue;

    // Skip superclusters father than current closest primitive
    if (minDist >= payload.closestDistance) continue;

    // Insertion sort
    l2ClusterDistances[0] = half(minDist);
    activeL2Clusters[0] = uint16_t(l2);
    [unroll]
    for (int i = 1; i < BRANCHING_FACTOR; ++i) {
      if (l2ClusterDistances[i-1] >= l2ClusterDistances[i]) break;
      half tmpDist = l2ClusterDistances[i-1]; l2ClusterDistances[i-1] = l2ClusterDistances[i]; l2ClusterDistances[i] = tmpDist;
      uint16_t tmpID = activeL2Clusters[i-1]; activeL2Clusters[i-1] = activeL2Clusters[i]; activeL2Clusters[i] = tmpID;
    }
  }

  // Traverse clusters near to far
  for (int l2 = 0; l2 < BRANCHING_FACTOR; ++l2) {
    float minDist = float(l2ClusterDistances[BRANCHING_FACTOR - 1 - l2]);
    int l2ClusterID = leafID * BRANCHING_FACTOR + activeL2Clusters[BRANCHING_FACTOR - 1 - l2];

    // break out when all superclusters from here on are too far
    if (minDist >= payload.closestDistance) break;   
    
    payload.stats.w++; // Count this as traversing a l2

    // Initialize active l1 list
    uint16_t activeL1Clusters[BRANCHING_FACTOR];
    half l1ClusterDistances[BRANCHING_FACTOR];
    [unroll]
    for (int l1 = 0; l1 < BRANCHING_FACTOR; ++l1) {
      activeL1Clusters[l1] = -1;
      l1ClusterDistances[l1] = 65504.f;
    }

    // Insert into active l1 list by distance far to near
    for (int l1 = 0; l1 < BRANCHING_FACTOR; ++l1) {
      int l1ClusterID = l2ClusterID * BRANCHING_FACTOR + l1;
      if (l1ClusterID > numL1Clusters) break;
      float3 aabbMin = gprt::load<half3>(record.l1clusters, l1ClusterID * 2 + 0);
      float3 aabbMax = gprt::load<half3>(record.l1clusters, l1ClusterID * 2 + 1);
      float minDist = getMinDist(queryOrigin, aabbMin, aabbMax);
      
      // Skip superclusters outside the search radius
      if (minDist >= maxSearchRange * maxSearchRange) continue;

      // Skip superclusters father than current closest primitive
      if (minDist >= payload.closestDistance) continue;

      // Insertion sort
      l1ClusterDistances[0] = half(minDist);
      activeL1Clusters[0] = uint16_t(l1);
      [unroll]
      for (int i = 1; i < BRANCHING_FACTOR; ++i) {
        if (l1ClusterDistances[i-1] >= l1ClusterDistances[i]) break;
        half tmpDist = l1ClusterDistances[i-1]; l1ClusterDistances[i-1] = l1ClusterDistances[i]; l1ClusterDistances[i] = tmpDist;
        uint16_t tmpID = activeL1Clusters[i-1]; activeL1Clusters[i-1] = activeL1Clusters[i]; activeL1Clusters[i] = tmpID;
      }
    }

    // Traverse clusters near to far
    for (int l1 = 0; l1 < BRANCHING_FACTOR; ++l1) {
      float minDist = float(l1ClusterDistances[BRANCHING_FACTOR - 1 - l1]);
      int l1ClusterID = l2ClusterID * BRANCHING_FACTOR + activeL1Clusters[BRANCHING_FACTOR - 1 - l1];

      // break out when all superclusters from here on are too far
      if (minDist >= payload.closestDistance) break;   
      
      payload.stats.z++; // Count this as traversing a supercluster

      // Initialize active l0 cluster list
      uint16_t activeL0Clusters[BRANCHING_FACTOR];
      half l0ClusterDistances[BRANCHING_FACTOR];
      [unroll]
      for (int l0 = 0; l0 < BRANCHING_FACTOR; ++l0) {
        activeL0Clusters[l0] = -1;
        l0ClusterDistances[l0] = 65504.f;
      }

      // Insert into active cluster list by distance far to near
      for (int l0 = 0; l0 < BRANCHING_FACTOR; ++l0) {
        int clusterID = l1ClusterID * BRANCHING_FACTOR + l0;
        if (clusterID > numL0Clusters) break;
        float3 aabbMin = gprt::load<half3>(record.l0clusters, clusterID * 2 + 0);
        float3 aabbMax = gprt::load<half3>(record.l0clusters, clusterID * 2 + 1);
        float minDist = getMinDist(queryOrigin, aabbMin, aabbMax);
        
        // Skip clusters outside the search radius
        if (minDist >= maxSearchRange * maxSearchRange) continue;

        // Skip clusters father than current closest primitive
        if (minDist >= payload.closestDistance) continue;

        // Insertion sort
        l0ClusterDistances[0] = half(minDist);
        activeL0Clusters[0] = uint16_t(l0);
        [unroll]
        for (int i = 1; i < BRANCHING_FACTOR; ++i) {
          if (l0ClusterDistances[i-1] >= l0ClusterDistances[i]) break;
          half tmpDist = l0ClusterDistances[i-1]; l0ClusterDistances[i-1] = l0ClusterDistances[i]; l0ClusterDistances[i] = tmpDist;
          uint16_t tmpID = activeL0Clusters[i-1]; activeL0Clusters[i-1] = activeL0Clusters[i]; activeL0Clusters[i] = tmpID;
        }
      }

      // Traverse clusters near to far
      for (int l0 = 0; l0 < BRANCHING_FACTOR; ++l0) {
        float minDist = float(l0ClusterDistances[BRANCHING_FACTOR - 1 - l0]);
        int l0ClusterID = l1ClusterID * BRANCHING_FACTOR + activeL0Clusters[BRANCHING_FACTOR - 1 - l0];

        // break out when all clusters from here on are too far
        if (minDist >= payload.closestDistance) break;

        payload.stats.y++; // Count this as traversing a cluster

        // Initialize active primitive list
        int activePrimitives[BRANCHING_FACTOR];
        float primitiveDistances[BRANCHING_FACTOR];
        [unroll]
        for (int pidx = 0; pidx < BRANCHING_FACTOR; ++pidx) {
          activePrimitives[pidx] = -1;
          primitiveDistances[pidx] = 1e20f;
        }

        // Insert into active primitive list by distance far to near
        bool clustetHit = false;
        for (int pidx = 0; pidx < BRANCHING_FACTOR; ++pidx) {
          uint32_t itemID = l0ClusterID * BRANCHING_FACTOR + pidx;
          if (itemID > numPrims) break;
          uint64_t primitive = gprt::load<uint64_t>(record.ids, itemID);
          float3 pos = morton_decode3D(uint32_t(primitive));
          float radius = asfloat(uint32_t(primitive >> 32ull));
          pos = (pos * (gaabbMax - gaabbMin)) + gaabbMin;
          float minDist = max(distance(pos, queryOrigin) - radius, 0.f);      
          
          // Skip primitives outside the search radius
          if (minDist >= maxSearchRange) continue;

          // Skip clusters father than current closest primitive
          if (minDist * minDist >= payload.closestDistance) continue;

          // Hilbert codes and primitive IDs interleaved in the same buffer
          uint32_t primitiveID = uint32_t(gprt::load<uint64_t>(record.codes, itemID)); 
          primitiveDistances[0] = minDist * minDist;
          activePrimitives[0] = primitiveID;
          [unroll]
          for (int i = 1; i < BRANCHING_FACTOR; ++i) {
            if (primitiveDistances[i-1] >= primitiveDistances[i]) break;
            float tmpDist = primitiveDistances[i-1]; primitiveDistances[i-1] = primitiveDistances[i]; primitiveDistances[i] = tmpDist;
            int tmpID = activePrimitives[i-1]; activePrimitives[i-1] = activePrimitives[i]; activePrimitives[i] = tmpID;
          }
        }

        // Traverse all primitives in this cluster from near to far
        [unroll]
        for (int pidx = 0; pidx < BRANCHING_FACTOR; ++pidx) {
          float minDist = primitiveDistances[BRANCHING_FACTOR - 1 - pidx];
          int primitiveID = activePrimitives[BRANCHING_FACTOR - 1 - pidx];

          // break out when all primitives from here on are too far
          if (minDist >= payload.closestDistance) break;

          payload.stats.x++; // Count this as traversing a primitive      
          int3 tri = gprt::load<int3>(record.triangles, primitiveID);
          float3 a = gprt::load<float3>(record.points, tri.x);
          float3 b = gprt::load<float3>(record.points, tri.y);
          float3 c = gprt::load<float3>(record.points, tri.z);
          if (isnan(a.x)) continue;
          float dist = getTriangleDist2(queryOrigin, a, b, c);
          
          // Primitive farther than furthest
          if (dist > pow(maxSearchRange, 2.f)) continue;
          if (dist >= payload.closestDistance) continue;
          
          // Newly found closest primitive
          payload.closestDistance = dist;
          payload.closestPrimitive = primitiveID;
        }
      }
    }
  }

  gprt::ignoreHit(); // forces traversal to continue to next supercluster
}

