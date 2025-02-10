/*
 * =====================================================================================
 *
 *    Filename:  sfc.slang
 *
 *    Description:  This file implements functions for encoding and
 *                  decoding between normalized Cartesian coordinates and space-filling curves,
 *                  specifically Hilbert and Morton codes. It provides efficient algorithms
 *                  for conversions in both directions - from Cartesian coordinates to Hilbert
 *                  and Morton codes and vice versa. These functions are essential for applications
 *                  that require mapping multidimensional data to one-dimensional space while
 *                  preserving locality, commonly used in computer graphics and spatial data analysis.
 *
 *                  The file contains the following main functions:
 *                  - hilbert_encode : Convert normalized Cartesian coordinates to Hilbert code.
 *                  - hilbert_decode : Convert Hilbert code to normalized Cartesian coordinates.
 *                  - morton_encode  : Convert normalized Cartesian coordinates to Morton code.
 *                  - morton_decode  : Convert Morton code to normalized Cartesian coordinates.
 *
 *                  Each of these functions additionally include 64-bit versions.
 *
 * =====================================================================================
 */

uint bitTranspose<let nDims : uint, let nBits : uint>(uint inCoords)
{
    uint nDims1 = nDims - 1;
    uint inB = nBits;
    uint utB;
    uint inFieldEnds = 1;
    // computes a bit mask with lower 'inB' bits set.
    uint inMask = ((uint(2) << (inB - 1)) - 1);
    uint coords = 0;
    for (uint utB = inB >> 1; utB != 0; utB = inB >> 1) {
        uint shiftAmt = nDims1 * utB;
        uint utFieldEnds = inFieldEnds | (inFieldEnds << (shiftAmt+utB));
        uint utMask = (utFieldEnds << utB) - utFieldEnds;
        uint utCoords = 0;
        uint d;
        if ((inB & 1) != 0)
        {
            uint inFieldStarts = inFieldEnds << (inB-1);
            uint oddShift = 2 * shiftAmt;
            for (d = 0; d < nDims; ++d)
            {
                uint temp = inCoords & inMask;
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
                uint temp = inCoords & inMask;
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

uint64_t bitTranspose_64<let nDims : uint, let nBits : uint>(uint64_t inCoords)
{
    uint nDims1 = nDims - 1;
    uint inB = nBits;
    uint utB;
    uint64_t inFieldEnds = 1;
    // computes a bit mask with lower 'inB' bits set.
    uint64_t inMask = ((uint64_t(2) << (inB - 1)) - 1);
    uint64_t coords = 0;
    for (uint utB = inB >> 1; utB != 0; utB = inB >> 1) {
        uint shiftAmt = nDims1 * utB;
        uint64_t utFieldEnds = inFieldEnds | (inFieldEnds << (shiftAmt + utB));
        uint64_t utMask = (utFieldEnds << utB) - utFieldEnds;
        uint64_t utCoords = 0;
        uint d;
        if ((inB & 1) != 0)
        {
            uint64_t inFieldStarts = inFieldEnds << (inB - 1);
            uint oddShift = 2 * shiftAmt;
            for (d = 0; d < nDims; ++d)
            {
                uint64_t temp = inCoords & inMask;
                inCoords >>= inB;
                coords |= (temp & inFieldStarts) << oddShift++;
                temp &= ~inFieldStarts;
                temp = (temp | (temp << shiftAmt)) & utMask;
                utCoords |= temp << (d * utB);
            }
        }
        else
        {
            for (d = 0; d < nDims; ++d)
            {
                uint64_t temp = inCoords & inMask;
                inCoords >>= inB;
                temp = (temp | (temp << shiftAmt)) & utMask;
                utCoords |= temp << (d * utB);
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

inline uint hilbert_c2i<let nDims : uint, let nBits : uint>(uint coord[nDims])
{  
  uint nDimsBits = nDims*nBits;
  uint index;
  uint d;
  uint coords = 0;
  for (d = nDims; bool(d--); )
  {
    coords <<= nBits;
    coords |= coord[d];
  }
  if (nBits > 1)
  {
    uint ndOnes = ((uint(2) << (nDims - 1)) - 1);
    uint nd1Ones= ndOnes >> 1; /* for adjust_rotation */
    uint b = nDimsBits;
    uint rotation = 0;
    uint flipBit = 0;
    uint nthbits = ((uint(2) << (nDimsBits - 1)) - 1) / ndOnes;
    coords = bitTranspose<nDims, nBits>(coords);
    coords ^= coords >> nDims;
    index = 0;
    do {
        uint bits = uint((coords >> (b-=nDims)) & ndOnes);
        uint arg = bits ^ flipBit;
        uint nRots = rotation;
        bits = uint(((((arg) >> (nRots)) | ((arg) << ((nDims) - (nRots)))) & ((uint(2) << (nDims - 1)) - 1)));
        index <<= nDims;
        index |= bits;
        flipBit = (uint)1 << rotation;
        // adjust rotation
        bits &= -bits & nd1Ones;
        while (bits != 0) {
            bits >>= 1;
            rotation++;
        }
        if (++rotation >= nDims) {
            rotation -= nDims;
        }
    } while (b != 0);
    index ^= nthbits >> 1;
  }
  else
    index = coords;
  for (d = 1; d < nDimsBits; d *= 2)
    index ^= index >> d;
  return index;
}

inline uint64_t hilbert_c2i_64<let nDims : uint, let nBits : uint>(uint64_t coord[nDims])
{
    uint nDimsBits = nDims * nBits;
    uint64_t index;
    uint d;
    uint64_t coords = 0;
    for (d = nDims; bool(d--);)
    {
        coords <<= nBits;
        coords |= coord[d];
    }
    if (nBits > 1)
    {
        uint ndOnes = ((uint(2) << (nDims - 1)) - 1);
        uint nd1Ones = ndOnes >> 1; /* for adjust_rotation */
        uint b = nDimsBits;
        uint rotation = 0;
        uint flipBit = 0;
        uint64_t nthbits = ((uint64_t(2) << (nDimsBits - 1)) - 1) / ndOnes;
        coords = bitTranspose_64<nDims, nBits>(coords);
        coords ^= coords >> nDims;
        index = 0;
        do {
            uint bits = uint((coords >> (b -= nDims)) & ndOnes);
            uint arg = bits ^ flipBit;
            uint nRots = rotation;
            bits = uint(((((arg) >> (nRots)) | ((arg) << ((nDims) - (nRots)))) & ((uint64_t(2) << (nDims - 1)) - 1)));
            index <<= nDims;
            index |= bits;
            flipBit = (uint)1 << rotation;
            // adjust rotation
            bits &= -bits & nd1Ones;
            while (bits != 0) {
                bits >>= 1;
                rotation++;
            }
            if (++rotation >= nDims) {
                rotation -= nDims;
            }
        } while (b != 0);
        index ^= nthbits >> 1;
    }
    else
        index = coords;
    for (d = 1; d < nDimsBits; d *= 2)
        index ^= index >> d;
    return index;
}

inline uint[nDims] hilbert_i2c<let nDims : uint, let nBits : uint>(uint index)
{
    uint coord[nDims];
    uint coords;
    uint nbOnes = ((uint(2) << (nBits - 1)) - 1);
    uint d;

    if (nBits > 1) {
        uint nDimsBits = nDims*nBits;
        uint ndOnes = ((uint(2) << (nDims - 1)) - 1);
        uint nd1Ones= ndOnes >> 1; /* for adjust_rotation */
        uint b = nDimsBits;
        uint rotation = 0;
        uint flipBit = 0;
        uint nthbits = ((uint(2) << (nDimsBits - 1)) - 1) / ndOnes;
        index ^= (index ^ nthbits) >> 1;
        coords = 0;
        do
        {
            uint bits = (uint)((index >> (b-=nDims)) & ndOnes);
            coords <<= nDims;
            uint arg = bits;
            uint nRots = rotation;
            coords |= uint((((arg) << (nRots)) | ((arg) >> ((nDims) - (nRots)))) & ((uint(2) << (nDims - 1)) - 1)) ^ flipBit;
            flipBit = (uint)1 << rotation;
            bits &= -bits & nd1Ones;
            while (bits != 0) {
                bits >>= 1;
                rotation++;
            }
            if (++rotation >= nDims) {
                rotation -= nDims;
            }
        } while (b != 0);
        for (b = nDims; b < nDimsBits; b *= 2)
            coords ^= coords >> b;
        coords = bitTranspose<nBits, nDims>(coords);
    }
    else
        coords = index ^ (index >> 1);
    for (d = 0; d < nDims; ++d)
    {
        coord[d] = coords & nbOnes;
        coords >>= nBits;
    }

    return coord;
}

inline uint64_t[nDims] hilbert_i2c_64<let nDims : uint, let nBits : uint>(uint64_t index)
{
    uint64_t coord[nDims];
    uint64_t coords;
    uint nbOnes = ((uint(2) << (nBits - 1)) - 1);
    uint d;

    if (nBits > 1) {
        uint nDimsBits = nDims * nBits;
        uint ndOnes = ((uint(2) << (nDims - 1)) - 1);
        uint nd1Ones = ndOnes >> 1; /* for adjust_rotation */
        uint b = nDimsBits;
        uint rotation = 0;
        uint flipBit = 0;
        uint64_t nthbits = ((uint64_t(2) << (nDimsBits - 1)) - 1) / ndOnes;
        index ^= (index ^ nthbits) >> 1;
        coords = 0;
        do
        {
            uint bits = (uint)((index >> (b -= nDims)) & ndOnes);
            coords <<= nDims;
            uint arg = bits;
            uint nRots = rotation;
            coords |= uint((((arg) << (nRots)) | ((arg) >> ((nDims) - (nRots)))) & ((uint64_t(2) << (nDims - 1)) - 1)) ^ flipBit;
            flipBit = (uint)1 << rotation;
            bits &= -bits & nd1Ones;
            while (bits != 0) {
                bits >>= 1;
                rotation++;
            }
            if (++rotation >= nDims) {
                rotation -= nDims;
            }
        } while (b != 0);
        for (b = nDims; b < nDimsBits; b *= 2)
            coords ^= coords >> b;
        coords = bitTranspose_64<nBits, nDims>(coords);
    }
    else
        coords = index ^ (index >> 1);
    for (d = 0; d < nDims; ++d)
    {
        coord[d] = coords & nbOnes;
        coords >>= nBits;
    }

    return coord;
}

inline uint hilbert_encode(float3 xyz)
{
    xyz *= (float)(1ul << 10);
    return hilbert_c2i<3, 10>( { uint(xyz.x), uint(xyz.y), uint(xyz.z)});
}

inline float3 hilbert_decode(uint code)
{
    let coord = hilbert_i2c<3, 10>(code);
    float3 c;
    c.x = float(coord[0]) / (float) (1ul << 10);
    c.y = float(coord[1]) / (float) (1ul << 10);
    c.z = float(coord[2]) / (float) (1ul << 10);
    return c;
}

inline uint64_t hilbert64_encode(float3 xyz)
{
    xyz *= (float)(1ull << 20);
    return hilbert_c2i_64<3, 20>( { uint64_t(xyz.x), uint64_t(xyz.y), uint64_t(xyz.z)});
}

inline float3 hilbert64_decode(uint64_t code)
{
  let coord = hilbert_i2c_64<3, 20>(code);
  float3 c;
  c.x = float(coord[0]) / (float) (1ull << 20);
  c.y = float(coord[1]) / (float) (1ull << 20);
  c.z = float(coord[2]) / (float) (1ull << 20);
  return c;
}

uint separate_bits(uint n)
{
    n &=                  0b00000000000000000000001111111111u;
    n = (n ^ (n << 16)) & 0b11111111000000000000000011111111u;
    n = (n ^ (n <<  8)) & 0b00000011000000001111000000001111u;
    n = (n ^ (n <<  4)) & 0b00000011000011000011000011000011u;
    n = (n ^ (n <<  2)) & 0b00001001001001001001001001001001u;
    return n;
};

uint compact_bits(uint n)
{
  n &=                  0b00001001001001001001001001001001u;
  n = (n ^ (n >>  2)) & 0b00000011000011000011000011000011u;
  n = (n ^ (n >>  4)) & 0b00000011000000001111000000001111u;
  n = (n ^ (n >>  8)) & 0b11111111000000000000000011111111u;
  n = (n ^ (n >> 16)) & 0b00000000000000000000001111111111u;
  return n;
};

uint64_t separate_bits_64(uint64_t n)
{
    n &=                  0b0000000000000000000000000000000000000000001111111111111111111111ull;
    n = (n ^ (n << 32)) & 0b1111111111111111000000000000000000000000000000001111111111111111ull;
    n = (n ^ (n << 16)) & 0b0000000011111111000000000000000011111111000000000000000011111111ull;
    n = (n ^ (n <<  8)) & 0b1111000000001111000000001111000000001111000000001111000000001111ull;
    n = (n ^ (n <<  4)) & 0b0011000011000011000011000011000011000011000011000011000011000011ull;
    n = (n ^ (n <<  2)) & 0b1001001001001001001001001001001001001001001001001001001001001001ull;
    return n;
};

uint64_t compact_bits_64(uint64_t n)
{
  n &=                  0b1001001001001001001001001001001001001001001001001001001001001001ull;
  n = (n ^ (n >>  2)) & 0b0011000011000011000011000011000011000011000011000011000011000011ull;
  n = (n ^ (n >>  4)) & 0b1111000000001111000000001111000000001111000000001111000000001111ull;
  n = (n ^ (n >>  8)) & 0b0000000011111111000000000000000011111111000000000000000011111111ull;
  n = (n ^ (n >> 16)) & 0b1111111111111111000000000000000000000000000000001111111111111111ull;
  n = (n ^ (n >> 32)) & 0b0000000000000000000000000000000000000000001111111111111111111111ull;
  return n;
};

inline uint morton_encode(float3 xyz)
{
    uint3 ixyz = uint3(xyz * (float)(1ul << 10));
    return separate_bits(ixyz.x) | (separate_bits(ixyz.y) << 1) | (separate_bits(ixyz.z) << 2);
}

inline float3 morton_decode(uint index)
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

inline uint64_t morton64_encode(float3 xyz)
{
    uint64_t3 ixyz = uint64_t3(xyz * (float)(1ull << 20));
    return separate_bits_64(ixyz.x) | (separate_bits_64(ixyz.y) << 1) | (separate_bits_64(ixyz.z) << 2);
}

inline float3 morton64_decode(uint64_t index)
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
