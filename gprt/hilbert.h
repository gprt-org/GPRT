/* C header file for Hilbert curve functions */
#if !defined(_hilbert_h_)
#define _hilbert_h_

#include <stdint.h>

/* define the bitmask_t type as an integer of sufficient size */
typedef unsigned long long bitmask_t;
/* define the halfmask_t type as an integer of 1/2 the size of bitmask_t */
typedef unsigned long halfmask_t;

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

#define DLOGB_BIT_TRANSPOSE
inline bitmask_t
bitTranspose(unsigned nDims, unsigned nBits, bitmask_t inCoords)
#if defined(DLOGB_BIT_TRANSPOSE)
{
  unsigned const nDims1 = nDims-1;
  unsigned inB = nBits;
  unsigned utB;
  bitmask_t inFieldEnds = 1;
  bitmask_t inMask = ones(bitmask_t,inB);
  bitmask_t coords = 0;

  while ((utB = inB / 2))
    {
      unsigned const shiftAmt = nDims1 * utB;
      bitmask_t const utFieldEnds =
	inFieldEnds | (inFieldEnds << (shiftAmt+utB));
      bitmask_t const utMask =
	(utFieldEnds << utB) - utFieldEnds;
      bitmask_t utCoords = 0;
      unsigned d;
      if (inB & 1)
	{
	  bitmask_t const inFieldStarts = inFieldEnds << (inB-1);
	  unsigned oddShift = 2*shiftAmt;
	  for (d = 0; d < nDims; ++d)
	    {
	      bitmask_t in = inCoords & inMask;
	      inCoords >>= inB;
	      coords |= (in & inFieldStarts) <<	oddShift++;
	      in &= ~inFieldStarts;
	      in = (in | (in << shiftAmt)) & utMask;
	      utCoords |= in << (d*utB);
	    }
	}
      else
	{
	  for (d = 0; d < nDims; ++d)
	    {
	      bitmask_t in = inCoords & inMask;
	      inCoords >>= inB;
	      in = (in | (in << shiftAmt)) & utMask;
	      utCoords |= in << (d*utB);
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
#else
{
  bitmask_t coords = 0;
  unsigned d;
  for (d = 0; d < nDims; ++d)
    {
      unsigned b;
      bitmask_t in = inCoords & ones(bitmask_t,nBits);
      bitmask_t out = 0;
      inCoords >>= nBits;
      for (b = nBits; b--;)
	{
	  out <<= nDims;
	  out |= rdbit(in, b);
	}
      coords |= out << d;
    }
  return coords;
}
#endif

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

inline void hilbert_i2c(unsigned nDims, unsigned nBits, bitmask_t index, bitmask_t coord[])
{
  if (nDims > 1)
    {
      bitmask_t coords;
      halfmask_t const nbOnes = ones(halfmask_t,nBits);
      unsigned d;

      if (nBits > 1)
	{
	  unsigned const nDimsBits = nDims*nBits;
	  halfmask_t const ndOnes = ones(halfmask_t,nDims);
	  halfmask_t const nd1Ones= ndOnes >> 1; /* for adjust_rotation */
	  unsigned b = nDimsBits;
	  unsigned rotation = 0;
	  halfmask_t flipBit = 0;
	  bitmask_t const nthbits = ones(bitmask_t,nDimsBits) / ndOnes;
	  index ^= (index ^ nthbits) >> 1;
	  coords = 0;
	  do
	    {
	      int bits = (index >> (b-=nDims)) & ndOnes;
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
 *  nDims:      Number of coordinates.
 *  nBits:      Number of bits/coordinate.
 *  coord:      Array of n nBits-bit coordinates.
 * Outputs:
 *  index:      Output index value.  nDims*nBits bits.
 * Assumptions:
 *      nDims*nBits <= (sizeof bitmask_t) * (bits_per_byte)
 */
inline bitmask_t hilbert_c2i(unsigned nDims, unsigned nBits, bitmask_t const coord[])
{
  if (nDims > 1)
    {
      unsigned const nDimsBits = nDims*nBits;
      bitmask_t index;
      unsigned d;
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
	  unsigned b = nDimsBits;
	  unsigned rotation = 0;
	  halfmask_t flipBit = 0;
	  bitmask_t const nthbits = ones(bitmask_t,nDimsBits) / ndOnes;
	  coords = bitTranspose(nDims, nBits, coords);
	  coords ^= coords >> nDims;
	  index = 0;
	  do
	    {
	      int bits = (coords >> (b-=nDims)) & ndOnes;
	      bits = rotateRight(flipBit ^ bits, rotation, nDims);
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
    bitmask_t coord[3];
    coord[0] = bitmask_t(x);
    coord[1] = bitmask_t(y);
    coord[2] = bitmask_t(z);
    return uint32_t(hilbert_c2i(3, 10, coord));
}

#endif /* _hilbert_h_ */