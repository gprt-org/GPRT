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

// Assuming at most 128 geometries for now, but this can be increased if needed.
// (Might be able to use the full 255... just need to test it out...)
#ifndef GEOM_ID_BVH2 
#define GEOM_ID_BVH2 255
#endif

#ifndef INVALID_ID
#define INVALID_ID UINT32_MAX
#endif

#if defined(__SLANG_COMPILER__)
#define __inline__ [ForceInline]
#define __mutating__ [mutating]

__inline__ int __popc(uint x) {
    return countbits(x);
}

__inline__ int __bfind(uint x) {
    return firstbithigh(x);
}

__inline__ float fstrict(float v) // prevents the compiler from optimizing FP32 math on host side
{
    return asfloat(asint(v));
}

__inline__ float __fsub_rd(float a, float b) // returns a-b rounded down
{
    float x = fstrict(fstrict(a) - fstrict(b));
    if (fstrict(x + fstrict(b)) > fstrict(a) || fstrict(fstrict(a) - x) < fstrict(b))
        if (x > -FLT_MAX && (~asint(x) & 0x7F800000u) != 0u)
            x = (x == 0.0f) ? -FLT_MIN : asfloat(asint(x) + ((x > 0.0f) ? -1 : +1));
    return x;
}

__inline__ float __fsub_ru(float a, float b) // returns a-b rounded up
{
    float x = fstrict(fstrict(a) - fstrict(b));
    if (fstrict(x + fstrict(b)) < fstrict(a) || fstrict(fstrict(a) - x) > fstrict(b))
        if (x < +FLT_MAX && (~asint(x) & 0x7F800000u) != 0u)
            x = (x == 0.0f) ? +FLT_MIN : asfloat(asint(x) + ((x > 0.0f) ? +1 : -1));
    return x;
}

__inline__ float __fmaf_rn(float a, float b, float c)    { return float( double(a) * double(b) + double(c) ); }
__inline__ float __fmul_rn(float a, float b)             { return a * b; }
__inline__ float __fadd_rn(float a, float b)             { return a + b; }

__inline__ float __fmul_rd(float x, float y) {
    float result = x * y;
    // Check if the result needs to be adjusted to round down
    if (result > x * y) {
        result = nextafter(result, -FLT_MAX);
    }
    return result;
}

__inline__ float __fmul_ru(float x, float y) {
    float result = x * y;
    // Check if the result needs to be adjusted to round up
    if (result < x * y) {
        result = nextafter(result, FLT_MAX);
    }
    return result;
}

__inline__ float fmaxof3f(float3 a) {
    return max(max(a.x, a.y), a.z);
}

__inline__ int   fastSelect      (int a, int b, int c)       { return (c >= 0) ? a : b; }
__inline__ float fastSelect      (float a, float b, int c)   { return (c >= 0) ? a : b; }

// Extracts the 8 exponent bits from the floating point number that conservatively bound it, avoiding infinity.
__inline__ uint floatToExponent(float num) {
    uint bits = asuint(num);
    uint exponentBits = (bits >> 23) & 0xFF; // Mask to get only the exponent bits
    // Avoiding 255, since that's infinity.
    return uint(max(min(exponentBits + 1, 254u), 2u));
}

// Convert an exponent back to a floating-point scale factor
__inline__ float exponentToFloat(uint exponent) {
    uint bits = exponent << 23;
    return asfloat(bits);
}

__inline__ float3 exponentToFloat(uint3 exponent) {
    uint3 bits = exponent << 23;
    return asfloat(bits);
}



// // Returns an 8-bit mask field to indicate which of the children are internal nodes.
// uint getIMask(uint assignedChildren[8], uint numLeaves) {
//     uint iMask = 0;
//     for (int i = 0; i < 8; ++i) {
//         if (assignedChildren[i] != INVALID_ID) {
//             uint relativeIndex = assignedChildren[i];
//             if (relativeIndex >= numLeaves) {
//                 iMask |= (1 << i);
//             }
//         }
//     }
//     return iMask;
// }

__inline__ uint shf_l_clamp(uint a, uint b, int c) {
    uint32_t  n = min(c, 32);    
    // shift concatenation of [b, a]
    // extract 32 msbs
    uint32_t  d = (b << n)      | (a >> (32-n));
    return d;
}

__inline__ uint8_t extract_byte32(uint i, uint n) { 
    return uint8_t(i >> (n * 8)); 
}

__inline__ uint8_t extract_byte64(uint64_t i, uint n) { 
    return uint8_t(i >> (n * 8)); 
}

// now some functions to set bytes
__inline__ uint32_t set_byte32(uint32_t i, uint32_t n, uint8_t byte) {
    return (i & ~(0xFF << (n * 8))) | (uint32_t(byte) << (n * 8));
}

__inline__ uint64_t set_byte64(uint64_t bits, uint byteIndex, uint8_t byte) {
    return (bits & ~(0xFFull << (byteIndex * 8))) | (uint64_t(byte) << (byteIndex * 8));
}

__inline__ uint  duplicateByte(uint value) { 
    uint byte = (value & 0xFF);
    uint dup = (byte << 24) | (byte << 16) | (byte << 8) | byte;
    return dup; 
}

// could probably optimize a bit if needed...
__inline__ uint8_t getOctant(float3 dir) {

    uint oct = shf_l_clamp( asuint(dir.z), 0, 1 );
    oct      = shf_l_clamp( asuint(dir.y), oct, 1 );
    oct      = shf_l_clamp( asuint(dir.x), oct, 1 );
    return uint8_t(oct);
}

__inline__ float3 getOctantDir(uint octant) {
    return float3(
                    (((octant >> 2) & 1) == 1) ? -1.0f : 1.0f,
			        (((octant >> 1) & 1) == 1) ? -1.0f : 1.0f,
			        (((octant >> 0) & 1) == 1) ? -1.0f : 1.0f);
}

__inline__ uint32_t sign_extend_s8x4(uint32_t word) {
    uint32_t result = (uint32_t(uint32_t(uint8_t(word >>  7) * 0xFF)) << 0)
                    | (uint32_t(uint32_t(uint8_t(word >> 15) * 0xFF)) << 8)
                    | (uint32_t(uint32_t(uint8_t(word >> 23) * 0xFF)) << 16)
                    | (uint32_t(uint32_t(uint8_t(word >> 31) * 0xFF)) << 24);
    return result;
}

__inline__ uint64_t sign_extend_s8x8(uint64_t word) {
    // Combine the sign-extended bytes
    uint64_t result = (uint64_t(uint64_t(uint8_t(word >>  7ull) * 0xFF)) << 0ull)
                    | (uint64_t(uint64_t(uint8_t(word >> 15ull) * 0xFF)) << 8ull)
                    | (uint64_t(uint64_t(uint8_t(word >> 23ull) * 0xFF)) << 16ull)
                    | (uint64_t(uint64_t(uint8_t(word >> 31ull) * 0xFF)) << 24ull)
                    | (uint64_t(uint64_t(uint8_t(word >> 39ull) * 0xFF)) << 32ull)
                    | (uint64_t(uint64_t(uint8_t(word >> 47ull) * 0xFF)) << 40ull)
                    | (uint64_t(uint64_t(uint8_t(word >> 55ull) * 0xFF)) << 48ull)
                    | (uint64_t(uint64_t(uint8_t(word >> 63ull) * 0xFF)) << 56ull);
    return result;
}

__inline__ uint64_t replicate_sign_bits64(uint64_t word) {
    // Step-by-step spread the sign bit to all other bits in each byte
    uint64_t spread_signs = (word & 0x8080808080808080ull) >> 7;
    spread_signs |= spread_signs << 1;
    spread_signs |= spread_signs << 2;
    spread_signs |= spread_signs << 4;
    return spread_signs;
}

__inline__ uint32_t vsignExtend4(uint32_t word) {
    // Step-by-step spread the sign bit to all other bits in each byte
    uint32_t spread_signs = (word & 0x80808080u) >> 7;
    spread_signs |= spread_signs << 1;
    spread_signs |= spread_signs << 2;
    spread_signs |= spread_signs << 4;
    return spread_signs;
}

__inline__ uint8_t4 extract_bytes32(uint i) { 
    uint8_t4 val = uint8_t4(
        uint8_t(i >> (0 * 8)),
        uint8_t(i >> (1 * 8)),
        uint8_t(i >> (2 * 8)),
        uint8_t(i >> (3 * 8))
    );
    return val; 
}

__inline__ uint bfe(uint val, int pos, int len) {
    uint mask = (1u << len) - 1u; // create a mask with `len` bits set to 1
    return (val >> pos) & mask; // Shift the value right by 'pos' and apply the mask
}

// Extract byte value, shift it left with another extracted byte and add the result to u32.
uint vshl_wrap_add_b0_b0(uint val, uint shift, uint addend)
{
    // Extract the second byte from each input
    uint byteVal = (val >> 0) & 0xFF;   // Shift right 0 bits and mask to get the second byte
    uint byteShift = (shift >> 0) & 0xFF;
    uint byteAddend = (addend >> 0) & 0xFF;

    // Shift the extracted byte from 'val' left by the amount specified in the extracted byte from 'shift'
    uint shifted = byteVal << byteShift;

    // Add the result to the extracted byte from 'addend'
    uint resultByte = (shifted + byteAddend) & 0xFF; // Ensure the result wraps at 0 bits

    // Combine the result back into the original 'val', only altering the second byte
    uint result = (val & 0xFFFFFF00) | (resultByte << 0); // Clear the second byte and set the new value

    return result;
}
uint vshl_wrap_add_b1_b1(uint val, uint shift, uint addend)
{
    // Extract the second byte from each input
    uint byteVal = (val >> 8) & 0xFF;   // Shift right 8 bits and mask to get the second byte
    uint byteShift = (shift >> 8) & 0xFF;
    uint byteAddend = (addend >> 8) & 0xFF;

    // Shift the extracted byte from 'val' left by the amount specified in the extracted byte from 'shift'
    uint shifted = byteVal << byteShift;

    // Add the result to the extracted byte from 'addend'
    uint resultByte = (shifted + byteAddend) & 0xFF; // Ensure the result wraps at 8 bits

    // Combine the result back into the original 'val', only altering the second byte
    uint result = (val & 0xFFFF00FF) | (resultByte << 8); // Clear the second byte and set the new value

    return result;
}
uint vshl_wrap_add_b2_b2(uint val, uint shift, uint addend)
{
    // Extract the second byte from each input
    uint byteVal = (val >> 16) & 0xFF;   // Shift right 16 bits and mask to get the second byte
    uint byteShift = (shift >> 16) & 0xFF;
    uint byteAddend = (addend >> 16) & 0xFF;

    // Shift the extracted byte from 'val' left by the amount specified in the extracted byte from 'shift'
    uint shifted = byteVal << byteShift;

    // Add the result to the extracted byte from 'addend'
    uint resultByte = (shifted + byteAddend) & 0xFF; // Ensure the result wraps at 16 bits

    // Combine the result back into the original 'val', only altering the second byte
    uint result = (val & 0xFF00FFFF) | (resultByte << 16); // Clear the second byte and set the new value

    return result;
}
uint vshl_wrap_add_b3_b3(uint val, uint shift, uint addend)
{
    // Extract the second byte from each input
    uint byteVal = (val >> 24) & 0xFF;   // Shift right 24 bits and mask to get the second byte
    uint byteShift = (shift >> 24) & 0xFF;
    uint byteAddend = (addend >> 24) & 0xFF;

    // Shift the extracted byte from 'val' left by the amount specified in the extracted byte from 'shift'
    uint shifted = byteVal << byteShift;

    // Add the result to the extracted byte from 'addend'
    uint resultByte = (shifted + byteAddend) & 0xFF; // Ensure the result wraps at 24 bits

    // Combine the result back into the original 'val', only altering the second byte
    uint result = (val & 0x00FFFFFF) | (resultByte << 24); // Clear the second byte and set the new value

    return result;
}

#else

#define __inline__ inline
#define __mutating__

__inline__ int __popc(unsigned int v) // population count
{
    // Adapted from:
    // http://stackoverflow.com/questions/109023/how-to-count-the-number-of-set-bits-in-a-32-bit-integer
    v -= (v >> 1) & 0x55555555u;
    v = (v & 0x33333333u) + ((v >> 2) & 0x33333333u);
    return (((v + (v >> 4)) & 0x0F0F0F0Fu) * 0x01010101u) >> 24;
}

__inline__ uint8_t extract_byte32(uint i, uint n) { 
    return uint8_t(i >> (n * 8)); 
}

__inline__ uint8_t extract_byte64(uint64_t i, uint n) { 
    return uint8_t(i >> (n * 8)); 
}

// now some functions to set bytes
__inline__ uint32_t set_byte32(uint32_t i, uint32_t n, uint8_t byte) {
    return (i & ~(0xFF << (n * 8))) | (uint32_t(byte) << (n * 8));
}

__inline__ uint64_t set_byte64(uint64_t bits, uint byteIndex, uint8_t byte) {
    return (bits & ~(0xFFull << (byteIndex * 8))) | (uint64_t(byte) << (byteIndex * 8));
}
#endif


struct HPLOCParams {
    int N;              // Number of primitives across all referenced geometries
    gprt::Buffer BVH2;  // BVH2 nodes, excluding leaves ((N - 1) x 32 bytes)
    gprt::Buffer BVH8;  // BVH8 nodes (ceil((N x 2 - 1) / 8) x 80 bytes)
    gprt::Buffer BVH8L; // BVH8 leaves (N x 64 bytes)
    gprt::Buffer AC;    // Some atomic counters for allocation and work scheduling (5 x 4 bytes)
    gprt::Buffer I;     // Cluster indices reordered by space filling curve codes (N x 4 bytes)
    gprt::Buffer C;     // Space filling curve codes, sorted in ascending order (N x 8 bytes)
    gprt::Buffer pID;   // BVH2 parent IDs. initialized to -1. (N x 4 bytes)

    gprt::Buffer indexPairs; // BVH2 -> BVH8 pairs (N x 8bytes)
    
    // For geometry
    gprt::Buffer rootBounds; // Two float3's storing aabb of trimesh
    gprt::Buffer primPrefix; // An exclusive prefix sum of the prim counts in each geometry
    gprt::Buffer triangles; // Triangle index buffers, one handle per geometry
    gprt::Buffer vertices;  // Triangle vertex buffers, one handle per geometry
};

// A 32 byte BVH2 node structure
struct BVH2Node {
    // [32b xlo] [32b ylo] [32b zlo] [1b - isInner] [7b - L Geom ID] [24b - L Prim ID]
    float4 aabbMinAndL;
    // [32b xhi] [32b yhi] [32b zhi] [1b - isInner] [7b - R Geom ID] [24b - R Prim ID]
    float4 aabbMaxAndR;

    #if defined(__SLANG_COMPILER__)
    __init(int2 leftCluster, int2 rightCluster, float2x3 new_bounds) {
        aabbMinAndL.xyz = new_bounds[0];
        aabbMaxAndR.xyz = new_bounds[1];
        uint32_t leftIndex  = (leftCluster.x  & 0x00FFFFFF) | ((leftCluster.y  & 0x000000FF) << 24);
        uint32_t rightIndex = (rightCluster.x & 0x00FFFFFF) | ((rightCluster.y & 0x000000FF) << 24);
        aabbMinAndL.w = asfloat(leftIndex);
        aabbMaxAndR.w = asfloat(rightIndex);
    }
    #endif

    float2x3 getBounds() {
        #if defined(__SLANG_COMPILER__)
        return float2x3(aabbMinAndL.xyz, aabbMaxAndR.xyz);
        #else
        float2x3 bounds;
        bounds[0] = aabbMinAndL.xyz();
        bounds[1] = aabbMaxAndR.xyz();
        return bounds;
        #endif
    }

    bool isChildLeaf(int childIndex) {
        #if defined(__SLANG_COMPILER__)
        uint32_t index = (childIndex == 0) ? asuint(aabbMinAndL.w) : asuint(aabbMaxAndR.w);
        #else
        uint32_t index = (childIndex == 0) ? *(uint32_t *)&aabbMinAndL.w : *(uint32_t*)&aabbMaxAndR.w;
        #endif
        uint32_t top8Bits = (index >> 24);
        if (top8Bits == 255) return false;
        return true;
    }

    uint32_t getChildPrimID(int childIndex) {
        #if defined(__SLANG_COMPILER__)
        uint32_t index = (childIndex == 0) ? asuint(aabbMinAndL.w) : asuint(aabbMaxAndR.w);
        #else
        uint32_t index = (childIndex == 0) ? *(uint32_t*)&aabbMinAndL.w : *(uint32_t*)&aabbMaxAndR.w;
        #endif
        return index & 0x00FFFFFF;
    }

    uint32_t getChildGeomID(int childIndex) {
        #if defined(__SLANG_COMPILER__)
        uint32_t index = (childIndex == 0) ? asuint(aabbMinAndL.w) : asuint(aabbMaxAndR.w);
        #else
        uint32_t index = (childIndex == 0) ? *(uint32_t*)&aabbMinAndL.w : *(uint32_t*)&aabbMaxAndR.w;
        #endif
        return (index >> 24) & 0x000000FF;
    }

    uint32_t getChildCluster(int childIndex) {
        #if defined(__SLANG_COMPILER__)
        uint32_t index = (childIndex == 0) ? asuint(aabbMinAndL.w) : asuint(aabbMaxAndR.w);
        #else
        uint32_t index = (childIndex == 0) ? *(uint32_t*)&aabbMinAndL.w : *(uint32_t*)&aabbMaxAndR.w;
        #endif
        return index;
    }
};

// The header for a BVH8 node from Ylitie et al. 2017.
// Each of the rows below can be read using an aligned 128-byte load.
struct BVH8NodeHeader {
    // Row 1, 32 bytes
    float3 pos; uint32_t scaleAndIMask;

    // Row 2, 32 bytes
    uint32_t firstNodeIdx; uint32_t firstPrimIdx;

    // Row 3, 32 bytes
    uint2 meta;

    __inline__ __mutating__ void setIMaskBit(uint32_t childSlot, bool isInner) {
        uint8_t mask = uint8_t(extract_byte32(scaleAndIMask, 3));
        mask = uint8_t((mask & ~(1 << childSlot)) | (uint8_t(isInner) << childSlot));
        scaleAndIMask = set_byte32(scaleAndIMask, 3, mask);
    }

    __inline__ __mutating__ void setScale(uint8_t scaleX, uint8_t scaleY, uint8_t scaleZ) {
        scaleAndIMask = set_byte32(scaleAndIMask, 0, scaleX);
        scaleAndIMask = set_byte32(scaleAndIMask, 1, scaleY);
        scaleAndIMask = set_byte32(scaleAndIMask, 2, scaleZ);
    }

    __inline__ uint8_t getScaleX() { return extract_byte32(scaleAndIMask, 0); }
    __inline__ uint8_t getScaleY() { return extract_byte32(scaleAndIMask, 1); }
    __inline__ uint8_t getScaleZ() { return extract_byte32(scaleAndIMask, 2); }
    __inline__ uint3 getScale() { return uint3(getScaleX(), getScaleY(), getScaleZ()); }

    __inline__ uint8_t getIMask() { return extract_byte32(scaleAndIMask, 3); }

    __inline__ __mutating__ void setInner(uint32_t childSlot) { 
        if (childSlot < 4) meta[0] = set_byte32(meta[0], childSlot, uint8_t(childSlot + 0b00111000));
        else meta[1] = set_byte32(meta[1], childSlot - 4, uint8_t(childSlot + 0b00111000));
    }
    __inline__ bool isInner(uint32_t childSlot) { 
        uint8_t value = uint8_t((childSlot < 4) ? extract_byte32(meta[0], childSlot) : extract_byte32(meta[1], childSlot - 4));
        return (value >= 0b00111000 && value < 0b01000000);
    }
    
    __inline__ __mutating__ void setLeaf(uint32_t childSlot, int primOfs, int numPrims) { 
        uint8_t value = (uint8_t)(primOfs + (0b11100000011000000010000000000000 >> (numPrims << 3))); 
        if (childSlot < 4) meta[0] = set_byte32(meta[0], childSlot, value);
        else meta[1] = set_byte32(meta[1], childSlot - 4, value);
    }
    __inline__ bool isLeaf(uint32_t childSlot) { 
        uint8_t value = uint8_t((childSlot < 4) ? extract_byte32(meta[0], childSlot) : extract_byte32(meta[1], childSlot - 4));
        return (value != 0b00000000 && (value < 0b00111000 || value >= 0b01000000)); 
    }
    
    __inline__ __mutating__ void setEmpty(uint32_t childSlot) { 
        if (childSlot < 4) meta[0] = set_byte32(meta[0], childSlot, 0b00000000);
        else meta[1] = set_byte32(meta[1], childSlot - 4, 0b00000000);
    }
    __inline__ bool isEmpty(uint32_t childSlot) { 
        uint8_t value = uint8_t((childSlot < 4) ? extract_byte32(meta[0], childSlot) : extract_byte32(meta[1], childSlot - 4));
        return (value == 0b00000000); 
    }

    __inline__ int getInnerChildSlot(uint32_t childSlot) { 
        uint8_t value = uint8_t((childSlot < 4) ? extract_byte32(meta[0], childSlot) : extract_byte32(meta[1], childSlot - 4));
        return value - 0b00111000; 
    }
    __inline__ int getLeafRemapOfs(uint32_t childSlot) { 
        uint8_t value = uint8_t((childSlot < 4) ? extract_byte32(meta[0], childSlot) : extract_byte32(meta[1], childSlot - 4));
        return value & 0b00011111; 
    }
    __inline__ int getLeafNumPrims(uint32_t childSlot) {
        uint8_t value = uint8_t((childSlot < 4) ? extract_byte32(meta[0], childSlot) : extract_byte32(meta[1], childSlot - 4));
        return __popc(value >> 5); 
    }
};

// An 80 byte BVH8 node structure, combining the header and the quantized child AABBs
struct BVH8Node {
    BVH8NodeHeader header;
    /*n2.xy*/uint2 lox;
    /*n2.zw*/uint2 loy;
    /*n3.xy*/uint2 loz;
    /*n3.zw*/uint2 hix;
    /*n4.xy*/uint2 hiy;
    /*n4.zw*/uint2 hiz;

    __inline__ uint8_t getLoX(uint32_t childSlot) { return extract_byte32((childSlot < 4) ? lox[0] : lox[1], childSlot % 4); }
    __inline__ uint8_t getLoY(uint32_t childSlot) { return extract_byte32((childSlot < 4) ? loy[0] : loy[1], childSlot % 4); }
    __inline__ uint8_t getLoZ(uint32_t childSlot) { return extract_byte32((childSlot < 4) ? loz[0] : loz[1], childSlot % 4); }
    __inline__ uint8_t getHiX(uint32_t childSlot) { return extract_byte32((childSlot < 4) ? hix[0] : hix[1], childSlot % 4); }
    __inline__ uint8_t getHiY(uint32_t childSlot) { return extract_byte32((childSlot < 4) ? hiy[0] : hiy[1], childSlot % 4); }
    __inline__ uint8_t getHiZ(uint32_t childSlot) { return extract_byte32((childSlot < 4) ? hiz[0] : hiz[1], childSlot % 4); }

    __inline__ __mutating__ void setLoX(uint32_t childSlot, uint8_t value) {
        if (childSlot < 4) lox[0] = set_byte32(lox[0], childSlot, value); 
        else               lox[1] = set_byte32(lox[1], childSlot - 4, value);
    }
    __inline__ __mutating__ void setLoY(uint32_t childSlot, uint8_t value) {
        if (childSlot < 4) loy[0] = set_byte32(loy[0], childSlot, value); 
        else               loy[1] = set_byte32(loy[1], childSlot - 4, value);
    }
    __inline__ __mutating__ void setLoZ(uint32_t childSlot, uint8_t value) {
        if (childSlot < 4) loz[0] = set_byte32(loz[0], childSlot, value); 
        else               loz[1] = set_byte32(loz[1], childSlot - 4, value);
    }
    __inline__ __mutating__ void setHiX(uint32_t childSlot, uint8_t value) {
        if (childSlot < 4) hix[0] = set_byte32(hix[0], childSlot, value); 
        else               hix[1] = set_byte32(hix[1], childSlot - 4, value);
    }
    __inline__ __mutating__ void setHiY(uint32_t childSlot, uint8_t value) {
        if (childSlot < 4) hiy[0] = set_byte32(hiy[0], childSlot, value); 
        else               hiy[1] = set_byte32(hiy[1], childSlot - 4, value);
    }
    __inline__ __mutating__ void setHiZ(uint32_t childSlot, uint8_t value) {
        if (childSlot < 4) hiz[0] = set_byte32(hiz[0], childSlot, value); 
        else               hiz[1] = set_byte32(hiz[1], childSlot - 4, value);
    }



    // uint8_t        lox[8];
    // uint8_t        loy[8];
    // uint8_t        loz[8];
    // uint8_t        hix[8];
    // uint8_t        hiy[8];
    // uint8_t        hiz[8]; 


};

struct BVH8Triangle {
    float3 v0;
    float3 v1;
    float3 v2;

    // original index of the triangle given by the user
    int userTriangleID; 
    uint32_t primBits;
    uint32_t debug;
};

struct TriangleRecord {
    gprt::Buffer BVH8L;
    gprt::Buffer triangles; // Triangle index buffers, one handle per geometry
    gprt::Buffer vertices;  // Triangle vertex buffers, one handle per geometry
};

struct TraverseRecord {
    uint N; // number of primitives
    gprt::Buffer BVH2; // BVH2 nodes (N-1 x 64 bytes)
    gprt::Buffer BVH8N; // BVH8 nodes (ceil((N x 2 - 1) / 8) x 80 bytes)
    gprt::Buffer BVH8L; // BVH8 leaves

    // One buffer per geometry
    gprt::Buffer primBuffers;
    gprt::Buffer vertBuffers;

    // An offset for the given instance to the SBT of callables
    // for nearest neighbor programs
    uint32_t InstanceContributionToHitGroupIndex;
};