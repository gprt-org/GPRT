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

#include "gprt_shared.h"

#define COLLECT_STATS

// The higher this number is, the more primitives we can store in our tree.
#define MAX_LEVELS 10

// The higher this number is, the more clusters we're going to touch
// relative to the number of primitives. 
#define BRANCHING_FACTOR 4

// With a branching factor of 4 and 12 levels (10 excluding the leaves),
// we can support up to 8 million primitives per nearest neighbor tree.

// Edit: nevermind... I had a bug with my previous minMaxDist function which was giving me some 
// incorrect intuition. I'm finding now that this is very helpful for the utah teapot.
// Edit edit: Downward pruning seems to fall apart when nodes are quantized. The reason 
// for why is because the nearest face might differ between the quantized and unquantized boxes.
// Nearest face only matches when quantized box is just a scaling up or down of the original. 
// It's very rare that this happens, but also, the performance improvements from this aren't very good.
//
// Edit edit edit: When using OBBs, it seems like the maximal distance to the box gives good performance 
// improvements.
// More updates, the "minMaxDistance" metric doesn't work for internal OBB nodes, since internal faces aren't guaranteed
// to contain primitives. Only OBB leaves can guarantee this. So, for internal nodes, we instead use the "maxDistance"
// for downward culling.
// More more updates... So long as internal OBBs are built to tightly fit the underlying primitives, then minMaxDist is
// "free game". However, it introduces some compute complexity which doesn't seem worth the effort
// #define ENABLE_DOWNAWARD_CULLING 

// Enables an LBVH reference, similar to Jakob and Guthe's knn.
// Note, we use this LBVH as a top level tree for our method, so 
// this is just about if we're using RT cores or not
#define ENABLE_LBVH_REFERENCE

// Uses a quantized representation for the active cluster queue
// I've bounced back and forth on if this helps or not. 
//   For untruncated queries, the ultimate bottleneck is when tree traversal fails and many prims 
//   are all approximately the same distance. There, its better to have an accurate queue for culling. 
//   However, I suspect the issue with culling has more to do with using AABBs over something tigher fitting
// #define ENABLE_QUEUE_QUANTIZATION


// Enables oriented bounding boxes in an attempt to improve culling performance. 
// Prior experiments seem to suggest that the minimum distance to an axis aligned bounding 
// box tends to be a poor fit for the minimum distance to the contained primitive. This
// issue seems to become more and more extreme as primitives are clustered together.
// This mode uses a singular value decomposition to compute the orientation of a bounding box 
// that best fits the underlying primitive distribution
// #define ENABLE_OBBS

// #define ENABLE_AABBS

// #define ENABLE_6DOPS

// Honestly I don't see much improvement by avoiding square roots. I don't think this is what the bottleneck is
// at least on my problems... Just going to take square roots for simplicity...
// #define USE_SQUARED_DISTS

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

namespace gprt {
  struct NNAccel {
      // input
      uint32_t numPrims;
      uint32_t numLevels;
      uint32_t numClusters[MAX_LEVELS];
      float maxSearchRange;

      gprt::Buffer points; 

      // Why are we separating these again?... 
      // They could just be "primitives"...
      gprt::Buffer edges; 
      gprt::Buffer triangles;

      // A sorted list of triangle indices, meant to be traversed
      // linearly from the leaves
      gprt::Buffer triangleLists;

      // Hilbert codes of quantized primitive centroids
      // One uint64_t per primitive
      gprt::Buffer codes;
      gprt::Buffer ids;

      // Buffer containing the global AABB. Pair of two floats
      gprt::Buffer aabb;        

      // Buffers of bounding primitives. 
      // If axis aligned bounding boxes, each is a pair of float3.
      // If bounding balls, each is a single float4 (xyzr).
      // If oriented bounding boxes, each is a triplet of float3s.
      //   - note, also reused for temporarily storing covariance matrices
      gprt::Buffer aabbs[MAX_LEVELS];
      gprt::Buffer oobbs[MAX_LEVELS];
      gprt::Buffer centers[MAX_LEVELS];

      // 3 floats for treelet aabb min, 
      // 3 bytes for scale exponent, one unused 
      // byte   15   14    13    12    11 10 9 8   7  6  5  4  3  2  1  0
      //       [??]  [sz]  [sy]  [sx]  [  zmin  ]  [  ymin  ]  [  xmin  ]
      // gprt::Buffer treelets;

      // If an "axis aligned bounding box", 64-bit integers, 6 bytes for bounding box, 2 unused.
      // byte   8    7   6     5     4     3     2     1           
      //       [?]  [?]  [zh]  [yh]  [xh]  [zl]  [yl]  [xl]        

      // If a "bounding ball", 32-bit integers, 3 bytes for center, 1 for radius.
      // byte   4    3    2    1           
      //       [r]  [z]  [y]  [x]        

      // If an "oriented bounding box", we use a 128-bit integer. 
      // 6 bytes for bounding box, 2 unused. 
      // Then 20 bits for Euler rotations in X, Y, then Z. 4 bits unused.
      //        15 14 13 12 11 10 9 8   7   6   5     4     3    2    1    0
      //       [ zr ]  [ yr ]  [ xr ]   [?] [?] [zh]  [yh]  [xh]  [zl]  [yl]  [xl]
      // gprt::Buffer children; 

      // An RT core tree
      gprt::Accel accel;

      // An LBVH tree
      // gprt::Buffer lbvhMortonCodes;

      // Primitive IDs that correspond to sorted morton codes. 
      // One uint32_t per primitive
      // gprt::Buffer lbvhIds;

      // numPrims-1 + numPrims long. 
      // The "numPrims-1" section contains inner nodes
      // The "numPrims" section contains leaves
      // Each node is an int4. 
      // "X" is left, "Y" is right, "Z" is parent, and "W" is leaf or -1 if internal node.
      // gprt::Buffer lbvhNodes;

      // numPrims-1 + numPrims long. Each aabb is a pair of float3.
      // gprt::Buffer lbvhAabbs;

  };
}

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

// Functions required for both building and querying
#ifdef __SLANG_COMPILER__

#include "gprt.slangh"

// Assuming matrix is normalized... Returns two euler results, so that we can pick the best one.
void mat3_to_eul2(in float3x3 mat, out float3 eul1, out float3 eul2)
{
  #define _EPSILON .00001f

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
#endif