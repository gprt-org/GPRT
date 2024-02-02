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

// The indexes of the traverse callables
#define TRAVERSE_CALLABLE 0

// The higher this number is, the more primitives we can store in our tree.
#define MAX_LEVELS 10

// The higher this number is, the more clusters we're going to touch
// relative to the number of primitives.
#define BRANCHING_FACTOR 4

struct NodeRecord {
  // Axis aligned bounding box of the node
  gprt::Buffer aabbs[MAX_LEVELS];
  
  // Oriented bounding box of the node
  gprt::Buffer oobbs[MAX_LEVELS];

  // Center points used for upper-bound culling
  gprt::Buffer centers[MAX_LEVELS];

  // The number of clusters in a level
  uint32_t numClusters[MAX_LEVELS];

  // total number of levels in the tree
  uint32_t numLevels;
  uint32_t numPrims;
  gprt::Buffer triangleLists;
};

// #define COLLECT_STATS

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

      // Buffer of uint8 counting the number of times a point is referenced.
      // A count of 1 means the point is "open".
      gprt::Buffer pointCounts;  

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
    gprt::Buffer vertCounts;
    gprt::Buffer trianglesExp;
#ifndef __SLANG_COMPILER__
    ExpandTrianglesParams(int numPrims, gprt::Buffer primIDs, gprt::Buffer triangles, gprt::Buffer vertices, gprt::Buffer vertCounts, gprt::Buffer trianglesExp) :
      numPrims(numPrims), primIDs(primIDs), triangles(triangles), vertices(vertices), vertCounts(vertCounts), trianglesExp(trianglesExp) {}
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

// void TraverseLeaf(in gprt::NNAccel record, uint32_t leafID, bool useAABBs, bool useOOBBs, uint distanceLevel, float3 queryOrigin, inout NNPayload payload, bool debug) {
//   int numPrims = record.numPrims;

//   // At this point, it appears that the fastest thing to do is just 
//   // let traversal proceed linearly at the leaves to drive upward culling.
//   for (uint32_t primID = 0; primID < BRANCHING_FACTOR; ++primID) {
//     uint32_t itemID = leafID * BRANCHING_FACTOR + primID;
//     if (itemID >= numPrims) continue;

//     // minDist = min(minDist, l0MinDist);

//     #ifdef COLLECT_STATS
//     payload.primsHit++; // Count this as traversing a primitive
//     #endif
//     float3 p = queryOrigin;

//     float3 a = gprt::load<float4>(record.triangleLists, itemID * 3 + 0).xyz;
//     float3 b = gprt::load<float4>(record.triangleLists, itemID * 3 + 1).xyz;

//     float3 ba = b - a; float3 pa = p - a;
//     float distToEdgeA = sqrt(_dot2(ba*clamp(dot(ba,pa)/_dot2(ba),0.0f,1.0f)-pa));
//     // if (distToEdgeA > payload.closestDistance) continue;

//     float3 c = gprt::load<float4>(record.triangleLists, itemID * 3 + 2).xyz;
//     float3 cb = c - b; float3 pb = p - b;

//     float distToEdgeB = sqrt(_dot2(cb*clamp(dot(cb,pb)/_dot2(cb),0.0f,1.0f)-pb));
//     // if (distToEdgeB > payload.closestDistance) continue;

//     float3 ac = a - c; float3 pc = p - c;
//     float distToEdgeC = sqrt(_dot2(ac*clamp(dot(ac,pc)/_dot2(ac),0.0f,1.0f)-pc));
//     // if (distToEdgeC > payload.closestDistance) continue;

//     float3 nor = cross( ba, ac );
//     float distToPlane = sqrt(dot(nor,pa)*dot(nor,pa)/_dot2(nor));
//     // if (distToPlane > payload.closestDistance) continue;

//     float distToEdges = min(min(distToEdgeA, distToEdgeB), distToEdgeC);

//     // inside/outside test 
//     bool inside = (sign(dot(cross(ba,nor),pa)) +
//                   sign(dot(cross(cb,nor),pb)) +
//                   sign(dot(cross(ac,nor),pc))<2.0f);

//     float dist = inside ? distToEdges : distToPlane;
    
//     // Primitive farther than furthest
//     if (dist > payload.closestDistance) continue;

//     // Newly found closest primitive
//     payload.closestDistance = dist;
//     payload.closestPrimitive = itemID;
//   }
// }



// // This method unfortunately seems pretty slow. 
// // Our hypothesis is that the "trail" here causes an indirect register read from the stack, which is slow.

// // N is the max number of levels of the tree
// // B is the branching factor
// void TraverseTreeFullStack<let N : int, let B : int>(gprt::Buffer aabbs[N], gprt::Buffer oobbs[N], uint32_t numClusters[N], 
//   in gprt::NNAccel record, uint distanceLevel, bool useAABBs, bool useOOBBs, float3 queryOrigin, float tmax, inout NNPayload payload, bool debug) {
//   payload.closestDistance = tmax;

//   float minDist = 0.f;

//   float closestDistAvg = 0.f;
//   float closestDistM2 = 0.f; // m2 aggregates the squared distance from the mean
//   float closestDistVar = 0.f;
//   int totalHits = 0;

//   Stack<B> stack[N];

//   int trail[N];
//   for (int i = 0; i < N; ++i) trail[i] = 0;  

//   // Traverse over all levels, starting from the top and going down
//   int level = N - 1;
//   int parentIndex = 0;
//   while (level < N) {
//     // If we're traversing down the tree...
//     if (trail[level] == 0) {
//       // Intersect and sort the children at this level
//       stack[level] = intersectAndSortChildren<B>(queryOrigin, payload.closestDistance, parentIndex * B, numClusters[level], useAABBs, aabbs[level], useOOBBs, oobbs[level], debug);
//     }

//     // Traverse all children from near to far
//     [unroll]
//     for (int i = 0; i < B; ++i) {
//       // If recursing up, skip forward to the child we were last on 
//       if (i < trail[level]) continue; 
      
//       // Fetch child index and distance from the sorted stack
//       int childIndex = stack[level].key(trail[level]);
//       float minDist = stack[level].value(trail[level]);
      
//       // Pop when all children from here on are too far
//       if (childIndex == -1 || minDist > payload.closestDistance) {
//         trail[level] = B;
//         break;
//       }

//       // if at this point we're at the bottom of the tree, traverse the leaf
//       if (level == 0) {
//         TraverseLeaf(record, childIndex, useAABBs, useOOBBs, distanceLevel, queryOrigin, payload, debug);
//         // Advance the trail now that we've processed this child.
//         trail[level]++;
//       } else {
//         // recurse down the tree
//         parentIndex = childIndex;
//         break;
//       }
//     }

//     if (trail[level] == BRANCHING_FACTOR) {
//       // Reset the trail when we're done with this level.
//       trail[level] = 0;
//       // Then, go back up to the parent level.
//       parentIndex /= BRANCHING_FACTOR;
//       level++;
//       // advance to the next node
//       trail[level]++;
//     } else {
//       // recurse down      
//       level--;
//     }
//   }

//   if (payload.closestPrimitive != -1) {
//     payload.closestPrimitive = uint32_t(gprt::load<uint64_t>(record.ids, payload.closestPrimitive)); 
//   }
// }

// void TraverseTree<let N : int, let B : int>(
//   in gprt::NNAccel record,
//   int parentIndex,
//   bool useAABBs, bool useOOBBs, int distanceLevel,
//   float3 queryOrigin, inout NNPayload payload, bool debug) 
// {
//   int start = parentIndex * B;
//   Stack<B> stack = intersectAndSortChildren<B>(queryOrigin, payload.closestDistance,
//                                                start, record.numClusters[N-1],
//                                                useAABBs, record.aabbs[N-1],
//                                                useOOBBs, record.oobbs[N-1]);
//   for (int i = 0; i < B; ++i) {
//     int currentIndex = stack.key(i);
//     float mindist = stack.value(i);
//     if (currentIndex == -1 || mindist > payload.closestDistance) return;
//     if (N > 1) TraverseTree<N-1, B>(record, currentIndex, useAABBs, useOOBBs, distanceLevel, queryOrigin, payload, debug);
//     else TraverseLeaf(record, currentIndex, useAABBs, useOOBBs, distanceLevel, queryOrigin, payload, debug);
//   }
// }  

// void TraverseTreeRecursive(in gprt::NNAccel record, uint distanceLevel, bool useAABBs, bool useOOBBs, float3 queryOrigin, float tmax, inout NNPayload payload, bool debug) {
//   switch (record.numLevels)
//   {
//     case  1 :  TraverseTree<1, BRANCHING_FACTOR>(record, 0, useAABBs, useOOBBs, distanceLevel, queryOrigin, payload, debug); break;
//     case  2 :  TraverseTree<2, BRANCHING_FACTOR>(record, 0, useAABBs, useOOBBs, distanceLevel, queryOrigin, payload, debug); break;
//     case  3 :  TraverseTree<3, BRANCHING_FACTOR>(record, 0, useAABBs, useOOBBs, distanceLevel, queryOrigin, payload, debug); break;
//     case  4 :  TraverseTree<4, BRANCHING_FACTOR>(record, 0, useAABBs, useOOBBs, distanceLevel, queryOrigin, payload, debug); break;
//     case  5 :  TraverseTree<5, BRANCHING_FACTOR>(record, 0, useAABBs, useOOBBs, distanceLevel, queryOrigin, payload, debug); break;
//     case  6 :  TraverseTree<6, BRANCHING_FACTOR>(record, 0, useAABBs, useOOBBs, distanceLevel, queryOrigin, payload, debug); break;
//     case  7 :  TraverseTree<7, BRANCHING_FACTOR>(record, 0, useAABBs, useOOBBs, distanceLevel, queryOrigin, payload, debug); break;
//     case  8 :  TraverseTree<8, BRANCHING_FACTOR>(record, 0, useAABBs, useOOBBs, distanceLevel, queryOrigin, payload, debug); break;
//     case  9 :  TraverseTree<9, BRANCHING_FACTOR>(record, 0, useAABBs, useOOBBs, distanceLevel, queryOrigin, payload, debug); break;
//     case 10 : TraverseTree<10, BRANCHING_FACTOR>(record, 0, useAABBs, useOOBBs, distanceLevel, queryOrigin, payload, debug); break;
//     default: break;
//   }  
//   if (payload.closestPrimitive != -1) {
//     payload.closestPrimitive = uint32_t(gprt::load<uint64_t>(record.ids, payload.closestPrimitive)); 
//   }
// }

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

struct TraversePayload {
  float3 Origin;
  float closestDistance;
  int closestPrimitive;
  int parentIndex;
  int debug;
  int level;
};

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
    TraversePayload traversePayload;
    traversePayload.Origin = queryOrigin;
    traversePayload.level = knnAccel.numLevels - 1;
    traversePayload.debug = debug;
    traversePayload.parentIndex = 0;
    traversePayload.closestDistance = tMax;
    traversePayload.closestPrimitive = -1;

    CallShader(TRAVERSE_CALLABLE, traversePayload);

    RaytracingAccelerationStructure nullAccel;
    RayDesc query;
    query.Origin = queryOrigin;

    payload.closestDistance = traversePayload.closestDistance;
    payload.closestPrimitive = traversePayload.closestPrimitive;
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

// in gprt::NNAccel record,
// int parentIndex,
// bool useAABBs, bool useOOBBs, int distanceLevel,
// float3 queryOrigin, inout NNPayload payload, bool debug)
// <let N : int, let B : int>(NodeRecord record, inout TraversePayload payload, int index, RayDesc query)



// void TraverseTree<let N : int, let B : int>(NodeRecord record, inout TraversePayload payload, int parentIndex, RayDesc query)
// {
//     int start = parentIndex * B;
//     Stack<B> stack = intersectAndSortChildren<B>(query.Origin, payload.closestDistance,
//                                                  start, record.numClusters[N - 1],
//                                                  true/*useAABBs*/, record.aabbs[N - 1],
//                                                  true/*useOOBBs*/, record.oobbs[N - 1]);
//     for (int i = 0; i < B; ++i) {
//         int childIndex = stack.key(i);
//         float mindist = stack.value(i);
//         if (childIndex == -1 || mindist > payload.closestDistance) return;
//         if (N > 1) TraverseTree<N - 1, B>(record, payload, childIndex, query);
//         else TraverseLeaf<B>(record, payload, childIndex, query);
//     }
// }

// [shader("miss")]
// void TraverseNode1(uniform NodeRecord record, inout TraversePayload payload) { TraverseTree<1, BRANCHING_FACTOR>(record, payload, 0, PollRayDesc());}

// [shader("miss")]
// void TraverseNode2(uniform NodeRecord record, inout TraversePayload payload) { TraverseTree<2, BRANCHING_FACTOR>(record, payload, 0, PollRayDesc());}

// [shader("miss")]
// void TraverseNode3(uniform NodeRecord record, inout TraversePayload payload) { TraverseTree<3, BRANCHING_FACTOR>(record, payload, 0, PollRayDesc());}

// [shader("miss")]
// void TraverseNode4(uniform NodeRecord record, inout TraversePayload payload) { TraverseTree<4, BRANCHING_FACTOR>(record, payload, 0, PollRayDesc());}

// [shader("miss")]
// void TraverseNode5(uniform NodeRecord record, inout TraversePayload payload) { TraverseTree<5, BRANCHING_FACTOR>(record, payload, 0, PollRayDesc());}

// [shader("miss")]
// void TraverseNode6(uniform NodeRecord record, inout TraversePayload payload) { TraverseTree<6, BRANCHING_FACTOR>(record, payload, 0, PollRayDesc());}

// [shader("miss")]
// void TraverseNode7(uniform NodeRecord record, inout TraversePayload payload) { TraverseTree<7, BRANCHING_FACTOR>(record, payload, 0, PollRayDesc());}

// [shader("miss")]
// void TraverseNode8(uniform NodeRecord record, inout TraversePayload payload) { TraverseTree<8, BRANCHING_FACTOR>(record, payload, 0, PollRayDesc());}

// [shader("miss")]
// void TraverseNode9(uniform NodeRecord record, inout TraversePayload payload) { TraverseTree<9, BRANCHING_FACTOR>(record, payload, 0, PollRayDesc());}

// [shader("miss")]
// void TraverseNode10(uniform NodeRecord record, inout TraversePayload payload) {TraverseTree<10, BRANCHING_FACTOR>(record, payload, 0, PollRayDesc());}

// For good performance, it's critical that the payload here is as small as possible.
// Therefore, we put the relevant accel data into the record, and only keep the intra-level
// data in the payload.
