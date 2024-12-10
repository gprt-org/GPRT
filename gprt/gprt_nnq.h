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


// Old, getting rid of eventually
#define MAX_LEVELS 10

#include "gprt_shared.h"
#include "gprt_hploc_shared.h"

// Nearest neighbor query description
struct NNQDesc {
    float3 Origin;
    float TMax;
};

namespace gprt {
  struct NNAccel {
    uint32_t TraverseBVH8Callable;
    uint32_t TraverseBVH2Callable;
    uint32_t TraverseLinearCallable;
    uint32_t padding;
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

struct TraversePayload {
  // Used to include / reject geometry instances based on the instance mask in each instance
  //   if(!((InstanceInclusionMask & InstanceMask) & 0xff)) { ignore intersection }
  uint InstanceInclusionMask : 8;
  
  // Offset to add into addressing calculations within shader tables for hit group indexing.
  uint QueryContributionToHitGroupIndex : 4;
  
  // Stride to multiply by GeometryConributionToShaderIndex (which is just the 0 based index the 
  // geometry was supplied by the app into its bottom-level acceleration structure.)
  // (Todo, document SBT indexing for NN traversals )
  uint MultiplierForGeometryContributionToShaderIndex : 4;
  
  // Miss shader index in addressing calculations within shader tables.
  // (Reduced to 8 bits from DXR's 16)
  uint MissShaderIndex : 6;

  // A valid combination of NN flags. 
  uint QueryFlags : 10;
  
  // The query description
  float4 OriginAndTMax;

  // Above is 6 registers, so we have 26 registers remaining for user data.

  // just debugging with these for now...
  uint hitNodes;
  uint hitPrims;
  float time;

  // User payload data
  uint UserPayload[2]; 
};


// inline
float4 OpConvertUToF(uint32_t4 u) {
    return spirv_asm {
        result : $$float4 = OpConvertUToF $u;
    };
}

float3 OpConvertUToF(uint32_t3 u) {
    return spirv_asm {
        result : $$float3 = OpConvertUToF $u;
    };
}

[ForceInline]
uint16_t bfe16(uint value, int offset, int bits)
{
    // __target_switch
    // {
    // case glsl: __intrinsic_asm "bitfieldExtract";
    // case spirv: 
    return uint16_t(spirv_asm {
        result:$$uint = OpBitFieldUExtract $value $offset $bits
    });
    // default:
    //     return (value >> offset) & ((1u << bits) - 1);
    // }
}

[ForceInline]
uint bfe(uint value, int offset, int bits)
{
    // __target_switch
    // {
    // case glsl: __intrinsic_asm "bitfieldExtract";
    // case spirv: 
    return spirv_asm {
        result:$$uint = OpBitFieldUExtract $value $offset $bits
    };
    // default:
    //     return (value >> offset) & ((1u << bits) - 1);
    // }
}

// [ForceInline]
// inline uint bfe(uint value, int offset, int bits)
// {
//     return spirv_asm {
//         result:$$uint = OpBitFieldUExtract $value $offset $bits
//     };
// }

[ForceInline]
inline uint3 bfe(uint3 value, int offset, int bits)
{
  // __target_switch
  //   {
  //   case glsl: __intrinsic_asm "bitfieldExtract";
    // case spirv: 
    return spirv_asm {
        result:$$uint3 = OpBitFieldUExtract $value $offset $bits
    };
  //   default:
        // return uint3(bfe(value.x, offset, bits), bfe(value.y, offset, bits), bfe(value.z, offset, bits));
    // }

    // return spirv_asm {
    //     result:$$uint3 = OpBitFieldUExtract $value $offset $bits
    // };
}

[ForceInline]
inline uint bfi(uint base, uint insert, int offset, int bits)
{
    return spirv_asm {
        result:$$uint = OpBitFieldInsert $base $insert $offset $bits
    };
}

// 2 registers
// struct StackEntry {
//     // Indicates whether the group is a primitive group or a node group.
//     uint32_t isPrimGroup : 1;

//     // A base index common to all items in the queue.
//     uint32_t baseIdx : 31;
    
//     // A queue of the relative indices offset from the base index.
//     uint32_t relIdxQueue : 24; // 3 bits per slot, 8 slots

//     // A mask indicating which items of the queue still need processing
//     uint32_t hits: 8; // 1 bit per slot, 8 slots

//     __init() {
//         isPrimGroup = 0;
//         baseIdx = 0;
//         relIdxQueue = 0;
//         hits = 0;
//     }
// };



// 2 registers
struct StackEntry {
    uint32_t R0;
    uint32_t R1;

    // Indicates whether the group is a primitive group or a node group.
    bool isPrimGroup() {return bool(bfe(R0, 31, 1));}

    [mutating]
    void setPrimGroup(bool primGroup) {R0 = bfi(R0, int(primGroup), 31, 1);}

    // A base index common to all items in the queue.
    uint32_t getBaseIndex() {return bfe(R0, 0, 31);}

    [mutating]
    void setBaseIndex(uint32_t index) {R0 = bfi(R0, index, 0, 31);}
    
    // A queue of the relative indices offset from the base index.
    uint32_t getRelIdxQueue() {return bfe(R1, 8, 24);}

    [mutating]
    void setRelIdxQueue(uint32_t relIdxQueue) {R1 = bfi(R1, relIdxQueue, 8, 24);}

    // A mask indicating which items of the queue still need processing
    uint32_t getHits() {return bfe(R1, 0, 8);}

    [mutating]
    void setHits(uint32_t hits) {R1 = bfi(R1, hits, 0, 8); } 

    __init() {
        R0 = R1 = 0;
    }
};

#define LOCAL_STACK_SIZE 8

struct TraversalState {
  uint doneTraversing;
  uint scheduleReorder;
  uint64_t BVH8L_ptr;

  StackEntry localStack[LOCAL_STACK_SIZE]; // 2*LOCAL_STACK_SIZE (ie, 16) registers
  uint sp; // stack pointer     (1 register)
  StackEntry nodeGroup;     //  (2 registers)
  StackEntry primGroup;     //  (2 registers)


  // uint32_t *BVH8N;
  // float4 *BVH8L;

  // note, repurposing tmax for search radius, and ignoring tmin.
  float3 origin; 
  float tmax;
  uint32_t itmax;
  // float3 direction; float tmax;

  float2 triUV;

  //int enablePostponing;

  // debug data...
  uint debug;
  uint debugHits;
  // float debugTime;

  __init() {
  }
};


// from real time collision detection
float findClosestPointTriangleSquared(float3 pa, float3 pb, float3 pc, float3 x, out float3 p, out float2 t)
{
    // source: real time collision detection
    // check if x in vertex region outside pa
    float3 ab = pb - pa;
    float3 ac = pc - pa;
    float3 ax = x - pa;
    float d1 = dot(ab, ax);
    float d2 = dot(ac, ax);
    if (d1 <= 0.0 && d2 <= 0.0)
    {
        // barycentric coordinates (1, 0, 0)
        t = float2(1.0, 0.0);
        p = pa;
        return dot(x - p, x - p);
    }

    // check if x in vertex region outside pb
    float3 bx = x - pb;
    float d3 = dot(ab, bx);
    float d4 = dot(ac, bx);
    if (d3 >= 0.0 && d4 <= d3)
    {
        // barycentric coordinates (0, 1, 0)
        t = float2(0.0, 1.0);
        p = pb;
        return dot(x - p, x - p);
    }

    // check if x in vertex region outside pc
    float3 cx = x - pc;
    float d5 = dot(ab, cx);
    float d6 = dot(ac, cx);
    if (d6 >= 0.0 && d5 <= d6)
    {
        // barycentric coordinates (0, 0, 1)
        t = float2(0.0, 0.0);
        p = pc;
        return dot(x - p, x - p);
    }

    // check if x in edge region of ab, if so return projection of x onto ab
    float vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0)
    {
        // barycentric coordinates (1 - v, v, 0)
        float v = d1 / (d1 - d3);
        t = float2(1.0 - v, v);
        p = pa + ab * v;
        return dot(x - p, x - p);
    }

    // check if x in edge region of ac, if so return projection of x onto ac
    float vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0)
    {
        // barycentric coordinates (1 - w, 0, w)
        float w = d2 / (d2 - d6);
        t = float2(1.0 - w, 0.0);
        p = pa + ac * w;
        return dot(x - p, x - p);
    }

    // check if x in edge region of bc, if so return projection of x onto bc
    float va = d3 * d6 - d5 * d4;
    if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0)
    {
        // barycentric coordinates (0, 1 - w, w)
        float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        t = float2(0.0, 1.0 - w);
        p = pb + (pc - pb) * w;
        return dot(x - p, x - p);
    }

    // x inside face region. Compute p through its barycentric coordinates (u, v, w)
    float denom = 1.0 / (va + vb + vc);
    float v = vb * denom;
    float w = vc * denom;
    t = float2(1.0 - v - w, v);
    p = pa + ab * v + ac * w; //= u*a + v*b + w*c, u = va*denom = 1.0f - v - w
    return dot(x - p, x - p);
}

  // float3 Origin;
  // float closestDistance;
  // int closestPrimitive;
  // int parentIndex;
  // int debug;
  // int level;

  

  // float3 queryOrigin, 
  // float tMax, 

  // out int closestPrimitive, 
  // out float closestDistance, 

static uint NN_FLAG_NONE = 0;
static uint NN_FLAG_DEBUG = 1;


struct NNQDebugData {
  uint hitNodes;
  uint hitPrims;
  float reportedDistance;
};

// Note, sizeof(payload) must be less than 26 registers.
NNQDebugData TraceNNQ<T>(
  gprt::NNAccel accel, 
  uint QueryFlags,
  uint InstanceInclusionMask,
  uint QueryContributionToHitGroupIndex,
  uint MultiplierForGeometryContributionToShaderIndex,
  uint MissShaderIndex,
  NNQDesc Query,
  inout T Payload,
  float debugTime = 0.0f)
{  
  TraversePayload payload;
  payload.InstanceInclusionMask = InstanceInclusionMask;
  payload.QueryContributionToHitGroupIndex = QueryContributionToHitGroupIndex;
  payload.MultiplierForGeometryContributionToShaderIndex = MultiplierForGeometryContributionToShaderIndex;
  payload.MissShaderIndex = MissShaderIndex;
  payload.QueryFlags = QueryFlags;
  payload.OriginAndTMax = float4(Query.Origin, Query.TMax);
  payload.UserPayload = reinterpret<uint[2], T>(Payload);


  

  payload.time = debugTime;
  
  TraversalState tstate;
  tstate.origin = payload.OriginAndTMax.xyz;
  // tstate.direction = float3(1.0, 1.0, 1.0);
  tstate.tmax = payload.OriginAndTMax.w;
  tstate.itmax = asuint(tstate.tmax);
  tstate.debug = bool(payload.QueryFlags & NN_FLAG_DEBUG);
  tstate.debugHits = 0;

  tstate.doneTraversing = false;
  tstate.scheduleReorder = false;
  
  // Put root on stack
  tstate.nodeGroup = StackEntry();
  tstate.nodeGroup.setHits(0b00000001);
  tstate.debugHits = 0;
  tstate.sp = 0;
  for (int i = 0; i < LOCAL_STACK_SIZE; ++i) {
    tstate.localStack[i] = StackEntry();
  }


  while (!bool(tstate.doneTraversing))
  { 
    CallShader(accel.TraverseBVH8Callable, tstate);
    tstate.debugHits++;

    
    // if (tstate.debugHits > 10) break;
  }

  

  payload.hitPrims = tstate.debugHits;
  payload.OriginAndTMax.w = sqrt(tstate.tmax);

  Payload = reinterpret<T>(payload.UserPayload);

  NNQDebugData debugData;
  debugData.hitNodes = payload.hitNodes;
  debugData.hitPrims = payload.hitPrims;
  debugData.reportedDistance = payload.OriginAndTMax.w;
  return debugData;
  // return (payload.OriginAndTMax.w);
}

#endif




      // gprt::Buffer BVH8N; // BVH8 nodes (ceil((N x 2 - 1) / 8) x 80 bytes)
      // gprt::Buffer BVH8L; // BVH8 leaves

      // // older traversal code below...


      // // input
      // uint32_t numPrims;
      // uint32_t numLevels;
      // uint32_t numClusters[MAX_LEVELS];
      // float maxSearchRange;

      // gprt::Buffer points;

      // // Buffer of uint8 counting the number of times a point is referenced.
      // // A count of 1 means the point is "open".
      // gprt::Buffer pointCounts;  

      // // Why are we separating these again?... 
      // // They could just be "primitives"...
      // gprt::Buffer edges; 
      // gprt::Buffer triangles;

      // // A sorted list of triangle indices, meant to be traversed
      // // linearly from the leaves
      // gprt::Buffer triangleLists;

      // // Hilbert codes of quantized primitive centroids
      // // One uint64_t per primitive
      // gprt::Buffer codes;
      // gprt::Buffer ids;

      // // Buffer containing the global AABB. Pair of two floats
      // gprt::Buffer aabb;        

      // // Buffers of bounding primitives. 
      // // If axis aligned bounding boxes, each is a pair of float3.
      // // If bounding balls, each is a single float4 (xyzr).
      // // If oriented bounding boxes, each is a triplet of float3s.
      // //   - note, also reused for temporarily storing covariance matrices
      // gprt::Buffer aabbs[MAX_LEVELS];
      // gprt::Buffer oobbs[MAX_LEVELS];
      // gprt::Buffer centers[MAX_LEVELS];

      // // 3 floats for treelet aabb min, 
      // // 3 bytes for scale exponent, one unused 
      // // byte   15   14    13    12    11 10 9 8   7  6  5  4  3  2  1  0
      // //       [??]  [sz]  [sy]  [sx]  [  zmin  ]  [  ymin  ]  [  xmin  ]
      // // gprt::Buffer treelets;

      // // If an "axis aligned bounding box", 64-bit integers, 6 bytes for bounding box, 2 unused.
      // // byte   8    7   6     5     4     3     2     1           
      // //       [?]  [?]  [zh]  [yh]  [xh]  [zl]  [yl]  [xl]        

      // // If a "bounding ball", 32-bit integers, 3 bytes for center, 1 for radius.
      // // byte   4    3    2    1           
      // //       [r]  [z]  [y]  [x]        

      // // If an "oriented bounding box", we use a 128-bit integer. 
      // // 6 bytes for bounding box, 2 unused. 
      // // Then 20 bits for Euler rotations in X, Y, then Z. 4 bits unused.
      // //        15 14 13 12 11 10 9 8   7   6   5     4     3    2    1    0
      // //       [ zr ]  [ yr ]  [ xr ]   [?] [?] [zh]  [yh]  [xh]  [zl]  [yl]  [xl]
      // // gprt::Buffer children; 

      // // An RT core tree
      // gprt::Accel accel;

      // // An LBVH tree
      // // gprt::Buffer lbvhMortonCodes;

      // // Primitive IDs that correspond to sorted morton codes. 
      // // One uint32_t per primitive
      // // gprt::Buffer lbvhIds;

      // // numPrims-1 + numPrims long. 
      // // The "numPrims-1" section contains inner nodes
      // // The "numPrims" section contains leaves
      // // Each node is an int4. 
      // // "X" is left, "Y" is right, "Z" is parent, and "W" is leaf or -1 if internal node.
      // // gprt::Buffer lbvhNodes;

      // // numPrims-1 + numPrims long. Each aabb is a pair of float3.
      // // gprt::Buffer lbvhAabbs;


// struct NNStats {
//     float primsHit;
//     float lHit[MAX_LEVELS];

//     float getTotalTouched() {
//         float totalTouched = primsHit;
//         for (int i = 0; i < MAX_LEVELS; ++i) {
//             totalTouched += lHit[i];
//         }
//         return totalTouched;
//     }

//     static NNStats Create() {
//         NNStats stats;
//         stats.primsHit = 0;
//         for (int i = 0; i < MAX_LEVELS; ++i) {
//             stats.lHit[i] = 0;
//         }
//         return stats;
//     };
// };