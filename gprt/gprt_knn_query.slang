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

#ifdef __SLANG_COMPILER__

#include "gprt_knn_shared.h"

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