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

// minDist computes the square of the distance from a point to a rectangle.
// If the point is contained in the rectangle then the distance is zero.
//
// Implemented per Definition 2 of "Nearest Neighbor Queries" by
// N. Roussopoulos, S. Kelley and F. Vincent, ACM SIGMOD, pages 71-79, 1995.
inline float getMinDist(float3 origin, float3 aabbMin, float3 aabbMax) {
  // For each dimension...
  // Determine which side that "p" is on.
  // Then, add the L2 distance to that side to our total
  float3 d = select((origin < aabbMin), origin - aabbMin, origin - aabbMax);
  return dot(d, d);
}

inline float2 getMinAndMinMaxDist(float3 p, float3 rp, float3 rq) {
  return float2(getMinDist(p, rp, rq), getMinMaxDist(p, rp, rq));
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

float getTriangleDist2( float3 p, float3 a, float3 b, float3 c )
{
  float3 ba = b - a; float3 pa = p - a;
  float3 cb = c - b; float3 pb = p - b;
  float3 ac = a - c; float3 pc = p - c;
  float3 nor = cross( ba, ac );

  return 
    (sign(dot(cross(ba,nor),pa)) +
     sign(dot(cross(cb,nor),pb)) +
     sign(dot(cross(ac,nor),pc))<2.0f)
     ?
     min( min(
     _dot2(ba*clamp(dot(ba,pa)/_dot2(ba),0.0f,1.0f)-pa),
     _dot2(cb*clamp(dot(cb,pb)/_dot2(cb),0.0f,1.0f)-pb) ),
     _dot2(ac*clamp(dot(ac,pc)/_dot2(ac),0.0f,1.0f)-pc) )
     :
     dot(nor,pa)*dot(nor,pa)/_dot2(nor);
}

#ifdef GPRT_DEVICE

// Payload for nearest neighbor queries
struct [raypayload] NNPayload {
  float closestDistance : read(anyhit, caller) : write(anyhit, caller);
  int closestPrimitive : read(anyhit, caller) : write(anyhit, caller);
  int4 stats : read(anyhit, caller) : write(anyhit, caller);
};

void TraceNN(float3 queryOrigin, in gprt::NNAccel knnAccel, out int closestPrimitive, out float closestDistance, inout int4 stats) {
  NNPayload payload;
  payload.closestPrimitive = -1;
  payload.closestDistance = 1e20f;
  payload.stats = stats;

  if (knnAccel.maxSearchRange > 0.f) {
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