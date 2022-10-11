// MIT License

// Copyright (c) 2022 Nathan V. Morrical

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "deviceCode.h"
#include "gprt.h"

struct Payload
{
[[vk::location(0)]] float3 color;
};

GPRT_RAYGEN_PROGRAM(AABBRayGen, (RayGenData, record))
{
  Payload payload;
  uint2 pixelID = DispatchRaysIndex().xy;
  float2 screen = (float2(pixelID) + 
                  float2(.5f, .5f)) / float2(record.fbSize);
  const int fbOfs = pixelID.x + record.fbSize.x * pixelID.y;

  RayDesc rayDesc;
  rayDesc.Origin = record.camera.pos;
  rayDesc.Direction = 
    normalize(record.camera.dir_00
    + screen.x * record.camera.dir_du
    + screen.y * record.camera.dir_dv
  );
  rayDesc.TMin = 0.0;
  rayDesc.TMax = 10000.0;
  RaytracingAccelerationStructure world = gprt::getAccelHandle(record.world);

  // store double precision ray
  gprt::store(record.dpRays, fbOfs * 2 + 0, double4(rayDesc.Origin.x, rayDesc.Origin.y, rayDesc.Origin.z, rayDesc.TMin));
  gprt::store(record.dpRays, fbOfs * 2 + 1, double4(rayDesc.Direction.x, rayDesc.Direction.y, rayDesc.Direction.z, rayDesc.TMax));

  TraceRay(
    world, // the tree
    RAY_FLAG_FORCE_OPAQUE, // ray flags
    0xff, // instance inclusion mask
    0, // ray type
    1, // number of ray types
    0, // miss type
    rayDesc, // the ray to trace
    payload // the payload IO
  );

  gprt::store(record.fbPtr, fbOfs, gprt::make_rgba(payload.color));
}

GPRT_MISS_PROGRAM(miss, (MissProgData, record), (Payload, payload))
{
  uint2 pixelID = DispatchRaysIndex().xy;  
  int pattern = (pixelID.x / 8) ^ (pixelID.y/8);
  payload.color = (pattern & 1) ? record.color1 : record.color0;
}

struct Attribute
{
  double2 bc;
};

#define EPSILON 2.2204460492503130808472633361816E-16
GPRT_COMPUTE_PROGRAM(DPTriangle, (DPTriangleData, record))
{
  int primID = DispatchThreadID.x;
  int3 indices = gprt::load<int3>(record.index, primID);
  double3 A = gprt::load<double3>(record.vertex, indices.x);
  double3 B = gprt::load<double3>(record.vertex, indices.y);
  double3 C = gprt::load<double3>(record.vertex, indices.z);
  double3 dpaabbmin = min(min(A, B), C);
  double3 dpaabbmax = max(max(A, B), C);
  float3 fpaabbmin = float3(dpaabbmin) - float3(EPSILON, EPSILON, EPSILON); // todo, round this below smallest float 
  float3 fpaabbmax = float3(dpaabbmax) + float3(EPSILON, EPSILON, EPSILON); // todo, round this below smallest float 
  gprt::store(record.aabbs, 2 * primID + 0, fpaabbmin);
  gprt::store(record.aabbs, 2 * primID + 1, fpaabbmax);
}

GPRT_CLOSEST_HIT_PROGRAM(DPTriangle, (DPTriangleData, record), (Payload, payload), (Attribute, attribute))
{
  double2 barycentrics = attribute.bc;
  payload.color = float3(barycentrics.x, barycentrics.y, 0.0);
}

double3 dcross (in double3 a, in double3 b) { return double3(a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x); }

float next_after(float a) {
  uint a_ = asuint(a);
  a_ += a < 0 ? -1 : 1;
  return asfloat(a);
}

GPRT_INTERSECTION_PROGRAM(DPTriangle, (DPTriangleData, record))
{
  uint2 pixelID = DispatchRaysIndex().xy;
  const int fbOfs = pixelID.x + record.fbSize.x * pixelID.y;
  
  double4 raydata1 = gprt::load<double4>(record.dpRays, fbOfs * 2 + 0);
  double4 raydata2 = gprt::load<double4>(record.dpRays, fbOfs * 2 + 1);

  double3 ro = double3(raydata1.x, raydata1.y, raydata1.z);//ObjectRayOrigin();
  double3 rd = double3(raydata2.x, raydata2.y, raydata2.z);//ObjectRayDirection();
  double tMin = raydata1.w;
  double tCurrent = raydata2.w;

  int primID = PrimitiveIndex();
  int3 indices = gprt::load<int3>(record.index, primID);
  double3 v0 = gprt::load<double3>(record.vertex, indices.x);
  double3 v1 = gprt::load<double3>(record.vertex, indices.y);
  double3 v2 = gprt::load<double3>(record.vertex, indices.z);

  double3 v1v0 = v1 - v0;
  double3 v2v0 = v2 - v0;
  double3 rov0 = ro - v0;

  double3  n = dcross( v1v0, v2v0 );
  double3  q = dcross( rov0, rd );
  double d = 1.0/dot( rd, n );
  double u = d*dot( -q, v2v0 );
  double v = d*dot(  q, v1v0 );
  double t = d*dot( -n, rov0 );

  if( u<0.0 || v<0.0 || (u+v)>1.0 ) t = -1.0;
  
  if (t > tCurrent) return;
  if (t < tMin) return;
  
  // update current double precision thit
  gprt::store<double>(record.dpRays, fbOfs * 8 + 7, t);

  Attribute attr;
  attr.bc = double2(u, v);
  float f32t = float(t);
  if (double(f32t) < t) f32t = next_after(f32t);
  ReportHit(f32t, /*hitKind*/ 0, attr);
}
