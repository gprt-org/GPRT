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

  const int fbOfs = pixelID.x + record.fbSize.x * pixelID.y;
    vk::RawBufferStore<uint32_t>(record.fbPtr + fbOfs * sizeof(uint32_t), 
      gprt::make_rgba(payload.color));
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
  int3 indices = vk::RawBufferLoad<int3>(record.index + sizeof(int3) * primID);
  double3 A = vk::RawBufferLoad<double3>(record.vertex + sizeof(int3) * indices.x);
  double3 B = vk::RawBufferLoad<double3>(record.vertex + sizeof(int3) * indices.y);
  double3 C = vk::RawBufferLoad<double3>(record.vertex + sizeof(int3) * indices.z);
  double3 dpaabbmin = min(min(A, B), C);
  double3 dpaabbmax = max(max(A, B), C);
  float3 fpaabbmin = float3(dpaabbmin) - float3(EPSILON, EPSILON, EPSILON); // todo, round this below smallest float 
  float3 fpaabbmax = float3(dpaabbmax) + float3(EPSILON, EPSILON, EPSILON); // todo, round this below smallest float 
  vk::RawBufferStore<float3>(record.aabbs + 2 * sizeof(float3) * primID, fpaabbmin);
  vk::RawBufferStore<float3>(record.aabbs + 2 * sizeof(float3) * primID + sizeof(float3), fpaabbmax);

  if (primID == 0) {
    printf("%f %f %f %f %f %f\n", 
    fpaabbmin.x, fpaabbmin.y, fpaabbmin.z,
    fpaabbmax.x, fpaabbmax.y, fpaabbmax.z);
  }
}

GPRT_CLOSEST_HIT_PROGRAM(DPTriangle, (DPTriangleData, record), (Payload, payload), (Attribute, attribute))
{
  double2 barycentrics = attribute.bc;
  payload.color = float3(barycentrics.x, barycentrics.y, 0.0);
}

GPRT_INTERSECTION_PROGRAM(DPTriangle, (DPTriangleData, record))
{
  float3 ro = ObjectRayOrigin();
  float3 rd = ObjectRayDirection();

  int primID = PrimitiveIndex();
  int3 indices = vk::RawBufferLoad<int3>(record.index + sizeof(int3) * primID);
  double3 A = vk::RawBufferLoad<double3>(record.vertex + sizeof(int3) * indices.x);
  double3 B = vk::RawBufferLoad<double3>(record.vertex + sizeof(int3) * indices.y);
  double3 C = vk::RawBufferLoad<double3>(record.vertex + sizeof(int3) * indices.z);

  float3 v0 = float3(A);
  float3 v1 = float3(B);
  float3 v2 = float3(C);

  float3 v1v0 = v1 - v0;
  float3 v2v0 = v2 - v0;
  float3 rov0 = ro - v0;

  float3  n = cross( v1v0, v2v0 );
  float3  q = cross( rov0, rd );
  float d = 1.0/dot( rd, n );
  float u = d*dot( -q, v2v0 );
  float v = d*dot(  q, v1v0 );
  float t = d*dot( -n, rov0 );

  if( u<0.0 || v<0.0 || (u+v)>1.0 ) t = -1.0;
  
  if (t > 0.0) {
    Attribute attr;
    attr.bc = double2(u, v);
    ReportHit(t, /*hitKind*/ 0, attr);
  }
}
