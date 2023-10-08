// MIT License

// Copyright (c) 2022 Nathan V. Morrical

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

#include "sharedCode.h"

[[vk::push_constant]] PushConstants pc;

float3
getPos(float px, float py, float k, float width, float depth, float height, float now) {
  float x = lerp(-1.f, 1.f, px);
  float y = lerp(-1.f, 1.f, py);
  float z = sin(now + k * x) * cos(now + k * y);
  float zoffset = x + y;
  return float3(x * width, y * depth, z * height + zoffset);
}

GPRT_COMPUTE_PROGRAM(Vertex, (TrianglesGeomData, record), (1, 1, 1)) {
  uint gridSize = record.gridSize;
  int TriID = DispatchThreadID.x;

  bool even = (TriID % 2) == 0;

  float gx = ((TriID / 2) % gridSize) / float(gridSize);
  float gy = ((TriID / 2) / gridSize) / float(gridSize);

  float dx = 1.f / float(gridSize);
  float dy = 1.f / float(gridSize);

  uint3 index = uint3(3 * TriID + 0, 3 * TriID + 1, 3 * TriID + 2);
  float3 v0, v1, v2;

  float height = .1;
  float width = 4.0;
  float depth = 4.0;
  float k = 20.f;

  if (even) {
    v0 = getPos(gx, gy, k, width, depth, height, pc.now);
    v1 = getPos(gx + dx, gy, k, width, depth, height, pc.now);
    v2 = getPos(gx + dx, gy + dy, k, width, depth, height, pc.now);
  } else {
    v0 = getPos(gx, gy, k, width, depth, height, pc.now);
    v1 = getPos(gx + dx, gy + dy, k, width, depth, height, pc.now);
    v2 = getPos(gx, gy + dy, k, width, depth, height, pc.now);
  }

  gprt::store(record.index, TriID, index);
  gprt::store(record.vertex, index.x, v0);
  gprt::store(record.vertex, index.y, v1);
  gprt::store(record.vertex, index.z, v2);
}

struct [raypayload] Payload {
  float3 color : read(caller) : write(closesthit, miss);
};

GPRT_RAYGEN_PROGRAM(RayGen, (RayGenData, record)) {
  Payload payload;
  uint2 pixelID = DispatchRaysIndex().xy;
  uint2 fbSize = DispatchRaysDimensions().xy;
  float2 screen = (float2(pixelID) + float2(.5f, .5f)) / float2(fbSize);
  RayDesc rayDesc;
  rayDesc.Origin = pc.camera.pos;
  rayDesc.Direction =
      normalize(pc.camera.dir_00 + screen.x * pc.camera.dir_du + screen.y * pc.camera.dir_dv);
  rayDesc.TMin = 0.0;
  rayDesc.TMax = 10000.0;
  RaytracingAccelerationStructure world = gprt::getAccelHandle(record.world);
  TraceRay(world, RAY_FLAG_FORCE_OPAQUE, 0xff, 0, 1, 0, rayDesc, payload);
  const int fbOfs = pixelID.x + fbSize.x * pixelID.y;
  gprt::store(record.frameBuffer, fbOfs, gprt::make_rgba(payload.color));
}

struct Attribute {
  float2 bc;
};

float3
hsv2rgb(float3 input) {
  float4 K = float4(1.0, 2.0 / 3.0, 1.0 / 3.0, 3.0);
  float3 p = abs(frac(input.xxx + K.xyz) * 6.0 - K.www);
  return input.z * lerp(K.xxx, clamp(p - K.xxx, 0.0, 1.0), input.y);
}

GPRT_CLOSEST_HIT_PROGRAM(ClosestHit, (TrianglesGeomData, record), (Payload, payload), (Attribute, attribute)) {
  uint primID = PrimitiveIndex();
  uint instanceID = InstanceIndex();
  int3 index = gprt::load<int3>(record.index, primID);
  float3 A = gprt::load<float3>(record.vertex, index.x);
  float3 B = gprt::load<float3>(record.vertex, index.y);
  float3 C = gprt::load<float3>(record.vertex, index.z);
  float3 Ng = normalize(cross(B - A, C - A));
  float3 rayDir = WorldRayDirection();

  float3 hitPos = ObjectRayOrigin() + RayTCurrent() * ObjectRayDirection();
  float3 color = hsv2rgb(float3(primID / 1000000.f, 1.0, 1.0));
  payload.color = (.5f + .5f * abs(dot(rayDir, Ng))) * color;
}

GPRT_MISS_PROGRAM(miss, (MissProgData, record), (Payload, payload)) {
  uint2 pixelID = DispatchRaysIndex().xy;
  int pattern = (pixelID.x / 32) ^ (pixelID.y / 32);
  payload.color = (pattern & 1) ? record.color1 : record.color0;
}
