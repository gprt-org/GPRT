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

#include "sharedCode.h"

[[vk::push_constant]] PushConstants pc;

[shader("compute")]
[numthreads(1,1,1)]
void Transform(uint3 DispatchThreadID : SV_DispatchThreadID, uniform ConstantBuffer<TransformData> record) {
  int numTransforms = record.numTransforms;
  int length = int(sqrt(numTransforms));

  int xid = DispatchThreadID.x % length;
  int yid = DispatchThreadID.x / length;

  float px = float(xid) / float(length);
  float py = float(yid) / float(length);

  float height = .1;
  float width = 4.0;
  float depth = 4.0;
  float k = 10.f;

  float x = lerp(-1.f, 1.f, px);
  float y = lerp(-1.f, 1.f, py);
  float z = sin(pc.now + k * x) * cos(pc.now + k * y);
  float zoffset = x + y;

  float3x4 transform = float3x4(
    float4(0.04, 0.0, 0.0, x * width),
    float4(0.0, 0.04, 0.0, y * depth),
    float4(0.0, 0.0, 0.04, z * height + zoffset)
  );

  int transformID = DispatchThreadID.x;
  gprt::store(record.transforms, transformID, transform);
}

struct Payload {
  float3 color;
};

[shader("raygeneration")]
void RayGen(uniform RayGenData record) {
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
  gprt::store(record.frameBuffer, fbOfs, gprt::make_bgra(payload.color));
}

float3
hsv2rgb(float3 input) {
  float4 K = float4(1.0, 2.0 / 3.0, 1.0 / 3.0, 3.0);
  float3 p = abs(frac(input.xxx + K.xyz) * 6.0 - K.www);
  return input.z * lerp(K.xxx, clamp(p - K.xxx, 0.0, 1.0), input.y);
}

[shader("closesthit")]
void ClosestHit(uniform TrianglesGeomData record, inout Payload payload) {
  uint primID = PrimitiveIndex();
  uint instanceID = InstanceIndex();
  int3 index = gprt::load<int3>(record.index, primID);
  float3 A = gprt::load<float3>(record.vertex, index.x);
  float3 B = gprt::load<float3>(record.vertex, index.y);
  float3 C = gprt::load<float3>(record.vertex, index.z);
  float3 Ng = normalize(cross(B - A, C - A));
  float3 rayDir = normalize(ObjectRayDirection());
  float3 hitPos = ObjectRayOrigin() + RayTCurrent() * ObjectRayDirection();
  float3 color = hsv2rgb(float3(instanceID / 25.f, 1.0, 1.0));
  payload.color = (.1f + .9f * abs(dot(rayDir, Ng))) * color;
}

[shader("miss")]
void miss(uniform MissProgData record, inout Payload payload) {
  uint2 pixelID = DispatchRaysIndex().xy;
  int pattern = (pixelID.x / 8) ^ (pixelID.y / 8);
  payload.color = bool(pattern & 1) ? record.color1 : record.color0;
}
