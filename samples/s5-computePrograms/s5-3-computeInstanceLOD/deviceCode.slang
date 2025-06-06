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

[[vk::push_constant]]
PushConstants pc;

[shader("compute")]
[numthreads(128, 1, 1)]
void
Transform(uint3 DispatchThreadID: SV_DispatchThreadID) {
  int instanceID = DispatchThreadID.x;

  float minDist = lerp(1.0, 3.0, .5 * sin(pc.now * .3) + .5);
  float maxDist = lerp(minDist, 8.0, .5 * sin(pc.now * .3) + .5);

  int instancesPerSide = int(sqrt(pc.numInstances));

  int xid = DispatchThreadID.x % instancesPerSide;
  int yid = DispatchThreadID.x / instancesPerSide;

  float px = float(xid) / float(instancesPerSide);
  float py = float(yid) / float(instancesPerSide);

  float height = .1;
  float width = 4.0;
  float depth = 4.0;
  float k = 10.f;

  float x = lerp(-1.f, 1.f, px);
  float y = lerp(-1.f, 1.f, py);
  float z = sin(.1 * pc.now + k * x) * cos(pc.now + k * y);
  float zoffset = x + y;

  float scale = .045;
  float3 center = float3(x * width, y * depth, z * height + zoffset);

  int numLODs = 6;

  float distCameraToObject = distance(pc.camera.pos, center);
  uint level = uint(lerp(numLODs - 1, 0.0, clamp((distCameraToObject - minDist) / (maxDist - minDist), 0.0, 1.0)));

  gprt::Instance instance = pc.meshLODs[level];

  instance.transform[0] = float4(scale, 0.0, 0.0, center.x);
  instance.transform[1] = float4(0.0, scale, 0.0, center.y);
  instance.transform[2] = float4(0.0, 0.0, scale, center.z);

  pc.instances[instanceID] = instance;
}

struct Payload {
  float3 color;
};

[shader("raygeneration")]
void
RayGen(uniform RayGenData record) {
  Payload payload;
  uint2 pixelID = DispatchRaysIndex().xy;
  uint2 fbSize = DispatchRaysDimensions().xy;
  float2 screen = (float2(pixelID) + float2(.5f, .5f)) / float2(fbSize);
  RayDesc rayDesc;
  rayDesc.Origin = pc.camera.pos;
  rayDesc.Direction = normalize(pc.camera.dir_00 + screen.x * pc.camera.dir_du + screen.y * pc.camera.dir_dv);
  rayDesc.TMin = 0.0;
  rayDesc.TMax = 10000.0;
  TraceRay(record.world, RAY_FLAG_FORCE_OPAQUE, 0xff, 0, 0, rayDesc, payload);
  const int fbOfs = pixelID.x + fbSize.x * pixelID.y;
  record.frameBuffer[fbOfs] = gprt::make_bgra(payload.color);
}

float3
hsv2rgb(float3 input) {
  float4 K = float4(1.0, 2.0 / 3.0, 1.0 / 3.0, 3.0);
  float3 p = abs(frac(input.xxx + K.xyz) * 6.0 - K.www);
  return input.z * lerp(K.xxx, clamp(p - K.xxx, 0.0, 1.0), input.y);
}

[shader("closesthit")]
void
ClosestHit(uniform TrianglesGeomData record, inout Payload payload, in float2 bc) {
  uint primID = PrimitiveIndex();
  uint instanceID = InstanceIndex();

  uint3 index = record.index[primID];
  float3 A = record.vertex[index.x];
  float3 B = record.vertex[index.y];
  float3 C = record.vertex[index.z];
  float3 Ng = normalize(cross(B - A, C - A));
  float3 rayDir = normalize(ObjectRayDirection());
  float3 hitPos = ObjectRayOrigin() + RayTCurrent() * ObjectRayDirection();

  float3 color = hsv2rgb(float3(lerp(.3, 1.0, (record.lvlOfDetail) / 5.f), 1.0, 1.0));

  // To render the wireframe of the triangles, we'll use the triangles' barycentrics
  float3 uvw = float3(1.0 - (bc.x + bc.y), bc.x, bc.y);
  float interiorShade = any(abs(uvw) < .1) ? 1. : .5;
  payload.color = (.1f + .9f * abs(dot(rayDir, Ng))) * interiorShade * color;
}

// A background with a vignette effect.
[shader("miss")]
void
miss(uniform MissProgData record, inout Payload payload) {
  float2 resolution = DispatchRaysDimensions().xy;
  float2 fragCoord = DispatchRaysIndex().xy;
  float2 p = (-resolution.xy + 2.0 * fragCoord) / resolution.y;
  float3 col = float3(0.08) * (1.0 - 0.3 * length(p)) + 0.02 * WorldRayDirection().y;
  payload.color = col;
}
