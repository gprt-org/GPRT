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
#include "vkrt.h"

struct Payload
{
[[vk::location(0)]] float3 color;
};


[[vk::shader_record_ext]]
ConstantBuffer<RayGenData> raygenSBTData;
[shader("raygeneration")]
void simpleRayGen() {
  Payload payload;
  uint2 pixelID = DispatchRaysIndex().xy;

  if (pixelID.x == 0 && pixelID.y == 0) {
    printf("Hello from your first raygen program!\n");
    printf("fbPtr %d\n", raygenSBTData.fbPtr);
    printf("Screen size %d %d\n", raygenSBTData.fbSize.x, raygenSBTData.fbSize.y);

    printf("camera origin %f %f %f\n", raygenSBTData.camera.pos.x, raygenSBTData.camera.pos.y, raygenSBTData.camera.pos.z);
    printf("camera dir_00 %f %f %f\n", raygenSBTData.camera.dir_00.x, raygenSBTData.camera.dir_00.y, raygenSBTData.camera.dir_00.z);
    printf("camera dir_du %f %f %f\n", raygenSBTData.camera.dir_du.x, raygenSBTData.camera.dir_du.y, raygenSBTData.camera.dir_du.z);
    printf("camera dir_dv %f %f %f\n", raygenSBTData.camera.dir_dv.x, raygenSBTData.camera.dir_dv.y, raygenSBTData.camera.dir_dv.z);
  }

  float2 screen = (float2(pixelID) + float2(.5f, .5f))  / float2(raygenSBTData.fbSize);
  RayDesc rayDesc;
  rayDesc.Origin = raygenSBTData.camera.pos;
  rayDesc.Direction = 
    normalize(raygenSBTData.camera.dir_00
    + screen.x * raygenSBTData.camera.dir_du
    + screen.y * raygenSBTData.camera.dir_dv
  );
  rayDesc.TMin = 0.001;
  rayDesc.TMax = 10000.0;

  RaytracingAccelerationStructure world = vkrt::getAccelHandle(raygenSBTData.world);
  
  TraceRay(world, RAY_FLAG_FORCE_OPAQUE, 0xff, 0, 0, 0, rayDesc, payload);
  // payload.color = abs(float3(screen.x, screen.y, 0.f)); //rayDesc.Direction);
  
  const int fbOfs = pixelID.x + raygenSBTData.fbSize.x * pixelID.y;
  vk::RawBufferStore<uint32_t>(raygenSBTData.fbPtr + fbOfs * sizeof(uint32_t), vkrt::make_rgba(payload.color));
}

[[vk::shader_record_ext]]
ConstantBuffer<TrianglesGeomData> geomSBTData;
[shader("closesthit")]
void TriangleMesh(inout Payload prd) // , in float2 attribs
{
  // compute normal:
  const int    primID = PrimitiveIndex();
  const int3   index  = vk::RawBufferLoad<int3>(geomSBTData.index  + sizeof(int3) * primID);
  const float3 A      = vk::RawBufferLoad<float3>(geomSBTData.vertex + sizeof(float3) * index.x);
  const float3 B      = vk::RawBufferLoad<float3>(geomSBTData.vertex + sizeof(float3) * index.y);
  const float3 C      = vk::RawBufferLoad<float3>(geomSBTData.vertex + sizeof(float3) * index.z);
  const float3 Ng     = normalize(cross(B-A,C-A));

  const float3 rayDir = WorldRayDirection();
  prd.color = (.2f + .8f * abs(dot(rayDir,Ng))) * geomSBTData.color;
}

[[vk::shader_record_ext]]
ConstantBuffer<MissProgData> missSBTData;
[shader("miss")]
void miss(inout Payload prd)
{
  uint2 pixelID = DispatchRaysIndex().xy;
  
  int pattern = (pixelID.x / 8) ^ (pixelID.y/8);
  prd.color = (pattern & 1) ? missSBTData.color1 : missSBTData.color0;
}
