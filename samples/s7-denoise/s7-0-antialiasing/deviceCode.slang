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

struct Payload {
  float spreadAngle;
  float3 color;
};

float3x3
AngleAxis3x3(float angle, float3 axis) {
  float c, s;
  sincos(angle, s, c);

  float t = 1 - c;
  float x = axis.x;
  float y = axis.y;
  float z = axis.z;

  return float3x3(t * x * x + c, t * x * y - s * z, t * x * z + s * y, t * x * y + s * z, t * y * y + c,
                  t * y * z - s * x, t * x * z - s * y, t * y * z + s * x, t * z * z + c);
}

[shader("compute")]
[numthreads(1, 1, 1)]
void
Transform(uint3 DispatchThreadID: SV_DispatchThreadID) {
  // This kernel animates the texture planes, rotating them in a circle
  // and placing them in a grid.
  int instanceID = DispatchThreadID.x;

  int transformX = instanceID / 2;
  int transformY = instanceID % 2;

  int numInstances = pc.numInstances;
  float angle = pc.now;
  float3x3 aa = AngleAxis3x3(angle, float3(0.0, 1.0, 0.0));

  gprt::Instance instance = pc.instances[instanceID];
  instance.transform = float3x4(float4(aa[0], lerp(-5, 5, transformX / float((numInstances / 2) - 1))),
                                float4(aa[1], lerp(-1, 1, transformY)), float4(aa[2], 0.0));

  pc.instances[instanceID] = instance;
}

[shader("raygeneration")]
void
raygen(uniform RayGenData record) {
  Payload payload;
  uint2 dims = DispatchRaysDimensions().xy;
  uint2 pixelID = DispatchRaysIndex().xy;
  float2 screen = (float2(pixelID) + float2(.5f, .5f)) / float2(dims);

  // Generate ray
  RayDesc rayDesc;
  rayDesc.Origin = pc.camera.pos;
  rayDesc.Direction = normalize(pc.camera.dir_00 + screen.x * pc.camera.dir_du + screen.y * pc.camera.dir_dv);
  rayDesc.TMin = 0.0;
  rayDesc.TMax = 1e38f;

  // compute the spreading angle of the ray to determine the sampling footprint
  float phi = pc.camera.fovy;
  float H = dims.y;
  payload.spreadAngle = atan(2.f * tan(phi / 2.f) / H);

  // Trace ray against surface
  TraceRay(record.world,            // the tree
           RAY_FLAG_FORCE_OPAQUE,   // ray flags
           0xff,                    // instance inclusion mask
           0,                       // ray type
           1,                       // number of ray types
           0,                       // miss type
           rayDesc,                 // the ray to trace
           payload                  // the payload IO
  );

  const int fbOfs = pixelID.x + dims.x * pixelID.y;
  record.frameBuffer[fbOfs] = gprt::make_bgra(payload.color);
}

[shader("closesthit")]
void
closesthit(uniform TrianglesGeomData record, inout Payload payload, float2 bc) {
  // compute normal:
  uint instanceID = InstanceIndex();
  uint primID = PrimitiveIndex();
  uint3 index = record.index[primID];
  float3 A = record.vertex[index.x];
  float3 B = record.vertex[index.y];
  float3 C = record.vertex[index.z];
  float3 Ng = normalize(cross(B - A, C - A));

  // compute texture coordinate:
  float2 TCA = record.texcoord[index.x];
  float2 TCB = record.texcoord[index.y];
  float2 TCC = record.texcoord[index.z];
  float2 TC = TCB * bc.x + TCC * bc.y + TCA * (1.f - (bc.x + bc.y));

  // fetch the texture and sampler for this plane
  DescriptorHandle<Texture2D> texture = record.texture;
  DescriptorHandle<SamplerState> sampler = record.samplers[0];

  // For now, we're using an approximate ray cone method to drive texture sampling.
  // The cone footprint grows larger with distance, and is grows larger at glancing angles
  float footprint = 0.f; // Set the footprint to 0 to cause aliasing.
  float2 DDX = float2(footprint, footprint) * .5f;
  float2 DDY = float2(footprint, footprint) * .5f;
  float4 color = float4(1.0, 0.0, 1.0, 1.0);

  // Here's how you'd normally sample a texture
  color = texture.SampleGrad(sampler, TC*(InstanceIndex() + 1.f), DDX, DDY);
  
  payload.color = color.rgb;
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
