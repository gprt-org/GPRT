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

[[vk::push_constant]]
PushConstants pc;

struct Payload {
  float3 color;
};

#define AA 3

// This ray generation program will kick off the ray tracing process,
// generating rays and tracing them into the world.
[shader("raygeneration")]
void
simpleRayGen(uniform RayGenData record) {
  Payload payload;
  uint2 pixelID = DispatchRaysIndex().xy;
  uint2 fbSize = DispatchRaysDimensions().xy;
  float2 screen = (float2(pixelID) + float2(.5f, .5f)) / float2(fbSize);

  float3 tot = float3(0.0);

  for (int m = 0; m < AA; m++) {
    for (int n = 0; n < AA; n++) {
      // AA offset
      float2 o = (float2(float(m), float(n)) / float(AA) - 0.5) / float2(fbSize);

      RayDesc rayDesc;
      rayDesc.Origin = pc.camera.pos;
      rayDesc.Direction = normalize(pc.camera.dir_00 + screen.x * pc.camera.dir_du + screen.y * pc.camera.dir_dv);
      rayDesc.Direction += o.x * pc.camera.dir_du + o.y * pc.camera.dir_dv;
      rayDesc.TMin = 0.0;
      rayDesc.TMax = 10000.0;
      RaytracingAccelerationStructure world = gprt::getAccelHandle(record.world);
      TraceRay(world,                   // the tree
               RAY_FLAG_FORCE_OPAQUE,   // ray flags
               0xff,                    // instance inclusion mask
               0,                       // ray type
               1,                       // number of ray types
               0,                       // miss type
               rayDesc,                 // the ray to trace
               payload                  // the payload IO
      );
      tot += payload.color;
    }
  }
  tot /= float(AA * AA);

  // dither to remove banding in the background
  tot += fract(sin(pixelID.x * float3(13, 1, 11) + pixelID.y * float3(1, 7, 5)) * 158.391832) / 255.0;

  const int fbOfs = pixelID.x + fbSize.x * pixelID.y;
  record.frameBuffer[fbOfs] = gprt::make_bgra(tot);
}

float3
pattern(in float2 uv) {
  float3 col = float3(0.6);
  col += 0.4 * smoothstep(-0.01, 0.01, cos(uv.x * 0.5) * cos(uv.y * 0.5));
  col *= smoothstep(-1.0, -0.98, cos(uv.x)) * smoothstep(-1.0, -0.98, cos(uv.y));
  return col;
}

[shader("closesthit")]
void
LSSClosestHit(uniform LSSGeomData record, inout Payload payload, in float u) {
  uint primID = PrimitiveIndex();
  uint2 indices = record.indices[primID];
  float4 pa = record.vertices[indices.x];
  float4 pb = record.vertices[indices.y];
  float4 p = lerp(pa, pb, u);

  float t = RayTCurrent();
  float3 ro = WorldRayOrigin();
  float3 rd = WorldRayDirection();
  float3 pos = ro + t * rd;
  float3 nor = normalize(pos - p.xyz);

  // shading/lighting
  float3 w = normalize(pb.xyz - pa.xyz);
  float3 u = normalize(cross(w, float3(0, 0, 1)));
  float3 v = normalize(cross(u, w));

  float3 q = mul(float3x3(u, v, w), (pos - pa.xyz));
  float3 col = pattern(float2(8.0, 32.0) * float2(atan2(q.y, q.x), q.z));

  float3 lig = normalize(float3(0.7, 0.6, 0.3));
  float3 hal = normalize(-rd + lig);
  float dif = clamp(dot(nor, lig), 0.0, 1.0);
  float amb = clamp(0.5 + 0.5 * dot(nor, float3(0.0, 1.0, 0.0)), 0.0, 1.0);

  col *= float3(0.2, 0.3, 0.4) * amb + float3(1.0, 0.9, 0.7) * dif;
  col += 0.4 * pow(clamp(dot(hal, nor), 0.0, 1.0), 12.0) * dif;

  // shading
  payload.color = col;
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
