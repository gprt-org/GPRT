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

// This ray generation program will kick off the ray tracing process,
// generating rays and tracing them into the world.
[shader("raygeneration")]
void
simpleRayGen(uniform RayGenData record) {
  Payload payload;
  uint2 pixelID = DispatchRaysIndex().xy;
  uint2 fbSize = DispatchRaysDimensions().xy;
  float2 screen = (float2(pixelID) + float2(.5f, .5f)) / float2(fbSize);

  RayDesc rayDesc;
  rayDesc.Origin = pc.camera.pos;
  rayDesc.Direction = normalize(pc.camera.dir_00 + screen.x * pc.camera.dir_du + screen.y * pc.camera.dir_dv);
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
LSSClosestHit(uniform LSSGeomData record, inout Payload payload, in float u) {
  uint primID = PrimitiveIndex();
  // Assuming 10 LSS segments (so, 11 vertices)
  float U = (primID + u) / 10.f;

  // Just for fun, animate the make the "u" parameter along the LSS go from -1 to 1
  U -= (sin(pc.time * .5) * .5 + .5);

  // coloring
  U *= 2.0;
  float3 col = (U > 0.0) ? pow(float3(0.9, 0.6, 0.3), 2.2) : pow(float3(0.65, 0.85, 1.0), 2.2);
  col *= 1.0 - exp(-6.0 * abs(U));
  col *= 0.8 + 0.2 * cos(150.0 * U);
  col = lerp(col, float3(1.0), 1.0 - smoothstep(0.0, 0.01, abs(U)));

  payload.color = col;
}

// This miss program will be called when rays miss all primitives.
// We often define some "default" ray payload behavior here,
// for example, returning a background color.
[shader("miss")]
void
miss(uniform MissProgData record, inout Payload payload) {
  uint2 pixelID = DispatchRaysIndex().xy;
  int pattern = (pixelID.x / 32) ^ (pixelID.y / 32);
  payload.color = bool(pattern & 1) ? record.color1 : record.color0;
}
