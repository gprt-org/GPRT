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
  float4 color;
  float3 normal;
  float hitT;
};

float4
over(float4 a, float4 b) {
  float4 o;
  o.a = a.a + b.a * (1.f - a.a);
  o.rgb = (a.rgb * a.a + b.rgb * b.a * (1.f - a.a)) / o.a;
  return o;
}

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
  rayDesc.TMin = 0.001;
  rayDesc.TMax = 10000.0;
  RaytracingAccelerationStructure world = gprt::getAccelHandle(record.world);

  // Trace our primary visibility ray
  TraceRay(world,                   // the tree
           RAY_FLAG_FORCE_OPAQUE,   // ray flags
           0b111111111,             // instance inclusion mask
           0,                       // ray type
           1,                       // number of ray types
           0,                       // miss type
           rayDesc,                 // the ray to trace
           payload                  // the payload IO
  );

  float4 shadedColor = float4(0.f, 0.f, 0.f, 0.f);

  // Did we hit something transparent?
  float4 transparentColor = float4(0.f, 0.f, 0.f, 0.f);
  if (payload.hitT != -1.f && payload.color.a < 1.f) {
    // cache the transparent object's color
    transparentColor = payload.color;

    // Move ray to hit object position
    rayDesc.Origin = rayDesc.Origin + payload.hitT * rayDesc.Direction;

    // Trace another ray to get color of what's behind
    TraceRay(world,                   // the tree
             RAY_FLAG_FORCE_OPAQUE,   // ray flags
             0b111111111,             // instance inclusion mask
             0,                       // ray type
             1,                       // number of ray types
             0,                       // miss type
             rayDesc,                 // the ray to trace
             payload                  // the payload IO
    );
  }

  // If at this point we hit the background, composite
  if (payload.hitT == -1.f) {
    // set the shaded color to what was returned
    shadedColor = over(transparentColor, payload.color);
  }

  // Else we hit something opaque...
  else {
    // Now trace a shadow ray towards the light
    float3 color = payload.color.rgb;
    float3 normal = payload.normal;
    float3 hitPos = rayDesc.Origin + payload.hitT * rayDesc.Direction;

    // note, adding a normal offset to avoid self-intersections
    rayDesc.Origin = hitPos + .001f * normal;
    rayDesc.Direction = normalize(pc.lightPos);

    // Trace our primary visibility ray
    TraceRay(world,                   // the tree
             RAY_FLAG_FORCE_OPAQUE,   // ray flags
             0b00000001,              // instance inclusion mask
             0,                       // ray type
             1,                       // number of ray types
             0,                       // miss type
             rayDesc,                 // the ray to trace
             payload                  // the payload IO
    );

    float ambient = .5f;
    float3 litColor = color * ambient;

    // We hit the light
    if (payload.hitT == -1.f) {
      litColor += color * max(dot(normal, rayDesc.Direction), 0.0f) * (1.f - ambient) * pc.lightColor;
    }

    shadedColor = over(transparentColor, float4(litColor, 1.f));
  }

  const int fbOfs = pixelID.x + fbSize.x * pixelID.y;
  record.frameBuffer[fbOfs] = gprt::make_bgra(shadedColor);
}

[shader("closesthit")]
void
TriangleMesh(uniform TrianglesGeomData record, inout Payload payload) {
  // compute normal:
  uint primID = PrimitiveIndex();
  uint instanceID = InstanceIndex();
  uint3 index = record.indices[primID];
  float3 A = record.vertices[index.x];
  float3 B = record.vertices[index.y];
  float3 C = record.vertices[index.z];

  payload.color = record.color;
  payload.hitT = RayTCurrent();
  // NOTE: we are assuming an identity transform from object to world
  payload.normal = normalize(cross(B - A, C - A));
  float normalSign = (dot(payload.normal, WorldRayDirection()) < 0.f) ? 1.f : -1.f;
  payload.normal = normalSign * payload.normal;
}

[shader("miss")]
void
miss(uniform MissProgData record, inout Payload payload) {
  uint2 pixelID = DispatchRaysIndex().xy;
  int pattern = (pixelID.x / 32) ^ (pixelID.y / 32);
  payload.color = float4(bool(pattern & 1) ? record.color1 : record.color0, 1.f);
  payload.hitT = -1.f;
}
