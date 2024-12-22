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
      rayDesc.Direction = normalize(rayDesc.Direction);
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

// This intersection program will be called when rays hit our axis
// aligned bounding boxes. Here, we can fetch per-geometry data and
// process that data, but we do not have access to the ray payload
// structure here.
//
// Instead, we pass data through a customizable Attributes structure
// for further processing by closest hit / any hit programs.
struct BBoxAttributes {
  float3 cen;
  float tN;
  float3 rad;
  float tF;
};

[shader("intersection")]
void
AABBIntersection(uniform AABBGeomData record) {
  uint primID = PrimitiveIndex();
  float3 ro = ObjectRayOrigin();
  float3 rd = ObjectRayDirection();
  float tcur = RayTCurrent();

  BBoxAttributes attr;

  // raytrace bounding box
  float3 bbmin = record.aabbs[primID * 2 + 0];
  float3 bbmax = record.aabbs[primID * 2 + 1];

  attr.cen = 0.5 * (bbmin + bbmax);
  attr.rad = 0.5 * (bbmax - bbmin);

  float3 m = 1.0 / rd;
  float3 n = m * (ro - attr.cen);
  float3 k = abs(m) * attr.rad;

  float3 t1 = -n - k;
  float3 t2 = -n + k;

  attr.tN = max(max(t1.x, t1.y), t1.z);
  attr.tF = min(min(t2.x, t2.y), t2.z);

  if (attr.tN > attr.tF || attr.tF < 0.0)
    return;

  int hitKind = 0;
  if (attr.tN > 0.0) {
    // front face
    if (attr.tN <= tcur)
      hitKind |= 1;
    // back face
    if (attr.tF <= tcur)
      hitKind |= 2;
  }

  if (hitKind != 0)
    ReportHit(attr.tN, hitKind, attr);
}

// This closest hit program will be called when our intersection program
// reports a hit between our ray and our custom primitives.
// Here, we can fetch per-geometry data, process that data, and send
// it back to our ray generation program.
//
// Note, since this is a custom AABB primitive, our intersection program
// above defines what attributes are passed to our closest hit program.
//
// Also note, this program is also called after all ReportHit's have been
// called and we can conclude which reported hit is closest.
[shader("closesthit")]
void
AABBClosestHit(uniform AABBGeomData record, inout Payload payload, in BBoxAttributes attr) {
  float3 ro = ObjectRayOrigin();
  float3 rd = ObjectRayDirection();
  int hitKind = HitKind();

  // front
  if (bool(hitKind & 1)) {
    float3 pos = ro + rd * attr.tN;
    float3 e = smoothstep(attr.rad - 0.03, attr.rad - 0.02, abs(pos - attr.cen));
    float al = 1.0 - (1.0 - e.x * e.y) * (1.0 - e.y * e.z) * (1.0 - e.z * e.x);
    payload.color = lerp(float3(0.0), float3(1.0), 0.15 + 0.85 * al);
  }

  // back
  if (bool(hitKind & 2)) {
    float3 pos = ro + rd * attr.tF;
    float3 e = smoothstep(attr.rad - 0.03, attr.rad - 0.02, abs(pos - attr.cen));
    float al = 1.0 - (1.0 - e.x * e.y) * (1.0 - e.y * e.z) * (1.0 - e.z * e.x);
    payload.color = lerp(payload.color, float3(1.0), 0.25 + 0.75 * al);
  }
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
