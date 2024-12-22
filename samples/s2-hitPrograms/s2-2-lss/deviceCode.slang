#include "sharedCode.h"

[[vk::push_constant]]
PushConstants pc;

struct Payload {
  float3 color;
};

[shader("closesthit")]
void LSSClosestHit(uniform LSSGeomData record, inout Payload payload, in float u) {
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

#define AA 3

// This ray generation program will kick off the ray tracing process,
// generating rays and tracing them into the world.
[shader("raygeneration")]
void raygen(uniform RayGenData record) {
  Payload payload;
  uint2 pixelID = DispatchRaysIndex().xy;
  uint2 iResolution = DispatchRaysDimensions().xy;

  RaytracingAccelerationStructure world = gprt::getAccelHandle(record.world);

  // camera movement
  float an = pc.time;
  float3 ro = float3(-6.0 * sin(an), 0.0, -6.0 * cos(an));
  float3 ta = float3(0.0, 0.0, 0.0);

  // camera matrix
  float3 ww = normalize(ta - ro);
  float3 uu = normalize(cross(ww, float3(0.0, -1.0, 0.0)));
  float3 vv = normalize(cross(uu, ww));

  float3 tot = float3(0.0);

  for(int m = 0; m < AA; m++) {
  for (int n = 0; n < AA; n++) {
    // pixel coordinates
    float2 o = float2(float(m), float(n)) / float(AA) - 0.5;
    float2 p = (2.0 * (pixelID + o) - iResolution.xy) / iResolution.y;

    // create view ray
    float3 rd = normalize(p.x * uu + p.y * vv + 6.0 * ww);

    RayDesc rayDesc;
    rayDesc.Origin = ro;
    rayDesc.Direction = rd;
    rayDesc.TMin = 0.0;
    rayDesc.TMax = 10000.0;
    TraceRay(world,           // the tree
              RAY_FLAG_NONE,   // ray flags
              0xff,            // instance inclusion mask
              0,               // ray type
              1,               // number of ray types
              0,               // miss index
              rayDesc,         // the ray to trace
              payload          // the payload IO
    );

    tot += payload.color;
  }
  }

  tot /= float(AA * AA);

  // dither to remove banding in the background
  tot += fract(sin(pixelID.x * float3(13, 1, 11) + pixelID.y * float3(1, 7, 5)) * 158.391832) / 255.0;

  const int fbOfs = pixelID.x + iResolution.x * pixelID.y;
  record.frameBuffer[fbOfs] = gprt::make_bgra(tot);
}

// A background with a vignette effect.
[shader("miss")]
void miss(inout Payload payload) {
  float2 resolution = DispatchRaysDimensions().xy;
  float2 fragCoord = DispatchRaysIndex().xy;
  float2 p = (-resolution.xy + 2.0 * fragCoord) / resolution.y;
  float3 col = float3(0.08) * (1.0 - 0.3 * length(p)) + 0.02 * WorldRayDirection().y;
  payload.color = col;
}
