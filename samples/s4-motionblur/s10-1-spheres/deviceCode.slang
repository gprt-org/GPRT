#include "sharedCode.h"

struct Payload {
  float3 color;
};

[shader("closesthit")]
void SphereClosestHit(ShaderRecord<SphereGeomData> record, PayloadState<Payload> payload) {
  uint primID = PrimitiveIndex();
  float4 p = record.posAndRadius[primID];

  float t = RayTCurrent();
  float3 ro = WorldRayOrigin();
  float3 rd = WorldRayDirection();
  float3 pos = ro + t * rd;
  float3 nor = normalize(pos - p.xyz);
  float occ = 0.5 + 0.5 * nor.y;

  float3 q = normalize(pos - p.xyz);
  float2 matuv = float2(16.0, 16.0) * float2(atan2(q.x, q.z), acos(q.y));

  float3 albedo = hsv2rgb(float3(float(primID) / 5.f, 1.0, 1.0));
  float3 col = pattern(matuv, albedo);
  float NDotV = dot(nor, -rd);
  payload.color = NDotV * col;
}

// This ray generation program will kick off the ray tracing process,
// generating rays and tracing them into the world.
[shader("raygeneration")]
void raygen(ShaderRecord<RayGenData> record, PushConstant<Constants> pc) {
  Payload payload;
  uint2 pixelID = DispatchRaysIndex().xy;
  uint2 iResolution = DispatchRaysDimensions().xy;

  // camera movement
  float an = pc.time;
  float3 ro = float3(-6.0 * sin(an), 0.0, -6.0 * cos(an));
  float3 ta = float3(0.0, 0.0, 0.0);

  // camera matrix
  float3 ww = normalize(ta - ro);
  float3 uu = normalize(cross(ww, float3(0.0, -1.0, 0.0)));
  float3 vv = normalize(cross(uu, ww));

  float3 tot = float3(0.0);

  for (int m = 0; m < AA; m++) {
  for (int n = 0; n < AA; n++) {
    // pixel coordinates
    float2 o = float2(float(m), float(n)) / float(AA) - 0.5;
    float2 p = (2.0 * (pixelID + o) - iResolution.xy) / iResolution.y;

    // create view ray
    float3 rd = normalize(p.x * uu + p.y * vv + 3.0 * ww);

    // Trace the ray into the scene
    RayDesc rayDesc;
    rayDesc.Origin = ro;
    rayDesc.Direction = rd;
    rayDesc.TMin = 0.0;
    rayDesc.TMax = 10000.0;
    TraceRay(record.world, RAY_FLAG_NONE, 0xff, 0, 0, rayDesc, payload);

    // Accumulate the color
    tot += payload.color;
  }
  }

  tot /= float(AA * AA);

  const int fbOfs = pixelID.x + iResolution.x * pixelID.y;
  record.frameBuffer[fbOfs] = gprt::make_bgra(tot);
}

[shader("miss")]
void miss(PayloadState<Payload> payload) {
  payload.color = float3(0.0);
}