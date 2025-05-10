#include "sharedCode.h"

[[vk::push_constant]]
PushConstants pc;

struct Payload {
  float3 color;
  int count;
};

// This closest hit shader runs when a ray hits a triangle.
// It processes per-geometry data and communicates with the ray generation shader.
// - The first parameter is the shader record type, representing SBT parameters for this shader.
// - The second is the ray payload type, used for passing data between shaders.
// - The third is the intersection attributes. For triangles, these are two "barycentrics", used 
//   for interpolating per-vertex values.
[shader("closesthit")]
void TriangleMesh(uniform TrianglesGeomData record, inout Payload payload, in float2 bc) {
  // float3 color = (PrimitiveIndex() == 0) ? float3(1.f, 0.f, 0.f) : float3(0.f, 1.f, 0.f);
  // payload.color = color;
}

[shader("anyhit")]
void TriangleMeshAnyHit(uniform TrianglesGeomData record, inout Payload payload, in float2 bc) {
  payload.count++;
  payload.color += (PrimitiveIndex() == 0) ? float3(1.f, 0.f, 0.f) : float3(0.f, 1.f, 0.f);

  IgnoreHit();
}

// This ray generation program will kick off the ray tracing process,
// generating rays and tracing them into the world.
[shader("raygeneration")]
void raygen(uniform RayGenData record) {
  Payload payload;
  payload.color = float3(0.f);
  payload.count = 0;

  uint2 pixelID = DispatchRaysIndex().xy;
  uint2 iResolution = DispatchRaysDimensions().xy;

  // camera movement
  float an = pc.time;

  float offset = 1e-2f;//lerp(1.0f, FLT_EPSILON, .5 + sin(an) *.5);
  float2 uv = float2(pixelID / 50) / float2(iResolution / 50);

  float3 ta = float3(lerp(float2(0.0, 0.0) - offset, float2(0.0, 0.0) + offset, uv), 0.0);
  // float3 ta = float3(0.f);
  float3 ro = float3(-4.f) + ta;

  // camera matrix
  float3 ww = normalize(ta - ro);
  float3 uu = normalize(cross(ww, float3(0.0, -1.0, 0.0)));
  float3 vv = normalize(cross(uu, ww));

  float3 tot = float3(0.0);

  // for (int m = 0; m < AA; m++)
  // for (int n = 0; n < AA; n++)
  {
    // pixel coordinates
    // float2 o = float2(float(m), float(n)) / float(AA) - 0.5;
    // float2 p = (2.0 * (pixelID + o) - iResolution.xy) / iResolution.y;

    // create view ray
    float3 rd = ww;   // normalize(ww);   // normalize(p.x * uu + p.y * vv + 1e19 * ww);
    // float3 rd = (0.f, 0.f, 1.f);   // normalize(ww);   // normalize(p.x * uu + p.y * vv + 1e19 * ww);
    
    // float3 rd = normalize(p.x * uu + p.y * vv + 3.f * ww);

    // Trace the ray into the scene
    RayDesc rayDesc;
    rayDesc.Origin = ro;
    rayDesc.Direction = rd;// * 1e-19;
    rayDesc.TMin = 0.0;
    rayDesc.TMax = 1e38f;
    TraceRay(record.world, RAY_FLAG_FORCE_NON_OPAQUE, 0xff, 0, 0, rayDesc, payload);

    // if (payload.count == 0) payload.color = float3(0.f);
    // if (payload.count == 1) payload.color = float3(0.3f);
    // if (payload.count == 2) payload.color = float3(1.f, 0.f, 0.f);

    // Accumulate the color
    tot = payload.color;
  }
  // tot /= float(AA * AA);

  const int fbOfs = pixelID.x + iResolution.x * pixelID.y;
  record.frameBuffer[fbOfs] = gprt::make_bgra(tot);
}

[shader("miss")]
void miss(inout Payload payload) {
  // payload.color = float3(0.0);
}
