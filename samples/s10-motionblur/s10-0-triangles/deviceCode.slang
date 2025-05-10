#include "sharedCode.h"

[[vk::push_constant]]
PushConstants pc;

struct Payload {
  float3 color;
};

uint murmurHash12(uint2 src) {
  const uint M = 0x5bd1e995u;
  uint h = 1190494759u;
  src *= M; src ^= src >> 24u; src *= M;
  h *= M; h ^= src.x; h *= M; h ^= src.y;
  h ^= h >> 13u; h *= M; h ^= h >> 15u;
  return h;
}

// 1 output, 2 inputs
float hash12(float2 src) {
  uint h = murmurHash12(asuint(src));
  return asfloat(h & 0x007fffffu | 0x3f800000u) - 1.0;
}

// This closest hit shader runs when a ray hits a triangle.
// It processes per-geometry data and communicates with the ray generation shader.
// - The first parameter is the shader record type, representing SBT parameters for this shader.
// - The second is the ray payload type, used for passing data between shaders.
// - The third is the intersection attributes. For triangles, these are two "barycentrics", used 
//   for interpolating per-vertex values.
[shader("closesthit")]
void TriangleMesh(uniform TrianglesGeomData record, inout Payload payload, in float2 bc) {
  payload.color = float3(bc.x, bc.y, 1.0 - (bc.x + bc.y));
}

// This ray generation program will kick off the ray tracing process,
// generating rays and tracing them into the world.
[shader("raygeneration")]
void raygen(uniform RayGenData record) {
  Payload payload;
  uint2 pixelID = DispatchRaysIndex().xy;
  uint2 iResolution = DispatchRaysDimensions().xy;

  // camera movement
  float an = pc.time;
  float3 ro = float3(-4.0 * sin(an), 0.0, -4.0 * cos(an));
  float3 ta = float3(0.0, 0.0, 0.0);

  // camera matrix
  float3 ww = normalize(ta - ro);
  float3 uu = normalize(cross(ww, float3(0.0, -1.0, 0.0)));
  float3 vv = normalize(cross(uu, ww));

  float3 tot = float3(0.0);

  for (int m = 0; m < AA; m++)
  for (int n = 0; n < AA; n++)
  {
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
    float Time = hash12(p + iResolution.xy * an);
    TraceMotionRay(record.world, RAY_FLAG_NONE, 0xff, 0, 0, rayDesc, Time, payload);

    // Accumulate the color
    tot += payload.color;
  }
  tot /= float(AA * AA);

  const int fbOfs = pixelID.x + iResolution.x * pixelID.y;
  record.frameBuffer[fbOfs] = gprt::make_bgra(tot);
}

[shader("miss")]
void miss(inout Payload payload) {
  payload.color = float3(0.0);
}
