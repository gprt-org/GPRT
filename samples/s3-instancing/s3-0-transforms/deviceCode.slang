#include "sharedCode.h"

[[vk::push_constant]]
PushConstants pc;

struct Payload {
  float3 color;
};

// This ray generation program will kick off the ray tracing process,
// generating rays and tracing them into the world.
[shader("raygeneration")]
void simpleRayGen(uniform RayGenData record) {
  Payload payload;
  uint2 pixelID = DispatchRaysIndex().xy;
  uint2 fbSize = DispatchRaysDimensions().xy;
  float2 screen = (float2(pixelID) + float2(.5f, .5f)) / float2(fbSize);

  RayDesc rayDesc;
  rayDesc.Origin = pc.camera.pos;
  rayDesc.Direction = normalize(pc.camera.dir_00 + screen.x * pc.camera.dir_du + screen.y * pc.camera.dir_dv);
  rayDesc.TMin = 0.001;
  rayDesc.TMax = 10000.0;
  TraceRay(record.world,                   // the tree
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

// This closest hit program will be called when rays hit triangles.
// Here, we can fetch per-geometry data, process that data, and send
// it back to our ray generation program.
[shader("closesthit")]
void TriangleMesh(uniform TrianglesGeomData record, inout Payload payload, in float2 bc) {
  // compute normal:
  uint primID = PrimitiveIndex();
  uint instanceID = InstanceIndex();
  uint3 index = record.index[primID];
  float3 A = record.vertex[index.x];
  float3 B = record.vertex[index.y];
  float3 C = record.vertex[index.z];
  float3 Ng = normalize(cross(B - A, C - A));
  float3 rayDir = WorldRayDirection();

  float3 color = hsv2rgb(float3(instanceID / 3.0, 1.0, 1.0));
  payload.color = (.1f + .9f * abs(dot(rayDir, Ng))) * color;
}

[shader("miss")]
void miss(inout Payload payload) {
  payload.color = float3(0.0);
}