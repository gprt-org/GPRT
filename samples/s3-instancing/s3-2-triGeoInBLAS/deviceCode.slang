#include "sharedCode.h"

[[vk::push_constant]]
PushConstants pc;

struct Payload {
  float3 color;
};

[shader("raygeneration")]
void raygen(uniform RayGenData record) {
  Payload payload;
  uint2 pixelID = DispatchRaysIndex().xy;
  uint2 fbSize = DispatchRaysDimensions().xy;
  float2 screen = (float2(pixelID) + float2(.5f, .5f)) / float2(fbSize);

  // Generate ray
  RayDesc rayDesc;
  rayDesc.Origin = pc.camera.pos;
  rayDesc.Direction = normalize(pc.camera.dir_00 + screen.x * pc.camera.dir_du + screen.y * pc.camera.dir_dv);
  rayDesc.TMin = 0.0;
  rayDesc.TMax = 1e38f;

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

  const int fbOfs = pixelID.x + fbSize.x * pixelID.y;
  record.frameBuffer[fbOfs] = gprt::make_bgra(payload.color);
}

[shader("closesthit")]
void closesthit(uniform TrianglesGeomData record, inout Payload payload) {
  // compute normal:
  uint primID = PrimitiveIndex();
  uint3 index = record.index[primID];
  float3 A = record.vertex[index.x];
  float3 B = record.vertex[index.y];
  float3 C = record.vertex[index.z];
  float3 Ng = normalize(cross(B - A, C - A));
  float3 rayDir = normalize(ObjectRayDirection());
  payload.color = (.2f + .8f * abs(dot(rayDir, Ng))) * record.color;
}

[shader("miss")]
void miss(inout Payload payload) {
  payload.color = float3(0.0);
}