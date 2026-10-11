#include "gprt.slang"

struct Parameters {
  uint64_t world;
  float *result;
};

[shader("raygeneration")]
void trace(uniform Parameters record) {
  uint64_t address = record.world;
  RaytracingAccelerationStructure world = spirv_asm {
    result: $$RaytracingAccelerationStructure = OpConvertUToAccelerationStructureKHR $address
  };
  RayDesc ray;
  ray.Origin = float3(0, 0, -1);
  ray.Direction = float3(0, 0, 1);
  ray.TMin = 0;
  ray.TMax = 100;
  float distance = -1;
  TraceRay(world, RAY_FLAG_FORCE_OPAQUE, 0xff, 0, 0, 0, ray, distance);
  record.result[0] = distance;
}

[shader("closesthit")]
void hit(inout float distance, in float2 barycentrics : SV_IntersectionAttributes) {
  distance = RayTCurrent();
}

[shader("miss")]
void miss(inout float distance) {
  distance = -1;
}
