// For convenience, we can include multiple GPU entry points in the same file.

#include "sharedCode.h"

[shader("raygeneration")]
void firstRayGen(uniform RayGenData record) {
  uint2 threadID = DispatchRaysIndex().xy;
  uint2 dims = DispatchRaysDimensions().xy;
  uint2 pixelID = threadID + uint2(0, 0); // First, have these threads write to the left of the image...
  uint2 fbSize = uint2(dims.x * 2, dims.y);

  if (threadID.x == 0 && threadID.y == 0) printf("Hello from first raygen!\n");

  // Generate a simple checkerboard pattern as a test.
  int pattern = (pixelID.x / 32) ^ (pixelID.y / 32);
  float3 color = bool(pattern & 1) ? record.color1 : record.color0;

  int fbOfs = pixelID.x + fbSize.x * pixelID.y;
  record.frameBuffer[fbOfs] = gprt::make_bgra(color);
}

[shader("raygeneration")]
void secondRayGen(uniform RayGenData record) {
  uint2 threadID = DispatchRaysIndex().xy;
  uint2 dims = DispatchRaysDimensions().xy;
  uint2 pixelID = threadID + uint2(dims.x, 0);   // ...Then have these threads write to the right of the image
  uint2 fbSize = uint2(dims.x * 2, dims.y);

  if (threadID.x == 0 && threadID.y == 0) printf("Hello from second raygen!\n");

  // Generate the SMPTE test pattern
  float2 u = float2(threadID) / float2(dims);
  u.y = 1.0 - u.y;
  float r = u.x * 7.0;
  float4 z, v = float4(0.075) + z, c = float4(0.0, 0.22, 0.35, 0.5);
  float3 C = ceil(r / float3(2.0, 4.0, 1.0)) - floor(ceil(r / float3(2.0, 4.0, 1.0)) / 2.0) * 2.0;
  if (u.y > 0.33) {
    C *= 0.75;
  } else if (u.y > 0.25) {
    C = float3(1.0 - C.xy, C.z) * 0.75 * C.z;
  } else if (r < 1.25) {
    C = c.xyz;
  } else if (r < 2.5) {
    C = float3(1.0);
  } else if (r < 3.75) {
    C = c.yxw;
  } else if (r < 5.0) {
    C = v.xyz;
  } else if (r < 5.33) {
    C = z.xyz;
  } else if (r < 6.0 && r > 5.67) {
    C = z.xyz + 0.15;
  } else {
    C = v.xyz;
  }

  int fbOfs = pixelID.x + fbSize.x * pixelID.y;
  record.frameBuffer[fbOfs] = gprt::make_bgra(C);
}