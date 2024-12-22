// This file contains structures which we can use as parameters for our device code.

#include "gprt.h"

struct RayGenData {
  uint *frameBuffer;
  int firstMissIndex;
  int secondMissIndex;
};

struct MissProgData {
  int maxIterations;
};

// some fun helper functions
#if defined(__SLANG_COMPILER__) // <- causes code below to only be included in the device code.

float3 genSMPTETestPattern(float2 uv) {
  uv.y = 1.0 - uv.y;
  float r = uv.x * 7.0;
  float4 z, v = float4(0.075) + z, c = float4(0.0, 0.22, 0.35, 0.5);
  float3 C = ceil(r / float3(2.0, 4.0, 1.0)) - floor(ceil(r / float3(2.0, 4.0, 1.0)) / 2.0) * 2.0;
  if (uv.y > 0.33) {
    C *= 0.75;
  } else if (uv.y > 0.25) {
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
  return C;
}

#endif