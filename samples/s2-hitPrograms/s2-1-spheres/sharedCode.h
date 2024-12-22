#include "gprt.h"

#define AA 3 // used for antialiasing

/* variables available to all programs */

/* variables for the sphere geometry */
struct SphereGeomData {
  float4 *posAndRadius;
};

struct RayGenData {
  uint *frameBuffer;
  gprt::Accel world;
};

/* variables for the miss program */
struct MissProgData {
  float3 color0;
  float3 color1;
};

/* Constants that change each frame */
struct PushConstants {
  float time;
};

#if defined(__SLANG_COMPILER__) // <- causes code below to only be included in the device code.
float3
hsv2rgb(float3 input) {
  float4 K = float4(1.0, 2.0 / 3.0, 1.0 / 3.0, 3.0);
  float3 p = abs(frac(input.xxx + K.xyz) * 6.0 - K.www);
  return input.z * lerp(K.xxx, clamp(p - K.xxx, 0.0, 1.0), input.y);
}

float3
pattern(in float2 uv, in float3 color) {
  float3 col = color * .6;
  col += 0.4 * smoothstep(-0.01, 0.01, cos(uv.x * 0.5) * cos(uv.y * 0.5));
  col *= smoothstep(-1.0, -0.98, cos(uv.x)) * smoothstep(-1.0, -0.98, cos(uv.y));
  return col;
}
#endif