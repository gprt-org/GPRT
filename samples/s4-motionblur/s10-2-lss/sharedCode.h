#include "gprt.h"

#define AA 3 // used for antialiasing

/* variables available to all programs */

/* variables for the line-swept sphere geometry */
struct LSSGeomData {
  float4 *vertices;
  uint2 *indices;
};

struct RayGenData {
  uint *frameBuffer;
  SurfaceAccelerationStructure world;
};

/* variables for the miss program */
struct MissProgData {
  float3 color0;
  float3 color1;
};

/* Constants that change each frame */
struct Constants {
  float time;
};

#if defined(__SLANG_COMPILER__) // <- causes code below to only be included in the device code.
float3 pattern(in float2 uv) {
  float3 col = float3(0.6);
  col += 0.4 * smoothstep(-0.01, 0.01, cos(uv.x * 0.5) * cos(uv.y * 0.5));
  col *= smoothstep(-1.0, -0.98, cos(uv.x)) * smoothstep(-1.0, -0.98, cos(uv.y));
  return col;
}
#endif
