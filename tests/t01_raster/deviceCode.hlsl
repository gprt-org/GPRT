#include "sharedCode.h"

#ifdef VERTEX

struct VSOutput {
  float4 Pos : SV_POSITION;
  [[vk::location(0)]]
  float3 Color : COLOR0;
};

GPRT_VERTEX_PROGRAM(simpleVertex, (VertexData, record), vertexID, VSOutput) {
  float2 positions[3] = { float2(0.0, -0.5), float2(0.5, 0.5), float2(-0.5, 0.5) };
  float3 colors[3] = { float3(1.0, 0.0, 0.0), float3(0.0, 1.0, 0.0), float3(0.0, 0.0, 1.0) };
  VSOutput output = (VSOutput) 0;
  output.Color = colors[vertexID];
  output.Pos = float4(positions[vertexID], 0.f, 1.f);
  return output;
}
#endif

#ifdef PIXEL
GPRT_PIXEL_PROGRAM(simplePixel, (PixelData, record), interpolatedColor) { return float4(interpolatedColor, 1.0f); }
#endif

GPRT_RAYGEN_PROGRAM(simpleRayGen, (RayGenData, record)) {}
