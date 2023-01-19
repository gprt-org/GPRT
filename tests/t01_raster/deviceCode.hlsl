#include "sharedCode.h"

#ifdef VERTEX

GPRT_VERTEX_PROGRAM(simpleVertex, (VertexData, record)) {
  uint32_t vertexID = VertexIndex();
  float2 positions[3] = { float2(0.0, -0.5), float2(0.5, 0.5), float2(-0.5, 0.5) };
  return float4(positions[vertexID], 0.f, 1.f);
}
#endif

#ifdef PIXEL
GPRT_PIXEL_PROGRAM(simplePixel, (PixelData, record)) {
  float2 bc = Barycentrics();
  return float4(bc.x, bc.y, 1.0 - (bc.x + bc.y), 1.0f);
}
#endif

GPRT_RAYGEN_PROGRAM(simpleRayGen, (RayGenData, record)) {}
