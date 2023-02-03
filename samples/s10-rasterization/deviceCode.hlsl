#include "sharedCode.h"

GPRT_VERTEX_PROGRAM(backgroundVertex, (BackgroundData, record)) {
  uint32_t vertexID = VertexIndex();
  float3 position = gprt::load<float3>(record.vertex, vertexID);
  return float4(position, 1.f);
}

GPRT_PIXEL_PROGRAM(backgroundPixel, (BackgroundData, record)) {
  int2 pixelID = Position().xy;
  int pattern = (pixelID.x / 32) ^ (pixelID.y / 32);
  float3 color = (pattern & 1) ? record.color1 : record.color0;
  return float4(color, 1.0f);
}

GPRT_VERTEX_PROGRAM(simpleVertex, (TrianglesGeomData, record)) {
  uint32_t vertexID = VertexIndex();
  float4x4 view = record.view;
  float4x4 proj = record.proj;
  float4 position = float4(gprt::load<float3>(record.vertex, vertexID), 1.f);
  position = mul(view, position);
  position = mul(proj, position);
  return position;
}

GPRT_PIXEL_PROGRAM(simplePixel, (TrianglesGeomData, record)) {
  float2 bc = Barycentrics();
  uint32_t primID = TriangleIndex();
  int3 index = gprt::load<int3>(record.index, primID);
  float3 C1 = gprt::load<float3>(record.color, index.x);
  float3 C2 = gprt::load<float3>(record.color, index.y);
  float3 C3 = gprt::load<float3>(record.color, index.z);
  float3 C = C2 * bc.x + C3 * bc.y + C1 * (1.f - (bc.x + bc.y));
  return float4(C, 1.0f);
}
