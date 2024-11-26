#include "sharedCode.h"

[[vk::push_constant]] PushConstants pc;

struct BGVertexStageOutput {
  float4 position : SV_POSITION;
};

layout(binding = 0) ConstantBuffer<BackgroundData> bgRecord;

[shader("vertex")]
BGVertexStageOutput backgroundVertex(uint vertexID : SV_VertexID) {
  BGVertexStageOutput output;
  float3 position = gprt::load<float3>(bgRecord.vertex, vertexID);
  output.position = float4(position, 1.f);
  return output;
}

[shader("fragment")]
float4 backgroundPixel(float4 position : SV_POSITION) {
  int2 pixelID = int2(position.xy);
  int pattern = (pixelID.x / 32) ^ (pixelID.y / 32);
  float3 color = bool(pattern & 1) ? bgRecord.color1 : bgRecord.color0;
  return float4(color, 1.0f);
}

struct GuiVertexStageOutput {
  float4 position : SV_POSITION;
};

layout(binding = 0) ConstantBuffer<GUIData> guiRecord;

[shader("vertex")]
GuiVertexStageOutput GUIVertex(uint vertexID : SV_VertexID) {
  GuiVertexStageOutput output;
  float3 position = gprt::load<float3>(guiRecord.vertex, vertexID);
  output.position = float4(position, 1.f);
  return output;
}

[shader("fragment")]
float4 GUIPixel(float4 position : SV_POSITION) {
  float2 uv = position.xy / guiRecord.resolution;  
  Texture2D texture = gprt::getTexture2DHandle(guiRecord.texture);
  SamplerState sampler = gprt::getDefaultSampler();
  return pow(texture.Sample(sampler, uv), 2.2);
}

struct TriVertexStageOutput {
  float4 position : SV_POSITION;
  float2 barycentrics : TEXCOORD0;
};

layout(binding = 0) ConstantBuffer<TrianglesGeomData> triRecord;

[shader("vertex")]
TriVertexStageOutput simpleVertex(uint vertexID : SV_VertexID) {
  TriVertexStageOutput output;

  float4x4 view = pc.view;
  float4x4 proj = pc.proj;
  float4 position = float4(gprt::load<float3>(triRecord.vertex, vertexID), 1.f);
  position = mul(view, position);
  position = mul(proj, position);

  output.position = position;
  output.barycentrics = (((vertexID % 3) == 0) ? float2(0.f, 0.f)
                       : ((vertexID % 3) == 1) ? float2(1.f, 0.f)
                                               : float2(0.f, 1.f));
  return output;
}

[shader("fragment")]
float4 simplePixel(float2 bc : TEXCOORD0, uint32_t primID : SV_PrimitiveID) {
  int3 index = gprt::load<int3>(triRecord.index, primID);
  float3 C1 = gprt::load<float3>(triRecord.color, index.x);
  float3 C2 = gprt::load<float3>(triRecord.color, index.y);
  float3 C3 = gprt::load<float3>(triRecord.color, index.z);
  float3 C = C2 * bc.x + C3 * bc.y + C1 * (1.f - (bc.x + bc.y));
  return float4(C, 1.0f);
}
