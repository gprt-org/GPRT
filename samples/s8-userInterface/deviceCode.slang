#include "sharedCode.h"

struct Payload {
  float3 color;
};

float4
over(float4 a, float4 b) {
  float4 result;
  result.a = a.a + b.a * (1.f - a.a);
  if (result.a == 0.f)
    return a;   // avoid NaN
  result.rgb = (a.rgb * a.a + b.rgb * b.a * (1.f - a.a)) / result.a;
  return result;
}

[shader("compute")]
[numthreads(1, 1, 1)]
void CompositeGui(uint3 DispatchThreadID: SV_DispatchThreadID, PushConstant<CompositeGuiConstants> pc) {
  int2 pixelID = DispatchThreadID.xy;
  const int fbOfs = pixelID.x + pc.fbSize.x * pixelID.y;
  float2 fragCoord = pixelID + float2(.5f, .5f);
  float2 uv = (fragCoord) / float2(pc.fbSize);

  // Load color of the rendered image
  float4 imageColor = pc.imageBuffer[fbOfs];

  // Sample the color from the GUI texture
  SamplerState sampler = gprt::getDefaultSampler();
  DescriptorHandle<Texture2D> guiTexture = pc.guiTexture;
  float4 guiColor = guiTexture.SampleGrad(sampler, uv, float2(0.f, 0.f), float2(0.f, 0.f));

  // Gamma correction
  guiColor.rgb = gprt::srgb_to_linear(guiColor.rgb);

  // Composite the GUI on top of the scene
  float4 pixelColor = over(guiColor, imageColor);
  pc.frameBuffer[fbOfs] = gprt::make_bgra(pixelColor);
}

[shader("raygeneration")]
void raygen(ShaderRecord<RayGenData> record, PushConstant<RTPushConstants> pc) {
  Payload payload;
  uint2 pixelID = DispatchRaysIndex().xy;
  uint2 fbSize = DispatchRaysDimensions().xy;
  float2 screen = (float2(pixelID) + float2(.5f, .5f)) / float2(fbSize);

  RayDesc rayDesc;
  rayDesc.Origin = pc.camera.pos;
  rayDesc.Direction = normalize(pc.camera.dir_00 + screen.x * pc.camera.dir_du + screen.y * pc.camera.dir_dv);
  rayDesc.TMin = 0.001;
  rayDesc.TMax = 10000.0;
  TraceRay(record.world,            // the tree
           RAY_FLAG_FORCE_OPAQUE,   // ray flags
           0xff,                    // instance inclusion mask
           0,                       // ray type
           0,                       // miss type
           rayDesc,                 // the ray to trace
           payload                  // the payload IO
  );

  const int fbOfs = pixelID.x + fbSize.x * pixelID.y;
  record.imageBuffer[fbOfs] = float4(payload.color, 1.0f);
}

[shader("closesthit")]
void closesthit(ShaderRecord<TrianglesGeomData> record, PayloadState<Payload> payload, HitAttribute<float2> bc) {
  payload.color = float3(bc.x, bc.y, 1.0 - (bc.x + bc.y));
}

// A background with a vignette effect.
[shader("miss")]
void miss(ShaderRecord<MissProgData> record, PayloadState<Payload> payload) {
  float2 resolution = DispatchRaysDimensions().xy;
  float2 fragCoord = DispatchRaysIndex().xy;
  float2 p = (-resolution.xy + 2.0 * fragCoord) / resolution.y;
  float3 col = float3(0.08) * (1.0 - 0.3 * length(p)) + 0.02 * WorldRayDirection().y;
  payload.color = col;
}
