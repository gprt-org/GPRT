#include "sharedCode.h"

struct Payload {
  float3 color;
};

float4 over(float4 a, float4 b) {
  float4 result;
  result.a = a.a + b.a * (1.f - a.a);
  if (result.a == 0.f)
    return a; // avoid NaN
  result.rgb = (a.rgb * a.a + b.rgb * b.a * (1.f - a.a)) / result.a;
  return result;
}

[shader("compute")] 
[numthreads(1,1,1)]
void CompositeGui(uint3 DispatchThreadID : SV_DispatchThreadID, uniform CompositeGuiConstants pc)
{
  int2 pixelID = DispatchThreadID.xy;
  const int fbOfs = pixelID.x + pc.fbSize.x * pixelID.y;
  float2 fragCoord = pixelID + float2(.5f, .5f);
  float2 uv = (fragCoord) / float2(pc.fbSize);

  // Load color of the rendered image
  float4 imageColor = gprt::load<float4>(pc.imageBuffer, fbOfs);

  // Gamma correction
  imageColor = pow(imageColor, 1.f / 2.2f);

  // Sample the color from the GUI texture
  SamplerState sampler = gprt::getDefaultSampler();
  Texture2D guiTexture = gprt::getTexture2DHandle(pc.guiTexture);
  float4 guiColor = guiTexture.SampleGrad(sampler, uv, float2(0.f, 0.f), float2(0.f, 0.f));
  
  // Composite the GUI on top of the scene
  float4 pixelColor = over(guiColor, imageColor);
  gprt::store(pc.frameBuffer, fbOfs, gprt::make_bgra(pixelColor));
}

[[vk::push_constant]] RTPushConstants pc;

[shader("raygeneration")] 
void raygen(uniform RayGenData record) {
  Payload payload;
  uint2 pixelID = DispatchRaysIndex().xy;
  uint2 fbSize = DispatchRaysDimensions().xy;
  float2 screen = (float2(pixelID) + float2(.5f, .5f)) / float2(fbSize);

  RayDesc rayDesc;
  rayDesc.Origin = pc.camera.pos;
  rayDesc.Direction =
      normalize(pc.camera.dir_00 + screen.x * pc.camera.dir_du + screen.y * pc.camera.dir_dv);
  rayDesc.TMin = 0.001;
  rayDesc.TMax = 10000.0;
  RaytracingAccelerationStructure world = gprt::getAccelHandle(record.world);
  TraceRay(world,                   // the tree
           RAY_FLAG_FORCE_OPAQUE,   // ray flags
           0xff,                    // instance inclusion mask
           0,                       // ray type
           1,                       // number of ray types
           0,                       // miss type
           rayDesc,                 // the ray to trace
           payload                  // the payload IO
  );

  const int fbOfs = pixelID.x + fbSize.x * pixelID.y;
  gprt::store(record.imageBuffer, fbOfs, float4(payload.color, 1.0f));
}

[shader("closesthit")] 
void closesthit(uniform TrianglesGeomData record, inout Payload payload, in float2 bc) {
  payload.color = float3(bc.x, bc.y, 1.0 - (bc.x + bc.y));
}

[shader("miss")] 
void miss(uniform MissProgData record, inout Payload payload) {
  uint2 pixelID = DispatchRaysIndex().xy;
  int pattern = (pixelID.x / 32) ^ (pixelID.y / 32);
  payload.color = bool(pattern & 1) ? record.color1 : record.color0;
}

