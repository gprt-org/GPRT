// MIT License

// Copyright (c) 2022 Nathan V. Morrical

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "sharedCode.h"

[[vk::push_constant]]
PushConstants pc;

struct Payload {
  float3 diffuseColor;
  float3 specularColor;
  float3 normal;
  float roughness;
  float3 pos;
  float3 posPrev;
  float hitT;
};

float3x3
AngleAxis3x3(float angle, float3 axis) {
  float c, s;
  sincos(angle, s, c);

  float t = 1 - c;
  float x = axis.x;
  float y = axis.y;
  float z = axis.z;

  return float3x3(t * x * x + c, t * x * y - s * z, t * x * z + s * y, t * x * y + s * z, t * y * y + c,
                  t * y * z - s * x, t * x * z - s * y, t * y * z + s * x, t * z * z + c);
}

[shader("compute")]
[numthreads(1, 1, 1)]
void
Transform(uint3 DispatchThreadID: SV_DispatchThreadID) {
  // This kernel animates the texture planes, rotating them in a circle
  // and placing them in a grid.
  int instanceID = DispatchThreadID.x;

  int transformX = instanceID / 2;
  int transformY = instanceID % 2;

  int numInstances = pc.numInstances;
  float angle = 0.f;//pc.now;
  float3x3 aa = AngleAxis3x3(angle, float3(0.0, 1.0, 0.0));

  gprt::Instance instance = pc.instances[instanceID];  
  pc.prevInstances[instanceID] = instance; // Copy previous instance

  instance.transform = float3x4(float4(aa[0], lerp(-5, 5, transformX / float((numInstances / 2) - 1))),
                                float4(aa[1], lerp(-1, 1, transformY)), float4(aa[2], 0.0));
  pc.instances[instanceID] = instance;
}

[shader("raygeneration")]
void
raygen(uniform RayGenData record) {
  Payload payload;
  uint2 dims = DispatchRaysDimensions().xy;
  uint2 pixelID = DispatchRaysIndex().xy;
    
  // Generate ray
  RayDesc rayDesc;
  rayDesc.Origin = record.currCamera.pos;
  float2 currUV = ((float2(pixelID) + 0.5) + record.jitter) / float2(dims);
  float2 currUV_nj   = currUV - record.jitter / float2(dims);   // no-jitter
  rayDesc.Direction = normalize(record.currCamera.dir_00 +
                              currUV.x * record.currCamera.dir_du +
                              currUV.y * record.currCamera.dir_dv);
                              
  rayDesc.TMin = 0.0;
  rayDesc.TMax = 1e38f;  
  payload.hitT = 1e38f;

  // Trace ray against surface
  TraceRay(record.world,            // the tree
           RAY_FLAG_FORCE_OPAQUE,   // ray flags
           0xff,                    // instance inclusion mask
           0,                       // ray type
           0,                       // miss type
           rayDesc,                 // the ray to trace
           payload                  // the payload IO
  );


  // ------------- previous-frame clip coordinates -------------
  float4 clipPrev = mul(float4(payload.posPrev, 1.0), record.prevViewProj);
  float  rcpWprev = rcp(clipPrev.w);

  // NDC  (-1…+1, +Y up)  →  UV  (0…1, +Y down)
  float2 prevUV = float2( clipPrev.x, -clipPrev.y ) * 0.5 * rcpWprev + 0.5;
  prevUV            -= record.prevJitter / float2(dims);        // no-jitter

  float2 motionVecPix = (prevUV - currUV) * float2(dims);      // perfectly uniform across screen

  if (payload.hitT == 65504.f)  motionVecPix = float2(0.f, 0.f); // no motion for background
  record.mvec[pixelID] = motionVecPix;        


  float znear = 0.01f; // near plane
  // float zfar = 1000.f; // far plane

  // float2 deJitteredCurr = currUV - record.jitter / float2(dims);

  float3 v_curr = payload.pos - record.currCamera.pos;
  // float3 v_prev = payload.posPrev - record.prevCamera.pos;

  float zViewCurr = dot(v_curr, record.currCamera.dir_00);   // same invDepth_prev denom
  // float zViewPrev = dot(v_prev, record.prevCamera.dir_00);   // same invDepth_prev denom
  float depthCurr = saturate( znear / zViewCurr );     // 1/z, mapped to 0…1
  // float depthPrev = saturate( znear / zViewPrev );     // 1/z, mapped to 0…1


  // float  invDepth_prev = rcp(zViewPrev);//1.0 / dot(v_prev, record.prevCamera.dir_00);

  // float2 prevSample = float2(0.f);
  // prevSample.x = dot(v_prev, record.prevCamera.dir_du) * invDepth_prev;
  // prevSample.y = dot(v_prev, record.prevCamera.dir_dv) * invDepth_prev;
  // prevSample   -= record.prevJitter / float2(dims);

  // float2 motionVec = prevSample - deJitteredCurr;
  // record.mvec[pixelID] = motionVec;//motionVec * .5 + .5;   // screen-space ∈ (-1…1)

  record.radiance[pixelID] = float4(payload.diffuseColor, 1.0);
  record.diffAlb[pixelID] = float4(payload.diffuseColor, 1.0);
  record.specAlb[pixelID] = float4(payload.specularColor, 1.0);

  record.depth[pixelID] = depthCurr;     // 1/z, mapped to 0…1
  record.nrmRgh[pixelID] = float4(payload.normal, payload.roughness);
}

[shader("closesthit")]
void
closesthit(uniform TrianglesGeomData record, inout Payload payload, float2 bc) {
  // compute normal:
  uint instanceID = InstanceIndex();
  uint primID = PrimitiveIndex();
  uint3 index = record.index[primID];
  float3 A = record.vertex[index.x];
  float3 B = record.vertex[index.y];
  float3 C = record.vertex[index.z];
  
  gprt::Instance prevInstance = pc.prevInstances[instanceID];
  gprt::Instance currInstance = pc.instances[instanceID];
  float3 P = A * bc.x + B * bc.y + C * (1.f - (bc.x + bc.y));
  payload.posPrev = mul(prevInstance.transform, float4(P, 1.f)).xyz;
  payload.pos = mul(currInstance.transform, float4(P, 1.f)).xyz;
  
  float3 Ng = normalize(cross(B - A, C - A));

  // compute texture coordinate:
  float2 TCA = record.texcoord[index.x];
  float2 TCB = record.texcoord[index.y];
  float2 TCC = record.texcoord[index.z];
  float2 TC = TCB * bc.x + TCC * bc.y + TCA * (1.f - (bc.x + bc.y));

  // fetch the texture and sampler for this plane
  DescriptorHandle<Texture2D<float4>> texture = record.texture;
  DescriptorHandle<SamplerState> sampler = record.samplers[0];

  // For now, we're using an approximate ray cone method to drive texture sampling.
  // The cone footprint grows larger with distance, and is grows larger at glancing angles
  float footprint = 0.f; // Set the footprint to 0 to cause aliasing.
  float2 DDX = float2(footprint, footprint) * .5f;
  float2 DDY = float2(footprint, footprint) * .5f;
  float4 color = float4(1.0, 0.0, 1.0, 1.0);

  // Here's how you'd normally sample a texture
  color = texture.SampleGrad(sampler, TC*(InstanceIndex() + 1.f), DDX, DDY);

  payload.diffuseColor = color.rgb;
  payload.specularColor = float3(0.f, 0.f, 0.f);

  // Transform normal from object space to world space using the adjugate of the instance transform. 
  float3x3 adj = adjugate(currInstance.transform);
  Ng = normalize(mul(adj, Ng));

  payload.normal = 0.5 * Ng + .5;
  payload.roughness = 1.f;

  payload.hitT = RayTCurrent();
}

// A background with a vignette effect.
[shader("miss")]
void
miss(uniform MissProgData record, inout Payload payload) {
  float2 resolution = DispatchRaysDimensions().xy;
  float2 fragCoord = DispatchRaysIndex().xy;
  float2 p = (-resolution.xy + 2.0 * fragCoord) / resolution.y;
  float3 col = float3(0.08) * (1.0 - 0.3 * length(p)) + 0.02 * WorldRayDirection().y;
  payload.diffuseColor = col;
  payload.specularColor = float3(0.f, 0.f, 0.f); // no specular

  // Assume the background is a flat surface facing the camera.
  payload.normal = 0.5 * normalize(-WorldRayDirection()) + .5;
  payload.roughness = 1.f;
  payload.hitT = 65504.f; // Set a large hitT to indicate no hit.

  payload.pos = WorldRayOrigin() + WorldRayDirection() * payload.hitT;
  payload.posPrev = payload.pos; // No motion for the background
}


[shader("compute")]
[numthreads(1, 1, 1)]
void
RenderPass(uint3 DispatchThreadID: SV_DispatchThreadID, uniform RenderPassParams pc) {
  int2 pixelID = DispatchThreadID.xy;
  if (pixelID.x >= pc.fbSize.x || pixelID.y >= pc.fbSize.y) {
    return;
  }

  const int fbOfs = pixelID.x + pc.fbSize.x * pixelID.y;
  pc.frameBuffer[fbOfs] = gprt::make_bgra(pc.resolvedColor[pixelID]);

  // pc.frameBuffer[fbOfs] = gprt::make_bgra(pc.debugBuffer[fbOfs]);
}
