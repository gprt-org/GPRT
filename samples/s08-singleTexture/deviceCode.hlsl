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

struct Payload {
  float spreadAngle;
  float3 color;
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

GPRT_COMPUTE_PROGRAM(Transform, (TransformData, record)) {
  // This kernel animates the texture planes, rotating them in a circle
  // and placing them in a grid.
  int transformID = DispatchThreadID.x;

  int transformX = transformID / 2;
  int transformY = transformID % 2;

  int numTransforms = record.numTransforms;
  float angle = record.now;
  float3x3 aa = AngleAxis3x3(angle, float3(0.0, 1.0, 0.0));

  float4 transforma = float4(aa[0], lerp(-5, 5, transformX / float((numTransforms / 2) - 1)));
  float4 transformb = float4(aa[1], lerp(-1, 1, transformY));
  float4 transformc = float4(aa[2], 0.0);
  float4 transformd = float4(0.0, 0.0, 0.0, 1.0);

  gprt::store(record.transforms, transformID * 4 + 0, transforma);
  gprt::store(record.transforms, transformID * 4 + 1, transformb);
  gprt::store(record.transforms, transformID * 4 + 2, transformc);
  gprt::store(record.transforms, transformID * 4 + 3, transformd);
}

GPRT_RAYGEN_PROGRAM(raygen, (RayGenData, record)) {
  Payload payload;
  uint2 dims = DispatchRaysDimensions().xy;
  uint2 pixelID = DispatchRaysIndex().xy;
  float2 screen = (float2(pixelID) + float2(.5f, .5f)) / float2(dims);

  // Generate ray
  RayDesc rayDesc;
  rayDesc.Origin = record.camera.pos;
  rayDesc.Direction =
      normalize(record.camera.dir_00 + screen.x * record.camera.dir_du + screen.y * record.camera.dir_dv);
  rayDesc.TMin = 0.0;
  rayDesc.TMax = 1e20f;

  // compute the spreading angle of the ray to determine the sampling footprint
  float phi = record.camera.fovy;
  float H = dims.y;
  payload.spreadAngle = atan(2.f * tan(phi / 2.f) / H);

  // Trace ray against surface
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

  const int fbOfs = pixelID.x + dims.x * pixelID.y;
  gprt::store(record.framebuffer, fbOfs, gprt::make_bgra(payload.color));
}

struct TriangleAttributes {
  float2 bc;
};

GPRT_CLOSEST_HIT_PROGRAM(closesthit, (TrianglesGeomData, record), (Payload, payload),
                         (TriangleAttributes, attributes)) {
  // compute normal:
  uint instanceID = InstanceIndex();
  uint primID = PrimitiveIndex();
  int3 index = gprt::load<int3>(record.index, primID);
  float3 A = gprt::load<float3>(record.vertex, index.x);
  float3 B = gprt::load<float3>(record.vertex, index.y);
  float3 C = gprt::load<float3>(record.vertex, index.z);
  float3 Ng = normalize(cross(B - A, C - A));

  // compute texture coordinate:
  float2 TCA = gprt::load<float2>(record.texcoord, index.x);
  float2 TCB = gprt::load<float2>(record.texcoord, index.y);
  float2 TCC = gprt::load<float2>(record.texcoord, index.z);
  float2 TC = TCB * attributes.bc.x + TCC * attributes.bc.y + TCA * (1.f - (attributes.bc.x + attributes.bc.y));

  // fetch the texture and sampler for this plane
  Texture2D texture = gprt::getTexture2DHandle(record.texture);
  SamplerState sampler = gprt::getSamplerHandle(record.samplers[instanceID]);

  // For now, we're using an approximate ray cone method to drive texture sampling.
  // The cone footprint grows larger with distance, and is grows larger at glancing angles
  float footprint = payload.spreadAngle * RayTCurrent() / (dot(Ng, normalize(ObjectRayDirection())));
  float2 DDX = float2(footprint, footprint) * .5f;
  float2 DDY = float2(footprint, footprint) * .5f;
  float4 color;

  // Here's how you'd normally sample a texture
  if (instanceID == 0) {
    color = texture.SampleGrad(sampler, TC, DDX, DDY);
  }
  // for the second texture, we want to demonstrate that mipmapping works.
  // We can use SampleLevel instead of SampleGrad to do this.
  else if (instanceID == 1) {
    float w, h;
    texture.GetDimensions(w, h);
    int levels = log2(max(w, h));
    color = texture.SampleLevel(sampler, TC, lerp(levels, 0, abs(sin(record.now))));
  }

  // For the 3rd and 4th textures, we want to zoom in to show the magnification filter.
  // We also want to force mip level 0.
  else if (instanceID == 2 || instanceID == 3) {
    TC = TC * .05 + .475;
    DDX *= .05;
    DDY *= .05;
    color = texture.SampleLevel(sampler, TC, 0);
  }
  // For the 5th and 6th textures, we want to zoom out and show how the
  // min filter blends when many texels fall within the sample footprint.
  else if (instanceID == 4 || instanceID == 5) {
    TC *= 10;
    DDX *= 10;
    DDY *= 10;
    color = texture.SampleGrad(sampler, TC, DDX, DDY);
  }

  // For the 7th and 8th textures, we also want to zoom out, now to show
  // the effect of anisotropic sampling. Higher the anisotroic samples, the
  // less blurry the image should look
  else if (instanceID == 6 || instanceID == 7) {
    TC *= 5;
    DDX *= 5;
    DDY *= 5;
    color = texture.SampleGrad(sampler, TC, DDX, DDY);
  }

  // For the last set of textures, we want to show wrapping and border colors,
  // so we zoom out again.
  else if (instanceID == 8 || instanceID == 9 || instanceID == 10 || instanceID == 11) {
    TC = TC * 2;
    color = texture.SampleGrad(sampler, TC, DDX, DDY);
  }

  payload.color = color.rgb;
}

GPRT_MISS_PROGRAM(miss, (MissProgData, record), (Payload, payload)) {
  uint2 pixelID = DispatchRaysIndex().xy;
  int pattern = (pixelID.x / 8) ^ (pixelID.y / 8);
  float3 color = (pattern & 1) ? record.color1 : record.color0;
  payload.color = color;
}
