// MIT License

// Copyright (c) 2022 Nathan V. Morrical

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "gprt.h"

/* variables available to all programs */

/* variables for the triangle mesh geometry */
struct TrianglesGeomData {
  /*! array/buffer of vertex indices */
  uint3 *index;   // vec3i*
  /*! array/buffer of vertex positions */
  float3 *vertex;   // vec3f *
  /*! array/buffer of vertex positions */
  float2 *texcoord;   // vec2f *
  /*! base color texture we use for the entire mesh */
  DescriptorHandle<Texture2D<float4>> texture;
  /*! an array of texture samplers to use */
  DescriptorHandle<SamplerState> samplers[12];
};

struct RayGenData {
  SurfaceAccelerationStructure world;
  DescriptorHandle<RWTexture2D<float4>> radiance;
  DescriptorHandle<RWTexture2D<float>> depth;
  DescriptorHandle<RWTexture2D<float2>> mvec;
  DescriptorHandle<RWTexture2D<float4>> diffAlb;
  DescriptorHandle<RWTexture2D<float4>> specAlb;
  DescriptorHandle<RWTexture2D<float4>> nrmRgh;

  int2 fbSize;

  struct Camera {
    float3 pos;
    float3 dir_00;
    float3 dir_du;
    float3 dir_dv;
    float fovy;
  };
  
  Camera currCamera;
  Camera prevCamera;

  float4x4 viewPrev;
  float4x4 viewCurr;
  float4x4 projPrev;
  float4x4 projCurr;

  float4x4 prevViewProj;

  float2 jitter;
  float2 prevJitter;
};

/* variables for the miss program */
struct MissProgData {
  float3 color0;
  float3 color1;
};

/* Constants that change each frame */
struct Constants {
  /*! the current time */
  float now;
  int frame;

  /*! array/buffer of instances to transform */
  gprt::Instance *prevInstances; // holds previous transform data
  gprt::Instance *instances;
  int numInstances;
};

struct RenderPassParams {
  int2 fbSize;
  DescriptorHandle<Texture2D<float4>> resolvedColor;
  uint32_t *frameBuffer;
};

#ifdef __SLANG_COMPILER__
float3x3 adjugate(in float3x4 m)
{
    return float3x3(cross(m[1].xyz, m[2].xyz),
                    cross(m[2].xyz, m[0].xyz),
                    cross(m[0].xyz, m[1].xyz));
};
#endif