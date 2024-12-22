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

struct TrianglesGeomData {
  float3 *vertex;
  uint3 *index;
};

struct BoundingBoxData {
  float3 *aabbs;
};

struct RayGenData {
  float4 *imageBuffer;
  gprt::Accel triangleTLAS;
  gprt::Accel obbAccel;
};

/* variables for the miss program */
struct MissProgData {
  float3 color0;
  float3 color1;
};

/* A small structure of constants that can change every frame without rebuilding the
  shader binding table. (must be 128 bytes or less) */
struct RTPushConstants {
  struct Camera {
    float3 pos;
    float3 dir_00;
    float3 dir_du;
    float3 dir_dv;
  } camera;
};

struct ComputeOBBConstants {
  int numIndices;
  float3 *vertices;
  uint3 *indices;
  float3 *eulRots;
  float3 *aabbs;
  gprt::Instance *instance;
  int numTrisToInclude;
};

struct CompositeGuiConstants {
  uint2 fbSize;
  float4 *imageBuffer;
  uint *frameBuffer;
  gprt::Texture guiTexture;
};
