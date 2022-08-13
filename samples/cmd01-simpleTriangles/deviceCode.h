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

#include "vkrt.h"

/* variables for the triangle mesh geometry */
struct TrianglesGeomData
{
  /*! base color we use for the entire mesh */
  float3 color; float pad;
  /*! array/buffer of vertex indices */
  uint64_t index; // vec3i*
  /*! array/buffer of vertex positions */
  uint64_t vertex; // vec3f *
};

struct RayGenData
{
  uint64_t fbPtr;
  int2 fbSize;
  // OptixTraversableHandle world;
  uint64_t world; // RaytracingAccelerationStructure*

  struct { 
    float3 pos;    float pad1;
    float3 dir_00; float pad2;
    float3 dir_du; float pad3;
    float3 dir_dv; float pad4;
  } camera;
};

/* variables for the miss program */
struct MissProgData
{
  float3  color0;  float pad1;
  float3  color1;  float pad2;
};
