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

// note! HLSL aligns to float4 boundaries!
struct TrianglesGeomData {
  /*! array/buffer of vertex indices */
  alignas(16) gprt::Buffer index;    // int3*
  alignas(16) gprt::Buffer vertex;   // float3*
  alignas(16) gprt::Buffer color;    // float3*
};

/* variables for the geometry representing the background */
struct BackgroundData {
  alignas(16) gprt::Buffer index;    // int3*
  alignas(16) gprt::Buffer vertex;   // float3*
  alignas(16) float3 color0;
  alignas(16) float3 color1;
};

/* variables for the geometry representing the background */
struct GUIData {
  alignas(16) gprt::Buffer index;    // int3*
  alignas(16) gprt::Buffer vertex;   // float3*
  alignas(16) gprt::Texture texture;
  alignas(8) float2 resolution;
};

/* Constants that change each frame */
struct PushConstants {
  alignas(16) float4x4 view;
  alignas(16) float4x4 proj;
};