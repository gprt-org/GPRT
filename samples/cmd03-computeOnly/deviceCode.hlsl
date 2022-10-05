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

#include "gprt.h"
#include "deviceCode.h"

// The first parameter here is the name of our entry point.
//
// The second is the type of the shader record.
GPRT_COMPUTE_PROGRAM(simpleCompute, (ComputeData, record))
{
  uint2 pixelID = DispatchThreadID.xy;

  if (pixelID.x == 0 && pixelID.y == 0) {
    printf("Hello from your first compute program!\n");
  }

  // Generate a simple checkerboard pattern as a test. 
  int pattern = (pixelID.x / 8) ^ (pixelID.y / 8);
  // alternate pattern, showing that pixel (0,0) is in the upper left corner
  // pattern = (pixelID.x*pixelID.x + pixelID.y*pixelID.y) / 100000;
  const float3 color = (pattern & 1) ? record.color1 : record.color0;

  // find the frame buffer location (x + width*y) and put the result there
  const int fbOfs = pixelID.x + record.fbSize.x * pixelID.y;
  vk::RawBufferStore<uint32_t>(
    record.fbPtr + fbOfs * sizeof(uint32_t), 
    gprt::make_rgba(color));
}
