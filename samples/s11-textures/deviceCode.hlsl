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

#include "sharedCode.h"

// Descriptor binding, then set number.
[[vk::binding(0, 0)]] Texture2D texture2ds[];
[[vk::binding(0, 0)]] SamplerState sampler2ds[];

// The first parameter here is the name of our entry point.
//
// The second is the type and name of the shader record. A shader record
// can be thought of as the parameters passed to this kernel.
GPRT_RAYGEN_PROGRAM(raygen, (RayGenData, record)) {
    uint2 dims = DispatchRaysDimensions().xy;
    uint2 pixelID = DispatchRaysIndex().xy;
    // float2 screen = (float2(pixelID) +
    //                  float2(.5f, .5f)) /
    //                 float2(dims);

    float2 screen = float2(pixelID) / float2(dims);

    Texture2D texture = texture2ds[0];
    SamplerState sampler = sampler2ds[0];

    float4 color = texture.SampleLevel(sampler, screen, 0);
    const int fbOfs = pixelID.x + dims.x * pixelID.y;
    gprt::store(record.framebuffer, fbOfs, gprt::make_bgra(color));

}
