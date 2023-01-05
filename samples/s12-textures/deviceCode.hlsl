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

struct Payload
{
    float3 color;
};

GPRT_RAYGEN_PROGRAM(raygen, (RayGenData, record))
{
    Payload payload;
    uint2 dims = DispatchRaysDimensions().xy;
    uint2 pixelID = DispatchRaysIndex().xy;
    float2 screen = (float2(pixelID) +
                     float2(.5f, .5f)) /
                    float2(dims);

    // Generate ray
    RayDesc rayDesc;
    rayDesc.Origin = record.camera.pos;
    rayDesc.Direction =
        normalize(record.camera.dir_00 + screen.x * record.camera.dir_du + screen.y * record.camera.dir_dv);
    rayDesc.TMin = 0.0;
    rayDesc.TMax = 1e20f;

    // Trace ray against surface
    RaytracingAccelerationStructure world = gprt::getAccelHandle(record.world);
    TraceRay(
        world,                 // the tree
        RAY_FLAG_FORCE_OPAQUE, // ray flags
        0xff,                  // instance inclusion mask
        0,                     // ray type
        1,                     // number of ray types
        0,                     // miss type
        rayDesc,               // the ray to trace
        payload                // the payload IO
    );

    const int fbOfs = pixelID.x + dims.x * pixelID.y;
    gprt::store(record.framebuffer, fbOfs, gprt::make_rgba(payload.color));
}

struct TriangleAttributes
{
    float2 bc;
};
 
GPRT_CLOSEST_HIT_PROGRAM(closesthit, (TrianglesGeomData, record), (Payload, payload), (TriangleAttributes, attributes))
{
    // compute normal:
    uint primID = PrimitiveIndex();
    int3 index = gprt::load<int3>(record.index, primID);
    float3 A = gprt::load<float3>(record.vertex, index.x);
    float3 B = gprt::load<float3>(record.vertex, index.y);
    float3 C = gprt::load<float3>(record.vertex, index.z);
    float3 Ng = normalize(cross(B - A, C - A));

    float2 TCA = gprt::load<float2>(record.texcoord, index.x);
    float2 TCB = gprt::load<float2>(record.texcoord, index.y);
    float2 TCC = gprt::load<float2>(record.texcoord, index.z);

    float2 TC = TCB * attributes.bc.x
              + TCC * attributes.bc.y
              + TCA * (1.f - (attributes.bc.x + attributes.bc.y));

    Texture2D texture = gprt::getTexture2DHandle(record.texture);
    SamplerState sampler = gprt::getSamplerHandle(record.sampler);

    // float4 color = texture.SampleLevel(sampler, TC, 0);
    float4 color = texture.SampleGrad(sampler, TC, float2(1.0, 1.0), float2(1.0, 1.0));

    float3 rayDir = normalize(ObjectRayDirection());
    payload.color = (.2f + .8f * abs(dot(rayDir, Ng))) * color.rgb;
}

GPRT_MISS_PROGRAM(miss, (MissProgData, record), (Payload, payload))
{
    uint2 pixelID = DispatchRaysIndex().xy;
    int pattern = (pixelID.x / 8) ^ (pixelID.y / 8);
    float3 color = (pattern & 1) ? record.color1 : record.color0;
    payload.color = color;
}
