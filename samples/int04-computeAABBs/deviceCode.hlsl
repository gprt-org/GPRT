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

#include "deviceCode.h"
#include "gprt.h"

struct Payload
{
  float3 color;
};

GPRT_RAYGEN_PROGRAM(AABBRayGen, (RayGenData, record))
{
  Payload payload;
  uint2 pixelID = DispatchRaysIndex().xy;
  float2 screen = (float2(pixelID) + 
                  float2(.5f, .5f)) / float2(record.fbSize);

  RayDesc rayDesc;
  rayDesc.Origin = record.camera.pos;
  rayDesc.Direction = 
    normalize(record.camera.dir_00
    + screen.x * record.camera.dir_du
    + screen.y * record.camera.dir_dv
  );
  rayDesc.TMin = 0.0;
  rayDesc.TMax = 10000.0;
  RaytracingAccelerationStructure world = gprt::getAccelHandle(record.world);
  TraceRay(
    world, // the tree
    RAY_FLAG_FORCE_OPAQUE, // ray flags
    0xff, // instance inclusion mask
    0, // ray type
    1, // number of ray types
    0, // miss type
    rayDesc, // the ray to trace
    payload // the payload IO
  );

  const int fbOfs = pixelID.x + record.fbSize.x * pixelID.y;
  gprt::store(record.fbPtr, fbOfs, gprt::make_rgba(payload.color));
}


struct Attribute
{
  float radius;
  float3 position;
};

GPRT_CLOSEST_HIT_PROGRAM(AABBClosestHit, (AABBGeomData, record), (Payload, payload), (Attribute, attribute))
{
  float3 origin = attribute.position;
  float3 hitPos = ObjectRayOrigin() + RayTCurrent() * ObjectRayDirection();

  float3 normal = normalize(hitPos - origin);
  
  // printf("TEST\n");
  payload.color = normal;//float3(1.f, 1.f, 1.f); //geomSBTData.color;
}


GPRT_COMPUTE_PROGRAM(AABBBounds, (AABBBoundsData, record))
{
  int primID = DispatchThreadID.x;
  float3 position = gprt::load<float3>(record.vertex, primID);
  float radius = gprt::load<float>(record.radius, primID);
  float3 aabbMin = position - float3(radius, radius, radius);
  float3 aabbMax = position + float3(radius, radius, radius);
  gprt::store(record.aabbs, 2 * primID, aabbMin);
  gprt::store(record.aabbs, 2 * primID + 1, aabbMax);
}

GPRT_INTERSECTION_PROGRAM(AABBIntersection, (AABBGeomData, record))
{
  uint primID = PrimitiveIndex();
  float3 position = gprt::load<float3>(record.vertex, primID);
  float radius = gprt::load<float>(record.radius, primID);

  float3 ro = ObjectRayOrigin();
  float3 rd = ObjectRayDirection();

  float3 oc = ro - position;
	float b = dot( oc, rd );
	float c = dot( oc, oc ) - radius * radius;
	float h = b*b - c;

	if( h<0.0 ) return; // -1.0;
	float tHit = -b - sqrt( h );

  Attribute attr;
  attr.radius = radius;
  attr.position = position;
  ReportHit(tHit, /*hitKind*/ 0, attr);
}

GPRT_MISS_PROGRAM(miss, (MissProgData, record), (Payload, payload))
{
  uint2 pixelID = DispatchRaysIndex().xy;  
  int pattern = (pixelID.x / 8) ^ (pixelID.y/8);
  payload.color = (pattern & 1) ? record.color1 : record.color0;
}
