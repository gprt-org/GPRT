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
  float4 color;
  float tHit;
};

inline float3 over(float3 Cin, float3 Cx, float Ain, float Ax)
{
  return Cin + Cx*Ax*(1.f-Ain);
} 

inline float over(float Ain, float Ax)
{
  return Ain + (1.f-Ain)*Ax;
}

GPRT_RAYGEN_PROGRAM(simpleRayGen, (RayGenData, record))
{
  Payload payload;
  uint2 pixelID = DispatchRaysIndex().xy;
  float2 screen = (float2(pixelID) + 
                  float2(.5f, .5f)) / float2(record.fbSize);

  // Generate ray
  RayDesc rayDesc;
  rayDesc.Origin = record.camera.pos;
  rayDesc.Direction = 
    normalize(record.camera.dir_00
    + screen.x * record.camera.dir_du
    + screen.y * record.camera.dir_dv
  );
  rayDesc.TMin = 0.0;
  rayDesc.TMax = 10.0;
  
  // Trace ray against surface
  RaytracingAccelerationStructure meshes = gprt::getAccelHandle(record.meshes);
  TraceRay(
    meshes, // the tree
    RAY_FLAG_FORCE_OPAQUE, // ray flags
    0xff, // instance inclusion mask
    0, // ray type
    1, // number of ray types
    0, // miss type
    rayDesc, // the ray to trace
    payload // the payload IO
  );
  
  float3 backgroundColor = payload.color.rgb;

  // Replace background color if we hit a surface, and update tHit distance
  if (payload.tHit > 0.0) {
    rayDesc.TMax = payload.tHit;
  }

  // Next, march ray through volume
  RaytracingAccelerationStructure cells = gprt::getAccelHandle(record.cells);
  float3 color = float3(0.0, 0.0, 0.0);
  float alpha = 0.0;
  for (float t = rayDesc.TMin; t < rayDesc.TMax; t += .05f) {
    if (alpha > .99f) break;
    float3 x = rayDesc.Origin + rayDesc.Direction * t;

    RayDesc pointDesc;
    pointDesc.Origin = x;
    pointDesc.Direction = float3(1.0, 1.0, 1.0);
    pointDesc.TMin = pointDesc.TMax = 0.0;

    TraceRay(
      cells, // the tree
      RAY_FLAG_FORCE_OPAQUE, // ray flags
      0xff, // instance inclusion mask
      0, // ray type
      1, // number of ray types
      0, // miss type
      pointDesc, // the ray to trace
      payload // the payload IO
    );

    // Composite volumetric color
    if (payload.tHit > 0.0) 
    {
      color = over(color, payload.color.rgb, alpha, payload.color.w);
      alpha = over(alpha, payload.color.w);
    }
  }

  float3 finalColor = over(color, backgroundColor, alpha, 1.0);

  const int fbOfs = pixelID.x + record.fbSize.x * pixelID.y;
  gprt::store(record.fbPtr, fbOfs, gprt::make_rgba(finalColor));
}

struct TriangleAttributes {
  float2 bc;
};

GPRT_CLOSEST_HIT_PROGRAM(TriangleMesh, (TrianglesGeomData, record), (Payload, payload), (TriangleAttributes, attributes))
{
  // compute normal:
  uint   primID = PrimitiveIndex();
  int3   index  = gprt::load<int3>(record.index, primID);
  float3 A      = gprt::load<float3>(record.vertex, index.x);
  float3 B      = gprt::load<float3>(record.vertex, index.y);
  float3 C      = gprt::load<float3>(record.vertex, index.z);
  float3 Ng     = normalize(cross(B-A,C-A));
  float3 rayDir = WorldRayDirection();
  payload.color = (.2f + .8f * abs(dot(rayDir,Ng))) * float4(record.color.x, record.color.y, record.color.z, 1.0);
  payload.tHit = RayTCurrent();
}

struct TetrahedraAttributes {
  float4 bc;
};

/* computes the (oriented) volume of the tet given by the four vertices */
inline float volume(in float3 P, in float3 A, in float3 B, in float3 C)
{
  return dot(P - A, cross(B - A, C - A));
}

GPRT_INTERSECTION_PROGRAM(TetrahedralMesh, (TetrahedraGeomData, record))
{
  uint   primID = PrimitiveIndex();
  int4   index  = gprt::load<int4>(record.index, primID);
  float3 P0      = gprt::load<float3>(record.vertex, index.x);
  float3 P1      = gprt::load<float3>(record.vertex, index.y);
  float3 P2      = gprt::load<float3>(record.vertex, index.z);
  float3 P3      = gprt::load<float3>(record.vertex, index.w);

  float3 P      = WorldRayOrigin();
  float vol_all = volume(P0, P1, P3, P2);
  if (vol_all == 0.f) return;

  const float bary0 = volume(P, P1, P3, P2) / vol_all;
  if (bary0 < 0.f) return;

  const float bary1 = volume(P, P0, P2, P3) / vol_all;
  if (bary1 < 0.f) return;

  const float bary2 = volume(P, P0, P3, P1) / vol_all;
  if (bary2 < 0.f) return;

  const float bary3 = volume(P, P0, P1, P2) / vol_all;
  if (bary3 < 0.f) return;

  TetrahedraAttributes attr;
  attr.bc = float4(bary0, bary1, bary2, bary3);
  ReportHit(0.0, /*hitKind*/ 0, attr);
}

GPRT_CLOSEST_HIT_PROGRAM(TetrahedralMesh, (TetrahedraGeomData, record), (Payload, payload), (TetrahedraAttributes, attributes))
{
  payload.color = float4(1.0, 0.0, 0.0, .1);
  payload.tHit = 1.f;
}

GPRT_MISS_PROGRAM(miss, (MissProgData, record), (Payload, payload))
{
  uint2 pixelID = DispatchRaysIndex().xy;  
  int pattern = (pixelID.x / 8) ^ (pixelID.y/8);
  float3 color = (pattern & 1) ? record.color1 : record.color0;
  payload.color = float4(color, 1.0);
  payload.tHit = -1.0;
}
