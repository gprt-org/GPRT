#include "sharedCode.h"

[[vk::push_constant]]
PushConstants pc;

struct Payload {
  float3 color;
};

// This closest hit program will be called when rays hit triangles.
// Here, we can fetch per-geometry data, process that data, and send
// it back to our ray generation program.
//
// The first parameter here is the name of our entry point.
//
// The second is the type and name of the shader record. A shader record
// can be thought of as the parameters passed to this kernel.
//
// The third is the type of the ray payload structure. We use the ray payload
// to pass data between this program and our ray generation program.
//
// The fourth is the type of the intersection attributes structure.
// For triangles, this is always a struct containing two floats
// called "barycentrics", which we use to interpolate per-vertex
// values.
[shader("closesthit")]
void TriangleMesh(uniform TrianglesGeomData record, inout Payload payload, in float2 bc) {
  payload.color = float3(bc.x, bc.y, 1.0 - (bc.x + bc.y));
}

#define AA 3

// This ray generation program will kick off the ray tracing process,
// generating rays and tracing them into the world.
//
// The first parameter here is the name of our entry point.
//
// The second is the type and name of the shader record. A shader record
// can be thought of as the parameters passed to this kernel.
[shader("raygeneration")]
void raygen(uniform RayGenData record) {
  Payload payload;
  uint2 pixelID = DispatchRaysIndex().xy;
  uint2 iResolution = DispatchRaysDimensions().xy;
  
  RaytracingAccelerationStructure world = gprt::getAccelHandle(record.world);

  // camera movement
  float an = pc.time;
  float3 ro = float3(-4.0 * sin(an), 0.0, -4.0 * cos(an));
  float3 ta = float3(0.0, 0.0, 0.0);

  // camera matrix
  float3 ww = normalize(ta - ro);
  float3 uu = normalize(cross(ww, float3(0.0, -1.0, 0.0)));
  float3 vv = normalize(cross(uu, ww)); 

  float3 tot = float3(0.0);

  for (int m = 0; m < AA; m++)
  for (int n = 0; n < AA; n++)
  {
    // pixel coordinates
    float2 o = float2(float(m), float(n)) / float(AA) - 0.5;
    float2 p = (2.0 * (pixelID + o) - iResolution.xy) / iResolution.y;

    // create view ray
    float3 rd = normalize(p.x * uu + p.y * vv + 3.0 * ww);

    RayDesc rayDesc;
    rayDesc.Origin = ro;
    rayDesc.Direction = rd;
    rayDesc.TMin = 0.0;
    rayDesc.TMax = 10000.0;
    TraceRay(world,          // the tree
             RAY_FLAG_NONE,  // ray flags
             0xff,           // instance inclusion mask
             0,              // ray type
             1,              // number of ray types
             0,              // miss index
             rayDesc,        // the ray to trace
             payload         // the payload IO
    );

    tot += payload.color;
  }
  tot /= float(AA * AA);

  const int fbOfs = pixelID.x + iResolution.x * pixelID.y;
  record.frameBuffer[fbOfs] = gprt::make_bgra(tot);
}

// A background with a vignette effect.
[shader("miss")]
void miss(inout Payload payload) {
  float2 resolution = DispatchRaysDimensions().xy;
  float2 fragCoord = DispatchRaysIndex().xy;
  float2 p = (-resolution.xy + 2.0 * fragCoord) / resolution.y;
  float3 col = float3(0.08) * (1.0 - 0.3 * length(p)) + 0.02 * WorldRayDirection().y;
  payload.color = col;
}
