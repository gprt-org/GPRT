#include "sharedCode.h"

[[vk::push_constant]]
PushConstants pc;

struct Payload {
  float3 color;
  float density;
};

[shader("closesthit")]
void SolidClosestHit(uniform SolidGeomData record, inout Payload payload, in float4 rstw) {
  // Per-vertex values are interpolated within the built-in intersector
  payload.density = rstw.w * 2.0;
  
  // For other attribute types, we'll use the "rst" values which parameterize our solid.
  uint kind = HitKind();

  // Different solids have different parameterizations. 
  float4 w0 = float4(0.), w1 = float4(0.);
  float3 rst = rstw.xyz;
  if (kind == HIT_KIND_TETRAHEDRON)     GetTetWeights(rst, w0); // <- helper functions in sharedCode.h
  else if (kind == HIT_KIND_PYRAMID)    GetPyrWeights(rst, w0, w1.x);
  else if (kind == HIT_KIND_WEDGE)      GetWedWeights(rst, w0.xyz, w1.xyz);
  else if (kind == HIT_KIND_HEXAHEDRON) GetHexWeights(rst, w0, w1);

  // The interpolated value can be found using an inner product.
  float4 vals0 = .5 + .5 * cos(pc.time * float4(1., 2., 3., 4.));
  float4 vals1 = .5 + .5 * cos(pc.time * float4(4., 5., 6., 7.));
  float val = dot(w0, vals0) + dot(w1, vals1);

  // Colormap the interpolated value
  payload.color = viridis(val) * EXPOSURE;
}

[shader("miss")]
void miss(inout Payload payload) {
  payload.density = 0.0;
}

// This ray generation program will kick off the ray tracing process,
// generating rays and tracing them into the world.
[shader("raygeneration")]
void raygen(uniform RayGenData record) {
  Payload payload;
  uint2 pixelID = DispatchRaysIndex().xy;
  uint2 iResolution = DispatchRaysDimensions().xy;
  uint frameID = pc.frameID;

  bool debug = all(pixelID == iResolution / 2);

  // 8 by 16 grid of 256x256 blue noise textures
  DescriptorHandle<Texture2D> stbn = record.stbn;

  // camera movement
  float an = pc.time * .1;
  float3 ro = float3(3.5 * sin(an), 0.7, 3.5 * cos(an));
  float3 ta = float3(0.0, -0.1, 0.0);
  
  // camera matrix
  float3 ww = normalize(ta - ro);
  float3 uu = normalize(cross(ww, float3(0.0, -1.0, 0.0)));
  float3 vv = normalize(cross(uu, ww));

  float4 tot = float4(0.0); 
 
  for (int m = 0; m < AA; m++) {
  for (int n = 0; n < AA; n++) {
    // pixel coordinates
    float2 o = float2(float(m), float(n)) / float(AA) - 0.5;
    float2 p = (2.0 * (pixelID + o) - iResolution.xy) / iResolution.y;

    // create view ray
    float3 rd = normalize(p.x * uu + p.y * vv + 3.0 * ww);

    // sample a random jitter value
    uint2 stbnCoord = GetSTBNCoordinate(m, n, pc.frameID, pixelID);
    float startRayOffset = stbn[stbnCoord].x;

    float4 color = float4(.0);
    float zMin = 2.0;
    float zMax = 5.0;
    float step = (zMax - zMin) / float(NUM_STEPS);

    PointDesc pointDesc;
    pointDesc.Origin = ro + rd * zMin + rd * step * startRayOffset;
    for (int i = 0; i < NUM_STEPS; i++)
    {
      TracePoint<Payload>(record.world,    // the tree
                 RAY_FLAG_NONE,   // ray flags
                 0xff,            // instance inclusion mask
                 0,               // "ray" type
                 0,               // miss index
                pointDesc,        // the point to query
                payload          // the payload IO
      );

      float rho = 1.0 - exp(-payload.density * step);
      color = over(color, float4(payload.color, rho));
      if (color.a > .95) break;

      pointDesc.Origin += rd * step;
    }

    tot += color;
  }
  }

  tot /= float(AA * AA);

  // Composite over a black background
  float4 color = over(tot, float4(float3(0.), 1.0));
  const int fbOfs = pixelID.x + iResolution.x * pixelID.y;
  record.frameBuffer[fbOfs] = gprt::make_bgra(color);
}
