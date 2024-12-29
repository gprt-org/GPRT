#include "sharedCode.h"

#include "math/MathConstants.slangh"
#include "math/CellInterpolators.slang"

[[vk::push_constant]]
PushConstants pc;


struct Payload {
  float3 color;
};

// This intersection program will be called when rays hit our axis
// aligned bounding boxes. Here, we can fetch per-geometry data and
// process that data, but we do not have access to the ray payload
// structure here.
//
// Instead, we pass data through a customizable Attributes structure
// for further processing by closest hit / any hit programs.
struct BBoxAttributes {
  float3 cen;
  float tN;
  float3 rad;
  float tF;
};

[shader("intersection")]
void AABBIntersection(uniform AABBGeomData record) {
  uint primID = PrimitiveIndex();
  float3 ro = ObjectRayOrigin();
  float3 rd = ObjectRayDirection();
  float tcur = RayTCurrent();

  BBoxAttributes attr;

  // raytrace bounding box
  float3 bbmin = record.aabbs[primID * 2 + 0];
  float3 bbmax = record.aabbs[primID * 2 + 1];

  attr.cen = 0.5 * (bbmin + bbmax);
  attr.rad = 0.5 * (bbmax - bbmin);

  float3 m = 1.0 / rd;
  float3 n = m * (ro - attr.cen);
  float3 k = abs(m) * attr.rad;

  float3 t1 = -n - k;
  float3 t2 = -n + k;

  attr.tN = max(max(t1.x, t1.y), t1.z);
  attr.tF = min(min(t2.x, t2.y), t2.z);

  if (attr.tN > attr.tF || attr.tF < 0.0)
    return;

  int hitKind = 0;
  if (attr.tN > 0.0) {
    // front face
    if (attr.tN <= tcur)
      hitKind |= 1;
    // back face
    if (attr.tF <= tcur)
      hitKind |= 2;
  }

  if (hitKind != 0)
    ReportHit(attr.tN, hitKind, attr);
}

// This closest hit program will be called when our intersection program
// reports a hit between our ray and our custom primitives.
// Here, we can fetch per-geometry data, process that data, and send
// it back to our ray generation program.
//
// Note, since this is a custom AABB primitive, our intersection program
// above defines what attributes are passed to our closest hit program.
//
// Also note, this program is also called after all ReportHit's have been
// called and we can conclude which reported hit is closest.
[shader("closesthit")]
void AABBClosestHit(uniform AABBGeomData record, inout Payload payload, in BBoxAttributes attr) {
  float3 ro = ObjectRayOrigin();
  float3 rd = ObjectRayDirection();
  int hitKind = HitKind();

  // front
  if (bool(hitKind & 1)) {
    float3 pos = ro + rd * attr.tN;
    float3 e = smoothstep(attr.rad - 0.03, attr.rad - 0.02, abs(pos - attr.cen));
    float al = 1.0 - (1.0 - e.x * e.y) * (1.0 - e.y * e.z) * (1.0 - e.z * e.x);
    payload.color = lerp(float3(0.0), float3(1.0), 0.15 + 0.85 * al);
  }

  // back
  if (bool(hitKind & 2)) {
    float3 pos = ro + rd * attr.tF;
    float3 e = smoothstep(attr.rad - 0.03, attr.rad - 0.02, abs(pos - attr.cen));
    float al = 1.0 - (1.0 - e.x * e.y) * (1.0 - e.y * e.z) * (1.0 - e.z * e.x);
    payload.color = lerp(payload.color, float3(1.0), 0.25 + 0.75 * al);
  }
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

  RaytracingAccelerationStructure world = gprt::getAccelHandle(record.world);
  
  // 8 by 16 grid of 256x256 blue noise textures
  Texture2D stbn = gprt::getTexture2DHandle(record.stbn);

  // camera movement
  float an = pc.time;
  float3 ro = float3(3.5 * cos(an), 0.7, 3.5 * sin(an));
  float3 ta = float3(0.0, -0.1, 0.0);
  
  // camera matrix
  float3 ww = normalize(ta - ro);
  float3 uu = normalize(cross(ww, float3(0.0, -1.0, 0.0)));
  float3 vv = normalize(cross(uu, ww));

  // create element
  QuadraticHexahedron e = CreateQuadraticHexahedron(pc.time);
  #define NVERTS 20
 
  float time = pc.time;   

  float[NVERTS] vertVals; 
  for (int i = 0; i < NVERTS; ++i) {
    vertVals[i] = .5 + .5 * cos(time * (1.f / NVERTS) * (i + 1));
  }

  float4 tot = float4(0.0); 
 
  for (int m = 0; m < AA; m++) {
  for (int n = 0; n < AA; n++) {
    // pixel coordinates
    float2 o = float2(float(m), float(n)) / float(AA) - 0.5;
    float2 p = (2.0 * (pixelID + o) - iResolution.xy) / iResolution.y;

    // create view ray
    float3 rd = normalize(p.x * uu + p.y * vv + 3.0 * ww);

    int frame = (m * AA * AA + n * AA + pc.frameID * AA * AA) % 128;
    uint2 gridCoord = int2(frame % 8, (frame / 8) % 16);
    uint2 texCoord = int2(pixelID.x % 256, pixelID.y % 256);
    uint2 coord = gridCoord * 256 + texCoord;
    float startRayOffset = stbn[coord].x;

    float4 color = float4(.0);
    float zMin = 2.0;
    float zMax = 5.0;
    float step = (zMax - zMin) / float(NUM_STEPS);
    float3 pt = ro + rd * zMin + rd * step * startRayOffset;
    for (int i = 0; i < NUM_STEPS; i++)
    {
      float3 rst;
      if (invertInterpolant(pt, e, rst)) {
        float[NVERTS] w = e::IsoToSupport(rst);
        float value = 0.0;
        for (int vid = 0; vid < NVERTS; ++vid) {
          value += w[vid] * vertVals[vid];
        }

        // Colormapped value and a hardcoded optical density
        float3 rgb = viridis(value) * EXPOSURE;
        float density = 2.0;
        float rho = 1.0 - exp(-density * step);
        color = over(color, float4(rgb, rho));
        if (color.a > .95) break;
      }
      pt += rd * step;
    }

    tot += color;


    // RayDesc rayDesc;
    // rayDesc.Origin = ro;
    // rayDesc.Direction = rd;
    // rayDesc.TMin = 0.0;
    // rayDesc.TMax = 10000.0;
    // TraceRay(world,           // the tree
    //           RAY_FLAG_NONE,   // ray flags
    //           0xff,            // instance inclusion mask
    //           0,               // ray type
    //           1,               // number of ray types
    //           0,               // miss index
    //           rayDesc,         // the ray to trace
    //           payload          // the payload IO
    // );

    // tot += payload.color;
  }
  }

  tot /= float(AA * AA);

  // Composite over a background
  float2 q = pixelID.xy / iResolution.xy;
  float2 v = -1.0 + 2.0 * q;
  v.x *= iResolution.x / iResolution.y;
  float4 color = over(tot, float4(float3(0.01) * (1.0 - 0.2 * length(v)), 1.0));

  const int fbOfs = pixelID.x + iResolution.x * pixelID.y;
  record.frameBuffer[fbOfs] = gprt::make_bgra(color);
}

[shader("miss")]
void miss(inout Payload payload) {
  payload.color = float3(0.0);
}