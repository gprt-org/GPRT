#include "sharedCode.h"

struct Payload {
  float3 color;
  float tHit;
};

float4
over(float4 a, float4 b) {
  float4 result;
  result.a = a.a + b.a * (1.f - a.a);
  if (result.a == 0.f)
    return a;   // avoid NaN
  result.rgb = (a.rgb * a.a + b.rgb * b.a * (1.f - a.a)) / result.a;
  return result;
}

[shader("compute")]
[numthreads(1, 1, 1)]
void
CompositeGui(uint3 DispatchThreadID: SV_DispatchThreadID, uniform CompositeGuiConstants pc) {
  int2 pixelID = DispatchThreadID.xy;
  const int fbOfs = pixelID.x + pc.fbSize.x * pixelID.y;
  float2 fragCoord = pixelID + float2(.5f, .5f);
  float2 uv = (fragCoord) / float2(pc.fbSize);

  // Load color of the rendered image
  float4 imageColor = pc.imageBuffer[fbOfs];

  // Sample the color from the GUI texture
  SamplerState sampler = gprt::getDefaultSampler();
  Texture2D guiTexture = gprt::getTexture2DHandle(pc.guiTexture);
  float4 guiColor = guiTexture.SampleGrad(sampler, uv, float2(0.f, 0.f), float2(0.f, 0.f));

  // Gamma correction
  guiColor = pow(guiColor, 2.2f);

  // Composite the GUI on top of the scene
  float4 pixelColor = over(guiColor, imageColor);
  pc.frameBuffer[fbOfs] = gprt::make_bgra(pixelColor);
}

[[vk::push_constant]]
RTPushConstants pc;

[shader("raygeneration")]
void
raygen(uniform RayGenData record) {
  Payload payload;
  uint2 pixelID = DispatchRaysIndex().xy;
  uint2 fbSize = DispatchRaysDimensions().xy;
  float2 screen = (float2(pixelID) + float2(.5f, .5f)) / float2(fbSize);

  RayDesc rayDesc;
  rayDesc.Origin = pc.camera.pos;
  rayDesc.Direction = normalize(pc.camera.dir_00 + screen.x * pc.camera.dir_du + screen.y * pc.camera.dir_dv);
  rayDesc.TMin = 0.001;
  rayDesc.TMax = 10000.0;
  RaytracingAccelerationStructure triangleTLAS = gprt::getAccelHandle(record.triangleTLAS);
  TraceRay(triangleTLAS, RAY_FLAG_FORCE_OPAQUE, 0xff, 0, 1, /*miss type*/ 0, rayDesc, payload);

  if (payload.tHit > 0.f)
    rayDesc.TMax = payload.tHit;

  RaytracingAccelerationStructure obbAccel = gprt::getAccelHandle(record.obbAccel);
  TraceRay(obbAccel, RAY_FLAG_FORCE_OPAQUE, 0xff, 0, 1, /*miss type*/ 1, rayDesc, payload);

  const int fbOfs = pixelID.x + fbSize.x * pixelID.y;
  record.imageBuffer[fbOfs] = float4(payload.color, 1.f);
}

[shader("closesthit")]
void
hitTriangle(uniform TrianglesGeomData record, inout Payload payload, in float2 bc) {
  payload.color = float3(bc.x, bc.y, 1.0 - (bc.x + bc.y));
  payload.tHit = RayTCurrent();
}

[Differentiable]
float
getSurfaceArea(float3 aabbMin, float3 aabbMax) {
  float w = aabbMax.x - aabbMin.x;
  float h = aabbMax.y - aabbMin.y;
  float d = aabbMax.z - aabbMin.z;
  return 2 * w * h + 2 * w * d + 2 * h * d;
}

[Differentiable]
float
getVolume(float3 aabbMin, float3 aabbMax) {
  float w = aabbMax.x - aabbMin.x;
  float h = aabbMax.y - aabbMin.y;
  float d = aabbMax.z - aabbMin.z;
  return w * h * d;
}

[Differentiable]
float3x3
eul_to_mat3(float3 eul) {
  float3x3 mat;
  float ci, cj, ch, si, sj, sh, cc, cs, sc, ss;

  sincos(eul[0], si, ci);
  sincos(eul[1], sj, cj);
  sincos(eul[2], sh, ch);
  cc = ci * ch;
  cs = ci * sh;
  sc = si * ch;
  ss = si * sh;

  mat[0][0] = cj * ch;
  mat[0][1] = sj * sc - cs;
  mat[0][2] = sj * cc + ss;
  mat[1][0] = cj * sh;
  mat[1][1] = sj * ss + cc;
  mat[1][2] = sj * cs - sc;
  mat[2][0] = -sj;
  mat[2][1] = cj * si;
  mat[2][2] = cj * ci;
  return mat;
}

void
atomicAccumulate(float *derivBufferPtr, uint idx, float val) {
  // No need to accumulate zeros.
  if (val == 0.f)
    return;

  Atomic<uint> *derivBuffer = (Atomic<uint> *) (derivBufferPtr);

  // Loop for as long as the compareExchange() fails, which means another thread
  // is trying to write to the same location.
  //
  for (;;) {
    uint oldInt = derivBuffer[idx].load();
    float oldFloat = asfloat(oldInt);

    float newFloat = oldFloat + val;

    uint newInt = asuint(newFloat);

    // compareExchange() returns the value at the location before the operation.
    // If it's changed, we have contention between threads & need to try again.
    //
    if (derivBuffer[idx].compareExchange(oldInt, newInt) == oldInt)
      break;
  }
}

// no_diff float
// atomicMin32f(no_diff in Buffer buffer, no_diff uint32_t index, inout float value) {
//   uint ret_i = asuint(buffers[buffer.index].Load<float>(index * sizeof(float)));
//   while (value < asfloat(ret_i)) {
//     uint old = ret_i;
//     buffers[buffer.index].InterlockedCompareExchange(index * sizeof(float), old, asuint(value), ret_i);
//     if (ret_i == old)
//       break;
//   }
//   return asfloat(ret_i);
// }

no_diff float
atomicMin(no_diff uint64_t bufferAddr, no_diff uint idx, inout float val) {
  Atomic<uint> *buffer = (Atomic<uint> *) (((float *) bufferAddr));

  // Loop for as long as the compareExchange() fails, which means another thread
  // is trying to write to the same location.
  //
  for (;;) {
    uint oldInt = buffer[idx].load();
    float oldFloat = asfloat(oldInt);

    val = min(oldFloat, val);

    uint newInt = asuint(val);

    // compareExchange() returns the value at the location before the operation.
    // If it's changed, we have contention between threads & need to try again.
    //
    if (buffer[idx].compareExchange(oldInt, newInt) == oldInt)
      break;
  }
  return val;
}

no_diff float
atomicMax(uint64_t bufferAddr, uint idx, inout float val) {
  Atomic<uint> *buffer = (Atomic<uint> *) (((float *) bufferAddr));

  // Loop for as long as the compareExchange() fails, which means another thread
  // is trying to write to the same location.
  //
  for (;;) {
    uint oldInt = buffer[idx].load();
    float oldFloat = asfloat(oldInt);

    val = max(oldFloat, val);

    uint newInt = asuint(val);

    // compareExchange() returns the value at the location before the operation.
    // If it's changed, we have contention between threads & need to try again.
    //
    if (buffer[idx].compareExchange(oldInt, newInt) == oldInt)
      break;
  }
  return val;
}

// Derivative of an atomic max is discontinuous, where only an exact match between value
// and the currently stored max is non-zero
[BackwardDerivativeOf(atomicMin)]
void
atomicMin(in uint64_t bufferAddr, uint32_t index, inout DifferentialPair<float> value) {
  float *buffer = (float *) bufferAddr;
  float ret = buffer[index];
  value = diffPair(value.p, abs(value.p - ret) < .000001f ? ret : 0.f);
}

// Derivative of an atomic max is discontinuous, where only an exact match between value
// and the currently stored max is non-zero
[BackwardDerivativeOf(atomicMax)]
void
atomicMax(in uint64_t bufferAddr, uint32_t index, inout DifferentialPair<float> value) {
  float *buffer = (float *) bufferAddr;
  float ret = buffer[index];
  value = diffPair(value.p, abs(value.p - ret) < .000001f ? ret : 0.f);
}

[Differentiable]
float
computeOBB(float3 eul, no_diff float3 a, no_diff float3 b, no_diff float3 c, no_diff uint64_t aabbsAddr) {
  // Compute triangle OBB
  float3x3 rot = eul_to_mat3(eul);
  float3 aabbMin = float3(1e38f);
  float3 aabbMax = float3(-1e38f);
  a = mul(a, rot);
  b = mul(b, rot);
  c = mul(c, rot);
  aabbMin = min(aabbMin, a);
  aabbMax = max(aabbMax, a);
  aabbMin = min(aabbMin, b);
  aabbMax = max(aabbMax, b);
  aabbMin = min(aabbMin, c);
  aabbMax = max(aabbMax, c);

  // In forward pass, we atomically min/max the OBB
  atomicMin(aabbsAddr, 0, aabbMin.x);
  atomicMin(aabbsAddr, 1, aabbMin.y);
  atomicMin(aabbsAddr, 2, aabbMin.z);
  atomicMax(aabbsAddr, 3, aabbMax.x);
  atomicMax(aabbsAddr, 4, aabbMax.y);
  atomicMax(aabbsAddr, 5, aabbMax.z);

  // Note, in the backward pass, combined aabb min and max will take on the
  // atomically combined OBB
  return getSurfaceArea(aabbMin, aabbMax) + getVolume(aabbMin, aabbMax);
}

[shader("compute")]
[numthreads(1, 1, 1)]
void
ClearOBB(uint3 DispatchThreadID: SV_DispatchThreadID, uniform ComputeOBBConstants params) {
  if (DispatchThreadID.x >= 1)
    return;
  params.aabbs[0] = float3(+1e38f);
  params.aabbs[1] = float3(-1e38f);

  // Clear the gradient
  params.eulRots[1] = float3(0.f);
}

[shader("compute")]
[numthreads(128, 1, 1)]
void
ComputeOBB(uint3 DispatchThreadID: SV_DispatchThreadID, uniform ComputeOBBConstants params) {
  if (DispatchThreadID.x >= params.numTrisToInclude)
    return;   // temp
  int triID = DispatchThreadID.x;

  uint3 tri = params.indices[triID];
  float3 a = params.vertices[tri.x];
  float3 b = params.vertices[tri.y];
  float3 c = params.vertices[tri.z];

  // Current euler rotation
  float3 eul = params.eulRots[0];

  // Compute the current OBB
  computeOBB(eul, a, b, c, (uint64_t) params.aabbs);

  // Also update the visualization of the OBB
  float3x3 rot = eul_to_mat3(eul);
  params.instance->transform = float3x4(float4(rot[0], 0), float4(rot[1], 0), float4(rot[2], 0));
}

[shader("compute")]
[numthreads(128, 1, 1)]
void
BackPropOBB(uint3 DispatchThreadID: SV_DispatchThreadID, uniform ComputeOBBConstants params) {
  if (DispatchThreadID.x >= params.numTrisToInclude)
    return;   // temp
  int triID = DispatchThreadID.x;

  uint3 tri = params.indices[triID];
  float3 a = params.vertices[tri.x];
  float3 b = params.vertices[tri.y];
  float3 c = params.vertices[tri.z];

  // Current euler rotation
  float3 eul = params.eulRots[0];
  DifferentialPair<float3> diffEul = diffPair(eul, float3(0));

  bwd_diff(computeOBB)(diffEul, a, b, c, (uint64_t) params.aabbs, /*dSurfaceArea*/ 1.0f);

  // In our application, the gradient is differentiated rotation
  float3 grad = diffEul.d;

  atomicAccumulate((float *) params.eulRots, 3 + 0, grad.x);
  atomicAccumulate((float *) params.eulRots, 3 + 1, grad.y);
  atomicAccumulate((float *) params.eulRots, 3 + 2, grad.z);

  // Have thread 0 report the current surface area
  if (DispatchThreadID.x == 0) {
    float3 aabbMin = params.aabbs[0];
    float3 aabbMax = params.aabbs[1];
    float SA = getSurfaceArea(aabbMin, aabbMax);
    params.eulRots[2] = float3(SA, 0.f, 0.f);
  }
}

struct BBoxAttributes {
  float3 cen;
  float tN;
  float3 rad;
  float tF;
};

[shader("intersection")]
void
intersectBoundingBox(uniform BoundingBoxData record) {

  float3 ro = ObjectRayOrigin();
  float3 rd = ObjectRayDirection();
  float tcur = RayTCurrent();

  BBoxAttributes attr;

  // raytrace bounding box
  float3 bbmin = record.aabbs[0];
  float3 bbmax = record.aabbs[1];

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

[shader("closesthit")]
void
hitBoundingBox(uniform BoundingBoxData record, inout Payload payload, in BoundingBoxData attr) {
  // Unfortunate driver bugs seem to cause "attr" here to be undefined / junk...

  float3 ro = ObjectRayOrigin();
  float3 rd = ObjectRayDirection();
  float tcur = payload.tHit;

  // raytrace bounding box
  float3 bbmin = record.aabbs[0];
  float3 bbmax = record.aabbs[1];

  float3 cen = 0.5 * (bbmin + bbmax);
  float3 rad = 0.5 * (bbmax - bbmin);

  float3 m = 1.0 / rd;
  float3 n = m * (ro - cen);
  float3 k = abs(m) * rad;

  float3 t1 = -n - k;
  float3 t2 = -n + k;

  float tN = max(max(t1.x, t1.y), t1.z);
  float tF = min(min(t2.x, t2.y), t2.z);

  int hitKind = HitKind();

  // front
  if (bool(hitKind & 1)) {
    float3 pos = ro + rd * tN;
    float3 e = smoothstep(rad - 0.03, rad - 0.02, abs(pos - cen));
    float al = 1.0 - (1.0 - e.x * e.y) * (1.0 - e.y * e.z) * (1.0 - e.z * e.x);
    payload.color = lerp(payload.color, float3(1.0), 0.15 + 0.85 * al);
  }

  // back
  if (bool(hitKind & 2)) {
    float3 pos = ro + rd * tF;
    float3 e = smoothstep(rad - 0.03, rad - 0.02, abs(pos - cen));
    float al = 1.0 - (1.0 - e.x * e.y) * (1.0 - e.y * e.z) * (1.0 - e.z * e.x);
    payload.color = lerp(payload.color, float3(1.0), 0.25 + 0.75 * al);
  }
}

[shader("miss")]
void
triMiss(uniform MissProgData record, inout Payload payload) {
  uint2 pixelID = DispatchRaysIndex().xy;
  int pattern = (pixelID.x / 32) ^ (pixelID.y / 32);
  payload.color = bool(pattern & 1) ? record.color1 : record.color0;
  payload.tHit = 1e38f;
}

[shader("miss")]
void
obbMiss(uniform MissProgData record, inout Payload payload) {}

