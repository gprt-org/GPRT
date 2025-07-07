#pragma once

#include "gprt_fallbacks.h"
import gprt_builtins;

[ForceInline]
T
LoadAligned<int alignment, T>(T *ptr) {
  return spirv_asm {
          result = OpLoad $$T $ptr Aligned !alignment;
  };
}

// To account for cases where AABBs are not assigned an intersection program.
[shader("intersection")]
void fallbackIntersection() {}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
// LINE SWEPT SPHERES
////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Intersects a ray with a linearly swept sphere (LSS).
 *
 * This function calculates the intersection of a ray with a LSS defined by two endpoints and varying radii. 
 * It supports optional end-cap intersection testing.
 *
 * @param rayOrigin     Origin of the ray.
 * @param rayDirection  Direction of the ray (should be normalized for accurate results).
 * @param curveStart    Starting point of the LSS.
 * @param curveEnd      Ending point of the LSS.
 * @param radiusStart   Radius of the LSS at the starting point.
 * @param radiusEnd     Radius of the LSS at the ending point.
 * @param enableCapStart Enable intersection testing with the starting cap.
 * @param enableCapEnd   Enable intersection testing with the ending cap.
 * @param rayMin         Minimum parameter value along the ray to consider for intersection.
 * @param rayMax         Maximum parameter value along the ray to consider for intersection.
 * @param returnExit     If true, returns the exit intersection; otherwise, returns the entry intersection.
 * @param[out] rayParam  Parameter value (t) along the ray at the intersection point.
 * @param[out] curveParam Parameter value (u) along the curve at the intersection point.
 * @return True if an intersection is found, otherwise false.
 */
[ForceInline]
bool
intersectRayLSS(float3 rayOrigin, float3 rayDirection,    
                 float3 curveStart, float3 curveEnd,      
                 float radiusStart, float radiusEnd,      
                 bool enableCapStart,                     
                 bool enableCapEnd,                       
                 float rayMin, float rayMax,              
                 bool returnExit,                         
                 out float rayParam, out float curveParam 
) {
    rayParam = 0.0;
    curveParam = 0.0;

    const float3 direction = rayDirection;
    const float3 curveMidpoint = 0.5f * (curveStart + curveEnd);
    const float radiusMidpoint = 0.5f * (radiusStart + radiusEnd);

    const float directionDot = dot(direction, direction);
    const float inverseDirectionDot = 1.0f / directionDot;

    const float alignmentOffset = dot(direction, curveMidpoint - rayOrigin) * inverseDirectionDot;
    rayOrigin += alignmentOffset * direction;
    
    const float3 offset = rayOrigin - curveMidpoint;
    const float3 curveAxis = curveEnd - curveStart;
    const float radiusDelta = radiusEnd - radiusStart;

    // Compute conic coefficients
    const float a = directionDot;
    const float b = dot(curveAxis, direction);
    const float c = dot(curveAxis, curveAxis) - radiusDelta * radiusDelta;
    const float d = dot(direction, offset);
    const float e = dot(curveAxis, offset) + radiusMidpoint * radiusDelta;
    const float f = dot(offset, offset) - radiusMidpoint * radiusMidpoint;

    // Conic t-equation coefficients
    const float At = b * b - a * c;
    const float Bt = b * e - c * d;
    const float Ct = e * e - c * f;
    const float discriminant = Bt * Bt - At * Ct;

    // Degeneracy checks
    const bool isSphere = (curveAxis.x == 0 && curveAxis.y == 0 && curveAxis.z == 0 && radiusDelta == 0);
    const bool useStartSphere = !enableCapEnd;
    const bool isShellDegenerate = (c <= 0) && (radiusDelta != 0);
    const bool isCurveInvalid = ((radiusStart <= 0 && radiusEnd <= 0) || isnan(radiusStart) || isnan(radiusEnd) || 
                                  isnan(curveStart.x) || isnan(curveEnd.x) || isinf(radiusStart) || isinf(curveStart.x));

    if ((!isSphere && !isShellDegenerate && discriminant < 0) || isCurveInvalid) {
        return false;
    }

    const float sqrtDiscriminant = sqrt(discriminant);
    const float inverseAt = 1.0f / At;
    const float t0 = (-Bt + (returnExit ? -sqrtDiscriminant : sqrtDiscriminant)) * inverseAt;

    const float inverseC = 1.0f / c;
    float u0 = (b * t0 + e) * inverseC;

    // Mid-section hit test
    if (fabs(u0) <= 0.5f && (u0 * radiusDelta >= -radiusMidpoint) && !isSphere && !isShellDegenerate) {
        float hitT = t0 + alignmentOffset;
        float hitU = u0 + 0.5f;

        if (hitT <= rayMin || hitT >= rayMax) {
            return false;
        }

        rayParam = hitT;
        curveParam = hitU;
        return true;
    }

    // End-cap testing
    bool useStartCap = isSphere ? useStartSphere : (u0 < 0.0f);

    if ((useStartCap && !enableCapStart) || (!useStartCap && !enableCapEnd)) {
        return false;
    }

    const float Bs = -d + (useStartCap ? -b : b) * 0.5f;
    const float BsBs = Bs * Bs;
    const float Cs = f + 0.25f * c - (useStartCap ? -e : e);
    const float AsCs = a * Cs;

    if (BsBs < AsCs) {
        return false;
    }

    const float sqrtCapDiscriminant = sqrt(BsBs - AsCs);
    const float capT = (Bs - (returnExit ? -sqrtCapDiscriminant : sqrtCapDiscriminant)) * inverseDirectionDot;

    rayParam = capT + alignmentOffset;
    curveParam = useStartCap ? 0.0f : 1.0f;

    if (rayParam <= rayMin || rayParam >= rayMax) {
        return false;
    }

    return true;
}

[shader("intersection")]
void
LSSIntersection(uniform uint32_t userData[64], uniform LSSParameters lss) {
  uint primID = PrimitiveIndex();
  uint2 I = LoadAligned<8>(lss.indices + primID);
  float4 P0 = LoadAligned<16>(lss.vertices + I.x);
  float4 P1 = LoadAligned<16>(lss.vertices + I.y);
  bool endcap0 = bool(lss.endcap0);
  bool endcap1 = bool(lss.endcap1);
  bool exitTest = bool(lss.exitTest);

  float t, u;
  if (intersectRayLSS(ObjectRayOrigin(), ObjectRayDirection(),   // input ray
                        P0.xyz, P1.xyz, P0.w, P1.w,                // input curve
                        endcap0,                                   // true: endcap0 enabled, false: endcap0 disabled
                        endcap1, RayTMin(), RayTCurrent(),         // input interval
                        exitTest, t, u))                           // output ray param (t), curve param (u)
  {
    // HitKindLssPrimitiveNV eventually...
    ReportHit(t, /*hitKind*/ 0, u);
  }
}

void computeLssAabb(float3 v0, float r0, bool endcap0, float3 v1, float r1, bool endcap1, out float3 bblo, out float3 bbhi)
{
  float3 bblov0 = v0, bbhiv0 = v0; 
  float3 bblov1 = v1, bbhiv1 = v1;

  float3 cme, cpe;
  if (!endcap0 || !endcap1) {
    // On average, one end cap will be missing (the case with chained LSS)
    // So I'm computing these terms up front, assuming they will be used.
    float rd = r1-r0;  // Change in radius
    float3 n = v0-v1;  // Direction of LSS, also normal of end cap circle
    float3 nsqr = n*n;
    float l2 = nsqr.x + nsqr.y + nsqr.z;
    float l2Inv = rcp(l2); // when l2 is +inf, rcp returns +0
    
    float3 X = 1.f - nsqr * l2Inv;    // sin of angle between n and the coordinate bases
    float  Y = 1.f - rd * rd * l2Inv; // sin of angle between n and any point of tangency on the spherical end cap
    float3 e = sqrt(abs(X * Y)); // unit circle extents aligned with the base circle subtending the cap.
    
    float3 c = rd * n * l2Inv;
    cme = c - e;
    cpe = c + e;
  }

  #define SUPPORT_NEGATIVE_R
  #ifdef SUPPORT_NEGATIVE_R
  if (r0 < 0.0) {
    bblov0 += (r0 * rcp(r1-r0)) * (v0-v1); bbhiv0 = bblov0;
  } else
  #endif
  if (endcap0) {
    bblov0 += -r0; bbhiv0 += r0;
  }
  else {
    bblov0 += r0 * cme; bbhiv0 += r0 * cpe;
  }  

  #ifdef SUPPORT_NEGATIVE_R
  if (r1 < 0.0) {
    bblov1 += (r1 * rcp(r1-r0)) * (v0-v1); bbhiv1 = bblov1;
  } else
  #endif
  if (endcap1) {
    bblov1 += -r1; bbhiv1 += r1;
  }
  else {
    bblov1 += r1 * cme; bbhiv1 += r1 * cpe;
  }

  bblo = min(bblov0, bblov1);
  bbhi = max(bbhiv0, bbhiv1);
}

[shader("compute")]
[numthreads(256, 1, 1)]
void
LSSBounds(uint3 DispatchThreadID: SV_DispatchThreadID, uniform LSSBoundsParameters lss) {
  int primID = DispatchThreadID.x;
  if (primID >= lss.count)
    return;

  uint2 I = LoadAligned<8>(lss.indices + primID);
  float4 P0 = LoadAligned<16>(lss.vertices + I.x);
  float4 P1 = LoadAligned<16>(lss.vertices + I.y);
  bool endcap0 = bool(lss.endcap0);
  bool endcap1 = bool(lss.endcap1);

  float3 aabbMin, aabbMax;
  computeLssAabb(P0.xyz, P0.w, endcap0, P1.xyz, P1.w, endcap1, aabbMin, aabbMax);

  uint32_t offset = lss.offset;
  lss.aabbs[(offset * 2) + 2 * primID] = aabbMin;
  lss.aabbs[(offset * 2) + 2 * primID + 1] = aabbMax;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SPHERES
////////////////////////////////////////////////////////////////////////////////////////////////////////////
[shader("intersection")]
void
SphereIntersection(uniform uint32_t userData[64], uniform SphereParameters s) {
  uint primID = PrimitiveIndex();
  float4 P0 = LoadAligned<16>(s.vertices + primID);
  bool exitTest = bool(s.exitTest);

  float t, u;
  if (intersectRayLSS(ObjectRayOrigin(), ObjectRayDirection(),   // input ray
                        P0.xyz, P0.xyz, P0.w, P0.w,                // input curve
                        true,                                      // true: endcap0 enabled, false: endcap0 disabled
                        true, RayTMin(), RayTCurrent(),            // input interval
                        exitTest, t, u))                           // output ray param (t), curve param (u)
  {
    // HitKindLssPrimitiveNV eventually...
    ReportHit(t, /*hitKind*/ 0, 0.0);
  }
}

[shader("compute")]
[numthreads(256, 1, 1)]
void SphereBounds(uint3 DispatchThreadID: SV_DispatchThreadID, uniform SphereBoundsParameters s) {
  int primID = DispatchThreadID.x;
  if (primID >= s.count)
    return;

  float4 P0 = LoadAligned<16>(s.vertices + primID);
  float3 aabbMin = +float3(FLT_MAX);
  float3 aabbMax = -float3(FLT_MAX);

  aabbMin = min(aabbMin, (P0.xyz - max(P0.w, 0.0)));
  aabbMax = max(aabbMax, (P0.xyz + max(P0.w, 0.0)));

  uint32_t offset = s.offset;
  s.aabbs[(offset * 2) + 2 * primID] = aabbMin;
  s.aabbs[(offset * 2) + 2 * primID + 1] = aabbMax;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SOLIDS
////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define GPRT_TETRAHEDRON      10
// #define GPRT_VOXEL 11
#define GPRT_HEXAHEDRON 12
#define GPRT_WEDGE 13
#define GPRT_PYRAMID 14
#define GPRT_TETRAHEDRAL_PAIR 23
// #define GPRT_PENTAGONAL_PRISM 15
// #define GPRT_HEXAGONAL_PRISM  16

int getVertexCount(uint32_t cellType) {
  switch (cellType) {
  case GPRT_TETRAHEDRON: return 4;
  case GPRT_PYRAMID: return 5;
  case GPRT_WEDGE: return 6;
  case GPRT_HEXAHEDRON: return 8;
  case GPRT_TETRAHEDRAL_PAIR: return 8;
  default: return 0;
  }
}

uint getSolidHitKind(uint32_t cellType) {
  switch (cellType) {
  case GPRT_TETRAHEDRON: return HIT_KIND_TETRAHEDRON;
  case GPRT_TETRAHEDRAL_PAIR: return HIT_KIND_TETRAHEDRON;
  case GPRT_PYRAMID: return HIT_KIND_PYRAMID;
  case GPRT_WEDGE: return HIT_KIND_WEDGE;
  case GPRT_HEXAHEDRON: return HIT_KIND_HEXAHEDRON;
  default: return 0;
  }
}

static float3 GetInitialRST(int numNodes) {
  float3 center = float3(.5);
  if (numNodes == 4) center = float3(.25);
  if (numNodes == 5) center.z = .2;
  return center;
};

static bool IsoIsContained(int numNodes, float3 rst) {
  float r = rst.x;
  float s = rst.y;
  float t = rst.z;
  bool contained = all(rst >= 0.0) && all(rst <= 1.0);
  if (numNodes == 4) {
    contained &= ((r + s + t) <= 1.0f);
  }
  if (numNodes == 6) {
    contained &= r + s <= 1.0f;
  }
  return contained;
};

// [ForceInline]
// bool
// intersectPointSolid(float3 P,      // The query point
//                     float3[8] Q,   // The nodal points defining the solid
//                     float[8] W,    // The corresponding per-node densities
//                     uint32_t nodeCount, 
//                     out float4 rstw)
// {
//   rstw = float4(0.);
//   float rcp_vol_all = rcp(determinant(float3x3(Q[1] - Q[0], Q[2] - Q[0], Q[3] - Q[0])));

//   // Determine how far the point is from each of the faces.
//   float w0 = determinant(float3x3(P - Q[1], Q[3] - Q[1], Q[2] - Q[1])) * rcp_vol_all;
//   if (w0 < 0.0) return false;
//   float w1 = determinant(float3x3(P - Q[0], Q[2] - Q[0], Q[3] - Q[0])) * rcp_vol_all;
//   if (w1 < 0.0) return false;
//   float w2 = determinant(float3x3(P - Q[0], Q[3] - Q[0], Q[1] - Q[0])) * rcp_vol_all;
//   if (w2 < 0.0) return false;
//   float w3 = determinant(float3x3(P - Q[0], Q[1] - Q[0], Q[2] - Q[0])) * rcp_vol_all;
//   if (w3 < 0.0) return false;

//   // 1 = w0 + w1 + w2 + w3, so 1 - w1 - w2 - w3 = w0
//   rstw.xyz = float3(w1, w2, w3);
//   rstw.w = dot(float4(w0, w1, w2, w3), float4(W[0], W[1], W[2], W[3]));
//   return true;
// }

#define NEWTON_ITERATIONS 5
#define CONVERGED_ERROR   1e-3
#define DIVERGED_ERROR    1e6

[Differentiable]
float[8] IsoToSupport(int numNodes, float3 rst) {
  float[8] w = { 0., 0., 0., 0., 0., 0., 0., 0. };
  float3 rstm = 1.0 - rst;
  if (numNodes == 4) {
    w[0] = 1.f - (rst.x + rst.y + rst.z);
    w[1] = rst.x;
    w[2] = rst.y;
    w[3] = rst.z;
  }
  if (numNodes == 5) {
    w[0] = rstm.x * rstm.y * rstm.z;
    w[1] = rst.x * rstm.y * rstm.z;
    w[2] = rst.x * rst.y * rstm.z;
    w[3] = rstm.x * rst.y * rstm.z;
    w[4] = rst.z;
  }
  if (numNodes == 6) {
    w[0] = (1.0 - rst.x - rst.y) * (1.0 - rst.z);
    w[1] = rst.x * (1.0 - rst.z);
    w[2] = rst.y * (1.0 - rst.z);
    w[3] = (1.0 - rst.x - rst.y) * rst.z;
    w[4] = rst.x * rst.z;
    w[5] = rst.y * rst.z;
  }
  if (numNodes == 8) {
    w[0] = rstm.x * rstm.y * rstm.z;
    w[1] = rst.x * rstm.y * rstm.z;
    w[2] = rst.x * rst.y * rstm.z;
    w[3] = rstm.x * rst.y * rstm.z;
    w[4] = rstm.x * rstm.y * rst.z;
    w[5] = rst.x * rstm.y * rst.z;
    w[6] = rst.x * rst.y * rst.z;
    w[7] = rstm.x * rst.y * rst.z;
  }
  return w;
};

// General method for inverting linear elements
bool intersectPointSolid(float3 P,      // The query point
                        float3[8] Q,   // The nodal points defining the solid
                        float[8] W,    // The corresponding per-node densities
                        uint32_t nodeCount, 
                        out float4 rstw) {
  // Initialize our canonical coordinates to be in the center of the element
  // We'll iteratively refine these with Newton's method to match the given "P"
  rstw.xyz = GetInitialRST(nodeCount);
  rstw.w = 0.0;

  float w[8];
  // Iteration for Newton's method
  bool converged = false;
  [unroll]
  for (int iteration = 0; iteration < NEWTON_ITERATIONS; iteration++) {
    // Compute support function values for the current position in "rst" space
    w = IsoToSupport(nodeCount, rstw.xyz);

    // Determine if the current cannonical coordinates accurately reflect the given query point
    float3 fcol = -P;
    [unroll] for (int i = 0; i < 8; ++i) fcol += Q[i] * w[i]; // todo, ensure this is an FMA / MAD

    // rst-derivatives
    let dwr = fwd_diff(IsoToSupport)(nodeCount, diffPair(rstw.xyz, float3(1.0, 0.0, 0.0))).getDifferential();
    let dws = fwd_diff(IsoToSupport)(nodeCount, diffPair(rstw.xyz, float3(0.0, 1.0, 0.0))).getDifferential();
    let dwt = fwd_diff(IsoToSupport)(nodeCount, diffPair(rstw.xyz, float3(0.0, 0.0, 1.0))).getDifferential();

    // Now get derivatives in world space
    float3 rcol = float3(0.f), scol = float3(0.f), tcol = float3(0.f);
    [unroll] for (int i = 0; i < 8; ++i) rcol += Q[i] * dwr[i]; // todo, ensure this is an FMA / MAD
    [unroll] for (int i = 0; i < 8; ++i) scol += Q[i] * dws[i]; // todo, ensure this is an FMA / MAD
    [unroll] for (int i = 0; i < 8; ++i) tcol += Q[i] * dwt[i]; // todo, ensure this is an FMA / MAD

    // Compute determinants and generate improvements
    float d = determinant(float3x3(rcol, scol, tcol));
    if (abs(d) < 1e-20) return false;   // Determinant too small, likely singular
    float dinv = rcp(d);

    // Newton Raphson update formula, x_new = x_old - f(x) / f'(x), but generalized to
    // multiple dimensions. fcol has principle values, while other two contain gradients,
    // so the ratio effectively calculates changes needed using Cramer's rule.
    rstw.x -= determinant(float3x3(fcol, scol, tcol)) * dinv;
    rstw.y -= determinant(float3x3(rcol, fcol, tcol)) * dinv;
    rstw.z -= determinant(float3x3(rcol, scol, fcol)) * dinv;
    if (all(abs(fcol) < CONVERGED_ERROR)) { converged = true; break;}
  }
  if (!converged) return false;

  // Check for containment of the final cannonical point location
  if (!IsoIsContained(nodeCount, rstw.xyz)) return false;

  [unroll] for (int i = 0; i < 8; ++i) rstw.w += W[i] * w[i];
  return true;
}

// Not particularly efficient at the moment, meant to be general and easy to maintain.
[shader("compute")]
[numthreads(256, 1, 1)]
void SolidBounds(uint3 DispatchThreadID: SV_DispatchThreadID, uniform SolidParameters s) {
  int primID = DispatchThreadID.x;
  if (primID >= s.count)
    return;

  uint8_t *types = s.types;
  uint8_t type = types[s.typesOffset + s.typesStride * primID];

  uint32_t numVertices = getVertexCount(type);
  float3 aabbMin = +float3(FLT_MAX);
  float3 aabbMax = -float3(FLT_MAX);
  float2 densMinMax = float2(+FLT_MAX, -FLT_MAX);

  // TODO: Try to load indices by groups of four.
  // TODO: Use aligned loads...
  // TODO: Add some validation and report an error if indices are invalid.
  uint8_t *idxstart = ((uint8_t *) s.indices) + (s.indicesOffset + s.indicesStride * primID);
  uint *indices = (uint *) idxstart;
  float4 *vertices = s.vertices;
  for (int i = 0; i < numVertices; ++i) {
    int index = indices[i];
    float4 vert = vertices[(s.verticesOffset + s.verticesStride * index) / sizeof(float4)];
    aabbMin = min(aabbMin, vert.xyz);
    aabbMax = max(aabbMax, vert.xyz);
    densMinMax = min(densMinMax.x, vert.w);
    densMinMax = max(densMinMax.y, vert.w);
  }

  uint32_t offset = s.offset;
  s.aabbs[(offset * 2) + 2 * primID] = float4(aabbMin.xyz, aabbMax.x);
  s.aabbs[(offset * 2) + 2 * primID + 1] = float4(aabbMax.yz, densMinMax);

  // printf("primID %d bounds %f %f %f %f %f %f\n", primID, aabbMin.x, aabbMin.y, aabbMin.z, aabbMax.x, aabbMax.y, aabbMax.z);
}

[shader("intersection")]
void
SolidIntersection(uniform uint32_t userData[64], uniform SolidParameters s) {
  uint primID = PrimitiveIndex();
  float4 QW[8] = { 0., 0., 0., 0., 0., 0., 0., 0. };
  uint8_t type = s.types[s.typesOffset + s.typesStride * primID];
  uint32_t numVertices = getVertexCount(type);

  // uint8_t* indices = (uint8_t *) s.indices;
  // uint8_t *idxstart = indices + (s.indicesOffset + s.indicesStride * primID);
  // uint4 *u4Indices = (uint4 *) idxstart;

  uint8_t *idxstart = ((uint8_t *)s.indices) + (s.indicesOffset + s.indicesStride * primID);
  uint *indices = (uint *)idxstart;
  float4 *vertices = s.vertices;
  for (int i = 0; i < numVertices; ++i) {
      int index = indices[i];
      float4 vert = vertices[(s.verticesOffset + s.verticesStride * index) / sizeof(float4)];
      QW[i] = vert;
      // aabbMin = min(aabbMin, vert.xyz);
      // aabbMax = max(aabbMax, vert.xyz);
      // densMinMax = min(densMinMax.x, vert.w);
      // densMinMax = max(densMinMax.y, vert.w);
  }




  // uint4 i0 = LoadAligned<16>(u4Indices + 0);

  // // printf("primID %d indices %d %d %d %d\n", primID, i0.x, i0.y, i0.z, i0.w);
  // float4 *vertices = s.vertices;

  // uint4 i0s = (s.verticesOffset + s.verticesStride * i0) / sizeof(float4);
  // // uint4 i0s = (0 + 16*i0) / sizeof(float4);

  // QW[0] = LoadAligned<16>(s.vertices + ((s.verticesOffset + s.verticesStride * i0.x)/sizeof(float4)) );
  // QW[1] = LoadAligned<16>(s.vertices + ((s.verticesOffset + s.verticesStride * i0.y)/sizeof(float4)) );
  // QW[2] = LoadAligned<16>(s.vertices + ((s.verticesOffset + s.verticesStride * i0.z)/sizeof(float4)) );
  // QW[3] = LoadAligned<16>(s.vertices + ((s.verticesOffset + s.verticesStride * i0.w)/sizeof(float4)) );
  // // printf("primID %d offset %d stride %d indices %d %d %d %d\n", primID, s.verticesOffset, s.verticesStride, i0s.x, i0s.y, i0s.z, i0s.w);

  // // printf("primID %d v0 %f %f %f %f\n", primID, QW[0].x, QW[0].y, QW[0].z, QW[0].w);

  // if (numVertices > 4) {
  //   uint4 i1 = LoadAligned<16>(u4Indices + 1);
  //   uint4 i1s = (s.verticesOffset + s.verticesStride * i1) / sizeof(float4);
  //   if (numVertices > 4) QW[4] = LoadAligned<16>(s.vertices + ((s.verticesOffset + s.verticesStride * i1.x)/sizeof(float4)) );
  //   if (numVertices > 5) QW[5] = LoadAligned<16>(s.vertices + ((s.verticesOffset + s.verticesStride * i1.y)/sizeof(float4)) );
  //   if (numVertices > 6) QW[6] = LoadAligned<16>(s.vertices + ((s.verticesOffset + s.verticesStride * i1.z)/sizeof(float4)) );
  //   if (numVertices > 7) QW[7] = LoadAligned<16>(s.vertices + ((s.verticesOffset + s.verticesStride * i1.w)/sizeof(float4)) );
  // }

  float4 rstw = float4(0.f);
  if (intersectPointSolid(ObjectRayOrigin(),   // The query point
                          { QW[0].xyz, QW[1].xyz, QW[2].xyz, QW[3].xyz, QW[4].xyz, QW[5].xyz, QW[6].xyz, QW[7].xyz },
                          { QW[0].w, QW[1].w, QW[2].w, QW[3].w, QW[4].w, QW[5].w, QW[6].w, QW[7].w },
                          numVertices,
                          rstw))
  {
    ReportHit(0.0, getSolidHitKind(type), rstw);
  }
}

// Quadratic, isoparametric cells
// GPRT_QUADRATIC_EDGE = 21,
// GPRT_QUADRATIC_TRIANGLE = 22,
// GPRT_QUADRATIC_QUAD = 23,
// GPRT_QUADRATIC_POLYGON = 36,
// #define GPRT_QUADRATIC_TETRA                  24
// #define GPRT_QUADRATIC_HEXAHEDRON             25
// #define GPRT_QUADRATIC_WEDGE                  26
// #define GPRT_QUADRATIC_PYRAMID                27
// #define GPRT_BIQUADRATIC_QUAD                 28
// #define GPRT_TRIQUADRATIC_HEXAHEDRON          29
// #define GPRT_TRIQUADRATIC_PYRAMID             37
// #define GPRT_QUADRATIC_LINEAR_QUAD            30
// #define GPRT_QUADRATIC_LINEAR_WEDGE           31
// #define GPRT_BIQUADRATIC_QUADRATIC_WEDGE      32
// #define GPRT_BIQUADRATIC_QUADRATIC_HEXAHEDRON 33
// #define GPRT_BIQUADRATIC_TRIANGLE             34

// // Cubic, isoparametric cell
// GPRT_CUBIC_LINE = 35,

// // Special class of cells formed by convex group of points
// GPRT_CONVEX_POINT_SET = 41,

// // Polyhedron cell (consisting of polygonal faces)
// GPRT_POLYHEDRON = 42,

// // Higher order cells in parametric form
// GPRT_PARAMETRIC_CURVE = 51,
// GPRT_PARAMETRIC_SURFACE = 52,
// GPRT_PARAMETRIC_TRI_SURFACE = 53,
// GPRT_PARAMETRIC_QUAD_SURFACE = 54,
// GPRT_PARAMETRIC_TETRA_REGION = 55,
// GPRT_PARAMETRIC_HEX_REGION = 56,

// // Higher order cells
// GPRT_HIGHER_ORDER_EDGE = 60,
// GPRT_HIGHER_ORDER_TRIANGLE = 61,
// GPRT_HIGHER_ORDER_QUAD = 62,
// GPRT_HIGHER_ORDER_POLYGON = 63,
// GPRT_HIGHER_ORDER_TETRAHEDRON = 64,
// GPRT_HIGHER_ORDER_WEDGE = 65,
// GPRT_HIGHER_ORDER_PYRAMID = 66,
// GPRT_HIGHER_ORDER_HEXAHEDRON = 67,

// // Arbitrary order Lagrange elements (formulated separated from generic higher order cells)
// GPRT_LAGRANGE_CURVE = 68,
// GPRT_LAGRANGE_TRIANGLE = 69,
// GPRT_LAGRANGE_QUADRILATERAL = 70,
// GPRT_LAGRANGE_TETRAHEDRON = 71,
// GPRT_LAGRANGE_HEXAHEDRON = 72,
// GPRT_LAGRANGE_WEDGE = 73,
// GPRT_LAGRANGE_PYRAMID = 74,

// // Arbitrary order Bezier elements (formulated separated from generic higher order cells)
// GPRT_BEZIER_CURVE = 75,
// GPRT_BEZIER_TRIANGLE = 76,
// GPRT_BEZIER_QUADRILATERAL = 77,
// GPRT_BEZIER_TETRAHEDRON = 78,
// GPRT_BEZIER_HEXAHEDRON = 79,
// GPRT_BEZIER_WEDGE = 80,
// GPRT_BEZIER_PYRAMID = 81,

// GPRT_NUMBER_OF_CELL_TYPES