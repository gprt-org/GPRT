#include "gprt_fallbacks.h"

[ForceInline]
T
LoadAligned<int alignment, T>(T *ptr) {
  return spirv_asm {
          result = OpLoad $$T $ptr Aligned !alignment;
  };
}

[ForceInline]
float
flipsign(float v, bool flag) {
  return (flag) ? -v : v;
}

[ForceInline]
bool
intersect_ray_lss(float3 rayOrig, float3 rayDir,              // input ray
                  float3 P0, float3 P1, float r0, float r1,   // input curve
                  bool endcap0,                               // true: endcap0 enabled, false: endcap0 disabled
                  bool endcap1,                               // true: endcap1 enabled, false: endcap1 disabled
                  float rayTmin, float rayTmax,               // input interval
                  bool exitTest,                              // false: return entry hits, true: return exit hits
                  out float t, out float u                    // output ray param (t), curve param (u)
) {
  t = 0.0;
  u = 0.0;                   // initialize to 0 for now
  const float3 D = rayDir;   // this is purely an alias name, optional
                             // Use the midpoint to anchor everything - this gives the best precision balance
  const float3 Pm = 0.5f * (P0 + P1);
  const float rm = 0.5f * (r0 + r1);
  // a is our first conic coefficient for the t^2 term
  const float a = dot(D, D);   // This is 1 when ray is normalized
  const float rcpA = 1.f / a;
  // precision fix -- push the ray to be as close as possible to the curve.
  const float dpa = dot(D, Pm - rayOrig) * rcpA;
  rayOrig += dpa * D;
  const float3 O = rayOrig - Pm;   // Recalculate since rayOrig moved
  const float3 A = P1 - P0;        // curve axis
  const float rd = r1 - r0;        // radius delta
                                   // Compute the conic coefficients.
                                   // curve: C = Pm + u A, r = rm + u rd
                                   //
                                   // We'll set:
                                   // Pm = 1/2 (P0 + P1)
                                   // rm = 1/2 (r0 + r1)
                                   // O = rayOrig - Pm
                                   //
                                   // ray: P = rayOrig + t rayDir
                                   //
  // sphere: |P-C| = r
  //
  // So our cone equation is: | ray(t) - curve(u) | = radius(u)
  //
  // Expanding:
  // | rayOrig + t V - Pm - u A |^2 = (rm + u rd)^2
  // Gives us:
  // ttV.V - 2tuA.V + uu(A.A - rd rd) - 2t(-O.V) - 2u (A.O + rd rm) + O.O - rm rm = 0
  //
  // Our canonical conic equation is at^2 - 2btu + cu^2 - 2dt - 2eu + f = 0
  // The single letter variables a through f below are these coefficients
  // a is already up top
  const float b = dot(A, D);
  const float c = dot(A, A) - rd * rd;
  const float d = dot(D, O);
  const float e = dot(A, O) + rm * rd;
  const float f = dot(O, O) - rm * rm;
  // Now solve our conic for u(t):
  // att - 2but + cuu - 2dt - 2eu + f = 0
  // Group the u terms
  // + cuu - 2u(bt + e) + (att - 2dt + f) = 0
  //
  // Au = c
  // Bu = bt + e
  // Cu = att - 2dt + f
  //
  // u(t) = [ Bu +/- sqrt(BuBu - AuCu) ] / Au
  //
  // solve for discrim = 0 - we are looking for the extremmal values of u,
  // which happen to coincide with the t values we're looking for!
  // so, Bu Bu - Au Cu = 0
  // tt(bb - ca) -2t(-be -cd) + (ee - cf) = 0
  //
  // At = bb-ca
  // Bt = -be-cd
  // Ct = ee-cf
  // t = [ Bt +/- sqrt(BtBt-AtCt) ] / At
  const float At = b * b - a * c;
  const float Bt = b * e - c * d;
  const float Ct = e * e - c * f;
  const float BtBt = Bt * Bt;
  const float AtCt = At * Ct;
  // sphere-degenerate and shell-degenerate handling
  const bool sphereDegenerate = (A.x == 0) && (A.y == 0) && (A.z == 0) && (rd == 0);
  const bool useP0Sphere = !endcap1;
  const bool cullSphere = sphereDegenerate && !endcap0 && !endcap1;
  const bool shellDegenerate = (c <= 0) && (rd != 0);
  const bool endcap0Valid = (r0 > 0) && endcap0;
  const bool endcap1Valid = (r1 > 0) && endcap1;
  const bool shellP0Outer = (rd < 0);
  const bool shellP1Outer = (rd > 0);
  const bool useP0Shell = endcap0Valid && (shellP0Outer || !endcap1Valid);
  const bool useP1Shell = endcap1Valid && (shellP1Outer || !endcap0Valid);
  const bool cullShell = shellDegenerate && !useP0Shell && !useP1Shell;
  // Infinite cone MISS
  const bool badCurve = ((r0 <= 0.f) && (r1 <= 0.f)) || cullShell || cullSphere || isnan(r0) || isnan(r1) ||
                        isnan(P0.x) || isnan(P0.y) || isnan(P0.z) || isnan(P1.x) || isnan(P1.y) || isnan(P1.z) ||
                        isinf(r0) || isinf(r1) || isinf(P0.x) || isinf(P0.y) || isinf(P0.z) || isinf(P1.x) ||
                        isinf(P1.y) || isinf(P1.z);

  if ((!sphereDegenerate && !shellDegenerate && BtBt < AtCt) || badCurve) {
    return false;
  }
  // Used in distance pipe to override useP0
  const bool isDegenerate = sphereDegenerate || shellDegenerate;
  const bool useP0Degenerate = sphereDegenerate ? useP0Sphere : useP0Shell;
  const float sqdiscrim = sqrt(BtBt - AtCt);
  const float rcpAt = 1.f / At;
  const float t0 = (-Bt + flipsign(sqdiscrim, exitTest)) * rcpAt;   // t0 == cone hit point, (exitTest) ? exit : entry
  // Calculate u (parametric length along curve)
  // u is in range [-.5, .5] when on the mid-section between endcaps
  const float rcpC = 1.f / c;
  float u0 = (b * t0 + e) * rcpC;
  // Mid-section cone HIT
  // NOTE: Below "if" is always FALSE for shells/spheres (i.e. midsection hit impossible)
  bool radiusIsNegative = u0 * rd < -rm;
  if (abs(u0) <= 0.5f && !radiusIsNegative && !isDegenerate) {
    float hitt = t0 + dpa;
    float hitu = u0 + 0.5f;
    if (hitt <= rayTmin || hitt >= rayTmax)
      return false;
    t = hitt;
    u = hitu;
    return true;
  }
  // Determine which end-cap to test
  bool useP0;
  if (isDegenerate) {
    useP0 = useP0Degenerate;
  } else {
    useP0 = ((u0 < 0.f) ^ radiusIsNegative);
  }
  // Skip endcap testing if they are disabled
  if ((useP0 && !endcap0Valid) || (!useP0 && !endcap1Valid))
    return false;
  // What follows is a simple sphere intersection with the appropriate end-cap sphere,
  // substituting values we already have anywhere possible.
  const float Bs =
      -d + flipsign(b, useP0) * 0.5f;   // Bs = dot(E, D) ==> dot(O,D) - dot(A, D) * endu, where E = O - A * endu;
  const float BsBs = Bs * Bs;
  const float Cs = f + 0.25f * c - flipsign(e, useP0);   // dot(E, E) - endr*endr;
  const float AsCs = a * Cs;
  // end-cap MISS
  if (BsBs < AsCs) {
    return false;
  }
  // end-cap Testing
  const float sqSdiscrim = sqrt(BsBs - AsCs);
  const float minEndt = (Bs - flipsign(sqSdiscrim, exitTest)) * rcpA;
  // end-cap HIT
  // The following u computation can be used by the shader to reconstruct the endcap
  u = (useP0) ? 0.f : 1.f;
  t = minEndt + dpa;
  // Unordered LEU/GEU comparisons
  if (t <= rayTmin || t >= rayTmax)
    return false;
  return true;
  /*
  Notes:
  - u is in the range (0.0f, 1.0f) for midsection hits
  - u is 0.0f for endcap 0 hits
  - u is 1.0f for endcap 1 hits
  */
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
  if (intersect_ray_lss(ObjectRayOrigin(), ObjectRayDirection(),   // input ray
                        P0.xyz, P1.xyz, P0.w, P1.w,                // input curve
                        endcap0,                                   // true: endcap0 enabled, false: endcap0 disabled
                        endcap1, RayTMin(), RayTCurrent(),         // input interval
                        exitTest, t, u))                           // output ray param (t), curve param (u)
  {
    // HitKindLssPrimitiveNV eventually...
    ReportHit(t, /*hitKind*/ 0, u);
  }
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

  float3 aabbMin = +float3(FLT_MAX);
  float3 aabbMax = -float3(FLT_MAX);

  aabbMin = min(aabbMin, (P0.xyz - max(P0.w, 0.0)));
  aabbMin = min(aabbMin, (P1.xyz - max(P1.w, 0.0)));

  aabbMax = max(aabbMax, (P0.xyz + max(P0.w, 0.0)));
  aabbMax = max(aabbMax, (P1.xyz + max(P1.w, 0.0)));

  uint32_t offset = lss.offset;
  lss.aabbs[(offset * 2) + 2 * primID] = aabbMin;
  lss.aabbs[(offset * 2) + 2 * primID + 1] = aabbMax;
}

[shader("intersection")]
void
SphereIntersection(uniform uint32_t userData[64], uniform SphereParameters s) {
  uint primID = PrimitiveIndex();
  float4 P0 = LoadAligned<16>(s.vertices + primID);
  bool exitTest = bool(s.exitTest);

  float t, u;
  if (intersect_ray_lss(ObjectRayOrigin(), ObjectRayDirection(),   // input ray
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
void
SphereBounds(uint3 DispatchThreadID: SV_DispatchThreadID, uniform SphereBoundsParameters s) {
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
