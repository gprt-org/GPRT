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
