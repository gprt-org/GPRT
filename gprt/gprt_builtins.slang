#pragma once 


#if !defined(__SLANG_COMPILER__)
/// Opaque type representing an acceleration structure containing solids which can be intersected by points.
struct SolidAccelerationStructure {
  const void* address;

  // Conversion constructor
  SolidAccelerationStructure(const void *addr) : address(addr) {}
  SolidAccelerationStructure() : address(nullptr) {}
};

/// Opaque type representing an acceleration structure containing surfaces which can be intersected by rays.
struct SurfaceAccelerationStructure {
  const void * address;

  // Conversion constructor
  SurfaceAccelerationStructure(const void *addr) : address(addr) {}
  SurfaceAccelerationStructure() : address(nullptr) {}
};
#endif 

#if defined(__SLANG_COMPILER__)

/// Opaque type representing an acceleration structure containing solids which can be intersected by points.
public struct SolidAccelerationStructure {
  internal uint64_t address;
  public __init() { address = 0; }
};


/// Opaque type representing an acceleration structure containing surfaces which can be intersected by rays.
public struct SurfaceAccelerationStructure {
    internal uint64_t address;
    public __init() { address = 0; }
};

// Keep below in sync with GPRTSolidTypes...

///
public static uint HIT_KIND_TETRAHEDRON = 10;
public static uint HIT_KIND_HEXAHEDRON = 12;
public static uint HIT_KIND_WEDGE = 13;
public static uint HIT_KIND_PYRAMID = 14;


public RaytracingAccelerationStructure
getAccelFromAddress(uint64_t address) {
  return spirv_asm {
    result: $$RaytracingAccelerationStructure = OpConvertUToAccelerationStructureKHR $address
  };
}

/// Describes a point for traversal through an acceleration structure.
/// @category raytracing
public struct PointDesc
{
  /// The center of the query sphere in world space.
  public float3 Origin;

  // /// Maximum radius away from the sphere origin to consider intersections.
  // public float TMax;

  public __init() {};
};

/// Traces a point through the acceleration structure.
/// @param AccelerationStructure The acceleration structure to traverse
/// @param RayFlags Flags controlling ray behavior
/// @param InstanceInclusionMask Mask for filtering instance visibility
/// @param RayContributionToHitGroupIndex Offset for hit group indexing
/// @param MissShaderIndex Index of the miss shader to execute if no hit is found
/// @param Point The query point origin to trace
/// @param Payload Structure for passing data between shaders
/// @remarks Core ray tracing function for initiating traversal
/// @category raytracing
[ForceInline]
[require(cuda_glsl_hlsl_spirv, raytracing_raygen_closesthit_miss)]
public void TracePoint<payload_t>(
    SolidAccelerationStructure    AccelerationStructure,
    uint                          RayFlags,
    uint                          InstanceInclusionMask,
    uint                          RayContributionToHitGroupIndex,
    uint                          MissShaderIndex,
    PointDesc                     Point,
    inout payload_t               Payload)
{
  RaytracingAccelerationStructure accel = getAccelFromAddress(AccelerationStructure.address);
  TraceRay(accel, RayFlags, InstanceInclusionMask, RayContributionToHitGroupIndex, 0, MissShaderIndex, { Point.Origin, 0.0, float3(1.0), 0.0 }, Payload);
}

/// Traces a ray through the acceleration structure.
/// @param AccelerationStructure The acceleration structure to traverse
/// @param RayFlags Flags controlling ray behavior
/// @param InstanceInclusionMask Mask for filtering instance visibility
/// @param RayContributionToHitGroupIndex Offset for hit group indexing
/// @param MissShaderIndex Index of the miss shader to execute if no hit is found
/// @param Ray Description of the ray to trace
/// @param Payload Structure for passing data between shaders
/// @remarks Core ray tracing function for initiating traversal
/// @category raytracing
[ForceInline]
[require(cuda_glsl_hlsl_spirv, raytracing_raygen_closesthit_miss)]
public void TraceRay<payload_t>(
    SurfaceAccelerationStructure    AccelerationStructure,
    uint                            RayFlags,
    uint                            InstanceInclusionMask,
    uint                            RayContributionToHitGroupIndex,
    uint                            MissShaderIndex,
    RayDesc                         Ray,
    inout payload_t                 Payload)
{
    RaytracingAccelerationStructure accel = getAccelFromAddress(AccelerationStructure.address);
#ifdef __DEBUG__
    {
      uint3 RayIndex = DispatchRaysIndex();
      // uint3 DispatchDims = DispatchRaysDimensions();
      if (all(Ray.Direction == 0.f)) printf("(Thread Index: %d %d %d): ERROR in TraceRay, RayDesc::Direction is all zero!\n", RayIndex.x, RayIndex.y, RayIndex.z);
      if (any(isnan(Ray.Direction))) printf("(Thread Index: %d %d %d): ERROR in TraceRay, RayDesc::Direction contans a nan component!\n", RayIndex.x, RayIndex.y, RayIndex.z);
    }
#endif
    TraceRay(accel, RayFlags, InstanceInclusionMask, RayContributionToHitGroupIndex, 0, MissShaderIndex, Ray, Payload);
}


/// Traces a point through the acceleration structure.
/// @param AccelerationStructure The acceleration structure to traverse
/// @param RayFlags Flags controlling ray behavior
/// @param InstanceInclusionMask Mask for filtering instance visibility
/// @param RayContributionToHitGroupIndex Offset for hit group indexing
/// @param MissShaderIndex Index of the miss shader to execute if no hit is found
/// @param Ray Description of the ray to trace
/// @param Payload Structure for passing data between shaders
/// @remarks Core ray tracing function for initiating traversal
/// @category raytracing
[ForceInline]
[require(cuda_glsl_hlsl_spirv, raytracing_raygen_closesthit_miss, hlsl_nvapi)]
public void TraceMotionRay<payload_t>(
    SurfaceAccelerationStructure    AccelerationStructure,
    uint                            RayFlags,
    uint                            InstanceInclusionMask,
    uint                            RayContributionToHitGroupIndex,
    uint                            MissShaderIndex,
    RayDesc                         Ray,
    float                           Time,
    inout payload_t                 Payload)
{
    RaytracingAccelerationStructure accel = getAccelFromAddress(AccelerationStructure.address);
#ifdef __DEBUG__
    {
      uint3 RayIndex = DispatchRaysIndex();
      // uint3 DispatchDims = DispatchRaysDimensions();
      if (all(Ray.Direction == 0.f)) printf("(Thread Index: %d %d %d): ERROR in TraceRay, RayDesc::Direction is all zero!\n", RayIndex.x, RayIndex.y, RayIndex.z);
      if (any(isnan(Ray.Direction))) printf("(Thread Index: %d %d %d): ERROR in TraceRay, RayDesc::Direction contans a nan component!\n", RayIndex.x, RayIndex.y, RayIndex.z);
    }
#endif
    TraceMotionRay(accel, RayFlags, InstanceInclusionMask, RayContributionToHitGroupIndex, 0, MissShaderIndex, Ray, Time, Payload);
}
#endif
