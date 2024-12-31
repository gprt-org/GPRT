#pragma once 


#if !defined(__SLANG_COMPILER__)
/// Opaque type representing an acceleration structure containing solids which can be intersected by points.
struct SolidAccelerationStructure {
  const void* address;

  // Conversion constructor
  SolidAccelerationStructure(const void* addr) : address(addr) {}
};

/// Opaque type representing an acceleration structure containing surfaces which can be intersected by rays.
struct SurfaceAccelerationStructure {
  const void * address;

  // Conversion constructor
  SurfaceAccelerationStructure(const void *addr) : address(addr) {}
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

/// Traces a sphere through the acceleration structure.
/// @param AccelerationStructure The acceleration structure to traverse
/// @param RayFlags Flags controlling ray behavior
/// @param InstanceInclusionMask Mask for filtering instance visibility
/// @param RayContributionToHitGroupIndex Offset for hit group indexing
/// @param MultiplierForGeometryContributionToHitGroupIndex Multiplier for geometry-based hit group indexing
/// @param MissShaderIndex Index of the miss shader to execute if no hit is found
/// @param Origin The query point origin to trace
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
    uint                          MultiplierForGeometryContributionToHitGroupIndex,
    uint                          MissShaderIndex,
    PointDesc                     Point,
    inout payload_t               Payload)
{
  RaytracingAccelerationStructure accel = getAccelFromAddress(AccelerationStructure.address);
  TraceRay(accel, RayFlags, InstanceInclusionMask, RayContributionToHitGroupIndex, MultiplierForGeometryContributionToHitGroupIndex, MissShaderIndex, { Point.Origin, 0.0, float3(1.0), 0.0 }, Payload);
}
#endif
