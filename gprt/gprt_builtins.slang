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
  public __init(uint64_t addr) { address = addr; }
};


/// Opaque type representing an acceleration structure containing surfaces which can be intersected by rays.
public struct SurfaceAccelerationStructure {
    internal uint64_t address;
    public __init() { address = 0; }
    public __init(uint64_t addr) { address = addr; }
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

// TODO...
public void ForwardRay(RaytracingAccelerationStructure AccelerationStructure,
                       uint RayContributionToHitGroupIndex,
                       uint MultiplierForGeometryContributionToHitGroupIndex,
                       uint MissShaderIndex,
                       RayDesc Ray)
{

}

/// Traces a ray through the acceleration structure similar to `TraceRay`,
/// but as rays intersect the bounds of instances, a "traversable" shader is invoked.
/// These traversable shaders decide how to forward the ray, potentially to another
/// acceleration structure or to drop the ray entirely. This allows for multiple
/// levels of instancing and dynamic LOD selection, but comes at the cost of 
/// additional returns from ray tracing cores to compute cores.
/// 
/// This is meant to mimic the proposed "Traversal Shaders" feature from the original 
/// DXR specification: 
//// https://github.com/microsoft/DirectX-Specs/blame/90bed3c7f2bfeeb7f9f5888713d99626c882412b/Raytracing.md#L5264-L5323
///
/// Unlike the original proposal, 
///
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
[require(cuda_glsl_hlsl_spirv, raytracing_raygen_closesthit_miss, rayquery, spvShaderInvocationReorderNV)]
public void TraceTraversalRay<payload_t>(
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
      if (all(Ray.Direction == 0.f)) printf("(Thread Index: %d %d %d): ERROR in TraceTraversalRay, RayDesc::Direction is all zero!\n", RayIndex.x, RayIndex.y, RayIndex.z);
      if (any(isnan(Ray.Direction))) printf("(Thread Index: %d %d %d): ERROR in TraceTraversalRay, RayDesc::Direction contans a nan component!\n", RayIndex.x, RayIndex.y, RayIndex.z);
    }
#endif
    TraceRay(accel, RayFlags, InstanceInclusionMask, RayContributionToHitGroupIndex, 0, MissShaderIndex, Ray, Payload);
}

#endif


// ## Traversal shaders

// Bottom-level acceleration structures currently support two types:
// triangle geometry or procedural geometry (via AABBs).

// We could add a third type of bottom-level acceleration structure:
// traversal. The goal of this node would be to make some procedural
// decision about what other acceleration structure (if any) to forward the
// ray that arrived at the node. This could allow a dynamic LOD selection,
// for example, by choosing a separate acceleration structure to forward
// the ray to that contains appropriate LOD geometry.

// A traversal node would be defined as AABBs (like procedural geometry)
// with an index into a shader table where a traversal shader is to be
// found. When one of these AABBs is hit by a ray the referenced traversal
// shader is invoked.  The traversal shader might be very simple, perhaps
// only choosing to "forward" the ray into another acceleration structure
// of its choice (or drop the ray).

// The following intrinsic looks a like [TraceRay()](#traceray) with fewer
// parameters:
// void ForwardRay(RaytracingAccelerationStructure AccelerationStructure,
//               uint RayContributionToHitGroupIndex,
//               uint MultiplierForGeometryContributionToHitGroupIndex,
//               uint MissShaderIndex,
//               RayDesc Ray);

// Ray processing for the forwarded ray behaves just like
// [TraceRay()](#traceray), with some exceptions.  The closest hit shader
// is only invoked for the lowest T across the original and any forwarded
// rays.  If [AcceptHitAndEndSearch()](#accepthitandendsearch) is invoked
// in the forwarded ray, that ends searching in the parent ray too
// (followed by closest hit selection as usual).
// - There's no payload in the parameter list, as that is forwarded from
//   the original ray.
//   - The traversal shader itself may want to be able to inspect the
//     payload (read-only?)
// - Other parameters to [TraceRay()](#traceray) such as payload, ray
//   flags, instance cull mask get forwarded with the new ray.
// - The ray can be defined arbitrarily, likely to be a transformed
//   version of the parent ray
// - There would be a user declared recursion limit on ForwardRay
//   recursion, perhaps separate from the user declared TraceRay
//   recursion limit.
//   - And/or perhaps ForwardRay specifies a **bottom**-level
//     acceleration structure to use (including defining instance
//     data).