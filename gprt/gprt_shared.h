#pragma once

// #define COLLECT_STATS

// The higher this number is, the more primitives we can store in our tree.
#define MAX_LEVELS 10

// The higher this number is, the more clusters we're going to touch
// relative to the number of primitives.
#define BRANCHING_FACTOR 4

// With a branching factor of 4 and 12 levels (10 excluding the leaves),
// we can support up to 8 million primitives per nearest neighbor tree.

// Edit: nevermind... I had a bug with my previous minMaxDist function which was giving me some
// incorrect intuition. I'm finding now that this is very helpful for the utah teapot.
// Edit edit: Downward pruning seems to fall apart when nodes are quantized. The reason
// for why is because the nearest face might differ between the quantized and unquantized boxes.
// Nearest face only matches when quantized box is just a scaling up or down of the original.
// It's very rare that this happens, but also, the performance improvements from this aren't very good.
//
// Edit edit edit: When using OBBs, it seems like the maximal distance to the box gives good performance
// improvements.
// More updates, the "minMaxDistance" metric doesn't work for internal OBB nodes, since internal faces aren't guaranteed
// to contain primitives. Only OBB leaves can guarantee this. So, for internal nodes, we instead use the "maxDistance"
// for downward culling.
// More more updates... So long as internal OBBs are built to tightly fit the underlying primitives, then minMaxDist is
// "free game". However, it introduces some compute complexity which doesn't seem worth the effort
// #define ENABLE_DOWNAWARD_CULLING

// Enables an LBVH reference, similar to Jakob and Guthe's knn.
// Note, we use this LBVH as a top level tree for our method, so
// this is just about if we're using RT cores or not
#define ENABLE_LBVH_REFERENCE

// Uses a quantized representation for the active cluster queue
// I've bounced back and forth on if this helps or not.
//   For untruncated queries, the ultimate bottleneck is when tree traversal fails and many prims
//   are all approximately the same distance. There, its better to have an accurate queue for culling.
//   However, I suspect the issue with culling has more to do with using AABBs over something tigher fitting
// #define ENABLE_QUEUE_QUANTIZATION

// Enables oriented bounding boxes in an attempt to improve culling performance.
// Prior experiments seem to suggest that the minimum distance to an axis aligned bounding
// box tends to be a poor fit for the minimum distance to the contained primitive. This
// issue seems to become more and more extreme as primitives are clustered together.
// This mode uses a singular value decomposition to compute the orientation of a bounding box
// that best fits the underlying primitive distribution
// #define ENABLE_OBBS

// #define ENABLE_AABBS

// #define ENABLE_6DOPS

// Honestly I don't see much improvement by avoiding square roots. I don't think this is what the bottleneck is
// at least on my problems... Just going to take square roots for simplicity...
// #define USE_SQUARED_DISTS

// Minimum guaranteed PC size (might require RADV on AMD)
#define PUSH_CONSTANTS_LIMIT 256

// By limiting ourselves to the following, we can improve compatibility across
// AMD, NVIDIA and Intel with compile time checks
// Intel ARC defines a 65535 limit on workgroup size per dimension
// Both Intel and AMD define a 1024 thread limit per workgroup
#define WORKGROUP_LIMIT   65535
#define THREADGROUP_LIMIT 1024

// Some constants for the device parallel scan implementation
#define SCAN_PARTITON_SIZE   8192
#define SCAN_PARTITION       (1 << 0)
#define SCAN_SELECT          (1 << 1)
#define SCAN_SELECT_POSITIVE (1 << 2)

#if defined(__SLANG_COMPILER__)
#define __CLASS_PUBLIC__ public
#define __PRIVATE__      private
#define __PUBLIC__       public
#define __MUTATING__     [mutating]
#else
#define __CLASS_PUBLIC__
#define __PRIVATE__ private:
#define __PUBLIC__  public:
#define __MUTATING__
#endif



// NOTE, struct must be synchronized with declaration in gprt_host.h
namespace gprt {

struct Instance {
  float3x4 transform;

  uint32_t instanceCustomIndex : 24;
  uint32_t mask : 8;
  uint32_t geometryIndex : 24;
  uint32_t flags : 8;

  uint64_t accelAddress;
};

/// @brief The primitive type for a "Traversable" acceleration structure,
/// featuring a 14-DOP structure and relevant metadata to facilitate a
/// "TraceTraversalRay" similar to the proposed DXR feature here: 
///  https://github.com/microsoft/DirectX-Specs/blame/90bed3c7f2bfeeb7f9f5888713d99626c882412b/Raytracing.md#L5264-L5323
///
/// We differ from the above by making careful use of transform pairs to keep the majority 
/// of ray traversal on the hardware.
///
/// Given an OptiX multilevel traversal setup like IAS->IAS->GAS, the driver would normally use 
/// a setup like [RTCores(A->B)]->[ComputeUnits]->RTCores(C)->[ComputeUnits]. 
/// This only works well if the occurance of C is much smaller than B. However, that's rarely the case,
/// as trees grow exponentially with depth. When C is larger than C, the above schema results in 
/// overly exhaustive context switches between RT units and compute units.
///
/// Instead, we assume a pair-wise scheme: [RTCores(A->B)]->[ComputeUnits]->RTCores(B->C)->[ComputeUnits].
/// In addition to being more efficient than a 2+1 level scheme, this pairwise traversal 
/// schema can be implemented in VK/DXR by combining the recursion capabilities of TraceRay 
/// for RTCores(B->C)->[ComputeUnits] with the flexibility of RayQuery to enable [RTCores(A->B)]->[ComputeUnits].
///
/// Because recursive traversals are costly, the following structure uses a tighter bounding structure called a "KDOP",
/// which is assumed to be in centered form.
///
/// Note, this is a personal extension which builds upon a combination of RayTracing
/// and RayQuery, and lives outside the VK / DXR specification.
struct Traversable {
  inline float dotn0(float3 v) { return v.x; }   // +/-X
  inline float dotn1(float3 v) { return v.y; } // +/-Y
  inline float dotn2(float3 v) { return v.z; } // +/-Z
  inline float dotn3(float3 v) { return (v.x + v.y + v.z); } // +/- (1,1,1)
  inline float dotn4(float3 v) { return (v.x + v.y - v.z); } // +/- (1,1,-1)
  inline float dotn5(float3 v) { return (v.x - v.y + v.z); } // +/- (1,-1,1)
  inline float dotn6(float3 v) { return (v.x - v.y - v.z); } // +/- (-1,1,1)

  float2 n0hi;
  float2 n1hi;
  float2 n2hi;
  float2 n3hi;
  float2 n4hi;
  float2 n5hi;
  float2 n6hi;

  float2 tmp;
  
  /// First three columns encode three KDOP directions, whose
  /// magnitudes encode corresponding bounding extents. The
  /// fourth column represents the center point of the KDOP.
  float3x4 transform;

  /// The index to a traversable 
  uint32_t traversableLow : 24;
  uint32_t visibilityMask : 8;
  uint32_t traversableHigh : 24;
  
  /// System populated values upon build.
  uint32_t reserved1 : 8;
  uint64_t reserved2;
};

// // https://publications.anl.gov/anlpubs/2014/12/79486.pdf
// // https://www.kitware.com/modeling-arbitrary-order-lagrange-finite-elements-in-the-visualization-toolkit/
// struct Solid {
//   // 0 := tetrahedron, 1 := pyramid, 2:= wedge, 3:= hexahedron,
//   // 4 := pentagonal prism, 5 := hexagonal prism, (6 and 7 reserved for future use)
//   uint32_t type : 4;
//   // 0 := Linear elements consisting solely of corner vertices, interpolated
//   //      using generalized barycentric coordinates.
//   // 1 := Lagrange elements consisting of corners and edge midpoints,
//   //      interpolated 
//   // (For both, interpolants match the nodal positions )
//   uint32_t edgeOrder : 2;
//   uint32_t faceOrder : 2;
//   uint32_t volumeOrder;
// };

// Set this using the "setAccel" function.
// // Meant for internal GPRT use. Call "gprtGetInstance" to get an instance object referencing a BLAS.
// void __setAccel(Accel blas) {
//   accelAddress = blas.address;
//   instanceCustomIndex = blas.index;
// }

// __PUBLIC__ uint64_t getAccelAddress() { return accelAddress; }
// __PUBLIC__ uint32_t getAccelIndex() { return instanceCustomIndex; }
// __PUBLIC__ uint32_t getSBTOffset() { return instanceShaderBindingTableRecordOffset; }

// __MUTATING__
// void setSBTOffset(uint32_t TLASOffset, uint32_t BLASOffsetWithinTLAS) {
//   instanceShaderBindingTableRecordOffset = TLASOffset + BLASOffsetWithinTLAS;
// }

// // Some useful constructors
// #if defined(__SLANG_COMPILER__)
// #else

//   // Instance(Accel blas) {
//   //   mask = 0b11111111;
//   //   transform =
//   //       float3x4(float4(1.0f, 0.0f, 0.0f, 0.0f), float4(0.0f, 1.0f, 0.0f, 0.0f), float4(0.0f, 0.0f, 1.0f,
//   0.0f));
//   //   flags = 0;
//   //   accelAddress = blas.address;
//   //   instanceCustomIndex = blas.index;
//   // };

//   Instance() {
//     mask = 0b11111111;
//     transform =
//         float3x4(float4(1.0f, 0.0f, 0.0f, 0.0f), float4(0.0f, 1.0f, 0.0f, 0.0f), float4(0.0f, 0.0f, 1.0f, 0.0f));
//     flags = 0;
//     accelAddress = 0;
//     instanceCustomIndex = 0;
//   };
// #endif


// Temporarily disabling... This is blocking us from dropping the buffers descriptor array

// // Parameters to the built-in scan compute kernels
// struct ScanParams {
//   uint32_t size;
//   uint32_t flags;
//   gprt::Buffer input;
//   gprt::Buffer output;
//   Buffer state;
// };


};   // namespace gprt