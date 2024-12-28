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


struct Texture {
  uint32_t index;
};

struct Sampler {
  uint32_t index;
};

// Update: now, we simply use Slang's pointer type for buffers
// Shared between Slang and C++
// struct Buffer {
//   // eventually will become address as features progress
//   uint32_t index;
//   uint32_t temp;
// };

/** The reference to the acceleration structure.
 * @note this handle is subject to change if the memory backing the accel changes.
 */
struct Accel {
  // A VkDeviceAddress
  uint64_t address;

  // A host-side index for SBT management
  uint32_t index;
  uint32_t numGeometries;
};

struct Instance {
  float3x4 transform;

  uint32_t __gprtInstanceIndex : 24;
  uint32_t mask : 8;
  uint32_t __gprtSBTOffset : 24;
  uint32_t flags : 8;

  uint64_t __gprtAccelAddress;
};

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