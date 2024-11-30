#pragma once

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
    typedef uint32_t Texture;
    typedef uint32_t Sampler;

    // Shared between Slang and C++
    struct Buffer {
        // eventually will become address as features progress
        uint32_t index;
        uint32_t size; 
    };

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

// Parameters to the built-in scan compute kernels
struct ScanParams {
  uint32_t size;
  uint32_t flags;
  Buffer input;
  Buffer output;
  Buffer state;
};
};   // namespace gprt