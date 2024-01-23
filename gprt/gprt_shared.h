#pragma once

// Minimum guaranteed PC size (might require RADV on AMD)
#define PUSH_CONSTANTS_LIMIT 256

// By limiting ourselves to the following, we can improve compatibility across 
// AMD, NVIDIA and Intel with compile time checks
// Intel ARC defines a 65535 limit on workgroup size per dimension
// Both Intel and AMD define a 1024 thread limit per workgroup
#define WORKGROUP_LIMIT 65535
#define THREADGROUP_LIMIT 1024

// NOTE, struct must be synchronized with declaration in gprt_host.h
namespace gprt {
    typedef uint32_t Texture;
    typedef uint32_t Sampler;

    // Shared between Slang and C++
    struct Buffer {
        // eventually will become address as features progress
        uint32_t index;
        uint32_t temp; 
    };

    struct Accel {
        uint64_t address;
    };
};