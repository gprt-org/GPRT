/**
 * @file gprt_device.h
 * @author Nate Morrical (natemorrical@gmail.com)
 * @brief This file defines the device-side interface for the general purpose
 * raytracing toolkit.
 * @version 0.1
 * @date 2022-11-03
 *
 * @copyright Copyright (c) 2022
 * MIT License
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#pragma once

// #ifndef FLT_MAX 
// #define FLT_MAX 3.402823466e+38
// #endif
// #ifndef FLT_MIN
// #define FLT_MIN 1.175494351e-38
// #endif

#define DBL_DECIMAL_DIG  17                      // # of decimal digits of rounding precision
#define DBL_DIG          15                      // # of decimal digits of precision
#define DBL_EPSILON      2.2204460492503131e-016 // smallest such that 1.0+DBL_EPSILON != 1.0
#define DBL_HAS_SUBNORM  1                       // type does support subnormal numbers
#define DBL_MANT_DIG     53                      // # of bits in mantissa
#define DBL_MAX          1.7976931348623158e+308 // max value
#define DBL_MAX_10_EXP   308                     // max decimal exponent
#define DBL_MAX_EXP      1024                    // max binary exponent
#define DBL_MIN          2.2250738585072014e-308 // min positive value
#define DBL_MIN_10_EXP   (-307)                  // min decimal exponent
#define DBL_MIN_EXP      (-1021)                 // min binary exponent
#define _DBL_RADIX       2                       // exponent radix
#define DBL_TRUE_MIN     4.9406564584124654e-324 // min positive value

#define FLT_DECIMAL_DIG  9                       // # of decimal digits of rounding precision
#define FLT_DIG          6                       // # of decimal digits of precision
#define FLT_EPSILON      1.192092896e-07F        // smallest such that 1.0+FLT_EPSILON != 1.0
#define FLT_HAS_SUBNORM  1                       // type does support subnormal numbers
#define FLT_GUARD        0
#define FLT_MANT_DIG     24                      // # of bits in mantissa
#define FLT_MAX          3.402823466e+38F        // max value
#define FLT_MAX_10_EXP   38                      // max decimal exponent
#define FLT_MAX_EXP      128                     // max binary exponent
#define FLT_MIN          1.175494351e-38F        // min normalized positive value
#define FLT_MIN_10_EXP   (-37)                   // min decimal exponent
#define FLT_MIN_EXP      (-125)                  // min binary exponent
#define FLT_NORMALIZE    0
#define FLT_RADIX        2                       // exponent radix
#define FLT_TRUE_MIN     1.401298464e-45F        // min positive value

#define LDBL_DIG         DBL_DIG                 // # of decimal digits of precision
#define LDBL_EPSILON     DBL_EPSILON             // smallest such that 1.0+LDBL_EPSILON != 1.0
#define LDBL_HAS_SUBNORM DBL_HAS_SUBNORM         // type does support subnormal numbers
#define LDBL_MANT_DIG    DBL_MANT_DIG            // # of bits in mantissa
#define LDBL_MAX         DBL_MAX                 // max value
#define LDBL_MAX_10_EXP  DBL_MAX_10_EXP          // max decimal exponent
#define LDBL_MAX_EXP     DBL_MAX_EXP             // max binary exponent
#define LDBL_MIN         DBL_MIN                 // min normalized positive value
#define LDBL_MIN_10_EXP  DBL_MIN_10_EXP          // min decimal exponent
#define LDBL_MIN_EXP     DBL_MIN_EXP             // min binary exponent
#define _LDBL_RADIX      _DBL_RADIX              // exponent radix
#define LDBL_TRUE_MIN    DBL_TRUE_MIN            // min positive value

#define HLF_EPSILON      9.77e-04h               // smallest such that 1.0h+HLF_EPSILON != 1.0h
#define HLF_MAX          65504.h                 // max value

#define DECIMAL_DIG      DBL_DECIMAL_DIG

#include "gprt_shared.h"

// Descriptor binding, then set number.
// Currently, 0, N is used for textures
// Then, 1, N is used for record data passed
// to compute and raster programs.
//
// Note, it's assumed that at address 0 into all these descriptors, we will have some "default"
[[vk::binding(0, 0)]] SamplerState samplers[];
[[vk::binding(0, 1)]] Texture1D texture1Ds[];
[[vk::binding(0, 2)]] Texture2D texture2Ds[];
[[vk::binding(0, 3)]] Texture3D texture3Ds[];

// globally coherent here causes memory barriers and syncs to flush data across
// the entire GPU such that other groups can see writes. Without this specifier,
// a memory barrier or sync will only flush a UAV within the current group.
[[vk::binding(0, 4)]] globallycoherent RWByteAddressBuffer buffers[];

namespace gprt {
inline uint32_t
make_8bit(const float f) {
  return min(255, max(0, int(f * 256.f)));
}

inline uint32_t
make_rgba(float3 color) {
  float gamma = 2.2;
  color = pow(color, float3(1.0f / gamma, 1.0f / gamma, 1.0f / gamma));
  return (make_8bit(color.x) << 0) + (make_8bit(color.y) << 8) + (make_8bit(color.z) << 16) + (0xffU << 24);
}

inline uint32_t
make_bgra(float3 color) {
  float gamma = 2.2;
  color = pow(color, float3(1.0f / gamma, 1.0f / gamma, 1.0f / gamma));
  return (make_8bit(color.z) << 0) + (make_8bit(color.y) << 8) + (make_8bit(color.x) << 16) + (0xffU << 24);
}

inline uint32_t
make_bgra(const float4 color) {
  return (make_8bit(color.z) << 0) + (make_8bit(color.y) << 8) + (make_8bit(color.x) << 16) +
         (make_8bit(color.w) << 24);
}

float4 over(float4 a, float4 b) {
  float4 result;
  result.a = a.a + b.a * (1.f - a.a);
  if (result.a == 0.f)
    return a; // avoid NaN
  result.rgb = (a.rgb * a.a + b.rgb * b.a * (1.f - a.a)) / result.a;
  return result;
}

// struct Buffer {
//   uint64_t x;
//   uint64_t y;
// };

template <typename T>
T
load(in Buffer buffer) {
  return vk::RawBufferLoad<T>(buffer.x);
}

template <typename T>
T
load(in Buffer buffer, uint64_t index) {
  return vk::RawBufferLoad<T>(buffer.x + index * sizeof(T));
}

template <typename T>
T
loadRaw(in Buffer buffer, uint64_t address) {
  return vk::RawBufferLoad<T>(buffer.x + address);
}

template <typename T>
void
store(in Buffer buffer, uint64_t index, in T value) {
  vk::RawBufferStore<T>(buffer.x + index * sizeof(T), value);
}

template <typename T>
void
storeRaw(in Buffer buffer, uint64_t address, in T value) {
  vk::RawBufferStore<T>(buffer.x + address, value);
}

// note, below  atomics are translated from
// here: https://github.com/treecode/Bonsai/blob/master/runtime/profiling/derived_atomic_functions.h

float
atomicMin32f(in Buffer buffer, uint32_t index, float value) {
  uint ret_i = asuint(buffers[buffer.y].Load<float>(index * sizeof(float)));
  while (value < asfloat(ret_i)) {
    uint old = ret_i;
    buffers[buffer.y].InterlockedCompareExchange(index * sizeof(float), old, asuint(value), ret_i);
    if (ret_i == old)
      break;
  }
  return asfloat(ret_i);
}

float3 atomicMin32f(in Buffer buffer, uint32_t index, float3 value) {
  float3 result;
  result.x = atomicMin32f(buffer, index * 3 + 0, value.x);
  result.y = atomicMin32f(buffer, index * 3 + 1, value.y);
  result.z = atomicMin32f(buffer, index * 3 + 2, value.z);
  return result;
}

float4 atomicMin32f(in Buffer buffer, uint32_t index, float4 value) {
  float4 result;
  result.x = atomicMin32f(buffer, index * 4 + 0, value.x);
  result.y = atomicMin32f(buffer, index * 4 + 1, value.y);
  result.z = atomicMin32f(buffer, index * 4 + 2, value.z);
  result.w = atomicMin32f(buffer, index * 4 + 3, value.w);
  return result;
}

float
atomicMax32f(in Buffer buffer, uint32_t index, float value) {
  uint ret_i = asuint(buffers[buffer.y].Load<float>(index * sizeof(float)));
  while (value > asfloat(ret_i)) {
    uint old = ret_i;
    buffers[buffer.y].InterlockedCompareExchange(index * sizeof(float), old, asuint(value), ret_i);
    if (ret_i == old)
      break;
  }
  return asfloat(ret_i);
}

float3 atomicMax32f(in Buffer buffer, uint32_t index, float3 value) {
  float3 result;
  result.x = atomicMax32f(buffer, index * 3 + 0, value.x);
  result.y = atomicMax32f(buffer, index * 3 + 1, value.y);
  result.z = atomicMax32f(buffer, index * 3 + 2, value.z);
  return result;
}

float4 atomicMax32f(in Buffer buffer, uint32_t index, float4 value) {
  float4 result;
  result.x = atomicMax32f(buffer, index * 4 + 0, value.x);
  result.y = atomicMax32f(buffer, index * 4 + 1, value.y);
  result.z = atomicMax32f(buffer, index * 4 + 2, value.z);
  result.w = atomicMax32f(buffer, index * 4 + 3, value.w);
  return result;
}

float
atomicAdd32f(in Buffer buffer, uint32_t index, float value) {
  uint old, newint;
  uint ret_i = asuint(buffers[buffer.y].Load<float>(index * sizeof(float)));
  do {
    old = ret_i;
    newint = asuint(asfloat(old) + value);
    buffers[buffer.y].InterlockedCompareExchange(index * sizeof(float), old, newint, ret_i);
  } while (ret_i != old);

  return asfloat(ret_i);
}

float3
atomicAdd32f(in Buffer buffer, uint32_t index, float3 value) {
  float3 result;
  result.x = atomicAdd32f(buffer, index * 3 + 0, value.x);
  result.y = atomicAdd32f(buffer, index * 3 + 1, value.y);
  result.z = atomicAdd32f(buffer, index * 3 + 2, value.z);
  return result;
}

float3x3
atomicAdd32f(in Buffer buffer, uint32_t index, float3x3 value) {
  float3x3 result;
  result._11 = atomicAdd32f(buffer, index * 9 + 0, value._11);
  result._12 = atomicAdd32f(buffer, index * 9 + 1, value._12);
  result._13 = atomicAdd32f(buffer, index * 9 + 2, value._13);

  result._21 = atomicAdd32f(buffer, index * 9 + 3, value._21);
  result._22 = atomicAdd32f(buffer, index * 9 + 4, value._22);
  result._23 = atomicAdd32f(buffer, index * 9 + 5, value._23);

  result._31 = atomicAdd32f(buffer, index * 9 + 6, value._31);
  result._32 = atomicAdd32f(buffer, index * 9 + 7, value._32);
  result._33 = atomicAdd32f(buffer, index * 9 + 8, value._33);
  return result;
}

uint32_t
atomicAdd(in Buffer buffer, uint32_t index, uint32_t value) {
  uint32_t old;
  buffers[buffer.y].InterlockedAdd(index * sizeof(uint32_t), value, old);
  return old;
}

uint32_t
atomicOr(in Buffer buffer, uint32_t index, uint32_t value) {
  uint32_t old;
  buffers[buffer.y].InterlockedOr(index * sizeof(uint32_t), value, old);
  return old;
}

// workaround for a bug with raw buffer load / store...
template <typename T>
T 
atomicLoad(in Buffer buffer, uint32_t index) {
  return buffers[buffer.y].Load<T>(index * sizeof(T));
}

template <typename T>
void 
atomicStore(in Buffer buffer, uint32_t index, in T value) {
  buffers[buffer.y].Store<T>(index * sizeof(T), value);
}

[[vk::ext_instruction(4447)]] RaytracingAccelerationStructure getAccelHandle(uint64_t ptr);

RaytracingAccelerationStructure
getAccelHandle(Accel accel) {
  return getAccelHandle(accel.x);
}

globallycoherent RWByteAddressBuffer
getBufferHandle(Buffer buffer) {
  return buffers[buffer.y];
}

Texture1D
getTexture1DHandle(gprt::Texture texture) {
  return texture1Ds[texture];
}

Texture2D
getTexture2DHandle(gprt::Texture texture) {
  return texture2Ds[texture];
}

Texture3D
getTexture3DHandle(gprt::Texture texture) {
  return texture3Ds[texture];
}


SamplerState
getSamplerHandle(gprt::Sampler sampler) {
  return samplers[sampler];
}

SamplerState
getDefaultSampler() {
  // We assume that there is a default sampler at address 0 here
  return samplers[0];
}

};   // namespace gprt

/*
  The DXC compiler unfortunately doesn't handle multiple mixed entry points
  of different stage kinds in the same compilation step. So we instead
  use these macros to selectively filter all but a particular shader type,
  and compile repeatedly for all shader kinds (compute, raygen, anyhit, etc)

  Down the road, this could also give us an opportunity to shim in an Embree
  backend.
*/

/* GPRT_RAW here makes sure macro expansion doesn't get ignored when generated as
part of the parent macro expansion. */
#define GPRT_RAW(...) __VA_ARGS__

/* GPRT_CAT here is similar to GPRT_RAW, making sure concatenation is preserved as
part of the parent macro expansion */
#define GPRT_CAT_(A, B) A##B
#define GPRT_CAT(A, B)  GPRT_CAT_(A, B)

/* TYPE_NAME_EXPAND transforms "(A,B)"" into "A B". We usually include the
parenthesis as part of the argument in the macro, like "TYPE_NAME_EXPAND ARG,
where ARG is "(type_, name)". */
#define TYPE_NAME_EXPAND(type_, name_) type_ name_
#define TYPE_EXPAND(type_, name_)      type_
#define NAME_EXPAND(type_, name_)      name_

#ifndef GPRT_RAYGEN_PROGRAM
#ifdef RAYGEN
#define GPRT_RAYGEN_PROGRAM(progName, RecordDecl)                                                                      \
  /* fwd decl for the kernel func to call */                                                                           \
  void progName(in GPRT_RAW(TYPE_NAME_EXPAND) RecordDecl);                                                                  \
                                                                                                                       \
  [[vk::shader_record_ext]] ConstantBuffer<GPRT_RAW(TYPE_EXPAND RecordDecl)> GPRT_CAT(GPRT_RAW(progName),                             \
                                                                            GPRT_RAW(TYPE_EXPAND RecordDecl));              \
                                                                                                                       \
  [shader("raygeneration")] void __raygen__##progName() {                                                              \
    progName(GPRT_CAT(GPRT_RAW(progName), GPRT_RAW(TYPE_EXPAND RecordDecl)));                                                         \
  }                                                                                                                    \
                                                                                                                       \
  /* now the actual device code that the user is writing: */                                                           \
  void progName(in GPRT_RAW(TYPE_NAME_EXPAND) RecordDecl) /* program args and body supplied by user ... */
#else
#define GPRT_RAYGEN_PROGRAM(progName, RecordDecl)                                                                      \
  /* Dont add entry point decorators, instead treat as just a function. */                                             \
  void progName(in GPRT_RAW(TYPE_NAME_EXPAND) RecordDecl) /* program args and body supplied by user ... */
#endif
#endif

#ifndef GPRT_CLOSEST_HIT_PROGRAM
#ifdef CLOSESTHIT
#define GPRT_CLOSEST_HIT_PROGRAM(progName, RecordDecl, PayloadDecl, AttributeDecl)                                     \
  /* fwd decl for the kernel func to call */                                                                           \
  void progName(in GPRT_RAW(TYPE_NAME_EXPAND) RecordDecl, inout GPRT_RAW(TYPE_NAME_EXPAND) PayloadDecl,                          \
                in GPRT_RAW(TYPE_NAME_EXPAND) AttributeDecl);                                                               \
                                                                                                                       \
  [[vk::shader_record_ext]] ConstantBuffer<GPRT_RAW(TYPE_EXPAND RecordDecl)> GPRT_CAT(GPRT_RAW(progName),                             \
                                                                            GPRT_RAW(TYPE_EXPAND RecordDecl));              \
                                                                                                                       \
  [shader("closesthit")] void __closesthit__##progName(inout GPRT_RAW(TYPE_NAME_EXPAND) PayloadDecl,                        \
                                                       in GPRT_RAW(TYPE_NAME_EXPAND) AttributeDecl) {                       \
    progName(GPRT_CAT(GPRT_RAW(progName), GPRT_RAW(TYPE_EXPAND RecordDecl)), GPRT_RAW(NAME_EXPAND PayloadDecl),                            \
             GPRT_RAW(NAME_EXPAND AttributeDecl));                                                                          \
  }                                                                                                                    \
                                                                                                                       \
  /* now the actual device code that the user is writing: */                                                           \
  void progName(in GPRT_RAW(TYPE_NAME_EXPAND) RecordDecl, inout GPRT_RAW(TYPE_NAME_EXPAND) PayloadDecl,                          \
                in GPRT_RAW(TYPE_NAME_EXPAND) AttributeDecl) /* program args and body supplied by user ... */
#else
#define GPRT_CLOSEST_HIT_PROGRAM(progName, RecordDecl, PayloadDecl, AttributeDecl)                                     \
  /* Dont add entry point decorators, instead treat as just a function. */                                             \
  void progName(in GPRT_RAW(TYPE_NAME_EXPAND) RecordDecl, inout GPRT_RAW(TYPE_NAME_EXPAND) PayloadDecl,                          \
                in GPRT_RAW(TYPE_NAME_EXPAND) AttributeDecl) /* program args and body supplied by user ... */
#endif
#endif

static bool _ignoreHit = false;
static bool _acceptHitAndEndSearch = false;
namespace gprt {
// See DXC issue #5158
  void
  ignoreHit() {
    _ignoreHit = true;
  }

  void
  acceptHitAndEndSearch() {
    _acceptHitAndEndSearch = true;
  }
};   // namespace gprt

#ifndef GPRT_ANY_HIT_PROGRAM
#ifdef ANYHIT
#define GPRT_ANY_HIT_PROGRAM(progName, RecordDecl, PayloadDecl, AttributeDecl)                                         \
  /* fwd decl for the kernel func to call */                                                                           \
  inline void progName(in GPRT_RAW(TYPE_NAME_EXPAND) RecordDecl, inout GPRT_RAW(TYPE_NAME_EXPAND) PayloadDecl,                   \
                       in GPRT_RAW(TYPE_NAME_EXPAND) AttributeDecl);                                                        \
                                                                                                                       \
  [[vk::shader_record_ext]] ConstantBuffer<GPRT_RAW(TYPE_EXPAND RecordDecl)> GPRT_CAT(GPRT_RAW(progName),                             \
                                                                            GPRT_RAW(TYPE_EXPAND RecordDecl));              \
                                                                                                                       \
  [shader("anyhit")] void __anyhit__##progName(inout GPRT_RAW(TYPE_NAME_EXPAND) PayloadDecl,                                \
                                               in GPRT_RAW(TYPE_NAME_EXPAND) AttributeDecl) {                               \
    progName(GPRT_CAT(GPRT_RAW(progName), GPRT_RAW(TYPE_EXPAND RecordDecl)), GPRT_RAW(NAME_EXPAND PayloadDecl),                            \
             GPRT_RAW(NAME_EXPAND AttributeDecl));                                                                          \
    if (_ignoreHit)                                                                                                    \
      IgnoreHit();                                                                                                     \
    if (_acceptHitAndEndSearch)                                                                                        \
      AcceptHitAndEndSearch();                                                                                         \
  }                                                                                                                    \
                                                                                                                       \
  /* now the actual device code that the user is writing: */                                                           \
  inline void progName(in GPRT_RAW(TYPE_NAME_EXPAND) RecordDecl, inout GPRT_RAW(TYPE_NAME_EXPAND) PayloadDecl,                   \
                       in GPRT_RAW(TYPE_NAME_EXPAND) AttributeDecl) /* program args and body supplied by user ... */
#else
#define GPRT_ANY_HIT_PROGRAM(progName, RecordDecl, PayloadDecl, AttributeDecl)                                         \
  /* Dont add entry point decorators, instead treat as just a function. */                                             \
  inline void progName(in GPRT_RAW(TYPE_NAME_EXPAND) RecordDecl, inout GPRT_RAW(TYPE_NAME_EXPAND) PayloadDecl,                   \
                       in GPRT_RAW(TYPE_NAME_EXPAND) AttributeDecl) /* program args and body supplied by user ... */
#endif
#endif

#ifndef GPRT_INTERSECTION_PROGRAM
#ifdef INTERSECTION
#define GPRT_INTERSECTION_PROGRAM(progName, RecordDecl)                                                                \
  /* fwd decl for the kernel func to call */                                                                           \
  void progName(in GPRT_RAW(TYPE_NAME_EXPAND) RecordDecl);                                                                  \
                                                                                                                       \
  [[vk::shader_record_ext]] ConstantBuffer<GPRT_RAW(TYPE_EXPAND RecordDecl)> GPRT_CAT(GPRT_RAW(progName),                             \
                                                                            GPRT_RAW(TYPE_EXPAND RecordDecl));              \
                                                                                                                       \
  [shader("intersection")] void __intersection__##progName() {                                                         \
    progName(GPRT_CAT(GPRT_RAW(progName), GPRT_RAW(TYPE_EXPAND RecordDecl)));                                                         \
  }                                                                                                                    \
                                                                                                                       \
  /* now the actual device code that the user is writing: */                                                           \
  void progName(in GPRT_RAW(TYPE_NAME_EXPAND) RecordDecl) /* program args and body supplied by user ... */
#else
#define GPRT_INTERSECTION_PROGRAM(progName, RecordDecl)                                                                \
  /* Dont add entry point decorators, instead treat as just a function. */                                             \
  void progName(in GPRT_RAW(TYPE_NAME_EXPAND) RecordDecl) /* program args and body supplied by user ... */
#endif
#endif

#ifndef GPRT_MISS_PROGRAM
#ifdef MISS
#define GPRT_MISS_PROGRAM(progName, RecordDecl, PayloadDecl)                                                           \
  /* fwd decl for the kernel func to call */                                                                           \
  void progName(in GPRT_RAW(TYPE_NAME_EXPAND) RecordDecl, inout GPRT_RAW(TYPE_NAME_EXPAND) PayloadDecl);                         \
                                                                                                                       \
  [[vk::shader_record_ext]] ConstantBuffer<GPRT_RAW(TYPE_EXPAND RecordDecl)> GPRT_CAT(GPRT_RAW(progName),                             \
                                                                            GPRT_RAW(TYPE_EXPAND RecordDecl));              \
                                                                                                                       \
  [shader("miss")] void __miss__##progName(inout GPRT_RAW(TYPE_NAME_EXPAND) PayloadDecl) {                                  \
    progName(GPRT_CAT(GPRT_RAW(progName), GPRT_RAW(TYPE_EXPAND RecordDecl)), GPRT_RAW(NAME_EXPAND PayloadDecl));                           \
  }                                                                                                                    \
                                                                                                                       \
  /* now the actual device code that the user is writing: */                                                           \
  void progName(in GPRT_RAW(TYPE_NAME_EXPAND) RecordDecl,                                                                   \
                inout GPRT_RAW(TYPE_NAME_EXPAND) PayloadDecl) /* program args and body supplied by user ... */
#else
#define GPRT_MISS_PROGRAM(progName, RecordDecl, PayloadDecl)                                                           \
  /* Dont add entry point decorators, instead treat as just a function. */                                             \
  void progName(in GPRT_RAW(TYPE_NAME_EXPAND) RecordDecl,                                                                   \
                inout GPRT_RAW(TYPE_NAME_EXPAND) PayloadDecl) /* program args and body supplied by user ... */
#endif
#endif

#ifndef GPRT_COMPUTE_PROGRAM
#ifdef COMPUTE
#define GPRT_COMPUTE_PROGRAM(progName, RecordDecl, NumThreads)                                                         \
  [[vk::binding(0, 5)]] ConstantBuffer<GPRT_RAW(TYPE_EXPAND RecordDecl)> GPRT_CAT(GPRT_RAW(progName), GPRT_RAW(TYPE_EXPAND RecordDecl));   \
                                                                                                                       \
  /* fwd decl for the kernel func to call */                                                                           \
  void progName(in GPRT_RAW(TYPE_NAME_EXPAND) RecordDecl, uint3 GroupThreadID, uint3 GroupID, uint3 DispatchThreadID,       \
                uint GroupIndex);                                                                                      \
                                                                                                                       \
  [numthreads NumThreads][shader("compute")] void __compute__##progName(uint3 groupThreadID                            \
                                                                        : SV_GroupThreadID, uint3 groupID              \
                                                                        : SV_GroupID, uint3 dispatchThreadID           \
                                                                        : SV_DispatchThreadID, uint groupIndex         \
                                                                        : SV_GroupIndex) {                             \
    progName(GPRT_CAT(GPRT_RAW(progName), GPRT_RAW(TYPE_EXPAND RecordDecl)), groupThreadID, groupID, dispatchThreadID, groupIndex);   \
  }                                                                                                                    \
                                                                                                                       \
  /* now the actual device code that the user is writing: */                                                           \
  void progName(in GPRT_RAW(TYPE_NAME_EXPAND) RecordDecl, uint3 GroupThreadID, uint3 GroupID, uint3 DispatchThreadID,       \
                uint GroupIndex) /* program args and body supplied by user ... */
#else
#define GPRT_COMPUTE_PROGRAM(progName, RecordDecl, NumThreads)                                                         \
  /* Dont add entry point decorators, instead treat as just a function. */                                             \
  void progName(in GPRT_RAW(TYPE_NAME_EXPAND) RecordDecl, uint3 GroupThreadID, uint3 GroupID, uint3 DispatchThreadID,       \
                uint GroupIndex) /* program args and body supplied by user ... */
#endif
#endif

static uint32_t _VertexIndex = -1;
static uint32_t _PrimitiveIndex = -1;
static float2 _Barycentrics = float2(0.f, 0.f);
static float4 _Position = float4(0.f, 0.f, 0.f, 0.f);

uint32_t
VertexIndex() {
  return _VertexIndex;
}

uint32_t
TriangleIndex() {
  return _PrimitiveIndex;
}

float2
Barycentrics() {
  return _Barycentrics;
}

float4
Position() {
  return _Position;
}

#ifndef GPRT_VERTEX_PROGRAM
#ifdef VERTEX
#define GPRT_VERTEX_PROGRAM(progName, RecordDecl)                                                                      \
  struct progName##VSOutput {                                                                                          \
    float4 position : SV_POSITION;                                                                                     \
    float2 barycentrics : TEXCOORD0;                                                                                   \
  };                                                                                                                   \
                                                                                                                       \
  [[vk::binding(0, 5)]] ConstantBuffer<GPRT_RAW(TYPE_EXPAND RecordDecl)> GPRT_CAT(GPRT_RAW(progName), GPRT_RAW(TYPE_EXPAND RecordDecl));   \
                                                                                                                       \
  /* fwd decl for the kernel func to call */                                                                           \
  float4 progName(in GPRT_RAW(TYPE_NAME_EXPAND) RecordDecl);                                                                \
                                                                                                                       \
  [shader("vertex")] progName##VSOutput __vertex__##progName(uint SVVID : SV_VertexID) {                               \
    _VertexIndex = SVVID;                                                                                              \
    _PrimitiveIndex = _VertexIndex % 3;                                                                                \
    progName##VSOutput output;                                                                                         \
    output.position = progName(GPRT_CAT(GPRT_RAW(progName), GPRT_RAW(TYPE_EXPAND RecordDecl)));                                       \
    output.barycentrics = (((SVVID % 3) == 0)   ? float2(0.f, 0.f)                                                     \
                           : ((SVVID % 3) == 1) ? float2(1.f, 0.f)                                                     \
                                                : float2(0.f, 1.f));                                                   \
    return output;                                                                                                     \
  }                                                                                                                    \
                                                                                                                       \
  /* now the actual device code that the user is writing: */                                                           \
  float4 progName(in GPRT_RAW(TYPE_NAME_EXPAND) RecordDecl) /* program args and body supplied by user ... */
#else
#define GPRT_VERTEX_PROGRAM(progName, RecordDecl)                                                                      \
  /* Dont add entry point decorators, instead treat as just a function. */                                             \
  float4 progName(in GPRT_RAW(TYPE_NAME_EXPAND) RecordDecl) /* program args and body supplied by user ... */
#endif
#endif

#ifndef GPRT_PIXEL_PROGRAM
#ifdef PIXEL
#define GPRT_PIXEL_PROGRAM(progName, RecordDecl)                                                                       \
  [[vk::binding(0, 5)]] ConstantBuffer<GPRT_RAW(TYPE_EXPAND RecordDecl)> GPRT_CAT(GPRT_RAW(progName), GPRT_RAW(TYPE_EXPAND RecordDecl));   \
                                                                                                                       \
  /* fwd decl for the kernel func to call */                                                                           \
  float4 progName(in GPRT_RAW(TYPE_NAME_EXPAND) RecordDecl);                                                                \
                                                                                                                       \
  [shader("pixel")] float4 __pixel__##progName(float2 baryWeights                                                      \
                                               : TEXCOORD0, float4 position                                            \
                                               : SV_Position, uint32_t PrimitiveID                                     \
                                               : SV_PrimitiveID)                                                       \
      : SV_TARGET {                                                                                                    \
    _Position = position;                                                                                              \
    _Barycentrics = baryWeights;                                                                                       \
    _PrimitiveIndex = PrimitiveID;                                                                                     \
    return progName(GPRT_CAT(GPRT_RAW(progName), GPRT_RAW(TYPE_EXPAND RecordDecl)));                                                  \
  }                                                                                                                    \
                                                                                                                       \
  /* now the actual device code that the user is writing: */                                                           \
  float4 progName(in GPRT_RAW(TYPE_NAME_EXPAND) RecordDecl) /* program args and body supplied by user ... */
#else
#define GPRT_PIXEL_PROGRAM(progName, RecordDecl)                                                                       \
  /* Dont add entry point decorators, instead treat as just a function. */                                             \
  float4 progName(in GPRT_RAW(TYPE_NAME_EXPAND) RecordDecl) /* program args and body supplied by user ... */
#endif
#endif