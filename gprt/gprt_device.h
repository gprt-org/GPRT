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

// struct PushConstants {
//   uint64_t r[16];
// };
// [[vk::push_constant]] PushConstants pc;

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
[[vk::binding(0, 4)]] RWByteAddressBuffer buffers[];

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

// struct Buffer {
//   uint64_t x;
//   uint64_t y;
// };

// ideally this buffer type would be a struct...
// but I'm running into a compiler bug reading one struct inside another one.

// x stores pointer, y stores size.
typedef uint64_t2 Buffer;
typedef uint64_t2 Accel;
typedef uint32_t Texture;
typedef uint32_t Sampler;

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
void
store(in Buffer buffer, uint64_t index, in T value) {
  vk::RawBufferStore<T>(buffer.x + index * sizeof(T), value);
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


[[vk::ext_instruction(4447)]] RaytracingAccelerationStructure getAccelHandle(uint64_t ptr);

RaytracingAccelerationStructure
getAccelHandle(Accel accel) {
  return getAccelHandle(accel.x);
}

RWByteAddressBuffer
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

/* RAW here makes sure macro expansion doesn't get ignored when generated as
part of the parent macro expansion. */
#define RAW(...) __VA_ARGS__

/* CAT here is similar to RAW, making sure concatenation is preserved as
part of the parent macro expansion */
#define CAT_(A, B) A##B
#define CAT(A, B)  CAT_(A, B)

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
  void progName(in RAW(TYPE_NAME_EXPAND) RecordDecl);                                                                  \
                                                                                                                       \
  [[vk::shader_record_ext]] ConstantBuffer<RAW(TYPE_EXPAND RecordDecl)> CAT(RAW(progName),                             \
                                                                            RAW(TYPE_EXPAND RecordDecl));              \
                                                                                                                       \
  [shader("raygeneration")] void __raygen__##progName() {                                                              \
    progName(CAT(RAW(progName), RAW(TYPE_EXPAND RecordDecl)));                                                         \
  }                                                                                                                    \
                                                                                                                       \
  /* now the actual device code that the user is writing: */                                                           \
  void progName(in RAW(TYPE_NAME_EXPAND) RecordDecl) /* program args and body supplied by user ... */
#else
#define GPRT_RAYGEN_PROGRAM(progName, RecordDecl)                                                                      \
  /* Dont add entry point decorators, instead treat as just a function. */                                             \
  void progName(in RAW(TYPE_NAME_EXPAND) RecordDecl) /* program args and body supplied by user ... */
#endif
#endif

#ifndef GPRT_CLOSEST_HIT_PROGRAM
#ifdef CLOSESTHIT
#define GPRT_CLOSEST_HIT_PROGRAM(progName, RecordDecl, PayloadDecl, AttributeDecl)                                     \
  /* fwd decl for the kernel func to call */                                                                           \
  void progName(in RAW(TYPE_NAME_EXPAND) RecordDecl, inout RAW(TYPE_NAME_EXPAND) PayloadDecl,                          \
                in RAW(TYPE_NAME_EXPAND) AttributeDecl);                                                               \
                                                                                                                       \
  [[vk::shader_record_ext]] ConstantBuffer<RAW(TYPE_EXPAND RecordDecl)> CAT(RAW(progName),                             \
                                                                            RAW(TYPE_EXPAND RecordDecl));              \
                                                                                                                       \
  [shader("closesthit")] void __closesthit__##progName(inout RAW(TYPE_NAME_EXPAND) PayloadDecl,                        \
                                                       in RAW(TYPE_NAME_EXPAND) AttributeDecl) {                       \
    progName(CAT(RAW(progName), RAW(TYPE_EXPAND RecordDecl)), RAW(NAME_EXPAND PayloadDecl),                            \
             RAW(NAME_EXPAND AttributeDecl));                                                                          \
  }                                                                                                                    \
                                                                                                                       \
  /* now the actual device code that the user is writing: */                                                           \
  void progName(in RAW(TYPE_NAME_EXPAND) RecordDecl, inout RAW(TYPE_NAME_EXPAND) PayloadDecl,                          \
                in RAW(TYPE_NAME_EXPAND) AttributeDecl) /* program args and body supplied by user ... */
#else
#define GPRT_CLOSEST_HIT_PROGRAM(progName, RecordDecl, PayloadDecl, AttributeDecl)                                     \
  /* Dont add entry point decorators, instead treat as just a function. */                                             \
  void progName(in RAW(TYPE_NAME_EXPAND) RecordDecl, inout RAW(TYPE_NAME_EXPAND) PayloadDecl,                          \
                in RAW(TYPE_NAME_EXPAND) AttributeDecl) /* program args and body supplied by user ... */
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
  inline void progName(in RAW(TYPE_NAME_EXPAND) RecordDecl, inout RAW(TYPE_NAME_EXPAND) PayloadDecl,                   \
                       in RAW(TYPE_NAME_EXPAND) AttributeDecl);                                                        \
                                                                                                                       \
  [[vk::shader_record_ext]] ConstantBuffer<RAW(TYPE_EXPAND RecordDecl)> CAT(RAW(progName),                             \
                                                                            RAW(TYPE_EXPAND RecordDecl));              \
                                                                                                                       \
  [shader("anyhit")] void __anyhit__##progName(inout RAW(TYPE_NAME_EXPAND) PayloadDecl,                                \
                                               in RAW(TYPE_NAME_EXPAND) AttributeDecl) {                               \
    progName(CAT(RAW(progName), RAW(TYPE_EXPAND RecordDecl)), RAW(NAME_EXPAND PayloadDecl),                            \
             RAW(NAME_EXPAND AttributeDecl));                                                                          \
    if (_ignoreHit)                                                                                                    \
      IgnoreHit();                                                                                                     \
    if (_acceptHitAndEndSearch)                                                                                        \
      AcceptHitAndEndSearch();                                                                                         \
  }                                                                                                                    \
                                                                                                                       \
  /* now the actual device code that the user is writing: */                                                           \
  inline void progName(in RAW(TYPE_NAME_EXPAND) RecordDecl, inout RAW(TYPE_NAME_EXPAND) PayloadDecl,                   \
                       in RAW(TYPE_NAME_EXPAND) AttributeDecl) /* program args and body supplied by user ... */
#else
#define GPRT_ANY_HIT_PROGRAM(progName, RecordDecl, PayloadDecl, AttributeDecl)                                         \
  /* Dont add entry point decorators, instead treat as just a function. */                                             \
  inline void progName(in RAW(TYPE_NAME_EXPAND) RecordDecl, inout RAW(TYPE_NAME_EXPAND) PayloadDecl,                   \
                       in RAW(TYPE_NAME_EXPAND) AttributeDecl) /* program args and body supplied by user ... */
#endif
#endif

#ifndef GPRT_INTERSECTION_PROGRAM
#ifdef INTERSECTION
#define GPRT_INTERSECTION_PROGRAM(progName, RecordDecl)                                                                \
  /* fwd decl for the kernel func to call */                                                                           \
  void progName(in RAW(TYPE_NAME_EXPAND) RecordDecl);                                                                  \
                                                                                                                       \
  [[vk::shader_record_ext]] ConstantBuffer<RAW(TYPE_EXPAND RecordDecl)> CAT(RAW(progName),                             \
                                                                            RAW(TYPE_EXPAND RecordDecl));              \
                                                                                                                       \
  [shader("intersection")] void __intersection__##progName() {                                                         \
    progName(CAT(RAW(progName), RAW(TYPE_EXPAND RecordDecl)));                                                         \
  }                                                                                                                    \
                                                                                                                       \
  /* now the actual device code that the user is writing: */                                                           \
  void progName(in RAW(TYPE_NAME_EXPAND) RecordDecl) /* program args and body supplied by user ... */
#else
#define GPRT_INTERSECTION_PROGRAM(progName, RecordDecl)                                                                \
  /* Dont add entry point decorators, instead treat as just a function. */                                             \
  void progName(in RAW(TYPE_NAME_EXPAND) RecordDecl) /* program args and body supplied by user ... */
#endif
#endif

#ifndef GPRT_MISS_PROGRAM
#ifdef MISS
#define GPRT_MISS_PROGRAM(progName, RecordDecl, PayloadDecl)                                                           \
  /* fwd decl for the kernel func to call */                                                                           \
  void progName(in RAW(TYPE_NAME_EXPAND) RecordDecl, inout RAW(TYPE_NAME_EXPAND) PayloadDecl);                         \
                                                                                                                       \
  [[vk::shader_record_ext]] ConstantBuffer<RAW(TYPE_EXPAND RecordDecl)> CAT(RAW(progName),                             \
                                                                            RAW(TYPE_EXPAND RecordDecl));              \
                                                                                                                       \
  [shader("miss")] void __miss__##progName(inout RAW(TYPE_NAME_EXPAND) PayloadDecl) {                                  \
    progName(CAT(RAW(progName), RAW(TYPE_EXPAND RecordDecl)), RAW(NAME_EXPAND PayloadDecl));                           \
  }                                                                                                                    \
                                                                                                                       \
  /* now the actual device code that the user is writing: */                                                           \
  void progName(in RAW(TYPE_NAME_EXPAND) RecordDecl,                                                                   \
                inout RAW(TYPE_NAME_EXPAND) PayloadDecl) /* program args and body supplied by user ... */
#else
#define GPRT_MISS_PROGRAM(progName, RecordDecl, PayloadDecl)                                                           \
  /* Dont add entry point decorators, instead treat as just a function. */                                             \
  void progName(in RAW(TYPE_NAME_EXPAND) RecordDecl,                                                                   \
                inout RAW(TYPE_NAME_EXPAND) PayloadDecl) /* program args and body supplied by user ... */
#endif
#endif

#ifndef GPRT_COMPUTE_PROGRAM
#ifdef COMPUTE
#define GPRT_COMPUTE_PROGRAM(progName, RecordDecl, NumThreads)                                                         \
  [[vk::binding(0, 5)]] ConstantBuffer<RAW(TYPE_EXPAND RecordDecl)> CAT(RAW(progName), RAW(TYPE_EXPAND RecordDecl));   \
                                                                                                                       \
  /* fwd decl for the kernel func to call */                                                                           \
  void progName(in RAW(TYPE_NAME_EXPAND) RecordDecl, uint3 GroupThreadID, uint3 GroupID, uint3 DispatchThreadID,       \
                uint GroupIndex);                                                                                      \
                                                                                                                       \
  [numthreads NumThreads][shader("compute")] void __compute__##progName(uint3 groupThreadID                            \
                                                                        : SV_GroupThreadID, uint3 groupID              \
                                                                        : SV_GroupID, uint3 dispatchThreadID           \
                                                                        : SV_DispatchThreadID, uint groupIndex         \
                                                                        : SV_GroupIndex) {                             \
    progName(CAT(RAW(progName), RAW(TYPE_EXPAND RecordDecl)), groupThreadID, groupID, dispatchThreadID, groupIndex);   \
  }                                                                                                                    \
                                                                                                                       \
  /* now the actual device code that the user is writing: */                                                           \
  void progName(in RAW(TYPE_NAME_EXPAND) RecordDecl, uint3 GroupThreadID, uint3 GroupID, uint3 DispatchThreadID,       \
                uint GroupIndex) /* program args and body supplied by user ... */
#else
#define GPRT_COMPUTE_PROGRAM(progName, RecordDecl, NumThreads)                                                         \
  /* Dont add entry point decorators, instead treat as just a function. */                                             \
  void progName(in RAW(TYPE_NAME_EXPAND) RecordDecl, uint3 GroupThreadID, uint3 GroupID, uint3 DispatchThreadID,       \
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
  [[vk::binding(0, 5)]] ConstantBuffer<RAW(TYPE_EXPAND RecordDecl)> CAT(RAW(progName), RAW(TYPE_EXPAND RecordDecl));   \
                                                                                                                       \
  /* fwd decl for the kernel func to call */                                                                           \
  float4 progName(in RAW(TYPE_NAME_EXPAND) RecordDecl);                                                                \
                                                                                                                       \
  [shader("vertex")] progName##VSOutput __vertex__##progName(uint SVVID : SV_VertexID) {                               \
    _VertexIndex = SVVID;                                                                                              \
    _PrimitiveIndex = _VertexIndex % 3;                                                                                \
    progName##VSOutput output;                                                                                         \
    output.position = progName(CAT(RAW(progName), RAW(TYPE_EXPAND RecordDecl)));                                       \
    output.barycentrics = (((SVVID % 3) == 0)   ? float2(0.f, 0.f)                                                     \
                           : ((SVVID % 3) == 1) ? float2(1.f, 0.f)                                                     \
                                                : float2(0.f, 1.f));                                                   \
    return output;                                                                                                     \
  }                                                                                                                    \
                                                                                                                       \
  /* now the actual device code that the user is writing: */                                                           \
  float4 progName(in RAW(TYPE_NAME_EXPAND) RecordDecl) /* program args and body supplied by user ... */
#else
#define GPRT_VERTEX_PROGRAM(progName, RecordDecl)                                                                      \
  /* Dont add entry point decorators, instead treat as just a function. */                                             \
  float4 progName(in RAW(TYPE_NAME_EXPAND) RecordDecl) /* program args and body supplied by user ... */
#endif
#endif

#ifndef GPRT_PIXEL_PROGRAM
#ifdef PIXEL
#define GPRT_PIXEL_PROGRAM(progName, RecordDecl)                                                                       \
  [[vk::binding(0, 5)]] ConstantBuffer<RAW(TYPE_EXPAND RecordDecl)> CAT(RAW(progName), RAW(TYPE_EXPAND RecordDecl));   \
                                                                                                                       \
  /* fwd decl for the kernel func to call */                                                                           \
  float4 progName(in RAW(TYPE_NAME_EXPAND) RecordDecl);                                                                \
                                                                                                                       \
  [shader("pixel")] float4 __pixel__##progName(float2 baryWeights                                                      \
                                               : TEXCOORD0, float4 position                                            \
                                               : SV_Position, uint32_t PrimitiveID                                     \
                                               : SV_PrimitiveID)                                                       \
      : SV_TARGET {                                                                                                    \
    _Position = position;                                                                                              \
    _Barycentrics = baryWeights;                                                                                       \
    _PrimitiveIndex = PrimitiveID;                                                                                     \
    return progName(CAT(RAW(progName), RAW(TYPE_EXPAND RecordDecl)));                                                  \
  }                                                                                                                    \
                                                                                                                       \
  /* now the actual device code that the user is writing: */                                                           \
  float4 progName(in RAW(TYPE_NAME_EXPAND) RecordDecl) /* program args and body supplied by user ... */
#else
#define GPRT_PIXEL_PROGRAM(progName, RecordDecl)                                                                       \
  /* Dont add entry point decorators, instead treat as just a function. */                                             \
  float4 progName(in RAW(TYPE_NAME_EXPAND) RecordDecl) /* program args and body supplied by user ... */
#endif
#endif