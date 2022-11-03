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
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#define alignas(alignment)

struct PushConstants {
  uint64_t r[16];
};
[[vk::push_constant]] PushConstants pc;

namespace gprt {
  inline uint32_t make_8bit(const float f)
  {
    return min(255,max(0,int(f*256.f)));
  }

  inline uint32_t make_rgba(const float3 color)
  {
    return
      (make_8bit(color.x) << 0) +
      (make_8bit(color.y) << 8) +
      (make_8bit(color.z) << 16) +
      (0xffU << 24);
  }

    // struct Buffer {
  //   uint64_t x;
  //   uint64_t y;
  // };
  
  // ideally this buffer type would be a struct...
  // but I'm running into a compiler bug reading one struct inside another one.
  // x stores pointer, y stores size.
  typedef uint64_t2 Buffer;

  template<typename T> 
  T load(in Buffer buffer, uint64_t index) {
    return vk::RawBufferLoad<T>(buffer.x + index * sizeof(T));
  }
  template<typename T> 
  void store(in Buffer buffer, uint64_t index, in T value) {
    vk::RawBufferStore<T>(buffer.x + index * sizeof(T), value);
  }

  // x stores pointer, y stores type
  typedef uint64_t2 Accel;

  [[vk::ext_instruction(4447)]]
  RaytracingAccelerationStructure getAccelHandle(uint64_t ptr);

  RaytracingAccelerationStructure getAccelHandle(Accel accel) {
    return getAccelHandle(accel.x);
  }

  void amdkludge() {
    if (pc.r[15]) {
      struct Stubstruct {
        int tmp;
      } stubstruct;
      RaytracingAccelerationStructure stubtree = getAccelHandle(0); 
      RayDesc rayDesc;
      TraceRay(
        stubtree, // the tree
        RAY_FLAG_FORCE_OPAQUE, // ray flags
        0xff, // instance inclusion mask
        0, // ray type
        0, // number of ray types
        0, // miss type
        rayDesc, // the ray to trace
        stubstruct // the payload IO
      );
      vk::RawBufferStore<int>(0, stubstruct.tmp);
    }
  }
};



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
#define CAT(A, B) CAT_(A, B)

/* TYPE_NAME_EXPAND transforms "(A,B)"" into "A B". We usually include the 
parenthesis as part of the argument in the macro, like "TYPE_NAME_EXPAND ARG, 
where ARG is "(type_, name)". */
#define TYPE_NAME_EXPAND(type_, name_) type_ name_
#define TYPE_EXPAND(type_, name_) type_
#define NAME_EXPAND(type_, name_) name_


#ifndef GPRT_RAYGEN_PROGRAM
#ifdef RAYGEN
#define GPRT_RAYGEN_PROGRAM(progName, RecordDecl)                               \
  /* fwd decl for the kernel func to call */                                    \
  void progName(in RAW(TYPE_NAME_EXPAND)RecordDecl);                            \
                                                                                \
  [[vk::shader_record_ext]]                                                     \
  ConstantBuffer<RAW(TYPE_EXPAND RecordDecl)>                                   \
    CAT(RAW(progName),RAW(TYPE_EXPAND RecordDecl));                             \
                                                                                \
  [shader("raygeneration")]                                                     \
  void __raygen__##progName()                                                   \
  {                                                                             \
    progName(CAT(RAW(progName),RAW(TYPE_EXPAND RecordDecl)));                   \
  }                                                                             \
                                                                                \
  /* now the actual device code that the user is writing: */                    \
  void progName(in RAW(TYPE_NAME_EXPAND)RecordDecl)                             \
/* program args and body supplied by user ... */
#else
#define GPRT_RAYGEN_PROGRAM(progName, RecordDecl)                               \
  /* Dont add entry point decorators, instead treat as just a function. */        \
  void progName(in RAW(TYPE_NAME_EXPAND)RecordDecl)                               \
  /* program args and body supplied by user ... */
#endif
#endif

#ifndef GPRT_CLOSEST_HIT_PROGRAM
#ifdef CLOSESTHIT
#define GPRT_CLOSEST_HIT_PROGRAM(progName,                                      \
                                 RecordDecl, PayloadDecl, AttributeDecl)        \
  /* fwd decl for the kernel func to call */                                    \
  void progName(in RAW(TYPE_NAME_EXPAND)RecordDecl,                             \
                inout RAW(TYPE_NAME_EXPAND)PayloadDecl,                         \
                in RAW(TYPE_NAME_EXPAND)AttributeDecl);                         \
                                                                                \
  [[vk::shader_record_ext]]                                                     \
  ConstantBuffer<RAW(TYPE_EXPAND RecordDecl)>                                   \
    CAT(RAW(progName),RAW(TYPE_EXPAND RecordDecl));                             \
                                                                                \
  [shader("closesthit")]                                                        \
  void __closesthit__##progName(inout RAW(TYPE_NAME_EXPAND)PayloadDecl,         \
                                in RAW(TYPE_NAME_EXPAND)AttributeDecl)          \
  {                                                                             \
    gprt::amdkludge();                                                                \
    progName(CAT(RAW(progName),RAW(TYPE_EXPAND RecordDecl)),                    \
             RAW(NAME_EXPAND PayloadDecl),                                      \
             RAW(NAME_EXPAND AttributeDecl));                                   \
  }                                                                             \
                                                                                \
  /* now the actual device code that the user is writing: */                    \
    void progName(in RAW(TYPE_NAME_EXPAND)RecordDecl,                           \
                inout RAW(TYPE_NAME_EXPAND)PayloadDecl,                         \
                in RAW(TYPE_NAME_EXPAND)AttributeDecl)                          \
/* program args and body supplied by user ... */
#else
#define GPRT_CLOSEST_HIT_PROGRAM(progName,                                      \
                                 RecordDecl, PayloadDecl, AttributeDecl)        \
  /* Dont add entry point decorators, instead treat as just a function. */      \
    void progName(in RAW(TYPE_NAME_EXPAND)RecordDecl,                           \
                inout RAW(TYPE_NAME_EXPAND)PayloadDecl,                         \
                in RAW(TYPE_NAME_EXPAND)AttributeDecl)                          \
  /* program args and body supplied by user ... */
#endif
#endif

#ifndef GPRT_ANY_HIT_PROGRAM
#ifdef ANYHIT
#define GPRT_ANY_HIT_PROGRAM(progName,                                          \
                                 RecordDecl, PayloadDecl, AttributeDecl)        \
  /* fwd decl for the kernel func to call */                                    \
  void progName(in RAW(TYPE_NAME_EXPAND)RecordDecl,                             \
                inout RAW(TYPE_NAME_EXPAND)PayloadDecl,                         \
                in RAW(TYPE_NAME_EXPAND)AttributeDecl);                         \
                                                                                \
  [[vk::shader_record_ext]]                                                     \
  ConstantBuffer<RAW(TYPE_EXPAND RecordDecl)>                                   \
    CAT(RAW(progName),RAW(TYPE_EXPAND RecordDecl));                             \
                                                                                \
  [shader("anyhit")]                                                            \
  void __anyhit__##progName(inout RAW(TYPE_NAME_EXPAND)PayloadDecl,             \
                                in RAW(TYPE_NAME_EXPAND)AttributeDecl)          \
  {                                                                             \
    progName(CAT(RAW(progName),RAW(TYPE_EXPAND RecordDecl)),                    \
             RAW(NAME_EXPAND PayloadDecl),                                      \
             RAW(NAME_EXPAND AttributeDecl));                                   \
  }                                                                             \
                                                                                \
  /* now the actual device code that the user is writing: */                    \
    void progName(in RAW(TYPE_NAME_EXPAND)RecordDecl,                           \
                inout RAW(TYPE_NAME_EXPAND)PayloadDecl,                         \
                in RAW(TYPE_NAME_EXPAND)AttributeDecl)                          \
/* program args and body supplied by user ... */
#else
#define GPRT_ANY_HIT_PROGRAM(progName,                                          \
                                 RecordDecl, PayloadDecl, AttributeDecl)        \
  /* Dont add entry point decorators, instead treat as just a function. */      \
    void progName(in RAW(TYPE_NAME_EXPAND)RecordDecl,                           \
                inout RAW(TYPE_NAME_EXPAND)PayloadDecl,                         \
                in RAW(TYPE_NAME_EXPAND)AttributeDecl)                          \
  /* program args and body supplied by user ... */
#endif
#endif

#ifndef GPRT_INTERSECTION_PROGRAM
#ifdef INTERSECTION
#define GPRT_INTERSECTION_PROGRAM(progName, RecordDecl)                         \
  /* fwd decl for the kernel func to call */                                    \
  void progName(in RAW(TYPE_NAME_EXPAND)RecordDecl);                            \
                                                                                \
  [[vk::shader_record_ext]]                                                     \
  ConstantBuffer<RAW(TYPE_EXPAND RecordDecl)>                                   \
    CAT(RAW(progName),RAW(TYPE_EXPAND RecordDecl));                             \
                                                                                \
  [shader("intersection")]                                                      \
  void __intersection__##progName()                                             \
  {                                                                             \
    progName(CAT(RAW(progName),RAW(TYPE_EXPAND RecordDecl)));                   \
  }                                                                             \
                                                                                \
  /* now the actual device code that the user is writing: */                    \
  void progName(in RAW(TYPE_NAME_EXPAND)RecordDecl)                             \
/* program args and body supplied by user ... */
#else
#define GPRT_INTERSECTION_PROGRAM(progName, RecordDecl)                         \
  /* Dont add entry point decorators, instead treat as just a function. */      \
  void progName(in RAW(TYPE_NAME_EXPAND)RecordDecl)                             \
  /* program args and body supplied by user ... */
#endif
#endif


#ifndef GPRT_MISS_PROGRAM
#ifdef MISS
#define GPRT_MISS_PROGRAM(progName, RecordDecl, PayloadDecl)                    \
  /* fwd decl for the kernel func to call */                                    \
  void progName(in RAW(TYPE_NAME_EXPAND)RecordDecl,                             \
                inout RAW(TYPE_NAME_EXPAND)PayloadDecl);                        \
                                                                                \
  [[vk::shader_record_ext]]                                                     \
  ConstantBuffer<RAW(TYPE_EXPAND RecordDecl)>                                   \
    CAT(RAW(progName),RAW(TYPE_EXPAND RecordDecl));                             \
                                                                                \
  [shader("miss")]                                                              \
  void __miss__##progName(inout RAW(TYPE_NAME_EXPAND)PayloadDecl)               \
  {                                                                             \
    progName(CAT(RAW(progName),RAW(TYPE_EXPAND RecordDecl)),                     \
             RAW(NAME_EXPAND PayloadDecl));                                     \
  }                                                                             \
                                                                                \
  /* now the actual device code that the user is writing: */                    \
  void progName(in RAW(TYPE_NAME_EXPAND)RecordDecl,                             \
              inout RAW(TYPE_NAME_EXPAND)PayloadDecl)                           \
/* program args and body supplied by user ... */
#else
#define GPRT_MISS_PROGRAM(progName, RecordDecl, PayloadDecl)                    \
  /* Dont add entry point decorators, instead treat as just a function. */      \
  void progName(in RAW(TYPE_NAME_EXPAND)RecordDecl,                             \
                inout RAW(TYPE_NAME_EXPAND)PayloadDecl)                         \
  /* program args and body supplied by user ... */
#endif
#endif

// We currently recycle ray generation programs to implement a user-side
// compute program. This allows us to recycle existing SBT record API
// for compute shader IO
#ifndef GPRT_COMPUTE_PROGRAM
#ifdef COMPUTE
#define GPRT_COMPUTE_PROGRAM(progName, RecordDecl)                              \
  /* fwd decl for the kernel func to call */                                    \
  void progName(in RAW(TYPE_NAME_EXPAND)RecordDecl,                             \
   uint GroupIndex, uint3 DispatchThreadID, uint3 GroupThreadID, uint3 GroupID);\
                                                                                \
  [[vk::shader_record_ext]]                                                     \
  ConstantBuffer<RAW(TYPE_EXPAND RecordDecl)>                                   \
    CAT(RAW(progName),RAW(TYPE_EXPAND RecordDecl));                             \
                                                                                \
  [shader("raygeneration")]                                                     \
  void __compute__##progName()                                                  \
  {                                                                             \
   progName(CAT(RAW(progName),RAW(TYPE_EXPAND RecordDecl)), 0,                  \
     DispatchRaysIndex(), uint3(0,0,0), uint3(0,0,0));                          \
  }                                                                             \
                                                                                \
  /* now the actual device code that the user is writing: */                    \
  void progName(in RAW(TYPE_NAME_EXPAND)RecordDecl,                             \
  uint GroupIndex, uint3 DispatchThreadID, uint3 GroupThreadID, uint3 GroupID)  \
/* program args and body supplied by user ... */
#else
#define GPRT_COMPUTE_PROGRAM(progName, RecordDecl)                              \
  /* Dont add entry point decorators, instead treat as just a function. */      \
  void progName(in RAW(TYPE_NAME_EXPAND)RecordDecl,                             \
    uint GroupIndex, uint3 DispatchThreadID, uint3 GroupThreadID, uint3 GroupID)\
  /* program args and body supplied by user ... */
#endif
#endif
