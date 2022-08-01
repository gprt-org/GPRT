// MIT License

// Copyright (c) 2022 Nathan V. Morrical

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <vulkan/vulkan.h>

#include "slang.h"
#include "slang-cpp-types.h"

#include <sys/types.h>
#include <stdint.h>

#include <assert.h>

#ifdef __cplusplus
# include <cstddef> 
#endif

#if defined(_MSC_VER)
#  define VKRT_DLL_EXPORT __declspec(dllexport)
#  define VKRT_DLL_IMPORT __declspec(dllimport)
#elif defined(__clang__) || defined(__GNUC__)
#  define VKRT_DLL_EXPORT __attribute__((visibility("default")))
#  define VKRT_DLL_IMPORT __attribute__((visibility("default")))
#else
#  define VKRT_DLL_EXPORT
#  define VKRT_DLL_IMPORT
#endif

#ifdef __cplusplus
# define VKRT_IF_CPP(a) a
#else
# define VKRT_IF_CPP(a) /* drop it */
#endif

#  ifdef __cplusplus
#    define VKRT_API extern "C" VKRT_DLL_EXPORT
#  else
#    define VKRT_API /* bla */
#  endif

#define VKRT_OFFSETOF(type,member)                       \
   (uint32_t)((char *)(&((struct type *)0)-> member )   \
   -                                                    \
   (char *)(((struct type *)0)))

// Terminal colors
#define VKRT_TERMINAL_RED "\033[0;31m"
#define VKRT_TERMINAL_GREEN "\033[0;32m"
#define VKRT_TERMINAL_LIGHT_GREEN "\033[1;32m"
#define VKRT_TERMINAL_YELLOW "\033[1;33m"
#define VKRT_TERMINAL_BLUE "\033[0;34m"
#define VKRT_TERMINAL_LIGHT_BLUE "\033[1;34m"
#define VKRT_TERMINAL_RESET "\033[0m"
#define VKRT_TERMINAL_DEFAULT VKRT_TERMINAL_RESET
#define VKRT_TERMINAL_BOLD "\033[1;1m"
#define VKRT_TERMINAL_MAGENTA "\e[35m"
#define VKRT_TERMINAL_LIGHT_MAGENTA "\e[95m"
#define VKRT_TERMINAL_CYAN "\e[36m"
#define VKRT_TERMINAL_LIGHT_RED "\033[1;31m"

typedef struct _VKRTContext       *VKRTContext;
typedef struct _VKRTBuffer        *VKRTBuffer;
typedef struct _VKRTTexture       *VKRTTexture;
typedef struct _VKRTGeom          *VKRTGeom;
typedef struct _VKRTGeomType      *VKRTGeomType;
typedef struct _VKRTVariable      *VKRTVariable;
typedef struct _VKRTModule        *VKRTModule;
typedef struct _VKRTGroup         *VKRTGroup;
typedef struct _VKRTRayGen        *VKRTRayGen;
typedef struct _VKRTMissProg      *VKRTMissProg;

/*! launch params (or "globals") are variables that can be put into
  device constant memory, accessible through Vulkan's push constants */
typedef struct _VKRTLaunchParams  *VKRTLaunchParams, *VKRTParams, *VKRTGlobals;

typedef enum
{
  VKRT_SBT_HITGROUPS = 0x1,
  VKRT_SBT_GEOMS     = VKRT_SBT_HITGROUPS,
  VKRT_SBT_RAYGENS   = 0x2,
  VKRT_SBT_MISSPROGS = 0x4,
  VKRT_SBT_ALL   = 0x7
} VKRTBuildSBTFlags;

typedef enum
  {
   VKRT_INVALID_TYPE = 0,

   VKRT_BUFFER=10,
   /*! a 64-bit int representing the number of elemnets in a buffer */
   VKRT_BUFFER_SIZE,
   VKRT_BUFFER_ID,
   VKRT_BUFFER_POINTER,
   VKRT_BUFPTR=VKRT_BUFFER_POINTER,

   VKRT_GROUP=20,

   /*! implicit variable of type integer that specifies the *index*
     of the given device. this variable type is implicit in the
     sense that it only gets _declared_ on the host, and gets set
     automatically during SBT creation */
   VKRT_DEVICE=30,

   /*! texture(s) */
   VKRT_TEXTURE=40,
   VKRT_TEXTURE_2D=VKRT_TEXTURE,


   /* all types that are naively copyable should be below this value,
      all that aren't should be above */
   _VKRT_BEGIN_COPYABLE_TYPES = 1000,
   
   
   VKRT_FLOAT=1000,
   VKRT_FLOAT2,
   VKRT_FLOAT3,
   VKRT_FLOAT4,

   VKRT_INT=1010,
   VKRT_INT2,
   VKRT_INT3,
   VKRT_INT4,
   
   VKRT_UINT=1020,
   VKRT_UINT2,
   VKRT_UINT3,
   VKRT_UINT4,
   
   /* 64 bit integers */
   VKRT_LONG=1030,
   VKRT_LONG2,
   VKRT_LONG3,
   VKRT_LONG4,

   VKRT_INT64   = VKRT_LONG,
   VKRT_INT64_2 = VKRT_LONG2,
   VKRT_INT64_3 = VKRT_LONG3,
   VKRT_INT64_4 = VKRT_LONG4,

   VKRT_ULONG=1040,
   VKRT_ULONG2,
   VKRT_ULONG3,
   VKRT_ULONG4,

   VKRT_UINT64   = VKRT_ULONG,
   VKRT_UINT64_2 = VKRT_ULONG2,
   VKRT_UINT64_3 = VKRT_ULONG3,
   VKRT_UINT64_4 = VKRT_ULONG4,

   VKRT_DOUBLE=1050,
   VKRT_DOUBLE2,
   VKRT_DOUBLE3,
   VKRT_DOUBLE4,
    
   VKRT_CHAR=1060,
   VKRT_CHAR2,
   VKRT_CHAR3,
   VKRT_CHAR4,

   /*! unsigend 8-bit integer */
   VKRT_UCHAR=1070,
   VKRT_UCHAR2,
   VKRT_UCHAR3,
   VKRT_UCHAR4,

   VKRT_SHORT=1080,
   VKRT_SHORT2,
   VKRT_SHORT3,
   VKRT_SHORT4,

   /*! unsigend 8-bit integer */
   VKRT_USHORT=1090,
   VKRT_USHORT2,
   VKRT_USHORT3,
   VKRT_USHORT4,

   VKRT_BOOL,
   VKRT_BOOL2,
   VKRT_BOOL3,
   VKRT_BOOL4,
   
   /*! just another name for a 64-bit data type - unlike
     VKRT_BUFFER_POINTER's (which gets translated from VKRTBuffer's
     to actual device-side poiners) these VKRT_RAW_POINTER types get
     copied binary without any translation. This is useful for
     owl-cuda interaction (where the user already has device
     pointers), but should not be used for logical buffers */
   VKRT_RAW_POINTER=VKRT_ULONG,
   VKRT_BYTE = VKRT_UCHAR,
   // VKRT_BOOL = VKRT_UCHAR,
   // VKRT_BOOL2 = VKRT_UCHAR2,
   // VKRT_BOOL3 = VKRT_UCHAR3,
   // VKRT_BOOL4 = VKRT_UCHAR4,


   /* matrix formats */
   VKRT_AFFINE3F=1300,

   /*! at least for now, use that for buffers with user-defined types:
     type then is "VKRT_USER_TYPE_BEGIN+sizeof(elementtype). Note
     that since we always _add_ the user type's size to this value
     this MUST be the last entry in the enum */
   VKRT_USER_TYPE_BEGIN=10000
  }
  VKRTDataType;

  inline size_t getSize(VKRTDataType type)
  {
        if (type == VKRT_INT) return sizeof(int32_t);
    else if (type == VKRT_INT2) return sizeof(int2);
    else if (type == VKRT_INT3) return sizeof(int3);
    else if (type == VKRT_INT4) return sizeof(int4);
    else if (type == VKRT_UINT) return sizeof(uint32_t);
    else if (type == VKRT_INT2) return sizeof(uint2);
    else if (type == VKRT_INT3) return sizeof(uint3);
    else if (type == VKRT_INT4) return sizeof(uint4);

    else if (type == VKRT_INT64) return sizeof(int64_t);
    else if (type == VKRT_INT64_2) return sizeof(int64_t) * 2;
    else if (type == VKRT_INT64_3) return sizeof(int64_t) * 3;
    else if (type == VKRT_INT64_4) return sizeof(int64_t) * 4;
    else if (type == VKRT_UINT64) return sizeof(uint64_t);
    else if (type == VKRT_UINT64_2) return sizeof(uint64_t) * 2;
    else if (type == VKRT_UINT64_3) return sizeof(uint64_t) * 3;
    else if (type == VKRT_UINT64_4) return sizeof(uint64_t) * 4;

    else if (type == VKRT_BUFFER) return sizeof(int64_t);

    else if (type == VKRT_FLOAT) return sizeof(float);
    else if (type == VKRT_FLOAT2) return sizeof(float2);
    else if (type == VKRT_FLOAT3) return sizeof(float3);
    else if (type == VKRT_FLOAT4) return sizeof(float4);
    else if (type > VKRT_USER_TYPE_BEGIN) return type - VKRT_USER_TYPE_BEGIN;
    else assert(false); return -1;// std::runtime_error("Unimplemented!");
  }

  #define VKRT_USER_TYPE(userType) ((VKRTDataType)(VKRT_USER_TYPE_BEGIN+sizeof(userType)))

typedef struct _VKRTVarDecl {
  const char *name;
  VKRTDataType type; // note, also includes size if VKRT_USER_TYPE
  uint32_t    offset;
} VKRTVarDecl;

typedef struct _VKRTVarDef {
  VKRTVarDecl decl;
  void* data = nullptr;
} VKRTVarDef;

VKRT_API VKRTModule vkrtModuleCreate(VKRTContext context, const char* spvCode);
VKRT_API void vkrtBuildPrograms(VKRTContext context);
VKRT_API void vkrtBuildPipeline(VKRTContext context);
VKRT_API void vkrtBuildSBT(VKRTContext context,
                         VKRTBuildSBTFlags flags VKRT_IF_CPP(=VKRT_SBT_ALL));

/*! creates a new device context with the gives list of devices. 

  If requested device IDs list if null it implicitly refers to the
  list "0,1,2,...."; if numDevices <= 0 it automatically refers to
  "all devices you can find". Examples:

  - vkrtContextCreate(nullptr,1) creates one device on the first GPU

  - vkrtContextCreate(nullptr,0) creates a context across all GPUs in
  the system

  - int gpu=2;vkrtContextCreate(&gpu,1) will create a context on GPU #2
  (where 2 refers to the CUDA device ordinal; from that point on, from
  vkrt's standpoint (eg, during vkrtBufferGetPointer() this GPU will
  from that point on be known as device #0 */
VKRT_API VKRTContext
vkrtContextCreate(int32_t *requestedDeviceIDs VKRT_IF_CPP(=nullptr),
                 int numDevices VKRT_IF_CPP(=0));

VKRT_API void
vkrtContextDestroy(VKRTContext context);


VKRT_API VKRTRayGen
vkrtRayGenCreate(VKRTContext  context,
                 VKRTModule module,
                 const char *programName,
                 size_t      sizeOfVarStruct,
                 VKRTVarDecl *vars,
                 int         numVars);
                
VKRT_API void 
vkrtRayGenRelease(VKRTRayGen rayGen);

VKRT_API VKRTMissProg
vkrtMissProgCreate(VKRTContext  context,
                   VKRTModule module,
                   const char *programName,
                   size_t      sizeOfVarStruct,
                   VKRTVarDecl *vars,
                   int         numVars);

/*! sets the given miss program for the given ray type */
VKRT_API void
vkrtMissProgSet(VKRTContext  context,
               int rayType,
               VKRTMissProg missProgToUse);

VKRT_API void 
vkrtMissProgRelease(VKRTMissProg missProg);

/*! creates a buffer that uses host pinned memory; that memory is
pinned on the host and accessible to all devices */
VKRT_API VKRTBuffer
vkrtHostPinnedBufferCreate(VKRTContext context, VKRTDataType type, size_t count);

/*! Releases all underlying Vulkan resources for the given buffer and frees any 
  underlying memory*/
VKRT_API void
vkrtBufferRelease(VKRTBuffer buffer);

/*! returns the device pointer of the given pointer for the given
  device ID. For host-pinned or managed memory buffers (where the
  buffer is shared across all devices) this pointer should be the
  same across all devices (and even be accessible on the host); for
  device buffers each device *may* see this buffer under a different
  address, and that address is not valid on the host. Note this
  function is paricuarly useful for CUDA-interop; allowing to
  cudaMemcpy to/from an owl buffer directly from CUDA code 
  
  // TODO! update for Vulkan...
  */
VKRT_API void *
vkrtBufferGetPointer(VKRTBuffer buffer, int deviceID VKRT_IF_CPP(=0));


VKRT_API void
vkrtBufferMap(VKRTBuffer buffer, int deviceID VKRT_IF_CPP(=0));

VKRT_API void
vkrtBufferUnmap(VKRTBuffer buffer, int deviceID VKRT_IF_CPP(=0));

/*! Executes a ray tracing pipeline with the given raygen program. 
  This call will block until the raygen program returns. */
VKRT_API void
vkrtRayGenLaunch2D(VKRTContext context, VKRTRayGen rayGen, int dims_x, int dims_y);

/*! 3D-launch variant of \see vkrtRayGenLaunch2D */
VKRT_API void
vkrtRayGenLaunch3D(VKRTContext context, VKRTRayGen rayGen, int dims_x, int dims_y, int dims_z);


// ------------------------------------------------------------------
// setters for "meta" types
// ------------------------------------------------------------------

// setters for variables on "RayGen"s
// VKRT_API void vkrtRayGenSetTexture(VKRTRayGen raygen, const char *name, VKRTTexture val);
// VKRT_API void vkrtRayGenSetPointer(VKRTRayGen raygen, const char *name, const void *val);
VKRT_API void vkrtRayGenSetBuffer(VKRTRayGen raygen, const char *name, VKRTBuffer val);
// VKRT_API void vkrtRayGenSetGroup(VKRTRayGen raygen, const char *name, VKRTGroup val);
VKRT_API void vkrtRayGenSetRaw(VKRTRayGen raygen, const char *name, const void *val);

// // setters for variables on "Geom"s
// VKRT_API void vkrtGeomSetTexture(VKRTGeom obj, const char *name, VKRTTexture val);
// VKRT_API void vkrtGeomSetPointer(VKRTGeom obj, const char *name, const void *val);
// VKRT_API void vkrtGeomSetBuffer(VKRTGeom obj, const char *name, VKRTBuffer val);
// VKRT_API void vkrtGeomSetGroup(VKRTGeom obj, const char *name, VKRTGroup val);
// VKRT_API void vkrtGeomSetRaw(VKRTGeom obj, const char *name, const void *val);

// // setters for variables on "Params"s
// VKRT_API void vkrtParamsSetTexture(VKRTParams obj, const char *name, VKRTTexture val);
// VKRT_API void vkrtParamsSetPointer(VKRTParams obj, const char *name, const void *val);
// VKRT_API void vkrtParamsSetBuffer(VKRTParams obj, const char *name, VKRTBuffer val);
// VKRT_API void vkrtParamsSetGroup(VKRTParams obj, const char *name, VKRTGroup val);
// VKRT_API void vkrtParamsSetRaw(VKRTParams obj, const char *name, const void *val);

// setters for variables on "MissProg"s
// VKRT_API void vkrtMissProgSetTexture(VKRTMissProg missprog, const char *name, VKRTTexture val);
// VKRT_API void vkrtMissProgSetPointer(VKRTMissProg missprog, const char *name, const void *val);
// VKRT_API void vkrtMissProgSetBuffer(VKRTMissProg missprog, const char *name, VKRTBuffer val);
// VKRT_API void vkrtMissProgSetGroup(VKRTMissProg missprog, const char *name, VKRTGroup val);
VKRT_API void vkrtMissProgSetRaw(VKRTMissProg missprog, const char *name, const void *val);