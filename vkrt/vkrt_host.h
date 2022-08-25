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

#include "hlsl++.h"
using namespace hlslpp;

#include <sys/types.h>
#include <stdint.h>

#include <assert.h>
#include <unordered_map>
#include <string>
#include <vector>

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
typedef struct _VKRTAccel         *VKRTAccel;
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

/*! enum that specifies the different possible memory layouts for
  passing transformation matrices */
typedef enum
  {
   /*! A 4x3-float column-major matrix format, where a matrix is
     specified through four float3's, the first three being the basis
     vectors of the linear transform, and the fourth one the
     translation part. */
   VKRT_MATRIX_FORMAT_COLUMN_MAJOR=0,

   /*! 3x4-float *row-major* layout as preferred by vulkan matching VkTransformMatrixKHR;
     not that, in this case, it doesn't matter if it's a 4x3 or 4x4 matrix,
     as the last row in a 4x4 row major matrix can simply be ignored */
   VKRT_MATRIX_FORMAT_ROW_MAJOR
  } VKRTMatrixFormat;

typedef enum
  {
   VKRT_INVALID_TYPE = 0,

   VKRT_BUFFER=10,
   /*! a 64-bit int representing the number of elements in a buffer */
   VKRT_BUFFER_SIZE,
   VKRT_BUFFER_ID,
   VKRT_BUFFER_POINTER,
   VKRT_BUFPTR=VKRT_BUFFER_POINTER,

   VKRT_ACCEL=20,

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

   VKRT_INT32_T   = VKRT_INT,
   VKRT_INT32_T2 = VKRT_INT2,
   VKRT_INT32_T3 = VKRT_INT3,
   VKRT_INT32_T4 = VKRT_INT4,

   VKRT_UINT=1020,
   VKRT_UINT2,
   VKRT_UINT3,
   VKRT_UINT4,

   VKRT_UINT32_T  = VKRT_UINT,
   VKRT_UINT32_T2 = VKRT_UINT2,
   VKRT_UINT32_T3 = VKRT_UINT3,
   VKRT_UINT32_T4 = VKRT_UINT4,

   /* 64 bit integers */
   VKRT_LONG=1030,
   VKRT_LONG2,
   VKRT_LONG3,
   VKRT_LONG4,

   VKRT_INT64_T  = VKRT_LONG,
   VKRT_INT64_T2 = VKRT_LONG2,
   VKRT_INT64_T3 = VKRT_LONG3,
   VKRT_INT64_T4 = VKRT_LONG4,

   VKRT_ULONG=1040,
   VKRT_ULONG2,
   VKRT_ULONG3,
   VKRT_ULONG4,

   VKRT_UINT64_T = VKRT_ULONG,
   VKRT_UINT64_T2 = VKRT_ULONG2,
   VKRT_UINT64_T3 = VKRT_ULONG3,
   VKRT_UINT64_T4 = VKRT_ULONG4,

   VKRT_DOUBLE=1050,
   VKRT_DOUBLE2,
   VKRT_DOUBLE3,
   VKRT_DOUBLE4,

   VKRT_CHAR=1060,
   VKRT_CHAR2,
   VKRT_CHAR3,
   VKRT_CHAR4,

   VKRT_INT8_T   = VKRT_CHAR,
   VKRT_INT8_T2 = VKRT_CHAR2,
   VKRT_INT8_T3 = VKRT_CHAR3,
   VKRT_INT8_T4 = VKRT_CHAR4,

   /*! unsigend 8-bit integer */
   VKRT_UCHAR=1070,
   VKRT_UCHAR2,
   VKRT_UCHAR3,
   VKRT_UCHAR4,

   VKRT_UINT8_T  = VKRT_UCHAR,
   VKRT_UINT8_T2 = VKRT_UCHAR2,
   VKRT_UINT8_T3 = VKRT_UCHAR3,
   VKRT_UINT8_T4 = VKRT_UCHAR4,

   VKRT_SHORT=1080,
   VKRT_SHORT2,
   VKRT_SHORT3,
   VKRT_SHORT4,

   VKRT_INT16_T  = VKRT_SHORT,
   VKRT_INT16_T2 = VKRT_SHORT2,
   VKRT_INT16_T3 = VKRT_SHORT3,
   VKRT_INT16_T4 = VKRT_SHORT4,

   /*! unsigend 8-bit integer */
   VKRT_USHORT=1090,
   VKRT_USHORT2,
   VKRT_USHORT3,
   VKRT_USHORT4,

   VKRT_UINT16_T  = VKRT_USHORT,
   VKRT_UINT16_T2 = VKRT_USHORT2,
   VKRT_UINT16_T3 = VKRT_USHORT3,
   VKRT_UINT16_T4 = VKRT_USHORT4,

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
   
   /* A type matching VkTransformMatrixKHR, row major 3x4 */
   VKRT_TRANSFORM=1400,

   /*! at least for now, use that for buffers with user-defined types:
     type then is "VKRT_USER_TYPE_BEGIN+sizeof(elementtype). Note
     that since we always _add_ the user type's size to this value
     this MUST be the last entry in the enum */
   VKRT_USER_TYPE_BEGIN=10000
  }
  VKRTDataType;

typedef enum
  {
   VKRT_USER,
   VKRT_TRIANGLES,
  //  VKRT_CURVES
  }
  VKRTGeomKind;

  inline size_t getSize(VKRTDataType type)
  {
         if (type == VKRT_INT8_T)  return sizeof(int8_t);
    else if (type == VKRT_INT8_T2) return sizeof(int8_t) * 2;
    else if (type == VKRT_INT8_T3) return sizeof(int8_t) * 3;
    else if (type == VKRT_INT8_T4) return sizeof(int8_t) * 4;

    else if (type == VKRT_UINT8_T)  return sizeof(uint8_t);
    else if (type == VKRT_UINT8_T2) return sizeof(uint8_t) * 2;
    else if (type == VKRT_UINT8_T3) return sizeof(uint8_t) * 3;
    else if (type == VKRT_UINT8_T4) return sizeof(uint8_t) * 4;

    else if (type == VKRT_INT16_T)  return sizeof(int16_t);
    else if (type == VKRT_INT16_T2) return sizeof(int16_t) * 2;
    else if (type == VKRT_INT16_T3) return sizeof(int16_t) * 3;
    else if (type == VKRT_INT16_T4) return sizeof(int16_t) * 4;

    else if (type == VKRT_UINT16_T)  return sizeof(uint16_t);
    else if (type == VKRT_UINT16_T2) return sizeof(uint16_t) * 2;
    else if (type == VKRT_UINT16_T3) return sizeof(uint16_t) * 3;
    else if (type == VKRT_UINT16_T4) return sizeof(uint16_t) * 4;

    else if (type == VKRT_INT32_T)  return sizeof(int32_t);
    else if (type == VKRT_INT32_T2) return sizeof(int32_t) * 2;
    else if (type == VKRT_INT32_T3) return sizeof(int32_t) * 3;
    else if (type == VKRT_INT32_T4) return sizeof(int32_t) * 4;

    else if (type == VKRT_UINT32_T)  return sizeof(uint32_t);
    else if (type == VKRT_UINT32_T2) return sizeof(uint32_t) * 2;
    else if (type == VKRT_UINT32_T3) return sizeof(uint32_t) * 3;
    else if (type == VKRT_UINT32_T4) return sizeof(uint32_t) * 4;

    else if (type == VKRT_INT64_T)  return sizeof(int64_t);
    else if (type == VKRT_INT64_T2) return sizeof(int64_t) * 2;
    else if (type == VKRT_INT64_T3) return sizeof(int64_t) * 3;
    else if (type == VKRT_INT64_T4) return sizeof(int64_t) * 4;

    else if (type == VKRT_UINT64_T)  return sizeof(uint64_t);
    else if (type == VKRT_UINT64_T2) return sizeof(uint64_t) * 2;
    else if (type == VKRT_UINT64_T3) return sizeof(uint64_t) * 3;
    else if (type == VKRT_UINT64_T4) return sizeof(uint64_t) * 4;

    else if (type == VKRT_INT64_T)  return sizeof(uint64_t);
    else if (type == VKRT_INT64_T2) return sizeof(uint64_t) * 2;
    else if (type == VKRT_INT64_T3) return sizeof(uint64_t) * 3;
    else if (type == VKRT_INT64_T4) return sizeof(uint64_t) * 4;

    else if (type == VKRT_BUFFER) return sizeof(uint64_t);
    else if (type == VKRT_BUFPTR) return sizeof(uint64_t);

    else if (type == VKRT_FLOAT) return sizeof(float);
    else if (type == VKRT_FLOAT2) return sizeof(float) * 2;
    else if (type == VKRT_FLOAT3) return sizeof(float) * 3;
    else if (type == VKRT_FLOAT4) return sizeof(float) * 4;

    else if (type == VKRT_DOUBLE) return sizeof(double);
    else if (type == VKRT_DOUBLE2) return sizeof(double) * 2;
    else if (type == VKRT_DOUBLE3) return sizeof(double) * 3;
    else if (type == VKRT_DOUBLE4) return sizeof(double) * 4;

    else if (type == VKRT_BOOL) return sizeof(bool);
    else if (type == VKRT_BOOL2) return sizeof(bool) * 2;
    else if (type == VKRT_BOOL3) return sizeof(bool) * 3;
    else if (type == VKRT_BOOL4) return sizeof(bool) * 4;

    else if (type == VKRT_BUFFER) return sizeof(uint64_t);
    else if (type == VKRT_ACCEL) return sizeof(uint64_t);
    else if (type == VKRT_TRANSFORM) return sizeof(float) * 3 * 4;

    // User Types have size encoded in their type enum
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

inline std::unordered_map<std::string, VKRTVarDef> checkAndPackVariables(
  const VKRTVarDecl *vars, int numVars)
{
  if (vars == nullptr && (numVars == 0 || numVars == -1))
    return {};

  // *copy* the vardecls here, so we can catch any potential memory
  // *access errors early

  assert(vars);
  if (numVars == -1) {
    // using -1 as count value for a variable list means the list is
    // null-terminated... so just count it
    for (numVars = 0; vars[numVars].name != nullptr; numVars++);
  }
  std::unordered_map<std::string, VKRTVarDef> varDefs;
  for (int i=0;i<numVars;i++) {
    assert(vars[i].name != nullptr);
    varDefs[vars[i].name].decl = vars[i];

    // allocate depending on the size of the variable...
    varDefs[vars[i].name].data = malloc(getSize(vars[i].type));
  }
  return varDefs;
}

inline std::vector<VKRTVarDecl> getDecls(
  std::unordered_map<std::string, VKRTVarDef> vars)
{
  std::vector<VKRTVarDecl> decls;
  for (auto &it : vars) {
    decls.push_back(it.second.decl);
  }
  return decls;
}

VKRT_API VKRTModule vkrtModuleCreate(VKRTContext context, const char* spvCode);
VKRT_API void vkrtModuleDestroy(VKRTModule module);

VKRT_API VKRTGeom
vkrtGeomCreate(VKRTContext  context,
              VKRTGeomType type);

VKRT_API void
vkrtGeomDestroy(VKRTGeom geometry);

// ==================================================================
// "Triangles" functions
// ==================================================================
VKRT_API void vkrtTrianglesSetVertices(VKRTGeom triangles,
                                      VKRTBuffer vertices,
                                      size_t count,
                                      size_t stride,
                                      size_t offset);
// VKRT_API void vkrtTrianglesSetMotionVertices(VKRTGeom triangles,
//                                            /*! number of vertex arrays
//                                                passed here, the first
//                                                of those is for t=0,
//                                                thelast for t=1,
//                                                everything is linearly
//                                                interpolated
//                                                in-between */
//                                            size_t    numKeys,
//                                            VKRTBuffer *vertexArrays,
//                                            size_t count,
//                                            size_t stride,
//                                            size_t offset);
VKRT_API void vkrtTrianglesSetIndices(VKRTGeom triangles,
                                     VKRTBuffer indices,
                                     size_t count,
                                     size_t stride,
                                     size_t offset);

/*! technically this is currently a no-op, but we have this function around to
  match OWL */
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

/*! set number of ray types to be used in this context; this should be
  done before any programs, pipelines, geometries, etc get
  created */
VKRT_API void
vkrtContextSetRayTypeCount(VKRTContext context,
                           size_t numRayTypes);

VKRT_API VKRTRayGen
vkrtRayGenCreate(VKRTContext  context,
                 VKRTModule module,
                 const char *programName,
                 size_t      sizeOfVarStruct,
                 VKRTVarDecl *vars,
                 int         numVars);

VKRT_API void
vkrtRayGenDestroy(VKRTRayGen rayGen);

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
vkrtMissProgDestroy(VKRTMissProg missProg);

// ------------------------------------------------------------------
/*! create a new acceleration structure for AABB geometries.

  \param numGeometries Number of geometries in this acceleration structure, must
  be non-zero.

  \param arrayOfChildGeoms A array of 'numGeometries' child
  geometries. Every geom in this array must be a valid vkrt geometry
  created with vkrtGeomCreate, and must be of a VKRT_GEOM_USER
  type.

  \param flags reserved for future use
*/
VKRT_API VKRTAccel
vkrtAABBAccelCreate(VKRTContext context,
                    size_t       numGeometries,
                    VKRTGeom    *arrayOfChildGeoms,
                    unsigned int flags VKRT_IF_CPP(=0));


// ------------------------------------------------------------------
/*! create a new acceleration structure for triangle geometries.

  \param numGeometries Number of geometries in this acceleration structure, must
  be non-zero.

  \param arrayOfChildGeoms A array of 'numGeometries' child
  geometries. Every geom in this array must be a valid vkrt geometry
  created with vkrtGeomCreate, and must be of a VKRT_GEOM_TRIANGLES
  type.

  \param flags reserved for future use
*/
VKRT_API VKRTAccel
vkrtTrianglesAccelCreate(VKRTContext context,
                            size_t     numGeometries,
                            VKRTGeom   *arrayOfChildGeoms,
                            unsigned int flags VKRT_IF_CPP(=0));

VKRT_API void 
vkrtTrianglesAccelSetTransforms(VKRTAccel trianglesAccel,
                                VKRTBuffer transforms//,
                                // size_t offset, // maybe I can support these too?
                                // size_t stride  // maybe I can support these too?
                                );

// // ------------------------------------------------------------------
// /*! create a new acceleration structure for "curves" geometries.

//   \param numGeometries Number of geometries in this acceleration structure,
//   must be non-zero.

//   \param arrayOfChildGeoms A array of 'numGeometries' child
//   geometries. Every geom in this array must be a valid vkrt geometry
//   created with vkrtGeomCreate, and must be of a VKRT_GEOM_CURVES
//   type.

//   \param flags reserved for future use

//   Note that in order to use curves geometries you _have_ to call
//   vkrtEnableCurves() before curves are used; in particular, curves
//   _have_ to already be enabled when the pipeline gets compiled.
// */
// VKRT_API VKRTAccel
// vkrtCurvesAccelCreate(VKRTContext context,
//                          size_t     numCurveGeometries,
//                          VKRTGeom   *curveGeometries,
//                          unsigned int flags VKRT_IF_CPP(=0));

// ------------------------------------------------------------------
/*! create a new instance acceleration structure with given number of
  instances. 
  
  \param numAccels Number of acceleration structures instantiated in the leaves
  of this acceleration structure, must be non-zero.

  \param arrayOfAccels A array of 'numInstances' child
  acceleration structures. No accel in this array can be an instance accel.  

  \param flags reserved for future use
*/
VKRT_API VKRTAccel
vkrtInstanceAccelCreate(VKRTContext context,
                        size_t numAccels,
                        VKRTAccel *arrayOfAccels,
                        unsigned int flags VKRT_IF_CPP(=0));

VKRT_API void 
vkrtInstanceAccelSetTransforms(VKRTAccel instanceAccel,
                               VKRTBuffer transforms//,
                               // size_t offset, // maybe I can support these too?
                               // size_t stride  // maybe I can support these too?
                               );

/*! sets the list of IDs to use for the child instnaces. By default
    the instance ID of child #i is simply i, but optix allows to
    specify a user-defined instnace ID for each instance, which with
    owl can be done through this array. Array size must match number
    of instances in the specified group */
VKRT_API void
vkrtInstanceAccelSetIDs(VKRTAccel instanceAccel,
                        const uint32_t *instanceIDs);

VKRT_API void
vkrtInstanceAccelSetVisibilityMasks(VKRTAccel instanceAccel,
                                    const uint8_t *visibilityMasks);

VKRT_API void
vkrtAccelDestroy(VKRTAccel accel);

VKRT_API void vkrtAccelBuild(VKRTAccel accel, VKRTModule module);

VKRT_API void vkrtAccelRefit(VKRTAccel accel, VKRTModule module);

VKRT_API VKRTGeomType
vkrtGeomTypeCreate(VKRTContext  context,
                   VKRTGeomKind kind,
                   size_t       sizeOfVarStruct,
                   VKRTVarDecl  *vars,
                   int          numVars);

VKRT_API void
vkrtGeomTypeDestroy(VKRTGeomType geomType);

VKRT_API void
vkrtGeomTypeSetClosestHit(VKRTGeomType type,
                          int rayType,
                          VKRTModule module,
                          const char *progName);

VKRT_API void
vkrtGeomTypeSetAnyHit(VKRTGeomType type,
                      int rayType,
                      VKRTModule module,
                      const char *progName);

VKRT_API void
vkrtGeomTypeSetIntersectProg(VKRTGeomType type,
                             int rayType,
                             VKRTModule module,
                             const char *progName);

VKRT_API void
vkrtGeomTypeSetBoundsProg(VKRTGeomType type,
                          VKRTModule module,
                          const char *progName);

/*! creates a buffer that uses host pinned memory; that memory is
pinned on the host and accessible to all devices */
VKRT_API VKRTBuffer
vkrtHostPinnedBufferCreate(VKRTContext context, VKRTDataType type, size_t count, 
  const void* init VKRT_IF_CPP(= nullptr));

/*! creates a device buffer where every device has its own local copy
  of the given buffer */
VKRT_API VKRTBuffer
vkrtDeviceBufferCreate(VKRTContext context, VKRTDataType type, size_t count, 
  const void* init VKRT_IF_CPP(= nullptr));

/*! Destroys all underlying Vulkan resources for the given buffer and frees any
  underlying memory*/
VKRT_API void
vkrtBufferDestroy(VKRTBuffer buffer);

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


#ifdef __cplusplus
// ------------------------------------------------------------------
// setters for variables of type "bool" (bools only on c++)
// ------------------------------------------------------------------

// setters for variables on "RayGen"s
VKRT_API void vkrtRayGenSet1b(VKRTRayGen raygen, const char *name, bool val);
VKRT_API void vkrtRayGenSet2b(VKRTRayGen raygen, const char *name, bool x, bool y);
VKRT_API void vkrtRayGenSet3b(VKRTRayGen raygen, const char *name, bool x, bool y, bool z);
VKRT_API void vkrtRayGenSet4b(VKRTRayGen raygen, const char *name, bool x, bool y, bool z, bool w);
VKRT_API void vkrtRayGenSet2bv(VKRTRayGen raygen, const char *name, const bool *val);
VKRT_API void vkrtRayGenSet3bv(VKRTRayGen raygen, const char *name, const bool *val);
VKRT_API void vkrtRayGenSet4bv(VKRTRayGen raygen, const char *name, const bool *val);

// setters for variables on "MissProg"s
VKRT_API void vkrtMissProgSet1b(VKRTMissProg missprog, const char *name, bool val);
VKRT_API void vkrtMissProgSet2b(VKRTMissProg missprog, const char *name, bool x, bool y);
VKRT_API void vkrtMissProgSet3b(VKRTMissProg missprog, const char *name, bool x, bool y, bool z);
VKRT_API void vkrtMissProgSet4b(VKRTMissProg missprog, const char *name, bool x, bool y, bool z, bool w);
VKRT_API void vkrtMissProgSet2bv(VKRTMissProg missprog, const char *name, const bool *val);
VKRT_API void vkrtMissProgSet3bv(VKRTMissProg missprog, const char *name, const bool *val);
VKRT_API void vkrtMissProgSet4bv(VKRTMissProg missprog, const char *name, const bool *val);

// setters for variables on "Geom"s
// VKRT_API void vkrtGeomSet1b(VKRTGeom geom, const char *name, bool val);
// VKRT_API void vkrtGeomSet2b(VKRTGeom geom, const char *name, bool x, bool y);
// VKRT_API void vkrtGeomSet3b(VKRTGeom geom, const char *name, bool x, bool y, bool z);
// VKRT_API void vkrtGeomSet4b(VKRTGeom geom, const char *name, bool x, bool y, bool z, bool w);
// VKRT_API void vkrtGeomSet2bv(VKRTGeom geom, const char *name, const bool *val);
// VKRT_API void vkrtGeomSet3bv(VKRTGeom geom, const char *name, const bool *val);
// VKRT_API void vkrtGeomSet4bv(VKRTGeom geom, const char *name, const bool *val);

// // setters for variables on "Params"s
// VKRT_API void vkrtParamsSet1b(OWLParams var, const char *name, bool val);
// VKRT_API void vkrtParamsSet2b(OWLParams var, const char *name, bool x, bool y);
// VKRT_API void vkrtParamsSet3b(OWLParams var, const char *name, bool x, bool y, bool z);
// VKRT_API void vkrtParamsSet4b(OWLParams var, const char *name, bool x, bool y, bool z, bool w);
// VKRT_API void vkrtParamsSet2bv(OWLParams var, const char *name, const bool *val);
// VKRT_API void vkrtParamsSet3bv(OWLParams var, const char *name, const bool *val);
// VKRT_API void vkrtParamsSet4bv(OWLParams var, const char *name, const bool *val);
#endif

// ------------------------------------------------------------------
// setters for variables of type "char"
// ------------------------------------------------------------------

// setters for variables on "RayGen"s
VKRT_API void vkrtRayGenSet1c(VKRTRayGen raygen, const char *name, int8_t val);
VKRT_API void vkrtRayGenSet2c(VKRTRayGen raygen, const char *name, int8_t x, int8_t y);
VKRT_API void vkrtRayGenSet3c(VKRTRayGen raygen, const char *name, int8_t x, int8_t y, int8_t z);
VKRT_API void vkrtRayGenSet4c(VKRTRayGen raygen, const char *name, int8_t x, int8_t y, int8_t z, int8_t w);
VKRT_API void vkrtRayGenSet2cv(VKRTRayGen raygen, const char *name, const int8_t *val);
VKRT_API void vkrtRayGenSet3cv(VKRTRayGen raygen, const char *name, const int8_t *val);
VKRT_API void vkrtRayGenSet4cv(VKRTRayGen raygen, const char *name, const int8_t *val);

// setters for variables on "MissProg"s
VKRT_API void vkrtMissProgSet1c(VKRTMissProg missprog, const char *name, int8_t val);
VKRT_API void vkrtMissProgSet2c(VKRTMissProg missprog, const char *name, int8_t x, int8_t y);
VKRT_API void vkrtMissProgSet3c(VKRTMissProg missprog, const char *name, int8_t x, int8_t y, int8_t z);
VKRT_API void vkrtMissProgSet4c(VKRTMissProg missprog, const char *name, int8_t x, int8_t y, int8_t z, int8_t w);
VKRT_API void vkrtMissProgSet2cv(VKRTMissProg missprog, const char *name, const int8_t *val);
VKRT_API void vkrtMissProgSet3cv(VKRTMissProg missprog, const char *name, const int8_t *val);
VKRT_API void vkrtMissProgSet4cv(VKRTMissProg missprog, const char *name, const int8_t *val);

// setters for variables on "Geom"s
VKRT_API void vkrtGeomSet1c(VKRTGeom geom, const char *name, int8_t val);
VKRT_API void vkrtGeomSet2c(VKRTGeom geom, const char *name, int8_t x, int8_t y);
VKRT_API void vkrtGeomSet3c(VKRTGeom geom, const char *name, int8_t x, int8_t y, int8_t z);
VKRT_API void vkrtGeomSet4c(VKRTGeom geom, const char *name, int8_t x, int8_t y, int8_t z, int8_t w);
VKRT_API void vkrtGeomSet2cv(VKRTGeom geom, const char *name, const int8_t *val);
VKRT_API void vkrtGeomSet3cv(VKRTGeom geom, const char *name, const int8_t *val);
VKRT_API void vkrtGeomSet4cv(VKRTGeom geom, const char *name, const int8_t *val);

// setters for variables on "Params"s
// VKRT_API void vkrtParamsSet1c(OWLParams obj, const char *name, int8_t val);
// VKRT_API void vkrtParamsSet2c(OWLParams obj, const char *name, int8_t x, int8_t y);
// VKRT_API void vkrtParamsSet3c(OWLParams obj, const char *name, int8_t x, int8_t y, int8_t z);
// VKRT_API void vkrtParamsSet4c(OWLParams obj, const char *name, int8_t x, int8_t y, int8_t z, int8_t w);
// VKRT_API void vkrtParamsSet2cv(OWLParams obj, const char *name, const int8_t *val);
// VKRT_API void vkrtParamsSet3cv(OWLParams obj, const char *name, const int8_t *val);
// VKRT_API void vkrtParamsSet4cv(OWLParams obj, const char *name, const int8_t *val);

// ------------------------------------------------------------------
// setters for variables of type "uint8_t"
// ------------------------------------------------------------------

// setters for variables on "RayGen"s
VKRT_API void vkrtRayGenSet1uc(VKRTRayGen raygen, const char *name, uint8_t val);
VKRT_API void vkrtRayGenSet2uc(VKRTRayGen raygen, const char *name, uint8_t x, uint8_t y);
VKRT_API void vkrtRayGenSet3uc(VKRTRayGen raygen, const char *name, uint8_t x, uint8_t y, uint8_t z);
VKRT_API void vkrtRayGenSet4uc(VKRTRayGen raygen, const char *name, uint8_t x, uint8_t y, uint8_t z, uint8_t w);
VKRT_API void vkrtRayGenSet2ucv(VKRTRayGen raygen, const char *name, const uint8_t *val);
VKRT_API void vkrtRayGenSet3ucv(VKRTRayGen raygen, const char *name, const uint8_t *val);
VKRT_API void vkrtRayGenSet4ucv(VKRTRayGen raygen, const char *name, const uint8_t *val);

// setters for variables on "MissProg"s
VKRT_API void vkrtMissProgSet1uc(VKRTMissProg missprog, const char *name, uint8_t val);
VKRT_API void vkrtMissProgSet2uc(VKRTMissProg missprog, const char *name, uint8_t x, uint8_t y);
VKRT_API void vkrtMissProgSet3uc(VKRTMissProg missprog, const char *name, uint8_t x, uint8_t y, uint8_t z);
VKRT_API void vkrtMissProgSet4uc(VKRTMissProg missprog, const char *name, uint8_t x, uint8_t y, uint8_t z, uint8_t w);
VKRT_API void vkrtMissProgSet2ucv(VKRTMissProg missprog, const char *name, const uint8_t *val);
VKRT_API void vkrtMissProgSet3ucv(VKRTMissProg missprog, const char *name, const uint8_t *val);
VKRT_API void vkrtMissProgSet4ucv(VKRTMissProg missprog, const char *name, const uint8_t *val);

// setters for variables on "Geom"s
VKRT_API void vkrtGeomSet1uc(VKRTGeom geom, const char *name, uint8_t val);
VKRT_API void vkrtGeomSet2uc(VKRTGeom geom, const char *name, uint8_t x, uint8_t y);
VKRT_API void vkrtGeomSet3uc(VKRTGeom geom, const char *name, uint8_t x, uint8_t y, uint8_t z);
VKRT_API void vkrtGeomSet4uc(VKRTGeom geom, const char *name, uint8_t x, uint8_t y, uint8_t z, uint8_t w);
VKRT_API void vkrtGeomSet2ucv(VKRTGeom geom, const char *name, const uint8_t *val);
VKRT_API void vkrtGeomSet3ucv(VKRTGeom geom, const char *name, const uint8_t *val);
VKRT_API void vkrtGeomSet4ucv(VKRTGeom geom, const char *name, const uint8_t *val);

// setters for variables on "Params"s
// VKRT_API void vkrtParamsSet1uc(OWLParams obj, const char *name, uint8_t val);
// VKRT_API void vkrtParamsSet2uc(OWLParams obj, const char *name, uint8_t x, uint8_t y);
// VKRT_API void vkrtParamsSet3uc(OWLParams obj, const char *name, uint8_t x, uint8_t y, uint8_t z);
// VKRT_API void vkrtParamsSet4uc(OWLParams obj, const char *name, uint8_t x, uint8_t y, uint8_t z, uint8_t w);
// VKRT_API void vkrtParamsSet2ucv(OWLParams obj, const char *name, const uint8_t *val);
// VKRT_API void vkrtParamsSet3ucv(OWLParams obj, const char *name, const uint8_t *val);
// VKRT_API void vkrtParamsSet4ucv(OWLParams obj, const char *name, const uint8_t *val);

// ------------------------------------------------------------------
// setters for variables of type "int16_t"
// ------------------------------------------------------------------

// setters for variables on "RayGen"s
VKRT_API void vkrtRayGenSet1s(VKRTRayGen raygen, const char *name, int16_t val);
VKRT_API void vkrtRayGenSet2s(VKRTRayGen raygen, const char *name, int16_t x, int16_t y);
VKRT_API void vkrtRayGenSet3s(VKRTRayGen raygen, const char *name, int16_t x, int16_t y, int16_t z);
VKRT_API void vkrtRayGenSet4s(VKRTRayGen raygen, const char *name, int16_t x, int16_t y, int16_t z, int16_t w);
VKRT_API void vkrtRayGenSet2sv(VKRTRayGen raygen, const char *name, const int16_t *val);
VKRT_API void vkrtRayGenSet3sv(VKRTRayGen raygen, const char *name, const int16_t *val);
VKRT_API void vkrtRayGenSet4sv(VKRTRayGen raygen, const char *name, const int16_t *val);

// setters for variables on "MissProg"s
VKRT_API void vkrtMissProgSet1s(VKRTMissProg missprog, const char *name, int16_t val);
VKRT_API void vkrtMissProgSet2s(VKRTMissProg missprog, const char *name, int16_t x, int16_t y);
VKRT_API void vkrtMissProgSet3s(VKRTMissProg missprog, const char *name, int16_t x, int16_t y, int16_t z);
VKRT_API void vkrtMissProgSet4s(VKRTMissProg missprog, const char *name, int16_t x, int16_t y, int16_t z, int16_t w);
VKRT_API void vkrtMissProgSet2sv(VKRTMissProg missprog, const char *name, const int16_t *val);
VKRT_API void vkrtMissProgSet3sv(VKRTMissProg missprog, const char *name, const int16_t *val);
VKRT_API void vkrtMissProgSet4sv(VKRTMissProg missprog, const char *name, const int16_t *val);

// setters for variables on "Geom"s
VKRT_API void vkrtGeomSet1s(VKRTGeom geom, const char *name, int16_t val);
VKRT_API void vkrtGeomSet2s(VKRTGeom geom, const char *name, int16_t x, int16_t y);
VKRT_API void vkrtGeomSet3s(VKRTGeom geom, const char *name, int16_t x, int16_t y, int16_t z);
VKRT_API void vkrtGeomSet4s(VKRTGeom geom, const char *name, int16_t x, int16_t y, int16_t z, int16_t w);
VKRT_API void vkrtGeomSet2sv(VKRTGeom geom, const char *name, const int16_t *val);
VKRT_API void vkrtGeomSet3sv(VKRTGeom geom, const char *name, const int16_t *val);
VKRT_API void vkrtGeomSet4sv(VKRTGeom geom, const char *name, const int16_t *val);

// setters for variables on "Params"s
// VKRT_API void vkrtParamsSet1s(OWLParams obj, const char *name, int16_t val);
// VKRT_API void vkrtParamsSet2s(OWLParams obj, const char *name, int16_t x, int16_t y);
// VKRT_API void vkrtParamsSet3s(OWLParams obj, const char *name, int16_t x, int16_t y, int16_t z);
// VKRT_API void vkrtParamsSet4s(OWLParams obj, const char *name, int16_t x, int16_t y, int16_t z, int16_t w);
// VKRT_API void vkrtParamsSet2sv(OWLParams obj, const char *name, const int16_t *val);
// VKRT_API void vkrtParamsSet3sv(OWLParams obj, const char *name, const int16_t *val);
// VKRT_API void vkrtParamsSet4sv(OWLParams obj, const char *name, const int16_t *val);

// ------------------------------------------------------------------
// setters for variables of type "uint16_t"
// ------------------------------------------------------------------

// setters for variables on "RayGen"s
VKRT_API void vkrtRayGenSet1us(VKRTRayGen raygen, const char *name, uint16_t val);
VKRT_API void vkrtRayGenSet2us(VKRTRayGen raygen, const char *name, uint16_t x, uint16_t y);
VKRT_API void vkrtRayGenSet3us(VKRTRayGen raygen, const char *name, uint16_t x, uint16_t y, uint16_t z);
VKRT_API void vkrtRayGenSet4us(VKRTRayGen raygen, const char *name, uint16_t x, uint16_t y, uint16_t z, uint16_t w);
VKRT_API void vkrtRayGenSet2usv(VKRTRayGen raygen, const char *name, const uint16_t *val);
VKRT_API void vkrtRayGenSet3usv(VKRTRayGen raygen, const char *name, const uint16_t *val);
VKRT_API void vkrtRayGenSet4usv(VKRTRayGen raygen, const char *name, const uint16_t *val);

// setters for variables on "MissProg"s
VKRT_API void vkrtMissProgSet1us(VKRTMissProg missprog, const char *name, uint16_t val);
VKRT_API void vkrtMissProgSet2us(VKRTMissProg missprog, const char *name, uint16_t x, uint16_t y);
VKRT_API void vkrtMissProgSet3us(VKRTMissProg missprog, const char *name, uint16_t x, uint16_t y, uint16_t z);
VKRT_API void vkrtMissProgSet4us(VKRTMissProg missprog, const char *name, uint16_t x, uint16_t y, uint16_t z, uint16_t w);
VKRT_API void vkrtMissProgSet2usv(VKRTMissProg missprog, const char *name, const uint16_t *val);
VKRT_API void vkrtMissProgSet3usv(VKRTMissProg missprog, const char *name, const uint16_t *val);
VKRT_API void vkrtMissProgSet4usv(VKRTMissProg missprog, const char *name, const uint16_t *val);

// setters for variables on "Geom"s
VKRT_API void vkrtGeomSet1us(VKRTGeom geom, const char *name, uint16_t val);
VKRT_API void vkrtGeomSet2us(VKRTGeom geom, const char *name, uint16_t x, uint16_t y);
VKRT_API void vkrtGeomSet3us(VKRTGeom geom, const char *name, uint16_t x, uint16_t y, uint16_t z);
VKRT_API void vkrtGeomSet4us(VKRTGeom geom, const char *name, uint16_t x, uint16_t y, uint16_t z, uint16_t w);
VKRT_API void vkrtGeomSet2usv(VKRTGeom geom, const char *name, const uint16_t *val);
VKRT_API void vkrtGeomSet3usv(VKRTGeom geom, const char *name, const uint16_t *val);
VKRT_API void vkrtGeomSet4usv(VKRTGeom geom, const char *name, const uint16_t *val);

// setters for variables on "Params"s
// VKRT_API void vkrtParamsSet1us(OWLParams obj, const char *name, uint16_t val);
// VKRT_API void vkrtParamsSet2us(OWLParams obj, const char *name, uint16_t x, uint16_t y);
// VKRT_API void vkrtParamsSet3us(OWLParams obj, const char *name, uint16_t x, uint16_t y, uint16_t z);
// VKRT_API void vkrtParamsSet4us(OWLParams obj, const char *name, uint16_t x, uint16_t y, uint16_t z, uint16_t w);
// VKRT_API void vkrtParamsSet2usv(OWLParams obj, const char *name, const uint16_t *val);
// VKRT_API void vkrtParamsSet3usv(OWLParams obj, const char *name, const uint16_t *val);
// VKRT_API void vkrtParamsSet4usv(OWLParams obj, const char *name, const uint16_t *val);

// ------------------------------------------------------------------
// setters for variables of type "int"
// ------------------------------------------------------------------

// setters for variables on "RayGen"s
VKRT_API void vkrtRayGenSet1i(VKRTRayGen raygen, const char *name, int32_t val);
VKRT_API void vkrtRayGenSet2i(VKRTRayGen raygen, const char *name, int32_t x, int32_t y);
VKRT_API void vkrtRayGenSet3i(VKRTRayGen raygen, const char *name, int32_t x, int32_t y, int32_t z);
VKRT_API void vkrtRayGenSet4i(VKRTRayGen raygen, const char *name, int32_t x, int32_t y, int32_t z, int32_t w);
VKRT_API void vkrtRayGenSet2iv(VKRTRayGen raygen, const char *name, const int32_t *val);
VKRT_API void vkrtRayGenSet3iv(VKRTRayGen raygen, const char *name, const int32_t *val);
VKRT_API void vkrtRayGenSet4iv(VKRTRayGen raygen, const char *name, const int32_t *val);

// setters for variables on "MissProg"s
VKRT_API void vkrtMissProgSet1i(VKRTMissProg missprog, const char *name, int32_t val);
VKRT_API void vkrtMissProgSet2i(VKRTMissProg missprog, const char *name, int32_t x, int32_t y);
VKRT_API void vkrtMissProgSet3i(VKRTMissProg missprog, const char *name, int32_t x, int32_t y, int32_t z);
VKRT_API void vkrtMissProgSet4i(VKRTMissProg missprog, const char *name, int32_t x, int32_t y, int32_t z, int32_t w);
VKRT_API void vkrtMissProgSet2iv(VKRTMissProg missprog, const char *name, const int32_t *val);
VKRT_API void vkrtMissProgSet3iv(VKRTMissProg missprog, const char *name, const int32_t *val);
VKRT_API void vkrtMissProgSet4iv(VKRTMissProg missprog, const char *name, const int32_t *val);

// setters for variables on "Geom"s
VKRT_API void vkrtGeomSet1i(VKRTGeom geom, const char *name, int32_t val);
VKRT_API void vkrtGeomSet2i(VKRTGeom geom, const char *name, int32_t x, int32_t y);
VKRT_API void vkrtGeomSet3i(VKRTGeom geom, const char *name, int32_t x, int32_t y, int32_t z);
VKRT_API void vkrtGeomSet4i(VKRTGeom geom, const char *name, int32_t x, int32_t y, int32_t z, int32_t w);
VKRT_API void vkrtGeomSet2iv(VKRTGeom geom, const char *name, const int32_t *val);
VKRT_API void vkrtGeomSet3iv(VKRTGeom geom, const char *name, const int32_t *val);
VKRT_API void vkrtGeomSet4iv(VKRTGeom geom, const char *name, const int32_t *val);

// setters for variables on "Params"s
// VKRT_API void vkrtParamsSet1i(OWLParams obj, const char *name, int32_t val);
// VKRT_API void vkrtParamsSet2i(OWLParams obj, const char *name, int32_t x, int32_t y);
// VKRT_API void vkrtParamsSet3i(OWLParams obj, const char *name, int32_t x, int32_t y, int32_t z);
// VKRT_API void vkrtParamsSet4i(OWLParams obj, const char *name, int32_t x, int32_t y, int32_t z, int32_t w);
// VKRT_API void vkrtParamsSet2iv(OWLParams obj, const char *name, const int32_t *val);
// VKRT_API void vkrtParamsSet3iv(OWLParams obj, const char *name, const int32_t *val);
// VKRT_API void vkrtParamsSet4iv(OWLParams obj, const char *name, const int32_t *val);

// ------------------------------------------------------------------
// setters for variables of type "uint32_t"
// ------------------------------------------------------------------

// setters for variables on "RayGen"s
VKRT_API void vkrtRayGenSet1ui(VKRTRayGen raygen, const char *name, uint32_t val);
VKRT_API void vkrtRayGenSet2ui(VKRTRayGen raygen, const char *name, uint32_t x, uint32_t y);
VKRT_API void vkrtRayGenSet3ui(VKRTRayGen raygen, const char *name, uint32_t x, uint32_t y, uint32_t z);
VKRT_API void vkrtRayGenSet4ui(VKRTRayGen raygen, const char *name, uint32_t x, uint32_t y, uint32_t z, uint32_t w);
VKRT_API void vkrtRayGenSet2uiv(VKRTRayGen raygen, const char *name, const uint32_t *val);
VKRT_API void vkrtRayGenSet3uiv(VKRTRayGen raygen, const char *name, const uint32_t *val);
VKRT_API void vkrtRayGenSet4uiv(VKRTRayGen raygen, const char *name, const uint32_t *val);

// setters for variables on "MissProg"s
VKRT_API void vkrtMissProgSet1ui(VKRTMissProg missprog, const char *name, uint32_t val);
VKRT_API void vkrtMissProgSet2ui(VKRTMissProg missprog, const char *name, uint32_t x, uint32_t y);
VKRT_API void vkrtMissProgSet3ui(VKRTMissProg missprog, const char *name, uint32_t x, uint32_t y, uint32_t z);
VKRT_API void vkrtMissProgSet4ui(VKRTMissProg missprog, const char *name, uint32_t x, uint32_t y, uint32_t z, uint32_t w);
VKRT_API void vkrtMissProgSet2uiv(VKRTMissProg missprog, const char *name, const uint32_t *val);
VKRT_API void vkrtMissProgSet3uiv(VKRTMissProg missprog, const char *name, const uint32_t *val);
VKRT_API void vkrtMissProgSet4uiv(VKRTMissProg missprog, const char *name, const uint32_t *val);

// setters for variables on "Geom"s
VKRT_API void vkrtGeomSet1ui(VKRTGeom geom, const char *name, uint32_t val);
VKRT_API void vkrtGeomSet2ui(VKRTGeom geom, const char *name, uint32_t x, uint32_t y);
VKRT_API void vkrtGeomSet3ui(VKRTGeom geom, const char *name, uint32_t x, uint32_t y, uint32_t z);
VKRT_API void vkrtGeomSet4ui(VKRTGeom geom, const char *name, uint32_t x, uint32_t y, uint32_t z, uint32_t w);
VKRT_API void vkrtGeomSet2uiv(VKRTGeom geom, const char *name, const uint32_t *val);
VKRT_API void vkrtGeomSet3uiv(VKRTGeom geom, const char *name, const uint32_t *val);
VKRT_API void vkrtGeomSet4uiv(VKRTGeom geom, const char *name, const uint32_t *val);

// setters for variables on "Params"s
// VKRT_API void vkrtParamsSet1ui(OWLParams obj, const char *name, uint32_t val);
// VKRT_API void vkrtParamsSet2ui(OWLParams obj, const char *name, uint32_t x, uint32_t y);
// VKRT_API void vkrtParamsSet3ui(OWLParams obj, const char *name, uint32_t x, uint32_t y, uint32_t z);
// VKRT_API void vkrtParamsSet4ui(OWLParams obj, const char *name, uint32_t x, uint32_t y, uint32_t z, uint32_t w);
// VKRT_API void vkrtParamsSet2uiv(OWLParams obj, const char *name, const uint32_t *val);
// VKRT_API void vkrtParamsSet3uiv(OWLParams obj, const char *name, const uint32_t *val);
// VKRT_API void vkrtParamsSet4uiv(OWLParams obj, const char *name, const uint32_t *val);

// ------------------------------------------------------------------
// setters for variables of type "float"
// ------------------------------------------------------------------

// setters for variables on "RayGen"s
VKRT_API void vkrtRayGenSet1f(VKRTRayGen raygen, const char *name, float val);
VKRT_API void vkrtRayGenSet2f(VKRTRayGen raygen, const char *name, float x, float y);
VKRT_API void vkrtRayGenSet3f(VKRTRayGen raygen, const char *name, float x, float y, float z);
VKRT_API void vkrtRayGenSet4f(VKRTRayGen raygen, const char *name, float x, float y, float z, float w);
VKRT_API void vkrtRayGenSet2fv(VKRTRayGen raygen, const char *name, const float *val);
VKRT_API void vkrtRayGenSet3fv(VKRTRayGen raygen, const char *name, const float *val);
VKRT_API void vkrtRayGenSet4fv(VKRTRayGen raygen, const char *name, const float *val);

// setters for variables on "MissProg"s
VKRT_API void vkrtMissProgSet1f(VKRTMissProg missprog, const char *name, float val);
VKRT_API void vkrtMissProgSet2f(VKRTMissProg missprog, const char *name, float x, float y);
VKRT_API void vkrtMissProgSet3f(VKRTMissProg missprog, const char *name, float x, float y, float z);
VKRT_API void vkrtMissProgSet4f(VKRTMissProg missprog, const char *name, float x, float y, float z, float w);
VKRT_API void vkrtMissProgSet2fv(VKRTMissProg missprog, const char *name, const float *val);
VKRT_API void vkrtMissProgSet3fv(VKRTMissProg missprog, const char *name, const float *val);
VKRT_API void vkrtMissProgSet4fv(VKRTMissProg missprog, const char *name, const float *val);

// setters for variables on "Geom"s
VKRT_API void vkrtGeomSet1f(VKRTGeom geom, const char *name, float val);
VKRT_API void vkrtGeomSet2f(VKRTGeom geom, const char *name, float x, float y);
VKRT_API void vkrtGeomSet3f(VKRTGeom geom, const char *name, float x, float y, float z);
VKRT_API void vkrtGeomSet4f(VKRTGeom geom, const char *name, float x, float y, float z, float w);
VKRT_API void vkrtGeomSet2fv(VKRTGeom geom, const char *name, const float *val);
VKRT_API void vkrtGeomSet3fv(VKRTGeom geom, const char *name, const float *val);
VKRT_API void vkrtGeomSet4fv(VKRTGeom geom, const char *name, const float *val);

// setters for variables on "Params"s
// VKRT_API void vkrtParamsSet1f(OWLParams obj, const char *name, float val);
// VKRT_API void vkrtParamsSet2f(OWLParams obj, const char *name, float x, float y);
// VKRT_API void vkrtParamsSet3f(OWLParams obj, const char *name, float x, float y, float z);
// VKRT_API void vkrtParamsSet4f(OWLParams obj, const char *name, float x, float y, float z, float w);
// VKRT_API void vkrtParamsSet2fv(OWLParams obj, const char *name, const float *val);
// VKRT_API void vkrtParamsSet3fv(OWLParams obj, const char *name, const float *val);
// VKRT_API void vkrtParamsSet4fv(OWLParams obj, const char *name, const float *val);

// ------------------------------------------------------------------
// setters for variables of type "double"
// ------------------------------------------------------------------

// setters for variables on "RayGen"s
VKRT_API void vkrtRayGenSet1d(VKRTRayGen raygen, const char *name, double val);
VKRT_API void vkrtRayGenSet2d(VKRTRayGen raygen, const char *name, double x, double y);
VKRT_API void vkrtRayGenSet3d(VKRTRayGen raygen, const char *name, double x, double y, double z);
VKRT_API void vkrtRayGenSet4d(VKRTRayGen raygen, const char *name, double x, double y, double z, double w);
VKRT_API void vkrtRayGenSet2dv(VKRTRayGen raygen, const char *name, const double *val);
VKRT_API void vkrtRayGenSet3dv(VKRTRayGen raygen, const char *name, const double *val);
VKRT_API void vkrtRayGenSet4dv(VKRTRayGen raygen, const char *name, const double *val);

// setters for variables on "MissProg"s
VKRT_API void vkrtMissProgSet1d(VKRTMissProg missprog, const char *name, double val);
VKRT_API void vkrtMissProgSet2d(VKRTMissProg missprog, const char *name, double x, double y);
VKRT_API void vkrtMissProgSet3d(VKRTMissProg missprog, const char *name, double x, double y, double z);
VKRT_API void vkrtMissProgSet4d(VKRTMissProg missprog, const char *name, double x, double y, double z, double w);
VKRT_API void vkrtMissProgSet2dv(VKRTMissProg missprog, const char *name, const double *val);
VKRT_API void vkrtMissProgSet3dv(VKRTMissProg missprog, const char *name, const double *val);
VKRT_API void vkrtMissProgSet4dv(VKRTMissProg missprog, const char *name, const double *val);

// setters for variables on "Geom"s
VKRT_API void vkrtGeomSet1d(VKRTGeom geom, const char *name, double val);
VKRT_API void vkrtGeomSet2d(VKRTGeom geom, const char *name, double x, double y);
VKRT_API void vkrtGeomSet3d(VKRTGeom geom, const char *name, double x, double y, double z);
VKRT_API void vkrtGeomSet4d(VKRTGeom geom, const char *name, double x, double y, double z, double w);
VKRT_API void vkrtGeomSet2dv(VKRTGeom geom, const char *name, const double *val);
VKRT_API void vkrtGeomSet3dv(VKRTGeom geom, const char *name, const double *val);
VKRT_API void vkrtGeomSet4dv(VKRTGeom geom, const char *name, const double *val);

// setters for variables on "Params"s
// VKRT_API void vkrtParamsSet1d(OWLParams obj, const char *name, double val);
// VKRT_API void vkrtParamsSet2d(OWLParams obj, const char *name, double x, double y);
// VKRT_API void vkrtParamsSet3d(OWLParams obj, const char *name, double x, double y, double z);
// VKRT_API void vkrtParamsSet4d(OWLParams obj, const char *name, double x, double y, double z, double w);
// VKRT_API void vkrtParamsSet2dv(OWLParams obj, const char *name, const double *val);
// VKRT_API void vkrtParamsSet3dv(OWLParams obj, const char *name, const double *val);
// VKRT_API void vkrtParamsSet4dv(OWLParams obj, const char *name, const double *val);

// ------------------------------------------------------------------
// setters for variables of type "int64_t"
// ------------------------------------------------------------------

// setters for variables on "RayGen"s
VKRT_API void vkrtRayGenSet1l(VKRTRayGen raygen, const char *name, int64_t val);
VKRT_API void vkrtRayGenSet2l(VKRTRayGen raygen, const char *name, int64_t x, int64_t y);
VKRT_API void vkrtRayGenSet3l(VKRTRayGen raygen, const char *name, int64_t x, int64_t y, int64_t z);
VKRT_API void vkrtRayGenSet4l(VKRTRayGen raygen, const char *name, int64_t x, int64_t y, int64_t z, int64_t w);
VKRT_API void vkrtRayGenSet2lv(VKRTRayGen raygen, const char *name, const int64_t *val);
VKRT_API void vkrtRayGenSet3lv(VKRTRayGen raygen, const char *name, const int64_t *val);
VKRT_API void vkrtRayGenSet4lv(VKRTRayGen raygen, const char *name, const int64_t *val);

// setters for variables on "MissProg"s
VKRT_API void vkrtMissProgSet1l(VKRTMissProg missprog, const char *name, int64_t val);
VKRT_API void vkrtMissProgSet2l(VKRTMissProg missprog, const char *name, int64_t x, int64_t y);
VKRT_API void vkrtMissProgSet3l(VKRTMissProg missprog, const char *name, int64_t x, int64_t y, int64_t z);
VKRT_API void vkrtMissProgSet4l(VKRTMissProg missprog, const char *name, int64_t x, int64_t y, int64_t z, int64_t w);
VKRT_API void vkrtMissProgSet2lv(VKRTMissProg missprog, const char *name, const int64_t *val);
VKRT_API void vkrtMissProgSet3lv(VKRTMissProg missprog, const char *name, const int64_t *val);
VKRT_API void vkrtMissProgSet4lv(VKRTMissProg missprog, const char *name, const int64_t *val);

// setters for variables on "Geom"s
VKRT_API void vkrtGeomSet1l(VKRTGeom geom, const char *name, int64_t val);
VKRT_API void vkrtGeomSet2l(VKRTGeom geom, const char *name, int64_t x, int64_t y);
VKRT_API void vkrtGeomSet3l(VKRTGeom geom, const char *name, int64_t x, int64_t y, int64_t z);
VKRT_API void vkrtGeomSet4l(VKRTGeom geom, const char *name, int64_t x, int64_t y, int64_t z, int64_t w);
VKRT_API void vkrtGeomSet2lv(VKRTGeom geom, const char *name, const int64_t *val);
VKRT_API void vkrtGeomSet3lv(VKRTGeom geom, const char *name, const int64_t *val);
VKRT_API void vkrtGeomSet4lv(VKRTGeom geom, const char *name, const int64_t *val);

// setters for variables on "Params"s
// VKRT_API void vkrtParamsSet1l(OWLParams obj, const char *name, int64_t val);
// VKRT_API void vkrtParamsSet2l(OWLParams obj, const char *name, int64_t x, int64_t y);
// VKRT_API void vkrtParamsSet3l(OWLParams obj, const char *name, int64_t x, int64_t y, int64_t z);
// VKRT_API void vkrtParamsSet4l(OWLParams obj, const char *name, int64_t x, int64_t y, int64_t z, int64_t w);
// VKRT_API void vkrtParamsSet2lv(OWLParams obj, const char *name, const int64_t *val);
// VKRT_API void vkrtParamsSet3lv(OWLParams obj, const char *name, const int64_t *val);
// VKRT_API void vkrtParamsSet4lv(OWLParams obj, const char *name, const int64_t *val);

// ------------------------------------------------------------------
// setters for variables of type "uint64_t"
// ------------------------------------------------------------------

// setters for variables on "RayGen"s
VKRT_API void vkrtRayGenSet1ul(VKRTRayGen raygen, const char *name, uint64_t val);
VKRT_API void vkrtRayGenSet2ul(VKRTRayGen raygen, const char *name, uint64_t x, uint64_t y);
VKRT_API void vkrtRayGenSet3ul(VKRTRayGen raygen, const char *name, uint64_t x, uint64_t y, uint64_t z);
VKRT_API void vkrtRayGenSet4ul(VKRTRayGen raygen, const char *name, uint64_t x, uint64_t y, uint64_t z, uint64_t w);
VKRT_API void vkrtRayGenSet2ulv(VKRTRayGen raygen, const char *name, const uint64_t *val);
VKRT_API void vkrtRayGenSet3ulv(VKRTRayGen raygen, const char *name, const uint64_t *val);
VKRT_API void vkrtRayGenSet4ulv(VKRTRayGen raygen, const char *name, const uint64_t *val);

// setters for variables on "MissProg"s
VKRT_API void vkrtMissProgSet1ul(VKRTMissProg missprog, const char *name, uint64_t val);
VKRT_API void vkrtMissProgSet2ul(VKRTMissProg missprog, const char *name, uint64_t x, uint64_t y);
VKRT_API void vkrtMissProgSet3ul(VKRTMissProg missprog, const char *name, uint64_t x, uint64_t y, uint64_t z);
VKRT_API void vkrtMissProgSet4ul(VKRTMissProg missprog, const char *name, uint64_t x, uint64_t y, uint64_t z, uint64_t w);
VKRT_API void vkrtMissProgSet2ulv(VKRTMissProg missprog, const char *name, const uint64_t *val);
VKRT_API void vkrtMissProgSet3ulv(VKRTMissProg missprog, const char *name, const uint64_t *val);
VKRT_API void vkrtMissProgSet4ulv(VKRTMissProg missprog, const char *name, const uint64_t *val);

// setters for variables on "Geom"s
VKRT_API void vkrtGeomSet1ul(VKRTGeom geom, const char *name, uint64_t val);
VKRT_API void vkrtGeomSet2ul(VKRTGeom geom, const char *name, uint64_t x, uint64_t y);
VKRT_API void vkrtGeomSet3ul(VKRTGeom geom, const char *name, uint64_t x, uint64_t y, uint64_t z);
VKRT_API void vkrtGeomSet4ul(VKRTGeom geom, const char *name, uint64_t x, uint64_t y, uint64_t z, uint64_t w);
VKRT_API void vkrtGeomSet2ulv(VKRTGeom geom, const char *name, const uint64_t *val);
VKRT_API void vkrtGeomSet3ulv(VKRTGeom geom, const char *name, const uint64_t *val);
VKRT_API void vkrtGeomSet4ulv(VKRTGeom geom, const char *name, const uint64_t *val);

// setters for variables on "Params"s
// VKRT_API void vkrtParamsSet1ul(OWLParams obj, const char *name, uint64_t val);
// VKRT_API void vkrtParamsSet2ul(OWLParams obj, const char *name, uint64_t x, uint64_t y);
// VKRT_API void vkrtParamsSet3ul(OWLParams obj, const char *name, uint64_t x, uint64_t y, uint64_t z);
// VKRT_API void vkrtParamsSet4ul(OWLParams obj, const char *name, uint64_t x, uint64_t y, uint64_t z, uint64_t w);
// VKRT_API void vkrtParamsSet2ulv(OWLParams obj, const char *name, const uint64_t *val);
// VKRT_API void vkrtParamsSet3ulv(OWLParams obj, const char *name, const uint64_t *val);
// VKRT_API void vkrtParamsSet4ulv(OWLParams obj, const char *name, const uint64_t *val);

// ------------------------------------------------------------------
// setters for "meta" types
// ------------------------------------------------------------------

// setters for variables on "RayGen"s
// VKRT_API void vkrtRayGenSetTexture(VKRTRayGen raygen, const char *name, VKRTTexture val);
// VKRT_API void vkrtRayGenSetPointer(VKRTRayGen raygen, const char *name, const void *val);
VKRT_API void vkrtRayGenSetBuffer(VKRTRayGen raygen, const char *name, VKRTBuffer val);
VKRT_API void vkrtRayGenSetAccel(VKRTRayGen raygen, const char *name, VKRTAccel val);
VKRT_API void vkrtRayGenSetRaw(VKRTRayGen raygen, const char *name, const void *val);

// // setters for variables on "Geom"s
// VKRT_API void vkrtGeomSetTexture(VKRTGeom obj, const char *name, VKRTTexture val);
// VKRT_API void vkrtGeomSetPointer(VKRTGeom obj, const char *name, const void *val);
VKRT_API void vkrtGeomSetBuffer(VKRTGeom obj, const char *name, VKRTBuffer val);
VKRT_API void vkrtGeomSetAccel(VKRTGeom obj, const char *name, VKRTAccel val);
VKRT_API void vkrtGeomSetRaw(VKRTGeom obj, const char *name, const void *val);

// // setters for variables on "Params"s
// VKRT_API void vkrtParamsSetTexture(VKRTParams obj, const char *name, VKRTTexture val);
// VKRT_API void vkrtParamsSetPointer(VKRTParams obj, const char *name, const void *val);
// VKRT_API void vkrtParamsSetBuffer(VKRTParams obj, const char *name, VKRTBuffer val);
// VKRT_API void vkrtParamsSetAccel(VKRTParams obj, const char *name, VKRTAccel val);
// VKRT_API void vkrtParamsSetRaw(VKRTParams obj, const char *name, const void *val);

// setters for variables on "MissProg"s
// VKRT_API void vkrtMissProgSetTexture(VKRTMissProg missprog, const char *name, VKRTTexture val);
// VKRT_API void vkrtMissProgSetPointer(VKRTMissProg missprog, const char *name, const void *val);
VKRT_API void vkrtMissProgSetBuffer(VKRTMissProg missprog, const char *name, VKRTBuffer val);
VKRT_API void vkrtMissProgSetAccel(VKRTMissProg missprog, const char *name, VKRTAccel val);
VKRT_API void vkrtMissProgSetRaw(VKRTMissProg missprog, const char *name, const void *val);