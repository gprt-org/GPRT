/**
 * @file gprt_host.h
 * @author Nate Morrical (natemorrical@gmail.com)
 * @brief This file defines the host-side interface for the general purpose 
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

#include <stdalign.h>
#include <vulkan/vulkan.h>

#include "linalg.h"
using namespace linalg;
using namespace linalg::detail;
using namespace linalg::aliases;
using namespace linalg::ostream_overloads;

#include <sys/types.h>
#include <stdint.h>

#include <assert.h>
#include <unordered_map>
#include <string>
#include <vector>
#include <map>

#ifdef __cplusplus
# include <cstddef>
#endif

#if defined(_MSC_VER)
#  define GPRT_DLL_EXPORT __declspec(dllexport)
#  define GPRT_DLL_IMPORT __declspec(dllimport)
#elif defined(__clang__) || defined(__GNUC__)
#  define GPRT_DLL_EXPORT __attribute__((visibility("default")))
#  define GPRT_DLL_IMPORT __attribute__((visibility("default")))
#else
#  define GPRT_DLL_EXPORT
#  define GPRT_DLL_IMPORT
#endif

#ifdef __cplusplus
# define GPRT_IF_CPP(a) a
#else
# define GPRT_IF_CPP(a) /* drop it */
#endif

#  ifdef __cplusplus
#    define GPRT_API extern "C" GPRT_DLL_EXPORT
#  else
#    define GPRT_API /* bla */
#  endif

#define GPRT_OFFSETOF(type,member)                       \
   (uint32_t)((char *)(&((struct type *)0)-> member )   \
   -                                                    \
   (char *)(((struct type *)0)))

// Terminal colors
#define GPRT_TERMINAL_RED "\033[0;31m"
#define GPRT_TERMINAL_GREEN "\033[0;32m"
#define GPRT_TERMINAL_LIGHT_GREEN "\033[1;32m"
#define GPRT_TERMINAL_YELLOW "\033[1;33m"
#define GPRT_TERMINAL_BLUE "\033[0;34m"
#define GPRT_TERMINAL_LIGHT_BLUE "\033[1;34m"
#define GPRT_TERMINAL_RESET "\033[0m"
#define GPRT_TERMINAL_DEFAULT GPRT_TERMINAL_RESET
#define GPRT_TERMINAL_BOLD "\033[1;1m"
#define GPRT_TERMINAL_MAGENTA "\e[35m"
#define GPRT_TERMINAL_LIGHT_MAGENTA "\e[95m"
#define GPRT_TERMINAL_CYAN "\e[36m"
#define GPRT_TERMINAL_LIGHT_RED "\033[1;31m"

typedef struct _GPRTContext    *GPRTContext;
typedef struct _GPRTBuffer     *GPRTBuffer;
typedef struct _GPRTTexture    *GPRTTexture;
typedef struct _GPRTGeom       *GPRTGeom;
typedef struct _GPRTGeomType   *GPRTGeomType;
typedef struct _GPRTVariable   *GPRTVariable;
typedef struct _GPRTModule     *GPRTModule;
typedef struct _GPRTAccel      *GPRTAccel;
typedef struct _GPRTRayGen     *GPRTRayGen;
typedef struct _GPRTMiss       *GPRTMiss;
typedef struct _GPRTCompute    *GPRTCompute;

typedef std::map<std::string, std::vector<uint8_t>> GPRTProgram;

namespace gprt {
  struct Buffer {
    uint64_t x;
    uint64_t y;
  };

  struct Accel {
    uint64_t x;
    uint64_t y;
  };
}

/*! launch params (or "globals") are variables that can be put into
  device constant memory, accessible through Vulkan's push constants */
typedef struct _GPRTLaunchParams  *GPRTLaunchParams, *GPRTParams, *GPRTGlobals;

typedef enum
{
  GPRT_SBT_HITGROUP = 0x1,
  GPRT_SBT_GEOM     = GPRT_SBT_HITGROUP,
  GPRT_SBT_RAYGEN   = 0x2,
  GPRT_SBT_MISSPROG = 0x4,
  GPRT_SBT_COMPUTE   = 0x8,
  GPRT_SBT_ALL   = 0x15
} GPRTBuildSBTFlags;

/*! enum that specifies the different possible memory layouts for
  passing transformation matrices */
typedef enum
  {
   /*! A 4x3-float column-major matrix format, where a matrix is
     specified through four float3's, the first three being the basis
     vectors of the linear transform, and the fourth one the
     translation part. */
   GPRT_MATRIX_FORMAT_COLUMN_MAJOR=0,

   /*! 3x4-float *row-major* layout as preferred by vulkan matching VkTransformMatrixKHR;
     not that, in this case, it doesn't matter if it's a 4x3 or 4x4 matrix,
     as the last row in a 4x4 row major matrix can simply be ignored */
   GPRT_MATRIX_FORMAT_ROW_MAJOR
  } GPRTMatrixFormat;

typedef enum
  {
   GPRT_INVALID_TYPE = 0,

   /* a 128-bit buffer type containing a pointer and a size */
   GPRT_BUFFER=10,

   /*! a 64-bit int representing the number of elements in a buffer */
   GPRT_BUFFER_SIZE,
   GPRT_BUFFER_ID,
   GPRT_BUFFER_POINTER,
   GPRT_BUFPTR=GPRT_BUFFER_POINTER,

  // 128 bits, 64 for address, 64 for kind
   GPRT_ACCEL=20,

   // just the raw 64-bit address
   GPRT_ACCEL_POINTER,

   /*! implicit variable of type integer that specifies the *index*
     of the given device. this variable type is implicit in the
     sense that it only gets _declared_ on the host, and gets set
     automatically during SBT creation */
   GPRT_DEVICE=30,

   /*! texture(s) */
   GPRT_TEXTURE=40,
   GPRT_TEXTURE_2D=GPRT_TEXTURE,


   /* all types that are naively copyable should be below this value,
      all that aren't should be above */
   _GPRT_BEGIN_COPYABLE_TYPES = 1000,


   GPRT_FLOAT=1000,
   GPRT_FLOAT2,
   GPRT_FLOAT3,
   GPRT_FLOAT4,

   GPRT_INT=1010,
   GPRT_INT2,
   GPRT_INT3,
   GPRT_INT4,

   GPRT_INT32_T   = GPRT_INT,
   GPRT_INT32_T2 = GPRT_INT2,
   GPRT_INT32_T3 = GPRT_INT3,
   GPRT_INT32_T4 = GPRT_INT4,

   GPRT_UINT=1020,
   GPRT_UINT2,
   GPRT_UINT3,
   GPRT_UINT4,

   GPRT_UINT32_T  = GPRT_UINT,
   GPRT_UINT32_T2 = GPRT_UINT2,
   GPRT_UINT32_T3 = GPRT_UINT3,
   GPRT_UINT32_T4 = GPRT_UINT4,

   /* 64 bit integers */
   GPRT_LONG=1030,
   GPRT_LONG2,
   GPRT_LONG3,
   GPRT_LONG4,

   GPRT_INT64_T  = GPRT_LONG,
   GPRT_INT64_T2 = GPRT_LONG2,
   GPRT_INT64_T3 = GPRT_LONG3,
   GPRT_INT64_T4 = GPRT_LONG4,

   GPRT_ULONG=1040,
   GPRT_ULONG2,
   GPRT_ULONG3,
   GPRT_ULONG4,

   GPRT_UINT64_T = GPRT_ULONG,
   GPRT_UINT64_T2 = GPRT_ULONG2,
   GPRT_UINT64_T3 = GPRT_ULONG3,
   GPRT_UINT64_T4 = GPRT_ULONG4,

   GPRT_DOUBLE=1050,
   GPRT_DOUBLE2,
   GPRT_DOUBLE3,
   GPRT_DOUBLE4,

   GPRT_CHAR=1060,
   GPRT_CHAR2,
   GPRT_CHAR3,
   GPRT_CHAR4,

   GPRT_INT8_T   = GPRT_CHAR,
   GPRT_INT8_T2 = GPRT_CHAR2,
   GPRT_INT8_T3 = GPRT_CHAR3,
   GPRT_INT8_T4 = GPRT_CHAR4,

   /*! unsigend 8-bit integer */
   GPRT_UCHAR=1070,
   GPRT_UCHAR2,
   GPRT_UCHAR3,
   GPRT_UCHAR4,

   GPRT_UINT8_T  = GPRT_UCHAR,
   GPRT_UINT8_T2 = GPRT_UCHAR2,
   GPRT_UINT8_T3 = GPRT_UCHAR3,
   GPRT_UINT8_T4 = GPRT_UCHAR4,

   GPRT_SHORT=1080,
   GPRT_SHORT2,
   GPRT_SHORT3,
   GPRT_SHORT4,

   GPRT_INT16_T  = GPRT_SHORT,
   GPRT_INT16_T2 = GPRT_SHORT2,
   GPRT_INT16_T3 = GPRT_SHORT3,
   GPRT_INT16_T4 = GPRT_SHORT4,

   /*! unsigend 8-bit integer */
   GPRT_USHORT=1090,
   GPRT_USHORT2,
   GPRT_USHORT3,
   GPRT_USHORT4,

   GPRT_UINT16_T  = GPRT_USHORT,
   GPRT_UINT16_T2 = GPRT_USHORT2,
   GPRT_UINT16_T3 = GPRT_USHORT3,
   GPRT_UINT16_T4 = GPRT_USHORT4,

   GPRT_BOOL,
   GPRT_BOOL2,
   GPRT_BOOL3,
   GPRT_BOOL4,

   /*! just another name for a 64-bit data type - unlike
     GPRT_BUFFER_POINTER's (which gets translated from GPRTBuffer's
     to actual device-side poiners) these GPRT_RAW_POINTER types get
     copied binary without any translation. This is useful for
     owl-cuda interaction (where the user already has device
     pointers), but should not be used for logical buffers */
   GPRT_RAW_POINTER=GPRT_ULONG,
   GPRT_BYTE = GPRT_UCHAR,
   // GPRT_BOOL = GPRT_UCHAR,
   // GPRT_BOOL2 = GPRT_UCHAR2,
   // GPRT_BOOL3 = GPRT_UCHAR3,
   // GPRT_BOOL4 = GPRT_UCHAR4,


   /* matrix formats */
   GPRT_AFFINE3F=1300,
   
   /* A type matching VkTransformMatrixKHR, row major 3x4 */
   GPRT_TRANSFORM=1400,
   GPRT_TRANSFORM_3X4=GPRT_TRANSFORM,
   GPRT_TRANSFORM_4X4=1401,

   /*! at least for now, use that for buffers with user-defined types:
     type then is "GPRT_USER_TYPE_BEGIN+sizeof(elementtype). Note
     that since we always _add_ the user type's size to this value
     this MUST be the last entry in the enum */
   GPRT_USER_TYPE_BEGIN=10000
  }
  GPRTDataType;

typedef enum
  {
   GPRT_UNKNOWN,
   GPRT_AABBS,
   GPRT_TRIANGLES,
  //  GPRT_CURVES
  }
  GPRTGeomKind;

  inline size_t getSize(GPRTDataType type)
  {
         if (type == GPRT_INT8_T)  return sizeof(int8_t);
    else if (type == GPRT_INT8_T2) return sizeof(int8_t) * 2;
    else if (type == GPRT_INT8_T3) return sizeof(int8_t) * 3;
    else if (type == GPRT_INT8_T4) return sizeof(int8_t) * 4;

    else if (type == GPRT_UINT8_T)  return sizeof(uint8_t);
    else if (type == GPRT_UINT8_T2) return sizeof(uint8_t) * 2;
    else if (type == GPRT_UINT8_T3) return sizeof(uint8_t) * 3;
    else if (type == GPRT_UINT8_T4) return sizeof(uint8_t) * 4;

    else if (type == GPRT_INT16_T)  return sizeof(int16_t);
    else if (type == GPRT_INT16_T2) return sizeof(int16_t) * 2;
    else if (type == GPRT_INT16_T3) return sizeof(int16_t) * 3;
    else if (type == GPRT_INT16_T4) return sizeof(int16_t) * 4;

    else if (type == GPRT_UINT16_T)  return sizeof(uint16_t);
    else if (type == GPRT_UINT16_T2) return sizeof(uint16_t) * 2;
    else if (type == GPRT_UINT16_T3) return sizeof(uint16_t) * 3;
    else if (type == GPRT_UINT16_T4) return sizeof(uint16_t) * 4;

    else if (type == GPRT_INT32_T)  return sizeof(int32_t);
    else if (type == GPRT_INT32_T2) return sizeof(int32_t) * 2;
    else if (type == GPRT_INT32_T3) return sizeof(int32_t) * 3;
    else if (type == GPRT_INT32_T4) return sizeof(int32_t) * 4;

    else if (type == GPRT_UINT32_T)  return sizeof(uint32_t);
    else if (type == GPRT_UINT32_T2) return sizeof(uint32_t) * 2;
    else if (type == GPRT_UINT32_T3) return sizeof(uint32_t) * 3;
    else if (type == GPRT_UINT32_T4) return sizeof(uint32_t) * 4;

    else if (type == GPRT_INT64_T)  return sizeof(int64_t);
    else if (type == GPRT_INT64_T2) return sizeof(int64_t) * 2;
    else if (type == GPRT_INT64_T3) return sizeof(int64_t) * 3;
    else if (type == GPRT_INT64_T4) return sizeof(int64_t) * 4;

    else if (type == GPRT_UINT64_T)  return sizeof(uint64_t);
    else if (type == GPRT_UINT64_T2) return sizeof(uint64_t) * 2;
    else if (type == GPRT_UINT64_T3) return sizeof(uint64_t) * 3;
    else if (type == GPRT_UINT64_T4) return sizeof(uint64_t) * 4;

    else if (type == GPRT_BUFFER) return 2 * sizeof(uint64_t);
    else if (type == GPRT_BUFPTR) return sizeof(uint64_t);

    else if (type == GPRT_FLOAT) return sizeof(float);
    else if (type == GPRT_FLOAT2) return sizeof(float) * 2;
    else if (type == GPRT_FLOAT3) return sizeof(float) * 3;
    else if (type == GPRT_FLOAT4) return sizeof(float) * 4;

    else if (type == GPRT_DOUBLE) return sizeof(double);
    else if (type == GPRT_DOUBLE2) return sizeof(double) * 2;
    else if (type == GPRT_DOUBLE3) return sizeof(double) * 3;
    else if (type == GPRT_DOUBLE4) return sizeof(double) * 4;

    else if (type == GPRT_BOOL) return sizeof(bool);
    else if (type == GPRT_BOOL2) return sizeof(bool) * 2;
    else if (type == GPRT_BOOL3) return sizeof(bool) * 3;
    else if (type == GPRT_BOOL4) return sizeof(bool) * 4;

    else if (type == GPRT_ACCEL) return 2 * sizeof(uint64_t);
    else if (type == GPRT_TRANSFORM) return sizeof(float) * 3 * 4;
    else if (type == GPRT_TRANSFORM_3X4) return sizeof(float) * 3 * 4;
    else if (type == GPRT_TRANSFORM_4X4) return sizeof(float) * 4 * 4;

    // User Types have size encoded in their type enum
    else if (type > GPRT_USER_TYPE_BEGIN) return type - GPRT_USER_TYPE_BEGIN;
    else assert(false); return -1;// std::runtime_error("Unimplemented!");
  }

  inline std::string getTypeString(GPRTDataType type)
  {
         if (type == GPRT_INT8_T)  return "int8_t";
    else if (type == GPRT_INT8_T2) return "int8_t2";
    else if (type == GPRT_INT8_T3) return "int8_t3";
    else if (type == GPRT_INT8_T4) return "int8_t4";

    else if (type == GPRT_UINT8_T)  return "uint8_t";
    else if (type == GPRT_UINT8_T2) return "uint8_t2";
    else if (type == GPRT_UINT8_T3) return "uint8_t3";
    else if (type == GPRT_UINT8_T4) return "uint8_t4";

    else if (type == GPRT_INT16_T)  return "int16_t";
    else if (type == GPRT_INT16_T2) return "int16_t2";
    else if (type == GPRT_INT16_T3) return "int16_t3";
    else if (type == GPRT_INT16_T4) return "int16_t4";

    else if (type == GPRT_UINT16_T)  return "int16_t";
    else if (type == GPRT_UINT16_T2) return "int16_t2";
    else if (type == GPRT_UINT16_T3) return "int16_t3";
    else if (type == GPRT_UINT16_T4) return "int16_t4";

    else if (type == GPRT_INT32_T)  return "int32_t";
    else if (type == GPRT_INT32_T2) return "int32_t2";
    else if (type == GPRT_INT32_T3) return "int32_t3";
    else if (type == GPRT_INT32_T4) return "int32_t4";

    else if (type == GPRT_UINT32_T)  return "uint32_t";
    else if (type == GPRT_UINT32_T2) return "uint32_t2";
    else if (type == GPRT_UINT32_T3) return "uint32_t3";
    else if (type == GPRT_UINT32_T4) return "uint32_t4";

    else if (type == GPRT_INT64_T)  return "int64_t";
    else if (type == GPRT_INT64_T2) return "int64_t2";
    else if (type == GPRT_INT64_T3) return "int64_t3";
    else if (type == GPRT_INT64_T4) return "int64_t4";

    else if (type == GPRT_UINT64_T)  return "uint64_t";
    else if (type == GPRT_UINT64_T2) return "uint64_t2";
    else if (type == GPRT_UINT64_T3) return "uint64_t3";
    else if (type == GPRT_UINT64_T4) return "uint64_t4";

    else if (type == GPRT_BUFFER) return "gprt::Buffer";
    else if (type == GPRT_BUFPTR) return "gprt::Buffer*";

    else if (type == GPRT_FLOAT) return "float";
    else if (type == GPRT_FLOAT2) return "float2";
    else if (type == GPRT_FLOAT3) return "float3";
    else if (type == GPRT_FLOAT4) return "float4";

    else if (type == GPRT_DOUBLE) return "double";
    else if (type == GPRT_DOUBLE2) return "double2";
    else if (type == GPRT_DOUBLE3) return "double3";
    else if (type == GPRT_DOUBLE4) return "double4";

    else if (type == GPRT_BOOL) return "bool";
    else if (type == GPRT_BOOL2) return "bool2"; 
    else if (type == GPRT_BOOL3) return "bool3"; 
    else if (type == GPRT_BOOL4) return "bool4"; 

    else if (type == GPRT_ACCEL) return "gprt::Accel";
    else if (type == GPRT_TRANSFORM) return "float3x4";
    else if (type == GPRT_TRANSFORM_3X4) return "float3x4";
    else if (type == GPRT_TRANSFORM_4X4) return "float4x4";

    // User Types have size encoded in their type enum
    else if (type > GPRT_USER_TYPE_BEGIN) return "userType";
    else assert(false); return "unknown";// std::runtime_error("Unimplemented!");
  }

  #define GPRT_USER_TYPE(userType) ((GPRTDataType)(GPRT_USER_TYPE_BEGIN+sizeof(userType)))

typedef struct _GPRTVarDecl {
  const char *name;
  GPRTDataType type; // note, also includes size if GPRT_USER_TYPE
  uint32_t    offset;
} GPRTVarDecl;

typedef struct _GPRTVarDef {
  GPRTVarDecl decl;
  void* data = nullptr;
} GPRTVarDef;

inline std::unordered_map<std::string, GPRTVarDef> checkAndPackVariables(
  const GPRTVarDecl *vars, int numVars)
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
  std::unordered_map<std::string, GPRTVarDef> varDefs;
  for (int i=0;i<numVars;i++) {
    assert(vars[i].name != nullptr);
    varDefs[vars[i].name].decl = vars[i];

    // allocate depending on the size of the variable...
    varDefs[vars[i].name].data = malloc(getSize(vars[i].type));
  }
  return varDefs;
}

inline std::vector<GPRTVarDecl> getDecls(
  std::unordered_map<std::string, GPRTVarDef> vars)
{
  std::vector<GPRTVarDecl> decls;
  for (auto &it : vars) {
    decls.push_back(it.second.decl);
  }
  return decls;
}

GPRT_API GPRTModule gprtModuleCreate(GPRTContext context, GPRTProgram spvCode);
GPRT_API void gprtModuleDestroy(GPRTModule module);

GPRT_API GPRTGeom
gprtGeomCreate(GPRTContext  context,
              GPRTGeomType type);

GPRT_API void
gprtGeomDestroy(GPRTGeom geometry);

GPRT_API void *
gprtGeomGetPointer(GPRTGeom geometry, int deviceID GPRT_IF_CPP(=0));

// ==================================================================
// "Triangles" functions
// ==================================================================
GPRT_API void gprtTrianglesSetVertices(GPRTGeom triangles,
                                      GPRTBuffer vertices,
                                      size_t count,
                                      size_t stride GPRT_IF_CPP(=sizeof(float3)),
                                      size_t offset GPRT_IF_CPP(=0));
// GPRT_API void gprtTrianglesSetMotionVertices(GPRTGeom triangles,
//                                            /*! number of vertex arrays
//                                                passed here, the first
//                                                of those is for t=0,
//                                                thelast for t=1,
//                                                everything is linearly
//                                                interpolated
//                                                in-between */
//                                            size_t    numKeys,
//                                            GPRTBuffer *vertexArrays,
//                                            size_t count,
//                                            size_t stride,
//                                            size_t offset);
GPRT_API void gprtTrianglesSetIndices(GPRTGeom triangles,
                                     GPRTBuffer indices,
                                     size_t count,
                                     size_t stride GPRT_IF_CPP(=sizeof(uint3)),
                                     size_t offset GPRT_IF_CPP(=0));

/*! set the aabb positions (minX, minY, minZ, maxX, maxY, maxZ) 
  for the given AABB geometry. This _has_ to be set before the accel(s) 
  that this geom is used in get built. */
GPRT_API void gprtAABBsSetPositions(GPRTGeom aabbs, 
                                    GPRTBuffer positions,
                                    size_t count,
                                    size_t stride GPRT_IF_CPP(=2*sizeof(float3)),
                                    size_t offset GPRT_IF_CPP(=0));

/* Builds the ray tracing pipeline over the raytracing programs. 
  This must be called after any acceleration structures are created.
*/
GPRT_API void gprtBuildPipeline(GPRTContext context);

GPRT_API void gprtBuildShaderBindingTable(GPRTContext context,
                         GPRTBuildSBTFlags flags GPRT_IF_CPP(=GPRT_SBT_ALL));

/** Tells the GPRT to create a window when once the context is made. 
 * @param initialWidth The width of the window in screen coordinates
 * @param initialHeight The height of the window in screen coordinates
 * @param title The title to put in the top bar of the window
*/
GPRT_API void gprtRequestWindow(
  uint32_t initialWidth, 
  uint32_t initialHeight, 
  const char *title);

/** If a window was requested, @returns true if the window's close button 
 * was clicked. This function can be called from any thread.
 * 
 * If a window was not requested (ie headless), this function always @returns
 * true.
*/
GPRT_API bool gprtWindowShouldClose(GPRTContext context);

/** If a window was requested, this function returns the position of the cursor
 * in screen coordinates relative to the upper left corner. 
 * 
 * If a window was not requested (ie headless), position arguments will be 
 * set to NULL.
 */
GPRT_API void gprtGetCursorPos(GPRTContext context, 
  double * xpos, double * ypos);

#define GPRT_RELEASE                0
#define GPRT_PRESS                  1
#define GPRT_REPEAT                 2

#define GPRT_MOUSE_BUTTON_1         0
#define GPRT_MOUSE_BUTTON_2         1
#define GPRT_MOUSE_BUTTON_3         2
#define GPRT_MOUSE_BUTTON_4         3
#define GPRT_MOUSE_BUTTON_5         4
#define GPRT_MOUSE_BUTTON_6         5
#define GPRT_MOUSE_BUTTON_7         6
#define GPRT_MOUSE_BUTTON_8         7
#define GPRT_MOUSE_BUTTON_LAST      GPRT_MOUSE_BUTTON_8
#define GPRT_MOUSE_BUTTON_LEFT      GPRT_MOUSE_BUTTON_1
#define GPRT_MOUSE_BUTTON_RIGHT     GPRT_MOUSE_BUTTON_2
#define GPRT_MOUSE_BUTTON_MIDDLE    GPRT_MOUSE_BUTTON_3

/** If a window was requested, this function returns the last state reported 
 * for the given mouse button. The returned state is one of GPRT_PRESS or 
 * GPRT_RELEASE.
 *  
 * If a window was not requested (ie headless), this function will return 
 * GPRT_RELEASE.
 */
GPRT_API int gprtGetMouseButton(GPRTContext context,
  int button);

/** If a window was requested, this function returns the time elapsed since 
 * GPRT was initialized. 
 *  
 * At the moment, if a window was not requested (ie headless), this function 
 * will return 0.
 */
GPRT_API double gprtGetTime(GPRTContext context);

/** creates a new device context with the gives list of devices.

  If requested device IDs list if null it implicitly refers to the
  list "0,1,2,...."; if numDevices <= 0 it automatically refers to
  "all devices you can find". Examples:

  - gprtContextCreate(nullptr,1) creates one device on the first GPU

  - gprtContextCreate(nullptr,0) creates a context across all GPUs in
  the system

  - int gpu=2;gprtContextCreate(&gpu,1) will create a context on GPU #2
  (where 2 refers to the CUDA device ordinal; from that point on, from
  gprt's standpoint (eg, during gprtBufferGetPointer() this GPU will
  from that point on be known as device #0 */
GPRT_API GPRTContext
gprtContextCreate(int32_t *requestedDeviceIDs GPRT_IF_CPP(=nullptr),
                 int numDevices GPRT_IF_CPP(=1));

GPRT_API void
gprtContextDestroy(GPRTContext context);

/*! set number of ray types to be used in this context; this should be
  done before any programs, pipelines, geometries, etc get
  created */
GPRT_API void
gprtContextSetRayTypeCount(GPRTContext context,
                           size_t numRayTypes);

/*! returns the number of ray types used in this context */
GPRT_API size_t
gprtContextGetRayTypeCount(GPRTContext context);

GPRT_API GPRTCompute
gprtComputeCreate(GPRTContext  context,
                 GPRTModule module,
                 const char *programName,
                 size_t      sizeOfVarStruct);

GPRT_API void
gprtComputeDestroy(GPRTCompute compute);

GPRT_API void *
gprtComputeGetPointer(GPRTCompute compute, int deviceID GPRT_IF_CPP(=0));

GPRT_API GPRTRayGen
gprtRayGenCreate(GPRTContext  context,
                 GPRTModule module,
                 const char *programName,
                 size_t      recordSize);

GPRT_API void
gprtRayGenDestroy(GPRTRayGen rayGen);

GPRT_API void *
gprtRayGenGetPointer(GPRTRayGen rayGen, int deviceID GPRT_IF_CPP(=0));

GPRT_API GPRTMiss
gprtMissCreate(GPRTContext  context,
                   GPRTModule module,
                   const char *programName,
                   size_t      recordSize);

/*! sets the given miss program for the given ray type */
GPRT_API void
gprtMissSet(GPRTContext  context,
               int rayType,
               GPRTMiss missProgToUse);

GPRT_API void
gprtMissDestroy(GPRTMiss missProg);

GPRT_API void *
gprtMissGetPointer(GPRTMiss missProg, int deviceID GPRT_IF_CPP(=0));

// ------------------------------------------------------------------
/*! create a new acceleration structure for AABB geometries.

  \param numGeometries Number of geometries in this acceleration structure, must
  be non-zero.

  \param arrayOfChildGeoms A array of 'numGeometries' child
  geometries. Every geom in this array must be a valid gprt geometry
  created with gprtGeomCreate, and must be of a GPRT_GEOM_USER
  type.

  \param flags reserved for future use
*/
GPRT_API GPRTAccel
gprtAABBAccelCreate(GPRTContext context,
                    size_t       numGeometries,
                    GPRTGeom    *arrayOfChildGeoms,
                    unsigned int flags GPRT_IF_CPP(=0));


// ------------------------------------------------------------------
/*! create a new acceleration structure for triangle geometries.

  \param numGeometries Number of geometries in this acceleration structure, must
  be non-zero.

  \param arrayOfChildGeoms A array of 'numGeometries' child
  geometries. Every geom in this array must be a valid gprt geometry
  created with gprtGeomCreate, and must be of a GPRT_GEOM_TRIANGLES
  type.

  \param flags reserved for future use
*/
GPRT_API GPRTAccel
gprtTrianglesAccelCreate(GPRTContext context,
                            size_t     numGeometries,
                            GPRTGeom   *arrayOfChildGeoms,
                            unsigned int flags GPRT_IF_CPP(=0));

// // ------------------------------------------------------------------
// /*! create a new acceleration structure for "curves" geometries.

//   \param numGeometries Number of geometries in this acceleration structure,
//   must be non-zero.

//   \param arrayOfChildGeoms A array of 'numGeometries' child
//   geometries. Every geom in this array must be a valid gprt geometry
//   created with gprtGeomCreate, and must be of a GPRT_GEOM_CURVES
//   type.

//   \param flags reserved for future use

//   Note that in order to use curves geometries you _have_ to call
//   gprtEnableCurves() before curves are used; in particular, curves
//   _have_ to already be enabled when the pipeline gets compiled.
// */
// GPRT_API GPRTAccel
// gprtCurvesAccelCreate(GPRTContext context,
//                          size_t     numCurveGeometries,
//                          GPRTGeom   *curveGeometries,
//                          unsigned int flags GPRT_IF_CPP(=0));

// ------------------------------------------------------------------
/*! create a new instance acceleration structure with given number of
  instances. 
  
  \param numAccels Number of acceleration structures instantiated in the leaves
  of this acceleration structure, must be non-zero.

  \param arrayOfAccels A array of 'numInstances' child
  acceleration structures. No accel in this array can be an instance accel.  

  \param flags reserved for future use
*/
GPRT_API GPRTAccel
gprtInstanceAccelCreate(GPRTContext context,
                        size_t numAccels,
                        GPRTAccel *arrayOfAccels,
                        unsigned int flags GPRT_IF_CPP(=0));

GPRT_API void 
gprtInstanceAccelSetTransforms(GPRTAccel instanceAccel,
                               GPRTBuffer transforms,
                               size_t stride,
                               size_t offset
                               );

GPRT_API void 
gprtInstanceAccelSet3x4Transforms(GPRTAccel instanceAccel,
                                  GPRTBuffer transforms);

GPRT_API void 
gprtInstanceAccelSet4x4Transforms(GPRTAccel instanceAccel,
                                  GPRTBuffer transforms);

/*! sets the list of IDs to use for the child instnaces. By default
    the instance ID of child #i is simply i, but optix allows to
    specify a user-defined instnace ID for each instance, which with
    owl can be done through this array. Array size must match number
    of instances in the specified group */
GPRT_API void
gprtInstanceAccelSetIDs(GPRTAccel instanceAccel,
                        const uint32_t *instanceIDs);

GPRT_API void
gprtInstanceAccelSetVisibilityMasks(GPRTAccel instanceAccel,
                                    const uint8_t *visibilityMasks);

GPRT_API void
gprtAccelDestroy(GPRTAccel accel);

GPRT_API void gprtAccelBuild(GPRTContext context, GPRTAccel accel);

GPRT_API void gprtAccelRefit(GPRTContext context, GPRTAccel accel);

GPRT_API gprt::Accel 
gprtAccelGetHandle(GPRTAccel accel, int deviceID GPRT_IF_CPP(=0));

GPRT_API GPRTGeomType
gprtGeomTypeCreate(GPRTContext  context,
                   GPRTGeomKind kind,
                   size_t       sizeOfVarStruct);

GPRT_API void
gprtGeomTypeDestroy(GPRTGeomType geomType);

GPRT_API void
gprtGeomTypeSetClosestHitProg(GPRTGeomType type,
                          int rayType,
                          GPRTModule module,
                          const char *progName);

GPRT_API void
gprtGeomTypeSetAnyHitProg(GPRTGeomType type,
                      int rayType,
                      GPRTModule module,
                      const char *progName);

GPRT_API void
gprtGeomTypeSetIntersectionProg(GPRTGeomType type,
                             int rayType,
                             GPRTModule module,
                             const char *progName);

/*! Creates a buffer that uses memory located on the host; that memory is 
accessible to all devices, but is slower to access on device.  */
GPRT_API GPRTBuffer
gprtHostBufferCreate(GPRTContext context, size_t size, size_t count, 
  const void* init GPRT_IF_CPP(= nullptr));

/*! Creates a buffer that uses memory located on the device; that memory is 
accessible only to the device, and requires mapping and unmapping to access 
on the host. */
GPRT_API GPRTBuffer
gprtDeviceBufferCreate(GPRTContext context, size_t size, size_t count, 
  const void* init GPRT_IF_CPP(= nullptr));

/*! Creates a buffer that uses memory located on the device; that memory is 
accessible to all devices, but is slower to access on the host, and is typically
limited in size depending on resizable BAR availability. */
GPRT_API GPRTBuffer
gprtSharedBufferCreate(GPRTContext context, size_t size, size_t count, 
  const void* init GPRT_IF_CPP(= nullptr));

/*! Destroys all underlying Vulkan resources for the given buffer and frees any
  underlying memory*/
GPRT_API void
gprtBufferDestroy(GPRTBuffer buffer);

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
GPRT_API void *
gprtBufferGetPointer(GPRTBuffer buffer, int deviceID GPRT_IF_CPP(=0));


GPRT_API void
gprtBufferMap(GPRTBuffer buffer, int deviceID GPRT_IF_CPP(=0));

GPRT_API void
gprtBufferUnmap(GPRTBuffer buffer, int deviceID GPRT_IF_CPP(=0));

GPRT_API gprt::Buffer 
gprtBufferGetHandle(GPRTBuffer buffer, int deviceID GPRT_IF_CPP(=0));

/** If a window was requested, this call interprets the given buffer as 
 * a B8G8R8A8 SRGB image sorted in row major buffer, and presents the contents 
 * to the window, potentially waiting for the screen to update before swapping.
 * 
 * If a window was not requested (ie headless), this function does nothing. 
*/
GPRT_API void gprtBufferPresent(GPRTContext context, GPRTBuffer buffer);

/** This call interprets the given buffer as a B8G8R8A8 SRGB image sorted in 
 * row major buffer, and saves the contents to the underlying filesystem.
*/
GPRT_API void gprtBufferSaveImage(GPRTBuffer buffer, 
  uint32_t width, uint32_t height, const char *imageName);

GPRT_API void
gprtRayGenLaunch1D(GPRTContext context, GPRTRayGen rayGen, int dims_x);

/*! Executes a ray tracing pipeline with the given raygen program.
  This call will block until the raygen program returns. */
GPRT_API void
gprtRayGenLaunch2D(GPRTContext context, GPRTRayGen rayGen, int dims_x, int dims_y);

/*! 3D-launch variant of \see gprtRayGenLaunch2D */
GPRT_API void
gprtRayGenLaunch3D(GPRTContext context, GPRTRayGen rayGen, int dims_x, int dims_y, int dims_z);

GPRT_API void
gprtComputeLaunch1D(GPRTContext context, GPRTCompute compute, int dims_x);

GPRT_API void
gprtComputeLaunch2D(GPRTContext context, GPRTCompute compute, int dims_x, int dims_y);

GPRT_API void
gprtComputeLaunch3D(GPRTContext context, GPRTCompute compute, int dims_x, int dims_y, int dims_z);

GPRT_API void gprtBeginProfile(GPRTContext context);

// returned results are in nanoseconds
GPRT_API float gprtEndProfile(GPRTContext context);
