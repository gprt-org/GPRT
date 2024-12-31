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

#include <vulkan/vulkan.h>

#include "MathConstants.slangh"
#include "Matrix.h"
#include "Vector.h"

// #include "linalg.h"
// using namespace linalg;
// using namespace linalg::detail; // causes conflicts
// using namespace linalg::aliases;
// using namespace linalg::ostream_overloads;

#include <stdint.h>
#include <sys/types.h>

#include <array>
#include <assert.h>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <map>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <vector>

#ifdef __cplusplus
#include <cstddef>
#endif

#if defined(_MSC_VER)
#define GPRT_DLL_EXPORT __declspec(dllexport)
#define GPRT_DLL_IMPORT __declspec(dllimport)
#elif defined(__clang__) || defined(__GNUC__)
#define GPRT_DLL_EXPORT __attribute__((visibility("default")))
#define GPRT_DLL_IMPORT __attribute__((visibility("default")))
#else
#define GPRT_DLL_EXPORT
#define GPRT_DLL_IMPORT
#endif

#ifdef __cplusplus
#define GPRT_IF_CPP(a) a
#else
#define GPRT_IF_CPP(a) /* drop it */
#endif

#ifdef __cplusplus
#define GPRT_API extern "C" GPRT_DLL_EXPORT
#else
#define GPRT_API /* bla */
#endif

#define GPRT_OFFSETOF(type, member)                                                                                    \
  (uint32_t) ((char *) (&((struct type *) 0)->member) - (char *) (((struct type *) 0)))

// Terminal colors
#define GPRT_TERMINAL_RED           "\033[0;31m"
#define GPRT_TERMINAL_GREEN         "\033[0;32m"
#define GPRT_TERMINAL_LIGHT_GREEN   "\033[1;32m"
#define GPRT_TERMINAL_YELLOW        "\033[1;33m"
#define GPRT_TERMINAL_BLUE          "\033[0;34m"
#define GPRT_TERMINAL_LIGHT_BLUE    "\033[1;34m"
#define GPRT_TERMINAL_RESET         "\033[0m"
#define GPRT_TERMINAL_DEFAULT       GPRT_TERMINAL_RESET
#define GPRT_TERMINAL_BOLD          "\033[1;1m"
#define GPRT_TERMINAL_MAGENTA       "\e[35m"
#define GPRT_TERMINAL_LIGHT_MAGENTA "\e[95m"
#define GPRT_TERMINAL_CYAN          "\e[36m"
#define GPRT_TERMINAL_LIGHT_RED     "\033[1;31m"

using GPRTContext = struct _GPRTContext *;
using GPRTModule = struct _GPRTModule *;
using GPRTAccel = struct _GPRTAccel *;
using GPRTBuffer = struct _GPRTBuffer *;
using GPRTTexture = struct _GPRTTexture *;
using GPRTSampler = struct _GPRTSampler *;
using GPRTGeom = struct _GPRTGeom *;
using GPRTGeomType = struct _GPRTGeomType *;
using GPRTRayGen = struct _GPRTRayGen *;
using GPRTMiss = struct _GPRTMiss *;
using GPRTCallable = struct _GPRTCallable *;
using GPRTCompute = struct _GPRTCompute *;

template <typename T> struct _GPRTBufferOf;
template <typename T> struct _GPRTTextureOf;
template <typename T> struct _GPRTRayGenOf;
template <typename T> struct _GPRTMissOf;
template <typename T> struct _GPRTCallableOf;
template <typename... T> struct _GPRTComputeOf;
template <typename T> struct _GPRTGeomOf;
template <typename T> struct _GPRTGeomTypeOf;
template <typename T> using GPRTRayGenOf = struct _GPRTRayGenOf<T> *;
template <typename T> using GPRTBufferOf = struct _GPRTBufferOf<T> *;
template <typename T> using GPRTTextureOf = struct _GPRTTextureOf<T> *;
template <typename T> using GPRTMissOf = struct _GPRTMissOf<T> *;
template <typename T> using GPRTCallableOf = struct _GPRTCallableOf<T> *;
template <typename... T> using GPRTComputeOf = struct _GPRTComputeOf<T...> *;
template <typename T> using GPRTGeomOf = struct _GPRTGeomOf<T> *;
template <typename T> using GPRTGeomTypeOf = struct _GPRTGeomTypeOf<T> *;

// GPRTPrograms are just SPIR-V binaries under the hood
using GPRTProgram = std::vector<uint8_t>;

// Shared internal data structures between GPU and CPU
#include "gprt_shared.h"

// General helper functions
static void
runtime_assert(bool condition, const char *message) {
  if (!condition) {
    std::cerr << "Runtime assertion failed: " << message << std::endl;
    std::abort();   // or throw an exception, depending on your needs
  }
}

template <typename T>
void
handleArg(std::array<char, PUSH_CONSTANTS_LIMIT> &buffer, size_t &offset, const T &arg) {
  static_assert(std::is_trivially_copyable<T>::value, "Non-trivially copyable types are not supported.");
  std::memcpy(buffer.data() + offset, &arg, sizeof(T));
  offset += sizeof(T);
}

template <typename... Args>
constexpr size_t
totalSizeOf() {
  return (sizeof(Args) + ... + 0);
}

/*! launch params (or "globals") are variables that can be put into
  device constant memory, accessible through Vulkan's push constants */
typedef struct _GPRTLaunchParams *GPRTLaunchParams, *GPRTParams, *GPRTGlobals;

typedef enum {
  GPRT_SBT_HITGROUP = 1,
  GPRT_SBT_GEOM = GPRT_SBT_HITGROUP,
  GPRT_SBT_RAYGEN = 2,
  GPRT_SBT_MISS = 4,
  GPRT_SBT_CALLABLE = 8,
  GPRT_SBT_ALL = 63
} GPRTBuildSBTFlags;

/*! enum that specifies the different possible memory layouts for
  passing transformation matrices */
typedef enum {
  /*! A 4x3-float column-major matrix format, where a matrix is
    specified through four float3's, the first three being the basis
    vectors of the linear transform, and the fourth one the
    translation part. */
  GPRT_MATRIX_FORMAT_COLUMN_MAJOR = 0,

  /*! 3x4-float *row-major* layout as preferred by vulkan matching
    VkTransformMatrixKHR; not that, in this case, it doesn't matter if it's a
    4x3 or 4x4 matrix, as the last row in a 4x4 row major matrix can simply be
    ignored */
  GPRT_MATRIX_FORMAT_ROW_MAJOR
} GPRTMatrixFormat;

typedef enum { 
  GPRT_UNKNOWN, GPRT_AABBS, GPRT_TRIANGLES, GPRT_SPHERES, GPRT_LSS, /*bilinear solids?*/GPRT_SOLIDS
} GPRTGeomKind;


// Indices below are made to match VTU/VTK where possible.
typedef enum {
  GPRT_TETRAHEDRON = 10,
  GPRT_HEXAHEDRON = 12,
  GPRT_WEDGE = 13, 
  GPRT_PYRAMID = 14, 
  GPRT_TETRAHEDRAL_PAIR = 23,
} GPRTSolidTypes;

  // // Linear cells
  // GPRT_TETRA = 10,
  // GPRT_VOXEL = 11,
  // GPRT_HEXAHEDRON = 12,
  // GPRT_WEDGE = 13,
  // GPRT_PYRAMID = 14,
  // GPRT_PENTAGONAL_PRISM = 15,
  // GPRT_HEXAGONAL_PRISM = 16,

  // // Quadratic, isoparametric cells
  // GPRT_QUADRATIC_TETRA = 24,
  // GPRT_QUADRATIC_HEXAHEDRON = 25,
  // GPRT_QUADRATIC_WEDGE = 26,
  // GPRT_QUADRATIC_PYRAMID = 27,


  // GPRT_TRIQUADRATIC_HEXAHEDRON = 29,
  // GPRT_TRIQUADRATIC_PYRAMID = 37,
  // GPRT_QUADRATIC_LINEAR_WEDGE = 31,
  // GPRT_BIQUADRATIC_QUADRATIC_WEDGE = 32,
  // GPRT_BIQUADRATIC_QUADRATIC_HEXAHEDRON = 33,

  // // Polyhedron cell (consisting of polygonal faces)
  // GPRT_POLYHEDRON = 42,

  // // Higher order cells
  // GPRT_HIGHER_ORDER_TETRAHEDRON = 64,
  // GPRT_HIGHER_ORDER_WEDGE = 65,
  // GPRT_HIGHER_ORDER_PYRAMID = 66,
  // GPRT_HIGHER_ORDER_HEXAHEDRON = 67,

  // // Arbitrary order Lagrange elements (formulated separated from generic higher order cells)
  // GPRT_LAGRANGE_TETRAHEDRON = 71,
  // GPRT_LAGRANGE_HEXAHEDRON = 72,
  // GPRT_LAGRANGE_WEDGE = 73,
  // GPRT_LAGRANGE_PYRAMID = 74,

  // // Arbitrary order Bezier elements (formulated separated from generic higher order cells)
  // GPRT_BEZIER_TETRAHEDRON = 78,
  // GPRT_BEZIER_HEXAHEDRON = 79,
  // GPRT_BEZIER_WEDGE = 80,
  // GPRT_BEZIER_PYRAMID = 81,
// } GPRTCellType;

typedef enum {
  GPRT_BUILD_MODE_UNINITIALIZED,
  GPRT_BUILD_MODE_FAST_BUILD_NO_UPDATE,
  GPRT_BUILD_MODE_FAST_BUILD_AND_UPDATE,
  GPRT_BUILD_MODE_FAST_TRACE_NO_UPDATE,
  GPRT_BUILD_MODE_FAST_TRACE_AND_UPDATE,
} GPRTBuildMode;

/*! supported formats for texels in textures */
typedef enum {
  GPRT_IMAGE_TYPE_1D = VK_IMAGE_TYPE_1D,
  GPRT_IMAGE_TYPE_2D = VK_IMAGE_TYPE_2D,
  GPRT_IMAGE_TYPE_3D = VK_IMAGE_TYPE_3D,
} GPRTImageType;

/*! supported formats for texels in textures */
typedef enum {
  // note, for now, values here are made to match VkFormat equivalents.
  GPRT_FORMAT_R8_UINT = VK_FORMAT_R8_UINT,
  GPRT_FORMAT_R16_UNORM = VK_FORMAT_R16_UNORM,
  GPRT_FORMAT_R8G8B8A8_UNORM = VK_FORMAT_R8G8B8A8_UNORM,
  GPRT_FORMAT_R8G8B8A8_SRGB = VK_FORMAT_R8G8B8A8_SRGB,
  GPRT_FORMAT_R32_SFLOAT = VK_FORMAT_R32_SFLOAT,
  GPRT_FORMAT_R32G32B32A32_SFLOAT = VK_FORMAT_R32G32B32A32_SFLOAT,
  GPRT_FORMAT_D32_SFLOAT = VK_FORMAT_D32_SFLOAT
} GPRTFormat;

/*! currently supported texture filter modes */
typedef enum { GPRT_FILTER_NEAREST = VK_FILTER_NEAREST, GPRT_FILTER_LINEAR = VK_FILTER_LINEAR } GPRTFilter;

/*! currently supported texture filter modes */
typedef enum {
  GPRT_SAMPLER_ADDRESS_MODE_REPEAT = VK_SAMPLER_ADDRESS_MODE_REPEAT,
  GPRT_SAMPLER_ADDRESS_MODE_CLAMP = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE,
  GPRT_SAMPLER_ADDRESS_MODE_BORDER = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER,
  GPRT_SAMPLER_ADDRESS_MODE_MIRROR = VK_SAMPLER_ADDRESS_MODE_MIRRORED_REPEAT
} GPRTSamplerAddressMode;

typedef enum {
  GPRT_BORDER_COLOR_TRANSPARENT_BLACK = VK_BORDER_COLOR_FLOAT_TRANSPARENT_BLACK,
  GPRT_BORDER_COLOR_OPAQUE_BLACK = VK_BORDER_COLOR_FLOAT_OPAQUE_BLACK,
  GPRT_BORDER_COLOR_OPAQUE_WHITE = VK_BORDER_COLOR_FLOAT_OPAQUE_WHITE
} GPRTBorderColor;

/*! Indicates if a texture is linear or SRGB */
typedef enum { GPRT_COLOR_SPACE_LINEAR, GPRT_COLOR_SPACE_SRGB } GPRTColorSpace;

GPRT_API GPRTModule gprtModuleCreate(GPRTContext context, GPRTProgram spvCode);
GPRT_API void gprtModuleDestroy(GPRTModule module);

GPRT_API GPRTGeom gprtGeomCreate(GPRTContext context, GPRTGeomType type);

template <typename T>
GPRTGeomOf<T>
gprtGeomCreate(GPRTContext context, GPRTGeomTypeOf<T> type) {
  return (GPRTGeomOf<T>) gprtGeomCreate(context, (GPRTGeomType) type);
}

GPRT_API void gprtGeomDestroy(GPRTGeom geometry);

template <typename T>
void
gprtGeomDestroy(GPRTGeomOf<T> geometry) {
  gprtGeomDestroy((GPRTGeom) geometry);
}

/**
 * @brief Returns a pointer to the parameters section of the record for this geometry, made
 * available on the device through the second parameter in GPRT_CLOSEST_HIT_PROGRAM,
 * GPRT_ANY_HIT_PROGRAM, GPRT_INTERSECTION_PROGRAM, GPRT_VERTEX_PROGRAM, and/or GPRT_PIXEL_PROGRAM.
 * Note, call @ref gprtBuildShaderBindingTable for these parameters to be uploaded to the device.
 *
 * @param geometry The geometry who's record pointer is to be returned.
 * @returns a pointer to the parameters section of the record.
 */
GPRT_API void *gprtGeomGetParameters(GPRTGeom geometry, int deviceID GPRT_IF_CPP(= 0));

/**
 * @brief Returns a pointer to the parameters section of the record for this geometry, made
 * available on the device through the second parameter in GPRT_CLOSEST_HIT_PROGRAM,
 * GPRT_ANY_HIT_PROGRAM, GPRT_INTERSECTION_PROGRAM, GPRT_VERTEX_PROGRAM, and/or GPRT_PIXEL_PROGRAM.
 * Note, call  @ref gprtBuildShaderBindingTable for these parameters to be uploaded to the device.
 *
 * @tparam T The type of the parameters structure stored in the record
 * @param geometry The geometry who's record pointer is to be returned.
 * @returns a pointer to the parameters section of the record.
 */
template <typename T>
T *
gprtGeomGetParameters(GPRTGeomOf<T> geometry, int deviceID GPRT_IF_CPP(= 0)) {
  return (T *) gprtGeomGetParameters((GPRTGeom) geometry, deviceID);
}

/**
 * @brief Copies the contents of the given parameters into the geometry record. Note, call
 * @ref gprtBuildShaderBindingTable for these parameters to be uploaded to the device.
 *
 * @param geometry The geometry who's record to assign the parameters to.
 * @param parameters A pointer to a parameters structure. Assumed to be "recordSize" bytes,
 * as specified by @ref gprtGeomTypeCreate.
 */
GPRT_API void gprtGeomSetParameters(GPRTGeom geometry, void *parameters, int deviceID GPRT_IF_CPP(= 0));

/**
 * @brief Copies the contents of the given parameters into the geometry record. Note, call
 * @ref gprtBuildShaderBindingTable for these parameters to be uploaded to the device.
 * @tparam T The type of the parameters structure stored in the record.
 * @param geometry The geometry who's record to assign the parameters to.
 * @param parameters A pointer to a parameters structure. Assumed to be "recordSize" bytes,
 * as specified by @ref gprtGeomTypeCreate.
 */
template <typename T>
void
gprtGeomSetParameters(GPRTGeomOf<T> geometry, T &parameters, int deviceID GPRT_IF_CPP(= 0)) {
  gprtGeomSetParameters((GPRTGeom) geometry, (void *) &parameters, deviceID);
}

// ==================================================================
// "Triangles" functions
// ==================================================================
GPRT_API void gprtTrianglesSetVertices(GPRTGeom triangles, GPRTBuffer vertices, uint32_t count,
                                       uint32_t stride GPRT_IF_CPP(= sizeof(float3)), uint32_t offset GPRT_IF_CPP(= 0));

template <typename T1, typename T2>
void
gprtTrianglesSetVertices(GPRTGeomOf<T1> triangles, GPRTBufferOf<T2> vertices, uint32_t count,
                         uint32_t stride GPRT_IF_CPP(= sizeof(float3)), uint32_t offset GPRT_IF_CPP(= 0)) {
  gprtTrianglesSetVertices((GPRTGeom) triangles, (GPRTBuffer) vertices, count, stride, offset);
}

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

GPRT_API void gprtTrianglesSetIndices(GPRTGeom triangles, GPRTBuffer indices, uint32_t count,
                                      uint32_t stride GPRT_IF_CPP(= sizeof(uint3)), uint32_t offset GPRT_IF_CPP(= 0));

template <typename T1, typename T2>
void
gprtTrianglesSetIndices(GPRTGeomOf<T1> triangles, GPRTBufferOf<T2> indices, uint32_t count,
                        uint32_t stride GPRT_IF_CPP(= sizeof(uint3)), uint32_t offset GPRT_IF_CPP(= 0)) {
  gprtTrianglesSetIndices((GPRTGeom) triangles, (GPRTBuffer) indices, count, stride, offset);
}

GPRT_API void gprtSpheresSetVertices(GPRTGeom sphereGeom, GPRTBuffer vertices, uint32_t count,
                                     uint32_t stride GPRT_IF_CPP(= sizeof(float4)), uint32_t offset GPRT_IF_CPP(= 0));

template <typename T1, typename T2>
void
gprtSpheresSetVertices(GPRTGeomOf<T1> sphereGeom, GPRTBufferOf<T2> vertices, uint32_t count,
                       uint32_t stride GPRT_IF_CPP(= sizeof(float4)), uint32_t offset GPRT_IF_CPP(= 0)) {
  gprtSpheresSetVertices((GPRTGeom) sphereGeom, (GPRTBuffer) vertices, count, stride, offset);
}

GPRT_API void gprtLSSSetVertices(GPRTGeom lssGeom, GPRTBuffer vertices, uint32_t count,
                                 uint32_t stride GPRT_IF_CPP(= sizeof(float4)), uint32_t offset GPRT_IF_CPP(= 0));

template <typename T1, typename T2>
void
gprtLSSSetVertices(GPRTGeomOf<T1> lssGeom, GPRTBufferOf<T2> vertices, uint32_t count,
                   uint32_t stride GPRT_IF_CPP(= sizeof(float4)), uint32_t offset GPRT_IF_CPP(= 0)) {
  gprtLSSSetVertices((GPRTGeom) lssGeom, (GPRTBuffer) vertices, count, stride, offset);
}

GPRT_API void gprtLSSSetIndices(GPRTGeom triangles, GPRTBuffer indices, uint32_t count,
                                uint32_t stride GPRT_IF_CPP(= sizeof(uint2)), uint32_t offset GPRT_IF_CPP(= 0));

template <typename T1, typename T2>
void
gprtLSSSetIndices(GPRTGeomOf<T1> triangles, GPRTBufferOf<T2> indices, uint32_t count,
                  uint32_t stride GPRT_IF_CPP(= sizeof(uint2)), uint32_t offset GPRT_IF_CPP(= 0)) {
  gprtLSSSetIndices((GPRTGeom) triangles, (GPRTBuffer) indices, count, stride, offset);
}

GPRT_API void gprtSolidsSetVertices(GPRTGeom solidsGeom, GPRTBuffer vertices, uint32_t count,
                                 uint32_t stride GPRT_IF_CPP(= sizeof(float4)), uint32_t offset GPRT_IF_CPP(= 0));

template <typename T1, typename T2>
void
gprtSolidsSetVertices(GPRTGeomOf<T1> solidsGeom, GPRTBufferOf<T2> vertices, uint32_t count,
                   uint32_t stride GPRT_IF_CPP(= sizeof(float4)), uint32_t offset GPRT_IF_CPP(= 0)) {
  gprtSolidsSetVertices((GPRTGeom) solidsGeom, (GPRTBuffer) vertices, count, stride, offset);
}

GPRT_API void gprtSolidsSetIndices(GPRTGeom solidsGeom, GPRTBuffer indices, uint32_t count,
                                   uint32_t stride GPRT_IF_CPP(= 2 * sizeof(uint4)), uint32_t offset GPRT_IF_CPP(= 0));

template <typename T1, typename T2>
void
gprtSolidsSetIndices(GPRTGeomOf<T1> solidsGeom, GPRTBufferOf<T2> indices, uint32_t count,
                  uint32_t stride GPRT_IF_CPP(= 2 * sizeof(uint4)), uint32_t offset GPRT_IF_CPP(= 0)) {
  gprtSolidsSetIndices((GPRTGeom) solidsGeom, (GPRTBuffer) indices, count, stride, offset);
}

GPRT_API void gprtSolidsSetTypes(GPRTGeom solidsGeom, GPRTBuffer types, uint32_t count,
                                   uint32_t stride GPRT_IF_CPP(= sizeof(uint8_t)), uint32_t offset GPRT_IF_CPP(= 0));

template <typename T1, typename T2>
void
gprtSolidsSetTypes(GPRTGeomOf<T1> solidsGeom, GPRTBufferOf<T2> types, uint32_t count,
                  uint32_t stride GPRT_IF_CPP(= sizeof(uint8_t)), uint32_t offset GPRT_IF_CPP(= 0)) {
  gprtSolidsSetTypes((GPRTGeom) solidsGeom, (GPRTBuffer) types, count, stride, offset);
}

/*! set the aabb positions (minX, minY, minZ, maxX, maxY, maxZ)
  for the given AABB geometry. This _has_ to be set before the accel(s)
  that this geom is used in get built. */
GPRT_API void gprtSolidsSetPositions(GPRTGeom aabbs, GPRTBuffer positions, uint32_t count,
                                    uint32_t stride GPRT_IF_CPP(= 8 * sizeof(float4)),
                                    uint32_t offset GPRT_IF_CPP(= 0));

template <typename T1, typename T2>
void
gprtSolidsSetPositions(GPRTGeomOf<T1> aabbs, GPRTBufferOf<T2> positions, uint32_t count,
                      uint32_t stride GPRT_IF_CPP(= 2 * sizeof(float3)), uint32_t offset GPRT_IF_CPP(= 0)) {
  gprtSolidsSetPositions((GPRTGeom) aabbs, (GPRTBuffer) positions, count, stride, offset);
}

/*! set the aabb positions (minX, minY, minZ, maxX, maxY, maxZ)
  for the given AABB geometry. This _has_ to be set before the accel(s)
  that this geom is used in get built. */
GPRT_API void gprtAABBsSetPositions(GPRTGeom aabbs, GPRTBuffer positions, uint32_t count,
                                    uint32_t stride GPRT_IF_CPP(= 2 * sizeof(float3)),
                                    uint32_t offset GPRT_IF_CPP(= 0));

template <typename T1, typename T2>
void
gprtAABBsSetPositions(GPRTGeomOf<T1> aabbs, GPRTBufferOf<T2> positions, uint32_t count,
                      uint32_t stride GPRT_IF_CPP(= 2 * sizeof(float3)), uint32_t offset GPRT_IF_CPP(= 0)) {
  gprtAABBsSetPositions((GPRTGeom) aabbs, (GPRTBuffer) positions, count, stride, offset);
}

GPRT_API void gprtBuildShaderBindingTable(GPRTContext context, GPRTBuildSBTFlags flags GPRT_IF_CPP(= GPRT_SBT_ALL));

/** Tells the GPRT to create a window when once the context is made.
 * @param initialWidth The width of the window in screen coordinates
 * @param initialHeight The height of the window in screen coordinates
 * @param title The title to put in the top bar of the window
 */
GPRT_API void gprtRequestWindow(uint32_t initialWidth, uint32_t initialHeight, const char *title);

/** If a window was requested, @returns true if the window's close button
 * was clicked. This function can be called from any thread.
 *
 * If a window was not requested (ie headless), this function always @returns
 * true.
 */
GPRT_API bool gprtWindowShouldClose(GPRTContext context);

/** If a window was requested, this function sets the title in the top bar of
 * the window to the given text.
 *
 * If a window was not requested (ie headless), this function does nothing
 */
GPRT_API void gprtSetWindowTitle(GPRTContext context, const char *title);

/** If a window was requested, this function returns the position of the cursor
 * in screen coordinates relative to the upper left corner.
 *
 * If a window was not requested (ie headless), position arguments will be
 * set to NULL.
 */
GPRT_API void gprtGetCursorPos(GPRTContext context, double *xpos, double *ypos);

#define GPRT_RELEASE 0
#define GPRT_PRESS   1
#define GPRT_REPEAT  2

#define GPRT_MOUSE_BUTTON_1      0
#define GPRT_MOUSE_BUTTON_2      1
#define GPRT_MOUSE_BUTTON_3      2
#define GPRT_MOUSE_BUTTON_4      3
#define GPRT_MOUSE_BUTTON_5      4
#define GPRT_MOUSE_BUTTON_6      5
#define GPRT_MOUSE_BUTTON_7      6
#define GPRT_MOUSE_BUTTON_8      7
#define GPRT_MOUSE_BUTTON_LAST   GPRT_MOUSE_BUTTON_8
#define GPRT_MOUSE_BUTTON_LEFT   GPRT_MOUSE_BUTTON_1
#define GPRT_MOUSE_BUTTON_RIGHT  GPRT_MOUSE_BUTTON_2
#define GPRT_MOUSE_BUTTON_MIDDLE GPRT_MOUSE_BUTTON_3

/** If a window was requested, this function returns the last state reported
 * for the given mouse button. The returned state is one of GPRT_PRESS or
 * GPRT_RELEASE.
 *
 * If a window was not requested (ie headless), this function will return
 * GPRT_RELEASE.
 */
GPRT_API int gprtGetMouseButton(GPRTContext context, int button);

/* The unknown key */
#define GPRT_KEY_UNKNOWN -1

/* Printable keys */
#define GPRT_KEY_SPACE         32
#define GPRT_KEY_APOSTROPHE    39 /* ' */
#define GPRT_KEY_COMMA         44 /* , */
#define GPRT_KEY_MINUS         45 /* - */
#define GPRT_KEY_PERIOD        46 /* . */
#define GPRT_KEY_SLASH         47 /* / */
#define GPRT_KEY_0             48
#define GPRT_KEY_1             49
#define GPRT_KEY_2             50
#define GPRT_KEY_3             51
#define GPRT_KEY_4             52
#define GPRT_KEY_5             53
#define GPRT_KEY_6             54
#define GPRT_KEY_7             55
#define GPRT_KEY_8             56
#define GPRT_KEY_9             57
#define GPRT_KEY_SEMICOLON     59 /* ; */
#define GPRT_KEY_EQUAL         61 /* = */
#define GPRT_KEY_A             65
#define GPRT_KEY_B             66
#define GPRT_KEY_C             67
#define GPRT_KEY_D             68
#define GPRT_KEY_E             69
#define GPRT_KEY_F             70
#define GPRT_KEY_G             71
#define GPRT_KEY_H             72
#define GPRT_KEY_I             73
#define GPRT_KEY_J             74
#define GPRT_KEY_K             75
#define GPRT_KEY_L             76
#define GPRT_KEY_M             77
#define GPRT_KEY_N             78
#define GPRT_KEY_O             79
#define GPRT_KEY_P             80
#define GPRT_KEY_Q             81
#define GPRT_KEY_R             82
#define GPRT_KEY_S             83
#define GPRT_KEY_T             84
#define GPRT_KEY_U             85
#define GPRT_KEY_V             86
#define GPRT_KEY_W             87
#define GPRT_KEY_X             88
#define GPRT_KEY_Y             89
#define GPRT_KEY_Z             90
#define GPRT_KEY_LEFT_BRACKET  91  /* [ */
#define GPRT_KEY_BACKSLASH     92  /* \ */
#define GPRT_KEY_RIGHT_BRACKET 93  /* ] */
#define GPRT_KEY_GRAVE_ACCENT  96  /* ` */
#define GPRT_KEY_WORLD_1       161 /* non-US #1 */
#define GPRT_KEY_WORLD_2       162 /* non-US #2 */

/* Function keys */
#define GPRT_KEY_ESCAPE        256
#define GPRT_KEY_ENTER         257
#define GPRT_KEY_TAB           258
#define GPRT_KEY_BACKSPACE     259
#define GPRT_KEY_INSERT        260
#define GPRT_KEY_DELETE        261
#define GPRT_KEY_RIGHT         262
#define GPRT_KEY_LEFT          263
#define GPRT_KEY_DOWN          264
#define GPRT_KEY_UP            265
#define GPRT_KEY_PAGE_UP       266
#define GPRT_KEY_PAGE_DOWN     267
#define GPRT_KEY_HOME          268
#define GPRT_KEY_END           269
#define GPRT_KEY_CAPS_LOCK     280
#define GPRT_KEY_SCROLL_LOCK   281
#define GPRT_KEY_NUM_LOCK      282
#define GPRT_KEY_PRINT_SCREEN  283
#define GPRT_KEY_PAUSE         284
#define GPRT_KEY_F1            290
#define GPRT_KEY_F2            291
#define GPRT_KEY_F3            292
#define GPRT_KEY_F4            293
#define GPRT_KEY_F5            294
#define GPRT_KEY_F6            295
#define GPRT_KEY_F7            296
#define GPRT_KEY_F8            297
#define GPRT_KEY_F9            298
#define GPRT_KEY_F10           299
#define GPRT_KEY_F11           300
#define GPRT_KEY_F12           301
#define GPRT_KEY_F13           302
#define GPRT_KEY_F14           303
#define GPRT_KEY_F15           304
#define GPRT_KEY_F16           305
#define GPRT_KEY_F17           306
#define GPRT_KEY_F18           307
#define GPRT_KEY_F19           308
#define GPRT_KEY_F20           309
#define GPRT_KEY_F21           310
#define GPRT_KEY_F22           311
#define GPRT_KEY_F23           312
#define GPRT_KEY_F24           313
#define GPRT_KEY_F25           314
#define GPRT_KEY_KP_0          320
#define GPRT_KEY_KP_1          321
#define GPRT_KEY_KP_2          322
#define GPRT_KEY_KP_3          323
#define GPRT_KEY_KP_4          324
#define GPRT_KEY_KP_5          325
#define GPRT_KEY_KP_6          326
#define GPRT_KEY_KP_7          327
#define GPRT_KEY_KP_8          328
#define GPRT_KEY_KP_9          329
#define GPRT_KEY_KP_DECIMAL    330
#define GPRT_KEY_KP_DIVIDE     331
#define GPRT_KEY_KP_MULTIPLY   332
#define GPRT_KEY_KP_SUBTRACT   333
#define GPRT_KEY_KP_ADD        334
#define GPRT_KEY_KP_ENTER      335
#define GPRT_KEY_KP_EQUAL      336
#define GPRT_KEY_LEFT_SHIFT    340
#define GPRT_KEY_LEFT_CONTROL  341
#define GPRT_KEY_LEFT_ALT      342
#define GPRT_KEY_LEFT_SUPER    343
#define GPRT_KEY_RIGHT_SHIFT   344
#define GPRT_KEY_RIGHT_CONTROL 345
#define GPRT_KEY_RIGHT_ALT     346
#define GPRT_KEY_RIGHT_SUPER   347
#define GPRT_KEY_MENU          348

#define GPRT_KEY_LAST GPRT_KEY_MENU

/** If a window was requested, this function returns the last state reported
 * for the given keyboard button. The returned state is one of GPRT_PRESS or
 * GPRT_RELEASE.
 *
 * If a window was not requested (ie headless), this function will return
 * GPRT_RELEASE.
 */
GPRT_API int gprtGetKey(GPRTContext context, int key);

/** If a window was requested, this function returns the time elapsed (in seconds)
 * since GPRT was initialized.
 *
 * At the moment, if a window was not requested (ie headless), this function
 * will return 0.
 */
GPRT_API double gprtGetTime(GPRTContext context);

/**
 * @brief If a window was requested, this function configures which textures
 * should be used when rasterizing the graphical user interface (using gprtGuiRasterize).
 *
 * @param context The GPRT context
 * @param colorAttachment The color attachment to rasterize the GUI into
 * @param depthAttachment The depth attachment to rasterize the GUI into
 */
GPRT_API void gprtGuiSetRasterAttachments(GPRTContext context, GPRTTexture colorAttachment,
                                          GPRTTexture depthAttachment);

/**
 * @brief  If a window was requested, this function configures which textures
 * should be used when rasterizing the graphical user interface (using gprtGuiRasterize).
 *
 * @tparam T1 The type of the color attachment.
 * @tparam T2 The type of the depth attachment
 * @param context The GPRT context
 * @param colorAttachment The color attachment to rasterize the GUI into
 * @param depthAttachment The depth attachment to rasterize the GUI into
 */
template <typename T1, typename T2>
void
gprtGuiSetRasterAttachments(GPRTContext context, GPRTTextureOf<T1> colorAttachment, GPRTTextureOf<T2> depthAttachment) {
  gprtGuiSetRasterAttachments(context, (GPRTTexture) colorAttachment, (GPRTTexture) depthAttachment);
}

/**
 * @brief If a window was requested, this function rasterizes the graphical user interface
 * into the texture attachments specified by gprtGuiSetRasterAttachments.
 *
 * @param context The GPRT context
 */
GPRT_API void gprtGuiRasterize(GPRTContext context);

/*! Requests the given size (in bytes) to reserve for parameters
  of ray tracing programs. Defaults to 256 bytes */
GPRT_API void gprtRequestRecordSize(uint32_t recordSize);

/*! set number of ray types to be used; this should be
  done before any programs, pipelines, geometries, etc get
  created */
GPRT_API void gprtRequestRayTypeCount(uint32_t numRayTypes);

/*! set maximum recursion depth available in a ray tracing pipeline.
 Currently defaults to 1, ie no recursion. */
GPRT_API void gprtRequestRayRecursionDepth(uint32_t rayRecursionDepth);

/*! Requests that ray queries be enabled for inline ray tracing support. */
GPRT_API void gprtRequestRayQueries();

GPRT_API void gprtRequestMaxAttributeSize(uint32_t attributeSize);

GPRT_API void gprtRequestMaxPayloadSize(uint32_t payloadSize);

/** creates a new device context with the gives list of devices.

  If requested device IDs list if null it implicitly refers to the
  list "0,1,2,...."; if numDevices <= 0 it automatically refers to
  "all devices you can find". Examples:

  - gprtContextCreate(nullptr,1) creates one device on the first GPU

  - gprtContextCreate(nullptr,0) creates a context across all GPUs in
  the system

  - int gpu=2;gprtContextCreate(&gpu,1) will create a context on GPU #2
  (where 2 refers to the vulkan device ordinal; from that point on, from
  gprt's standpoint (eg, during gprtBufferGetHostPointer() this GPU will
  from that point on be known as device #0 */
GPRT_API GPRTContext gprtContextCreate(int32_t *requestedDeviceIDs GPRT_IF_CPP(= nullptr),
                                       int numDevices GPRT_IF_CPP(= 1));

GPRT_API void gprtContextDestroy(GPRTContext context);

/**
 * @brief Creates a "compute" handle which describes a compute device program to call and the parameters to
 * pass into that compute device program. Compute programs handle data generation and transformation, but
 * can also trace rays so long as inline ray tracing is used.
 *
 * @param context The GPRT context
 * @param module The GPRT module containing device entrypoint definitions, made with @ref gprtModuleCreate.
 * @param entrypoint The name of the compute program to run, ie the first parameter following
 * GPRT_COMPUTE_PROGRAM in the device code.
 */
GPRT_API GPRTCompute gprtComputeCreate(GPRTContext context, GPRTModule module, const char *entrypoint);

/**
 * @brief Creates a "compute" handle which describes a compute device program to call and the parameters to
 * pass into that compute device program. Compute programs handle data generation and transformation, but
 * can also trace rays so long as inline ray tracing is used.
 *
 * @param context The GPRT context
 * @param module The GPRT module containing device entrypoint definitions, made with @ref gprtModuleCreate.
 * @param entrypoint The name of the compute program to run, ie the first parameter following
 * GPRT_COMPUTE_PROGRAM in the device code.
 */
template <typename... Uniforms>
GPRTComputeOf<Uniforms...>
gprtComputeCreate(GPRTContext context, GPRTModule module, std::string entrypoint) {
  return (GPRTComputeOf<Uniforms...>) gprtComputeCreate(context, module, entrypoint.c_str());
}

GPRT_API void gprtComputeDestroy(GPRTCompute compute);

template <typename... Uniforms>
void
gprtComputeDestroy(GPRTComputeOf<Uniforms...> compute) {
  gprtComputeDestroy((GPRTCompute) compute);
}

/**
 * @brief Creates a "raygen" handle which describes a ray generation device program to call and the parameters to
 * pass into that ray generation program. Ray generation programs can trace rays in hardware, and allow for shader
 * execution reordering for improved performance.
 *
 * @param context The GPRT context
 * @param module The GPRT module containing device entrypoint definitions, made with @ref gprtModuleCreate.
 * @param entrypoint The name of the raygen program to run, ie the first parameter following
 * GPRT_RAYGEN_PROGRAM in the device code.
 * @param recordSize The size of the parameters structure that will be passed into that raygen program
 */
GPRT_API GPRTRayGen gprtRayGenCreate(GPRTContext context, GPRTModule module, const char *entrypoint, size_t recordSize);

/**
 * @brief Creates a "raygen" handle which describes a ray generation device program to call and the parameters to
 * pass into that ray generation program. Ray generation programs can trace rays in hardware, and allow for shader
 * execution reordering for improved performance.
 *
 * @tparam T The type of the parameters structure stored in the record.
 * @param context The GPRT context
 * @param module The GPRT module containing device entrypoint definitions, made with @ref gprtModuleCreate.
 * @param entrypoint The name of the raygen program to run, ie the first parameter following
 * GPRT_RAYGEN_PROGRAM in the device code.
 */
template <typename T>
GPRTRayGenOf<T>
gprtRayGenCreate(GPRTContext context, GPRTModule module, const char *entrypoint) {
  return (GPRTRayGenOf<T>) gprtRayGenCreate(context, module, entrypoint, sizeof(T));
}

// Specialization for void
template <> GPRTRayGenOf<void> gprtRayGenCreate<void>(GPRTContext context, GPRTModule module, const char *entrypoint);

GPRT_API void gprtRayGenDestroy(GPRTRayGen rayGen);

template <typename T>
void
gprtRayGenDestroy(GPRTRayGenOf<T> rayGen) {
  gprtRayGenDestroy((GPRTRayGen) rayGen);
}

/**
 * @brief Returns a pointer to the parameters section of the record for this ray generation program, made
 * available on the device through the second parameter in GPRT_RAYGEN_PROGRAM. Note, call
 * @ref gprtBuildShaderBindingTable for these parameters to be uploaded to the device.
 *
 * @param rayGen The ray generation program who's record pointer is to be returned.
 * @returns a pointer to the parameters section of the record.
 */
GPRT_API void *gprtRayGenGetParameters(GPRTRayGen rayGen, int deviceID GPRT_IF_CPP(= 0));

/**
 * @brief Returns a pointer to the parameters section of the record for this ray generation program, made
 * available on the device through the second parameter in GPRT_RAYGEN_PROGRAM. Note, call
 * @ref gprtBuildShaderBindingTable for these parameters to be uploaded to the device.
 *
 * @tparam T The type of the parameters structure stored in the record
 * @param rayGen The ray generation program who's record pointer is to be returned.
 * @returns a pointer to the parameters section of the record.
 */
template <typename T>
T *
gprtRayGenGetParameters(GPRTRayGenOf<T> rayGen, int deviceID GPRT_IF_CPP(= 0)) {
  return (T *) gprtRayGenGetParameters((GPRTRayGen) rayGen, deviceID);
}

/**
 * @brief Copies the contents of the given parameters into the raygen record. Note, call
 * @ref gprtBuildShaderBindingTable for these parameters to be uploaded to the device.
 *
 * @param rayGen The ray generation program who's record to assign the parameters to.
 * @param parameters A pointer to a parameters structure. Assumed to be "recordSize" bytes,
 * as specified by @ref gprtRayGenCreate.
 */
GPRT_API void gprtRayGenSetParameters(GPRTRayGen rayGen, void *parameters, int deviceID GPRT_IF_CPP(= 0));

/**
 * @brief Copies the contents of the given parameters into the geometry record. Note, call
 * @ref gprtBuildShaderBindingTable for these parameters to be uploaded to the device.
 * @tparam T The type of the parameters structure stored in the record.
 * @param rayGen The ray generation program who's record to assign the parameters to.
 * @param parameters A pointer to a parameters structure. Assumed to be "recordSize" bytes,
 * as specified by @ref gprtRayGenCreate.
 */
template <typename T>
void
gprtRayGenSetParameters(GPRTRayGenOf<T> rayGen, T &parameters, int deviceID GPRT_IF_CPP(= 0)) {
  gprtRayGenSetParameters((GPRTRayGen) rayGen, (void *) &parameters, deviceID);
}

/**
 * @brief Creates a "miss" handle that describes a miss device program and the parameters to pass into that miss
 * program. Miss programs are called when traced rays do not hit any primitives. Which miss program is called currently
 * depends on order of miss creation, and which "miss" index is used when tracing a ray on the device.
 *
 * @param context The GPRT context
 * @param module The GPRT module containing device entrypoint definitions, made with @ref gprtModuleCreate.
 * @param entrypoint The name of the miss program to run, ie the first parameter following
 * GPRT_MISS_PROGRAM in the device code.
 * @param recordSize The size of the parameters structure that will be passed into that miss program
 */
GPRT_API GPRTMiss gprtMissCreate(GPRTContext context, GPRTModule module, const char *entrypoint, size_t recordSize);

/**
 * @brief Creates a "miss" handle that describes a miss device program and the parameters to pass into that miss
 * program. Miss programs are called when traced rays do not hit any primitives. Which miss program is called currently
 * depends on order of miss creation, and which "miss" index is used when tracing a ray on the device.
 *
 * @tparam T The type of the parameters structure stored in the record.
 * @param context The GPRT context
 * @param module The GPRT module containing device entrypoint definitions, made with @ref gprtModuleCreate.
 * @param entrypoint The name of the miss program to run, ie the first parameter following
 * GPRT_MISS_PROGRAM in the device code.
 */
template <typename T>
GPRTMissOf<T>
gprtMissCreate(GPRTContext context, GPRTModule module, const char *entrypoint) {
  return (GPRTMissOf<T>) gprtMissCreate(context, module, entrypoint, sizeof(T));
}

// Specialization for void
template <> GPRTMissOf<void> gprtMissCreate<void>(GPRTContext context, GPRTModule module, const char *entrypoint);

GPRT_API void gprtMissSet(GPRTContext context, int rayType, GPRTMiss missProgToUse);

template <typename T>
void
gprtMissSet(GPRTContext context, int rayType, GPRTMissOf<T> missProgToUse) {
  gprtMissSet(context, rayType, (GPRTMiss) missProgToUse);
}

GPRT_API void gprtMissDestroy(GPRTMiss missProg);

template <typename T>
void
gprtMissDestroy(GPRTMissOf<T> missProg) {
  gprtMissDestroy((GPRTMiss) missProg);
}
/**
 * @brief Returns a pointer to the parameters section of the record for this miss program, made
 * available on the device through the second parameter in GPRT_MISS_PROGRAM. Note, call
 * @ref gprtBuildShaderBindingTable for these parameters to be uploaded to the device.
 *
 * @param missProg The miss program who's record pointer is to be returned.
 * @returns a pointer to the parameters section of the record.
 */
GPRT_API void *gprtMissGetParameters(GPRTMiss missProg, int deviceID GPRT_IF_CPP(= 0));

/**
 * @brief Returns a pointer to the parameters section of the record for this miss program, made
 * available on the device through the second parameter in GPRT_MISS_PROGRAM. Note, call
 * @ref gprtBuildShaderBindingTable for these parameters to be uploaded to the device.
 *
 * @tparam T The type of the parameters structure stored in the record
 * @param missProg The miss program who's record pointer is to be returned.
 * @returns a pointer to the parameters section of the record.
 */
template <typename T>
T *
gprtMissGetParameters(GPRTMissOf<T> missProg, int deviceID GPRT_IF_CPP(= 0)) {
  return (T *) gprtMissGetParameters((GPRTMiss) missProg, deviceID);
}

/**
 * @brief Copies the contents of the given parameters into the miss record. Note, call
 * @ref gprtBuildShaderBindingTable for these parameters to be uploaded to the device.
 *
 * @param miss The miss program who's record to assign the parameters to.
 * @param parameters A pointer to a parameters structure. Assumed to be "recordSize" bytes,
 * as specified by @ref gprtMissCreate.
 */
GPRT_API void gprtMissSetParameters(GPRTMiss miss, void *parameters, int deviceID GPRT_IF_CPP(= 0));

/**
 * @brief Copies the contents of the given parameters into the geometry record. Note, call
 * @ref gprtBuildShaderBindingTable for these parameters to be uploaded to the device.
 *
 * @tparam T The type of the parameters structure stored in the record.
 * @param miss The miss program who's record to assign the parameters to.
 * @param parameters A pointer to a parameters structure. Assumed to be "recordSize" bytes,
 * as specified by @ref gprtMissCreate.
 */
template <typename T>
void
gprtMissSetParameters(GPRTMissOf<T> miss, T &parameters, int deviceID GPRT_IF_CPP(= 0)) {
  gprtMissSetParameters((GPRTMiss) miss, (void *) &parameters, deviceID);
}

GPRT_API uint32_t gprtMissGetIndex(GPRTMiss missProg, int deviceID GPRT_IF_CPP(= 0));

template <typename T>
uint32_t
gprtMissGetIndex(GPRTMissOf<T> missProg, int deviceID GPRT_IF_CPP(= 0)) {
  return gprtMissGetIndex((GPRTMiss) missProg, deviceID);
}

GPRT_API GPRTCallable gprtCallableCreate(GPRTContext context, GPRTModule module, const char *entrypoint,
                                         size_t recordSize);

template <typename T>
GPRTCallableOf<T>
gprtCallableCreate(GPRTContext context, GPRTModule module, const char *entrypoint) {
  return (GPRTCallableOf<T>) gprtCallableCreate(context, module, entrypoint, sizeof(T));
}

// Specialization for void
template <>
GPRTCallableOf<void> gprtCallableCreate<void>(GPRTContext context, GPRTModule module, const char *entrypoint);

GPRT_API void gprtCallableDestroy(GPRTCallable callableProg);

template <typename T>
void
gprtCallableDestroy(GPRTCallableOf<T> callableProg) {
  gprtCallableDestroy((GPRTCallable) callableProg);
}

GPRT_API void *gprtCallableGetParameters(GPRTCallable callableProg, int deviceID GPRT_IF_CPP(= 0));

template <typename T>
T *
gprtCallableGetParameters(GPRTCallableOf<T> callableProg, int deviceID GPRT_IF_CPP(= 0)) {
  return (T *) gprtCallableGetParameters((GPRTCallable) callableProg, deviceID);
}

GPRT_API void gprtCallableSetParameters(GPRTCallable callable, void *parameters, int deviceID GPRT_IF_CPP(= 0));

template <typename T>
void
gprtCallableSetParameters(GPRTCallableOf<T> callable, T &parameters, int deviceID GPRT_IF_CPP(= 0)) {
  gprtCallableSetParameters((GPRTCallable) callable, (void *) &parameters, deviceID);
}

// ------------------------------------------------------------------
/*! create a new acceleration structure for AABB geometries.

  \param numGeometries Number of geometries in this acceleration structure, must
  be non-zero.

  \param arrayOfChildGeoms An array of 'numGeometries' child
  geometries. Every geom in this array must be a valid gprt geometry
  created with gprtGeomCreate, and must be of a GPRT_GEOM_USER
  type.

  \param flags reserved for future use
*/
GPRT_API GPRTAccel gprtAABBAccelCreate(GPRTContext context, GPRTGeom *geom, unsigned int flags GPRT_IF_CPP(= 0));

template <typename T>
GPRTAccel
gprtAABBAccelCreate(GPRTContext context, GPRTGeomOf<T> &geom, unsigned int flags GPRT_IF_CPP(= 0)) {
  return gprtAABBAccelCreate(context, (GPRTGeom *) &geom, flags);
}

// ------------------------------------------------------------------
/*! create a new acceleration structure for triangle geometries.

  \param numGeometries Number of geometries in this acceleration structure, must
  be non-zero.

  \param arrayOfChildGeoms An array of 'numGeometries' child
  geometries. Every geom in this array must be a valid gprt geometry
  created with gprtGeomCreate, and must be of a GPRT_TRIANGLES
  type.

  \param flags reserved for future use
*/
GPRT_API GPRTAccel gprtTriangleAccelCreate(GPRTContext context, GPRTGeom *geom, unsigned int flags GPRT_IF_CPP(= 0));

template <typename T>
GPRTAccel
gprtTriangleAccelCreate(GPRTContext context, GPRTGeomOf<T> &geom, unsigned int flags GPRT_IF_CPP(= 0)) {
  return gprtTriangleAccelCreate(context, (GPRTGeom *) &geom, flags);
}

// ------------------------------------------------------------------
/*! create a new acceleration structure for sphere geometries.

  \param numGeometries Number of geometries in this acceleration structure, must
  be non-zero.

  \param arrayOfChildGeoms An array of 'numGeometries' child
  geometries. Every geom in this array must be a valid gprt geometry
  created with gprtGeomCreate, and must be of a GPRT_SPHERES
  type.

  \param flags reserved for future use
*/
GPRT_API GPRTAccel gprtSphereAccelCreate(GPRTContext context, GPRTGeom *geom, unsigned int flags GPRT_IF_CPP(= 0));

template <typename T>
GPRTAccel
gprtSphereAccelCreate(GPRTContext context, GPRTGeomOf<T> &geom, unsigned int flags GPRT_IF_CPP(= 0)) {
  return gprtSphereAccelCreate(context, (GPRTGeom *) &geom, flags);
}

// ------------------------------------------------------------------
/*! create a new acceleration structure for line swept sphere geometries.

  \param numGeometries Number of geometries in this acceleration structure, must
  be non-zero.

  \param arrayOfChildGeoms An array of 'numGeometries' child
  geometries. Every geom in this array must be a valid gprt geometry
  created with gprtGeomCreate, and must be of a GPRT_LSS
  type.

  \param flags reserved for future use
*/
GPRT_API GPRTAccel gprtLSSAccelCreate(GPRTContext context, GPRTGeom *geom,
                                      unsigned int flags GPRT_IF_CPP(= 0));

template <typename T>
GPRTAccel
gprtLSSAccelCreate(GPRTContext context, GPRTGeomOf<T> &geom, unsigned int flags GPRT_IF_CPP(= 0)) {
  return gprtLSSAccelCreate(context, (GPRTGeom *) &geom, flags);
}

// ------------------------------------------------------------------
/*! create a new acceleration structure for solid geometries.
  \param flags reserved for future use
*/
GPRT_API GPRTAccel gprtSolidAccelCreate(GPRTContext context, GPRTGeom *geom, unsigned int flags GPRT_IF_CPP(= 0));

template <typename T>
GPRTAccel
gprtSolidAccelCreate(GPRTContext context, GPRTGeomOf<T> &geom, unsigned int flags GPRT_IF_CPP(= 0)) {
  return gprtSolidAccelCreate(context, (GPRTGeom *) &geom, flags);
}

// ------------------------------------------------------------------
/*! create a new instance acceleration structure with given number of
  instances.

  \param numInstances Number of acceleration structures instantiated in the leaves
  of this acceleration structure, must be non-zero.

  \param instances A buffer of 'numInstances' instances.
*/
GPRT_API GPRTAccel gprtInstanceAccelCreate(GPRTContext context, uint numInstances,
                                           GPRTBufferOf<gprt::Instance> instances);

GPRT_API void gprtAccelDestroy(GPRTAccel accel);

/**
 * @brief Builds the given acceleration structure so that it can be used on the
 * device for ray tracing.
 *
 * @param context The GPRT context
 * @param accel The acceleration structure to build.
 * @param mode The build mode to use when constructing the acceleration structure.
 * 1. GPRT_BUILD_MODE_FAST_BUILD_NO_UPDATE
 *   Fastest possible build, but slower trace than 3 or 4. Good for fully-dynamic geometry like
 *   particles, destruction, changing prim counts, or moving wildly (explosions) where per-frame
 *   rebuild is required.
 *
 * 2. GPRT_BUILD_MODE_FAST_BUILD_AND_UPDATE
 *   Slightly slower build than 1, but allows very fast update. Good for lower level-of-detail
 *   dynamic objects that are unlikely to be hit by too many rays but still need to be refitted
 *   per frame to be correct.
 *
 * 3. GPRT_BUILD_MODE_FAST_TRACE_NO_UPDATE
 *   Fastest possible trace, but disallows updates. Slower to build than 1 or 2. This is a good
 *   default choice for static geometry.
 *
 * 4. GPRT_BUILD_MODE_FAST_TRACE_AND_UPDATE
 *   Fastest trace possible while still allowing for updates. Updates are slightly slower than 2.
 *   Trace is a bit slower than 3. Good for high level-of-detail dynamic objects that are expected
 *   to be hit by a significant number of rays.
 *
 * @param allowCompaction Enables the tree to be compacted with gprtAccelCompact, potentially
 * significantly reducing its memory footprint. Enabling this feature may take more time and
 * memory than a normal build, and so should only be used when the compaction feature is needed.
 *
 * @param minimizeMemory Sacrifices build and trace performance to reduce memory consumption. Enable only
 * when an application is under so much memory pressure that ray tracing isn't feasible without optimizing
 * for memory consumption as much as possible.
 */
GPRT_API void gprtAccelBuild(GPRTContext context, GPRTAccel accel, GPRTBuildMode mode, bool allowCompaction = false,
                             bool minimizeMemory = false);

/**
 * @brief Updates the structure of a tree to account for changes to the underlying primitives.
 * Updating is much faster than building, but degrades the accel's effectiveness as a spatial
 * data structure (eg, by updating tree bounding boxes while preserving original tree topology).
 *
 * As a general rule, only dynamic objects should be considered for update.
 * Lean towards rebuilding instance accels over updating, especially when instance transforms change
 * significantly from frame to frame.
 *
 * Example: grass waving in the wind -> update
 * Example: a mesh exploding -> don't update, instead rebuild.
 * Example: Skinned characters -> if modeled in t-pose, then every update will assume that feet are
 * close together. It might be better in this case to build multiple acceleration structures up-front,
 * then use the closest match as a source for refit.
 *
 * @param context The GPRT context
 * @param accel The acceleration structure to update.
 *
 * @note An accel must have been previously built with mode GPRT_BUILD_MODE_FAST_BUILD_AND_UPDATE or
 * GPRT_BUILD_MODE_FAST_TRACE_AND_UPDATE.
 */
GPRT_API void gprtAccelUpdate(GPRTContext context, GPRTAccel accel);

/**
 * @brief Compaction is a fast way to potentially reclaim a significant amount of memory from a tree.
 * There are no performance downsides when tracing rays against compacted acceleration structures.
 *
 * For static geometry, compaction is generally a good idea.
 *
 * For updatable geometry, it makes sense to compact accels that have a long lifetime (compaction and update
 * are not mutually exclusive!)
 *
 * For fully dynamic geometry that's rebuilt every frame (as opposed to updated), there's generally no
 * benefit to be had from compacting an accel.
 *
 * One reason to not use compaction might be to exploit the guarantee of accel storage requirements
 * increasing monotonically with primitive count - this does not hold true in the context of compaction.
 *
 * @param context The GPRT context
 * @param accel The acceleration structure to compact.
 *
 * @note An accel must have been previously built with "allowCompaction" set to "true".
 */
GPRT_API void gprtAccelCompact(GPRTContext context, GPRTAccel accel);

GPRT_API size_t gprtAccelGetSize(GPRTAccel _accel, int deviceID GPRT_IF_CPP(= 0));

GPRT_API gprt::Accel gprtAccelGetHandle(GPRTAccel accel, int deviceID GPRT_IF_CPP(= 0));

GPRT_API const void* gprtAccelGetDeviceAddress(GPRTAccel accel, int deviceID GPRT_IF_CPP(= 0));

/**
 * @brief Returns an instance of the given acceleration structure, which can then be used as an input
 * primitive for an instance acceleration structure.
 *
 * @param blas The bottom level acceleration structure to create an instance of.
 *
 * @note that @param accel must be built before an instance can be returned.
 *
 * If the blas is rebuilt, the user might need to create a new instance using this call.
 * Any returned instance objects do not need to be destroyed.
 * */
GPRT_API gprt::Instance gprtAccelGetInstance(GPRTAccel blas);

/**
 * @brief Creates a "geometry type", which describes the base primitive kind, device programs to call during
 * intersection, and the parameters to pass into these device programs.
 *
 * @param context The GPRT context
 * @param kind The base primitive kind, (triangles, AABBs, curves, etc)
 * @param recordSize The size of the parameters structure that will be passed into the device programs
 */
GPRT_API GPRTGeomType gprtGeomTypeCreate(GPRTContext context, GPRTGeomKind kind, size_t recordSize);

/**
 * @brief Creates a "geometry type", which describes the base primitive kind, device programs to call during
 * intersection, and the parameters to pass into these device programs.
 * @tparam T The template type of the parameters structure that will be passed into the device programs
 * @param context The GPRT context
 * @param kind The base primitive kind, (triangles, AABBs, curves, etc)
 */
template <typename T>
GPRTGeomTypeOf<T>
gprtGeomTypeCreate(GPRTContext context, GPRTGeomKind kind) {
  return (GPRTGeomTypeOf<T>) gprtGeomTypeCreate(context, kind, sizeof(T));
}

// Specialization for void
template <> GPRTGeomTypeOf<void> gprtGeomTypeCreate<void>(GPRTContext context, GPRTGeomKind kind);

GPRT_API void gprtGeomTypeDestroy(GPRTGeomType geomType);

template <typename T>
void
gprtGeomTypeDestroy(GPRTGeomTypeOf<T> geomType) {
  gprtGeomTypeDestroy((GPRTGeomType) geomType);
}

GPRT_API void gprtGeomTypeSetClosestHitProg(GPRTGeomType type, int rayType, GPRTModule module, const char *entrypoint);

template <typename T>
void
gprtGeomTypeSetClosestHitProg(GPRTGeomTypeOf<T> type, int rayType, GPRTModule module, const char *entrypoint) {
  gprtGeomTypeSetClosestHitProg((GPRTGeomType) type, rayType, module, entrypoint);
}

GPRT_API void gprtGeomTypeSetAnyHitProg(GPRTGeomType type, int rayType, GPRTModule module, const char *entrypoint);

template <typename T>
void
gprtGeomTypeSetAnyHitProg(GPRTGeomTypeOf<T> type, int rayType, GPRTModule module, const char *entrypoint) {
  gprtGeomTypeSetAnyHitProg((GPRTGeomType) type, rayType, module, entrypoint);
}

GPRT_API void gprtGeomTypeSetIntersectionProg(GPRTGeomType type, int rayType, GPRTModule module,
                                              const char *entrypoint);

template <typename T>
void
gprtGeomTypeSetIntersectionProg(GPRTGeomTypeOf<T> type, int rayType, GPRTModule module, const char *entrypoint) {
  gprtGeomTypeSetIntersectionProg((GPRTGeomType) type, rayType, module, entrypoint);
}

/**
 * @brief Creates a sampler object for use in sampling textures. Behavior below
 * defines how texture.SampleLevel and texture.SampleGrad operate. The "sampled
 * footprint" mentioned below refers to the texture coordinate differentials
 * given to Texture.SampleGrad.
 *
 * @param context The GPRT context used to make the sampler.
 * @param magFilter How to sample a texture when a texel's footprint covers more
 * than the sampled footprint. The default is GPRT_FILTER_LINEAR, which takes
 * the four closest texels and bilinearly interpolates among them.
 * GPRT_FILTER_NEAREST returns the nearest texel.
 * @param minFilter How the texture is sampled when a texel's footprint is
 * smaller than the sampled footprint. The default is GPRT_FILTER_LINEAR, which
 * uses mipmapping and a trilinear filter to filter the eight closest texels
 * between the two closest mip levels.
 * @param anisotropy The number of samples to take, between 1 and 16 and
 * typically a power of 2, along the axis of the sampled footprint that has the
 * highest density of texels. A higher value gives a less blurry result than a
 * basic mipmap, but at the cost of more texture samples being used.
 * @param addressMode How the image is wrapped both horizontally and vertically
 * when the sampled beyond the interval [0, 1). The default is
 * GPRT_SAMPLER_ADDRESS_MODE_REPEAT, which causes the texture to repeat when
 * sampled beyond the normal interval.
 * @param borderColor If address mode is set to GPRT_ADDRESS_MODE_BORDER, this
 * sets the border color to use.
 * @return GPRTSampler The sampler object that was made.
 */
GPRT_API GPRTSampler
gprtSamplerCreate(GPRTContext context, GPRTFilter magFilter GPRT_IF_CPP(= GPRT_FILTER_LINEAR),
                  GPRTFilter minFilter GPRT_IF_CPP(= GPRT_FILTER_LINEAR),
                  GPRTFilter mipFilter GPRT_IF_CPP(= GPRT_FILTER_LINEAR), uint32_t anisotropy GPRT_IF_CPP(= 1),
                  GPRTSamplerAddressMode addressMode GPRT_IF_CPP(= GPRT_SAMPLER_ADDRESS_MODE_REPEAT),
                  GPRTBorderColor borderColor GPRT_IF_CPP(= GPRT_BORDER_COLOR_OPAQUE_BLACK));

GPRT_API gprt::Sampler gprtSamplerGetHandle(GPRTSampler sampler);

GPRT_API void gprtSamplerDestroy(GPRTSampler);

GPRT_API GPRTTexture gprtHostTextureCreate(GPRTContext context, GPRTImageType type, GPRTFormat format, uint32_t width,
                                           uint32_t height, uint32_t depth, bool allocateMipmap,
                                           const void *init GPRT_IF_CPP(= nullptr));

template <typename T>
GPRTTextureOf<T>
gprtHostTextureCreate(GPRTContext context, GPRTImageType type, GPRTFormat format, uint32_t width, uint32_t height,
                      uint32_t depth, bool allocateMipmap, const T *init GPRT_IF_CPP(= nullptr)) {
  return (GPRTTextureOf<T>) gprtHostTextureCreate(context, type, format, width, height, depth, allocateMipmap,
                                                  (void *) init);
}

GPRT_API GPRTTexture gprtDeviceTextureCreate(GPRTContext context, GPRTImageType type, GPRTFormat format, uint32_t width,
                                             uint32_t height, uint32_t depth, bool allocateMipmap,
                                             const void *init GPRT_IF_CPP(= nullptr));

template <typename T>
GPRTTextureOf<T>
gprtDeviceTextureCreate(GPRTContext context, GPRTImageType type, GPRTFormat format, uint32_t width, uint32_t height,
                        uint32_t depth, bool allocateMipmap, const T *init GPRT_IF_CPP(= nullptr)) {
  return (GPRTTextureOf<T>) gprtDeviceTextureCreate(context, type, format, width, height, depth, allocateMipmap,
                                                    (void *) init);
}

GPRT_API GPRTTexture gprtSharedTextureCreate(GPRTContext context, GPRTImageType type, GPRTFormat format, uint32_t width,
                                             uint32_t height, uint32_t depth, bool allocateMipmap,
                                             const void *init GPRT_IF_CPP(= nullptr));

template <typename T>
GPRTTextureOf<T>
gprtSharedTextureCreate(GPRTContext context, GPRTImageType type, GPRTFormat format, uint32_t width, uint32_t height,
                        uint32_t depth, bool allocateMipmap, const T *init GPRT_IF_CPP(= nullptr)) {
  return (GPRTTextureOf<T>) gprtSharedTextureCreate(context, type, format, width, height, depth, allocateMipmap,
                                                    (void *) init);
}

GPRT_API void *gprtTextureGetPointer(GPRTTexture texture, int deviceID GPRT_IF_CPP(= 0));

template <typename T>
T *
gprtTextureGetPointer(GPRTTextureOf<T> texture, int deviceID GPRT_IF_CPP(= 0)) {
  return (T *) gprtTextureGetPointer((GPRTTexture) texture, deviceID);
}

GPRT_API void gprtTextureMap(GPRTTexture texture, int deviceID GPRT_IF_CPP(= 0));

template <typename T>
void
gprtTextureMap(GPRTTextureOf<T> texture, int deviceID GPRT_IF_CPP(= 0)) {
  gprtTextureMap((GPRTTexture) texture, deviceID);
}

GPRT_API void gprtTextureUnmap(GPRTTexture texture, int deviceID GPRT_IF_CPP(= 0));

template <typename T>
void
gprtTextureUnmap(GPRTTextureOf<T> texture, int deviceID GPRT_IF_CPP(= 0)) {
  gprtTextureUnmap((GPRTTexture) texture, deviceID);
}

GPRT_API gprt::Texture gprtTextureGetHandle(GPRTTexture texture, int deviceID GPRT_IF_CPP(= 0));

template <typename T>
gprt::Texture
gprtTextureGetHandle(GPRTTextureOf<T> texture, int deviceID GPRT_IF_CPP(= 0)) {
  return gprtTextureGetHandle((GPRTTexture) texture, deviceID);
}

// Returns the number of bytes between each row of texels in the image.
// Note, this might be larger than the width of the image.
GPRT_API size_t gprtTextureGetRowPitch(GPRTTexture texture);

// Returns the number of bytes between each slice of a 3D image.
// Note, this might be larger than the width*height*bytes of the image.
GPRT_API size_t gprtTextureGetDepthPitch(GPRTTexture texture);

// Generates mipmaps for the specified texture object.
GPRT_API void gprtTextureGenerateMipmap(GPRTTexture texture);

/** If a window was requested, this call presents the contents of the texture
 * to the window, potentially waiting for the screen to update before swapping.
 *
 * If a window was not requested (ie headless), this function does nothing.
 */
GPRT_API void gprtTexturePresent(GPRTContext context, GPRTTexture texture);

template <typename T>
void
gprtTexturePresent(GPRTContext context, GPRTTextureOf<T> texture) {
  gprtTexturePresent(context, (GPRTTexture) texture);
}

/**
 * @brief This call saves the contents of the given texture to the underlying filesystem.
 *
 * @param texture The GPRT texture to save
 * @param imageName The path to the PNG file to store the contents to
 */
GPRT_API void gprtTextureSaveImage(GPRTTexture texture, const char *imageName);

/**
 * @brief This call saves the contents of the given texture to the underlying filesystem.
 *
 * @tparam T The template type of the given texture
 * @param texture The GPRT texture to save
 * @param imageName The path to the PNG file to store the contents to
 */
template <typename T>
void
gprtTextureSaveImage(GPRTTextureOf<T> texture, const char *imageName) {
  gprtTextureSaveImage((GPRTTexture) texture, imageName);
}

/**
 * @brief Clears all values of the given texture to 0
 *
 * @param texture The texture to be cleared
 */
GPRT_API void gprtTextureClear(GPRTTexture texture);

/**
 * @brief Clears all values of the given texture to 0
 *
 * @tparam T The template type of the given texture
 * @param texture The texture to be cleared
 */
template <typename T>
void
gprtTextureClear(GPRTTextureOf<T> texture) {
  gprtTextureClear((GPRTTexture) texture);
}

/*! Destroys all underlying Vulkan resources for the given texture and frees any
  underlying memory*/
GPRT_API void gprtTextureDestroy(GPRTTexture texture);

template <typename T>
void
gprtTextureDestroy(GPRTTextureOf<T> texture) {
  gprtTextureDestroy((GPRTTexture) texture);
}

/**
 * @brief Creates a buffer using host memory.
 *
 * This function creates a buffer that utilizes memory located on the host, and that is accessible to all devices.
 * The buffer created by this function is optimized for high-speed access from the host.
 * It is suitable for operations where the buffer is primarily accessed and manipulated by
 * the CPU.
 *
 * @note The underlying host memory backing this buffer is pinned (ie page-locked, non-paged),
 * meaning that it stays resident in RAM and doesn't need to be copied from the disk or paged back
 * into RAM before the GPU can access it. Reads and writes from the GPU are done using the system's
 * Direct Memory Access (DMA) controller.
 *
 * @param context   The GPRTContext in which the buffer is to be created.
 * @param size      The size of each element in the buffer.
 * @param count     The number of elements in the buffer. Defaults to 1 (single element).
 * @param init      Optional pointer to initial data for buffer initialization. Defaults to nullptr.
 * @param alignment Byte alignment for the buffer, must be a power of two.
 *                  Larger alignment optimizes access but increase memory usage. Defaults to 16.
 * @return GPRTBuffer Returns a handle to the created buffer that resides in host memory.
 */
GPRT_API GPRTBuffer gprtHostBufferCreate(GPRTContext context, size_t size, size_t count = 1, const void *init = nullptr,
                                         size_t alignment = 16);

/**
 * @brief Creates a typed buffer using host memory.
 *
 * This templated function creates a buffer of type T that allows for stricter type safety. Like the non-templated
 * version, the created buffer is accessible to all devices and is pre-mapped to the host, but
 * with slower access speeds on devices compared to the host.
 *
 * @note The underlying host memory backing this buffer is pinned (ie page-locked, non-paged),
 * meaning that it stays resident in RAM and doesn't need to be copied from the disk or paged back
 * into RAM before the GPU can access it. Reads and writes from the GPU are done using the system's
 * Direct Memory Access (DMA) controller.
 *
 * @tparam T        The data type of elements in the buffer.
 * @param context   The GPRTContext in which the buffer is to be created.
 * @param count     The number of elements in the buffer. Defaults to 1 (single element).
 * @param init      Optional pointer to initial data for buffer initialization. Defaults to nullptr.
 * @param alignment Byte alignment for the buffer, must be a power of two.
 *                  Larger alignment optimizes access but increase memory usage. Defaults to 16.
 * @return GPRTBufferOf<T> Returns a handle to the created buffer that resides in host memory
 *                         and is typed according to the specified template parameter T.
 */
template <typename T>
GPRTBufferOf<T>
gprtHostBufferCreate(GPRTContext context, size_t count = 1, const T *init = nullptr, size_t alignment = 16) {
  return (GPRTBufferOf<T>) gprtHostBufferCreate(context, sizeof(T), count, init, alignment);
}

/**
 * @brief Creates a buffer using device memory.
 *
 * This function creates a buffer that utilizes memory located on the device (GPU).
 * The buffer created by this function is optimized for high-speed access from the device.
 * It is suitable for operations where the buffer is primarily accessed and manipulated by
 * the GPU. Reading from and writing to the buffer from the host requires mapping, which triggers
 * an underlying buffer copy to and from the host system.
 *
 * @param context   The GPRTContext in which the buffer is to be created.
 * @param size      The size of each element in the buffer.
 * @param count     The number of elements in the buffer. Defaults to 1 (single element).
 * @param init      Optional pointer to initial data for buffer initialization. Defaults to nullptr.
 * @param alignment Byte alignment for the buffer, must be a power of two.
 *                  Larger alignment optimizes access but increase memory usage. Defaults to 16.
 * @return GPRTBuffer Returns a handle to the created buffer that resides in device memory.
 */
GPRT_API GPRTBuffer gprtDeviceBufferCreate(GPRTContext context, size_t size, size_t count = 1,
                                           const void *init = nullptr, size_t alignment = 16);

/**
 * @brief Creates a typed buffer using device memory.
 *
 * This templated function creates a buffer of type T that utilizes memory located on the device (GPU).
 * It allows for the buffer to be easily used with specific data types. Like the non-templated
 * version, this buffer is optimized for high-speed GPU access, making it well-suited for operations
 * where the buffer is primarily accessed and manipulated by the device. Reading from and writing to the
 * buffer from the host requires mapping, which triggers an underlying buffer copy to and from the host system.
 *
 * @tparam T        The data type of elements in the buffer.
 * @param context   The GPRTContext in which the buffer is to be created.
 * @param count     The number of elements in the buffer. Defaults to 1 (single element).
 * @param init      Optional pointer to initial data for buffer initialization. Defaults to nullptr.
 * @param alignment Byte alignment for the buffer, must be a power of two.
 *                  Larger alignment optimizes access but increase memory usage. Defaults to 16.
 * @return GPRTBufferOf<T> Returns a handle to the created buffer that resides in device memory
 *                         and is typed according to the specified template parameter T.
 */
template <typename T>
GPRTBufferOf<T>
gprtDeviceBufferCreate(GPRTContext context, size_t count = 1, const T *init = nullptr, size_t alignment = 16) {
  return (GPRTBufferOf<T>) gprtDeviceBufferCreate(context, sizeof(T), count, init, alignment);
}

/**
 * @brief Creates a shared buffer using device memory accessible to all devices.
 *
 * This function creates a buffer in the device memory, accessible by both the host and other
 * devices. Access from the host might be slower compared to direct device access. The buffer's
 * size may be influenced by resizable BAR, allowing the CPU to access more of the device's memory.
 *
 * @note In the PCI configuration space, BARs are used to
 * hold memory addresses. These memory addresses are used by the CPU to access the memory of the PCI device.
 * Traditionally, these addresses were of a fixed size, limiting the amount of memory of the device that the
 * CPU could directly access at any given time (typically to 256MB windows). With "Resizable BAR," this
 * limitation is overcome by allowing these base address registers to be resized, enabling the CPU to
 * access more of the device's memory directly.
 *
 * @param context   The GPRTContext in which the buffer is to be created.
 * @param size      The size of each element in the buffer.
 * @param count     The number of elements in the buffer. Defaults to 1 (single element).
 * @param init      Optional pointer to initial data for buffer initialization. Defaults to nullptr.
 * @param alignment Byte alignment for the buffer, must be a power of two.
 *                  Larger alignment optimizes access but increase memory usage. Defaults to 16.
 * @return GPRTBuffer Returns a handle to the created buffer that uses shared device memory.
 */
GPRT_API GPRTBuffer gprtSharedBufferCreate(GPRTContext context, size_t size, size_t count = 1,
                                           const void *init = nullptr, size_t alignment = 16);

/**
 * @brief Creates a shared, typed buffer using device memory accessible to all devices.
 *
 * This templated function creates a buffer of type T in the device memory,
 * accessible by both the host and other devices. While the buffer can be accessed
 * by the host, such access might be slower compared to direct device access.
 * The buffer's size and efficiency may be influenced by resizable BAR technology,
 * enabling more direct CPU access to the device's memory.
 *
 * @note In the PCI configuration space, BARs are used to
 * hold memory addresses. These memory addresses are used by the CPU to access the memory of the PCI device.
 * Traditionally, these addresses were of a fixed size, limiting the amount of memory of the device that the
 * CPU could directly access at any given time (typically to 256MB windows). With "Resizable BAR," this
 * limitation is overcome by allowing these base address registers to be resized, enabling the CPU to
 * access more of the device's memory directly.
 *
 * @tparam T        The data type of elements in the buffer.
 * @param context   The GPRTContext in which the buffer is to be created.
 * @param count     The number of elements of type T in the buffer. Defaults to 1.
 * @param init      Optional pointer to an array of type T for initializing the buffer.
 *                  Defaults to nullptr.
 * @param alignment Byte alignment for the buffer, must be a power of two.
 *                  Larger alignment values may optimize device access at the cost of
 *                  increased memory usage, while smaller values are more memory-efficient
 *                  but might lead to reduced performance. Defaults to 16.
 * @return GPRTBufferOf<T> Returns a handle to the created buffer using shared device memory,
 *                         and is typed according to the specified template parameter T.
 */
template <typename T>
GPRTBufferOf<T>
gprtSharedBufferCreate(GPRTContext context, size_t count = 1, const T *init = nullptr, size_t alignment = 16) {
  return (GPRTBufferOf<T>) gprtSharedBufferCreate(context, sizeof(T), count, init, alignment);
}

/**
 * @brief Clears all values of the given buffer to 0
 *
 * @param buffer The buffer to be cleared
 */
GPRT_API void gprtBufferClear(GPRTBuffer buffer);

/**
 * @brief Clears all values of the given buffer to 0
 *
 * @tparam T The template type of the given buffer
 * @param buffer The buffer to be cleared
 */
template <typename T>
void
gprtBufferClear(GPRTBufferOf<T> buffer) {
  gprtBufferClear((GPRTBuffer) buffer);
}

/*! Destroys all underlying Vulkan resources for the given buffer and frees any
  underlying memory*/
GPRT_API void gprtBufferDestroy(GPRTBuffer buffer);

template <typename T>
void
gprtBufferDestroy(GPRTBufferOf<T> buffer) {
  gprtBufferDestroy((GPRTBuffer) buffer);
}

/***
 * @brief Returns the size of the given buffer
 *
 * @param buffer The buffer to measure
 * @return The size of the buffer in bytes
 */
GPRT_API size_t gprtBufferGetSize(GPRTBuffer buffer, int deviceID GPRT_IF_CPP(= 0));

/***
 * @brief Returns the size of the given buffer
 *
 * @tparam T The template type of the given buffer
 * @param buffer The buffer to measure
 * @return The size of the buffer in bytes
 */
template <typename T>
size_t
gprtBufferGetSize(GPRTBufferOf<T> buffer) {
  return gprtBufferGetSize((GPRTBuffer) buffer);
}

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
GPRT_API void *gprtBufferGetHostPointer(GPRTBuffer buffer, int deviceID GPRT_IF_CPP(= 0));

template <typename T>
T *
gprtBufferGetHostPointer(GPRTBufferOf<T> buffer, int deviceID GPRT_IF_CPP(= 0)) {
  return (T *) gprtBufferGetHostPointer((GPRTBuffer) buffer, deviceID);
}

GPRT_API void gprtBufferMap(GPRTBuffer buffer, int deviceID GPRT_IF_CPP(= 0));

template <typename T>
void
gprtBufferMap(GPRTBufferOf<T> buffer, int deviceID GPRT_IF_CPP(= 0)) {
  gprtBufferMap((GPRTBuffer) buffer, deviceID);
}

GPRT_API void gprtBufferUnmap(GPRTBuffer buffer, int deviceID GPRT_IF_CPP(= 0));

template <typename T>
void
gprtBufferUnmap(GPRTBufferOf<T> buffer, int deviceID GPRT_IF_CPP(= 0)) {
  gprtBufferUnmap((GPRTBuffer) buffer, deviceID);
}

GPRT_API void *gprtBufferGetDevicePointer(GPRTBuffer buffer, int deviceID GPRT_IF_CPP(= 0));

template <typename T>
T *
gprtBufferGetDevicePointer(GPRTBufferOf<T> buffer, int deviceID GPRT_IF_CPP(= 0)) {
  return (T *) gprtBufferGetDevicePointer((GPRTBuffer) buffer, deviceID);
}

/***
 * @brief Resizes the buffer so that it contains \p count elements, where each element is \p size bytes. If the buffer
 * is already the correct size, this function has no effect.
 *
 * @param context The GPRT context
 * @param buffer The buffer to resize
 * @param size The size of an individual element in the buffer
 * @param count The total number of elements contained in the buffer.
 * @param preserveContents If true, preserves the contents of the buffer when resized
 *
 * \warning This call will reset the buffer device address. Make sure to reassign the address to any parameter records
 * and rebuild the shader binding table. When preserving contents, a device copy is executed from an old buffer to a new
 * buffer. Host pinned buffers require allocating a new buffer before releasing the old to preserve prior values.
 */
GPRT_API void gprtBufferResize(GPRTContext context, GPRTBuffer buffer, size_t size, size_t count, bool preserveContents,
                               int deviceID GPRT_IF_CPP(= 0));

/***
 * @brief Resizes the buffer so that it contains \p count elements, where each element is "sizeof(T)" bytes. If the
 * buffer is already the correct size, this function has no effect.
 *
 * @tparam T The template type of the given buffer
 *
 * @param context The GPRT context
 * @param buffer The buffer to resize
 * @param count The total number of elements contained in the buffer.
 * @param preserveContents If true, preserves the contents of the buffer when resized
 *
 * \warning This call will reset the buffer device address. Make sure to reassign the address to any parameter records
 * and rebuild the shader binding table.  When preserving contents, a device copy is executed from an old buffer to a
 * new buffer. Host pinned buffers require allocating a new buffer before releasing the old to preserve prior values.
 */
template <typename T>
void
gprtBufferResize(GPRTContext context, GPRTBufferOf<T> buffer, size_t count, bool preserveContents,
                 int deviceID GPRT_IF_CPP(= 0)) {
  gprtBufferResize(context, (GPRTBuffer) buffer, sizeof(T), count, preserveContents, deviceID);
}

/***
 * @brief Copies the \p count elements of each \p size bytes from \p src into \p dst , reading from \p srcOffset and
 * writing to \p dstOffset .
 *
 * @param context The GPRT context
 * @param src The source buffer to copy elements from
 * @param dst The destination buffer to copy elements into
 * @param srcOffset The first element location to copy from
 * @param dstOffset The first element location to copy into
 * @param size The size of an individual element in the buffer
 * @param count The total number of elements to copy
 */
GPRT_API void gprtBufferCopy(GPRTContext context, GPRTBuffer src, GPRTBuffer dst, size_t srcOffset, size_t dstOffset,
                             size_t size, size_t count, int srcDeviceID GPRT_IF_CPP(= 0),
                             int dstDeviceID GPRT_IF_CPP(= 0));

/***
 * @brief Copies the \p count elements of each \p size bytes from \p src into \p dst , reading from \p srcOffset and
 * writing to \p dstOffset .
 *
 * @tparam T The template type of the given buffer
 *
 * @param context The GPRT context
 * @param src The source buffer to copy elements from
 * @param dst The destination buffer to copy elements into
 * @param srcOffset The first element location to copy from
 * @param dstOffset The first element location to copy into
 * @param count The total number of elements to copy
 */
template <typename T>
void
gprtBufferCopy(GPRTContext context, GPRTBufferOf<T> src, GPRTBufferOf<T> dst, uint32_t srcOffset, uint32_t dstOffset,
               uint32_t count, int srcDeviceID GPRT_IF_CPP(= 0), int dstDeviceID GPRT_IF_CPP(= 0)) {
  gprtBufferCopy(context, (GPRTBuffer) src, (GPRTBuffer) dst, srcOffset, dstOffset, sizeof(T), count, srcDeviceID,
                 dstDeviceID);
}

GPRT_API void gprtBufferTextureCopy(GPRTContext context, GPRTBuffer buffer, GPRTTexture texture, uint32_t bufferOffset,
                                    uint32_t bufferRowLength, uint32_t bufferImageHeight, uint32_t imageOffsetX,
                                    uint32_t imageOffsetY, uint32_t imageOffsetZ, uint32_t imageExtentX,
                                    uint32_t imageExtentY, uint32_t imageExtentZ, int srcDeviceID GPRT_IF_CPP(= 0),
                                    int dstDeviceID GPRT_IF_CPP(= 0));

template <typename T1, typename T2>
void
gprtBufferTextureCopy(GPRTContext context, GPRTBufferOf<T1> buffer, GPRTTextureOf<T2> texture, uint32_t bufferOffset,
                      uint32_t bufferRowLength, uint32_t bufferImageHeight, uint32_t imageOffsetX,
                      uint32_t imageOffsetY, uint32_t imageOffsetZ, uint32_t imageExtentX, uint32_t imageExtentY,
                      uint32_t imageExtentZ, int srcDeviceID GPRT_IF_CPP(= 0), int dstDeviceID GPRT_IF_CPP(= 0)) {
  gprtBufferTextureCopy(context, (GPRTBuffer) buffer, (GPRTTexture) texture, bufferOffset, bufferRowLength,
                        bufferImageHeight, imageOffsetX, imageOffsetY, imageOffsetZ, imageExtentX, imageExtentY,
                        imageExtentZ, srcDeviceID, dstDeviceID);
}

/**
 * @brief Computes a device-wide exclusive prefix sum, and returns the total amount.
 * Important note, currently the maximum aggregate sum supported is 2^30.
 * This has to do with maintaining coherency between thread blocks and being limited to 32 bit atomics
 *
 * Exclusive sum requires a temporary "scratch" space
 *
 * Input and output buffers *can* reference the same buffer object.
 *
 * @param context The GPRT context
 * @param input An input buffer of 32-bit unsigned integers to be summed
 * @param output An output buffer of 32-bit unsigned integers that will store the exclusive sum.
 * @param scratch A scratch buffer to facilitate the exclusive sum. If null, scratch memory will be allocated and
 * released internally. If a buffer is given, then if that buffer is undersized, the buffer will be allocated / resized
 * and returned by reference. Otherwise, the scratch buffer will be used directly without any device side allocations.
 * Generally, requires 2*log_1024(N) ints of scratch memory.
 */
GPRT_API uint32_t gprtBufferExclusiveSum(GPRTContext context, GPRTBuffer input, GPRTBuffer output,
                                         GPRTBuffer scratch GPRT_IF_CPP(= 0));

/**
 * @brief Computes a device-wide exclusive prefix sum, and returns the total amount.
 * Important note, currently the maximum aggregate sum supported is 2^30.
 * This has to do with maintaining coherency between thread blocks and being limited to 32 bit atomics
 *
 * Exclusive sum requires a temporary "scratch" space
 *
 * Input and output buffers *can* reference the same buffer object.
 *
 * @tparam T1 The template type of the given buffer (currently only uint32_t is supported)
 * @tparam T2 The template type of the scratch buffer (uint8_t is assumed)
 *
 * @param context The GPRT context
 * @param input An input buffer of 32-bit unsigned integers to be summed
 * @param output An output buffer of 32-bit unsigned integers that will store the exclusive sum.
 * @param scratch A scratch buffer to facilitate the exclusive sum. If null, scratch memory will be allocated and
 * released internally. If a buffer is given, then if that buffer is undersized, the buffer will be allocated / resized
 * and returned by reference. Otherwise, the scratch buffer will be used directly without any device side allocations.
 */
template <typename T1, typename T2>
uint32_t
gprtBufferExclusiveSum(GPRTContext context, GPRTBufferOf<T1> input, GPRTBufferOf<T1> output,
                       GPRTBufferOf<T2> scratch GPRT_IF_CPP(= 0)) {
  return gprtBufferExclusiveSum(context, (GPRTBuffer) input, (GPRTBuffer) output, (GPRTBuffer) scratch);
}

/**
 * @brief Constructs a partitioned output sequence from 32-bit integers based on their sign and a selection. The total
 * number of selections is then returned to the user.
 *
 * Copies of selected integers are compacted into the beginning of "output" and maintain their original relative
 * ordering, while copies of the unselected items are compacted into the rear of "output" in reverse order.
 *
 * Partition requires a temporary "scratch" space
 *
 * Input and output buffers *cannot* reference the same buffer object.
 *
 * @param context The GPRT context
 * @param input An input buffer containing a sequence of 32-bit integers to be reordered
 * @param selectPositive If true, places positive integers to the left and negative integers to the right.
 *                       If false, places negative integers to the left and positive integers to the right.
 * @param output An output buffer containing the partitioned sequences by their sign.
 * @param scratch A scratch buffer to facilitate the partitioning. If null, scratch memory will be allocated and
 * released internally. If a buffer is given, then if that buffer is undersized, the buffer will be allocated / resized
 * and returned by reference. Otherwise, the scratch buffer will be used directly without any device side allocations.
 */
GPRT_API uint32_t gprtBufferPartition(GPRTContext context, GPRTBuffer input, bool selectPositive, GPRTBuffer output,
                                      GPRTBuffer scratch GPRT_IF_CPP(= 0));

/**
 * @brief Constructs a partitioned output sequence from 32-bit integers based on their sign and a selection. The total
 * number of selections is then returned to the user.
 *
 * Copies of selected integers are compacted into the beginning of "output" and maintain their original relative
 * ordering, while copies of the unselected items are compacted into the rear of "output" in reverse order.
 *
 * Partition requires a temporary "scratch" space
 *
 * Input and output buffers *cannot* reference the same buffer object.
 *
 * @param context The GPRT context
 * @param input An input buffer containing a sequence of 32-bit integers to be reordered
 * @param selectPositive If true, places positive integers to the left and negative integers to the right.
 *                       If false, places negative integers to the left and positive integers to the right.
 * @param output An output buffer containing the partitioned sequences by their sign.
 * @param scratch A scratch buffer to facilitate the partitioning. If null, scratch memory will be allocated and
 * released internally. If a buffer is given, then if that buffer is undersized, the buffer will be allocated / resized
 * and returned by reference. Otherwise, the scratch buffer will be used directly without any device side allocations.
 */
template <typename T1, typename T2>
uint32_t
gprtBufferPartition(GPRTContext context, GPRTBufferOf<T1> input, bool selectPositive, GPRTBufferOf<T1> output,
                    GPRTBufferOf<T2> scratch GPRT_IF_CPP(= 0)) {
  return gprtBufferPartition(context, (GPRTBuffer) input, selectPositive, (GPRTBuffer) output, (GPRTBuffer) scratch);
}

/**
 * @brief Compacts a sequence of 32-bit integers based on their sign and a selection. The total number of
 * selections is then returned to the user.
 *
 * Copies of selected integers are compacted into the beginning of "output" and maintain their original relative
 * ordering.
 *
 * Select requires a temporary "scratch" space
 *
 * Input and output buffers *cannot* reference the same buffer object.
 *
 * @param context The GPRT context
 * @param input An input buffer containing a sequence of 32-bit integers to be reordered
 * @param selectPositive If true, keeps positive integers. If false, keeps negative integers.
 * @param output An output buffer containing the selected items
 * @param scratch A scratch buffer to facilitate the selection. If null, scratch memory will be allocated and released
 * internally. If a buffer is given, then if that buffer is undersized, the buffer will be allocated / resized and
 * returned by reference. Otherwise, the scratch buffer will be used directly without any device side allocations.
 */
GPRT_API uint32_t gprtBufferSelect(GPRTContext context, GPRTBuffer input, bool selectPositive, GPRTBuffer output,
                                   GPRTBuffer scratch GPRT_IF_CPP(= 0));

/**
 * @brief Compacts a sequence of 32-bit integers based on their sign and a selection. The total number of
 * selections is then returned to the user.
 *
 * Copies of selected integers are compacted into the beginning of "output" and maintain their original relative
 * ordering.
 *
 * Select requires a temporary "scratch" space
 *
 * Input and output buffers *cannot* reference the same buffer object.
 *
 * @param context The GPRT context
 * @param input An input buffer containing a sequence of 32-bit integers to be reordered
 * @param selectPositive If true, keeps positive integers. If false, keeps negative integers.
 * @param output An output buffer containing the selected items
 * @param scratch A scratch buffer to facilitate the selection. If null, scratch memory will be allocated and released
 * internally. If a buffer is given, then if that buffer is undersized, the buffer will be allocated / resized and
 * returned by reference. Otherwise, the scratch buffer will be used directly without any device side allocations.
 */
template <typename T1, typename T2>
uint32_t
gprtBufferSelect(GPRTContext context, GPRTBufferOf<T1> input, bool selectPositive, GPRTBufferOf<T1> output,
                 GPRTBufferOf<T2> scratch GPRT_IF_CPP(= 0)) {
  return gprtBufferSelect(context, (GPRTBuffer) input, selectPositive, (GPRTBuffer) output, (GPRTBuffer) scratch);
}

/**
 * @brief Sorts the input buffer using a GPU-parallel radix sorter.
 * Radix sort requires a temporary "scratch" space
 *
 *
 * @param context The GPRT context
 * @param buffer A buffer of 64-bit unsigned integers
 * @param scratch A scratch buffer to facilitate the sort. If null, scratch memory will be allocated and released
 * internally. If a buffer is given, then if that buffer is undersized, the buffer will be allocated / resized and
 * returned by reference. Otherwise, the scratch buffer will be used directly without any device side allocations.
 */
GPRT_API void gprtBufferSort(GPRTContext context, GPRTBuffer buffer, GPRTBuffer scratch GPRT_IF_CPP(= 0));

/**
 * @brief Sorts the input buffer using a GPU-parallel radix sorter.
 * Radix sort requires a temporary "scratch" space
 *
 * @tparam T1 The template type of the given buffer (currently only uint32_t is supported)
 * @tparam T2 The template type of the scratch buffer (uint8_t is assumed)
 *
 * @param context The GPRT context
 * @param buffer A buffer of 64-bit unsigned integers
 * @param scratch A scratch buffer to facilitate the sort. If null, scratch memory will be allocated and released
 * internally. If a buffer is given, then if that buffer is undersized, the buffer will be allocated / resized and
 * returned by reference. Otherwise, the scratch buffer will be used directly without any device side allocations.
 */
template <typename T1, typename T2>
void
gprtBufferSort(GPRTContext context, GPRTBufferOf<T1> buffer, GPRTBufferOf<T2> scratch GPRT_IF_CPP(= 0)) {
  gprtBufferSort(context, (GPRTBuffer) buffer, (GPRTBuffer) scratch);
}

/**
 * @brief Sorts the input key-value pairs by key using a GPU-parallel radix sorter.
 * Radix sort requires a temporary "scratch" space
 *
 * @param context The GPRT context
 * @param keys A buffer of 64-bit unsigned integer keys
 * @param values A buffer of 64-bit values
 * @param scratch A scratch buffer to facilitate the sort. If null, scratch memory will be allocated and released
 * internally. If a buffer is given, then if that buffer is undersized, the buffer will be allocated / resized and
 * returned by reference. Otherwise, the scratch buffer will be used directly without any device side allocations.
 */
GPRT_API void gprtBufferSortPayload(GPRTContext context, GPRTBuffer keys, GPRTBuffer values,
                                    GPRTBuffer scratch GPRT_IF_CPP(= 0));

/**
 * @brief Sorts the input buffer using a GPU-parallel radix sorter.
 * Radix sort requires a temporary "scratch" space
 *
 * @tparam T1 The template type of the given buffer (currently only uint32_t is supported)
 * @tparam T2 The template type of the given buffer (currently only uint32_t is supported)
 * @tparam T3 The template type of the scratch buffer (uint8_t is assumed)
 *
 * @param context The GPRT context
 * @param keys A buffer of 32-bit unsigned integer keys
 * @param values A buffer of 32-bit values
 * @param scratch A scratch buffer to facilitate the sort. If null, scratch memory will be allocated and released
 * internally. If a buffer is given, then if that buffer is undersized, the buffer will be allocated / resized and
 * returned by reference. Otherwise, the scratch buffer will be used directly without any device side allocations.
 */
template <typename T1, typename T2, typename T3>
void
gprtBufferSortPayload(GPRTContext context, GPRTBufferOf<T1> keys, GPRTBufferOf<T2> values,
                      GPRTBufferOf<T3> scratch GPRT_IF_CPP(= 0)) {
  gprtBufferSortPayload(context, (GPRTBuffer) keys, (GPRTBuffer) values, (GPRTBuffer) scratch);
}

// GPRT_API gprt::Buffer gprtBufferGetHandle(GPRTBuffer buffer, int deviceID GPRT_IF_CPP(= 0));

// template <typename T>
// gprt::Buffer
// gprtBufferGetHandle(GPRTBufferOf<T> buffer, int deviceID GPRT_IF_CPP(= 0)) {
//   return gprtBufferGetHandle((GPRTBuffer) buffer, deviceID);
// }

// GPRT_API uint64_t gprtBufferGetDeviceAddress(GPRTBuffer buffer, int deviceID GPRT_IF_CPP(= 0));
// template <typename T>
// uint64_t
// gprtBufferGetDeviceAddress(GPRTBufferOf<T> buffer, int deviceID GPRT_IF_CPP(= 0)) {
//   return gprtBufferGetDeviceAddress((GPRTBuffer) buffer, deviceID);
// }

/** If a window was requested, this call interprets the given buffer as
 * a B8G8R8A8 SRGB image sorted in row major buffer, and presents the contents
 * to the window, potentially waiting for the screen to update before swapping.
 *
 * If a window was not requested (ie headless), this function does nothing.
 */
GPRT_API void gprtBufferPresent(GPRTContext context, GPRTBuffer buffer);

template <typename T>
void
gprtBufferPresent(GPRTContext context, GPRTBufferOf<T> buffer) {
  gprtBufferPresent(context, (GPRTBuffer) buffer);
}

/** This call interprets the given buffer as a B8G8R8A8 SRGB image sorted in
 * row major buffer, and saves the contents to the underlying filesystem.
 */
GPRT_API void gprtBufferSaveImage(GPRTBuffer buffer, uint32_t width, uint32_t height, const char *imageName);

template <typename T>
void
gprtBufferSaveImage(GPRTBufferOf<T> buffer, uint32_t width, uint32_t height, const char *imageName) {
  gprtBufferSaveImage((GPRTBuffer) buffer, width, height, imageName);
}

GPRT_API void gprtRayGenLaunch1D(GPRTContext context, GPRTRayGen rayGen, uint32_t dims_x,
                                 size_t pushConstantsSize GPRT_IF_CPP(= 0), void *pushConstants GPRT_IF_CPP(= 0));

template <typename RecordType>
void
gprtRayGenLaunch1D(GPRTContext context, GPRTRayGenOf<RecordType> rayGen, uint32_t dims_x) {
  gprtRayGenLaunch1D(context, (GPRTRayGen) rayGen, dims_x);
}

template <typename RecordType, typename PushConstantsType>
void
gprtRayGenLaunch1D(GPRTContext context, GPRTRayGenOf<RecordType> rayGen, uint32_t dims_x,
                   PushConstantsType pushConstants) {
  static_assert(sizeof(PushConstantsType) <= 128, "Current GPRT push constant size limited to 128 bytes or less");
  gprtRayGenLaunch1D(context, (GPRTRayGen) rayGen, dims_x, sizeof(PushConstantsType), &pushConstants);
}

/*! Executes a ray tracing pipeline with the given raygen program.
  This call will block until the raygen program returns. */
GPRT_API void gprtRayGenLaunch2D(GPRTContext context, GPRTRayGen rayGen, uint32_t dims_x, uint32_t dims_y,
                                 size_t pushConstantsSize GPRT_IF_CPP(= 0), void *pushConstants GPRT_IF_CPP(= 0));

template <typename RecordType>
void
gprtRayGenLaunch2D(GPRTContext context, GPRTRayGenOf<RecordType> rayGen, uint32_t dims_x, uint32_t dims_y) {
  gprtRayGenLaunch2D(context, (GPRTRayGen) rayGen, dims_x, dims_y);
}

template <typename RecordType, typename PushConstantsType>
void
gprtRayGenLaunch2D(GPRTContext context, GPRTRayGenOf<RecordType> rayGen, uint32_t dims_x, uint32_t dims_y,
                   PushConstantsType pushConstants) {
  static_assert(sizeof(PushConstantsType) <= 128, "Current GPRT push constant size limited to 128 bytes or less");
  gprtRayGenLaunch2D(context, (GPRTRayGen) rayGen, dims_x, dims_y, sizeof(PushConstantsType), &pushConstants);
}

/*! 3D-launch variant of \see gprtRayGenLaunch2D */
GPRT_API void gprtRayGenLaunch3D(GPRTContext context, GPRTRayGen rayGen, uint32_t dims_x, uint32_t dims_y,
                                 uint32_t dims_z, size_t pushConstantsSize GPRT_IF_CPP(= 0),
                                 void *pushConstants GPRT_IF_CPP(= 0));

template <typename RecordType>
void
gprtRayGenLaunch3D(GPRTContext context, GPRTRayGenOf<RecordType> rayGen, uint32_t dims_x, uint32_t dims_y,
                   uint32_t dims_z) {
  gprtRayGenLaunch3D(context, (GPRTRayGen) rayGen, dims_x, dims_y, dims_z);
}

template <typename RecordType, typename PushConstantsType>
void
gprtRayGenLaunch3D(GPRTContext context, GPRTRayGenOf<RecordType> rayGen, uint32_t dims_x, uint32_t dims_y,
                   uint32_t dims_z, PushConstantsType pushConstants) {
  static_assert(sizeof(PushConstantsType) <= PUSH_CONSTANTS_LIMIT,
                "Push constants type size exceeds PUSH_CONSTANTS_LIMIT bytes");
  gprtRayGenLaunch3D(context, (GPRTRayGen) rayGen, dims_x, dims_y, dims_z, sizeof(PushConstantsType), &pushConstants);
}

// declaration for internal implementation
void _gprtComputeLaunch(GPRTCompute compute, uint3 numGroups, uint3 groupSize,
                        std::array<char, PUSH_CONSTANTS_LIMIT> pushConstants);

// Case where compute program uniforms are not known at compilation time
template <typename... Uniforms>
void
gprtComputeLaunch(GPRTCompute compute, uint3 numGroups, uint3 groupSize, Uniforms... uniforms) {
  static_assert(totalSizeOf<Uniforms...>() <= PUSH_CONSTANTS_LIMIT,
                "Total size of arguments exceeds PUSH_CONSTANTS_LIMIT bytes");

  runtime_assert(numGroups[0] <= WORKGROUP_LIMIT && numGroups[1] <= WORKGROUP_LIMIT && numGroups[2] <= WORKGROUP_LIMIT,
                 "Workgroup count exceeds WORKGROUP_LIMIT. Increase numthreads and reduce the number of workgroups.");

  runtime_assert(
      groupSize[0] * groupSize[1] * groupSize[2] <= THREADGROUP_LIMIT,
      "Thread count per group exceeds THREADGROUP_LIMIT. Reduce thread count and increase workgroup number.");

  std::array<char, PUSH_CONSTANTS_LIMIT> pushConstants{};   // Initialize with zero
  size_t offset = 0;

  // Serialize each argument into the buffer
  (handleArg(pushConstants, offset, uniforms), ...);

  _gprtComputeLaunch(compute, numGroups, groupSize, pushConstants);
}

// Case where compute program has compile-time type-safe uniform arguments
template <typename... Uniforms>
void
gprtComputeLaunch(GPRTComputeOf<Uniforms...> compute, uint3 numGroups, uint3 groupSize, Uniforms... uniforms) {
  static_assert(totalSizeOf<Uniforms...>() <= PUSH_CONSTANTS_LIMIT,
                "Total size of arguments exceeds PUSH_CONSTANTS_LIMIT bytes");

  runtime_assert(numGroups[0] <= WORKGROUP_LIMIT && numGroups[1] <= WORKGROUP_LIMIT && numGroups[2] <= WORKGROUP_LIMIT,
                 "Workgroup count exceeds WORKGROUP_LIMIT. Increase numthreads and reduce the number of workgroups.");

  runtime_assert(
      groupSize[0] * groupSize[1] * groupSize[2] <= THREADGROUP_LIMIT,
      "Thread count per group exceeds THREADGROUP_LIMIT. Reduce thread count and increase workgroup number.");

  std::array<char, PUSH_CONSTANTS_LIMIT> pushConstants{};   // Initialize with zero
  size_t offset = 0;

  // Serialize each argument into the buffer
  (handleArg(pushConstants, offset, uniforms), ...);

  _gprtComputeLaunch((GPRTCompute) compute, numGroups, groupSize, pushConstants);
}

GPRT_API void gprtBeginProfile(GPRTContext context);

// returned results are in milliseconds
GPRT_API float gprtEndProfile(GPRTContext context);
