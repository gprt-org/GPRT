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

#include <cstdalign>
#include <vulkan/vulkan.h>

#include "linalg.h"
using namespace linalg;
using namespace linalg::detail;
using namespace linalg::aliases;
using namespace linalg::ostream_overloads;

#include <stdint.h>
#include <sys/types.h>

#include <assert.h>
#include <map>
#include <string>
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

#define GPRT_OFFSETOF(type, member) (uint32_t)((char *) (&((struct type *) 0)->member) - (char *) (((struct type *) 0)))

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
using GPRTCompute = struct _GPRTCompute *;

template <typename T> struct _GPRTBufferOf;
template <typename T> struct _GPRTTextureOf;
template <typename T> struct _GPRTRayGenOf;
template <typename T> struct _GPRTMissOf;
template <typename T> struct _GPRTComputeOf;
template <typename T> struct _GPRTGeomOf;
template <typename T> struct _GPRTGeomTypeOf;
template <typename T> using GPRTRayGenOf = struct _GPRTRayGenOf<T> *;
template <typename T> using GPRTBufferOf = struct _GPRTBufferOf<T> *;
template <typename T> using GPRTTextureOf = struct _GPRTTextureOf<T> *;
template <typename T> using GPRTMissOf = struct _GPRTMissOf<T> *;
template <typename T> using GPRTComputeOf = struct _GPRTComputeOf<T> *;
template <typename T> using GPRTGeomOf = struct _GPRTGeomOf<T> *;
template <typename T> using GPRTGeomTypeOf = struct _GPRTGeomTypeOf<T> *;

using GPRTProgram = std::map<std::string, std::vector<uint8_t>>;

namespace gprt {
struct Buffer {
  uint64_t x;
  uint64_t y;
};

struct Accel {
  uint64_t x;
  uint64_t y;
};

struct Texture {
  uint64_t x;
  uint64_t y;
};

struct Sampler {
  uint64_t x;
  uint64_t y;
};
}   // namespace gprt

/*! launch params (or "globals") are variables that can be put into
  device constant memory, accessible through Vulkan's push constants */
typedef struct _GPRTLaunchParams *GPRTLaunchParams, *GPRTParams, *GPRTGlobals;

typedef enum {
  GPRT_SBT_HITGROUP = 1,
  GPRT_SBT_GEOM = GPRT_SBT_HITGROUP,
  GPRT_SBT_RAYGEN = 2,
  GPRT_SBT_MISS = 4,
  GPRT_SBT_COMPUTE = 8,
  GPRT_SBT_RASTER = 16,
  GPRT_SBT_ALL = 31
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
  GPRT_UNKNOWN,
  GPRT_AABBS,
  GPRT_TRIANGLES,
  //  GPRT_CURVES
} GPRTGeomKind;

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
gprtGeomSetParameters(GPRTGeomOf<T> geometry, T *parameters, int deviceID GPRT_IF_CPP(= 0)) {
  gprtGeomSetParameters((GPRTGeom) geometry, (void *) parameters, deviceID);
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
                        uint32_t stride GPRT_IF_CPP(= sizeof(float3)), uint32_t offset GPRT_IF_CPP(= 0)) {
  gprtTrianglesSetIndices((GPRTGeom) triangles, (GPRTBuffer) indices, count, stride, offset);
}

/*! set the aabb positions (minX, minY, minZ, maxX, maxY, maxZ)
  for the given AABB geometry. This _has_ to be set before the accel(s)
  that this geom is used in get built. */
GPRT_API void gprtAABBsSetPositions(GPRTGeom aabbs, GPRTBuffer positions, size_t count,
                                    size_t stride GPRT_IF_CPP(= 2 * sizeof(float3)), size_t offset GPRT_IF_CPP(= 0));

template <typename T1, typename T2>
void
gprtAABBsSetPositions(GPRTGeomOf<T1> aabbs, GPRTBufferOf<T2> positions, size_t count,
                      size_t stride GPRT_IF_CPP(= 2 * sizeof(float3)), size_t offset GPRT_IF_CPP(= 0)) {
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

/*! set number of ray types to be used; this should be
  done before any programs, pipelines, geometries, etc get
  created */
GPRT_API void gprtRequestRayTypeCount(uint32_t numRayTypes);

/*! Requests that ray queries be enabled for inline ray tracing support. */
GPRT_API void gprtRequestRayQueries();

/** creates a new device context with the gives list of devices.

  If requested device IDs list if null it implicitly refers to the
  list "0,1,2,...."; if numDevices <= 0 it automatically refers to
  "all devices you can find". Examples:

  - gprtContextCreate(nullptr,1) creates one device on the first GPU

  - gprtContextCreate(nullptr,0) creates a context across all GPUs in
  the system

  - int gpu=2;gprtContextCreate(&gpu,1) will create a context on GPU #2
  (where 2 refers to the vulkan device ordinal; from that point on, from
  gprt's standpoint (eg, during gprtBufferGetPointer() this GPU will
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
 * @param recordSize The size of the parameters structure that will be passed into that compute program
 */
GPRT_API GPRTCompute gprtComputeCreate(GPRTContext context, GPRTModule module, const char *entrypoint,
                                       size_t recordSize);

/**
 * @brief Creates a "compute" handle which describes a compute device program to call and the parameters to
 * pass into that compute device program. Compute programs handle data generation and transformation, but
 * can also trace rays so long as inline ray tracing is used.
 *
 * @tparam T The type of the parameters structure stored in the record.
 * @param context The GPRT context
 * @param module The GPRT module containing device entrypoint definitions, made with @ref gprtModuleCreate.
 * @param entrypoint The name of the compute program to run, ie the first parameter following
 * GPRT_COMPUTE_PROGRAM in the device code.
 */
template <typename T>
GPRTComputeOf<T>
gprtComputeCreate(GPRTContext context, GPRTModule module, const char *entrypoint) {
  return (GPRTComputeOf<T>) gprtComputeCreate(context, module, entrypoint, sizeof(T));
}

GPRT_API void gprtComputeDestroy(GPRTCompute compute);

template <typename T>
void
gprtComputeDestroy(GPRTComputeOf<T> compute) {
  gprtComputeDestroy((GPRTCompute) compute);
}

/**
 * @brief Returns a pointer to the parameters section of the record for this compute program, made
 * available on the device through the second parameter in GPRT_COMPUTE_PROGRAM. Note, call
 * @ref gprtBuildShaderBindingTable for these parameters to be uploaded to the device.
 *
 * @param compute The compute program who's record pointer is to be returned.
 * @returns a pointer to the parameters section of the record.
 */
GPRT_API void *gprtComputeGetParameters(GPRTCompute compute, int deviceID GPRT_IF_CPP(= 0));

/**
 * @brief Returns a pointer to the parameters section of the record for this compute program, made
 * available on the device through the second parameter in GPRT_COMPUTE_PROGRAM. Note, call
 * @ref gprtBuildShaderBindingTable for these parameters to be uploaded to the device.
 *
 * @tparam T The type of the parameters structure stored in the record
 * @param compute The compute program who's record pointer is to be returned.
 * @returns a pointer to the parameters section of the record.
 */
template <typename T>
T *
gprtComputeGetParameters(GPRTComputeOf<T> compute) {
  return (T *) gprtComputeGetParameters((GPRTCompute) compute);
}

/**
 * @brief Copies the contents of the given parameters into the compute record. Note, call
 * @ref gprtBuildShaderBindingTable for these parameters to be uploaded to the device.
 *
 * @param compute The compute program who's record to assign the parameters to.
 * @param parameters A pointer to a parameters structure. Assumed to be "recordSize" bytes,
 * as specified by @ref gprtComputeCreate.
 */
GPRT_API void gprtComputeSetParameters(GPRTCompute compute, void *parameters, int deviceID GPRT_IF_CPP(= 0));

/**
 * @brief Copies the contents of the given parameters into the geometry record. Note, call
 * @ref gprtBuildShaderBindingTable for these parameters to be uploaded to the device.
 * @tparam T The type of the parameters structure stored in the record.
 * @param compute The compute program who's record to assign the parameters to.
 * @param parameters A pointer to a parameters structure. Assumed to be "recordSize" bytes,
 * as specified by @ref gprtComputeCreate.
 */
template <typename T>
void
gprtComputeSetParameters(GPRTComputeOf<T> compute, T *parameters, int deviceID GPRT_IF_CPP(= 0)) {
  gprtComputeSetParameters((GPRTCompute) compute, (void *) parameters, deviceID);
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
gprtRayGenSetParameters(GPRTRayGenOf<T> rayGen, T *parameters, int deviceID GPRT_IF_CPP(= 0)) {
  gprtRayGenSetParameters((GPRTRayGen) rayGen, (void *) parameters, deviceID);
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
gprtMissSetParameters(GPRTMissOf<T> miss, T *parameters, int deviceID GPRT_IF_CPP(= 0)) {
  gprtMissSetParameters((GPRTMiss) miss, (void *) parameters, deviceID);
}

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
GPRT_API GPRTAccel gprtAABBAccelCreate(GPRTContext context, size_t numGeometries, GPRTGeom *arrayOfChildGeoms,
                                       unsigned int flags GPRT_IF_CPP(= 0));

template <typename T>
GPRTAccel
gprtAABBAccelCreate(GPRTContext context, size_t numGeometries, GPRTGeomOf<T> *arrayOfChildGeoms,
                    unsigned int flags GPRT_IF_CPP(= 0)) {
  return gprtAABBAccelCreate(context, numGeometries, (GPRTGeom *) arrayOfChildGeoms, flags);
}

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
GPRT_API GPRTAccel gprtTrianglesAccelCreate(GPRTContext context, size_t numGeometries, GPRTGeom *arrayOfChildGeoms,
                                            unsigned int flags GPRT_IF_CPP(= 0));

template <typename T>
GPRTAccel
gprtTrianglesAccelCreate(GPRTContext context, size_t numGeometries, GPRTGeomOf<T> *arrayOfChildGeoms,
                         unsigned int flags GPRT_IF_CPP(= 0)) {
  return gprtTrianglesAccelCreate(context, numGeometries, (GPRTGeom *) arrayOfChildGeoms, flags);
}

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
GPRT_API GPRTAccel gprtInstanceAccelCreate(GPRTContext context, uint32_t numAccels, GPRTAccel *arrayOfAccels,
                                           unsigned int flags GPRT_IF_CPP(= 0));

GPRT_API void gprtInstanceAccelSetTransforms(GPRTAccel instanceAccel, GPRTBuffer transforms, uint32_t stride,
                                             uint32_t offset);

template <typename T>
void
gprtInstanceAccelSetTransforms(GPRTAccel instanceAccel, GPRTBufferOf<T> transforms, uint32_t stride, uint32_t offset) {
  gprtInstanceAccelSetTransforms(instanceAccel, (GPRTBuffer) transforms, stride, offset);
}

GPRT_API void gprtInstanceAccelSet3x4Transforms(GPRTAccel instanceAccel, GPRTBuffer transforms);

template <typename T>
void
gprtInstanceAccelSet3x4Transforms(GPRTAccel instanceAccel, GPRTBufferOf<T> transforms) {
  gprtInstanceAccelSet3x4Transforms((GPRTAccel) instanceAccel, (GPRTBuffer) transforms);
}

GPRT_API void gprtInstanceAccelSet4x4Transforms(GPRTAccel instanceAccel, GPRTBuffer transforms);

template <typename T>
void
gprtInstanceAccelSet4x4Transforms(GPRTAccel instanceAccel, GPRTBufferOf<T> transforms) {
  gprtInstanceAccelSet4x4Transforms(instanceAccel, (GPRTBuffer) transforms);
}

/*! sets the list of IDs to use for the child instnaces. By default
    the instance ID of child #i is simply i, but optix allows to
    specify a user-defined instnace ID for each instance, which with
    owl can be done through this array. Array size must match number
    of instances in the specified group */
GPRT_API void gprtInstanceAccelSetIDs(GPRTAccel instanceAccel, const uint32_t *instanceIDs);

/**
 * @brief Assigns a 32-bit int buffer to the instance acceleration structure
 * for the visibility mask, with only last 8 bits used. Rays will bitwise AND
 * their instance inclusion mask with these masks, skipping over instances when
 * the result is "0".
 *
 * @param instanceAccel The instance acceleration structure
 * @param masks A buffer of 32-bit ints, one int per instance in the instance
 * acceleration structure.
 */
GPRT_API void gprtInstanceAccelSetVisibilityMasks(GPRTAccel instanceAccel, GPRTBuffer masks);

/**
 * @brief Assigns a 32-bit int buffer to the instance acceleration structure
 * for the visibility mask, with only last 8 bits used. Rays will bitwise AND
 * their instance inclusion mask with these masks, skipping over instances when
 * the result is "0".
 *
 * @tparam T The template type of the given buffer (will be cast to a 32-bit
 * int)
 * @param instanceAccel The instance acceleration structure
 * @param masks A buffer of 32-bit ints, one int per instance in the instance
 * acceleration structure.
 */
template <typename T>
void
gprtInstanceAccelSetVisibilityMasks(GPRTAccel instanceAccel, GPRTBufferOf<T> masks) {
  gprtInstanceAccelSetVisibilityMasks(instanceAccel, (GPRTBuffer) masks);
}

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

/**
 * @brief Creates a "geometry type", which describes the base primitive kind, device programs to call during
 * intersection and/or rasterization, and the parameters to pass into these device programs.
 *
 * @param context The GPRT context
 * @param kind The base primitive kind, (triangles, AABBs, curves, etc)
 * @param recordSize The size of the parameters structure that will be passed into the device programs
 */
GPRT_API GPRTGeomType gprtGeomTypeCreate(GPRTContext context, GPRTGeomKind kind, size_t recordSize);

/**
 * @brief Creates a "geometry type", which describes the base primitive kind, device programs to call during
 * intersection and/or rasterization, and the parameters to pass into these device programs.
 * @tparam T The template type of the parameters structure that will be passed into the device programs
 * @param context The GPRT context
 * @param kind The base primitive kind, (triangles, AABBs, curves, etc)
 */
template <typename T>
GPRTGeomTypeOf<T>
gprtGeomTypeCreate(GPRTContext context, GPRTGeomKind kind) {
  return (GPRTGeomTypeOf<T>) gprtGeomTypeCreate(context, kind, sizeof(T));
}

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

GPRT_API void gprtGeomTypeSetVertexProg(GPRTGeomType type, int rasterType, GPRTModule module, const char *entrypoint);

template <typename T>
void
gprtGeomTypeSetVertexProg(GPRTGeomTypeOf<T> type, int rasterType, GPRTModule module, const char *entrypoint) {
  gprtGeomTypeSetVertexProg((GPRTGeomType) type, rasterType, module, entrypoint);
}

GPRT_API void gprtGeomTypeSetPixelProg(GPRTGeomType type, int rasterType, GPRTModule module, const char *entrypoint);

template <typename T>
void
gprtGeomTypeSetPixelProg(GPRTGeomTypeOf<T> type, int rasterType, GPRTModule module, const char *entrypoint) {
  gprtGeomTypeSetPixelProg((GPRTGeomType) type, rasterType, module, entrypoint);
}

GPRT_API void gprtGeomTypeSetRasterAttachments(GPRTGeomType type, int rasterType, GPRTTexture colorAttachment,
                                               GPRTTexture depthAttachment);

template <typename T1, typename T2, typename T3>
void
gprtGeomTypeSetRasterAttachments(GPRTGeomTypeOf<T1> type, int rasterType, GPRTTextureOf<T2> colorAttachment,
                                 GPRTTextureOf<T3> depthAttachment) {
  gprtGeomTypeSetRasterAttachments((GPRTGeomType) type, rasterType, (GPRTTexture) colorAttachment,
                                   (GPRTTexture) depthAttachment);
}

void gprtGeomTypeRasterize(GPRTContext context, GPRTGeomType geomType, uint32_t numGeometry, GPRTGeom *geometry,
                           uint32_t rasterType = 0, uint32_t *instanceCounts = nullptr);

template <typename T>
void
gprtGeomTypeRasterize(GPRTContext context, GPRTGeomTypeOf<T> geomType, uint32_t numGeometry, GPRTGeomOf<T> *geometry,
                      uint32_t rayType = 0, uint32_t *instanceCounts = nullptr) {
  gprtGeomTypeRasterize(context, (GPRTGeomType) geomType, numGeometry, (GPRTGeom *) geometry, rayType, instanceCounts);
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

/*! Creates a buffer that uses memory located on the host; that memory is
accessible to all devices, but is slower to access on device.  */
GPRT_API GPRTBuffer gprtHostBufferCreate(GPRTContext context, size_t size, size_t count GPRT_IF_CPP(= 1),
                                         const void *init GPRT_IF_CPP(= nullptr));

template <typename T>
GPRTBufferOf<T>
gprtHostBufferCreate(GPRTContext context, size_t count GPRT_IF_CPP(= 1), const T *init GPRT_IF_CPP(= nullptr)) {
  return (GPRTBufferOf<T>) gprtHostBufferCreate(context, sizeof(T), count, init);
}

/*! Creates a buffer that uses memory located on the device; that memory is
accessible only to the device, and requires mapping and unmapping to access
on the host. */
GPRT_API GPRTBuffer gprtDeviceBufferCreate(GPRTContext context, size_t size, size_t count GPRT_IF_CPP(= 1),
                                           const void *init GPRT_IF_CPP(= nullptr));

template <typename T>
GPRTBufferOf<T>
gprtDeviceBufferCreate(GPRTContext context, size_t count GPRT_IF_CPP(= 1), const T *init GPRT_IF_CPP(= nullptr)) {
  return (GPRTBufferOf<T>) gprtDeviceBufferCreate(context, sizeof(T), count, init);
}

/*! Creates a buffer that uses memory located on the device; that memory is
accessible to all devices, but is slower to access on the host, and is typically
limited in size depending on resizable BAR availability. */
GPRT_API GPRTBuffer gprtSharedBufferCreate(GPRTContext context, size_t size, size_t count GPRT_IF_CPP(= 1),
                                           const void *init GPRT_IF_CPP(= nullptr));

template <typename T>
GPRTBufferOf<T>
gprtSharedBufferCreate(GPRTContext context, size_t count GPRT_IF_CPP(= 1), const T *init GPRT_IF_CPP(= nullptr)) {
  return (GPRTBufferOf<T>) gprtSharedBufferCreate(context, sizeof(T), count, init);
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
GPRT_API void *gprtBufferGetPointer(GPRTBuffer buffer, int deviceID GPRT_IF_CPP(= 0));

template <typename T>
T *
gprtBufferGetPointer(GPRTBufferOf<T> buffer, int deviceID GPRT_IF_CPP(= 0)) {
  return (T *) gprtBufferGetPointer((GPRTBuffer) buffer, deviceID);
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
                      uint32_t bufferRowLength, uint32_t bufferImageHeight, uint32_t imageOffsetX, uint32_t imageOffsetY,
                      uint32_t imageOffsetZ, uint32_t imageExtentX, uint32_t imageExtentY, uint32_t imageExtentZ,
                      int srcDeviceID GPRT_IF_CPP(= 0), int dstDeviceID GPRT_IF_CPP(= 0)) {
  gprtBufferTextureCopy(context, (GPRTBuffer) buffer, (GPRTTexture) texture, bufferOffset, bufferRowLength,
                        bufferImageHeight, imageOffsetX, imageOffsetY, imageOffsetZ, imageExtentX, imageExtentY,
                        imageExtentZ, srcDeviceID, dstDeviceID);
}

/**
 * @brief Sorts the input buffer using a GPU-parallel radix sorter.
 * Radix sort requires a temporary "scratch" space
 *
 *
 * @param context The GPRT context
 * @param buffer A buffer of 32-bit unsigned integers
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
 * @param buffer A buffer of 32-bit unsigned integers
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
 * @param keys A buffer of 32-bit unsigned integer keys
 * @param values A buffer of 32-bit values
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

GPRT_API gprt::Buffer gprtBufferGetHandle(GPRTBuffer buffer, int deviceID GPRT_IF_CPP(= 0));

template <typename T>
gprt::Buffer
gprtBufferGetHandle(GPRTBufferOf<T> buffer, int deviceID GPRT_IF_CPP(= 0)) {
  return gprtBufferGetHandle((GPRTBuffer) buffer, deviceID);
}

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

GPRT_API void gprtRayGenLaunch1D(GPRTContext context, GPRTRayGen rayGen, uint32_t dims_x);

template <typename T>
void
gprtRayGenLaunch1D(GPRTContext context, GPRTRayGenOf<T> rayGen, uint32_t dims_x) {
  gprtRayGenLaunch1D(context, (GPRTRayGen) rayGen, dims_x);
}

/*! Executes a ray tracing pipeline with the given raygen program.
  This call will block until the raygen program returns. */
GPRT_API void gprtRayGenLaunch2D(GPRTContext context, GPRTRayGen rayGen, uint32_t dims_x, uint32_t dims_y);

template <typename T>
void
gprtRayGenLaunch2D(GPRTContext context, GPRTRayGenOf<T> rayGen, uint32_t dims_x, uint32_t dims_y) {
  gprtRayGenLaunch2D(context, (GPRTRayGen) rayGen, dims_x, dims_y);
}

/*! 3D-launch variant of \see gprtRayGenLaunch2D */
GPRT_API void gprtRayGenLaunch3D(GPRTContext context, GPRTRayGen rayGen, uint32_t dims_x, uint32_t dims_y, uint32_t dims_z);

template <typename T>
void
gprtRayGenLaunch3D(GPRTContext context, GPRTRayGenOf<T> rayGen, uint32_t dims_x, uint32_t dims_y, uint32_t dims_z) {
  gprtRayGenLaunch3D(context, (GPRTRayGen) rayGen, dims_x, dims_y, dims_z);
}

GPRT_API void gprtComputeLaunch1D(GPRTContext context, GPRTCompute compute, uint32_t x_workgroups);

template <typename T>
void
gprtComputeLaunch1D(GPRTContext context, GPRTComputeOf<T> compute, uint32_t x_workgroups) {
  gprtComputeLaunch1D(context, (GPRTCompute) compute, x_workgroups);
}

GPRT_API void gprtComputeLaunch2D(GPRTContext context, GPRTCompute compute, uint32_t x_workgroups, uint32_t y_workgroups);

template <typename T>
void
gprtComputeLaunch2D(GPRTContext context, GPRTComputeOf<T> compute, uint32_t x_workgroups, uint32_t y_workgroups) {
  gprtComputeLaunch2D(context, (GPRTCompute) compute, x_workgroups, y_workgroups);
}

GPRT_API void gprtComputeLaunch3D(GPRTContext context, GPRTCompute compute, uint32_t x_workgroups, uint32_t y_workgroups,
                                  uint32_t z_workgroups);

template <typename T>
void
gprtComputeLaunch3D(GPRTContext context, GPRTComputeOf<T> compute, uint32_t x_workgroups, uint32_t y_workgroups,
                    uint32_t z_workgroups) {
  gprtComputeLaunch3D(context, (GPRTCompute) compute, x_workgroups, y_workgroups, z_workgroups);
}

GPRT_API void gprtBeginProfile(GPRTContext context);

// returned results are in milliseconds
GPRT_API float gprtEndProfile(GPRTContext context);
