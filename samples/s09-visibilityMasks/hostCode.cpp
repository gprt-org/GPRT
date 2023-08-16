// MIT License

// Copyright (c) 2022 Nathan V. Morrical

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

// This program sets up a single geometric object, a mesh for a cube, and
// its acceleration structure, then ray traces it.

// public GPRT API
#include <gprt.h>

// our shared data structures between host and device
#include "sharedCode.h"

#define LOG(message)                                                                                                   \
  std::cout << GPRT_TERMINAL_BLUE;                                                                                     \
  std::cout << "#gprt.sample(main): " << message << std::endl;                                                         \
  std::cout << GPRT_TERMINAL_DEFAULT;
#define LOG_OK(message)                                                                                                \
  std::cout << GPRT_TERMINAL_LIGHT_BLUE;                                                                               \
  std::cout << "#gprt.sample(main): " << message << std::endl;                                                         \
  std::cout << GPRT_TERMINAL_DEFAULT;

extern GPRTProgram s09_deviceCode;

// Scene geometry. Looks a bit intimidating, but basically just a couple
// hand-plotted triangles, two definiting a floor plane, 16 triangles defining
// a wall with a hole in it for a window, and then two triangles filling that
// hole with a window.
const int NUM_FLOOR_VERTICES = 6;
float3 floorVertices[NUM_FLOOR_VERTICES] = {
    {0.0, 0.0, -6.0}, {3.0, 0.0, -6.0}, {3.0, 0.0, 6.0}, {0.0, 0.0, -6.0}, {3.0, 0.0, 6.0}, {0.0, 0.0, 6.0},
};

const int NUM_FLOOR_INDICES = 6;
int3 floorIndices[NUM_FLOOR_INDICES] = {
    {0, 1, 2}, {3, 4, 5}, {6, 7, 8}, {9, 10, 11}, {12, 13, 14}, {15, 16, 17},
};

const int NUM_WALL_VERTICES = 48;
float3 wallVertices[NUM_WALL_VERTICES] = {
    {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {1.0, 1.0, 0.0}, {0.0, 0.0, 0.0}, {1.0, 1.0, 0.0}, {0.0, 1.0, 0.0},
    {1.0, 0.0, 0.0}, {2.0, 0.0, 0.0}, {2.0, 1.0, 0.0}, {1.0, 0.0, 0.0}, {2.0, 1.0, 0.0}, {1.0, 1.0, 0.0},
    {2.0, 0.0, 0.0}, {3.0, 0.0, 0.0}, {3.0, 1.0, 0.0}, {2.0, 0.0, 0.0}, {3.0, 1.0, 0.0}, {2.0, 1.0, 0.0},
    {0.0, 1.0, 0.0}, {1.0, 1.0, 0.0}, {1.0, 3.0, 0.0}, {0.0, 1.0, 0.0}, {1.0, 3.0, 0.0}, {0.0, 3.0, 0.0},
    {2.0, 1.0, 0.0}, {3.0, 1.0, 0.0}, {3.0, 3.0, 0.0}, {2.0, 1.0, 0.0}, {3.0, 3.0, 0.0}, {2.0, 3.0, 0.0},
    {0.0, 3.0, 0.0}, {1.0, 3.0, 0.0}, {1.0, 4.0, 0.0}, {0.0, 3.0, 0.0}, {1.0, 4.0, 0.0}, {0.0, 4.0, 0.0},
    {1.0, 3.0, 0.0}, {2.0, 3.0, 0.0}, {2.0, 4.0, 0.0}, {1.0, 3.0, 0.0}, {2.0, 4.0, 0.0}, {1.0, 4.0, 0.0},
    {2.0, 3.0, 0.0}, {3.0, 3.0, 0.0}, {3.0, 4.0, 0.0}, {2.0, 3.0, 0.0}, {3.0, 4.0, 0.0}, {2.0, 4.0, 0.0},
};

const int NUM_WALL_INDICES = 16;
int3 wallIndices[NUM_WALL_INDICES] = {
    {0, 1, 2},    {3, 4, 5},    {6, 7, 8},    {9, 10, 11},  {12, 13, 14}, {15, 16, 17}, {18, 19, 20}, {21, 22, 23},
    {24, 25, 26}, {27, 28, 29}, {30, 31, 32}, {33, 34, 35}, {36, 37, 38}, {39, 40, 41}, {42, 43, 44}, {45, 46, 47}};

const int NUM_WINDOW_VERTICES = 6;
float3 windowVertices[NUM_WINDOW_VERTICES] = {
    {1.0, 1.0, 0.0}, {2.0, 1.0, 0.0}, {2.0, 3.0, 0.0}, {1.0, 1.0, 0.0}, {2.0, 3.0, 0.0}, {1.0, 3.0, 0.0},
};

const int NUM_WINDOW_INDICES = 6;
int3 windowIndices[NUM_WINDOW_INDICES] = {
    {0, 1, 2}, {3, 4, 5}, {6, 7, 8}, {9, 10, 11}, {12, 13, 14}, {15, 16, 17},
};

// We'll make a couple copies of the above geometry. Just transforming
// those copies around using the transforms below. Note, we need to
// transform the floor, wall, and window, so 3 transforms per "copy".
const int NUM_INSTANCES = 15;
float4x4 transforms[NUM_INSTANCES] = {
    transpose(translation_matrix(float3(-6.0, 0.0, 0.0))), transpose(translation_matrix(float3(-6.0, 0.0, 0.0))),
    transpose(translation_matrix(float3(-6.0, 0.0, 0.0))), transpose(translation_matrix(float3(-3.0, 0.0, 0.0))),
    transpose(translation_matrix(float3(-3.0, 0.0, 0.0))), transpose(translation_matrix(float3(-3.0, 0.0, 0.0))),
    transpose(translation_matrix(float3(0.0, 0.0, 0.0))),  transpose(translation_matrix(float3(0.0, 0.0, 0.0))),
    transpose(translation_matrix(float3(0.0, 0.0, 0.0))),  transpose(translation_matrix(float3(+3.0, 0.0, 0.0))),
    transpose(translation_matrix(float3(+3.0, 0.0, 0.0))), transpose(translation_matrix(float3(+3.0, 0.0, 0.0))),
    transpose(translation_matrix(float3(+6.0, 0.0, 0.0))), transpose(translation_matrix(float3(+6.0, 0.0, 0.0))),
    transpose(translation_matrix(float3(+6.0, 0.0, 0.0))),
};

// initial image resolution
const int2 fbSize = {1400, 460};

// final image output
const char *outFileName = "s09-visibilityMasks.png";

// Initial camera parameters
float3 lookFrom = {1.5f, 6.f, -10.f};
float3 lookAt = {1.5f, 1.5f, -1.5f};
float3 lookUp = {0.f, -1.f, 0.f};
float cosFovy = 0.66f;

// Light position
float3 lightPos = {1.5f, 6.0f, 6.0f};
float3 lightColor = {253.f / 255.f, 251.f / 255.f, 211.f / 255.f};

#include <iostream>
int
main(int ac, char **av) {
  // In this example, we'll be using a mechanism in modern GPU ray tracing
  // frameworks called "visibility masks". We'll use these masks to make
  // our glass window meshes "invisible" to a shadow ray traced towards
  // the sun.
  LOG("gprt example '" << av[0] << "' starting up");

  gprtRequestWindow(fbSize.x, fbSize.y, "S09 Visibility Masks");
  GPRTContext context = gprtContextCreate();
  GPRTModule module = gprtModuleCreate(context, s09_deviceCode);

  // ##################################################################
  // set up all the GPU kernels we want to run
  // ##################################################################

  GPRTGeomTypeOf<TrianglesGeomData> trianglesGeomType = gprtGeomTypeCreate<TrianglesGeomData>(context, GPRT_TRIANGLES);
  gprtGeomTypeSetClosestHitProg(trianglesGeomType, 0, module, "TriangleMesh");

  GPRTRayGenOf<RayGenData> rayGen = gprtRayGenCreate<RayGenData>(context, module, "simpleRayGen");

  GPRTMissOf<MissProgData> miss = gprtMissCreate<MissProgData>(context, module, "miss");

  // ##################################################################
  // set the parameters for those kernels
  // ##################################################################

  RayGenData *rayGenData = gprtRayGenGetParameters(rayGen);

  // Setup pixel frame buffer
  GPRTBufferOf<uint32_t> frameBuffer = gprtDeviceBufferCreate<uint32_t>(context, fbSize.x * fbSize.y);
  rayGenData->frameBuffer = gprtBufferGetHandle(frameBuffer);

  // Here, we'll setup our initial light position and color
  rayGenData->lightPos = lightPos;
  rayGenData->lightColor = lightColor;

  // Miss program checkerboard background colors
  MissProgData *missData = gprtMissGetParameters(miss);
  missData->color0 = float3(0.1f, 0.1f, 0.1f);
  missData->color1 = float3(0.0f, 0.0f, 0.0f);

  LOG("building geometries ...");

  // First, we'll make a floor mesh
  GPRTBufferOf<float3> floorVertexBuffer = gprtDeviceBufferCreate<float3>(context, NUM_FLOOR_VERTICES, floorVertices);
  GPRTBufferOf<int3> floorIndexBuffer = gprtDeviceBufferCreate<int3>(context, NUM_FLOOR_INDICES, floorIndices);
  GPRTGeomOf<TrianglesGeomData> floorGeom = gprtGeomCreate<TrianglesGeomData>(context, trianglesGeomType);
  gprtTrianglesSetVertices(floorGeom, floorVertexBuffer, NUM_FLOOR_VERTICES);
  gprtTrianglesSetIndices(floorGeom, floorIndexBuffer, NUM_FLOOR_INDICES);
  GPRTAccel floorAccel = gprtTriangleAccelCreate(context, 1, &floorGeom);
  gprtAccelBuild(context, floorAccel, GPRT_BUILD_MODE_FAST_TRACE_NO_UPDATE);
  TrianglesGeomData *floorData = gprtGeomGetParameters(floorGeom);

  // The floor will have a brown-ish color to it
  floorData->color = float4(153.f / 255.f, 121.f / 255.f, 80.f / 255.f, 1.f);
  floorData->vertices = gprtBufferGetHandle(floorVertexBuffer);
  floorData->indices = gprtBufferGetHandle(floorIndexBuffer);

  // Then, we'll make a wall with a hole in it where our window will go.
  GPRTBufferOf<float3> wallVertexBuffer = gprtDeviceBufferCreate<float3>(context, NUM_WALL_VERTICES, wallVertices);
  GPRTBufferOf<int3> wallIndexBuffer = gprtDeviceBufferCreate<int3>(context, NUM_WALL_INDICES, wallIndices);
  GPRTGeomOf<TrianglesGeomData> wallGeom = gprtGeomCreate<TrianglesGeomData>(context, trianglesGeomType);
  gprtTrianglesSetVertices(wallGeom, wallVertexBuffer, NUM_WALL_VERTICES);
  gprtTrianglesSetIndices(wallGeom, wallIndexBuffer, NUM_WALL_INDICES);
  GPRTAccel wallAccel = gprtTriangleAccelCreate(context, 1, &wallGeom);
  gprtAccelBuild(context, wallAccel, GPRT_BUILD_MODE_FAST_TRACE_NO_UPDATE);
  TrianglesGeomData *wallData = gprtGeomGetParameters(wallGeom);

  // The wall has an off-white color
  wallData->color = float4(230.f / 255.f, 225.f / 255.f, 221.f / 255.f, 1.f);
  wallData->vertices = gprtBufferGetHandle(wallVertexBuffer);
  wallData->indices = gprtBufferGetHandle(wallIndexBuffer);

  // Finally we'll make a window
  GPRTBufferOf<float3> windowVertexBuffer =
      gprtDeviceBufferCreate<float3>(context, NUM_WINDOW_VERTICES, windowVertices);
  GPRTBufferOf<int3> windowIndexBuffer = gprtDeviceBufferCreate<int3>(context, NUM_WINDOW_INDICES, windowIndices);
  GPRTGeomOf<TrianglesGeomData> windowGeom = gprtGeomCreate<TrianglesGeomData>(context, trianglesGeomType);
  gprtTrianglesSetVertices(windowGeom, windowVertexBuffer, NUM_WINDOW_VERTICES);
  gprtTrianglesSetIndices(windowGeom, windowIndexBuffer, NUM_WINDOW_INDICES);
  GPRTAccel windowAccel = gprtTriangleAccelCreate(context, 1, &windowGeom);
  gprtAccelBuild(context, windowAccel, GPRT_BUILD_MODE_FAST_TRACE_NO_UPDATE);
  TrianglesGeomData *windowData = gprtGeomGetParameters(windowGeom);

  // The window has a blue-ish color, and an alpha transparency of 50%.
  windowData->color = float4(199.f / 255.f, 227.f / 255.f, 225.f / 255.f, 0.5f);
  windowData->vertices = gprtBufferGetHandle(windowVertexBuffer);
  windowData->indices = gprtBufferGetHandle(windowIndexBuffer);

  // Now stick both of these into a tree.
  // Note, we're making multiple instances of the same wall and window.
  // The transforms buffer will place these walls and windows into the world
  std::vector<GPRTAccel> BLAS = {
      floorAccel,  wallAccel,  windowAccel, floorAccel,  wallAccel,  windowAccel, floorAccel,  wallAccel,
      windowAccel, floorAccel, wallAccel,   windowAccel, floorAccel, wallAccel,   windowAccel,
  };

  GPRTBufferOf<float4x4> transformsBuffer = gprtDeviceBufferCreate<float4x4>(context, NUM_INSTANCES, transforms);

  GPRTAccel world = gprtInstanceAccelCreate(context, (uint32_t)BLAS.size(), BLAS.data());
  gprtInstanceAccelSet4x4Transforms(world, transformsBuffer);

  // This is new! We want our wall and floor instances to cast shadows,
  // but we don't want our window to cast a shadow.

  // And so, we set the wall and floor visibility masks to 0b11111111,
  // meaning, any ray traced with any visibility bits "on" will hit those
  // meshes.

  // For the window, we use a visibility mask of 0b11111110.
  //                            ( This bit is important   ^ )
  // The last bit being 0 means that instance will be invisible
  // to rays traced with a visibility mask of 0b00000001, since
  // 0b00000001 & 0b1111110 == 0
  std::vector<uint32_t> masks = {
      0b11111111, 0b11111111, 0b11111110, 0b11111111, 0b11111111, 0b11111110, 0b11111111, 0b11111111,
      0b11111110, 0b11111111, 0b11111111, 0b11111110, 0b11111111, 0b11111111, 0b11111110,
  };

  GPRTBufferOf<uint32_t> masksBuffer = gprtDeviceBufferCreate<uint32_t>(context, masks.size(), masks.data());
  gprtInstanceAccelSetVisibilityMasks(world, masksBuffer);

  // Now that our instance acceleration structure is setup, build it.
  gprtAccelBuild(context, world, GPRT_BUILD_MODE_FAST_TRACE_NO_UPDATE);

  // Set the accel handle
  rayGenData->world = gprtAccelGetHandle(world);

  gprtBuildShaderBindingTable(context, GPRT_SBT_ALL);

  // ##################################################################
  // now that everything is ready: launch it ....
  // ##################################################################

  LOG("launching ...");

  bool firstFrame = true;
  double xpos = 0.f, ypos = 0.f;
  double lastxpos, lastypos;
  do {
    float speed = .001f;
    lastxpos = xpos;
    lastypos = ypos;
    gprtGetCursorPos(context, &xpos, &ypos);
    if (firstFrame) {
      lastxpos = xpos;
      lastypos = ypos;
    }
    int state = gprtGetMouseButton(context, GPRT_MOUSE_BUTTON_LEFT);

    // If we click the mouse, we should rotate the camera
    // Here, we implement some simple camera controls
    if (state == GPRT_PRESS || firstFrame) {
      firstFrame = false;
      float4 position = {lookFrom.x, lookFrom.y, lookFrom.z, 1.f};
      float4 pivot = {lookAt.x, lookAt.y, lookAt.z, 1.0};
#ifndef M_PI
#define M_PI 3.1415926f
#endif

      // step 1 : Calculate the amount of rotation given the mouse movement.
      float deltaAngleX = (2 * M_PI / fbSize.x);
      float deltaAngleY = (M_PI / fbSize.y);
      float xAngle = float(lastxpos - xpos) * deltaAngleX;
      float yAngle = float(lastypos - ypos) * deltaAngleY;

      // step 2: Rotate the camera around the pivot point on the first axis.
      float4x4 rotationMatrixX = rotation_matrix(rotation_quat(lookUp, xAngle));
      position = (mul(rotationMatrixX, (position - pivot))) + pivot;

      // step 3: Rotate the camera around the pivot point on the second axis.
      float3 lookRight = cross(lookUp, normalize(pivot - position).xyz());
      float4x4 rotationMatrixY = rotation_matrix(rotation_quat(lookRight, yAngle));
      lookFrom = ((mul(rotationMatrixY, (position - pivot))) + pivot).xyz();

      // ----------- compute variable values  ------------------
      float3 camera_pos = lookFrom;
      float3 camera_d00 = normalize(lookAt - lookFrom);
      float aspect = float(fbSize.x) / float(fbSize.y);
      float3 camera_ddu = cosFovy * aspect * normalize(cross(camera_d00, lookUp));
      float3 camera_ddv = cosFovy * normalize(cross(camera_ddu, camera_d00));
      camera_d00 -= 0.5f * camera_ddu;
      camera_d00 -= 0.5f * camera_ddv;

      // ----------- set variables  ----------------------------
      RayGenData *raygenData = gprtRayGenGetParameters(rayGen);
      raygenData->camera.pos = camera_pos;
      raygenData->camera.dir_00 = camera_d00;
      raygenData->camera.dir_du = camera_ddu;
      raygenData->camera.dir_dv = camera_ddv;

      // Use this to upload all set parameters to our ray tracing device
      gprtBuildShaderBindingTable(context, GPRT_SBT_RAYGEN);
    }

    rayGenData->lightPos = float3(3.f * sin((float)gprtGetTime(context)), 3.f, 3.f * cos((float)gprtGetTime(context)));
    gprtBuildShaderBindingTable(context, GPRT_SBT_RAYGEN);

    // Calls the GPU raygen kernel function
    gprtRayGenLaunch2D(context, rayGen, fbSize.x, fbSize.y);

    // If a window exists, presents the framebuffer here to that window
    gprtBufferPresent(context, frameBuffer);
  }
  // returns true if "X" pressed or if in "headless" mode
  while (!gprtWindowShouldClose(context));

  // Save final frame to an image
  LOG("done with launch, writing frame buffer to " << outFileName);
  gprtBufferSaveImage(frameBuffer, fbSize.x, fbSize.y, outFileName);
  LOG_OK("written rendered frame buffer to file " << outFileName);

  // ##################################################################
  // and finally, clean up
  // ##################################################################

  LOG("cleaning up ...");

  gprtBufferDestroy(floorVertexBuffer);
  gprtBufferDestroy(floorIndexBuffer);
  gprtGeomDestroy(floorGeom);
  gprtAccelDestroy(floorAccel);

  gprtBufferDestroy(wallVertexBuffer);
  gprtBufferDestroy(wallIndexBuffer);
  gprtGeomDestroy(wallGeom);
  gprtAccelDestroy(wallAccel);

  gprtBufferDestroy(windowVertexBuffer);
  gprtBufferDestroy(windowIndexBuffer);
  gprtGeomDestroy(windowGeom);
  gprtAccelDestroy(windowAccel);

  gprtBufferDestroy(transformsBuffer);
  gprtBufferDestroy(masksBuffer);
  gprtAccelDestroy(world);

  gprtBufferDestroy(frameBuffer);
  gprtRayGenDestroy(rayGen);
  gprtMissDestroy(miss);
  gprtGeomTypeDestroy(trianglesGeomType);
  gprtModuleDestroy(module);
  gprtContextDestroy(context);

  LOG_OK("seems all went OK; app is done, this should be the last output ...");
}
