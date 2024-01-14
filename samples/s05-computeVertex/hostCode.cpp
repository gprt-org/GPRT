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

const int GRID_SIDE_LENGTH = 1000;

extern GPRTProgram s05_deviceCode;

// initial image resolution
const int2 fbSize = {1400, 460};

const char *outFileName = "s04-computeAABBs.png";

float3 lookFrom = {-2.f, -2.f, 0.f};
float3 lookAt = {0.f, 0.f, 0.f};
float3 lookUp = {0.f, 0.f, -1.f};
float cosFovy = 0.3f;

#include <iostream>
int
main(int ac, char **av) {
  // In this example, we will use a GPU compute kernel to animate the vertices
  // of a mesh, like you might do in a traditional rasterization pipeline.
  // Every time the vertices move, we must rebuild the underlying acceleration
  // structure. Fortunately, GPRT makes these tree builds fast and easy.

  // create a context on the first device:
  gprtRequestWindow(fbSize.x, fbSize.y, "S05 Compute Vertex");
  GPRTContext context = gprtContextCreate(nullptr, 1);
  GPRTModule module = gprtModuleCreate(context, s05_deviceCode);

  // Structure of parameters that change each frame. We can edit these 
  // without rebuilding the shader binding table.
  PushConstants pc;
  pc.now = 0.f;

  // ##################################################################
  // set up all the GPU kernels we want to run
  // ##################################################################

  // -------------------------------------------------------
  // Setup geometry types
  // -------------------------------------------------------
  GPRTGeomTypeOf<TrianglesGeomData> trianglesGeomType = gprtGeomTypeCreate<TrianglesGeomData>(context, GPRT_TRIANGLES);
  gprtGeomTypeSetClosestHitProg(trianglesGeomType, 0, module, "ClosestHit");

  // -------------------------------------------------------
  // set up vertex program to animate vertices
  // -------------------------------------------------------
  GPRTComputeOf<TrianglesGeomData> vertexProgram = gprtComputeCreate<TrianglesGeomData>(context, module, "Vertex");

  // -------------------------------------------------------
  // set up ray gen program
  // -------------------------------------------------------
  GPRTRayGenOf<RayGenData> rayGen = gprtRayGenCreate<RayGenData>(context, module, "RayGen");

  // -------------------------------------------------------
  // set up miss
  // -------------------------------------------------------
  GPRTMissOf<MissProgData> miss = gprtMissCreate<MissProgData>(context, module, "miss");

  // ##################################################################
  // set the parameters for our triangle mesh and compute kernel
  // ##################################################################

  // Note, the vertex and index buffers are empty here. We will fill
  // them in on the device by using our vertex program.
  unsigned int numTriangles = 2 * GRID_SIDE_LENGTH * GRID_SIDE_LENGTH;
  unsigned int numVertices = 3 * numTriangles;
  GPRTBufferOf<float3> vertexBuffer = gprtDeviceBufferCreate<float3>(context, numVertices, nullptr);
  GPRTBufferOf<uint3> indexBuffer = gprtDeviceBufferCreate<uint3>(context, numTriangles, nullptr);

  GPRTGeomOf<TrianglesGeomData> trianglesGeom = gprtGeomCreate(context, trianglesGeomType);

  // It is _okay_ to give our triangles geometry unpopulated buffers here
  // for the vertices and indices, so long as they're filled in before
  // we go to build our acceleration structure.
  gprtTrianglesSetVertices(trianglesGeom, vertexBuffer, numVertices);
  gprtTrianglesSetIndices(trianglesGeom, indexBuffer, numTriangles);

  // Parameters for the geometry when a ray hits it.
  TrianglesGeomData *geomData = gprtGeomGetParameters(trianglesGeom);
  geomData->vertex = gprtBufferGetHandle(vertexBuffer);
  geomData->index = gprtBufferGetHandle(indexBuffer);
  geomData->gridSize = GRID_SIDE_LENGTH;

  // Parameters for our vertex program that'll animate our vertices
  TrianglesGeomData vertexData;
  vertexData.vertex = gprtBufferGetHandle(vertexBuffer);
  vertexData.index = gprtBufferGetHandle(indexBuffer);
  vertexData.gridSize = GRID_SIDE_LENGTH;

  // Build the shader binding table to upload parameters to the device
  gprtBuildShaderBindingTable(context, GPRT_SBT_COMPUTE);

  // Now, compute triangles in parallel with a vertex compute shader
  gprtComputeLaunch(vertexProgram, {(numTriangles + 31u) / 32u,1,1}, {32,1,1}, vertexData);

  // Now that our vertex buffer and index buffer are filled, we can compute
  // our triangles acceleration structure.
  GPRTAccel trianglesAccel = gprtTriangleAccelCreate(context, 1, &trianglesGeom);
  gprtAccelBuild(context, trianglesAccel, GPRT_BUILD_MODE_FAST_TRACE_AND_UPDATE);

  GPRTAccel world = gprtInstanceAccelCreate(context, 1, &trianglesAccel);
  gprtAccelBuild(context, world, GPRT_BUILD_MODE_FAST_TRACE_AND_UPDATE);

  // ##################################################################
  // set the parameters for the rest of our kernels
  // ##################################################################

  // Setup pixel frame buffer
  GPRTBufferOf<uint32_t> frameBuffer = gprtDeviceBufferCreate<uint32_t>(context, fbSize.x * fbSize.y);

  // Raygen program frame buffer
  RayGenData *rayGenData = gprtRayGenGetParameters(rayGen);
  rayGenData->frameBuffer = gprtBufferGetHandle(frameBuffer);

  // Assign the tree handle to our ray generation program's record
  rayGenData->world = gprtAccelGetHandle(world);

  // Miss program checkerboard background colors
  MissProgData *missData = gprtMissGetParameters(miss);
  missData->color0 = float3(0.1f, 0.1f, 0.1f);
  missData->color1 = float3(0.0f, 0.0f, 0.0f);

  // ##################################################################
  // build *SBT* required to trace the groups
  // ##################################################################

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
      pc.camera.pos = lookFrom;
      pc.camera.dir_00 = normalize(lookAt - lookFrom);
      float aspect = float(fbSize.x) / float(fbSize.y);
      pc.camera.dir_du = cosFovy * aspect * normalize(cross(pc.camera.dir_00, lookUp));
      pc.camera.dir_dv = cosFovy * normalize(cross(pc.camera.dir_du, pc.camera.dir_00));
      pc.camera.dir_00 -= 0.5f * pc.camera.dir_du;
      pc.camera.dir_00 -= 0.5f * pc.camera.dir_dv;
    }

    // update time to move primitives. then, rebuild accel.
    vertexData.now = float(gprtGetTime(context));
    gprtComputeLaunch(vertexProgram, {(numTriangles + 31) / 32, 1, 1}, {32,1,1}, vertexData);

    // Now that the vertices have moved, we need to update our bottom level tree.
    // Note, updates should only be used when primitive counts are unchanged and 
    // movement is relatively small.  
    gprtAccelUpdate(context, trianglesAccel);

    // And since the bottom level tree is part of the top level tree, we need
    // to update the top level tree as well
    gprtAccelUpdate(context, world);

    // Calls the GPU raygen kernel function
    gprtRayGenLaunch2D(context, rayGen, fbSize.x, fbSize.y, pc);

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

  gprtBufferDestroy(vertexBuffer);
  gprtBufferDestroy(indexBuffer);
  gprtBufferDestroy(frameBuffer);
  gprtRayGenDestroy(rayGen);
  gprtMissDestroy(miss);
  gprtComputeDestroy(vertexProgram);
  gprtAccelDestroy(trianglesAccel);
  gprtAccelDestroy(world);
  gprtGeomDestroy(trianglesGeom);
  gprtGeomTypeDestroy(trianglesGeomType);
  gprtModuleDestroy(module);
  gprtContextDestroy(context);

  LOG_OK("seems all went OK; app is done, this should be the last output ...");
}
