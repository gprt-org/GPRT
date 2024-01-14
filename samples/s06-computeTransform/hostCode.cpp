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

// for generating meshes
#include <generator.hpp>
using namespace generator;

#define LOG(message)                                                                                                   \
  std::cout << GPRT_TERMINAL_BLUE;                                                                                     \
  std::cout << "#gprt.sample(main): " << message << std::endl;                                                         \
  std::cout << GPRT_TERMINAL_DEFAULT;
#define LOG_OK(message)                                                                                                \
  std::cout << GPRT_TERMINAL_LIGHT_BLUE;                                                                               \
  std::cout << "#gprt.sample(main): " << message << std::endl;                                                         \
  std::cout << GPRT_TERMINAL_DEFAULT;

extern GPRTProgram s06_deviceCode;

// initial image resolution
const int2 fbSize = {1400, 460};

const char *outFileName = "s06-computeTransform.png";

float3 lookFrom = {-2.f, -2.f, 0.f};
float3 lookAt = {0.f, 0.f, 0.f};
float3 lookUp = {0.f, 0.f, -1.f};
float cosFovy = 0.3f;

// A class we'll use to quickly generate meshes and bottom level trees
template <typename T> struct Mesh {
  std::vector<float3> vertices;
  std::vector<uint3> indices;
  GPRTBufferOf<float3> vertexBuffer;
  GPRTBufferOf<uint3> indexBuffer;
  GPRTGeomOf<TrianglesGeomData> geometry;
  GPRTAccel accel;

  Mesh(){};
  Mesh(GPRTContext context, GPRTGeomTypeOf<TrianglesGeomData> geomType, T generator) {
    // Use the generator to generate vertices and indices
    auto vertGenerator = generator.vertices();
    auto triGenerator = generator.triangles();
    while (!vertGenerator.done()) {
      auto vertex = vertGenerator.generate();
      auto position = vertex.position;
      vertices.push_back(float3(position[0], position[1], position[2]));
      vertGenerator.next();
    }
    while (!triGenerator.done()) {
      Triangle triangle = triGenerator.generate();
      auto vertices = triangle.vertices;
      indices.push_back(uint3(vertices[0], vertices[1], vertices[2]));
      triGenerator.next();
    }

    // Upload those to the device, create the geometry
    vertexBuffer = gprtDeviceBufferCreate<float3>(context, vertices.size(), vertices.data());
    indexBuffer = gprtDeviceBufferCreate<uint3>(context, indices.size(), indices.data());
    geometry = gprtGeomCreate(context, geomType);

    gprtTrianglesSetVertices(geometry, vertexBuffer, vertices.size());
    gprtTrianglesSetIndices(geometry, indexBuffer, indices.size());
    TrianglesGeomData *geomData = gprtGeomGetParameters(geometry);
    geomData->vertex = gprtBufferGetHandle(vertexBuffer);
    geomData->index = gprtBufferGetHandle(indexBuffer);

    // Build the bottom level acceleration structure
    accel = gprtTriangleAccelCreate(context, 1, &geometry);
    gprtAccelBuild(context, accel, GPRT_BUILD_MODE_FAST_TRACE_NO_UPDATE, /*allow compaction*/true);
    // gprtAccelCompact(context, accel);
  };

  void cleanupMesh() {
    gprtAccelDestroy(accel);
    gprtGeomDestroy(geometry);
    gprtBufferDestroy(vertexBuffer);
    gprtBufferDestroy(indexBuffer);
  };
};

#include <iostream>
int
main(int ac, char **av) {
  // In this example, we'll use a instance transform program to manipulate
  // instance positions in parallel on the GPU. The point of this example
  // is to show that the primitives of an instance tree can be efficiently
  // manipulated, just like any other primitive.

  // create a context on the first device:
  gprtRequestWindow(fbSize.x, fbSize.y, "S06 Compute Transform");
  GPRTContext context = gprtContextCreate(nullptr, 1);
  GPRTModule module = gprtModuleCreate(context, s06_deviceCode);

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
  // set up instance transform program to animate instances
  // -------------------------------------------------------
  auto transformProgram = gprtComputeCreate<PushConstants>(context, module, "Transform");

  // -------------------------------------------------------
  // set up ray gen program
  // -------------------------------------------------------
  GPRTRayGenOf<RayGenData> rayGen = gprtRayGenCreate<RayGenData>(context, module, "RayGen");

  // -------------------------------------------------------
  // set up miss
  // -------------------------------------------------------
  GPRTMissOf<MissProgData> miss = gprtMissCreate<MissProgData>(context, module, "miss");

  // ------------------------------------------------------------------
  // bottom level mesh instances
  // ------------------------------------------------------------------

  // For this example, we'll be animating a grid of utah teapot meshes.

  // We begin by making one teapot mesh, storing that mesh in a bottom
  // level acceleration structure.
  Mesh<TeapotMesh> instanceMesh(context, trianglesGeomType, TeapotMesh{12});

  // Next, we'll create a grid of references to the same bottom level
  // acceleration structure. This saves memory and improves performance over
  // creating duplicate meshes.
  uint32_t numInstances = 50 * 50;
  std::vector<GPRTAccel> instanceTrees(numInstances);
  for (int i = 0; i < numInstances; ++i) {
    instanceTrees[i] = instanceMesh.accel;
  }

  // ------------------------------------------------------------------
  // the instance acceleration structure
  // ------------------------------------------------------------------

  // Similar to the computeVertex example, it is _okay_ to give our instance
  // acceleration structure an unpopulated buffer of transforms, so long as
  // those transforms are filled in before we go to build our acceleration
  // structure.
  GPRTBufferOf<float3x4> transformBuffer = gprtHostBufferCreate<float3x4>(context, numInstances, nullptr);
  GPRTAccel world = gprtInstanceAccelCreate(context, numInstances, instanceTrees.data());
  gprtInstanceAccelSet3x4Transforms(world, transformBuffer);

  // Parameters for our transform program that'll animate our transforms
  pc.transforms = gprtBufferGetHandle(transformBuffer);
  pc.numTransforms = numInstances;

  // Build the shader binding table to upload parameters to the device
  gprtBuildShaderBindingTable(context, GPRT_SBT_COMPUTE);

  // Now, compute transforms in parallel with a transform compute shader
  gprtComputeLaunch(transformProgram, {numInstances,1,1}, {1,1,1}, pc);

  // Now that the transforms are set, we can build our top level acceleration
  // structure
  gprtAccelBuild(context, world, GPRT_BUILD_MODE_FAST_TRACE_AND_UPDATE);

  // ##################################################################
  // set the parameters for the rest of our kernels
  // ##################################################################

  // Setup pixel frame buffer
  GPRTBuffer frameBuffer = gprtDeviceBufferCreate(context, sizeof(uint32_t), fbSize.x * fbSize.y);

  // Raygen program frame buffer
  RayGenData *rayGenData = gprtRayGenGetParameters(rayGen);
  rayGenData->frameBuffer = gprtBufferGetHandle(frameBuffer);
  rayGenData->world = gprtAccelGetHandle(world);

  // Miss program checkerboard background colors
  MissProgData *missData = gprtMissGetParameters(miss);
  missData->color0 = float3(0.1f, 0.1f, 0.1f);
  missData->color1 = float3(0.0f, 0.0f, 0.0f);

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

    // update time to move instance transforms. Then, update only instance
    // accel.
    pc.now = float(gprtGetTime(context));
    gprtComputeLaunch(transformProgram, {numInstances, 1, 1}, {1,1,1}, pc);
    gprtAccelUpdate(context, world);

    // Now, trace rays
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

  gprtBufferDestroy(frameBuffer);
  gprtBufferDestroy(transformBuffer);
  gprtRayGenDestroy(rayGen);
  gprtMissDestroy(miss);
  gprtComputeDestroy(transformProgram);
  gprtAccelDestroy(world);
  instanceMesh.cleanupMesh();
  gprtGeomTypeDestroy(trianglesGeomType);
  gprtModuleDestroy(module);
  gprtContextDestroy(context);

  LOG_OK("seems all went OK; app is done, this should be the last output ...");
}
