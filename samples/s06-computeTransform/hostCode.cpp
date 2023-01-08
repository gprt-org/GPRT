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

#define LOG(message)                                                           \
  std::cout << GPRT_TERMINAL_BLUE;                                             \
  std::cout << "#gprt.sample(main): " << message << std::endl;                 \
  std::cout << GPRT_TERMINAL_DEFAULT;
#define LOG_OK(message)                                                        \
  std::cout << GPRT_TERMINAL_LIGHT_BLUE;                                       \
  std::cout << "#gprt.sample(main): " << message << std::endl;                 \
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
    vertexBuffer = gprtDeviceBufferCreate<float3>(context, vertices.size(),
                                          vertices.data());
    indexBuffer = gprtDeviceBufferCreate<uint3>(context, indices.size(),
                                         indices.data());
    geometry = gprtGeomCreate(context, geomType);

    gprtTrianglesSetVertices(geometry, vertexBuffer, vertices.size());
    gprtTrianglesSetIndices(geometry, indexBuffer, indices.size());
    TrianglesGeomData *geomData = gprtGeomGetPointer(geometry);
    geomData->vertex = gprtBufferGetHandle(vertexBuffer);
    geomData->index = gprtBufferGetHandle(indexBuffer);
    
    // Build the bottom level acceleration structure
    accel = gprtTrianglesAccelCreate(context, 1, &geometry);
    gprtAccelBuild(context, accel);
  };

  void cleanupMesh() {
    gprtAccelDestroy(accel);
    gprtGeomDestroy(geometry);
    gprtBufferDestroy(vertexBuffer);
    gprtBufferDestroy(indexBuffer);
  };
};

#include <iostream>
int main(int ac, char **av) {
  // In this example, we'll use a instance transform program to manipulate
  // instance positions in parallel on the GPU. The point of this example
  // is to show that the primitives of an instance tree can be efficiently
  // manipulated, just like any other primitive.

  // create a context on the first device:
  gprtRequestWindow(fbSize.x, fbSize.y, "S06 Compute Transform");
  GPRTContext context = gprtContextCreate(nullptr, 1);
  GPRTModule module = gprtModuleCreate(context, s06_deviceCode);

  // ##################################################################
  // set up all the GPU kernels we want to run
  // ##################################################################

  // -------------------------------------------------------
  // Setup geometry types
  // -------------------------------------------------------

  GPRTGeomTypeOf<TrianglesGeomData> trianglesGeomType = 
    gprtGeomTypeCreate<TrianglesGeomData>(context, GPRT_TRIANGLES);
  gprtGeomTypeSetClosestHitProg(trianglesGeomType, 0, module, "ClosestHit");

  // -------------------------------------------------------
  // set up instance transform program to animate instances
  // -------------------------------------------------------
  GPRTComputeOf<TransformData> transformProgram = 
    gprtComputeCreate<TransformData>(context, module, "Transform");

  // -------------------------------------------------------
  // set up ray gen program
  // -------------------------------------------------------
  GPRTRayGenOf<RayGenData> rayGen = gprtRayGenCreate<RayGenData>(context, module, "RayGen");

  // -------------------------------------------------------
  // set up miss
  // -------------------------------------------------------
  GPRTMissOf<MissProgData> miss =
      gprtMissCreate<MissProgData>(context, module, "miss");

  // Note, we'll need to call this again after creating our acceleration
  // structures, as acceleration structures will introduce new shader
  // binding table records to the pipeline.
  gprtBuildPipeline(context);

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
  int numInstances = 50 * 50;
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
  GPRTBufferOf<float3x4> transformBuffer =
      gprtDeviceBufferCreate<float3x4>(context, numInstances, nullptr);
  GPRTAccel world =
      gprtInstanceAccelCreate(context, numInstances, instanceTrees.data());
  gprtInstanceAccelSet3x4Transforms(world, transformBuffer);

  // Parameters for our transform program that'll animate our transforms
  TransformData* transformData = gprtComputeGetPointer(transformProgram);
  transformData->transforms = gprtBufferGetHandle(transformBuffer);
  transformData->numTransforms = numInstances;
  transformData->now = 0.f;

  // Build the shader binding table to upload parameters to the device
  gprtBuildShaderBindingTable(context, GPRT_SBT_COMPUTE);

  // Now, compute transforms in parallel with a transform compute shader
  gprtComputeLaunch1D(context, transformProgram, numInstances);

  // Now that the transforms are set, we can build our top level acceleration
  // structure
  gprtAccelBuild(context, world);

  // ##################################################################
  // set the parameters for the rest of our kernels
  // ##################################################################

  // Setup pixel frame buffer
  GPRTBuffer frameBuffer =
      gprtDeviceBufferCreate(context, sizeof(uint32_t), fbSize.x * fbSize.y);
  
  // Raygen program frame buffer
  RayGenData *rayGenData = gprtRayGenGetPointer(rayGen);
  rayGenData->frameBuffer = gprtBufferGetHandle(frameBuffer);

  // Miss program checkerboard background colors
  MissProgData *missData = gprtMissGetPointer(miss);
  missData->color0 = float3(0.1f, 0.1f, 0.1f);
  missData->color1 = float3(0.0f, 0.0f, 0.0f);

  // ##################################################################
  // build *SBT* required to trace the groups
  // ##################################################################

  // re-build the pipeline to account for newly introduced geometry
  gprtBuildPipeline(context);
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
      float xAngle = (lastxpos - xpos) * deltaAngleX;
      float yAngle = (lastypos - ypos) * deltaAngleY;

      // step 2: Rotate the camera around the pivot point on the first axis.
      float4x4 rotationMatrixX = rotation_matrix(rotation_quat(lookUp, xAngle));
      position = (mul(rotationMatrixX, (position - pivot))) + pivot;

      // step 3: Rotate the camera around the pivot point on the second axis.
      float3 lookRight = cross(lookUp, normalize(pivot - position).xyz());
      float4x4 rotationMatrixY =
          rotation_matrix(rotation_quat(lookRight, yAngle));
      lookFrom = ((mul(rotationMatrixY, (position - pivot))) + pivot).xyz();

      // ----------- compute variable values  ------------------
      float3 camera_pos = lookFrom;
      float3 camera_d00 = normalize(lookAt - lookFrom);
      float aspect = float(fbSize.x) / float(fbSize.y);
      float3 camera_ddu =
          cosFovy * aspect * normalize(cross(camera_d00, lookUp));
      float3 camera_ddv = cosFovy * normalize(cross(camera_ddu, camera_d00));
      camera_d00 -= 0.5f * camera_ddu;
      camera_d00 -= 0.5f * camera_ddv;

      // ----------- set variables  ----------------------------
      RayGenData *raygenData = gprtRayGenGetPointer(rayGen);
      raygenData->camera.pos = camera_pos;
      raygenData->camera.dir_00 = camera_d00;
      raygenData->camera.dir_du = camera_ddu;
      raygenData->camera.dir_dv = camera_ddv;

      gprtBuildShaderBindingTable(context, GPRT_SBT_RAYGEN);
    }

    // update time to move instance transforms. Then, rebuild only instance
    // accel.
    transformData->now = float(gprtGetTime(context));
    gprtBuildShaderBindingTable(context, GPRT_SBT_COMPUTE);
    gprtComputeLaunch1D(context, transformProgram, numInstances);
    gprtAccelBuild(context, world);

    rayGenData->world = gprtAccelGetHandle(world);
    gprtBuildShaderBindingTable(context, GPRT_SBT_HITGROUP);

    // Note! we don't need to rebuild the pipeline here, since no geometry was
    // made or destroyed, only updated.

    // Now, trace rays
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
