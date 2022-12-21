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

#define LOG(message)                                                           \
  std::cout << GPRT_TERMINAL_BLUE;                                             \
  std::cout << "#gprt.sample(main): " << message << std::endl;                 \
  std::cout << GPRT_TERMINAL_DEFAULT;
#define LOG_OK(message)                                                        \
  std::cout << GPRT_TERMINAL_LIGHT_BLUE;                                       \
  std::cout << "#gprt.sample(main): " << message << std::endl;                 \
  std::cout << GPRT_TERMINAL_DEFAULT;

extern GPRTProgram s02_deviceCode;

// Vertices and indices of a cube
const int NUM_VERTICES = 8;
float3 vertices[NUM_VERTICES] = {{-1.f, -1.f, -1.f}, {+1.f, -1.f, -1.f},
                                 {-1.f, +1.f, -1.f}, {+1.f, +1.f, -1.f},
                                 {-1.f, -1.f, +1.f}, {+1.f, -1.f, +1.f},
                                 {-1.f, +1.f, +1.f}, {+1.f, +1.f, +1.f}};

const int NUM_INDICES = 12;
int3 indices[NUM_INDICES] = {{0, 1, 3}, {2, 3, 0}, {5, 7, 6}, {5, 6, 4},
                             {0, 4, 5}, {0, 5, 1}, {2, 3, 7}, {2, 7, 6},
                             {1, 5, 7}, {1, 7, 3}, {4, 0, 2}, {4, 2, 6}};

// Several affine transformation matrices to place instances of our cube in the
// world. The first, second, third and third column represent "right", "up", and
// "forward" basis respectively. The last column represents the position.
const int NUM_INSTANCES = 3;
float transforms[NUM_INSTANCES][12] = {
    {0.5f, 0.0f, 0.0f, -1.5f, 0.0f, 0.5f, 0.0f, 0.0f, 0.0f, 0.0f, 0.5f, 0.0f},
    {0.5f, 0.0f, 0.0f, 0.0f, 0.0f, 0.5f, 0.0f, 0.0f, 0.0f, 0.0f, 0.5f, 0.0f},
    {0.5f, 0.0f, 0.0f, 1.5f, 0.0f, 0.5f, 0.0f, 0.0f, 0.0f, 0.0f, 0.5f, 0.0f}};

// initial image resolution
const int2 fbSize = {1400, 460};

// final image output
const char *outFileName = "s02-instances.png";

// Initial camera parameters
float3 lookFrom = {-0.f, 2.f, -3.f};
float3 lookAt = {0.f, 0.f, 0.f};
float3 lookUp = {0.f, 1.f, 0.f};
float cosFovy = 0.66f;

#include <iostream>
int main(int ac, char **av) {
  // This example is very similar to the last, but with one key difference.
  // Now, we create multiple "instances", or copies, of the same bottom level
  // acceleration structure in one top level tree.
  LOG("gprt example '" << av[0] << "' starting up");

  // create a context on the first device:
  gprtRequestWindow(fbSize.x, fbSize.y, "S02 Instances");
  GPRTContext context = gprtContextCreate();
  GPRTModule module = gprtModuleCreate(context, s02_deviceCode);

  // ##################################################################
  // set up all the GPU kernels we want to run
  // ##################################################################

  // -------------------------------------------------------
  // declare geometry type
  // -------------------------------------------------------
  GPRTGeomTypeT<TrianglesGeomData> trianglesGeomType =
      gprtGeomTypeCreate<TrianglesGeomData>(context, GPRT_TRIANGLES);
  gprtGeomTypeSetClosestHitProg(trianglesGeomType, 0, module, "TriangleMesh");

  // -------------------------------------------------------
  // set up miss prog
  // -------------------------------------------------------
  GPRTMissT<MissProgData> miss = gprtMissCreate<MissProgData>(context, module, "miss");

  // -------------------------------------------------------
  // set up ray gen program
  // -------------------------------------------------------
  GPRTRayGenT<RayGenData> rayGen = gprtRayGenCreate<RayGenData>(context, module, "simpleRayGen");

  // ##################################################################
  // set the parameters for those kernels
  // ##################################################################

  // Setup pixel frame buffer
  GPRTBufferT<uint32_t> frameBuffer =
      gprtDeviceBufferCreate<uint32_t>(context, fbSize.x * fbSize.y);
  
  // Raygen program frame buffer
  RayGenData *rayGenData = gprtRayGenGetPointer(rayGen);
  rayGenData->fbPtr = gprtBufferGetHandle(frameBuffer);
  rayGenData->fbSize = fbSize;

  // Miss program checkerboard background colors
  MissProgData *missData = gprtMissGetPointer(miss);
  missData->color0 = float3(0.1f, 0.1f, 0.1f);
  missData->color1 = float3(0.0f, 0.0f, 0.0f);

  LOG("building geometries ...");

  // Create our cube mesh
  GPRTBufferT<float3> vertexBuffer =
      gprtDeviceBufferCreate<float3>(context, NUM_VERTICES, vertices);
  GPRTBufferT<int3> indexBuffer =
      gprtDeviceBufferCreate<int3>(context, NUM_INDICES, indices);
  GPRTGeomT<TrianglesGeomData> trianglesGeom = 
      gprtGeomCreate<TrianglesGeomData>(context, trianglesGeomType);
  gprtTrianglesSetVertices(trianglesGeom, vertexBuffer, NUM_VERTICES);
  gprtTrianglesSetIndices(trianglesGeom, indexBuffer, NUM_INDICES);

  // Here, we additionally set the vertex and index parameters of our goemetry
  // so that we can access these buffers when a ray hits the mesh
  TrianglesGeomData *triangleData = gprtGeomGetPointer(trianglesGeom);
  triangleData->vertex = gprtBufferGetHandle(vertexBuffer);
  triangleData->index = gprtBufferGetHandle(indexBuffer);

  // Place that single cube mesh in a bottom level acceleration structure
  GPRTAccel trianglesAccel =
      gprtTrianglesAccelCreate(context, 1, &trianglesGeom);
  gprtAccelBuild(context, trianglesAccel);

  // We will now create three instances of that cube mesh. On the gpu,
  // we will use the "instance ID" to determine what color each cube should be.

  // First, we create a list of BLAS objects
  GPRTAccel triangleAccelRefs[NUM_INSTANCES] = {trianglesAccel, trianglesAccel,
                                                trianglesAccel};

  // Then, we create a transform buffer, one transform per instance.
  // These transforms are defined at the top of our program, in the "transforms"
  // array referenced by the last parameter here.
  GPRTBufferT<float3x4> transformBuffer = 
    gprtDeviceBufferCreate<float3x4>(context, NUM_INSTANCES, transforms);

  // Finally, we create a top level acceleration structure here.
  GPRTAccel world =
      gprtInstanceAccelCreate(context, NUM_INSTANCES, triangleAccelRefs);
  // Similar to how we set the vertex and index buffers on triangle primitives,
  // we set the transforms buffer here for instance primitives
  gprtInstanceAccelSet3x4Transforms(world, transformBuffer);
  gprtAccelBuild(context, world);

  // Here, we place a reference to our TLAS in the ray generation
  // kernel's parameters, so that we can access that tree when
  // we go to trace our rays.
  rayGenData->world = gprtAccelGetHandle(world);
  
  // ##################################################################
  // build the pipeline and shader binding table
  // ##################################################################
  gprtBuildPipeline(context);
  gprtBuildShaderBindingTable(context);

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

      // Use this to upload all set parameters to our ray tracing device
      gprtBuildShaderBindingTable(context, GPRT_SBT_RAYGEN);
    }

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

  gprtBufferDestroy(vertexBuffer);
  gprtBufferDestroy(indexBuffer);
  gprtBufferDestroy(frameBuffer);
  gprtBufferDestroy(transformBuffer);
  gprtRayGenDestroy(rayGen);
  gprtMissDestroy(miss);
  gprtAccelDestroy(trianglesAccel);
  gprtAccelDestroy(world);
  gprtGeomDestroy(trianglesGeom);
  gprtGeomTypeDestroy(trianglesGeomType);
  gprtModuleDestroy(module);
  gprtContextDestroy(context);

  LOG_OK("seems all went OK; app is done, this should be the last output ...");
}