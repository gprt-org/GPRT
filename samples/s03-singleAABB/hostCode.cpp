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

extern GPRTProgram s03_deviceCode;

// The extents of our bounding box
float3 aabbPositions[2] = {{-1.0f, -1.0f, -1.0f}, {1.0f, 1.0f, 1.0f}};

// initial image resolution
const int2 fbSize = {1400, 460};

// final image output
const char *outFileName = "s03-singleAABB.png";

// Initial camera parameters
float3 lookFrom = {3.5f, 3.5f, 3.5f};
float3 lookAt = {0.f, 0.f, 0.f};
float3 lookUp = {0.f, -1.f, 0.f};
float cosFovy = 0.66f;

#include <iostream>
int main(int ac, char **av) {
  // In this example, we will create an axis aligned bounding box (AABB). These
  // are more general than triangle primitives, and can be used for custom
  // primitive types.
  LOG("gprt example '" << av[0] << "' starting up");

  // create a context on the first device:
  gprtRequestWindow(fbSize.x, fbSize.y, "S03 Single AABB");
  GPRTContext context = gprtContextCreate();
  GPRTModule module = gprtModuleCreate(context, s03_deviceCode);

  // ##################################################################
  // set up all the GPU kernels we want to run
  // ##################################################################

  // -------------------------------------------------------
  // declare geometry type
  // -------------------------------------------------------
  GPRTGeomTypeT<AABBGeomData> aabbGeomType =
      gprtGeomTypeCreate<AABBGeomData>(context,
                                       GPRT_AABBS /* <- This is new! */);
  gprtGeomTypeSetClosestHitProg(aabbGeomType, 0, module, "AABBClosestHit");
  gprtGeomTypeSetIntersectionProg(aabbGeomType, 0, module, "AABBIntersection");

  // -------------------------------------------------------
  // set up miss
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

  // Create our AABB geometry. Every AABB is defined using two float3's. The
  // first float3 defines the bottom lower left near corner, and the second
  // float3 defines the upper far right corner.
  GPRTBufferT<float3> aabbPositionsBuffer =
      gprtDeviceBufferCreate<float3>(context, 2, aabbPositions);
  GPRTGeomT<AABBGeomData> aabbGeom = gprtGeomCreate<AABBGeomData>(context, aabbGeomType);
  gprtAABBsSetPositions(aabbGeom, aabbPositionsBuffer, 1 /* just one aabb */);

  // Note, we must create an "AABB" accel rather than a triangles accel.
  GPRTAccel aabbAccel = gprtAABBAccelCreate(context, 1, &aabbGeom);
  gprtAccelBuild(context, aabbAccel);

  // triangle and AABB accels can be combined in a top level tree
  GPRTAccel world = gprtInstanceAccelCreate(context, 1, &aabbAccel);
  gprtAccelBuild(context, world);

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

    // If a window exists, presents the frame buffer here to that window
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

  gprtBufferDestroy(aabbPositionsBuffer);
  gprtBufferDestroy(frameBuffer);
  gprtRayGenDestroy(rayGen);
  gprtMissDestroy(miss);
  gprtAccelDestroy(aabbAccel);
  gprtAccelDestroy(world);
  gprtGeomDestroy(aabbGeom);
  gprtGeomTypeDestroy(aabbGeomType);
  gprtModuleDestroy(module);
  gprtContextDestroy(context);

  LOG_OK("seems all went OK; app is done, this should be the last output ...");
}
