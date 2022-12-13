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

// our device-side data structures
#include "deviceCode.h"

#define LOG(message)                                                           \
  std::cout << GPRT_TERMINAL_BLUE;                                             \
  std::cout << "#gprt.sample(main): " << message << std::endl;                 \
  std::cout << GPRT_TERMINAL_DEFAULT;
#define LOG_OK(message)                                                        \
  std::cout << GPRT_TERMINAL_LIGHT_BLUE;                                       \
  std::cout << "#gprt.sample(main): " << message << std::endl;                 \
  std::cout << GPRT_TERMINAL_DEFAULT;

extern GPRTProgram s01_deviceCode;

// Vertices are the points that define our triangles
const int NUM_VERTICES = 3;
float3 vertices[NUM_VERTICES] = {
    {-1.f, -.5f, 0.f},
    {+1.f, -.5f, 0.f},
    {0.f, +.5f, 0.f},
};

// Indices connect those vertices together.
// Here, vertex 0 connects to 1, which connects to 2 to form a triangle.
const int NUM_INDICES = 1;
int3 indices[NUM_INDICES] = {{0, 1, 2}};

// initial image resolution
const int2 fbSize = {1400, 460};

// final image output
const char *outFileName = "s01-singleTriangle.png";

// Initial camera parameters
float3 lookFrom = {0.f, 0.f, -4.f};
float3 lookAt = {0.f, 0.f, 0.f};
float3 lookUp = {0.f, -1.f, 0.f};
float cosFovy = 0.66f;

#include <iostream>
int main(int ac, char **av) {
  LOG("gprt example '" << av[0] << "' starting up");

  LOG("building module, programs, and pipeline");

  // create a context on the first device:
  gprtRequestWindow(fbSize.x, fbSize.y, "S01 Single Triangle");
  GPRTContext context = gprtContextCreate();
  GPRTModule module = gprtModuleCreate(context, s01_deviceCode);

  // ##################################################################
  // set up all the GPU kernels we want to run
  // ##################################################################

  // First, we need to declare our geometry type.
  // This includes all GPU kernels tied to the geometry, as well as the
  // parameters passed to the geometry when hit by rays.
  GPRTVarDecl trianglesGeomVars[] = {
      {/* For now, no parameters. This sentinel to mark end of list */}};
  GPRTGeomType trianglesGeomType = gprtGeomTypeCreate(
      context, GPRT_TRIANGLES, sizeof(TrianglesGeomData), trianglesGeomVars);
  gprtGeomTypeSetClosestHitProg(trianglesGeomType, 0, module, "TriangleMesh");

  // We'll also need a ray generation program.
  GPRTVarDecl rayGenVars[] = {
      {"fbSize", GPRT_INT2, GPRT_OFFSETOF(RayGenData, fbSize)},
      {"fbPtr", GPRT_BUFFER, GPRT_OFFSETOF(RayGenData, fbPtr)},
      {"world", GPRT_ACCEL, GPRT_OFFSETOF(RayGenData, world)},
      {"camera.pos", GPRT_FLOAT3, GPRT_OFFSETOF(RayGenData, camera.pos)},
      {"camera.dir_00", GPRT_FLOAT3, GPRT_OFFSETOF(RayGenData, camera.dir_00)},
      {"camera.dir_du", GPRT_FLOAT3, GPRT_OFFSETOF(RayGenData, camera.dir_du)},
      {"camera.dir_dv", GPRT_FLOAT3, GPRT_OFFSETOF(RayGenData, camera.dir_dv)},
      {/* sentinel to mark end of list */}};
  GPRTRayGen rayGen = gprtRayGenCreate(context, module, "simpleRayGen",
                                       sizeof(RayGenData), rayGenVars, -1);

  // Finally, we need a "miss" program, which will be called when
  // a ray misses all triangles. Just like geometry declarations
  // and ray tracing programs, miss programs have parameters
  GPRTVarDecl missProgVars[] = {
      {"color0", GPRT_FLOAT3, GPRT_OFFSETOF(MissProgData, color0)},
      {"color1", GPRT_FLOAT3, GPRT_OFFSETOF(MissProgData, color1)},
      {/* sentinel to mark end of list */}};
  GPRTMiss miss = gprtMissCreate(context, module, "miss", sizeof(MissProgData),
                                 missProgVars, -1);

  // ##################################################################
  // set the parameters for those kernels
  // ##################################################################

  // Setup pixel frame buffer
  GPRTBuffer frameBuffer =
      gprtDeviceBufferCreate(context, GPRT_INT, fbSize.x * fbSize.y);
  gprtRayGenSetBuffer(rayGen, "fbPtr", frameBuffer);
  gprtRayGenSet2iv(rayGen, "fbSize", (int32_t *)&fbSize);

  // Miss program checkerboard background colors
  gprtMissSet3f(miss, "color0", 0.1f, 0.1f, 0.1f);
  gprtMissSet3f(miss, "color1", 0.0f, 0.0f, 0.0f);

  LOG("building geometries ...");

  // The vertex and index buffers here define the triangle vertices
  // and how those vertices are connected together.
  GPRTBuffer vertexBuffer =
      gprtDeviceBufferCreate(context, GPRT_FLOAT3, NUM_VERTICES, vertices);
  GPRTBuffer indexBuffer =
      gprtDeviceBufferCreate(context, GPRT_INT3, NUM_INDICES, indices);

  // Next, we will create an instantiation of our geometry declaration.
  GPRTGeom trianglesGeom = gprtGeomCreate(context, trianglesGeomType);
  // We use these calls to tell the geometry what buffers store triangle
  // indices and vertices
  gprtTrianglesSetVertices(trianglesGeom, vertexBuffer, NUM_VERTICES,
                           sizeof(float3), 0);
  gprtTrianglesSetIndices(trianglesGeom, indexBuffer, NUM_INDICES, sizeof(int3),
                          0);

  // Once we have our geometry, we need to place that geometry into an
  // acceleration structure. These acceleration structures allow rays to
  // determine which triangle the ray hits in a sub-linear amount of time.
  // This first acceleration structure level is called a bottom level
  // acceleration structure, or a BLAS.
  GPRTAccel trianglesAccel =
      gprtTrianglesAccelCreate(context, 1, &trianglesGeom);
  gprtAccelBuild(context, trianglesAccel);

  // We can then make multiple "instances", or copies, of that BLAS in
  // a top level acceleration structure, or a TLAS. (we'll cover this later.)
  // Rays can only be traced into TLAS, so for now we just make one BLAS
  // instance.
  GPRTAccel world = gprtInstanceAccelCreate(context, 1, &trianglesAccel);
  gprtAccelBuild(context, world);

  // Here, we place a reference to our TLAS in the ray generation
  // kernel's parameters, so that we can access that tree when
  // we go to trace our rays.
  gprtRayGenSetAccel(rayGen, "world", world);

  // ##################################################################
  // build the pipeline and shader binding table
  // ##################################################################

  // We must build the pipeline after all geometry instances are created.
  // The pipeline contains programs for each geometry that might be hit by
  // a ray.
  gprtBuildPipeline(context);

  // Next, the shader binding table is used to assign parameters to our ray
  // generation and miss programs. We also use the shader binding table to
  // map parameters to geometry depending on the ray type and instance.
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
      gprtRayGenSet3fv(rayGen, "camera.pos", (float *)&camera_pos);
      gprtRayGenSet3fv(rayGen, "camera.dir_00", (float *)&camera_d00);
      gprtRayGenSet3fv(rayGen, "camera.dir_du", (float *)&camera_ddu);
      gprtRayGenSet3fv(rayGen, "camera.dir_dv", (float *)&camera_ddv);

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
