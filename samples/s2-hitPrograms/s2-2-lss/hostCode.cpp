// This program sets up a single geometric object, a mesh for a cube, and
// its acceleration structure, then ray traces it.

#include <gprt.h>      // Public GPRT API
#include "sharedCode.h" // Shared data between host and device

extern GPRTProgram s2_2_deviceCode;

// The positions and radii of our line-swept sphere primitive
const int NUM_VERTICES = 11;
float4 vertices[NUM_VERTICES] = {
    float4(-4 + 0 * 0.8, sin(-4 + 0 * 0.8), 0.0, 0.1),  float4(-4 + 1 * 0.8, sin(-4 + 1 * 0.8), 0.0, 0.2),
    float4(-4 + 2 * 0.8, sin(-4 + 2 * 0.8), 0.0, 0.3),  float4(-4 + 3 * 0.8, sin(-4 + 3 * 0.8), 0.0, 0.4),
    float4(-4 + 4 * 0.8, sin(-4 + 4 * 0.8), 0.0, 0.5),  float4(-4 + 5 * 0.8, sin(-4 + 5 * 0.8), 0.0, 0.6),
    float4(-4 + 6 * 0.8, sin(-4 + 6 * 0.8), 0.0, 0.5),  float4(-4 + 7 * 0.8, sin(-4 + 7 * 0.8), 0.0, 0.4),
    float4(-4 + 8 * 0.8, sin(-4 + 8 * 0.8), 0.0, 0.3),  float4(-4 + 9 * 0.8, sin(-4 + 9 * 0.8), 0.0, 0.2),
    float4(-4 + 10 * 0.8, sin(-4 + 10 * 0.8), 0.0, 0.1)};

// Indices connect those vertices together.
// Here, vertex 0 connects to 1.
const int NUM_INDICES = 10;
uint2 indices[NUM_INDICES] = {{0, 1}, {1, 2}, {2, 3}, {3, 4}, {4, 5}, {5, 6}, {6, 7}, {7, 8}, {8, 9}, {9, 10}};

// initial image resolution
const int2 fbSize = {1400, 460};

// final image output
const char *outFileName = "s03-lineSweptSpheres.png";

// Initial camera parameters
float3 lookFrom = {0.0f, 0.0f, 6.0f};
float3 lookAt = {0.f, 0.f, 0.f};
float3 lookUp = {0.f, -1.f, 0.f};
float cosFovy = 0.66f;

#include <iostream>
int
main(int ac, char **av) {
  // In this example, we will create an axis aligned bounding box (AABB). These
  // are more general than triangle primitives, and can be used for custom
  // primitive types.

  // create a context on the first device:
  gprtRequestWindow(fbSize.x, fbSize.y, "S03 Line-Swept Spheres");
  GPRTContext context = gprtContextCreate();
  GPRTModule module = gprtModuleCreate(context, s2_2_deviceCode);

  // ##################################################################
  // set up all the GPU kernels we want to run
  // ##################################################################

  // -------------------------------------------------------
  // declare geometry type
  // -------------------------------------------------------
  GPRTGeomTypeOf<LSSGeomData> lssGeomType = gprtGeomTypeCreate<LSSGeomData>(context, GPRT_LSS /* <- This is new! */);
  gprtGeomTypeSetClosestHitProg(lssGeomType, 0, module, "LSSClosestHit");

  // -------------------------------------------------------
  // set up miss
  // -------------------------------------------------------
  GPRTMissOf<MissProgData> miss = gprtMissCreate<MissProgData>(context, module, "miss");

  // -------------------------------------------------------
  // set up ray gen program
  // -------------------------------------------------------
  GPRTRayGenOf<RayGenData> rayGen = gprtRayGenCreate<RayGenData>(context, module, "raygen");
  // ##################################################################
  // set the parameters for those kernels
  // ##################################################################

  // Setup pixel frame buffer
  GPRTBufferOf<uint32_t> frameBuffer = gprtDeviceBufferCreate<uint32_t>(context, fbSize.x * fbSize.y);

  // Raygen program frame buffer
  RayGenData *rayGenData = gprtRayGenGetParameters(rayGen);
  rayGenData->frameBuffer = gprtBufferGetDevicePointer(frameBuffer);

  // Miss program checkerboard background colors
  MissProgData *missData = gprtMissGetParameters(miss);
  missData->color0 = float3(0.1f, 0.1f, 0.1f);
  missData->color1 = float3(0.0f, 0.0f, 0.0f);


  // Create our LSS geometry. Every LSS is defined using two float4's. The
  // first float4 defines a starting "xyz" position and starting radius "w", and then
  // the second float4 defines the end position and end radius.
  GPRTBufferOf<float4> vertexBuffer = gprtDeviceBufferCreate<float4>(context, NUM_VERTICES, vertices);
  GPRTBufferOf<uint2> indexBuffer = gprtDeviceBufferCreate<uint2>(context, NUM_INDICES, indices);
  GPRTGeomOf<LSSGeomData> lssGeom = gprtGeomCreate<LSSGeomData>(context, lssGeomType);
  gprtLSSSetVertices(lssGeom, vertexBuffer, NUM_VERTICES);
  gprtLSSSetIndices(lssGeom, indexBuffer, NUM_INDICES);

  // Note, we must create an "AABB" accel rather than a triangles accel.
  GPRTAccel lssAccel = gprtLSSAccelCreate(context, 1, &lssGeom);
  gprtAccelBuild(context, lssAccel, GPRT_BUILD_MODE_FAST_TRACE_NO_UPDATE);

  gprt::Instance instance = gprtAccelGetInstance(lssAccel);
  GPRTBufferOf<gprt::Instance> instanceBuffer = gprtDeviceBufferCreate(context, 1, &instance);

  // triangle and AABB accels can be combined in a top level tree
  GPRTAccel world = gprtInstanceAccelCreate(context, 1, instanceBuffer);
  gprtAccelBuild(context, world, GPRT_BUILD_MODE_FAST_TRACE_NO_UPDATE);

  rayGenData->world = gprtAccelGetHandle(world);

  LSSGeomData lssParams;
  lssParams.indices = gprtBufferGetDevicePointer(indexBuffer);
  lssParams.vertices = gprtBufferGetDevicePointer(vertexBuffer);
  gprtGeomSetParameters(lssGeom, &lssParams);

  gprtBuildShaderBindingTable(context);

  // ##################################################################
  // now that everything is ready: launch it ....
  // ##################################################################


  // Structure of parameters that change each frame. We can edit these
  // without rebuilding the shader binding table.
  PushConstants pc;

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
      float4x4 rotationMatrixX = math::matrixFromRotation(xAngle, lookUp);
      position = (mul(rotationMatrixX, (position - pivot))) + pivot;

      // step 3: Rotate the camera around the pivot point on the second axis.
      float3 lookRight = cross(lookUp, normalize(pivot - position).xyz());
      float4x4 rotationMatrixY = math::matrixFromRotation(yAngle, lookRight);
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

    pc.time = gprtGetTime(context);

    // Calls the GPU raygen kernel function
    gprtRayGenLaunch2D(context, rayGen, fbSize.x, fbSize.y, pc);

    // If a window exists, presents the frame buffer here to that window
    gprtBufferPresent(context, frameBuffer);
  }
  // returns true if "X" pressed or if in "headless" mode
  while (!gprtWindowShouldClose(context));

  // Save final frame to an image
  gprtBufferSaveImage(frameBuffer, fbSize.x, fbSize.y, outFileName);

  // ##################################################################
  // and finally, clean up
  // ##################################################################

  gprtContextDestroy(context);
}
