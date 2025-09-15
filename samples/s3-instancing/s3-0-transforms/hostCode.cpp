#include <gprt.h>      // Public GPRT API
#include "sharedCode.h" // Shared data between host and device

extern GPRTProgram s3_0_deviceCode;

// Vertices and indices of a cube
const int NUM_VERTICES = 8;
float3 vertices[NUM_VERTICES] = {{-1.f, -1.f, -1.f}, {+1.f, -1.f, -1.f}, {-1.f, +1.f, -1.f}, {+1.f, +1.f, -1.f},
                                 {-1.f, -1.f, +1.f}, {+1.f, -1.f, +1.f}, {-1.f, +1.f, +1.f}, {+1.f, +1.f, +1.f}};

const int NUM_INDICES = 12;
uint3 indices[NUM_INDICES] = {{0, 1, 3}, {2, 3, 0}, {5, 7, 6}, {5, 6, 4}, {0, 4, 5}, {0, 5, 1},
                             {2, 3, 7}, {2, 7, 6}, {1, 5, 7}, {1, 7, 3}, {4, 0, 2}, {4, 2, 6}};

// Several affine transformation matrices to place instances of our cube in the
// world. The first, second, third and third column represent "right", "up", and
// "forward" basis respectively. The last column represents the position.
const int NUM_INSTANCES = 3;
float3x4 transforms[NUM_INSTANCES] = {{0.5f, 0.0f, 0.0f, -1.5f, 0.0f, 0.5f, 0.0f, 0.0f, 0.0f, 0.0f, 0.5f, 0.0f},
                                      {0.5f, 0.0f, 0.0f, 0.0f, 0.0f, 0.5f, 0.0f, 0.0f, 0.0f, 0.0f, 0.5f, 0.0f},
                                      {0.5f, 0.0f, 0.0f, 1.5f, 0.0f, 0.5f, 0.0f, 0.0f, 0.0f, 0.0f, 0.5f, 0.0f}};

// initial image resolution
const int2 fbSize = {1400, 460};

// final image output
const char *outFileName = "s3-0-transforms.png";

// Initial camera parameters
float3 lookFrom = {-0.f, 2.f, -3.f};
float3 lookAt = {0.f, 0.f, 0.f};
float3 lookUp = {0.f, 1.f, 0.f};
float cosFovy = 0.66f;

#include <iostream>
int
main(int ac, char **av) {
  // This example is very similar to the last, but with one key difference.
  // Now, we create multiple "instances", or copies, of the same bottom level
  // acceleration structure in one top level tree.

  // create a context on the first device:
  gprtRequestWindow(fbSize.x, fbSize.y, "S02 Instances");
  GPRTContext context = gprtContextCreate();
  GPRTModule module = gprtModuleCreate(context, s3_0_deviceCode);

  // ##################################################################
  // set up all the GPU kernels we want to run
  // ##################################################################

  // -------------------------------------------------------
  // declare geometry type
  // -------------------------------------------------------
  GPRTGeomTypeOf<TrianglesGeomData> trianglesGeomType = gprtGeomTypeCreate<TrianglesGeomData>(context, GPRT_TRIANGLES);
  gprtGeomTypeSetClosestHitProg(trianglesGeomType, 0, module, "TriangleMesh");

  // -------------------------------------------------------
  // set up miss prog
  // -------------------------------------------------------
  GPRTMissOf<MissProgData> miss = gprtMissCreate<MissProgData>(context, module, "miss");

  // -------------------------------------------------------
  // set up ray gen program
  // -------------------------------------------------------
  GPRTRayGenOf<RayGenData> rayGen = gprtRayGenCreate<RayGenData>(context, module, "simpleRayGen");

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


  // Create our cube mesh
  GPRTBufferOf<float3> vertexBuffer = gprtDeviceBufferCreate<float3>(context, NUM_VERTICES, vertices);
  GPRTBufferOf<uint3> indexBuffer = gprtDeviceBufferCreate<uint3>(context, NUM_INDICES, indices);
  GPRTGeomOf<TrianglesGeomData> trianglesGeom = gprtGeomCreate<TrianglesGeomData>(context, trianglesGeomType);
  gprtTrianglesSetVertices(trianglesGeom, vertexBuffer, NUM_VERTICES);
  gprtTrianglesSetIndices(trianglesGeom, indexBuffer, NUM_INDICES);

  // Here, we additionally set the vertex and index parameters of our goemetry
  // so that we can access these buffers when a ray hits the mesh
  TrianglesGeomData *triangleData = gprtGeomGetParameters(trianglesGeom);
  triangleData->vertex = gprtBufferGetDevicePointer(vertexBuffer);
  triangleData->index = gprtBufferGetDevicePointer(indexBuffer);

  // Place that single cube mesh in a bottom level acceleration structure
  GPRTAccel trianglesAccel = gprtTriangleAccelCreate(context, trianglesGeom);
  gprtAccelBuild(context, trianglesAccel);

  // We will now create three instances of that cube mesh. On the gpu,
  // we will use the "instance ID" to determine what color each cube should be.

  // First, we create a list of BLAS objects
  gprt::Instance instance = gprtAccelGetInstance(trianglesAccel);
  gprt::Instance instances[NUM_INSTANCES] = {instance, instance, instance};

  // Here, we'll assign a transform to each of these instances to place them in our world.
  for (int i = 0; i < NUM_INSTANCES; ++i) {
    instances[i].transform = transforms[i];
  }

  // Then, we'll create a buffer to hold these instance primitives
  GPRTBufferOf<gprt::Instance> instanceBuffer =
      gprtDeviceBufferCreate<gprt::Instance>(context, NUM_INSTANCES, instances);

  // We'll also create a top level accel where these instances will be stored in the leaves
  GPRTAccel world = gprtInstanceAccelCreate(context, NUM_INSTANCES, instanceBuffer);

  // With our instances fully populated, we can now build our accel.
  gprtAccelBuild(context, world);

  // Here, we place a reference to our TLAS in the ray generation
  // kernel's parameters, so that we can access that tree when
  // we go to trace our rays.
  rayGenData->world = gprtAccelGetDeviceAddress(world);

  gprtBuildShaderBindingTable(context);

  // ##################################################################
  // now that everything is ready: launch it ....
  // ##################################################################

  // Structure of parameters that change each frame. We can edit these
  // without rebuilding the shader binding table.
  Constants pc;


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

    // Calls the GPU raygen kernel function
    gprtRayGenLaunch2D(context, rayGen, fbSize.x, fbSize.y, pc);

    // If a window exists, presents the framebuffer here to that window
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