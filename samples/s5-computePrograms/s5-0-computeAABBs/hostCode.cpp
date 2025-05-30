#include <gprt.h>      // Public GPRT API
#include "sharedCode.h" // Shared data between host and device

extern GPRTProgram s5_0_deviceCode;

// Vertices and radii that will be used to define virtual spheres.
// We want to use a compute shader to generate bounding boxes in parallel over these.
const int NUM_VERTICES = 11;
float3 vertices[NUM_VERTICES] = {
    {0.0f, 0.0f, 0.0f}, {0.1f, 0.0f, 0.0f}, {0.2f, 0.0f, 0.0f}, {0.3f, 0.0f, 0.0f},
    {0.4f, 0.0f, 0.0f}, {0.5f, 0.0f, 0.0f}, {0.6f, 0.0f, 0.0f}, {0.7f, 0.0f, 0.0f},
    {0.8f, 0.0f, 0.0f}, {0.9f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f},
};

float radii[NUM_VERTICES] = {.015f, .025f, .035f, .045f, .055f, .065f, .055f, .045f, .035f, .025f, .015f};

// initial image resolution
const int2 fbSize = {1400, 460};

// final image output
const char *outFileName = "s5-0-computeAABBs.png";

// Initial camera parameters
float3 lookFrom = {0.5f, 0.0f, 0.6f};
float3 lookAt = {0.5f, 0.f, 0.f};
float3 lookUp = {0.f, -1.f, 0.f};
float cosFovy = 0.66f;

int main(int ac, char **av) {
  // In this example, we'll use a compute shader to generate a set of
  // procedural axis aligned bounding boxes in parallel on the GPU. Each
  // AABB will contain a single sphere.

  // create a context on the first device:
  gprtRequestMaxAttributeSize(2 * sizeof(float4));
  gprtRequestWindow(fbSize.x, fbSize.y, "S04 Compute AABB");
  GPRTContext context = gprtContextCreate(nullptr, 1);
  GPRTModule module = gprtModuleCreate(context, s5_0_deviceCode);

  // ##################################################################
  // set up all the GPU kernels we want to run
  // ##################################################################

  // -------------------------------------------------------
  // declare geometry type
  // -------------------------------------------------------
  GPRTGeomTypeOf<AABBGeomData> customGeomType = gprtGeomTypeCreate<AABBGeomData>(context, GPRT_AABBS);
  gprtGeomTypeSetClosestHitProg(customGeomType, 0, module, "AABBClosestHit");
  gprtGeomTypeSetIntersectionProg(customGeomType, 0, module, "AABBIntersection");

  // -------------------------------------------------------
  // set up sphere bounding box compute program
  // -------------------------------------------------------
  GPRTComputeOf<AABBGeomData> boundsProgram = gprtComputeCreate<AABBGeomData>(context, module, "ComputeBounds");

  // -------------------------------------------------------
  // set up miss
  // -------------------------------------------------------
  GPRTMissOf<MissProgData> miss = gprtMissCreate<MissProgData>(context, module, "miss");

  // -------------------------------------------------------
  // set up ray gen program
  // -------------------------------------------------------
  GPRTRayGenOf<RayGenData> rayGen = gprtRayGenCreate<RayGenData>(context, module, "simpleRayGen");

  // ##################################################################
  // set the parameters for our compute kernel
  // ##################################################################

  // ------------------------------------------------------------------
  // aabb mesh
  // ------------------------------------------------------------------
  GPRTBufferOf<float3> vertexBuffer = gprtDeviceBufferCreate<float3>(context, NUM_VERTICES, vertices);
  GPRTBufferOf<float> radiusBuffer = gprtDeviceBufferCreate<float>(context, NUM_VERTICES, radii);
  GPRTBufferOf<float3> aabbPositionsBuffer = gprtDeviceBufferCreate<float3>(context, NUM_VERTICES * 2, nullptr);

  GPRTGeomOf<AABBGeomData> aabbGeom = gprtGeomCreate(context, customGeomType);
  gprtAABBsSetPositions(aabbGeom, aabbPositionsBuffer, NUM_VERTICES);

  AABBGeomData geomData;
  geomData.vertex = gprtBufferGetDevicePointer(vertexBuffer);
  geomData.radius = gprtBufferGetDevicePointer(radiusBuffer);
  geomData.aabbs = gprtBufferGetDevicePointer(aabbPositionsBuffer);
  gprtGeomSetParameters(aabbGeom, geomData);

  // Launch the compute kernel, which will populate our aabbPositionsBuffer
  gprtComputeLaunch(boundsProgram, {NUM_VERTICES, 1, 1}, {1, 1, 1}, geomData);

  // Now that the aabbPositionsBuffer is filled, we can compute our AABB
  // acceleration structure
  GPRTAccel aabbAccel = gprtAABBAccelCreate(context, aabbGeom);
  gprtAccelBuild(context, aabbAccel);

  gprt::Instance instance = gprtAccelGetInstance(aabbAccel);
  GPRTBufferOf<gprt::Instance> instanceBuffer = gprtDeviceBufferCreate(context, 1, &instance);

  GPRTAccel world = gprtInstanceAccelCreate(context, 1, instanceBuffer);
  gprtAccelBuild(context, world);

  // ##################################################################
  // set the parameters for the rest of our kernels
  // ##################################################################

  // Setup pixel frame buffer
  GPRTBufferOf<uint32_t> frameBuffer = gprtDeviceBufferCreate<uint32_t>(context, fbSize.x * fbSize.y);

  // Raygen program frame buffer
  RayGenData *rayGenData = gprtRayGenGetParameters(rayGen);
  rayGenData->frameBuffer = gprtBufferGetDevicePointer(frameBuffer);
  rayGenData->world = gprtAccelGetDeviceAddress(world);

  // Miss program checkerboard background colors
  MissProgData *missData = gprtMissGetParameters(miss);
  missData->color0 = float3(0.1f, 0.1f, 0.1f);
  missData->color1 = float3(0.0f, 0.0f, 0.0f);

  // Upload our newly assigned parameters to the shader binding table.
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

  return 0;
}
