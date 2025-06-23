#include <gprt.h>      // Public GPRT API
#include "sharedCode.h" // Shared data between host and device

extern GPRTProgram s4_5_deviceCode;

// The extents of our bounding box at key 0 and key 1, plus two degenerage AABBs at the extremes.
const int NUM_AABB_POSITIONS = 6;
float3 t0_aabbPositions[NUM_AABB_POSITIONS] = {
  {-1.0f, -1.0f, -1.0f}, {0.0f, 0.0f, 0.0f},
  // AABB containing geometry at key 0.
  {-1.f, -1.f, -1.f},  {-1.f, -1.f, -1.f},
  {0.f, 0.f, 0.f},  {0.f, 0.f, 0.f},
};

float3 t1_aabbPositions[NUM_AABB_POSITIONS] = {
  {0.0f, 0.0f, 0.0f}, {1.0f, 1.0f, 1.0f},
  // AABB containing geometry at key 1.
  {-1.f, -1.f, -1.f},  {-1.f, -1.f, -1.f},
  {+1.f, +1.f, +1.f},  {+1.f, +1.f, +1.f},
};

// initial image resolution
const int2 fbSize = {1400, 460};

// final image output
const char *outFileName = "s4-5-mbaabbs.png";

#include <iostream>
int main(int ac, char **av) {
  // In this example, we will create an axis aligned bounding box (AABB). These
  // are more general than triangle primitives, and can be used for custom
  // primitive types.

  gprtRequestMaxAttributeSize(2 * sizeof(float4));

  // New, requests to use the motion blur extension.
  gprtRequestMotionBlur();

  // create a context on the first device:
  gprtRequestWindow(fbSize.x, fbSize.y, "S4 5 Motion Blurred AABB");
  GPRTContext context = gprtContextCreate();
  GPRTModule module = gprtModuleCreate(context, s4_5_deviceCode);

  // ##################################################################
  // set up all the GPU kernels we want to run
  // ##################################################################

  // -------------------------------------------------------
  // declare geometry type
  // -------------------------------------------------------
  GPRTGeomTypeOf<AABBGeomData> aabbGeomType =
      gprtGeomTypeCreate<AABBGeomData>(context, GPRT_AABBS /* <- This is new! */);
  gprtGeomTypeSetClosestHitProg(aabbGeomType, 0, module, "AABBClosestHit");
  gprtGeomTypeSetIntersectionProg(aabbGeomType, 0, module, "AABBIntersection");

  // -------------------------------------------------------
  // set up miss
  // -------------------------------------------------------
  GPRTMissOf<void> miss = gprtMissCreate<void>(context, module, "miss");

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

  // Create our AABB geometry. Every AABB is defined using two float3's. The
  // first float3 defines the bottom lower left near corner, and the second
  // float3 defines the upper far right corner.
  GPRTBufferOf<float3> t0_aabbPositionsBuffer = gprtDeviceBufferCreate<float3>(context, NUM_AABB_POSITIONS, t0_aabbPositions);
  GPRTBufferOf<float3> t1_aabbPositionsBuffer = gprtDeviceBufferCreate<float3>(context, NUM_AABB_POSITIONS, t1_aabbPositions);
  
  // New: use "SetMotionPositions" for motion blur. (if an object shouldn't have motion blur, 
  // then you can just use the normal "SetPositions" call )
  GPRTGeomOf<AABBGeomData> aabbGeom = gprtGeomCreate<AABBGeomData>(context, aabbGeomType);
  gprtAABBsSetMotionPositions(aabbGeom, t0_aabbPositionsBuffer, t1_aabbPositionsBuffer, NUM_AABB_POSITIONS/2);
  AABBGeomData *geomData = gprtGeomGetParameters(aabbGeom);
  geomData->t0_aabbs = gprtBufferGetDevicePointer(t0_aabbPositionsBuffer);
  geomData->t1_aabbs = gprtBufferGetDevicePointer(t1_aabbPositionsBuffer);

  // Note, we must create an "AABB" accel rather than a triangles accel.
  GPRTBuildParams blasParams;
  blasParams.buildMode = GPRT_BUILD_MODE_FAST_TRACE_NO_UPDATE; // Fast trace, no updates allowed
  blasParams.hasMotionBlur = true; // Enable motion blur for the BLAS
  GPRTAccel aabbAccel = gprtAABBAccelCreate(context, aabbGeom);
  gprtAccelBuild(context, aabbAccel, blasParams);

  gprt::Instance instance = gprtAccelGetInstance(aabbAccel);
  GPRTBufferOf<gprt::Instance> instanceBuffer = gprtDeviceBufferCreate(context, 1, &instance);

  // triangle and AABB accels can be combined in a top level tree
  GPRTBuildParams tlasParams;
  tlasParams.buildMode = GPRT_BUILD_MODE_FAST_TRACE_NO_UPDATE; // Fast trace, allows updates
  tlasParams.hasMotionBlur = false; // Going with a static TLAS for now with two degen tris 
  GPRTAccel world = gprtInstanceAccelCreate(context, 1, instanceBuffer);
  gprtAccelBuild(context, world, tlasParams);

  rayGenData->world = gprtAccelGetDeviceAddress(world);

  gprtBuildShaderBindingTable(context);

  // ##################################################################
  // now that everything is ready: launch it ....
  // ##################################################################


  // Structure of parameters that change each frame. We can edit these
  // without rebuilding the shader binding table.
  PushConstants pc;
  do {
    pc.time = float(gprtGetTime(context));

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
