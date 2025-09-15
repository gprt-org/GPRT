#include <gprt.h>      // Public GPRT API
#include "sharedCode.h" // Shared data between host and device

extern GPRTProgram s3_3_deviceCode;

// The extents of our bounding box
float3 firstAabbPositions[2] = {float3(-0.5f) + float3(-2.f, 0.f, 0.f), float3(0.5f) + float3(-2.f, 0.f, 0.f)};

float3 secondAabbPositions[2] = {float3(-0.5f) + float3(0.f, 0.f, 0.f), float3(0.5f) + float3(0.f, 0.f, 0.f)};

float3 thirdAabbPositions[2] = {float3(-0.5f) + float3(2.f, 0.f, 0.f), float3(0.5f) + float3(2.f, 0.f, 0.f)};

// initial image resolution
const int2 fbSize = {1400, 460};

// final image output
const char *outFileName = "s3-3-AABBGeoInBLAS.png";

#include <iostream>
int main(int ac, char **av) {
  // In this example, we will create an axis aligned bounding box (AABB). These
  // are more general than triangle primitives, and can be used for custom
  // primitive types.

  gprtRequestMaxAttributeSize(2 * sizeof(float4));

  // create a context on the first device:
  gprtRequestWindow(fbSize.x, fbSize.y, "S3-3 Multiple AABB Geoetries in a Common BLAS");
  GPRTContext context = gprtContextCreate();
  GPRTModule module = gprtModuleCreate(context, s3_3_deviceCode);

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
  GPRTBufferOf<float3> firstAabbPositionsBuffer = gprtDeviceBufferCreate<float3>(context, 2, firstAabbPositions);
  GPRTBufferOf<float3> secondAabbPositionsBuffer = gprtDeviceBufferCreate<float3>(context, 2, secondAabbPositions);
  GPRTBufferOf<float3> thirdAabbPositionsBuffer = gprtDeviceBufferCreate<float3>(context, 2, thirdAabbPositions);  
  GPRTGeomOf<AABBGeomData> firstAabbGeom = gprtGeomCreate<AABBGeomData>(context, aabbGeomType);
  GPRTGeomOf<AABBGeomData> secondAabbGeom = gprtGeomCreate<AABBGeomData>(context, aabbGeomType);
  GPRTGeomOf<AABBGeomData> thirdAabbGeom = gprtGeomCreate<AABBGeomData>(context, aabbGeomType);
  gprtAABBsSetPositions(firstAabbGeom, firstAabbPositionsBuffer, 1 /* just one aabb */);
  gprtAABBsSetPositions(secondAabbGeom, secondAabbPositionsBuffer, 1 /* just one aabb */);
  gprtAABBsSetPositions(thirdAabbGeom, thirdAabbPositionsBuffer, 1 /* just one aabb */);

  AABBGeomData firstAABBGeomData, secondAABBGeomData, thirdAABBGeomData;
  firstAABBGeomData.aabbs = gprtBufferGetDevicePointer(firstAabbPositionsBuffer);
  secondAABBGeomData.aabbs = gprtBufferGetDevicePointer(secondAabbPositionsBuffer);
  thirdAABBGeomData.aabbs = gprtBufferGetDevicePointer(thirdAabbPositionsBuffer);
  gprtGeomSetParameters(firstAabbGeom, firstAABBGeomData);
  gprtGeomSetParameters(secondAabbGeom, secondAABBGeomData);
  gprtGeomSetParameters(thirdAabbGeom, thirdAABBGeomData);

  // Note, we must create an "AABB" accel rather than a triangles accel.
  std::vector<GPRTGeomOf<AABBGeomData>> geoms = {firstAabbGeom, secondAabbGeom, thirdAabbGeom};
  GPRTAccel aabbAccel = gprtAABBAccelCreate(context, geoms.size(), geoms.data());
  gprtAccelBuild(context, aabbAccel, GPRT_BUILD_MODE_FAST_TRACE_NO_UPDATE);

  gprt::Instance instance = gprtAccelGetInstance(aabbAccel);
  GPRTBufferOf<gprt::Instance> instanceBuffer = gprtDeviceBufferCreate(context, 1, &instance);

  // triangle and AABB accels can be combined in a top level tree
  GPRTAccel world = gprtInstanceAccelCreate(context, 1, instanceBuffer);
  gprtAccelBuild(context, world, GPRT_BUILD_MODE_FAST_TRACE_NO_UPDATE);

  rayGenData->world = gprtAccelGetHandle(world);

  gprtBuildShaderBindingTable(context);

  // ##################################################################
  // now that everything is ready: launch it ....
  // ##################################################################


  // Structure of parameters that change each frame. We can edit these
  // without rebuilding the shader binding table.
  Constants pc;
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
