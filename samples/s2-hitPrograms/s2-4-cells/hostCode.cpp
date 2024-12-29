#include <gprt.h>      // Public GPRT API
#include "sharedCode.h" // Shared data between host and device

// library for image output
#define STB_IMAGE_STATIC
#define STB_IMAGE_IMPLEMENTATION
#include "stb/stb_image.h"

extern GPRTProgram s2_4_deviceCode;

// The extents of our bounding box
float3 aabbPositions[2] = {{-1.0f, -1.0f, -1.0f}, {1.0f, 1.0f, 1.0f}};

// initial image resolution
const int2 fbSize = {1400, 460};

// final image output
const char *outFileName = "s03-singleAABB.png";

#include <iostream>
int main(int ac, char **av) {
  // In this example, we will create an axis aligned bounding box (AABB). These
  // are more general than triangle primitives, and can be used for custom
  // primitive types.

  gprtRequestMaxAttributeSize(2 * sizeof(float4));

  // create a context on the first device:
  gprtRequestWindow(fbSize.x, fbSize.y, "S03 Single AABB");
  GPRTContext context = gprtContextCreate();
  GPRTModule module = gprtModuleCreate(context, s2_4_deviceCode);

  // ##################################################################
  // set up all the GPU kernels we want to run
  // ##################################################################

  int x, y, comp;
  stbi_us *pixels = stbi_load_16("C:\\Users\\Nate\\git\\GPRT\\samples\\s2-hitPrograms\\s2-4-cells\\256x256_l128_s16.png", &x, &y, &comp, 0);
  std::vector<stbi_us> stbn(x * y);
  for (int yi = 0; yi < y; ++yi) 
  {
    for (int xi = 0; xi < x; ++xi) {
      stbn[yi * x + xi] = pixels[(yi * x + xi) * comp + 0];
    }
  }
  
  GPRTTextureOf<stbi_us> stbnTex = gprtDeviceTextureCreate<stbi_us>(
      context, GPRT_IMAGE_TYPE_2D, GPRT_FORMAT_R16_UNORM, x, y, /*depth*/ 1,
      /* generate mipmaps */ false, stbn.data());

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
  GPRTBufferOf<float3> aabbPositionsBuffer = gprtDeviceBufferCreate<float3>(context, 2, aabbPositions);
  GPRTGeomOf<AABBGeomData> aabbGeom = gprtGeomCreate<AABBGeomData>(context, aabbGeomType);
  gprtAABBsSetPositions(aabbGeom, aabbPositionsBuffer, 1 /* just one aabb */);
  AABBGeomData *geomData = gprtGeomGetParameters(aabbGeom);
  geomData->aabbs = gprtBufferGetDevicePointer(aabbPositionsBuffer);

  // Note, we must create an "AABB" accel rather than a triangles accel.
  GPRTAccel aabbAccel = gprtAABBAccelCreate(context, aabbGeom);
  gprtAccelBuild(context, aabbAccel, GPRT_BUILD_MODE_FAST_TRACE_NO_UPDATE);

  gprt::Instance instance = gprtAccelGetInstance(aabbAccel);
  GPRTBufferOf<gprt::Instance> instanceBuffer = gprtDeviceBufferCreate(context, 1, &instance);

  // triangle and AABB accels can be combined in a top level tree
  GPRTAccel world = gprtInstanceAccelCreate(context, 1, instanceBuffer);
  gprtAccelBuild(context, world, GPRT_BUILD_MODE_FAST_TRACE_NO_UPDATE);

  rayGenData->world = gprtAccelGetHandle(world);
  rayGenData->stbn = gprtTextureGetHandle(stbnTex);

  gprtBuildShaderBindingTable(context);

  // ##################################################################
  // now that everything is ready: launch it ....
  // ##################################################################


  // Structure of parameters that change each frame. We can edit these
  // without rebuilding the shader binding table.
  PushConstants pc;
  pc.frameID = 0;
  do {
    pc.time = float(gprtGetTime(context));
    pc.frameID = (pc.frameID + 1) % 128;

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
