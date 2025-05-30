#include <gprt.h>      // Public GPRT API
#include "sharedCode.h" // Shared data between host and device

// library for image output
#define STB_IMAGE_STATIC
#define STB_IMAGE_IMPLEMENTATION
#include "stb/stb_image.h"

extern GPRTProgram s2_4_deviceCode;

// initial image resolution
const int2 fbSize = {1400, 460};

// final image output
const char *outFileName = "s2_4-solids.png";

int main(int ac, char **av) {
  // Create a rendering window
  gprtRequestWindow(fbSize.x, fbSize.y, "S2_4 Solids");
  
  // Initialize GPRT context and modules
  GPRTContext context = gprtContextCreate();
  GPRTModule module = gprtModuleCreate(context, s2_4_deviceCode);

  // New: Create a "solid" geometry type and set it's closest-hit program
  GPRTGeomTypeOf<SolidGeomData> solidGeomType =
      gprtGeomTypeCreate<SolidGeomData>(context, GPRT_SOLIDS /* <- This is new! */);
  gprtGeomTypeSetClosestHitProg(solidGeomType, 0, module, "SolidClosestHit");

  // New: Create our solid geometry.
  GPRTGeomOf<SolidGeomData> solidGeom = gprtGeomCreate<SolidGeomData>(context, solidGeomType);
  SolidGeomData *geomData = gprtGeomGetParameters(solidGeom);

  // Upload vertex, index, and solid type data to GPU buffers
  std::vector<float4> vertices; std::vector<uint4> indices; std::vector<uint8_t> types;
  generateSolids(vertices, indices, types);
  
  // Each solid vertex has a position (xyz) and density (w)
  GPRTBufferOf<float4> solidVertices = gprtDeviceBufferCreate<float4>(context, vertices.size(), vertices.data());
  gprtSolidsSetVertices(solidGeom, solidVertices, vertices.size());

  // Each solid primitive will read up to eight consecutive indices, depending on the type.
  GPRTBufferOf<uint4> solidIndices = gprtDeviceBufferCreate<uint4>(context, indices.size(), indices.data());
  gprtSolidsSetIndices(solidGeom, solidIndices, indices.size()/2, /*indices stride*/ 2 * sizeof(uint4));

  // Solids can vary in their types. If all elements are known to be the same type, one can create a type 
  // array of one element, then set the type stride to 0 when creating the solid geometry. 
  GPRTBufferOf<uint8_t> solidTypes = gprtDeviceBufferCreate<uint8_t>(context, types.size(), types.data());
  gprtSolidsSetTypes(solidGeom, solidTypes, types.size(), /*types stride. Set to 0 if we only have one type*/ 1);

  // Place the geometry into a BLAS. Note, we must create a "Solid" accel rather than a triangles accel.
  GPRTAccel solidAccel = gprtSolidAccelCreate(context, solidGeom);
  gprtAccelBuild(context, solidAccel, GPRT_BUILD_MODE_FAST_TRACE_NO_UPDATE);
  
  // Multiple different BLAS instances can be combined in a top level tree
  gprt::Instance instance = gprtAccelGetInstance(solidAccel);
  GPRTBufferOf<gprt::Instance> instanceBuffer = gprtDeviceBufferCreate(context, 1, &instance);
  GPRTAccel world = gprtInstanceAccelCreate(context, 1, instanceBuffer);
  gprtAccelBuild(context, world, GPRT_BUILD_MODE_FAST_TRACE_NO_UPDATE);

  // Set up ray generation and miss programs
  GPRTRayGenOf<RayGenData> rayGen = gprtRayGenCreate<RayGenData>(context, module, "raygen");
  GPRTMissOf<void> miss = gprtMissCreate<void>(context, module, "miss");
  RayGenData *rayGenData = gprtRayGenGetParameters(rayGen);

  GPRTBufferOf<uint32_t> frameBuffer = gprtDeviceBufferCreate<uint32_t>(context, fbSize.x * fbSize.y);
  rayGenData->frameBuffer = gprtBufferGetDevicePointer(frameBuffer);

  // New: note that "world" is a "SolidAccelerationStructure" type.
  // Solid accels have defined behavior for points which otherwise cannot intersect surfaces.
  rayGenData->world = gprtAccelGetDeviceAddress(world);

  // To render the solids volumetrically, we'll use the spatiotemporal blue noise texture below.
  // At a high level, this texture tells us how to randomly jitter our ray during ray marching, 
  // such that the noise is well stratified and "nice to look at".
  int x, y, comp;
  stbi_us *pixels = stbi_load_16(ASSETS_DIRECTORY "256x256_l128_s16.png", &x, &y, &comp, 0);
  std::vector<stbi_us> stbn(x * y);
  for (int yi = 0; yi < y; ++yi) {
    for (int xi = 0; xi < x; ++xi) {
      stbn[yi * x + xi] = pixels[(yi * x + xi) * comp + 0];
    }
  }
  
  GPRTTextureOf<stbi_us> stbnTex = gprtDeviceTextureCreate<stbi_us>(
      context, GPRT_IMAGE_TYPE_2D, GPRT_FORMAT_R16_UNORM, x, y, /*depth*/ 1,
      /* generate mipmaps */ false, stbn.data());
  rayGenData->stbn = gprtTextureGet2DHandle(stbnTex);

  // Build the Shader Binding Table (SBT), updating all parameters.
  gprtBuildShaderBindingTable(context);

  // Main render loop
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

  // Clean up resources
  gprtContextDestroy(context);
}
