#include <gprt.h>      // Public GPRT API
#include "sharedCode.h" // Shared data between host and device

extern GPRTProgram s2_2_deviceCode;

// The positions and radii of our line-swept sphere primitive
const int NUM_VERTICES = 2;
float4 vertices[NUM_VERTICES] = {
    float4(-0.4,-0.1,0.1, 0.4),
    float4( 0.4, 0.3,0.3, 0.06)
};

// Indices connect those vertices together.
const int NUM_INDICES = 1;
uint2 indices[NUM_INDICES] = {{0, 1}};

// Initial image resolution
const int2 fbSize = {1400, 460};

// Output file name for the rendered image
const char *outFileName = "s2-3-lss.png";

#include <iostream>
int main(int ac, char **av) {
  // Create a rendering window
  gprtRequestWindow(fbSize.x, fbSize.y, "S2-3 Line-Swept Spheres");

  // Initialize GPRT context and modules
  GPRTContext context = gprtContextCreate();
  GPRTModule module = gprtModuleCreate(context, s2_2_deviceCode);

  // New: Create a "line swept sphere" (LSS) geometry type
  auto lssGeomType = gprtGeomTypeCreate<LSSGeomData>(context, GPRT_LSS);
  gprtGeomTypeSetClosestHitProg(lssGeomType, 0, module, "LSSClosestHit");

  // Upload vertex and index data to GPU buffers
  GPRTBufferOf<float4> vertexBuffer = gprtDeviceBufferCreate<float4>(context, NUM_VERTICES, vertices);
  GPRTBufferOf<uint2> indexBuffer = gprtDeviceBufferCreate<uint2>(context, NUM_INDICES, indices);

  // New: Create geometry instance and set vertex and index buffers
  GPRTGeomOf<LSSGeomData> lssGeom = gprtGeomCreate<LSSGeomData>(context, lssGeomType);
  gprtLSSSetVertices(lssGeom, vertexBuffer, NUM_VERTICES);
  gprtLSSSetIndices(lssGeom, indexBuffer, NUM_INDICES);

  // New: Setup uniform parameters made available in the closest hit entry point 
  // when rays hit line-swept spheres.
  LSSGeomData lssParams;
  lssParams.indices = gprtBufferGetDevicePointer(indexBuffer);
  lssParams.vertices = gprtBufferGetDevicePointer(vertexBuffer);
  gprtGeomSetParameters(lssGeom, lssParams);

  // Create and build BLAS
  GPRTAccel lssAccel = gprtLSSAccelCreate(context, lssGeom);
  gprtAccelBuild(context, lssAccel, GPRT_BUILD_MODE_FAST_TRACE_NO_UPDATE);

  // Create and build TLAS
  gprt::Instance instance = gprtAccelGetInstance(lssAccel);
  GPRTBufferOf<gprt::Instance> instanceBuffer = gprtDeviceBufferCreate(context, 1, &instance);
  GPRTAccel world = gprtInstanceAccelCreate(context, 1, instanceBuffer);
  gprtAccelBuild(context, world, GPRT_BUILD_MODE_FAST_TRACE_NO_UPDATE);

  // Set up ray generation and miss programs
  GPRTMissOf<void> miss = gprtMissCreate<void>(context, module, "miss");
  GPRTRayGenOf<RayGenData> rayGen = gprtRayGenCreate<RayGenData>(context, module, "raygen");
  
  // Set up ray generation parameters
  GPRTBufferOf<uint32_t> frameBuffer = gprtDeviceBufferCreate<uint32_t>(context, fbSize.x * fbSize.y);
  RayGenData *rayGenData = gprtRayGenGetParameters(rayGen);
  rayGenData->frameBuffer = gprtBufferGetDevicePointer(frameBuffer);
  rayGenData->world = gprtAccelGetDeviceAddress(world);

  // Build the Shader Binding Table
  gprtBuildShaderBindingTable(context);

  // Main render loop
  PushConstants pc;
  do {
    pc.time = float(gprtGetTime(context));
    gprtRayGenLaunch2D(context, rayGen, fbSize.x, fbSize.y, pc);
    gprtBufferPresent(context, frameBuffer);
  }
  while (!gprtWindowShouldClose(context));

  // Save final frame to an image
  gprtBufferSaveImage(frameBuffer, fbSize.x, fbSize.y, outFileName);

  // Clean up resources
  gprtContextDestroy(context);

  return 0;
}
