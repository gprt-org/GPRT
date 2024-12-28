#include <gprt.h>      // Public GPRT API
#include "sharedCode.h" // Shared data between host and device

extern GPRTProgram s2_1_deviceCode;

// Vertices defining five spheres. Each vertex stores the sphere center (xyz) and radius (w).
const int NUM_VERTICES = 5;
float4 vertices[NUM_VERTICES] = {
    float4(-4 + 1 * 0.8, sin(-4 + 1 * 0.8), 0.0, 0.3),
    float4(-4 + 3 * 0.8, sin(-4 + 3 * 0.8), 0.0, 0.2),
    float4(-4 + 5 * 0.8, sin(-4 + 5 * 0.8), 0.0, 0.5),
    float4(-4 + 7 * 0.8, sin(-4 + 7 * 0.8), 0.0, 0.4),
    float4(-4 + 9 * 0.8, sin(-4 + 9 * 0.8), 0.0, 0.6),
};

// Initial image resolution
const int2 fbSize = {1400, 460};

// Output file name for the rendered image
const char *outFileName = "s2-1-spheres.png";

#include <iostream>
int main(int ac, char **av) {
  // Create a rendering window
  gprtRequestWindow(fbSize.x, fbSize.y, "S2-1 Spheres");

  // Initialize GPRT context and modules
  GPRTContext context = gprtContextCreate();
  GPRTModule module = gprtModuleCreate(context, s2_1_deviceCode);

  // New: Create a "spheres" geometry type and set it's closest-hit program
  auto sphereGeomType = gprtGeomTypeCreate<SphereGeomData>(context, GPRT_SPHERES);
  gprtGeomTypeSetClosestHitProg(sphereGeomType, 0, module, "SphereClosestHit");

  // Upload vertex data to GPU buffer
  auto vertexBuffer = gprtDeviceBufferCreate<float4>(context, NUM_VERTICES, vertices);

  // New: Create geometry instance and set vertex buffer
  auto sphereGeom = gprtGeomCreate<SphereGeomData>(context, sphereGeomType);
  gprtSpheresSetVertices(sphereGeom, vertexBuffer, NUM_VERTICES);

  // New: Setup uniform parameters made available in the closest hit entry point 
  // when rays hit spheres.
  SphereGeomData sphereParams;
  sphereParams.posAndRadius = gprtBufferGetDevicePointer(vertexBuffer);
  gprtGeomSetParameters(sphereGeom, sphereParams);

  // Create and build BLAS
  GPRTAccel sphereAccel = gprtSphereAccelCreate(context, sphereGeom);
  gprtAccelBuild(context, sphereAccel, GPRT_BUILD_MODE_FAST_TRACE_NO_UPDATE);

  // Create and build TLAS
  gprt::Instance instance = gprtAccelGetInstance(sphereAccel);
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
  rayGenData->world = gprtAccelGetHandle(world);

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
