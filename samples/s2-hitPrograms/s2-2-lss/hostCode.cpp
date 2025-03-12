#include <gprt.h>      // Public GPRT API
#include "sharedCode.h" // Shared data between host and device

extern GPRTProgram s2_2_deviceCode;

// The positions and radii of our line-swept sphere primitive
const float3 N = normalize(float3(1.0, 0.0, 0.0));
const float3 T = normalize(float3(0.0, 1.0, 0.0));
const int NUM_VERTICES = 4;
float4 vertices[NUM_VERTICES] = {
    float4(0.,0.0,0.0, 0.0) + float4(N , 0.0),
    float4(0.,0.0,0.0, 0.5),
    float4(0.,0.0,0.0, 0.0) - float4(N , 0.0)
};

// Indices connect those vertices together.
const int NUM_INDICES = 2;
uint2 indices[NUM_INDICES] = {{0, 1}, {2, 3}};

// Initial image resolution
// const int2 fbSize = {1400, 460};
const int2 fbSize = {1024, 1024};

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
  GPRTBufferOf<float4> vertexBuffer = gprtHostBufferCreate<float4>(context, NUM_VERTICES, vertices);
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
  GPRTAccel lssAccel = gprtLSSAccelCreate(context, lssGeom, GPRT_LSS_NO_END_CAPS);
  gprtAccelBuild(context, lssAccel, GPRT_BUILD_MODE_FAST_BUILD_AND_UPDATE);

  // Create and build TLAS
  gprt::Instance instance = gprtAccelGetInstance(lssAccel);
  GPRTBufferOf<gprt::Instance> instanceBuffer = gprtDeviceBufferCreate(context, 1, &instance);
  GPRTAccel world = gprtInstanceAccelCreate(context, 1, instanceBuffer);
  gprtAccelBuild(context, world, GPRT_BUILD_MODE_FAST_BUILD_AND_UPDATE);

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
    pc.time = float(gprtGetTime(context)) * .5;
    gprtRayGenLaunch2D(context, rayGen, fbSize.x, fbSize.y, pc);
    gprtBufferPresent(context, frameBuffer);

    float4* verts = gprtBufferGetHostPointer(vertexBuffer);

    // parameters of ellipse
    float b = .5; // height
    float a = std::max((cos(pc.time)* .5f + 0.5f) * b, (1.f / 64.f) * b); // width

    float scale = b / a;
    float A = b;
    float B = (b*b) / a;
    float C = sqrt(A*A + B*B);

    verts[0] = float4(-N*a,0.); //
    verts[1] = float4(+N * B, C); //(sin(pc.time)* .5f + 0.5f); 
    verts[2] = float4(-N * B, C); //(sin(pc.time)* .5f + 0.5f); 
    verts[3] = float4(+N*a,0.); //(sin(pc.time)* .5f + 0.5f); 
    gprtAccelBuild(context, lssAccel, GPRT_BUILD_MODE_FAST_BUILD_AND_UPDATE);
    gprtAccelBuild(context, world, GPRT_BUILD_MODE_FAST_BUILD_AND_UPDATE);
  }
  while (!gprtWindowShouldClose(context));

  // Save final frame to an image
  gprtBufferSaveImage(frameBuffer, fbSize.x, fbSize.y, outFileName);

  // Clean up resources
  gprtContextDestroy(context);

  return 0;
}
