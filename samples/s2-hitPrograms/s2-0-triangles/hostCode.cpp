#include <gprt.h>      // Public GPRT API
#include "sharedCode.h" // Shared data between host and device

extern GPRTProgram s2_0_deviceCode;
extern GPRTProgram s2_0_missCode;

// Vertices defining the triangle
const int NUM_VERTICES = 3;
float3 vertices[NUM_VERTICES] = {
    {-1.f, -.5f, 0.f},
    {+1.f, -.5f, 0.f},
    {0.f, +.5f, 0.f},
};

// Indices defining the connections between vertices
const int NUM_INDICES = 1;
uint3 indices[NUM_INDICES] = {{0, 1, 2}};

// Initial image resolution
const int2 fbSize = {1400, 460};

// Output file name for the rendered image
const char *outFileName = "s2-0-triangles.png";

int main(int ac, char **av) {
  // Create a rendering window
  gprtRequestWindow(fbSize.x, fbSize.y, "S01 Single Triangle");

  // Initialize GPRT context and modules
  GPRTContext context = gprtContextCreate();
  GPRTModule module = gprtModuleCreate(context, s2_0_deviceCode);

  // New: Create a "triangle" geometry type and set it's closest-hit program
  auto trianglesGeomType = gprtGeomTypeCreate<TrianglesGeomData>(context, GPRT_TRIANGLES);
  gprtGeomTypeSetClosestHitProg(trianglesGeomType, 0, module, "TriangleMesh");

  // Upload vertex and index data to GPU buffers
  auto vertexBuffer = gprtDeviceBufferCreate<float3>(context, NUM_VERTICES, vertices);
  auto indexBuffer = gprtDeviceBufferCreate<uint3>(context, NUM_INDICES, indices);

  // New: Create geometry instance and set vertex and index buffers
  auto trianglesGeom = gprtGeomCreate<TrianglesGeomData>(context, trianglesGeomType);
  gprtTrianglesSetVertices(trianglesGeom, vertexBuffer, NUM_VERTICES);
  gprtTrianglesSetIndices(trianglesGeom, indexBuffer, NUM_INDICES);

  // Place the geometry into a bottom-level acceleration structure (BLAS).
  // A BLAS organizes triangles into a data structure that allows rays to quickly
  // determine potential intersections, significantly speeding up ray tracing by narrowing down
  // the search to relevant geometry instead of testing every triangle.
  GPRTAccel trianglesAccel = gprtTriangleAccelCreate(context, trianglesGeom);
  gprtAccelBuild(context, trianglesAccel);

  // Create a single instance of the BLAS in a top-level acceleration structure (TLAS), required for ray tracing.
  // (We'll cover TLAS in more depth later)
  gprt::Instance instance = gprtAccelGetInstance(trianglesAccel);
  auto instanceBuffer = gprtDeviceBufferCreate<gprt::Instance>(context, 1, &instance);
  GPRTAccel world = gprtInstanceAccelCreate(context, 1, instanceBuffer);
  gprtAccelBuild(context, world);

  // Set up ray generation and miss programs
  GPRTRayGenOf<RayGenData> rayGen = gprtRayGenCreate<RayGenData>(context, module, "raygen");
  GPRTMissOf<void> miss = gprtMissCreate<void>(context, module, "miss");

  // New: Here, we place a reference to our TLAS in the ray generation
  // kernel's parameters, so that we can access that tree when
  // we go to trace our rays.
  RayGenData *rayGenData = gprtRayGenGetParameters(rayGen);
  rayGenData->world = gprtAccelGetDeviceAddress(world);

  GPRTBufferOf<uint32_t> frameBuffer = gprtDeviceBufferCreate<uint32_t>(context, fbSize.x * fbSize.y);
  rayGenData->frameBuffer = gprtBufferGetDevicePointer(frameBuffer);

  // Build the Shader Binding Table (SBT), updating all parameters.
  gprtBuildShaderBindingTable(context, GPRT_SBT_ALL);

  // Main render loop
  RTConstants constants;
  do {
    constants.time = float(gprtGetTime(context));
    gprtRayGenLaunch2D(context, rayGen, fbSize.x, fbSize.y, constants);
    gprtBufferPresent(context, frameBuffer);
  }
  while (!gprtWindowShouldClose(context));

  // Save final frame to an image
  gprtBufferSaveImage(frameBuffer, fbSize.x, fbSize.y, outFileName);

  // Clean up resources
  gprtContextDestroy(context);

  return 0;
}
