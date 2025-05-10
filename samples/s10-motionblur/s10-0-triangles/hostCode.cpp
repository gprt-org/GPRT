#include <gprt.h>      // Public GPRT API
#include "sharedCode.h" // Shared data between host and device

extern GPRTProgram s10_0_deviceCode;

// Vertices defining the triangle at key 0 and key 1
const int NUM_VERTICES = 3;
float3 t0_vertices[NUM_VERTICES] = {
    {-1.f, -.5f, 0.f},
    {+1.f, -.5f, 0.f},
    {0.f, +.5f, 0.f},
};

float3 t1_vertices[NUM_VERTICES] = {
  {0.f, -1.5f, 1.f},
  {+2.f, -1.5f, 1.f},
  {2.f, +1.5f, 1.f},
};

// Indices defining the connections between vertices
const int NUM_INDICES = 1;
uint3 indices[NUM_INDICES] = {{0, 1, 2}};

// Initial image resolution
const int2 fbSize = {1400, 460};

// Output file name for the rendered image
const char *outFileName = "s10-0-triangles.png";

int main(int ac, char **av) {
  // Create a rendering window
  gprtRequestWindow(fbSize.x, fbSize.y, "S10 0 Motion Blurred Triangle");

  // New, requests to use the motion blur extension.
  gprtRequestMotionBlur();

  // Initialize GPRT context and modules
  GPRTContext context = gprtContextCreate();
  GPRTModule module = gprtModuleCreate(context, s10_0_deviceCode);

  // New: Create a "triangle" geometry type and set it's closest-hit program
  auto trianglesGeomType = gprtGeomTypeCreate<TrianglesGeomData>(context, GPRT_TRIANGLES);
  gprtGeomTypeSetClosestHitProg(trianglesGeomType, 0, module, "TriangleMesh");

  // Upload vertex and index data to GPU buffers
  auto t0_vertexBuffer = gprtDeviceBufferCreate<float3>(context, NUM_VERTICES, t0_vertices);
  auto t1_vertexBuffer = gprtDeviceBufferCreate<float3>(context, NUM_VERTICES, t1_vertices);
  auto indexBuffer = gprtDeviceBufferCreate<uint3>(context, NUM_INDICES, indices);

  // New: use "SetMotionVertices" for motion blur. (if an object shouldn't have motion blur, 
  // then you can just use the normal "SetVertices" call )
  auto trianglesGeom = gprtGeomCreate<TrianglesGeomData>(context, trianglesGeomType);
  gprtTrianglesSetMotionVertices(trianglesGeom, t0_vertexBuffer, t1_vertexBuffer, NUM_VERTICES);
  gprtTrianglesSetIndices(trianglesGeom, indexBuffer, NUM_INDICES);

  // Place the geometry into a bottom-level acceleration structure (BLAS).
  // A BLAS organizes triangles into a data structure that allows rays to quickly
  // determine potential intersections, significantly speeding up ray tracing by narrowing down
  // the search to relevant geometry instead of testing every triangle.
  GPRTAccel trianglesAccel = gprtTriangleAccelCreate(context, trianglesGeom);
  gprtAccelBuild(context, trianglesAccel, GPRT_BUILD_MODE_FAST_TRACE_NO_UPDATE);

  // Create a single instance of the BLAS in a top-level acceleration structure (TLAS), required for ray tracing.
  // (We'll cover TLAS in more depth later)
  gprt::Instance instance = gprtAccelGetInstance(trianglesAccel);
  auto instanceBuffer = gprtDeviceBufferCreate<gprt::Instance>(context, 1, &instance);
  GPRTAccel world = gprtInstanceAccelCreate(context, 1, instanceBuffer);
  gprtAccelBuild(context, world, GPRT_BUILD_MODE_FAST_TRACE_NO_UPDATE);

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
