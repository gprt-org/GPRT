#include <gprt.h>      // Public GPRT API
#include "sharedCode.h" // Shared data between host and device
#include <iostream>    // For input/output

// External reference to device code
extern GPRTProgram s00_deviceCode;

// Image resolution and output file name
const int2 fbSize = {1400, 460};
const char *outFileName = "s00-singleRayGen.png";

int main() {
  // Initialize a window for rendering
  gprtRequestWindow(fbSize.x, fbSize.y, "Single Ray Generation Example");

  // Create a GPRT context for managing the rendering process
  GPRTContext context = gprtContextCreate();

  // Compile device code into a module for GPU execution
  GPRTModule module = gprtModuleCreate(context, s00_deviceCode);

  // Create a ray generation shader using the module
  GPRTRayGenOf<RayGenData> rayGen = gprtRayGenCreate<RayGenData>(context, module, "simpleRayGen");

  // Create a framebuffer to store pixel colors
  GPRTBufferOf<uint32_t> frameBuffer = gprtDeviceBufferCreate<uint32_t>(context, fbSize.x * fbSize.y);

  // Set up shader parameters
  RayGenData *data = gprtRayGenGetParameters(rayGen);
  data->color0 = float3(0.1f, 0.1f, 0.1f); // Background color
  data->color1 = float3(0.0f, 0.0f, 0.0f); // Secondary color
  data->frameBuffer = gprtBufferGetDevicePointer(frameBuffer);

  // Build the Shader Binding Table (SBT)
  gprtBuildShaderBindingTable(context, GPRT_SBT_RAYGEN);

  // Render loop: repeatedly launch the ray generation shader
  do {
    gprtRayGenLaunch2D(context, rayGen, fbSize.x, fbSize.y);
    gprtBufferPresent(context, frameBuffer); // Display to window if available
  } while (!gprtWindowShouldClose(context));

  // Save the final image to a file
  gprtBufferSaveImage(frameBuffer, fbSize.x, fbSize.y, outFileName);

  // Clean up resources
  gprtContextDestroy(context);

  return 0;
}