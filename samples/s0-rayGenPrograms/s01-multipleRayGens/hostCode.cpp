#include <gprt.h>      // Public GPRT API
#include "sharedCode.h" // Shared data between host and device
#include <iostream>    // For input/output

// External reference to device code
extern GPRTProgram s01_deviceCode;

// Image resolution and output file name
const int2 fbSize = {1400, 460};
const char *outFileName = "s01-multipleRayGens.png";

int main() {
  // Initialize a window for rendering
  gprtRequestWindow(fbSize.x, fbSize.y, "Multiple Ray Generation Example");

  // Create a GPRT context for managing the rendering process
  GPRTContext context = gprtContextCreate();

  // Compile device code into a module for GPU execution
  GPRTModule module = gprtModuleCreate(context, s01_deviceCode);

  // Create two ray generation shaders using the module
  GPRTRayGenOf<RayGenData> firstRayGen = gprtRayGenCreate<RayGenData>(context, module, "firstRayGen");
  GPRTRayGenOf<RayGenData> secondRayGen = gprtRayGenCreate<RayGenData>(context, module, "secondRayGen");

  // Create a framebuffer to store pixel colors
  GPRTBufferOf<uint32_t> frameBuffer = gprtDeviceBufferCreate<uint32_t>(context, fbSize.x * fbSize.y);

  // When two programs use the same parameters, we can share a common structure like this:
  RayGenData data;
  data.color0 = float3(0.1f, 0.1f, 0.1f); // Background color
  data.color1 = float3(0.0f, 0.0f, 0.0f); // Secondary color
  data.frameBuffer = gprtBufferGetDevicePointer(frameBuffer);
  gprtRayGenSetParameters(firstRayGen, &data);
  gprtRayGenSetParameters(secondRayGen, &data);
  
  // Build the Shader Binding Table (SBT)
  gprtBuildShaderBindingTable(context, GPRT_SBT_RAYGEN);

  // Render loop: repeatedly launch both ray generation shaders, one after the other.
  do {
    gprtRayGenLaunch2D(context, firstRayGen, fbSize.x / 2, fbSize.y);
    gprtRayGenLaunch2D(context, secondRayGen, fbSize.x / 2, fbSize.y);
    gprtBufferPresent(context, frameBuffer); // Display to window if available
  } while (!gprtWindowShouldClose(context));

  // Save the final image to a file
  gprtBufferSaveImage(frameBuffer, fbSize.x, fbSize.y, outFileName);

  // Clean up resources
  gprtContextDestroy(context);

  return 0;
}