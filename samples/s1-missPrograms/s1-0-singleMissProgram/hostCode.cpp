#include <gprt.h>      // Public GPRT API
#include "sharedCode.h" // Shared data between host and device

// External reference to device code
extern GPRTProgram s1_0_deviceCode;

// Image resolution and output file name
const int2 fbSize = {1400, 460};
const char *outFileName = "s1-0-singleMiss.png";

int main() {
  gprtRequestWindow(fbSize.x, fbSize.y, "Single Miss Program Example");
  GPRTContext context = gprtContextCreate();
  GPRTModule module = gprtModuleCreate(context, s1_0_deviceCode);
  GPRTRayGenOf<RayGenData> rayGen = gprtRayGenCreate<RayGenData>(context, module, "raygen");

  // New: Create a single miss program
  GPRTMissOf<MissProgData> miss = gprtMissCreate<MissProgData>(context, module, "miss");

  // Set constant raygen parameters
  GPRTBufferOf<uint32_t> frameBuffer = gprtDeviceBufferCreate<uint32_t>(context, fbSize.x * fbSize.y);
  RayGenData *raygenRecord = gprtRayGenGetParameters(rayGen);
  raygenRecord->frameBuffer = gprtBufferGetDevicePointer(frameBuffer);
  
  // Set constant miss parameters
  MissProgData *missRecord = gprtMissGetParameters(miss);
  missRecord->maxIterations = 100;

  // Build the Shader Binding Table (SBT), updating both ray generation and miss parameters.
  gprtBuildShaderBindingTable(context, (GPRTBuildSBTFlags)(GPRT_SBT_RAYGEN | GPRT_SBT_MISS));

  // Render loop: repeatedly launch the ray generation shader
  do {
    gprtRayGenLaunch2D(context, rayGen, fbSize.x, fbSize.y);
    gprtBufferPresent(context, frameBuffer); // Display to window if available
  } 
  // returns true if "X" pressed or if in "headless" mode
  while (!gprtWindowShouldClose(context));

  // Save the final image to a file
  gprtBufferSaveImage(frameBuffer, fbSize.x, fbSize.y, outFileName);

  // Clean up resources
  gprtContextDestroy(context);

  return 0;
}