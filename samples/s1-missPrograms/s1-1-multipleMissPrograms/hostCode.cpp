#include <gprt.h>      // Public GPRT API
#include "sharedCode.h" // Shared data between host and device

// External reference to device code
extern GPRTProgram s1_1_deviceCode;

// Image resolution and output file name
const int2 fbSize = {1400, 460};
const char *outFileName = "s1-1-multipleMiss.png";

int main() {
  gprtRequestWindow(fbSize.x, fbSize.y, "Multiple Miss Programs Example");
  GPRTContext context = gprtContextCreate();
  GPRTModule module = gprtModuleCreate(context, s1_1_deviceCode);
  GPRTRayGenOf<RayGenData> rayGen = gprtRayGenCreate<RayGenData>(context, module, "raygen");

  // New: Create two miss programs
  GPRTMissOf<MissProgData> firstMiss = gprtMissCreate<MissProgData>(context, module, "firstMiss");
  GPRTMissOf<MissProgData> secondMiss = gprtMissCreate<MissProgData>(context, module, "secondMiss");

  // Set constant raygen parameters
  GPRTBufferOf<uint32_t> frameBuffer = gprtDeviceBufferCreate<uint32_t>(context, fbSize.x * fbSize.y);
  RayGenData *raygenRecord = gprtRayGenGetParameters(rayGen);
  raygenRecord->frameBuffer = gprtBufferGetDevicePointer(frameBuffer);
  raygenRecord->firstMissIndex = gprtMissGetIndex(firstMiss);
  raygenRecord->secondMissIndex = gprtMissGetIndex(secondMiss);
  
  // Set constant miss parameters
  MissProgData *firstMissRecord = gprtMissGetParameters(firstMiss);
  firstMissRecord->maxIterations = 100;

  MissProgData *secondMissRecord = gprtMissGetParameters(secondMiss);
  secondMissRecord->maxIterations = 10;

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