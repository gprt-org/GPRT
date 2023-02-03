// MIT License

// Copyright (c) 2022 Nathan V. Morrical

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

// public GPRT API
#include <gprt.h>

// our shared data structures between host and device
#include "sharedCode.h"

#define LOG(message)                                                                                                   \
  std::cout << GPRT_TERMINAL_BLUE;                                                                                     \
  std::cout << "#gprt.sample(main): " << message << std::endl;                                                         \
  std::cout << GPRT_TERMINAL_DEFAULT;
#define LOG_OK(message)                                                                                                \
  std::cout << GPRT_TERMINAL_LIGHT_BLUE;                                                                               \
  std::cout << "#gprt.sample(main): " << message << std::endl;                                                         \
  std::cout << GPRT_TERMINAL_DEFAULT;

extern GPRTProgram s00_deviceCode;

// initial image resolution
const int2 fbSize = {1400, 460};

// final image output
const char *outFileName = "s00-rayGenOnly.png";

#include <iostream>
int
main(int ac, char **av) {
  // The output window will show comments for many of the methods called.
  // Walking through the code line by line with a debugger is educational.
  LOG("gprt example '" << av[0] << "' starting up");

  // ##################################################################
  // set up all the GPU kernels we want to run
  // ##################################################################

  LOG("building module, programs, and pipeline");

  // Since we'll be using a window, we request support for an image swapchain
  // If a window can't be made, we can still use GPRT, but a window wont appear.
  gprtRequestWindow(fbSize.x, fbSize.y, "Int00 Raygen Only");

  // Initialize Vulkan, and create a "gprt device," a context to hold the
  // ray generation shader and output buffer. The "1" is the number of devices
  // requested.
  GPRTContext gprt = gprtContextCreate(nullptr, 1);

  // SPIR-V is the intermediate code that the GPU deviceCode.hlsl shader program
  // is converted into. You can see the machine-centric SPIR-V code in
  // build\samples\cmd00-rayGenOnly\deviceCode.spv
  // We store this SPIR-V intermediate code representation in a GPRT module.
  GPRTModule module = gprtModuleCreate(gprt, s00_deviceCode);

  // All ray tracing programs start off with a "Ray Generation" kernel.
  // Allocate room for one RayGen shader, create it, and hold on to it with
  // the "gprt" context
  GPRTRayGenOf<RayGenData> rayGen = gprtRayGenCreate<RayGenData>(gprt, module, "simpleRayGen");

  // ------------------------------------------------------------------
  // allocating buffers
  // ------------------------------------------------------------------

  // Our framebuffer here will be used to hold pixel color values
  // that we'll present to the window / save to an image
  LOG("allocating frame buffer");
  GPRTBufferOf<uint32_t> frameBuffer = gprtDeviceBufferCreate<uint32_t>(gprt, fbSize.x * fbSize.y);

  // ------------------------------------------------------------------
  // build the shader binding table, used by rays to map geometry,
  // instances and ray types to GPU kernels
  // ------------------------------------------------------------------
  RayGenData *data = gprtRayGenGetParameters(rayGen);
  data->color0 = float3(0.1f, 0.1f, 0.1f);
  data->color1 = float3(0.0f, 0.0f, 0.0f);
  data->frameBuffer = gprtBufferGetHandle(frameBuffer);

  // Build a shader binding table entry for the ray generation record.
  gprtBuildShaderBindingTable(gprt, GPRT_SBT_RAYGEN);

  // ##################################################################
  // now that everything is ready: launch it ....
  // ##################################################################
  LOG("executing the launch ...");
  do {
    // Calls the GPU raygen kernel function
    gprtRayGenLaunch2D(gprt, rayGen, fbSize.x, fbSize.y);

    // If a window exists, presents the framebuffer here to that window
    gprtBufferPresent(gprt, frameBuffer);
  }
  // returns true if "X" pressed or if in "headless" mode
  while (!gprtWindowShouldClose(gprt));

  // Save final frame to an image
  LOG("done with launch, writing frame buffer to " << outFileName);
  gprtBufferSaveImage(frameBuffer, fbSize.x, fbSize.y, outFileName);
  LOG_OK("written rendered frame buffer to file " << outFileName);

  // ##################################################################
  // and finally, clean up
  // ##################################################################

  LOG("cleaning up ...");
  gprtBufferDestroy(frameBuffer);
  gprtRayGenDestroy(rayGen);
  gprtModuleDestroy(module);
  gprtContextDestroy(gprt);

  LOG_OK("seems all went OK; app is done, this should be the last output ...");
}
