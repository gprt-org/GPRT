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

#define LOG(message)                                                           \
  std::cout << GPRT_TERMINAL_BLUE;                                             \
  std::cout << "#gprt.sample(main): " << message << std::endl;                 \
  std::cout << GPRT_TERMINAL_DEFAULT;
#define LOG_OK(message)                                                        \
  std::cout << GPRT_TERMINAL_LIGHT_BLUE;                                       \
  std::cout << "#gprt.sample(main): " << message << std::endl;                 \
  std::cout << GPRT_TERMINAL_DEFAULT;

extern GPRTProgram t00_deviceCode;

// initial image resolution
const int2 fbSize = {1400, 460};

// final image output
const char *outFileName = "s00-rayGenOnly.png";

#include <iostream>
int main(int ac, char **av) {
  // The output window will show comments for many of the methods called.
  // Walking through the code line by line with a debugger is educational.
  LOG("gprt example '" << av[0] << "' starting up");

  // ##################################################################
  // set up all the GPU kernels we want to run
  // ##################################################################

  LOG("building module, programs, and pipeline");

  // Initialize Vulkan, and create a "gprt device," a context to hold the
  // ray generation shader and output buffer. The "1" is the number of devices
  // requested.
  GPRTContext gprt = gprtContextCreate(nullptr, 1);

  // SPIR-V is the intermediate code that the GPU deviceCode.hlsl shader program
  // is converted into. You can see the machine-centric SPIR-V code in
  // build\samples\cmd00-rayGenOnly\deviceCode.spv
  // We store this SPIR-V intermediate code representation in a GPRT module.
  GPRTModule module = gprtModuleCreate(gprt, t00_deviceCode);

  // All ray tracing programs start off with a "Ray Generation" kernel.
  // Allocate room for one RayGen shader, create it, and hold on to it with
  // the "gprt" context
  GPRTRayGenOf<RayGenData> rayGen =
      gprtRayGenCreate<RayGenData>(gprt, module, "simpleRayGen");

  // (re-)builds all vulkan programs, with current pipeline settings
  gprtBuildPipeline(gprt);

  // ------------------------------------------------------------------
  // allocating buffers
  // ------------------------------------------------------------------

  // Our framebuffer here will be used to hold pixel color values
  // that we'll present to the window / save to an image
  LOG("allocating frame buffer");
  GPRTBufferOf<XYZW> testBuffer = gprtDeviceBufferCreate<XYZW>(gprt, 1);

  // ------------------------------------------------------------------
  // build the shader binding table, used by rays to map geometry,
  // instances and ray types to GPU kernels
  // ------------------------------------------------------------------
  RayGenData *data = gprtRayGenGetPointer(rayGen);
  data->test = gprtBufferGetHandle(testBuffer);
  
  // Build a shader binding table entry for the ray generation record.
  gprtBuildShaderBindingTable(gprt, GPRT_SBT_RAYGEN);

  // Calls the GPU raygen kernel function
  gprtRayGenLaunch2D(gprt, rayGen, 1, 1);

  // ##################################################################
  // now that everything is ready: launch it ....
  // ##################################################################
  
  // print it out
  gprtBufferMap(testBuffer);
  XYZW *test = gprtBufferGetPointer(testBuffer);
  std::cout<< test->x << " " << test->y << " " << test->z << " " << test->w << std::endl;
  gprtBufferUnmap(testBuffer);

  // ##################################################################
  // and finally, clean up
  // ##################################################################

  LOG("cleaning up ...");
  gprtBufferDestroy(testBuffer);
  gprtRayGenDestroy(rayGen);
  gprtModuleDestroy(module);
  gprtContextDestroy(gprt);

  LOG_OK("seems all went OK; app is done, this should be the last output ...");
}
