// MIT License

// Copyright (c) 2022 Nathan V. Morrical

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

// public GPRT API
#include <gprt.h>

// our device-side data structures
#include "deviceCode.h"

#define LOG(message)                                            \
  std::cout << GPRT_TERMINAL_BLUE;                               \
  std::cout << "#gprt.sample(main): " << message << std::endl;   \
  std::cout << GPRT_TERMINAL_DEFAULT;
#define LOG_OK(message)                                         \
  std::cout << GPRT_TERMINAL_LIGHT_BLUE;                         \
  std::cout << "#gprt.sample(main): " << message << std::endl;   \
  std::cout << GPRT_TERMINAL_DEFAULT;

extern GPRTProgram int00_deviceCode;

// initial image resolution
const int2 fbSize = {800,600};

#include <iostream>
int main(int ac, char **av)
{
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
  // ray generation shader and output buffer. The "1" is the number of devices requested.
  GPRTContext gprt = gprtContextCreate(nullptr, 1);

  // SPIR-V is the intermediate code that the GPU deviceCode.hlsl shader program is converted into.
  // You can see the machine-centric SPIR-V code in
  // build\samples\cmd00-rayGenOnly\deviceCode.spv
  // We store this SPIR-V intermediate code representation in a GPRT module.
  GPRTModule module = gprtModuleCreate(gprt,int00_deviceCode);

  // All ray tracing programs start off with a "Ray Generation" kernel.
  // All "parameters" we'll pass to that ray generation kernel are defined here.
  GPRTVarDecl rayGenVars[]
    = {
      { "fbPtr", GPRT_BUFFER, GPRT_OFFSETOF(RayGenData, fbPtr) },
      { "fbSize", GPRT_INT2,  GPRT_OFFSETOF(RayGenData, fbSize) },
      { "color0", GPRT_FLOAT3, GPRT_OFFSETOF(RayGenData, color0) },
      { "color1", GPRT_FLOAT3, GPRT_OFFSETOF(RayGenData, color1) },
      { /* sentinel: */ nullptr }
  };
  // Allocate room for one RayGen shader, create it, and
  // hold on to it with the "gprt" context
  GPRTRayGen rayGen
      = gprtRayGenCreate(gprt, module, "simpleRayGen",
                      sizeof(RayGenData),rayGenVars,-1);

  // (re-)builds all vulkan programs, with current pipeline settings
  gprtBuildPipeline(gprt);

  // ------------------------------------------------------------------
  // allocating buffers
  // ------------------------------------------------------------------
  
  // Our framebuffer here will be used to hold pixel color values
  // that we'll present to the window / save to an image
  LOG("allocating frame buffer");
  GPRTBuffer frameBuffer = gprtDeviceBufferCreate(gprt,
                                          /*type:*/GPRT_INT,
                                          /*size:*/fbSize.x*fbSize.y);

  // ------------------------------------------------------------------
  // build the shader binding table, used by rays to map geometry, 
  // instances and ray types to GPU kernels
  // ------------------------------------------------------------------
  gprtRayGenSet3f(rayGen,"color0",0.1f,0.1f,0.1f);
  gprtRayGenSet3f(rayGen,"color1",.0f,.0f,.0f);
  gprtRayGenSetBuffer(rayGen,"fbPtr",frameBuffer);
  gprtRayGenSet2i(rayGen,"fbSize",fbSize.x,fbSize.y);
  
  // Build a shader binding table entry for the ray generation record.
  gprtBuildShaderBindingTable(gprt, GPRT_SBT_RAYGEN);


  // ##################################################################
  // now that everything is ready: launch it ....
  // ##################################################################
  LOG("executing the launch ...");
  while (!gprtWindowShouldClose(gprt))
  {
    gprtRayGenLaunch2D(gprt,rayGen,fbSize.x,fbSize.y);
    gprtPresentBuffer(gprt, frameBuffer);
  } while (!gprtWindowShouldClose(gprt));

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
