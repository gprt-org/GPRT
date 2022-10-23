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
// external helper stuff for image output
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb/stb_image_write.h"

#define LOG(message)                                            \
  std::cout << GPRT_TERMINAL_BLUE;                               \
  std::cout << "#gprt.sample(main): " << message << std::endl;   \
  std::cout << GPRT_TERMINAL_DEFAULT;
#define LOG_OK(message)                                         \
  std::cout << GPRT_TERMINAL_LIGHT_BLUE;                         \
  std::cout << "#gprt.sample(main): " << message << std::endl;   \
  std::cout << GPRT_TERMINAL_DEFAULT;

extern std::map<std::string, std::vector<uint8_t>> cmd00_deviceCode; // wrap this map into some internal type

// When run, this program produces this PNG as output.
// In this case the correct result is a red and light gray checkerboard,
// as nothing is actually rendered
const char *outFileName = "s00-rayGenOnly.png";
// image resolution
const int2 fbSize = {800,600};

#include <iostream>
int main(int ac, char **av)
{
  // The output window will show comments for many of the methods called.
  // Walking through the code line by line with a debugger is educational.
  LOG("gprt example '" << av[0] << "' starting up");

  // ##################################################################
  // set up all the *CODE* we want to run
  // ##################################################################

  LOG("building module, programs, and pipeline");

  // Initialize Vulkan, and create a "gprt device," a context to hold the
  // ray generation shader and output buffer. The "1" is the number of devices requested.
  GPRTContext gprt = gprtContextCreate(nullptr, 1);

  // SPIR-V is the intermediate code that the GPU deviceCode.hlsl shader program is converted into.
  // You can see the machine-centric SPIR-V code in
  // build\samples\cmd00-rayGenOnly\deviceCode.spv
  // We store this SPIR-V intermediate code representation in a GPRT module.
  GPRTModule module = gprtModuleCreate(gprt,cmd00_deviceCode);

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
                      sizeof(RayGenData),rayGenVars,-1); // -1 maybe should be the default here...

  // (re-)builds all vulkan programs, with current pipeline settings
  gprtBuildPrograms(gprt);

  // ------------------------------------------------------------------
  // alloc buffers
  // ------------------------------------------------------------------
  LOG("allocating frame buffer");
  // Create a frame buffer as page-locked, aka "pinned" memory.
  // GPU writes to CPU memory directly (slow) but no transfers needed
  GPRTBuffer frameBuffer = gprtHostBufferCreate(gprt,
                                          /*type:*/GPRT_INT,
                                          /*size:*/fbSize.x*fbSize.y);

  // ------------------------------------------------------------------
  // build Shader Binding Table (SBT) required to trace the groups
  // ------------------------------------------------------------------
  gprtRayGenSet3f(rayGen,"color0",0.1f,0.1f,0.1f);
  gprtRayGenSet3f(rayGen,"color1",.0f,.0f,.0f);
  gprtRayGenSetBuffer(rayGen,"fbPtr",frameBuffer);
  gprtRayGenSet2i(rayGen,"fbSize",fbSize.x,fbSize.y);
  // Build a shader binding table entry for the ray generation record.
  gprtBuildSBT(gprt);

  // ##################################################################
  // now that everything is ready: launch it ....
  // ##################################################################
  LOG("executing the launch ...");
  gprtRayGenLaunch2D(gprt,rayGen,fbSize.x,fbSize.y);

  LOG("done with launch, writing frame buffer to " << outFileName);
  const uint32_t *fb = (const uint32_t*)gprtBufferGetPointer(frameBuffer,0);
  stbi_write_png(outFileName,fbSize.x,fbSize.y,4,
                 fb,(uint32_t)(fbSize.x)*sizeof(uint32_t));
  LOG_OK("written rendered frame buffer to file "<<outFileName);

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
