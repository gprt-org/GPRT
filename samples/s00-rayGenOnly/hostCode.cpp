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

extern GPRTProgram s00_deviceCode;

// initial image resolution
const int2 fbSize = {1400, 460};

// final image output
const char *outFileName = "s00-rayGenOnly.png";

#include <iostream>
int main(int ac, char **av)
{
  // The output window will show comments for many of the methods called.
  // We recommend walking through the code line by line with a debugger. 
  LOG("gprt example '" << av[0] << "' starting up");

  // ##################################################################
  // set up all the GPU kernels we want to run
  // ##################################################################

  LOG("building module, programs, and pipeline");

  // Since we'll be using a window, we request support for one here. 
  // If a window can't be made, we can still use GPRT, but a window wont appear. 
  // And if you know you don't need a window, then this call is optional.
  gprtRequestWindow(fbSize.x, fbSize.y, "S00 Raygen Only");

  // Initialize GPRT. Under the hood, this identifies all compatible GPUs (ones
  // that support ray tracing) selects from that compatible list. If a window
  // was requested, one will appear after this call. 
  GPRTContext context = gprtContextCreate();

  // Device code is represented through SPIR-V, which is the intermediate code 
  // that the GPU deviceCode.hlsl shader program is compiled into.
  // You can see the machine-centric SPIR-V code in
  // build\s00-rayGenOnly\deviceCode.spv
  // We store this SPIR-V intermediate code representation in a GPRT module.
  GPRTModule module = gprtModuleCreate(context, s00_deviceCode);

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
      = gprtRayGenCreate(context, module, "simpleRayGen",
                      sizeof(RayGenData),rayGenVars,-1);

  // (re-)builds all gprt programs, with current pipeline settings
  gprtBuildPipeline(context);

  // ##################################################################
  // allocating GPU buffers
  // ##################################################################
  
  // When working with GPUs, we work with two different pools of memory:
  // traditional CPU memory and GPU memory. GPU programs can only read
  // from GPU buffers of memory. The CPU can temporarily read and write
  // to GPU bufer memory when that buffer is "mapped" to the CPU.

  // Our frame buffer here will be used to hold pixel color values
  // that we'll present to the window and save to an image
  LOG("allocating frame buffer");
  GPRTBuffer frameBuffer = gprtDeviceBufferCreate(
                              context,
                              GPRT_INT, /* type */ 
                              fbSize.x*fbSize.y /* size in count */);
  // See also gprtHostBufferCreate and gprtSharedBufferCreate.
  // To get a pointer to this buffer, call gprtBufferMap, then 
  // gprtBufferGetPointer. When done writing to the pointer, call 
  // gprtBufferUnmap.

  // ##################################################################
  // Build the shader binding table
  // ##################################################################
  
  // The shader binding table is used by the GPU to map parameters to GPU 
  // kernels. 

  gprtRayGenSet3f(rayGen,"color0",0.1f,0.1f,0.1f);
  gprtRayGenSet3f(rayGen,"color1",.0f,.0f,.0f);
  gprtRayGenSetBuffer(rayGen,"fbPtr",frameBuffer);
  gprtRayGenSet2i(rayGen,"fbSize",fbSize.x,fbSize.y);
  
  // Build a shader binding table entry for the ray generation record.
  gprtBuildShaderBindingTable(context, GPRT_SBT_RAYGEN);


  // ##################################################################
  // Launching
  // ##################################################################

  // To execute code on the GPU, we use "gprtRayGenLaunch". Only ray generation
  // and compute programs can be launched. 
  // ( compute programs will be covered later :) )
  LOG("executing the launch ...");
  do 
  {
    // Calls the GPU raygen kernel function
    gprtRayGenLaunch2D(context,rayGen,fbSize.x,fbSize.y);
    
    // If a window exists, presents the frame buffer here to that window
    gprtBufferPresent(context, frameBuffer); 
  } 
  // returns true if "X" pressed or if in "headless" mode
  while (!gprtWindowShouldClose(context)); 

  // Save final frame to an image
  LOG("done with launch, writing frame buffer to " << outFileName);
  gprtBufferSaveImage(frameBuffer, fbSize.x, fbSize.y, outFileName);
  LOG_OK("written rendered frame buffer to file "<<outFileName);

  // ##################################################################
  // and finally, clean up
  // ##################################################################

  LOG("cleaning up ...");
  gprtBufferDestroy(frameBuffer);
  gprtRayGenDestroy(rayGen);
  gprtModuleDestroy(module);

  // If a window was made, this call is the one that will actually destroy 
  // that window. 
  gprtContextDestroy(context);

  LOG_OK("seems all went OK; app is done, this should be the last output ...");
}
