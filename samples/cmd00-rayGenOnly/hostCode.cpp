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

// public VKRT API
#include <vkrt.h>
// our device-side data structures
#include "deviceCode.h"
// external helper stuff for image output
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb/stb_image_write.h"

#define LOG(message)                                            \
  std::cout << VKRT_TERMINAL_BLUE;                               \
  std::cout << "#vkrt.sample(main): " << message << std::endl;   \
  std::cout << VKRT_TERMINAL_DEFAULT;
#define LOG_OK(message)                                         \
  std::cout << VKRT_TERMINAL_LIGHT_BLUE;                         \
  std::cout << "#vkrt.sample(main): " << message << std::endl;   \
  std::cout << VKRT_TERMINAL_DEFAULT;

extern "C" char deviceCode_spv[];
extern "C" uint32_t deviceCode_spv_size;

// extern "C" char simpleRayGen_spv[];
// extern "C" uint32_t simpleRayGen_spv_size;
// extern "C" char simpleMissProg_spv[];
// extern "C" uint32_t simpleMissProg_spv_size;

const int2 fbSize = {1,1};

#include <iostream>
int main(int ac, char **av) 
{
    LOG("vkrt example '" << av[0] << "' starting up");

    // Initialize Vulkan, and create a "vkrt device," a context to hold the
    // ray generation shader and output buffer. The "1" is the number of devices requested.
    VKRTContext vkrt = vkrtContextCreate(nullptr, 1);

    VKRTModule module = vkrtModuleCreate(vkrt,std::string(deviceCode_spv, deviceCode_spv + deviceCode_spv_size).c_str());
    
    VKRTVarDecl rayGenVars[]
        = {
        { "data", VKRT_INT,    VKRT_OFFSETOF(RayGenData,data) },
        { /* sentinel: */ nullptr }
    };
    // Allocate room for one RayGen shader, create it, and
    // hold on to it with the "owl" context
    VKRTRayGen rayGen
        = vkrtRayGenCreate(vkrt, module, "simpleRayGen",
                        sizeof(RayGenData),rayGenVars,-1);

    VKRTVarDecl missProgVars[]
        = {
        { /* sentinel: */ nullptr }
    };
    VKRTMissProg missProg
        = vkrtMissProgCreate(vkrt, module, "simpleMissProg",
                            sizeof(MissProgData),missProgVars,-1);

    // (re-)builds all optix programs, with current pipeline settings
    // vkrtBuildPrograms(vkrt);

    // Create the pipeline. Note that vkrt will (kindly) warn there are no geometry and no miss programs defined.
    vkrtBuildPipeline(vkrt);

    // Build a shader binding table entry for the ray generation record.
    vkrtBuildSBT(vkrt);

    int data = 42;
    vkrtRayGenSetRaw(rayGen, "data", &data);
    vkrtRayGenLaunch2D(vkrt,rayGen,fbSize.x,fbSize.y);


    // Now finally, cleanup
    vkrtMissProgRelease(missProg);
    vkrtRayGenRelease(rayGen);
    vkrtContextDestroy(vkrt);
}

