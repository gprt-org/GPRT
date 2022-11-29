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
// GLuint fbTexture {0};

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

  // Since we'll be using a window, we request support for an image swapchain
  gprtRequestWindow(fbSize.x, fbSize.y, "Int00 Raygen Only");

  // Initialize Vulkan, and create a "gprt device," a context to hold the
  // ray generation shader and output buffer. The "1" is the number of devices requested.
  GPRTContext gprt = gprtContextCreate(nullptr, 1);

  // SPIR-V is the intermediate code that the GPU deviceCode.hlsl shader program is converted into.
  // You can see the machine-centric SPIR-V code in
  // build\samples\cmd00-rayGenOnly\deviceCode.spv
  // We store this SPIR-V intermediate code representation in a GPRT module.
  GPRTModule module = gprtModuleCreate(gprt,int00_deviceCode);

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
  gprtBuildShaderBindingTable(gprt);


  // ##################################################################
  // create a window we can use to display and interact with the image
  // ##################################################################

  void* pixels = gprtBufferGetPointer(frameBuffer);

  // ##################################################################
  // now that everything is ready: launch it ....
  // ##################################################################
  LOG("executing the launch ...");
  // while (!glfwWindowShouldClose(window))
  while (!gprtWindowShouldClose(gprt))
  {
    gprtRayGenLaunch2D(gprt,rayGen,fbSize.x,fbSize.y);

    gprtSwapBuffers(gprt);

    // if (fbTexture == 0)
    //   glGenTextures(1, &fbTexture);

    // glBindTexture(GL_TEXTURE_2D, fbTexture);
    // GLenum texFormat = GL_RGBA;
    // GLenum texelType = GL_UNSIGNED_BYTE;
    // glTexImage2D(GL_TEXTURE_2D, 0, texFormat, fbSize.x, fbSize.y, 0, GL_RGBA,
    //               texelType, pixels);

    // glDisable(GL_LIGHTING);
    // glColor3f(1, 1, 1);

    // glMatrixMode(GL_MODELVIEW);
    // glLoadIdentity();

    // glEnable(GL_TEXTURE_2D);
    // glBindTexture(GL_TEXTURE_2D, fbTexture);
    // glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    // glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    // glDisable(GL_DEPTH_TEST);

    // glViewport(0, 0, fbSize.x, fbSize.y);

    // glMatrixMode(GL_PROJECTION);
    // glLoadIdentity();
    // glOrtho(0.f, (float)fbSize.x, 0.f, (float)fbSize.y, -1.f, 1.f);

    // glBegin(GL_QUADS);
    // {
    //   glTexCoord2f(0.f, 0.f);
    //   glVertex3f(0.f, 0.f, 0.f);

    //   glTexCoord2f(0.f, 1.f);
    //   glVertex3f(0.f, (float)fbSize.y, 0.f);

    //   glTexCoord2f(1.f, 1.f);
    //   glVertex3f((float)fbSize.x, (float)fbSize.y, 0.f);

    //   glTexCoord2f(1.f, 0.f);
    //   glVertex3f((float)fbSize.x, 0.f, 0.f);
    // }
    // glEnd();

    // glfwSwapBuffers(window);
    // glfwPollEvents();
  }

  // ##################################################################
  // and finally, clean up
  // ##################################################################

  // glfwDestroyWindow(window);
  // glfwTerminate();

  LOG("cleaning up ...");
  gprtBufferDestroy(frameBuffer);
  gprtRayGenDestroy(rayGen);
  gprtModuleDestroy(module);
  gprtContextDestroy(gprt);

  LOG_OK("seems all went OK; app is done, this should be the last output ...");
}
