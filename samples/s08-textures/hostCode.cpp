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

// for generating meshes
#include <generator.hpp>
using namespace generator;

// library for image output
#define STB_IMAGE_STATIC
#define STB_IMAGE_IMPLEMENTATION
#include "stb/stb_image.h"

#define LOG(message)                                                           \
  std::cout << GPRT_TERMINAL_BLUE;                                             \
  std::cout << "#gprt.sample(main): " << message << std::endl;                 \
  std::cout << GPRT_TERMINAL_DEFAULT;
#define LOG_OK(message)                                                        \
  std::cout << GPRT_TERMINAL_LIGHT_BLUE;                                       \
  std::cout << "#gprt.sample(main): " << message << std::endl;                 \
  std::cout << GPRT_TERMINAL_DEFAULT;

template <typename T> struct Mesh {
  std::vector<float3> vertices;
  std::vector<float2> texcoords;
  std::vector<uint3> indices;
  GPRTBufferOf<float2> texcoordBuffer;
  GPRTBufferOf<float3> vertexBuffer;
  GPRTBufferOf<uint3> indexBuffer;
  GPRTGeomOf<TrianglesGeomData> geometry;

  Mesh(){};
  Mesh(GPRTContext context, GPRTGeomTypeOf<TrianglesGeomData> geomType, T generator, 
      GPRTTextureOf<stbi_uc> texture, GPRTSampler sampler, float4x4 transform) {
    auto vertGenerator = generator.vertices();
    auto triGenerator = generator.triangles();
    while (!vertGenerator.done()) {
      auto vertex = vertGenerator.generate();
      auto position = vertex.position;
      float4 p = mul(transform, float4(vertex.position[0], vertex.position[1],
                                       vertex.position[2], 1.0));
      float2 tc = float2(vertex.texCoord[0], vertex.texCoord[1]);
      vertices.push_back(p.xyz());
      texcoords.push_back(tc);
      vertGenerator.next();
    }
    while (!triGenerator.done()) {
      Triangle triangle = triGenerator.generate();
      auto vertices = triangle.vertices;
      indices.push_back(uint3(vertices[0], vertices[1], vertices[2]));
      triGenerator.next();
    }

    vertexBuffer = gprtDeviceBufferCreate<float3>(context, vertices.size(),
                                          vertices.data());
    texcoordBuffer = gprtDeviceBufferCreate<float2>(context, texcoords.size(),
                                          texcoords.data());
    indexBuffer = gprtDeviceBufferCreate<uint3>(context, indices.size(),
                                         indices.data());
    geometry = gprtGeomCreate(context, geomType);
    TrianglesGeomData *geomData = gprtGeomGetPointer(geometry);
    gprtTrianglesSetVertices(geometry, vertexBuffer, vertices.size());
    gprtTrianglesSetIndices(geometry, indexBuffer, indices.size());
    geomData->vertex = gprtBufferGetHandle(vertexBuffer);
    geomData->texcoord = gprtBufferGetHandle(texcoordBuffer);
    geomData->index = gprtBufferGetHandle(indexBuffer);
    geomData->texture = gprtTextureGetHandle(texture);
    geomData->sampler = gprtSamplerGetHandle(sampler);
  };

  void cleanup() {
    gprtGeomDestroy(geometry);
    gprtBufferDestroy(vertexBuffer);
    gprtBufferDestroy(texcoordBuffer);
    gprtBufferDestroy(indexBuffer);
  };
};

extern GPRTProgram s08_deviceCode;

// initial image resolution
 int2 fbSize = {1024, 1024};

// final image output
const char *outFileName = "s08-textures.png";

float3 lookFrom = {0.f, 0.0f, 10.f};
float3 lookAt = {0.f, 0.f, 0.f};
float3 lookUp = {0.f, 1.f, 0.f};
float cosFovy = 0.4f;

#include <iostream>
int main(int ac, char **av) {
  // The output window will show comments for many of the methods called.
  // Walking through the code line by line with a debugger is educational.
  LOG("gprt example '" << av[0] << "' starting up");

  // ##################################################################
  // set up all the GPU kernels we want to run
  // ##################################################################

  LOG("building module, programs, and pipeline");

  int texWidth, texHeight, texChannels;
  stbi_uc* pixels = stbi_load(ASSETS_DIRECTORY "image.png", &texWidth, &texHeight, &texChannels, STBI_rgb_alpha);
  // stbi_uc* pixels = stbi_load("C:/Users/Nate/Pictures/caribou.png", &texWidth, &texHeight, &texChannels, STBI_rgb_alpha);

  // Since we'll be using a window, we request support for an image swapchain
  // If a window can't be made, we can still use GPRT, but a window wont appear.
  gprtRequestWindow(fbSize.x, fbSize.y, "Int00 Raygen Only");

  // Initialize Vulkan, and create a "gprt device," a context to hold the
  // ray generation shader and output buffer. The "1" is the number of devices
  // requested.
  GPRTContext context = gprtContextCreate(nullptr, 1);

  // SPIR-V is the intermediate code that the GPU deviceCode.hlsl shader program
  // is converted into. You can see the machine-centric SPIR-V code in
  // build\samples\cmd00-rayGenOnly\deviceCode.spv
  // We store this SPIR-V intermediate code representation in a GPRT module.
  GPRTModule module = gprtModuleCreate(context, s08_deviceCode);

  GPRTGeomTypeOf<TrianglesGeomData> trianglesGeomType =
      gprtGeomTypeCreate<TrianglesGeomData>(context, GPRT_TRIANGLES);
  gprtGeomTypeSetClosestHitProg(trianglesGeomType, 0, module, "closesthit");

  // All ray tracing programs start off with a "Ray Generation" kernel.
  // Allocate room for one RayGen shader, create it, and hold on to it with
  // the "gprt" context
  GPRTRayGenOf<RayGenData> rayGen =
      gprtRayGenCreate<RayGenData>(context, module, "raygen");

  
  GPRTMissOf<MissProgData> miss = gprtMissCreate<MissProgData>(context, module, "miss");
  
  // Miss program checkerboard background colors
  MissProgData *missData = gprtMissGetPointer(miss);
  missData->color0 = float3(0.1f, 0.1f, 0.1f);
  missData->color1 = float3(0.0f, 0.0f, 0.0f);

  // ------------------------------------------------------------------
  // allocating buffers
  // ------------------------------------------------------------------

  // Our framebuffer here will be used to hold pixel color values
  // that we'll present to the window / save to an image
  LOG("allocating frame buffer");
  GPRTBufferOf<uint32_t> frameBuffer =
      gprtDeviceBufferCreate<uint32_t>(context, fbSize.x * fbSize.y);

  GPRTTextureOf<stbi_uc> texture = 
      gprtDeviceTextureCreate<stbi_uc>(context, 
        GPRT_IMAGE_TYPE_2D, GPRT_FORMAT_R8G8B8A8_UNORM,
        texWidth, texHeight, 1, true, pixels);

  GPRTSampler sampler = 
    gprtSamplerCreate(context, 
      GPRT_FILTER_LINEAR, GPRT_FILTER_LINEAR, GPRT_FILTER_LINEAR,
      GPRT_SAMPLER_ADDRESS_MODE_WRAP, 
      GPRT_BORDER_COLOR_OPAQUE_BLACK);

  // (re-)builds all vulkan programs, with current pipeline settings
  gprtBuildPipeline(context);
  
  // ------------------------------------------------------------------
  // build the shader binding table, used by rays to map geometry,
  // instances and ray types to GPU kernels
  // ------------------------------------------------------------------
  RayGenData *data = gprtRayGenGetPointer(rayGen);
  data->framebuffer = gprtBufferGetHandle(frameBuffer);
  //data->texture = gprtTextureGetHandle(texture);

  // ------------------------------------------------------------------
  // Meshes
  // ------------------------------------------------------------------
  #ifndef M_PI
  #define M_PI 3.14
  #endif
  Mesh<PlaneMesh> planeMesh(
      context, trianglesGeomType, PlaneMesh{}, texture, sampler, identity);
  std::vector<GPRTGeomOf<TrianglesGeomData>> geoms = {planeMesh.geometry};
  GPRTAccel trianglesBLAS =
      gprtTrianglesAccelCreate(context, geoms.size(), geoms.data());
  GPRTAccel trianglesTLAS = gprtInstanceAccelCreate(context, 1, &trianglesBLAS);
  gprtAccelBuild(context, trianglesBLAS);
  gprtAccelBuild(context, trianglesTLAS);
  
  data->world = gprtAccelGetHandle(trianglesTLAS);

  gprtBuildPipeline(context);

  // Build a shader binding table entry for the ray generation record.
  gprtBuildShaderBindingTable(context, GPRT_SBT_RAYGEN);

  // ##################################################################
  // now that everything is ready: launch it ....
  // ##################################################################
  LOG("executing the launch ...");
  bool firstFrame = true;
  double xpos = 0.f, ypos = 0.f;
  double lastxpos, lastypos;
  do {
    float speed = .001f;
    lastxpos = xpos;
    lastypos = ypos;
    gprtGetCursorPos(context, &xpos, &ypos);
    if (firstFrame) {
      lastxpos = xpos;
      lastypos = ypos;
    }
    int state = gprtGetMouseButton(context, GPRT_MOUSE_BUTTON_LEFT);

    // If we click the mouse, we should rotate the camera
    // Here, we implement some simple camera controls
    if (state == GPRT_PRESS || firstFrame) {
      firstFrame = false;
      float4 position = {lookFrom.x, lookFrom.y, lookFrom.z, 1.f};
      float4 pivot = {lookAt.x, lookAt.y, lookAt.z, 1.0};
#ifndef M_PI
#define M_PI 3.1415926f
#endif

      // step 1 : Calculate the amount of rotation given the mouse movement.
      float deltaAngleX = (2 * M_PI / fbSize.x);
      float deltaAngleY = (M_PI / fbSize.y);
      float xAngle = (lastxpos - xpos) * deltaAngleX;
      float yAngle = (lastypos - ypos) * deltaAngleY;

      // step 2: Rotate the camera around the pivot point on the first axis.
      float4x4 rotationMatrixX = rotation_matrix(rotation_quat(lookUp, xAngle));
      position = (mul(rotationMatrixX, (position - pivot))) + pivot;

      // step 3: Rotate the camera around the pivot point on the second axis.
      float3 lookRight = cross(lookUp, normalize(pivot - position).xyz());
      float4x4 rotationMatrixY =
          rotation_matrix(rotation_quat(lookRight, yAngle));
      lookFrom = ((mul(rotationMatrixY, (position - pivot))) + pivot).xyz();

      // ----------- compute variable values  ------------------
      float3 camera_pos = lookFrom;
      float3 camera_d00 = normalize(lookAt - lookFrom);
      float aspect = float(fbSize.x) / float(fbSize.y);
      float3 camera_ddu =
          cosFovy * aspect * normalize(cross(camera_d00, lookUp));
      float3 camera_ddv = cosFovy * normalize(cross(camera_ddu, camera_d00));
      camera_d00 -= 0.5f * camera_ddu;
      camera_d00 -= 0.5f * camera_ddv;

      // ----------- set variables  ----------------------------
      RayGenData *raygenData = gprtRayGenGetPointer(rayGen);
      raygenData->camera.pos = camera_pos;
      raygenData->camera.dir_00 = camera_d00;
      raygenData->camera.dir_du = camera_ddu;
      raygenData->camera.dir_dv = camera_ddv;

      gprtBuildShaderBindingTable(context, GPRT_SBT_RAYGEN);
    }

    TrianglesGeomData* planeMeshData = gprtGeomGetPointer(planeMesh.geometry);
    planeMeshData->time = gprtGetTime(context);
    
    gprtBuildShaderBindingTable(context, GPRT_SBT_GEOM);

    // Calls the GPU raygen kernel function
    gprtRayGenLaunch2D(context, rayGen, fbSize.x, fbSize.y);

    // If a window exists, presents the framebuffer here to that window
    gprtBufferPresent(context, frameBuffer);
  }
  // returns true if "X" pressed or if in "headless" mode
  while (!gprtWindowShouldClose(context));


  // Save final frame to an image
  LOG("done with launch, writing frame buffer to " << outFileName);
  gprtBufferSaveImage(frameBuffer, fbSize.x, fbSize.y, outFileName);
  LOG_OK("written rendered frame buffer to file " << outFileName);

  // ##################################################################
  // and finally, clean up
  // ##################################################################

  LOG("cleaning up ...");
  planeMesh.cleanup();
  gprtSamplerDestroy(sampler);
  gprtTextureDestroy(texture);
  gprtBufferDestroy(frameBuffer);
  gprtRayGenDestroy(rayGen);
  gprtMissDestroy(miss);
  gprtAccelDestroy(trianglesBLAS);
  gprtAccelDestroy(trianglesTLAS);
  gprtGeomTypeDestroy(trianglesGeomType);
  gprtModuleDestroy(module);
  gprtContextDestroy(context);

  LOG_OK("seems all went OK; app is done, this should be the last output ...");
}
