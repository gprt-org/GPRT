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

// library for image output
#define STB_IMAGE_STATIC
#define STB_IMAGE_IMPLEMENTATION
#include "stb/stb_image.h"

extern GPRTProgram s6_0_deviceCode;

// Vertices are the points that define our texture plane
const int NUM_VERTICES = 4;
float3 vertices[NUM_VERTICES] = {
    {-0.9f, -0.9f, 0.f},
    {+0.9f, -0.9f, 0.f},
    {-0.9f, +0.9f, 0.f},
    {+0.9f, +0.9f, 0.f},
};

// texture coordinates map vertices to texture locations
float2 texcoords[NUM_VERTICES] = {
    {0.f, 0.f},
    {1.f, 0.f},
    {0.f, 1.f},
    {1.f, 1.f},
};

// Indices connect those vertices together.
const int NUM_INDICES = 2;
uint3 indices[NUM_INDICES] = {{0, 1, 2}, {1, 3, 2}};

// initial image resolution
const int2 fbSize = {1400, 460};

// final image output
const char *outFileName = "s09-textures.png";

float3 lookFrom = {0.f, 0.0f, 10.f};
float3 lookAt = {0.f, 0.f, 0.f};
float3 lookUp = {0.f, 1.f, 0.f};
float cosFovy = 0.4f;

int main(int ac, char **av) {
  // The output window will show comments for many of the methods called.
  // Walking through the code line by line with a debugger is educational.

  // ##################################################################
  // set up all the GPU kernels we want to run
  // ##################################################################


  // Load the texture we'll display
  int texWidth, texHeight, texChannels;
  stbi_uc *pixels = stbi_load(ASSETS_DIRECTORY "checkerboard.png", &texWidth, &texHeight, &texChannels, STBI_rgb_alpha);

  gprtRequestWindow(fbSize.x, fbSize.y, "S09 Single Texture");
  GPRTContext context = gprtContextCreate(nullptr, 1);
  GPRTModule module = gprtModuleCreate(context, s6_0_deviceCode);
  PushConstants pc;
  pc.now = 0.f;

  // ##################################################################
  // set up all the GPU kernels we want to run
  // ##################################################################

  GPRTGeomTypeOf<TrianglesGeomData> trianglesGeomType = gprtGeomTypeCreate<TrianglesGeomData>(context, GPRT_TRIANGLES);
  gprtGeomTypeSetClosestHitProg(trianglesGeomType, 0, module, "closesthit");

  auto transformProgram = gprtComputeCreate<PushConstants>(context, module, "Transform");

  GPRTRayGenOf<RayGenData> rayGen = gprtRayGenCreate<RayGenData>(context, module, "raygen");

  GPRTMissOf<MissProgData> miss = gprtMissCreate<MissProgData>(context, module, "miss");

  // ------------------------------------------------------------------
  // Create our texture and sampler
  // ------------------------------------------------------------------

  GPRTTextureOf<stbi_uc> texture = gprtDeviceTextureCreate<stbi_uc>(
      context, GPRT_IMAGE_TYPE_2D, GPRT_FORMAT_R8G8B8A8_UNORM, texWidth, texHeight, /*depth*/ 1,
      /* generate mipmaps */ true, pixels);

  std::vector<GPRTSampler> samplers = {
      // First texture will use the default sampler
      gprtSamplerCreate(context),
      // Next texture we'll show off mipmapping, so we'll use the
      // default sampler here too
      gprtSamplerCreate(context),

      // Then here, we'll demonstrate linear vs nearest for the magfilter
      gprtSamplerCreate(context, GPRT_FILTER_NEAREST), gprtSamplerCreate(context, GPRT_FILTER_LINEAR),

      // Here, we'll demonstrate linear vs nearest for the minFilter
      gprtSamplerCreate(context, GPRT_FILTER_LINEAR, GPRT_FILTER_NEAREST),
      gprtSamplerCreate(context, GPRT_FILTER_LINEAR, GPRT_FILTER_LINEAR),

      // Anisotropy of 1 vs 16. Should see less blurring with higher
      // anisotropy values
      gprtSamplerCreate(context, GPRT_FILTER_LINEAR, GPRT_FILTER_LINEAR, GPRT_FILTER_LINEAR, 1),
      gprtSamplerCreate(context, GPRT_FILTER_LINEAR, GPRT_FILTER_LINEAR, GPRT_FILTER_LINEAR, 16),

      // Changing wrap mode
      gprtSamplerCreate(context, GPRT_FILTER_LINEAR, GPRT_FILTER_LINEAR, GPRT_FILTER_LINEAR, 1,
                        GPRT_SAMPLER_ADDRESS_MODE_REPEAT),
      gprtSamplerCreate(context, GPRT_FILTER_LINEAR, GPRT_FILTER_LINEAR, GPRT_FILTER_LINEAR, 1,
                        GPRT_SAMPLER_ADDRESS_MODE_CLAMP),

      // Changing border mode
      gprtSamplerCreate(context, GPRT_FILTER_LINEAR, GPRT_FILTER_LINEAR, GPRT_FILTER_LINEAR, 1,
                        GPRT_SAMPLER_ADDRESS_MODE_BORDER, GPRT_BORDER_COLOR_OPAQUE_BLACK),
      gprtSamplerCreate(context, GPRT_FILTER_LINEAR, GPRT_FILTER_LINEAR, GPRT_FILTER_LINEAR, 1,
                        GPRT_SAMPLER_ADDRESS_MODE_BORDER, GPRT_BORDER_COLOR_OPAQUE_WHITE)};

  // ------------------------------------------------------------------
  // Meshes
  // ------------------------------------------------------------------
  GPRTBufferOf<float3> vertexBuffer = gprtDeviceBufferCreate<float3>(context, NUM_VERTICES, vertices);
  GPRTBufferOf<float2> texcoordBuffer = gprtDeviceBufferCreate<float2>(context, NUM_VERTICES, texcoords);
  GPRTBufferOf<uint3> indexBuffer = gprtDeviceBufferCreate<uint3>(context, NUM_INDICES, indices);

  GPRTGeomOf<TrianglesGeomData> plane = gprtGeomCreate(context, trianglesGeomType);

  gprtTrianglesSetVertices(plane, vertexBuffer, NUM_VERTICES);
  gprtTrianglesSetIndices(plane, indexBuffer, NUM_INDICES);

  TrianglesGeomData *planeData = gprtGeomGetParameters(plane);
  planeData->index = gprtBufferGetDevicePointer(indexBuffer);
  planeData->vertex = gprtBufferGetDevicePointer(vertexBuffer);
  planeData->texcoord = gprtBufferGetDevicePointer(texcoordBuffer);
  planeData->texture = gprtTextureGetHandle(texture);
  for (uint32_t i = 0; i < samplers.size(); ++i) {
    planeData->samplers[i] = gprtSamplerGetHandle(samplers[i]);
  }

  GPRTAccel trianglesBLAS = gprtTriangleAccelCreate(context, plane);
  gprtAccelBuild(context, trianglesBLAS, GPRT_BUILD_MODE_FAST_TRACE_NO_UPDATE);

  std::vector<gprt::Instance> instances(samplers.size(), gprtAccelGetInstance(trianglesBLAS));

  GPRTBufferOf<gprt::Instance> instancesBuffer =
      gprtDeviceBufferCreate<gprt::Instance>(context, instances.size(), instances.data());
  GPRTAccel trianglesTLAS = gprtInstanceAccelCreate(context, instances.size(), instancesBuffer);

  pc.instances = gprtBufferGetDevicePointer(instancesBuffer);
  pc.numInstances = (uint32_t) instances.size();
  gprtComputeLaunch(transformProgram, {samplers.size(), 1, 1}, {1, 1, 1}, pc);

  // gprtInstanceAccelSet4x4Transforms(trianglesTLAS, transformBuffer);

  gprtAccelBuild(context, trianglesTLAS, GPRT_BUILD_MODE_FAST_TRACE_AND_UPDATE);

  // ------------------------------------------------------------------
  // Setup the ray generation and miss programs
  // ------------------------------------------------------------------
  GPRTBufferOf<uint32_t> frameBuffer = gprtDeviceBufferCreate<uint32_t>(context, fbSize.x * fbSize.y);

  RayGenData *raygenData = gprtRayGenGetParameters(rayGen);
  raygenData->frameBuffer = gprtBufferGetDevicePointer(frameBuffer);
  raygenData->world = gprtAccelGetDeviceAddress(trianglesTLAS);

  // Miss program checkerboard background colors
  MissProgData *missData = gprtMissGetParameters(miss);
  missData->color0 = float3(0.1f, 0.1f, 0.1f);
  missData->color1 = float3(0.0f, 0.0f, 0.0f);

  // Upload our newly assigned parameters to the shader binding table.
  gprtBuildShaderBindingTable(context);

  // ##################################################################
  // now that everything is ready: launch it ....
  // ##################################################################
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
      float4x4 rotationMatrixX = math::matrixFromRotation(xAngle, lookUp);
      position = (mul(rotationMatrixX, (position - pivot))) + pivot;

      // step 3: Rotate the camera around the pivot point on the second axis.
      float3 lookRight = cross(lookUp, normalize(pivot - position).xyz());
      float4x4 rotationMatrixY = math::matrixFromRotation(yAngle, lookRight);
      lookFrom = ((mul(rotationMatrixY, (position - pivot))) + pivot).xyz();

      // ----------- compute variable values  ------------------
      pc.camera.pos = lookFrom;
      pc.camera.dir_00 = normalize(lookAt - lookFrom);
      float aspect = float(fbSize.x) / float(fbSize.y);
      pc.camera.dir_du = cosFovy * aspect * normalize(cross(pc.camera.dir_00, lookUp));
      pc.camera.dir_dv = cosFovy * normalize(cross(pc.camera.dir_du, pc.camera.dir_00));
      pc.camera.dir_00 -= 0.5f * pc.camera.dir_du;
      pc.camera.dir_00 -= 0.5f * pc.camera.dir_dv;
      pc.camera.fovy = cosFovy;
    }

    // Animate transforms
    pc.now = .5f * (float) gprtGetTime(context);
    gprtComputeLaunch(transformProgram, {instances.size(), 1, 1}, {1, 1, 1}, pc);
    gprtAccelUpdate(context, trianglesTLAS);

    // Calls the GPU raygen kernel function
    gprtRayGenLaunch2D(context, rayGen, fbSize.x, fbSize.y, pc);

    // If a window exists, presents the framebuffer here to that window
    gprtBufferPresent(context, frameBuffer);
  }
  // returns true if "X" pressed or if in "headless" mode
  while (!gprtWindowShouldClose(context));

  // Save final frame to an image
  gprtBufferSaveImage(frameBuffer, fbSize.x, fbSize.y, outFileName);

  // ##################################################################
  // and finally, clean up
  // ##################################################################

  gprtContextDestroy(context);

  return 0;
}
