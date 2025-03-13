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

extern GPRTProgram s7_0_deviceCode;

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
const int2 fbSize = {1920, 1080};

// final image output
const char *outFileName = "s07-antialiasing.hdr";

float3 lookFrom = {0.f, 0.0f, 10.f};
float3 lookAt = {0.f, 0.f, 0.f};
float3 lookUp = {0.f, 1.f, 0.f};
float cosFovy = 0.4f;

float2 GetCurrentPixelOffset(uint32_t frameIndex)
{
    // Halton jitter
    float2 Result(0.0f, 0.0f);

    constexpr int BaseX = 2;
    int Index = frameIndex + 1;
    float InvBase = 1.0f / BaseX;
    float Fraction = InvBase;
    while (Index > 0)
    {
        Result.x += (Index % BaseX) * Fraction;
        Index /= BaseX;
        Fraction *= InvBase;
    }

    constexpr int BaseY = 3;
    Index = frameIndex + 1;
    InvBase = 1.0f / BaseY;
    Fraction = InvBase;
    while (Index > 0)
    {
        Result.y += (Index % BaseY) * Fraction;
        Index /= BaseY;
        Fraction *= InvBase;
    }

    Result.x -= 0.5f;
    Result.y -= 0.5f;
    return Result;
}

int main(int ac, char **av) {
  // The output window will show comments for many of the methods called.
  // Walking through the code line by line with a debugger is educational.

  // ##################################################################
  // set up all the GPU kernels we want to run
  // ##################################################################


  // Load the texture we'll display
  int texWidth, texHeight, texChannels;
  stbi_uc *pixels = stbi_load(ASSETS_DIRECTORY "checkerboard.png", &texWidth, &texHeight, &texChannels, STBI_rgb_alpha);

  gprtRequestDenoiser(fbSize.x, fbSize.y, GPRTDenoiseFlags(GPRT_DENOISE_FLAGS_MVLOWRES | GPRT_DENOISE_FLAGS_DO_SHARPENING | GPRT_DENOISE_FLAGS_AUTO_EXPOSURE | GPRT_DENOISE_FLAGS_DEPTH_INVERTED));

  gprtRequestWindow(fbSize.x, fbSize.y, "S07 Antialiasing");
  GPRTContext context = gprtContextCreate(nullptr, 1);
  GPRTModule module = gprtModuleCreate(context, s7_0_deviceCode);
  PushConstants pc;
  pc.now = 0.f;

  uint2 renderFBSize = fbSize;
  gprtGetDenoiserRenderSize(context, &renderFBSize.x, &renderFBSize.y);

  // ##################################################################
  // set up all the GPU kernels we want to run
  // ##################################################################

  GPRTGeomTypeOf<TrianglesGeomData> trianglesGeomType = gprtGeomTypeCreate<TrianglesGeomData>(context, GPRT_TRIANGLES);
  gprtGeomTypeSetClosestHitProg(trianglesGeomType, 0, module, "closesthit");

  auto transformProgram = gprtComputeCreate<PushConstants>(context, module, "Transform");
  auto renderpassProgram = gprtComputeCreate<RenderPassParams>(context, module, "RenderPass");

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
  planeData->texture = gprtTextureGet2DHandle(texture);
  for (uint32_t i = 0; i < samplers.size(); ++i) {
    planeData->samplers[i] = gprtSamplerGetHandle(samplers[i]);
  }

  GPRTAccel trianglesBLAS = gprtTriangleAccelCreate(context, plane);
  gprtAccelBuild(context, trianglesBLAS, GPRT_BUILD_MODE_FAST_TRACE_NO_UPDATE);

  std::vector<gprt::Instance> instances(samplers.size(), gprtAccelGetInstance(trianglesBLAS));

  GPRTBufferOf<gprt::Instance> instancesBuffer =
      gprtDeviceBufferCreate<gprt::Instance>(context, instances.size(), instances.data());
  GPRTBufferOf<gprt::Instance> prevInstancesBuffer =
      gprtDeviceBufferCreate<gprt::Instance>(context, instances.size(), instances.data());
  GPRTAccel trianglesTLAS = gprtInstanceAccelCreate(context, instances.size(), instancesBuffer);

  pc.prevInstances = gprtBufferGetDevicePointer(prevInstancesBuffer);
  pc.instances = gprtBufferGetDevicePointer(instancesBuffer);
  pc.numInstances = (uint32_t) instances.size();
  gprtComputeLaunch(transformProgram, {samplers.size(), 1, 1}, {1, 1, 1}, pc);

  // gprtInstanceAccelSet4x4Transforms(trianglesTLAS, transformBuffer);

  gprtAccelBuild(context, trianglesTLAS, GPRT_BUILD_MODE_FAST_TRACE_AND_UPDATE);

  // ------------------------------------------------------------------
  // Setup the ray generation and miss programs
  // ------------------------------------------------------------------
  GPRTBufferOf<float4> tmpRenderBuffer = gprtDeviceBufferCreate<float4>(context, renderFBSize.x *  renderFBSize.y);
  GPRTBufferOf<float> tmpDepthBuffer = gprtDeviceBufferCreate<float>(context, renderFBSize.x *  renderFBSize.y);
  GPRTBufferOf<float2> tmpMVecBuffer = gprtDeviceBufferCreate<float2>(context, renderFBSize.x *  renderFBSize.y);
  
  GPRTTextureOf<float4> renderBuffer = gprtDeviceTextureCreate<float4>(context, GPRT_IMAGE_TYPE_2D, GPRT_FORMAT_R32G32B32A32_SFLOAT, renderFBSize.x, renderFBSize.y, 1, false);
  GPRTTextureOf<float> depthBuffer = gprtDeviceTextureCreate<float>(context, GPRT_IMAGE_TYPE_2D, GPRT_FORMAT_R32_SFLOAT, renderFBSize.x, renderFBSize.y, 1, false);
  GPRTTextureOf<float2> mvecBuffer = gprtDeviceTextureCreate<float2>(context, GPRT_IMAGE_TYPE_2D, GPRT_FORMAT_R32G32_SFLOAT, renderFBSize.x, renderFBSize.y, 1, false);  
  GPRTTextureOf<float4> resolvedBuffer = gprtDeviceTextureCreate<float4>(context, GPRT_IMAGE_TYPE_2D, GPRT_FORMAT_R32G32B32A32_SFLOAT, fbSize.x, fbSize.y, 1, false);
  GPRTBufferOf<uint32_t> frameBuffer = gprtDeviceBufferCreate<uint32_t>(context, fbSize.x * fbSize.y);

  RayGenData *raygenData = gprtRayGenGetParameters(rayGen);
  // raygenData->renderBuffer = gprtTextureGet2DHandle(renderBuffer);
  raygenData->renderBuffer = gprtBufferGetDevicePointer(tmpRenderBuffer);
  raygenData->depthBuffer = gprtBufferGetDevicePointer(tmpDepthBuffer);
  raygenData->mvecBuffer = gprtBufferGetDevicePointer(tmpMVecBuffer);
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
  pc.frame = 0;
  do {
    float2 jitter = GetCurrentPixelOffset(pc.frame);
    raygenData->jitter = jitter;


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
      float deltaAngleX = (2 * M_PI / renderFBSize.x);
      float deltaAngleY = (M_PI / renderFBSize.y);
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
      if (!firstFrame) {
        raygenData->prevCamera = raygenData->currCamera;
      }

      float aspect = float(renderFBSize.x) / float(renderFBSize.y);
      float fovY = 2.0f * acos(cosFovy);

      // Compute half extents of the image plane at unit distance.
      float halfHeight = tan(0.5f * fovY);
      float halfWidth  = aspect * halfHeight;

      // Compute the world-space offset for one pixel.
      float pixelStepX = (2.0f * halfWidth)  / float(renderFBSize.x);
      float pixelStepY = (2.0f * halfHeight) / float(renderFBSize.y);

      // Compute camera basis vectors.
      float3 forward = normalize(lookAt - lookFrom);
      float3 right   = normalize(cross(forward, lookUp)); // right direction
      float3 up      = normalize(cross(right, forward));  // recomputed up

      raygenData->currCamera.pos = lookFrom;
      raygenData->currCamera.dir_00 = normalize(lookAt - lookFrom);      

      raygenData->currCamera.dir_du = cosFovy * aspect * normalize(cross(raygenData->currCamera.dir_00, lookUp));
      raygenData->currCamera.dir_dv = cosFovy * normalize(cross(raygenData->currCamera.dir_du, raygenData->currCamera.dir_00));
      
      // Compute the ray direction for the lower left pixel of the image plane.
      // raygenData->currCamera.dir_00 = forward - halfWidth * right - halfHeight * up;

      raygenData->currCamera.dir_00 -= 0.5f * raygenData->currCamera.dir_du;
      raygenData->currCamera.dir_00 -= 0.5f * raygenData->currCamera.dir_dv;
      raygenData->currCamera.fovy = cosFovy;

      if (firstFrame)
      {
        raygenData->prevCamera = raygenData->currCamera; 
      }
    }

    gprtBuildShaderBindingTable(context, GPRT_SBT_RAYGEN);

    // Animate transforms
    pc.now = .5f * (float) gprtGetTime(context);
    
    gprtComputeLaunch(transformProgram, {instances.size(), 1, 1}, {1, 1, 1}, pc);
    gprtAccelUpdate(context, trianglesTLAS);

    // Calls the GPU raygen kernel function
    gprtRayGenLaunch2D(context, rayGen, renderFBSize.x, renderFBSize.y, pc);

    gprtBufferTextureCopy(context, tmpRenderBuffer, renderBuffer, 0, renderFBSize.x, renderFBSize.y, 0, 0, 0, renderFBSize.x, renderFBSize.y, 1);
    gprtBufferTextureCopy(context, tmpDepthBuffer, depthBuffer, 0, renderFBSize.x, renderFBSize.y, 0, 0, 0, renderFBSize.x, renderFBSize.y, 1);
    gprtBufferTextureCopy(context, tmpMVecBuffer, mvecBuffer, 0, renderFBSize.x, renderFBSize.y, 0, 0, 0, renderFBSize.x, renderFBSize.y, 1);

    // gprtTextureSaveImage(renderBuffer, "renderBuffer.hdr");

    GPRTDenoiseParams denoiseParams = {};
    denoiseParams.unresolvedColor = renderBuffer;
    denoiseParams.resolvedColor = resolvedBuffer;
    denoiseParams.depthBuffer = depthBuffer;
    denoiseParams.diffuseMotionVectors = mvecBuffer;
    denoiseParams.jitter = jitter;
    gprtTextureDenoise(context, denoiseParams);


    // gprtTextureSaveImage(resolvedBuffer, outFileName);


    RenderPassParams rppc;
    rppc.resolvedColor = gprtTextureGet2DHandle(resolvedBuffer);
    // rppc.resolvedColor = gprtTextureGet2DHandle(depthBuffer);
    rppc.frameBuffer = gprtBufferGetDevicePointer(frameBuffer);
    rppc.fbSize = fbSize;
    gprtComputeLaunch(renderpassProgram, {fbSize.x, fbSize.y, 1}, {1, 1, 1}, rppc);

    // If a window exists, presents the framebuffer here to that window
    gprtBufferPresent(context, frameBuffer);

    pc.frame = (pc.frame + 1) % 4000;
  }
  // returns true if "X" pressed or if in "headless" mode
  while (!gprtWindowShouldClose(context));

  gprtTextureSaveImage(resolvedBuffer, outFileName);

  // Save final frame to an image
  // gprtBufferSaveImage(frameBuffer, fbSize.x, fbSize.y, outFileName);

  // ##################################################################
  // and finally, clean up
  // ##################################################################

  gprtContextDestroy(context);

  return 0;
}
