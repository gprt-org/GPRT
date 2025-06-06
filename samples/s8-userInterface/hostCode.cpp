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

// This program sets up a single geometric object, a mesh for a cube, and
// its acceleration structure, then ray traces it.

// public GPRT API
#include <gprt.h>

// our shared data structures between host and device
#include "sharedCode.h"

// The framework we'll use to create a user interface
#include "imgui.h"

extern GPRTProgram s8_0_deviceCode;

// Vertices are the points that define our triangles
const int NUM_VERTICES = 3;
float3 vertices[NUM_VERTICES] = {
    {-1.f, -.5f, 0.f},
    {+1.f, -.5f, 0.f},
    {0.f, +.5f, 0.f},
};

// Indices connect those vertices together.
// Here, vertex 0 connects to 1, which connects to 2 to form a triangle.
const int NUM_INDICES = 1;
uint3 indices[NUM_INDICES] = {{0, 1, 2}};

// initial image resolution
const uint2 fbSize = {1400, 460};

// final image output
const char *outFileName = "s12-imgui.png";

// Initial camera parameters
float3 lookFrom = {0.f, 0.f, -4.f};
float3 lookAt = {0.f, 0.f, 0.f};
float3 lookUp = {0.f, -1.f, 0.f};
float cosFovy = 0.66f;

int main(int ac, char **av) {
  // create a context on the first device:
  gprtRequestWindow(fbSize.x, fbSize.y, "S12 ImGui Ray Tracing");
  GPRTContext context = gprtContextCreate();
  GPRTModule module = gprtModuleCreate(context, s8_0_deviceCode);

  // ##################################################################
  // set up all the GPU kernels we want to run
  // ##################################################################

  // A kernel for compositing imgui and handling temporal antialiasing
  GPRTComputeOf<CompositeGuiConstants> CompositeGui =
      gprtComputeCreate<CompositeGuiConstants>(context, module, "CompositeGui");

  GPRTGeomTypeOf<TrianglesGeomData> trianglesGeomType = gprtGeomTypeCreate<TrianglesGeomData>(context, GPRT_TRIANGLES);
  gprtGeomTypeSetClosestHitProg(trianglesGeomType, 0, module, "closesthit");

  GPRTRayGenOf<RayGenData> rayGen = gprtRayGenCreate<RayGenData>(context, module, "raygen");

  GPRTMissOf<MissProgData> miss = gprtMissCreate<MissProgData>(context, module, "miss");

  // ##################################################################
  // set the parameters for those kernels
  // ##################################################################

  auto imageBuffer = gprtDeviceBufferCreate<float4>(context, fbSize.x * fbSize.y);

  GPRTTextureParams f32TexParams;
  f32TexParams.type = GPRT_IMAGE_TYPE_2D;
  f32TexParams.format = GPRT_FORMAT_R32G32B32A32_SFLOAT;
  f32TexParams.width = fbSize.x;
  f32TexParams.height = fbSize.y;
  auto imageTexture = gprtDeviceTextureCreate<float4>(context, f32TexParams, nullptr);

  GPRTBufferOf<uint32_t> frameBuffer = gprtDeviceBufferCreate<uint32_t>(context, fbSize.x * fbSize.y);

  // Raygen program frame buffer
  RayGenData *rayGenData = gprtRayGenGetParameters(rayGen);
  rayGenData->imageBuffer = gprtBufferGetDevicePointer(imageBuffer);

  // Miss program checkerboard background colors
  MissProgData *missData = gprtMissGetParameters(miss);
  missData->color0 = float3(0.1f, 0.1f, 0.1f);
  missData->color1 = float3(0.0f, 0.0f, 0.0f);

  // This is new, setup GUI frame buffer. We'll rasterize the GUI to this texture, then composite the GUI on top of the
  // rendered scene.
  GPRTTextureParams srgbTexParams = f32TexParams, d32TexParams = f32TexParams;
  srgbTexParams.format = GPRT_FORMAT_R8G8B8A8_SRGB;
  d32TexParams.format = GPRT_FORMAT_D32_SFLOAT;
  GPRTTextureOf<uint32_t> guiColorAttachment = gprtDeviceTextureCreate<uint32_t>(context, srgbTexParams, nullptr);
  GPRTTextureOf<float> guiDepthAttachment = gprtDeviceTextureCreate<float>(context, d32TexParams, nullptr);
  gprtGuiSetRasterAttachments(context, guiColorAttachment, guiDepthAttachment);


  // The vertex and index buffers here define the triangle vertices
  // and how those vertices are connected together.
  GPRTBufferOf<float3> vertexBuffer = gprtDeviceBufferCreate<float3>(context, NUM_VERTICES, vertices);
  GPRTBufferOf<uint3> indexBuffer = gprtDeviceBufferCreate<uint3>(context, NUM_INDICES, indices);

  // Next, we will create an instantiation of our geometry declaration.
  GPRTGeomOf<TrianglesGeomData> trianglesGeom = gprtGeomCreate<TrianglesGeomData>(context, trianglesGeomType);

  // We use these calls to tell the geometry what buffers store triangle
  // indices and vertices
  gprtTrianglesSetVertices(trianglesGeom, vertexBuffer, NUM_VERTICES);
  gprtTrianglesSetIndices(trianglesGeom, indexBuffer, NUM_INDICES);

  // Placing that geometry into a BLAS.
  GPRTAccel trianglesAccel = gprtTriangleAccelCreate(context, trianglesGeom);
  gprtAccelBuild(context, trianglesAccel);

  // Placing that BLAS into a TLAS.
  gprt::Instance instance = gprtAccelGetInstance(trianglesAccel);
  GPRTBufferOf<gprt::Instance> instancesBuffer = gprtDeviceBufferCreate<gprt::Instance>(context, 1, &instance);
  GPRTAccel world = gprtInstanceAccelCreate(context, 1, instancesBuffer);
  gprtAccelBuild(context, world);

  // Here, we place a reference to our TLAS in the ray generation
  // kernel's parameters
  rayGenData->world = gprtAccelGetDeviceAddress(world);

  // Upload our newly assigned parameters to the shader binding table.
  gprtBuildShaderBindingTable(context);

  // ##################################################################
  // now that everything is ready: launch it ....
  // ##################################################################


  CompositeGuiConstants guiPC;
  guiPC.fbSize = fbSize;
  guiPC.frameBuffer = gprtBufferGetDevicePointer(frameBuffer);
  guiPC.imageBuffer = gprtBufferGetDevicePointer(imageBuffer);
  guiPC.guiTexture = gprtTextureGet2DHandle<float4>(guiColorAttachment);

  RTPushConstants rtPC;

  bool firstFrame = true;
  double xpos = 0.f, ypos = 0.f;
  double lastxpos, lastypos;
  do {
    ImGuiIO &io = ImGui::GetIO();
    ImGui::NewFrame();

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
    if (state == GPRT_PRESS && !io.WantCaptureMouse || firstFrame) {
      firstFrame = false;
      float4 position = {lookFrom.x, lookFrom.y, lookFrom.z, 1.f};
      float4 pivot = {lookAt.x, lookAt.y, lookAt.z, 1.0};
#ifndef M_PI
#define M_PI 3.1415926f
#endif

      // step 1 : Calculate the amount of rotation given the mouse movement.
      float deltaAngleX = (2 * M_PI / fbSize.x);
      float deltaAngleY = (M_PI / fbSize.y);
      float xAngle = float(lastxpos - xpos) * deltaAngleX;
      float yAngle = float(lastypos - ypos) * deltaAngleY;

      // step 2: Rotate the camera around the pivot point on the first axis.
      float4x4 rotationMatrixX = math::matrixFromRotation(xAngle, lookUp);
      position = (mul(rotationMatrixX, (position - pivot))) + pivot;

      // step 3: Rotate the camera around the pivot point on the second axis.
      float3 lookRight = cross(lookUp, normalize(pivot - position).xyz());
      float4x4 rotationMatrixY = math::matrixFromRotation(yAngle, lookRight);
      lookFrom = ((mul(rotationMatrixY, (position - pivot))) + pivot).xyz();

      // ----------- compute variable values  ------------------
      float3 camera_pos = lookFrom;
      float3 camera_d00 = normalize(lookAt - lookFrom);
      float aspect = float(fbSize.x) / float(fbSize.y);
      float3 camera_ddu = cosFovy * aspect * normalize(cross(camera_d00, lookUp));
      float3 camera_ddv = cosFovy * normalize(cross(camera_ddu, camera_d00));
      camera_d00 -= 0.5f * camera_ddu;
      camera_d00 -= 0.5f * camera_ddv;

      // ----------- set variables  ----------------------------
      rtPC.camera.pos = camera_pos;
      rtPC.camera.dir_00 = camera_d00;
      rtPC.camera.dir_du = camera_ddu;
      rtPC.camera.dir_dv = camera_ddv;
    }

    // Call the GPU raygen kernel function
    gprtRayGenLaunch2D(context, rayGen, fbSize.x, fbSize.y, rtPC);

    // Set our ImGui state
    bool show_demo_window = true;
    if (show_demo_window)
      ImGui::ShowDemoWindow(&show_demo_window);
    ImGui::EndFrame();

    // Rasterize our gui
    gprtTextureClear(guiDepthAttachment);
    gprtTextureClear(guiColorAttachment);
    gprtGuiRasterize(context);

    // Finally, composite the gui onto the screen using a compute shader.
    gprtBufferTextureCopy(context, imageBuffer, imageTexture, 0, 0, 0, 0, 0, 0, fbSize.x, fbSize.y, 1);

    gprtComputeLaunch(CompositeGui, {fbSize.x, fbSize.y, 1}, {1, 1, 1}, guiPC);

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


  gprtTextureDestroy(guiColorAttachment);
  gprtTextureDestroy(guiDepthAttachment);

  gprtGeomDestroy(trianglesGeom);
  gprtGeomTypeDestroy(trianglesGeomType);
  gprtModuleDestroy(module);
  gprtContextDestroy(context);

  return 0;
}
