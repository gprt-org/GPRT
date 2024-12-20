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

#define LOG(message)                                                                                                   \
  std::cout << GPRT_TERMINAL_BLUE;                                                                                     \
  std::cout << "#gprt.sample(main): " << message << std::endl;                                                         \
  std::cout << GPRT_TERMINAL_DEFAULT;
#define LOG_OK(message)                                                                                                \
  std::cout << GPRT_TERMINAL_LIGHT_BLUE;                                                                               \
  std::cout << "#gprt.sample(main): " << message << std::endl;                                                         \
  std::cout << GPRT_TERMINAL_DEFAULT;

extern GPRTProgram s12_deviceCode_A;

// Vertices are the points that define our triangles
const int NUM_TRI_VERTICES = 3;
float3 triVertices[NUM_TRI_VERTICES] = {
    {-1.f, -.5f, 0.f},
    {+1.f, -.5f, 0.f},
    {0.f, +.5f, 0.f},
};

float3 triColors[NUM_TRI_VERTICES] = {
    {1.f, 0.f, 0.f},
    {0.f, 1.f, 0.f},
    {0.f, 0.f, 1.f},
};

// Indices connect those vertices together.
// Here, vertex 0 connects to 1, which connects to 2 to form a triangle.
const int NUM_TRI_INDICES = 1;
uint3 triIndices[NUM_TRI_INDICES] = {{0, 1, 2}};

// These vertices and indices are used to define two triangles
// that will act as a backdrop
const int NUM_BACKDROP_VERTICES = 4;
float3 backdropVertices[NUM_BACKDROP_VERTICES] = {
    {-1.f, -1.f, 0.5f},
    {+1.f, -1.f, 0.5f},
    {-1.f, +1.f, 0.5f},
    {+1.f, +1.f, 0.5f},
};

const int NUM_BACKDROP_INDICES = 2;
uint3 backdropIndices[NUM_BACKDROP_INDICES] = {{0, 1, 2}, {1, 3, 2}};

// These vertices and indices are used to define two triangles
// that will act as our overlaying GUI
const int NUM_GUI_VERTICES = 4;
float3 guiVertices[NUM_GUI_VERTICES] = {
    {-1.f, -1.f, 0.f},
    {+1.f, -1.f, 0.f},
    {-1.f, +1.f, 0.f},
    {+1.f, +1.f, 0.f},
};

const int NUM_GUI_INDICES = 2;
uint3 guiIndices[NUM_GUI_INDICES] = {{0, 1, 2}, {1, 3, 2}};
// initial image resolution
const int2 fbSize = {1400, 460};

// final image output
const char *outFileName = "s12-imgui.png";

// Initial camera parameters
float3 lookFrom = {0.f, 0.f, -4.f};
float3 lookAt = {0.f, 0.f, 0.f};
float3 lookUp = {0.f, -1.f, 0.f};
float cosFovy = 0.66f;

#include <iostream>
int
main(int ac, char **av) {
  LOG("gprt example '" << av[0] << "' starting up");

  LOG("building module, programs, and pipeline");

  // create a context on the first device:
  gprtRequestWindow(fbSize.x, fbSize.y, "S12 ImGui Rasterization");
  GPRTContext context = gprtContextCreate();
  GPRTModule module = gprtModuleCreate(context, s12_deviceCode_A);

  // ##################################################################
  // set up all the GPU kernels we want to run
  // ##################################################################

  GPRTGeomTypeOf<TrianglesGeomData> trianglesGeomType = gprtGeomTypeCreate<TrianglesGeomData>(context, GPRT_TRIANGLES);
  gprtGeomTypeSetVertexProg(trianglesGeomType, 0, module, "simpleVertex");
  gprtGeomTypeSetPixelProg(trianglesGeomType, 0, module, "simplePixel");

  GPRTGeomTypeOf<BackgroundData> backdropGeomType = gprtGeomTypeCreate<BackgroundData>(context, GPRT_TRIANGLES);
  gprtGeomTypeSetVertexProg(backdropGeomType, 0, module, "backgroundVertex");
  gprtGeomTypeSetPixelProg(backdropGeomType, 0, module, "backgroundPixel");

  GPRTGeomTypeOf<GUIData> guiGeomType = gprtGeomTypeCreate<GUIData>(context, GPRT_TRIANGLES);
  gprtGeomTypeSetVertexProg(guiGeomType, 0, module, "GUIVertex");
  gprtGeomTypeSetPixelProg(guiGeomType, 0, module, "GUIPixel");

  // ##################################################################
  // set the parameters for those kernels
  // ##################################################################

  // Setup pixel frame buffer
  GPRTTextureOf<uint32_t> colorAttachment = gprtDeviceTextureCreate<uint32_t>(
      context, GPRT_IMAGE_TYPE_2D, GPRT_FORMAT_R8G8B8A8_SRGB, fbSize.x, fbSize.y, 1, false, nullptr);
  GPRTTextureOf<float> depthAttachment = gprtDeviceTextureCreate<float>(
      context, GPRT_IMAGE_TYPE_2D, GPRT_FORMAT_D32_SFLOAT, fbSize.x, fbSize.y, 1, false, nullptr);
  gprtGeomTypeSetRasterAttachments(backdropGeomType, 0, colorAttachment, depthAttachment);
  gprtGeomTypeSetRasterAttachments(trianglesGeomType, 0, colorAttachment, depthAttachment);
  gprtGeomTypeSetRasterAttachments(guiGeomType, 0, colorAttachment, depthAttachment);

  // This is new, setup GUI frame buffer. We'll rasterize the GUI to this texture, then composite the GUI on top of the
  // rendered scene.
  GPRTTextureOf<uint32_t> guiColorAttachment = gprtDeviceTextureCreate<uint32_t>(
      context, GPRT_IMAGE_TYPE_2D, GPRT_FORMAT_R8G8B8A8_SRGB, fbSize.x, fbSize.y, 1, false, nullptr);
  GPRTTextureOf<float> guiDepthAttachment = gprtDeviceTextureCreate<float>(
      context, GPRT_IMAGE_TYPE_2D, GPRT_FORMAT_D32_SFLOAT, fbSize.x, fbSize.y, 1, false, nullptr);
  gprtGuiSetRasterAttachments(context, guiColorAttachment, guiDepthAttachment);

  LOG("building geometries ...");
  GPRTBufferOf<float3> triVertexBuffer = gprtDeviceBufferCreate<float3>(context, NUM_TRI_VERTICES, triVertices);
  GPRTBufferOf<float3> triColorBuffer = gprtDeviceBufferCreate<float3>(context, NUM_TRI_VERTICES, triColors);
  GPRTBufferOf<uint3> triIndexBuffer = gprtDeviceBufferCreate<uint3>(context, NUM_TRI_INDICES, triIndices);
  GPRTGeomOf<TrianglesGeomData> trianglesGeom = gprtGeomCreate<TrianglesGeomData>(context, trianglesGeomType);
  gprtTrianglesSetVertices(trianglesGeom, triVertexBuffer, NUM_TRI_VERTICES);
  gprtTrianglesSetIndices(trianglesGeom, triIndexBuffer, NUM_TRI_INDICES);
  TrianglesGeomData *tridata = gprtGeomGetParameters(trianglesGeom);
  tridata->color = gprtBufferGetDevicePointer(triColorBuffer);
  tridata->vertex = gprtBufferGetDevicePointer(triVertexBuffer);
  tridata->index = gprtBufferGetDevicePointer(triIndexBuffer);

  GPRTBufferOf<float3> backdropVertexBuffer =
      gprtDeviceBufferCreate<float3>(context, NUM_BACKDROP_VERTICES, backdropVertices);
  GPRTBufferOf<uint3> backdropIndexBuffer =
      gprtDeviceBufferCreate<uint3>(context, NUM_BACKDROP_INDICES, backdropIndices);
  GPRTGeomOf<BackgroundData> bgGeom = gprtGeomCreate<BackgroundData>(context, backdropGeomType);
  gprtTrianglesSetVertices(bgGeom, backdropVertexBuffer, NUM_BACKDROP_VERTICES);
  gprtTrianglesSetIndices(bgGeom, backdropIndexBuffer, NUM_BACKDROP_INDICES);
  BackgroundData *bgdata = gprtGeomGetParameters(bgGeom);
  bgdata->vertex = gprtBufferGetDevicePointer(backdropVertexBuffer);
  bgdata->index = gprtBufferGetDevicePointer(backdropIndexBuffer);
  bgdata->color0 = float3(0.1f, 0.1f, 0.1f);
  bgdata->color1 = float3(0.0f, 0.0f, 0.0f);

  GPRTBufferOf<float3> guiVertexBuffer = gprtDeviceBufferCreate<float3>(context, NUM_GUI_VERTICES, guiVertices);
  GPRTBufferOf<uint3> guiIndexBuffer = gprtDeviceBufferCreate<uint3>(context, NUM_GUI_INDICES, guiIndices);
  GPRTGeomOf<GUIData> guiGeom = gprtGeomCreate<GUIData>(context, guiGeomType);
  gprtTrianglesSetVertices(guiGeom, guiVertexBuffer, NUM_GUI_VERTICES);
  gprtTrianglesSetIndices(guiGeom, guiIndexBuffer, NUM_GUI_INDICES);
  GUIData *guiData = gprtGeomGetParameters(guiGeom);
  guiData->vertex = gprtBufferGetDevicePointer(guiVertexBuffer);
  guiData->index = gprtBufferGetDevicePointer(guiIndexBuffer);
  guiData->texture = gprtTextureGetHandle(guiColorAttachment);
  guiData->resolution = float2((float) fbSize.x, (float) fbSize.y);

  // Upload our newly assigned parameters to the shader binding table.
  gprtBuildShaderBindingTable(context);

  // ##################################################################
  // now that everything is ready: launch it ....
  // ##################################################################

  LOG("launching ...");

  PushConstants pc;

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

      float aspect = float(fbSize.x) / float(fbSize.y);
      pc.view = math::matrixFromLookAt(position.xyz(), pivot.xyz(), lookUp);
      pc.proj = math::perspective(cosFovy, aspect, 0.1f, 1000.f);
    }

    // Draw our background and triangle
    gprtTextureClear(depthAttachment);
    gprtTextureClear(colorAttachment);

    std::vector<GPRTGeomOf<BackgroundData>> drawList1 = {bgGeom};
    gprtGeomTypeRasterize(context, backdropGeomType, drawList1.size(), drawList1.data(), 0, nullptr, pc);
    gprtTextureClear(depthAttachment);

    std::vector<GPRTGeomOf<TrianglesGeomData>> drawList2 = {trianglesGeom};
    gprtGeomTypeRasterize(context, trianglesGeomType, drawList2.size(), drawList2.data(), 0, nullptr, pc);
    gprtTextureClear(depthAttachment);

    // Set our ImGui state
    bool show_demo_window = true;
    if (show_demo_window)
      ImGui::ShowDemoWindow(&show_demo_window);
    ImGui::EndFrame();

    // Rasterize our gui
    gprtTextureClear(guiDepthAttachment);
    gprtTextureClear(guiColorAttachment);
    gprtGuiRasterize(context);

    // Finally, composite the gui onto the screen.
    std::vector<GPRTGeomOf<GUIData>> drawList3 = {guiGeom};
    gprtGeomTypeRasterize(context, guiGeomType, drawList3.size(), drawList3.data(), 0, nullptr, pc);

    // If a window exists, presents the framebuffer here to that window
    gprtTexturePresent(context, colorAttachment);
  }
  // returns true if "X" pressed or if in "headless" mode
  while (!gprtWindowShouldClose(context));

  // Save final frame to an image
  LOG("done with launch, writing frame buffer to " << outFileName);
  gprtTextureSaveImage(colorAttachment, outFileName);
  LOG_OK("written rendered frame buffer to file " << outFileName);

  // ##################################################################
  // and finally, clean up
  // ##################################################################

  LOG("cleaning up ...");

  gprtBufferDestroy(triVertexBuffer);
  gprtBufferDestroy(triColorBuffer);
  gprtBufferDestroy(triIndexBuffer);

  gprtBufferDestroy(backdropVertexBuffer);
  gprtBufferDestroy(backdropIndexBuffer);

  gprtBufferDestroy(guiVertexBuffer);
  gprtBufferDestroy(guiIndexBuffer);

  gprtTextureDestroy(colorAttachment);
  gprtTextureDestroy(depthAttachment);

  gprtTextureDestroy(guiColorAttachment);
  gprtTextureDestroy(guiDepthAttachment);

  gprtGeomDestroy(bgGeom);
  gprtGeomDestroy(trianglesGeom);
  gprtGeomDestroy(guiGeom);
  gprtGeomTypeDestroy(trianglesGeomType);
  gprtGeomTypeDestroy(backdropGeomType);
  gprtGeomTypeDestroy(guiGeomType);
  gprtModuleDestroy(module);
  gprtContextDestroy(context);

  LOG_OK("seems all went OK; app is done, this should be the last output ...");
}
