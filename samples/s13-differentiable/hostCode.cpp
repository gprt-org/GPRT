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

// for generating meshes
#include <generator.hpp>
using namespace generator;

// The framework we'll use to create a user interface
#include "imgui.h"

#include <random>


#define LOG(message)                                                                                                   \
  std::cout << GPRT_TERMINAL_BLUE;                                                                                     \
  std::cout << "#gprt.sample(main): " << message << std::endl;                                                         \
  std::cout << GPRT_TERMINAL_DEFAULT;
#define LOG_OK(message)                                                                                                \
  std::cout << GPRT_TERMINAL_LIGHT_BLUE;                                                                               \
  std::cout << "#gprt.sample(main): " << message << std::endl;                                                         \
  std::cout << GPRT_TERMINAL_DEFAULT;

extern GPRTProgram s13_deviceCode;

// initial image resolution
const uint2 fbSize = {1400, 460};

// final image output
const char *outFileName = "s12-imgui.png";

// Initial camera parameters
float3 lookFrom = {0.f, -6.f, 0.0f};
float3 lookAt = {0.f, 0.f, 0.0f};
float3 lookUp = {0.f, 0.f, -1.f};
float cosFovy = 0.66f;

// Optimizer parameters
bool useVectorAdam = true;
float adamBeta1 = 0.9f;
float adamBeta2 = 0.999f;
float learningRate = 2.f;

// A class we'll use to quickly generate meshes and bottom level trees
template <typename T> struct Mesh {
  std::vector<float3> vertices;
  std::vector<uint3> indices;
  GPRTBufferOf<float3> vertexBuffer;
  GPRTBufferOf<uint3> indexBuffer;
  GPRTGeomOf<TrianglesGeomData> geometry;
  GPRTAccel accel;

  Mesh(){};
  Mesh(GPRTContext context, GPRTGeomTypeOf<TrianglesGeomData> geomType, T generator) {
    // Use the generator to generate vertices and indices
    auto vertGenerator = generator.vertices();
    auto triGenerator = generator.triangles();
    while (!vertGenerator.done()) {
      auto vertex = vertGenerator.generate();
      auto position = vertex.position;
      vertices.push_back(float3(position[0], position[1], position[2]));
      vertGenerator.next();
    }
    while (!triGenerator.done()) {
      Triangle triangle = triGenerator.generate();
      auto vertices = triangle.vertices;
      indices.push_back(uint3(vertices[0], vertices[1], vertices[2]));
      triGenerator.next();
    }

    // Upload those to the device, create the geometry
    vertexBuffer = gprtDeviceBufferCreate<float3>(context, vertices.size(), vertices.data());
    indexBuffer = gprtDeviceBufferCreate<uint3>(context, indices.size(), indices.data());
    geometry = gprtGeomCreate(context, geomType);

    gprtTrianglesSetVertices(geometry, vertexBuffer, vertices.size());
    gprtTrianglesSetIndices(geometry, indexBuffer, indices.size());
    TrianglesGeomData *geomData = gprtGeomGetParameters(geometry);
    geomData->vertex = gprtBufferGetHandle(vertexBuffer);
    geomData->index = gprtBufferGetHandle(indexBuffer);

    // Build the bottom level acceleration structure
    accel = gprtTriangleAccelCreate(context, 1, &geometry);
    gprtAccelBuild(context, accel, GPRT_BUILD_MODE_FAST_TRACE_NO_UPDATE, /*allow compaction*/true);
    // gprtAccelCompact(context, accel);
  };

  void cleanupMesh() {
    gprtAccelDestroy(accel);
    gprtGeomDestroy(geometry);
    gprtBufferDestroy(vertexBuffer);
    gprtBufferDestroy(indexBuffer);
  };
};

uint32_t divUp(int x, int y) {
  return (x + (y - 1)) / y;
}

// Initialize adam optimizer
float3 m1 = float3(0.f, 0.f, 0.f);
float3 m2 = float3(0.f, 0.f, 0.f);
int step = 1;

void reset() {
  step = 1;
  m1 = float3(0.f, 0.f, 0.f);
  m2 = float3(0.f, 0.f, 0.f);
}

#include <iostream>
int
main(int ac, char **av) {
  srand(time(0));

  LOG("gprt example '" << av[0] << "' starting up");

  LOG("building module, programs, and pipeline");

  // create a context on the first device:
  gprtRequestWindow(fbSize.x, fbSize.y, "S13 Differentiable");
  GPRTContext context = gprtContextCreate();
  GPRTModule module = gprtModuleCreate(context, s13_deviceCode);

  // ##################################################################
  // set up all the GPU kernels we want to run
  // ##################################################################

  // A kernel for compositing imgui and handling temporal antialiasing 
  auto CompositeGui = gprtComputeCreate<CompositeGuiConstants>(context, module, "CompositeGui");
  
  // Differentiable kernel for computing a tightly fitting oriented bounding box
  auto ClearOBB = gprtComputeCreate<ComputeOBBConstants>(context, module, "ClearOBB");
  auto ComputeOBB = gprtComputeCreate<ComputeOBBConstants >(context, module, "ComputeOBB");
  auto BackPropOBB = gprtComputeCreate<ComputeOBBConstants >(context, module, "BackPropOBB");

  GPRTGeomTypeOf<TrianglesGeomData> trianglesGeomType = gprtGeomTypeCreate<TrianglesGeomData>(context, GPRT_TRIANGLES);
  gprtGeomTypeSetClosestHitProg(trianglesGeomType, 0, module, "hitTriangle");

  GPRTGeomTypeOf<BoundingBoxData> aabbType = gprtGeomTypeCreate<BoundingBoxData>(context, GPRT_AABBS);
  gprtGeomTypeSetIntersectionProg(aabbType, 0, module, "intersectBoundingBox");
  gprtGeomTypeSetClosestHitProg(aabbType, 0, module, "hitBoundingBox");
  
  GPRTRayGenOf<RayGenData> rayGen = gprtRayGenCreate<RayGenData>(context, module, "raygen");
  
  GPRTMissOf<MissProgData> triMiss = gprtMissCreate<MissProgData>(context, module, "triMiss");
  GPRTMissOf<MissProgData> obbMiss = gprtMissCreate<MissProgData>(context, module, "obbMiss");

  // ##################################################################
  // set the parameters for those kernels
  // ##################################################################

  auto imageBuffer =
      gprtDeviceBufferCreate<float4>(context, fbSize.x * fbSize.y);

  auto imageTexture = gprtDeviceTextureCreate<float4>(
      context, GPRT_IMAGE_TYPE_2D, GPRT_FORMAT_R32G32B32A32_SFLOAT, fbSize.x,
      fbSize.y, 1, false, nullptr);

  GPRTBufferOf<uint32_t> frameBuffer = gprtDeviceBufferCreate<uint32_t>(context, fbSize.x * fbSize.y);

  // Raygen program frame buffer
  RayGenData *rayGenData = gprtRayGenGetParameters(rayGen);
  rayGenData->imageBuffer = gprtBufferGetHandle(imageBuffer);

  // Miss program checkerboard background colors
  MissProgData *missData = gprtMissGetParameters(triMiss);
  missData->color0 = float3(0.1f, 0.1f, 0.1f);
  missData->color1 = float3(0.0f, 0.0f, 0.0f);

  // This is new, setup GUI frame buffer. We'll rasterize the GUI to this texture, then composite the GUI on top of the
  // rendered scene.
  GPRTTextureOf<uint32_t> guiColorAttachment = gprtDeviceTextureCreate<uint32_t>(
      context, GPRT_IMAGE_TYPE_2D, GPRT_FORMAT_R8G8B8A8_SRGB, fbSize.x, fbSize.y, 1, false, nullptr);
  GPRTTextureOf<float> guiDepthAttachment = gprtDeviceTextureCreate<float>(
      context, GPRT_IMAGE_TYPE_2D, GPRT_FORMAT_D32_SFLOAT, fbSize.x, fbSize.y, 1, false, nullptr);
  gprtGuiSetRasterAttachments(context, guiColorAttachment, guiDepthAttachment);

  // We begin by making one teapot mesh, storing that mesh in a bottom
  // level acceleration structure.
  Mesh<TorusKnotMesh> mesh(context, trianglesGeomType, TorusKnotMesh{2, 3, 16, 288});

  // Buffers to hold triangle and OBB data
  float3 eul = float3(1.f, 2.f, 3.f);
  float3 deul = float3(0.f, 0.f, 0.f);  
  float3 tmp = float3(0.f, 0.f, 0.f);  // just using these to store intermediate results
  std::vector<float3> euls = {eul, deul, tmp};
  GPRTBufferOf<float3> eulRots = gprtDeviceBufferCreate<float3>(context, 3, euls.data());
  GPRTBufferOf<float3> aabbPositions = gprtDeviceBufferCreate<float3>(context, 2, nullptr);
  GPRTBufferOf<float3x4> transformBuffer = gprtHostBufferCreate<float3x4>(context, 1, nullptr);

  ComputeOBBConstants obbPC;
  obbPC.aabbs = gprtBufferGetHandle(aabbPositions);
  obbPC.eulRots = gprtBufferGetHandle(eulRots);
  obbPC.vertices = gprtBufferGetHandle(mesh.vertexBuffer);
  obbPC.indices = gprtBufferGetHandle(mesh.indexBuffer);
  obbPC.transforms = gprtBufferGetHandle(transformBuffer);
  obbPC.numIndices = mesh.indices.size();
  obbPC.numTrisToInclude = mesh.indices.size();

  gprtBuildShaderBindingTable(context, GPRT_SBT_ALL);
  gprtComputeLaunch(ClearOBB, {1,1,1}, {1,1,1}, obbPC);
  gprtComputeLaunch(ComputeOBB, {divUp(obbPC.numTrisToInclude, 128), 1, 1}, {128,1,1}, obbPC);
  gprtComputeLaunch(BackPropOBB, {divUp(obbPC.numTrisToInclude, 128), 1, 1}, {128,1,1}, obbPC);
  
  LOG("building geometries ...");

  // // Create triangle geometry 
  // GPRTGeomOf<TrianglesGeomData> trianglesGeom = gprtGeomCreate<TrianglesGeomData>(context, trianglesGeomType);
  // gprtTrianglesSetVertices(trianglesGeom, vertexBuffer, NUM_VERTICES);
  // gprtTrianglesSetIndices(trianglesGeom, indexBuffer, NUM_INDICES);
  
  // Place that triangle geometry into a triangles BLAS.
  // GPRTAccel trianglesAccel = gprtTrianglesAccelCreate(context, 1, &mesh.accel);
  // gprtAccelBuild(context, trianglesAccel, GPRT_BUILD_MODE_FAST_TRACE_NO_UPDATE);

  // Placing that triangles BLAS into a TLAS.
  GPRTAccel triangleTLAS = gprtInstanceAccelCreate(context, 1, &mesh.accel);
  gprtAccelBuild(context, triangleTLAS, GPRT_BUILD_MODE_FAST_TRACE_NO_UPDATE);

  // Create initial aabb geometry 
  GPRTGeomOf<BoundingBoxData> aabbGeom = gprtGeomCreate<BoundingBoxData>(context, aabbType);
  BoundingBoxData* aabbGeomData = gprtGeomGetParameters(aabbGeom);
  aabbGeomData->aabbs = gprtBufferGetHandle(aabbPositions);
  gprtAABBsSetPositions(aabbGeom, aabbPositions, 1);

  // Place that geometry into an AABB BLAS.
  GPRTAccel aabbAccel = gprtAABBAccelCreate(context, 1, &aabbGeom);
  gprtAccelBuild(context, aabbAccel, GPRT_BUILD_MODE_FAST_TRACE_AND_UPDATE);

  // Placing that AABB BLAS into a TLAS.
  GPRTAccel obbAccel = gprtInstanceAccelCreate(context, 1, &aabbAccel);
  gprtInstanceAccelSet3x4Transforms(obbAccel, transformBuffer);
  gprtAccelBuild(context, obbAccel, GPRT_BUILD_MODE_FAST_TRACE_AND_UPDATE);

  // Here, we place a reference to our TLAS in the ray generation
  // kernel's parameters
  rayGenData->triangleTLAS = gprtAccelGetHandle(triangleTLAS);
  rayGenData->obbAccel = gprtAccelGetHandle(obbAccel);

  gprtBuildShaderBindingTable(context, GPRT_SBT_ALL);

  // ##################################################################
  // now that everything is ready: launch it ....
  // ##################################################################

  LOG("launching ...");



  CompositeGuiConstants guiPC;
  guiPC.fbSize = fbSize;
  guiPC.frameBuffer = gprtBufferGetHandle(frameBuffer);
  guiPC.imageBuffer = gprtBufferGetHandle(imageBuffer);
  guiPC.guiTexture = gprtTextureGetHandle(guiColorAttachment);

  RTPushConstants rtPC;

  std::vector<float> surfaceAreas;

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
      float4x4 rotationMatrixX = rotation_matrix(rotation_quat(lookUp, xAngle));
      position = (mul(rotationMatrixX, (position - pivot))) + pivot;

      // step 3: Rotate the camera around the pivot point on the second axis.
      float3 lookRight = cross(lookUp, normalize(pivot - position).xyz());
      float4x4 rotationMatrixY = rotation_matrix(rotation_quat(lookRight, yAngle));
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

    // Update the OBB
    if (step == 1) surfaceAreas.clear();

    gprtBufferClear(aabbPositions);
    gprtComputeLaunch(ClearOBB, {1,1,1}, {1,1,1}, obbPC);
    gprtComputeLaunch(ComputeOBB, {divUp(obbPC.numTrisToInclude, 128), 1, 1}, {128,1,1}, obbPC);
    gprtComputeLaunch(BackPropOBB, {divUp(obbPC.numTrisToInclude, 128), 1, 1}, {128,1,1}, obbPC);
      
    // Optimize using Adam
    gprtBufferMap(eulRots);
    float3 *eulPtr = gprtBufferGetPointer(eulRots);
    float LR = powf(10.f, -learningRate);
    float3 grad = eulPtr[1];
    float3 jitter = float3((float)rand() / RAND_MAX, (float)rand() / RAND_MAX, (float)rand() / RAND_MAX) - 0.5f;
    if (step == 1) {
      m1 = float3(0.f, 0.f, 0.f);
      m2 = float3(0.f, 0.f, 0.f);
    }
    m1 = adamBeta1 * m1 + (1 - adamBeta1) * grad;
    if (useVectorAdam) {
      m2 = adamBeta2 * m2 + (1 - adamBeta2) * sqrt(dot(grad, grad));
    } else {
      m2 = adamBeta2 * m2 + (1 - adamBeta2) * (grad * grad);
    }
    
    float3 m_hat = m1 / (1.f - powf(adamBeta1, float(step)));
    float3 v_hat = m2 / (1.f - powf(adamBeta2, float(step)));

    float3 update = -LR * m_hat / (v_hat + 1E-6f);
    eulPtr[0] = eulPtr[0] + update + LR * 1E-10f * jitter;
    step++;

    // rebuild OBB accel
    gprtAccelUpdate(context, aabbAccel);
    gprtAccelUpdate(context, obbAccel);

    gprtTrianglesSetIndices(mesh.geometry, mesh.indexBuffer, obbPC.numTrisToInclude);
    gprtAccelBuild(context, mesh.accel, GPRT_BUILD_MODE_FAST_BUILD_NO_UPDATE);
    gprtAccelBuild(context, triangleTLAS, GPRT_BUILD_MODE_FAST_BUILD_NO_UPDATE);
    rayGenData->triangleTLAS = gprtAccelGetHandle(triangleTLAS);

    // Call the GPU raygen kernel function
    gprtRayGenLaunch2D(context, rayGen, fbSize.x, fbSize.y, rtPC);

    // Set our ImGui state
    ImGui::Text("Eul %f %f %f", eulPtr[0].x, eulPtr[0].y, eulPtr[0].z);
    ImGui::Text("First order moment estimate %.3f %.3f %.3f", m1.x, m1.y, m1.z);
    ImGui::Text("Second order moment estimate %.3f %.3f %.3f", m2.x, m2.y, m2.z);
    ImGui::Text("Derivative Eul %f %f %f", eulPtr[1].x, eulPtr[1].y, eulPtr[1].z);
    ImGui::Text("Surface Area %f", eulPtr[2].x);
    surfaceAreas.push_back(eulPtr[2].x);

    ImGui::PlotLines("Surface Area", surfaceAreas.data(), surfaceAreas.size(), 0, 0, 3.4028235E38F, 3.4028235E38F, {200.f, 100.f}, 4);
    
    // ImGui::Text("Derivative surface Area %f", eulPtr[2].y);

    ImGui::Text("Step %d", step);

    if (ImGui::Checkbox("Use Vector Adam Optimizer", &useVectorAdam)) reset();
    
    if (ImGui::SliderInt("Tris to include", &obbPC.numTrisToInclude, 1, mesh.indices.size())) reset();
    
    ImGui::SliderFloat("Learning rate 1e10^-n", &learningRate, .1, 2);
    ImGui::SliderFloat("Adam Beta 1 1e10^-n", &adamBeta1, .9, .999);
    ImGui::SliderFloat("Adam Beta 2 1e10^-n", &adamBeta2, .9, .999);

    if (ImGui::Button("Reset")) {
      reset();
      eulPtr[0] = 6.28f * float3((float)rand() / RAND_MAX, (float)rand() / RAND_MAX, (float)rand() / RAND_MAX) - 3.14f;
    }

    gprtBufferUnmap(eulRots);
    ImGui::EndFrame();

    // Rasterize our gui
    gprtTextureClear(guiDepthAttachment);
    gprtTextureClear(guiColorAttachment);
    gprtGuiRasterize(context);

    // Finally, composite the gui onto the screen using a compute shader.
    gprtBufferTextureCopy(context, imageBuffer, imageTexture, 0, 0, 0, 0, 0, 0,
                          fbSize.x, fbSize.y, 1);

    gprtComputeLaunch(CompositeGui, {fbSize.x, fbSize.y, 1}, {1,1,1}, guiPC);

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

  
  gprtTextureDestroy(guiColorAttachment);
  gprtTextureDestroy(guiDepthAttachment);

  // gprtGeomDestroy(trianglesGeom);
  mesh.cleanupMesh();
  gprtGeomTypeDestroy(trianglesGeomType);
  gprtModuleDestroy(module);
  gprtContextDestroy(context);

  LOG_OK("seems all went OK; app is done, this should be the last output ...");
}
