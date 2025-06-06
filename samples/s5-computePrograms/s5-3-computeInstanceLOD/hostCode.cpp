#include <gprt.h>      // Public GPRT API
#include "sharedCode.h" // Shared data between host and device
#include <generator.hpp> // for generating meshes
using namespace generator;

extern GPRTProgram s5_3_deviceCode;

// initial image resolution
const int2 fbSize = {1400, 460};

const char *outFileName = "s5-3-computeInstanceLOD.png";

// Number of refinement levels to use
const int NUM_LODS = 6;

float3 lookFrom = {-2.f, -2.f, 0.f};
float3 lookAt = {0.f, 0.f, 0.f};
float3 lookUp = {0.f, 0.f, -1.f};
float cosFovy = 0.3f;

// A class we'll use to quickly generate meshes and bottom level trees
template <typename T> struct Mesh {
  std::vector<float3> vertices;
  std::vector<uint3> indices;
  GPRTBufferOf<float3> vertexBuffer;
  GPRTBufferOf<uint3> indexBuffer;
  GPRTGeomOf<TrianglesGeomData> geometry;
  GPRTAccel accel;

  Mesh() {};
  Mesh(GPRTContext context, GPRTGeomTypeOf<TrianglesGeomData> geomType, int lvlOfDetail, T generator) {
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
    geomData->vertex = gprtBufferGetDevicePointer(vertexBuffer);
    geomData->index = gprtBufferGetDevicePointer(indexBuffer);
    geomData->lvlOfDetail = lvlOfDetail;

    GPRTBuildParams buildParams;
    buildParams.buildMode = GPRT_BUILD_MODE_FAST_TRACE_NO_UPDATE;
    buildParams.allowCompaction = true;

    // Build the bottom level acceleration structure
    accel = gprtTriangleAccelCreate(context, geometry);
    gprtAccelBuild(context, accel, buildParams);
    // gprtAccelCompact(context, accel);
  };

  void cleanupMesh() {
    gprtAccelDestroy(accel);
    gprtGeomDestroy(geometry);
    gprtBufferDestroy(vertexBuffer);
    gprtBufferDestroy(indexBuffer);
  };
};

int main(int ac, char **av) {
  // In this example, we'll use a compute program to select an instance to
  // render depending on its distance from the camera, enabling level of
  // detail refinement.

  // create a context on the first device:
  gprtRequestWindow(fbSize.x, fbSize.y, "S07 Compute Instance Level of Detail");
  GPRTContext context = gprtContextCreate(nullptr, 1);
  GPRTModule module = gprtModuleCreate(context, s5_3_deviceCode);

  // Structure of parameters that change each frame. We can edit these
  // without rebuilding the shader binding table.
  PushConstants pc;
  pc.now = 0.f;

  // ##################################################################
  // set up all the GPU kernels we want to run
  // ##################################################################

  // -------------------------------------------------------
  // Setup geometry types
  // -------------------------------------------------------

  GPRTGeomTypeOf<TrianglesGeomData> trianglesGeomType = gprtGeomTypeCreate<TrianglesGeomData>(context, GPRT_TRIANGLES);
  gprtGeomTypeSetClosestHitProg(trianglesGeomType, 0, module, "ClosestHit");

  // -------------------------------------------------------
  // set up instance transform program to animate instances
  // -------------------------------------------------------
  auto transformProgram = gprtComputeCreate<PushConstants>(context, module, "Transform");

  // -------------------------------------------------------
  // set up ray gen program
  // -------------------------------------------------------
  GPRTRayGenOf<RayGenData> rayGen = gprtRayGenCreate<RayGenData>(context, module, "RayGen");

  // -------------------------------------------------------
  // set up miss
  // -------------------------------------------------------
  GPRTMissOf<MissProgData> miss = gprtMissCreate<MissProgData>(context, module, "miss");

  // ------------------------------------------------------------------
  // bottom level mesh instances
  // ------------------------------------------------------------------

  // For this example, we'll be animating a grid of utah teapot meshes.

  // We begin by making a set of teapot meshes of different levels of detail, storing
  // each into it's own bottom level acceleration structure
  std::vector<Mesh<TeapotMesh>> lodMeshes(NUM_LODS);
  std::vector<gprt::Instance> lodInstances(NUM_LODS);
  for (int i = 0; i < NUM_LODS; ++i) {
    lodMeshes[i] = Mesh<TeapotMesh>(context, trianglesGeomType, i, TeapotMesh{i + 1});
    lodInstances[i] = gprtAccelGetInstance(lodMeshes[i].accel);
  }

  // Place a reference to each LOD in a buffer that we can use on the device.
  GPRTBufferOf<gprt::Instance> meshLODBuffer =
      gprtDeviceBufferCreate<gprt::Instance>(context, NUM_LODS, lodInstances.data());

  // // Next, we'll create a grid of references to the same bottom level
  // // acceleration structure. This saves memory and improves performance over
  // // creating duplicate meshes.
  // // uint32_t numInstances = 500 * 500;
  uint32_t numInstances = 32 * 32;

  // ------------------------------------------------------------------
  // the instance acceleration structure
  // ------------------------------------------------------------------

  GPRTBufferOf<gprt::Instance> instancesBuffer = gprtDeviceBufferCreate<gprt::Instance>(context, numInstances);
  GPRTAccel world = gprtInstanceAccelCreate(context, numInstances, instancesBuffer);

  // Parameters for our transform program that'll animate our transforms
  pc.meshLODs = gprtBufferGetDevicePointer(meshLODBuffer);
  pc.instances = gprtBufferGetDevicePointer(instancesBuffer);
  pc.numInstances = numInstances;

  // Now, compute transforms in parallel with a transform compute shader
  gprtComputeLaunch(transformProgram, {int((numInstances + 127) / 128), 1, 1}, {128, 1, 1}, pc);

  GPRTBuildParams buildParams;
  buildParams.buildMode = GPRT_BUILD_MODE_FAST_BUILD_AND_UPDATE;

  // Now that the transforms are set, we can build our top level acceleration
  // structure
  gprtAccelBuild(context, world, buildParams);

  // ##################################################################
  // set the parameters for the rest of our kernels
  // ##################################################################

  // Setup pixel frame buffer
  GPRTBufferOf<uint32_t> frameBuffer = gprtDeviceBufferCreate<uint32_t>(context, fbSize.x * fbSize.y);

  // Raygen program frame buffer
  RayGenData *rayGenData = gprtRayGenGetParameters(rayGen);
  rayGenData->frameBuffer = gprtBufferGetDevicePointer(frameBuffer);
  rayGenData->world = gprtAccelGetDeviceAddress(world);

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
      pc.camera.pos = lookFrom;
      pc.camera.dir_00 = normalize(lookAt - lookFrom);
      float aspect = float(fbSize.x) / float(fbSize.y);
      pc.camera.dir_du = cosFovy * aspect * normalize(cross(pc.camera.dir_00, lookUp));
      pc.camera.dir_dv = cosFovy * normalize(cross(pc.camera.dir_du, pc.camera.dir_00));
      pc.camera.dir_00 -= 0.5f * pc.camera.dir_du;
      pc.camera.dir_00 -= 0.5f * pc.camera.dir_dv;
    }

    // update time to move instance transforms. Then, update only instance
    // accel.
    pc.now = float(gprtGetTime(context));
    gprtComputeLaunch(transformProgram, {int((numInstances + 127) / 128), 1, 1}, {128, 1, 1}, pc);
    gprtAccelUpdate(context, world);

    // Need to synchronize the compute here, as compute by default runs concurrently to graphics
    gprtComputeSynchronize(context);

    // Now, trace rays
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
