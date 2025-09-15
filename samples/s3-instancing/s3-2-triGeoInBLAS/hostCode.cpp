#include <gprt.h>      // Public GPRT API
#include "sharedCode.h" // Shared data between host and device
#include <generator.hpp> // for generating meshes
using namespace generator;

extern GPRTProgram s3_2_deviceCode;

template <typename T> struct Mesh {
  std::vector<float3> vertices;
  std::vector<uint3> indices;
  GPRTBufferOf<float3> vertexBuffer;
  GPRTBufferOf<uint3> indexBuffer;
  GPRTGeomOf<TrianglesGeomData> geometry;

  Mesh() {};
  Mesh(GPRTContext context, GPRTGeomTypeOf<TrianglesGeomData> geomType, T generator, float3 color, float4x4 transform) {
    auto vertGenerator = generator.vertices();
    auto triGenerator = generator.triangles();
    while (!vertGenerator.done()) {
      auto vertex = vertGenerator.generate();
      auto position = vertex.position;
      float4 p = mul(transform, float4(vertex.position[0], vertex.position[1], vertex.position[2], 1.0));
      vertices.push_back(p.xyz());
      vertGenerator.next();
    }
    while (!triGenerator.done()) {
      Triangle triangle = triGenerator.generate();
      auto vertices = triangle.vertices;
      indices.push_back(uint3(vertices[0], vertices[1], vertices[2]));
      triGenerator.next();
    }

    vertexBuffer = gprtDeviceBufferCreate<float3>(context, vertices.size(), vertices.data());
    indexBuffer = gprtDeviceBufferCreate<uint3>(context, indices.size(), indices.data());
    geometry = gprtGeomCreate(context, geomType);
    TrianglesGeomData *geomData = gprtGeomGetParameters(geometry);
    gprtTrianglesSetVertices(geometry, vertexBuffer, (uint32_t) vertices.size());
    gprtTrianglesSetIndices(geometry, indexBuffer, (uint32_t) indices.size());
    geomData->vertex = gprtBufferGetDevicePointer(vertexBuffer);
    geomData->index = gprtBufferGetDevicePointer(indexBuffer);
    geomData->color = color;
  };

  void cleanup() {
    gprtGeomDestroy(geometry);
    gprtBufferDestroy(vertexBuffer);
    gprtBufferDestroy(indexBuffer);
  };
};

// initial image resolution
const int2 fbSize = {1400, 460};

const char *outFileName = "s3-2-TriGeoInBLAS.png";

float3 lookFrom = {10.f, 10.0f, 10.f};
float3 lookAt = {0.f, 0.f, 1.f};
float3 lookUp = {0.f, 0.f, -1.f};
float cosFovy = 0.4f;

int main(int ac, char **av) {
  // This example serves to demonstrate that multiple geometry can be placed
  // in the same bottom level acceleration structure.

  // create a context on the first device:
  gprtRequestWindow(fbSize.x, fbSize.y, "S08 Multiple Geometry");
  GPRTContext context = gprtContextCreate(nullptr, 1);
  GPRTModule module = gprtModuleCreate(context, s3_2_deviceCode);

  // ##################################################################
  // set up all the GPU kernels we want to run
  // ##################################################################

  // -------------------------------------------------------
  // Setup geometry types
  // -------------------------------------------------------
  GPRTGeomTypeOf<TrianglesGeomData> trianglesGeomType = gprtGeomTypeCreate<TrianglesGeomData>(context, GPRT_TRIANGLES);
  gprtGeomTypeSetClosestHitProg(trianglesGeomType, 0, module, "closesthit");

  // -------------------------------------------------------
  // set up ray gen program
  // -------------------------------------------------------
  GPRTRayGenOf<RayGenData> rayGen = gprtRayGenCreate<RayGenData>(context, module, "raygen");

  // -------------------------------------------------------
  // set up miss prog
  // -------------------------------------------------------
  GPRTMissOf<MissProgData> miss = gprtMissCreate<MissProgData>(context, module, "miss");

// ------------------------------------------------------------------
// Meshes
// ------------------------------------------------------------------
#ifndef M_PI
#define M_PI 3.14
#endif
  Mesh<TorusKnotMesh> torusMesh1(
      context, trianglesGeomType, TorusKnotMesh{2, 3, 32, 192}, float3(1, 0, 0),
      math::matrixFromTranslation(float3(2 * sin(2 * M_PI * .33), 2 * cos(2 * M_PI * .33), 1.5f)));
  Mesh<TorusKnotMesh> torusMesh2(
      context, trianglesGeomType, TorusKnotMesh{2, 5, 32, 192}, float3(0, 1, 0),
      math::matrixFromTranslation(float3(2 * sin(2 * M_PI * .66), 2 * cos(2 * M_PI * .66), 1.5f)));
  Mesh<TorusKnotMesh> torusMesh3(
      context, trianglesGeomType, TorusKnotMesh{2, 7, 32, 192}, float3(0, 0, 1),
      math::matrixFromTranslation(float3(2 * sin(2 * M_PI * 1.0), 2 * cos(2 * M_PI * 1.0), 1.5f)));
  Mesh<CappedCylinderMesh> floorMesh(context, trianglesGeomType, CappedCylinderMesh{5, 4, 128}, float3(1, 1, 1),
                                     math::matrixFromTranslation(float3(0.0f, 0.0f, -4.0f)));
  std::vector<GPRTGeomOf<TrianglesGeomData>> geoms = {torusMesh1.geometry, torusMesh2.geometry, torusMesh3.geometry,
                                                      floorMesh.geometry};
  GPRTAccel trianglesBLAS = gprtTriangleAccelCreate(context, geoms.size(), geoms.data());
  gprtAccelBuild(context, trianglesBLAS, GPRT_BUILD_MODE_FAST_TRACE_NO_UPDATE);

  gprt::Instance instance = gprtAccelGetInstance(trianglesBLAS);
  GPRTBufferOf<gprt::Instance> instances = gprtDeviceBufferCreate<gprt::Instance>(context, 1, &instance);
  GPRTAccel trianglesTLAS = gprtInstanceAccelCreate(context, 1, instances);
  gprtAccelBuild(context, trianglesTLAS, GPRT_BUILD_MODE_FAST_TRACE_NO_UPDATE);

  // ##################################################################
  // set the parameters for our kernels
  // ##################################################################

  // Setup pixel frame buffer
  GPRTBufferOf<uint32_t> frameBuffer = gprtDeviceBufferCreate<uint32_t>(context, fbSize.x * fbSize.y);

  // Raygen program frame buffer
  RayGenData *rayGenData = gprtRayGenGetParameters(rayGen);
  rayGenData->frameBuffer = gprtBufferGetDevicePointer(frameBuffer);
  rayGenData->world = gprtAccelGetHandle(trianglesTLAS);

  // Miss program checkerboard background colors
  MissProgData *missData = gprtMissGetParameters(miss);
  missData->color0 = float3(0.1f, 0.1f, 0.1f);
  missData->color1 = float3(0.0f, 0.0f, 0.0f);

  // Upload our newly assigned parameters to the shader binding table.
  gprtBuildShaderBindingTable(context);

  // ##################################################################
  // now that everything is ready: launch it ....
  // ##################################################################

  // Structure of parameters that change each frame. We can edit these
  // without rebuilding the shader binding table.
  Constants pc;

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
      float deltaAngleX = float(2 * M_PI / fbSize.x);
      float deltaAngleY = float(M_PI / fbSize.y);
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
}
