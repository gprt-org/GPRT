// This program sets up a single geometric object, a mesh for a cube, and
// its acceleration structure, then ray traces it.

#include <gprt.h>      // Public GPRT API
#include "sharedCode.h" // Shared data between host and device

extern GPRTProgram s2_0_deviceCode;

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
int3 indices[NUM_INDICES] = {{0, 1, 2}};

// initial image resolution
const int2 fbSize = {1400, 460};

// final image output
const char *outFileName = "s01-singleTriangle.png";

// Initial camera parameters
float3 lookFrom = {0.f, 0.f, -4.f};
float3 lookAt = {0.f, 0.f, 0.f};
float3 lookUp = {0.f, -1.f, 0.f};
float cosFovy = 0.66f;

int main(int ac, char **av) {
  gprtRequestWindow(fbSize.x, fbSize.y, "S01 Single Triangle");
  GPRTContext context = gprtContextCreate();
  GPRTModule module = gprtModuleCreate(context, s2_0_deviceCode);
  GPRTRayGenOf<RayGenData> rayGen = gprtRayGenCreate<RayGenData>(context, module, "raygen");
  GPRTMissOf<MissProgData> miss = gprtMissCreate<MissProgData>(context, module, "miss");

  // First, we need to declare our geometry type.
  // This includes all GPU kernels tied to the geometry, as well as the
  // parameters passed to the geometry when hit by rays.
  GPRTGeomTypeOf<TrianglesGeomData> trianglesGeomType = gprtGeomTypeCreate<TrianglesGeomData>(context, GPRT_TRIANGLES);
  gprtGeomTypeSetClosestHitProg(trianglesGeomType, 0, module, "TriangleMesh");

  // Setup pixel frame buffer
  GPRTBufferOf<uint32_t> frameBuffer = gprtDeviceBufferCreate<uint32_t>(context, fbSize.x * fbSize.y);

  // Raygen program frame buffer
  RayGenData *rayGenData = gprtRayGenGetParameters(rayGen);
  rayGenData->frameBuffer = gprtBufferGetDevicePointer(frameBuffer);

  // Miss program checkerboard background colors
  MissProgData *missData = gprtMissGetParameters(miss);
  missData->color0 = float3(0.1f, 0.1f, 0.1f);
  missData->color1 = float3(0.0f, 0.0f, 0.0f);

  // The vertex and index buffers here define the triangle vertices
  // and how those vertices are connected together.
  GPRTBufferOf<float3> vertexBuffer = gprtDeviceBufferCreate<float3>(context, NUM_VERTICES, vertices);
  GPRTBufferOf<int3> indexBuffer = gprtDeviceBufferCreate<int3>(context, NUM_INDICES, indices);

  // Next, we will create an instantiation of our geometry declaration.
  GPRTGeomOf<TrianglesGeomData> trianglesGeom = gprtGeomCreate<TrianglesGeomData>(context, trianglesGeomType);
  // We use these calls to tell the geometry what buffers store triangle
  // indices and vertices
  gprtTrianglesSetVertices(trianglesGeom, vertexBuffer, NUM_VERTICES);
  gprtTrianglesSetIndices(trianglesGeom, indexBuffer, NUM_INDICES);

  // Once we have our geometry, we need to place that geometry into an
  // acceleration structure. These acceleration structures allow rays to
  // determine which triangle the ray hits in a sub-linear amount of time.
  // This first acceleration structure level is called a bottom level
  // acceleration structure, or a BLAS.
  GPRTAccel trianglesAccel = gprtTriangleAccelCreate(context, 1, &trianglesGeom);
  gprtAccelBuild(context, trianglesAccel, GPRT_BUILD_MODE_FAST_TRACE_NO_UPDATE);

  // We can then make multiple "instances", or copies, of that BLAS in
  // a top level acceleration structure, or a TLAS. (we'll cover this later.)
  // Rays can only be traced into TLAS, so for now we just make one BLAS
  // instance.
  gprt::Instance instance = gprtAccelGetInstance(trianglesAccel);
  GPRTBufferOf<gprt::Instance> instanceBuffer = gprtDeviceBufferCreate<gprt::Instance>(context, 1, &instance);

  GPRTAccel world = gprtInstanceAccelCreate(context, 1, instanceBuffer);
  gprtAccelBuild(context, world, GPRT_BUILD_MODE_FAST_TRACE_NO_UPDATE);

  // Here, we place a reference to our TLAS in the ray generation
  // kernel's parameters, so that we can access that tree when
  // we go to trace our rays.
  rayGenData->world = gprtAccelGetHandle(world);

  gprtBuildShaderBindingTable(context, GPRT_SBT_ALL);

  // Structure of parameters that change each frame. We can edit these
  // without rebuilding the shader binding table.
  PushConstants pc;

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
      pc.camera.pos = camera_pos;
      pc.camera.dir_00 = camera_d00;
      pc.camera.dir_du = camera_ddu;
      pc.camera.dir_dv = camera_ddv;
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

  // Clean up resources
  gprtContextDestroy(context);

  return 0;
}
