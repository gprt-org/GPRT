#include <gprt.h>      // Public GPRT API
#include "sharedCode.h" // Shared data between host and device

#include "hplocShared.h"

#include <generator.hpp> // for generating meshes
using namespace generator;

extern GPRTProgram s5_9_deviceCode;
extern GPRTProgram s5_9_hplocBuildDeviceCode;

template <typename T> struct Mesh {
  std::vector<float3> vertices;
  std::vector<uint3> indices;
  GPRTBufferOf<float3> vertexBuffer;
  GPRTBufferOf<uint3> indexBuffer;

  Mesh() {};
  Mesh(GPRTContext context, T generator) {
    auto vertGenerator = generator.vertices();
    auto triGenerator = generator.triangles();
    while (!vertGenerator.done()) {
      auto vertex = vertGenerator.generate();
      auto position = vertex.position;
      float4 p = float4(vertex.position[0], vertex.position[1], vertex.position[2], 1.0);
      // float4 p = mul(transform, float4(vertex.position[0], vertex.position[1], vertex.position[2], 1.0));
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
  };

  void cleanup() {
    gprtBufferDestroy(vertexBuffer);
    gprtBufferDestroy(indexBuffer);
  };
};


// Vertices and radii that will be used to define virtual spheres.
// We want to use a compute shader to generate bounding boxes in parallel over these.
const int NUM_VERTICES = 11;
float3 vertices[NUM_VERTICES] = {
    {0.0f, 0.0f, 0.0f}, {0.1f, 0.0f, 0.0f}, {0.2f, 0.0f, 0.0f}, {0.3f, 0.0f, 0.0f},
    {0.4f, 0.0f, 0.0f}, {0.5f, 0.0f, 0.0f}, {0.6f, 0.0f, 0.0f}, {0.7f, 0.0f, 0.0f},
    {0.8f, 0.0f, 0.0f}, {0.9f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f},
};

float radii[NUM_VERTICES] = {.015f, .025f, .035f, .045f, .055f, .065f, .055f, .045f, .035f, .025f, .015f};

// initial image resolution
const int2 fbSize = {1400, 460};

// final image output
const char *outFileName = "s5-9-computeHPLOC.png";

// Initial camera parameters
float3 lookFrom = {0.5f, 0.0f, 0.6f};
float3 lookAt = {0.5f, 0.f, 0.f};
float3 lookUp = {0.f, -1.f, 0.f};
float cosFovy = 0.66f;

int main(int ac, char **av) {
  // In this example, we'll use a compute shader to generate a set of
  // procedural axis aligned bounding boxes in parallel on the GPU. Each
  // AABB will contain a single sphere.

  // create a context on the first device:
  gprtRequestMaxAttributeSize(2 * sizeof(float4));
  gprtRequestWindow(fbSize.x, fbSize.y, "S04 Compute AABB");
  GPRTContext context = gprtContextCreate(nullptr, 1);
  GPRTModule module = gprtModuleCreate(context, s5_9_deviceCode);

  GPRTModule hplocBuildModule = gprtModuleCreate(context, s5_9_hplocBuildDeviceCode);

  Mesh<SphereMesh> testMesh = Mesh<SphereMesh>(context, SphereMesh{});
  float3* vertexBufferPtr = gprtBufferGetDevicePointer(testMesh.vertexBuffer);
  uint3* indexBufferPtr = gprtBufferGetDevicePointer(testMesh.indexBuffer);

  HPLOCParams hploc;
  GPRTComputeOf<HPLOCParams> hplocBounds = gprtComputeCreate<HPLOCParams>(context, hplocBuildModule, "HPLOC_Bounds");
  GPRTComputeOf<HPLOCParams> hplocSFC = gprtComputeCreate<HPLOCParams>(context, hplocBuildModule, "HPLOC_SFC");
  GPRTComputeOf<HPLOCParams> hplocBuildBVH2 = gprtComputeCreate<HPLOCParams>(context, hplocBuildModule, "HPLOC_Build");
  GPRTComputeOf<HPLOCParams> hplocToBVHN = gprtComputeCreate<HPLOCParams>(context, hplocBuildModule, "HPLOC_ToBVHN");

  // One BVH might consist of multiple geometries. Here, we focus on just one geom.
  hploc.M = 1; // number of meshes
  hploc.N = testMesh.indices.size(); // total number of triangles across all geometries. 
  std::vector<uint32_t> triPrefixSum = {0};

  GPRTBufferOf<uint3*> triangleBuffers = gprtDeviceBufferCreate<uint3*>(context, 1, &indexBufferPtr);
  GPRTBufferOf<float3*> vertexBuffers = gprtDeviceBufferCreate<float3*>(context, 1, &vertexBufferPtr);

  // Space filling curve codes we'll use to initialize agglomerative clustering.
  GPRTBufferOf<uint2> clusterIndices = gprtDeviceBufferCreate<uint2>(context, hploc.N, nullptr, 16);
  GPRTBufferOf<uint64_t> spaceFillingCodes = gprtDeviceBufferCreate<uint64_t>(context, hploc.N, nullptr, 16);
  
  // For scheduling the BVH2->BVH-N planner.
  GPRTBufferOf<uint64_t> indexPairs = gprtDeviceBufferCreate<uint64_t>(context, hploc.N, nullptr, 16);
  GPRTBufferOf<uint8_t> scratchBuffer = gprtDeviceBufferCreate<uint8_t>((GPRTContext)context); // to be resized as needed.
  
  // Initialize this one to -1s
  GPRTBufferOf<float3> rootBounds = gprtDeviceBufferCreate<float3>(context, 2, nullptr, 16, /* create a descriptor handle for globally coherent atomics*/ true);
  GPRTBufferOf<uint32_t> bvh2ParentIDs = gprtDeviceBufferCreate<uint32_t>(context, hploc.N, nullptr, 16, /* create a descriptor handle for globally coherent atomics*/ true);
  GPRTBufferOf<uint32_t> atomicCounters = gprtDeviceBufferCreate<uint32_t>(context, 5, nullptr, 16, /* create a descriptor handle for globally coherent atomics*/ true);
  GPRTBufferOf<uint32_t> primitivePrefix = gprtDeviceBufferCreate<uint32_t>(context, 1, triPrefixSum.data());
  hploc.pID = gprtDeviceBufferGetHandle(bvh2ParentIDs);
  hploc.triangles = gprtBufferGetDevicePointer(triangleBuffers);
  hploc.vertices = gprtBufferGetDevicePointer(vertexBuffers);
  hploc.AC = gprtDeviceBufferGetHandle(atomicCounters);
  hploc.I = gprtBufferGetDevicePointer(clusterIndices);
  hploc.C = gprtBufferGetDevicePointer(spaceFillingCodes);
  hploc.indexPairs = gprtBufferGetDevicePointer(indexPairs);
  hploc.rootBounds = gprtDeviceBufferGetHandle(rootBounds);
  
  {
    gprtBufferMap(rootBounds);
    float3* rootBoundsPtr = gprtBufferGetHostPointer(rootBounds);
    rootBoundsPtr[0] = float3(1e38f, 1e38f, 1e38f);
    rootBoundsPtr[1] = float3(-1e38f, -1e38f, -1e38f);
    gprtBufferUnmap(rootBounds);
  }

  // Compute root bounds through atomic reduction. (also counts the number of valid primitives)
  gprtComputeLaunch(hplocBounds, {(127 + hploc.N) / 128, 1, 1}, {128, 1, 1}, hploc);

  // Compute space filling curve codes (also initializes parent IDs to -1 and BVH2->BVHN index pairs array)
  gprtComputeLaunch(hplocSFC, int3((hploc.N + 127) / 128, 1, 1), int3(128, 1, 1), hploc);

  // Sort the cluster indices by their corresponding space filling codes
  gprtBufferSortPayload((GPRTContext)context, spaceFillingCodes, clusterIndices, scratchBuffer);

  // Main bottom up BVH2 build routine
  gprtComputeLaunch(hplocBuildBVH2, {(hploc.N + 31) / 32, 1, 1}, {32, 1, 1}, hploc);

  uint32_t errorCode = 0;
  {
    gprtBufferMap(atomicCounters);
    uint32_t* counters = gprtBufferGetHostPointer(atomicCounters);
    uint32_t errorCode = counters[5];
    gprtBufferUnmap(atomicCounters);
  }
  if (errorCode != 0) {
    throw std::runtime_error("HPLOC build failed with error code :\n " + std::to_string(errorCode));
    return errorCode;
  }

  // int lastScratchSize = gprtBufferGetSize(scratchBuffer);
  //int currentScratchSize = gprtBufferGetSize(scratchBuffer);

  {
    gprtBufferMap(rootBounds);
    float3* rootBoundsPtr = gprtBufferGetHostPointer(rootBounds);
    std::cout<<"Bounds are " << rootBoundsPtr[0].x << " " << rootBoundsPtr[0].y << " " << rootBoundsPtr[0].z << ", "
                            << rootBoundsPtr[1].x << " " << rootBoundsPtr[1].y << " " << rootBoundsPtr[1].z << std::endl; 
    gprtBufferUnmap(rootBounds);
  }



  // ##################################################################
  // set up all the GPU kernels we want to run
  // ##################################################################

  // -------------------------------------------------------
  // declare geometry type
  // -------------------------------------------------------
  GPRTGeomTypeOf<AABBGeomData> customGeomType = gprtGeomTypeCreate<AABBGeomData>(context, GPRT_AABBS);
  gprtGeomTypeSetClosestHitProg(customGeomType, 0, module, "AABBClosestHit");
  gprtGeomTypeSetIntersectionProg(customGeomType, 0, module, "AABBIntersection");

  // -------------------------------------------------------
  // set up sphere bounding box compute program
  // -------------------------------------------------------
  GPRTComputeOf<AABBGeomData> boundsProgram = gprtComputeCreate<AABBGeomData>(context, module, "ComputeBounds");

  // -------------------------------------------------------
  // set up miss
  // -------------------------------------------------------
  GPRTMissOf<MissProgData> miss = gprtMissCreate<MissProgData>(context, module, "miss");

  // -------------------------------------------------------
  // set up ray gen program
  // -------------------------------------------------------
  GPRTRayGenOf<RayGenData> rayGen = gprtRayGenCreate<RayGenData>(context, module, "simpleRayGen");

  // ##################################################################
  // set the parameters for our compute kernel
  // ##################################################################

  // ------------------------------------------------------------------
  // aabb mesh
  // ------------------------------------------------------------------
  GPRTBufferOf<float3> vertexBuffer = gprtDeviceBufferCreate<float3>(context, NUM_VERTICES, vertices);
  GPRTBufferOf<float> radiusBuffer = gprtDeviceBufferCreate<float>(context, NUM_VERTICES, radii);
  GPRTBufferOf<float3> aabbPositionsBuffer = gprtDeviceBufferCreate<float3>(context, NUM_VERTICES * 2, nullptr);

  GPRTGeomOf<AABBGeomData> aabbGeom = gprtGeomCreate(context, customGeomType);
  gprtAABBsSetPositions(aabbGeom, aabbPositionsBuffer, NUM_VERTICES);

  AABBGeomData geomData;
  geomData.vertex = gprtBufferGetDevicePointer(vertexBuffer);
  geomData.radius = gprtBufferGetDevicePointer(radiusBuffer);
  geomData.aabbs = gprtBufferGetDevicePointer(aabbPositionsBuffer);
  gprtGeomSetParameters(aabbGeom, geomData);

  // Launch the compute kernel, which will populate our aabbPositionsBuffer
  gprtComputeLaunch(boundsProgram, {NUM_VERTICES, 1, 1}, {1, 1, 1}, geomData);

  // Now that the aabbPositionsBuffer is filled, we can compute our AABB
  // acceleration structure
  GPRTAccel aabbAccel = gprtAABBAccelCreate(context, aabbGeom);
  gprtAccelBuild(context, aabbAccel, GPRT_BUILD_MODE_FAST_TRACE_NO_UPDATE);

  gprt::Instance instance = gprtAccelGetInstance(aabbAccel);
  GPRTBufferOf<gprt::Instance> instanceBuffer = gprtDeviceBufferCreate(context, 1, &instance);

  GPRTAccel world = gprtInstanceAccelCreate(context, 1, instanceBuffer);
  gprtAccelBuild(context, world, GPRT_BUILD_MODE_FAST_TRACE_NO_UPDATE);

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

  return 0;
}
