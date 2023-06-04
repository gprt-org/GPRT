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

#include <iomanip>

// our shared data structures between host and device
#include "sharedCode.h"

// The software linear bvh builder included in GPRT. Useful for 
// scenarios where full control over traversal is required
#include "gprt_lbvh.h"

// for loading meshes
#define TINYOBJLOADER_IMPLEMENTATION
#include <tiny_obj_loader.h>

#define LOG(message)                                                                                                   \
  std::cout << GPRT_TERMINAL_BLUE;                                                                                     \
  std::cout << "#gprt.sample(main): " << message << std::endl;                                                         \
  std::cout << GPRT_TERMINAL_DEFAULT;
#define LOG_OK(message)                                                                                                \
  std::cout << GPRT_TERMINAL_LIGHT_BLUE;                                                                               \
  std::cout << "#gprt.sample(main): " << message << std::endl;                                                         \
  std::cout << GPRT_TERMINAL_DEFAULT;

extern GPRTProgram s12_deviceCode;
extern GPRTProgram lbvhDeviceCode;

// A class we'll use to quickly generate meshes and bottom level trees
template <typename T> struct Mesh {
  std::vector<float3> vertices;
  std::vector<uint3> indices;
  GPRTBufferOf<float3> vertexBuffer;
  GPRTBufferOf<uint3> indexBuffer;
  Mesh(){};
  Mesh(GPRTContext context, T generator) {
    // Use the generator to generate vertices and indices
    auto vertGenerator = generator.vertices();
    auto triGenerator = generator.triangles();
    while (!vertGenerator.done()) {
      auto vertex = vertGenerator.generate();
      auto position = vertex.position;
      vertices.push_back(float3(position[0], position[2], position[1]) * .3f);// + float3(0.f, .5f, 0.f));
      vertGenerator.next();
    }
    while (!triGenerator.done()) {
      Triangle triangle = triGenerator.generate();
      auto vertices = triangle.vertices;
      indices.push_back(uint3(vertices[0], vertices[1], vertices[2]));
      triGenerator.next();
    }

    // Upload those to the device
    vertexBuffer = gprtDeviceBufferCreate<float3>(context, vertices.size(), vertices.data());
    indexBuffer = gprtDeviceBufferCreate<uint3>(context, indices.size(), indices.data());
  };

  void cleanup() {
    gprtBufferDestroy(vertexBuffer);
    gprtBufferDestroy(indexBuffer);
  };
};

// initial image resolution
// const int2 fbSize = {1400, 460};
const int2 fbSize = {1024, 1024};

// final image output
const char *outFileName = "s12-swBVH.png";

// Initial camera parameters
float3 lookFrom = {1.7f, 2.4f, -2.8f};
float3 lookAt = {0.0f, 0.5f, 0.0f};
float3 lookUp = {0.f, -1.f, 0.f};
float cosFovy = 0.66f;

#include <iostream>
int
main(int ac, char **av) {
  // In this example, we'll use compute shaders to build a sofware-traversable 
  // acceleration structure in parallel on the GPU. We'll use this tree for custom
  // tree traversal, namely a closest point on triangle query to compute a 
  // signed distance field.
  LOG("gprt example '" << av[0] << "' starting up");

  // create a context on the first device:
  gprtRequestWindow(fbSize.x, fbSize.y, "S04 Compute AABB");
  GPRTContext context = gprtContextCreate(nullptr, 1);
  GPRTModule module = gprtModuleCreate(context, s12_deviceCode);
  GPRTModule lbvhModule = gprtModuleCreate(context, lbvhDeviceCode);

  // ##################################################################
  // set up all the GPU kernels we want to run
  // ##################################################################

  // -------------------------------------------------------
  // set up LBVH programs for a triangle-based SW tree. We
  // will use this SW tree for closest-point-on-triangle queries
  // -------------------------------------------------------

  GPRTComputeOf<LBVHData> computeBounds = gprtComputeCreate<LBVHData>(context, lbvhModule, "ComputeTriangleBounds");
  GPRTComputeOf<LBVHData> computeCodes = gprtComputeCreate<LBVHData>(context, lbvhModule, "ComputeTriangleMortonCodes");
  GPRTComputeOf<LBVHData> makeNodes = gprtComputeCreate<LBVHData>(context, lbvhModule, "MakeNodes");
  GPRTComputeOf<LBVHData> splitNodes = gprtComputeCreate<LBVHData>(context, lbvhModule, "SplitNodes");
  GPRTComputeOf<LBVHData> buildHierarchy = gprtComputeCreate<LBVHData>(context, lbvhModule, "BuildTriangleHierarchy");

  // Triangle mesh we'll build the SW BVH over

  std::string inputfile = ASSETS_DIRECTORY "utah_teapot.obj";
  tinyobj::ObjReaderConfig reader_config;
  reader_config.mtl_search_path = ASSETS_DIRECTORY "./"; // Path to material files
  tinyobj::ObjReader reader;

  if (!reader.ParseFromFile(inputfile, reader_config)) {
    if (!reader.Error().empty()) {
        std::cerr << "TinyObjReader: " << reader.Error();
    }
    exit(1);
  }

  if (!reader.Warning().empty()) {
    std::cout << "TinyObjReader: " << reader.Warning();
  }

  auto& attrib = reader.GetAttrib();
  auto& shapes = reader.GetShapes();

  std::vector<float3> vertices;
  std::vector<int3> indices;

  // Loop over shapes
  for (size_t s = 0; s < shapes.size(); s++) {
    // Loop over faces(polygon)
    size_t index_offset = 0;
    for (size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) {
      size_t fv = size_t(shapes[s].mesh.num_face_vertices[f]);

      if (fv != 3) throw std::runtime_error("Error, expected only triangle faces");

      // Loop over vertices in the face.
      for (size_t v = 0; v < fv; v++) {
        // access to vertex
        tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];
        tinyobj::real_t vx = attrib.vertices[3*size_t(idx.vertex_index)+0];
        tinyobj::real_t vy = attrib.vertices[3*size_t(idx.vertex_index)+1];
        tinyobj::real_t vz = attrib.vertices[3*size_t(idx.vertex_index)+2];

        vertices.push_back(float3(vx, vy, vz));

        // // Check if `normal_index` is zero or positive. negative = no normal data
        // if (idx.normal_index >= 0) {
        //   tinyobj::real_t nx = attrib.normals[3*size_t(idx.normal_index)+0];
        //   tinyobj::real_t ny = attrib.normals[3*size_t(idx.normal_index)+1];
        //   tinyobj::real_t nz = attrib.normals[3*size_t(idx.normal_index)+2];
        // }

        // // Check if `texcoord_index` is zero or positive. negative = no texcoord data
        // if (idx.texcoord_index >= 0) {
        //   tinyobj::real_t tx = attrib.texcoords[2*size_t(idx.texcoord_index)+0];
        //   tinyobj::real_t ty = attrib.texcoords[2*size_t(idx.texcoord_index)+1];
        // }

        // Optional: vertex colors
        // tinyobj::real_t red   = attrib.colors[3*size_t(idx.vertex_index)+0];
        // tinyobj::real_t green = attrib.colors[3*size_t(idx.vertex_index)+1];
        // tinyobj::real_t blue  = attrib.colors[3*size_t(idx.vertex_index)+2];
      }
      indices.push_back(int3(0 + index_offset, 1 + index_offset, 2 + index_offset));
      index_offset += fv;

      // per-face material
      shapes[s].mesh.material_ids[f];
    }
  }

  for (uint32_t i = 0; i < indices.size(); ++i) {
    int3 triangle = indices[i];
    if (triangle.x < 0 || triangle.x >= vertices.size()) throw std::runtime_error("invalid vertex");
    if (triangle.y < 0 || triangle.y >= vertices.size()) throw std::runtime_error("invalid vertex");
    if (triangle.z < 0 || triangle.z >= vertices.size()) throw std::runtime_error("invalid vertex");
  }


  LBVHData lbvhParams = {};
  lbvhParams.numPrims = indices.size();
  lbvhParams.numInner = lbvhParams.numPrims - 1;
  lbvhParams.numNodes = 2 * lbvhParams.numPrims - 1;
  
  // Input to LBVH construction
  GPRTBufferOf<float3> vertexBuffer = gprtDeviceBufferCreate<float3>(context, vertices.size(), vertices.data());
  GPRTBufferOf<int3> indexBuffer = gprtDeviceBufferCreate<int3>(context, indices.size(), indices.data());
  lbvhParams.triangles = gprtBufferGetHandle(indexBuffer);
  lbvhParams.positions = gprtBufferGetHandle(vertexBuffer);

  // Output / intermediate buffers
  GPRTBufferOf<uint8_t> scratch = gprtDeviceBufferCreate<uint8_t>(context);
  GPRTBufferOf<uint32_t> mortonCodes = gprtDeviceBufferCreate<uint32_t>(context, lbvhParams.numPrims);
  GPRTBufferOf<uint32_t> ids = gprtDeviceBufferCreate<uint32_t>(context, lbvhParams.numPrims);
  GPRTBufferOf<int4> nodes = gprtDeviceBufferCreate<int4>(context, lbvhParams.numNodes);
  GPRTBufferOf<float3> aabbs = gprtDeviceBufferCreate<float3>(context, 2 * lbvhParams.numNodes);
  lbvhParams.mortonCodes = gprtBufferGetHandle(mortonCodes);
  lbvhParams.ids = gprtBufferGetHandle(ids);
  lbvhParams.nodes = gprtBufferGetHandle(nodes);
  lbvhParams.aabbs = gprtBufferGetHandle(aabbs);

  // initialize root AABB
  gprtBufferMap(aabbs);
  float3* aabbPtr = gprtBufferGetPointer(aabbs);
  aabbPtr[0].x = aabbPtr[0].y = aabbPtr[0].z = 1e20f;
  aabbPtr[1].x = aabbPtr[1].y = aabbPtr[1].z = -1e20f;
  gprtBufferUnmap(aabbs);

  gprtComputeSetParameters(computeBounds,  &lbvhParams);
  gprtComputeSetParameters(computeCodes,   &lbvhParams);
  gprtComputeSetParameters(makeNodes,      &lbvhParams);
  gprtComputeSetParameters(splitNodes,     &lbvhParams);
  gprtComputeSetParameters(buildHierarchy, &lbvhParams);

  gprtBuildShaderBindingTable(context, GPRT_SBT_COMPUTE);

  gprtComputeLaunch1D(context, computeBounds, lbvhParams.numPrims);
  gprtComputeLaunch1D(context, computeCodes, lbvhParams.numPrims);
  gprtBufferSortPayload(context, mortonCodes, ids, scratch);
  {
    gprtBufferMap(mortonCodes);
    gprtBufferMap(ids);
    uint32_t *keys = gprtBufferGetPointer(mortonCodes);
    uint32_t *values = gprtBufferGetPointer(ids);

    for (int i = 0; i < lbvhParams.numPrims; ++i) {
      if (i > 0 && keys[i] < keys[i - 1]) {
        std::cout<<keys[i-1]<<std::endl;
        std::cout<<keys[i]<<std::endl;
        throw std::runtime_error("Error, keys out of order!");
      }
    }

    gprtBufferUnmap(mortonCodes);
    gprtBufferUnmap(ids);
  }

  gprtComputeLaunch1D(context, makeNodes, lbvhParams.numNodes);
  gprtComputeLaunch1D(context, splitNodes, lbvhParams.numInner);
  // {
  //   gprtBufferMap(nodes);
  //   // gprtBufferMap(aabbs);
  //   int4 *nodePtr = gprtBufferGetPointer(nodes);
  //   // float3 *aabbPtr = gprtBufferGetPointer(aabbs);
  //   for (uint32_t i = 0; i < lbvhParams.numNodes; ++i) {
  //     std::cout<< std::setw(4) << nodePtr[i].x << " " << std::setw(4) << nodePtr[i].y << " " << std::setw(4) << nodePtr[i].z << " " << std::setw(4) << nodePtr[i].w << " ";
  //     std::cout<<std::endl;
  //     // std::cout<<"\taabb (" << aabbPtr[i*2+0].x << " " << aabbPtr[i*2+0].y << " " << aabbPtr[i*2+0].z << ")";
  //     // std::cout<<", (" << aabbPtr[i*2+1].x << " " << aabbPtr[i*2+1].y << " " << aabbPtr[i*2+1].z << ")"<< std::endl;
  //   }
  //   gprtBufferUnmap(nodes);
  //   // gprtBufferUnmap(aabbs);
  // }

  gprtComputeLaunch1D(context, buildHierarchy, lbvhParams.numPrims);


  // -------------------------------------------------------
  // set up ray gen program
  // -------------------------------------------------------
  GPRTRayGenOf<RayGenData> rayGen = gprtRayGenCreate<RayGenData>(context, module, "simpleRayGen");

  // ##################################################################
  // set the parameters for the rest of our kernels
  // ##################################################################

  // Setup pixel frame buffer
  GPRTBuffer frameBuffer = gprtDeviceBufferCreate(context, sizeof(uint32_t), fbSize.x * fbSize.y);
  GPRTBufferOf<float4> accumBuffer = gprtDeviceBufferCreate<float4>(context, fbSize.x * fbSize.y);

  // Raygen program frame buffer
  RayGenData *rayGenData = gprtRayGenGetParameters(rayGen);
  rayGenData->frameBuffer = gprtBufferGetHandle(frameBuffer);
  rayGenData->accumBuffer = gprtBufferGetHandle(accumBuffer);

  rayGenData->lbvh = lbvhParams;

  rayGenData->cuttingPlane = 0.f;

  gprtBuildShaderBindingTable(context, GPRT_SBT_ALL);

  // ##################################################################
  // now that everything is ready: launch it ....
  // ##################################################################

  LOG("launching ...");

  bool firstFrame = true;
  double xpos = 0.f, ypos = 0.f;
  double lastxpos, lastypos;
  int iFrame = 0;
  do {
    float speed = .001f;
    lastxpos = xpos;
    lastypos = ypos;
    gprtGetCursorPos(context, &xpos, &ypos);
    if (firstFrame) {
      lastxpos = xpos;
      lastypos = ypos;
    }
    int lstate = gprtGetMouseButton(context, GPRT_MOUSE_BUTTON_LEFT);
    int rstate = gprtGetMouseButton(context, GPRT_MOUSE_BUTTON_RIGHT);

    // If we click the mouse, we should rotate the camera
    // Here, we implement some simple camera controls
    if (rstate == GPRT_PRESS || firstFrame) {
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
      RayGenData *raygenData = gprtRayGenGetParameters(rayGen);
      raygenData->camera.pos = camera_pos;
      raygenData->camera.dir_00 = camera_d00;
      raygenData->camera.dir_du = camera_ddu;
      raygenData->camera.dir_dv = camera_ddv;

      iFrame = 0;
    }

    if (lstate == GPRT_PRESS || firstFrame) {
      RayGenData *raygenData = gprtRayGenGetParameters(rayGen);
      raygenData->cuttingPlane = (xpos / fbSize.x) * 2.f - 1.0f;
      iFrame = 0;
    }
    
    rayGenData->iTime = gprtGetTime(context) * .5;
    rayGenData->iFrame = iFrame;

    // Use this to upload all set parameters to our ray tracing device
    gprtBuildShaderBindingTable(context, GPRT_SBT_RAYGEN);

    // Calls the GPU raygen kernel function
    gprtRayGenLaunch2D(context, rayGen, fbSize.x, fbSize.y);

    // If a window exists, presents the framebuffer here to that window
    gprtBufferPresent(context, frameBuffer);

    iFrame++;
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

  gprtBufferDestroy(frameBuffer);
  gprtRayGenDestroy(rayGen);

  gprtComputeDestroy(computeBounds);
  gprtComputeDestroy(computeCodes);
  gprtComputeDestroy(makeNodes);
  gprtComputeDestroy(splitNodes);
  gprtComputeDestroy(buildHierarchy);
  gprtModuleDestroy(module);
  gprtContextDestroy(context);

  LOG_OK("seems all went OK; app is done, this should be the last output ...");
}
