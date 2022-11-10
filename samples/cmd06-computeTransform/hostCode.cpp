// MIT License

// Copyright (c) 2022 Nathan V. Morrical

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

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
// our device-side data structures
#include "deviceCode.h"
// external helper stuff for image output
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb/stb_image_write.h"
// for generating meshes
#include <generator.hpp>
using namespace generator;

#define LOG(message)                                            \
  std::cout << GPRT_TERMINAL_BLUE;                               \
  std::cout << "#gprt.sample(main): " << message << std::endl;   \
  std::cout << GPRT_TERMINAL_DEFAULT;
#define LOG_OK(message)                                         \
  std::cout << GPRT_TERMINAL_LIGHT_BLUE;                         \
  std::cout << "#gprt.sample(main): " << message << std::endl;   \
  std::cout << GPRT_TERMINAL_DEFAULT;

extern GPRTProgram cmd06_deviceCode;

const char *outFileName = "s06-computeTransform.png";
const int2 fbSize = {700,230};
const float3 lookFrom = {-2.f,-2.f,0.f};
const float3 lookAt = {0.f,0.f,0.f};
const float3 lookUp = {0.f,0.f,1.f};
const float cosFovy = 0.3f;


float transform[3][4] = 
  {
    1.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 1.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 1.0f, 0.0f
  };

template <typename T>
struct Mesh {
  std::vector<float3> vertices;
  std::vector<uint3> indices;
  GPRTBuffer vertexBuffer;
  GPRTBuffer indexBuffer;
  GPRTGeom geometry;
  GPRTAccel accel;

  Mesh() {};
  Mesh(GPRTContext context, GPRTGeomType geomType, T generator)
  {
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

    vertexBuffer
      = gprtDeviceBufferCreate(context,GPRT_FLOAT3,vertices.size(),vertices.data());
    indexBuffer
      = gprtDeviceBufferCreate(context,GPRT_UINT3,indices.size(),indices.data());
    geometry
      = gprtGeomCreate(context,geomType);
    gprtTrianglesSetVertices(geometry, vertexBuffer, 
                            vertices.size(), sizeof(float3), 0);
    gprtTrianglesSetIndices(geometry, indexBuffer, 
                            indices.size(), sizeof(uint3), 0);
    gprtGeomSetBuffer(geometry,"vertex",vertexBuffer);
    gprtGeomSetBuffer(geometry,"index",indexBuffer);
    accel = gprtTrianglesAccelCreate(context,1,&geometry);
    gprtAccelBuild(context, accel);
  };

  void cleanupMesh() {
    gprtAccelDestroy(accel);
    gprtGeomDestroy(geometry);
    gprtBufferDestroy(vertexBuffer);
    gprtBufferDestroy(indexBuffer);
  };
};


#include <iostream>
int main(int ac, char **av)
{
  // create a context on the first device:
  GPRTContext context = gprtContextCreate(nullptr,1);
  GPRTModule module = gprtModuleCreate(context,cmd06_deviceCode);

  // -------------------------------------------------------
  // Setup programs and geometry types
  // -------------------------------------------------------
  GPRTVarDecl rayGenVars[] = {
    { "fbSize",        GPRT_INT2,   GPRT_OFFSETOF(RayGenData,fbSize)},
    { "fbPtr",         GPRT_BUFFER, GPRT_OFFSETOF(RayGenData,fbPtr)},
    { "world",         GPRT_ACCEL,  GPRT_OFFSETOF(RayGenData,world)},
    { "camera.pos",    GPRT_FLOAT3, GPRT_OFFSETOF(RayGenData,camera.pos)},
    { "camera.dir_00", GPRT_FLOAT3, GPRT_OFFSETOF(RayGenData,camera.dir_00)},
    { "camera.dir_du", GPRT_FLOAT3, GPRT_OFFSETOF(RayGenData,camera.dir_du)},
    { "camera.dir_dv", GPRT_FLOAT3, GPRT_OFFSETOF(RayGenData,camera.dir_dv)},
    { /* sentinel to mark end of list */ }
  };
  GPRTRayGen rayGen
    = gprtRayGenCreate(context,module,"RayGen",
                      sizeof(RayGenData),
                      rayGenVars);

  GPRTVarDecl missVars[]
    = {
    { "color0", GPRT_FLOAT3, GPRT_OFFSETOF(MissProgData,color0)},
    { "color1", GPRT_FLOAT3, GPRT_OFFSETOF(MissProgData,color1)},
    { /* sentinel to mark end of list */ }
  };
  GPRTMiss miss
    = gprtMissCreate(context,module,"miss",sizeof(MissProgData),
                        missVars);

  GPRTVarDecl trianglesGeomVars[] = {
    { "index",  GPRT_BUFFER, GPRT_OFFSETOF(TrianglesGeomData,index)},
    { "vertex",  GPRT_BUFFER, GPRT_OFFSETOF(TrianglesGeomData,vertex)},
    { "now",  GPRT_FLOAT, GPRT_OFFSETOF(TrianglesGeomData,now)},
    { "gridSize",  GPRT_UINT, GPRT_OFFSETOF(TrianglesGeomData,gridSize)},
    { /* sentinel to mark end of list */ }
  };
  GPRTGeomType trianglesGeomType
    = gprtGeomTypeCreate(context,
                        GPRT_TRIANGLES,
                        sizeof(TrianglesGeomData),
                        trianglesGeomVars);
  gprtGeomTypeSetClosestHitProg(trianglesGeomType,0,
                           module,"ClosestHit");
  
  GPRTVarDecl transformVars[] = {
    { "transforms",  GPRT_BUFFER, GPRT_OFFSETOF(TransformData,transforms)},
    { "numTransforms",  GPRT_INT, GPRT_OFFSETOF(TransformData,numTransforms)},
    { "now",  GPRT_FLOAT, GPRT_OFFSETOF(TransformData,now)},
    { /* sentinel to mark end of list */ }
  };
  GPRTCompute transformProgram
    = gprtComputeCreate(context,module,"Transform",
                      sizeof(TransformData),
                      transformVars);

  gprtBuildPipeline(context);

  // ------------------------------------------------------------------
  // bottom level mesh instances
  // ------------------------------------------------------------------
  int numInstances = 50 * 50;

  Mesh<TeapotMesh> instanceMesh(context, trianglesGeomType, TeapotMesh{12});
  std::vector<GPRTAccel> instanceTrees(numInstances);
  for (int i = 0; i < numInstances; ++i) {
    instanceTrees[i] = instanceMesh.accel;
  }
  
  // ------------------------------------------------------------------
  // the instance acceleration structure
  // ------------------------------------------------------------------
  GPRTBuffer transformBuffer
    = gprtDeviceBufferCreate(context,GPRT_TRANSFORM,numInstances,nullptr);
  GPRTAccel world = gprtInstanceAccelCreate(context,numInstances,instanceTrees.data());
  gprtInstanceAccelSetTransforms(world, transformBuffer);

  // fill transform buffer in parallel using a compute shader
  gprtComputeSetBuffer(transformProgram, "transforms", transformBuffer);
  gprtComputeSet1i(transformProgram, "numTransforms", numInstances);
  gprtComputeSet1f(transformProgram, "now", 0.0f);

  gprtBuildShaderBindingTable(context, GPRT_SBT_COMPUTE);
  gprtComputeLaunch1D(context,transformProgram,numInstances);

  // now that the transforms and references are set, we can build our TLAS
  gprtAccelBuild(context, world);

  // ----------- set variables  ----------------------------
  gprtMissSet3f(miss,"color0",0.1f,0.1f,0.1f);
  gprtMissSet3f(miss,"color1",.0f,.0f,.0f);

  // ----------- set raygen variables  ----------------------------
  GPRTBuffer frameBuffer
    = gprtHostBufferCreate(context,GPRT_INT,fbSize.x*fbSize.y);
  gprtRayGenSetBuffer(rayGen,"fbPtr", frameBuffer);
  gprtRayGenSet2iv(rayGen,"fbSize", (int32_t*)&fbSize);
  gprtRayGenSetAccel(rayGen,"world", world);

  float3 camera_pos = lookFrom;
  float3 camera_d00
    = normalize(lookAt-lookFrom);
  float aspect = float(fbSize.x) / float(fbSize.y);
  float3 camera_ddu
    = cosFovy * aspect * normalize(cross(camera_d00,lookUp));
  float3 camera_ddv
    = cosFovy * normalize(cross(camera_ddu,camera_d00));
  camera_d00 -= 0.5f * camera_ddu;
  camera_d00 -= 0.5f * camera_ddv;

  gprtRayGenSet3f(rayGen,"camera.pos",   camera_pos.x, camera_pos.y, camera_pos.z);
  gprtRayGenSet3f(rayGen,"camera.dir_00",camera_d00.x, camera_d00.y, camera_d00.z);
  gprtRayGenSet3f(rayGen,"camera.dir_du",camera_ddu.x, camera_ddu.y, camera_ddu.z);
  gprtRayGenSet3f(rayGen,"camera.dir_dv",camera_ddv.x, camera_ddv.y, camera_ddv.z);

  // ##################################################################
  // build *SBT* required to trace the groups
  // ##################################################################
  
  // re-build the pipeline to account for newly introduced geometry
  gprtBuildPipeline(context);
  gprtBuildShaderBindingTable(context, GPRT_SBT_ALL);

  // ##################################################################
  // create a window we can use to display and interact with the image
  // ##################################################################
  LOG("launching ...");

  gprtRayGenLaunch2D(context,rayGen,fbSize.x,fbSize.y);
  
  LOG("done with launch, writing picture ...");
  // for host pinned mem it doesn't matter which device we query...
  const uint32_t *fb
    = (const uint32_t*)gprtBufferGetPointer(frameBuffer,0);
  assert(fb);
  stbi_flip_vertically_on_write(true);
  stbi_write_png(outFileName,fbSize.x,fbSize.y,4,
                 fb,int(fbSize.x) * sizeof(uint32_t));
  LOG_OK("written rendered frame buffer to file "<<outFileName);

  // ##################################################################
  // and finally, clean up
  // ##################################################################

  LOG("cleaning up ...");
  gprtBufferDestroy(frameBuffer);
  gprtBufferDestroy(transformBuffer);
  gprtRayGenDestroy(rayGen);
  gprtMissDestroy(miss);
  gprtComputeDestroy(transformProgram);
  gprtAccelDestroy(world);
  instanceMesh.cleanupMesh();
  gprtGeomTypeDestroy(trianglesGeomType);
  gprtModuleDestroy(module);
  gprtContextDestroy(context);

  LOG_OK("seems all went OK; app is done, this should be the last output ...");
}
