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

extern GPRTProgram cmd07_deviceCode;

template <typename T>
struct Mesh {
  std::vector<float3> vertices;
  std::vector<uint3> indices;
  GPRTBuffer vertexBuffer;
  GPRTBuffer indexBuffer;
  GPRTGeom geometry;

  Mesh() {};
  Mesh(GPRTContext context, GPRTGeomType geomType, T generator, float3 color, float4x4 transform)
  {
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
    gprtGeomSet3f(geometry,"color",color.x, color.y, color.z);
  };

  void cleanup() {
    gprtGeomDestroy(geometry);
    gprtBufferDestroy(vertexBuffer);
    gprtBufferDestroy(indexBuffer);
  };
};

// initial image resolution
const char *outFileName = "s07-multipleGeometry.png";
const int2 fbSize = {700,230};
const float3 lookFrom = {10.f,10.0f,10.f};
const float3 lookAt = {0.f,0.f,1.f};
const float3 lookUp = {0.f,0.f,1.f};
const float cosFovy = 0.4f;

#include <iostream>
int main(int ac, char **av)
{
  LOG("gprt example '" << av[0] << "' starting up");

  // create a context on the first device:
  GPRTContext context = gprtContextCreate(nullptr,1);
  GPRTModule module = gprtModuleCreate(context,cmd07_deviceCode);

  // ##################################################################
  // set up all the *GEOMETRY* graph we want to render
  // ##################################################################

  // -------------------------------------------------------
  // declare geometry type
  // -------------------------------------------------------
  GPRTVarDecl trianglesGeomVars[] = {
    { "index",  GPRT_BUFFER, GPRT_OFFSETOF(TrianglesGeomData,index)},
    { "vertex", GPRT_BUFFER, GPRT_OFFSETOF(TrianglesGeomData,vertex)},
    { "color",  GPRT_FLOAT3, GPRT_OFFSETOF(TrianglesGeomData,color)},
    { /* sentinel to mark end of list */ }
  };
  GPRTGeomType trianglesGeomType
    = gprtGeomTypeCreate(context,
                        GPRT_TRIANGLES,
                        sizeof(TrianglesGeomData),
                        trianglesGeomVars,-1);
  gprtGeomTypeSetClosestHitProg(trianglesGeomType,0,
                           module,"closesthit");

  // ##################################################################
  // set up all the *GEOMS* we want to run that code on
  // ##################################################################

  LOG("building geometries ...");

  // ------------------------------------------------------------------
  // Meshes
  // ------------------------------------------------------------------
  #ifndef M_PI
  #define M_PI 3.14
  #endif
  Mesh<TorusKnotMesh> torusMesh1(context, trianglesGeomType, TorusKnotMesh{2, 3, 32, 192}, float3(1,0,0), translation_matrix(float3(2*sin(2*M_PI*.33), 2*cos(2*M_PI*.33), 1.5f)));
  Mesh<TorusKnotMesh> torusMesh2(context, trianglesGeomType, TorusKnotMesh{2, 5, 32, 192}, float3(0,1,0), translation_matrix(float3(2*sin(2*M_PI*.66), 2*cos(2*M_PI*.66), 1.5f)));
  Mesh<TorusKnotMesh> torusMesh3(context, trianglesGeomType, TorusKnotMesh{2, 7, 32, 192}, float3(0,0,1), translation_matrix(float3(2*sin(2*M_PI*1.0), 2*cos(2*M_PI*1.0), 1.5f)));
  Mesh<CappedCylinderMesh> floorMesh(context, trianglesGeomType, CappedCylinderMesh{5, 4, 128}, float3(1,1,1), translation_matrix(float3(0.0f, 0.0f, -4.0f)));
  std::vector<GPRTGeom> geoms = {torusMesh1.geometry, torusMesh2.geometry, torusMesh3.geometry, floorMesh.geometry};
  GPRTAccel trianglesBLAS = gprtTrianglesAccelCreate(context,geoms.size(),geoms.data());
  GPRTAccel trianglesTLAS = gprtInstanceAccelCreate(context,1,&trianglesBLAS);
  gprtAccelBuild(context, trianglesBLAS);
  gprtAccelBuild(context, trianglesTLAS);

  GPRTBuffer frameBuffer
    = gprtHostBufferCreate(context,GPRT_INT,fbSize.x*fbSize.y);

  // ##################################################################
  // set miss and raygen program required for SBT
  // ##################################################################

  // -------------------------------------------------------
  // set up miss prog
  // -------------------------------------------------------
  GPRTVarDecl missProgVars[]
    = {
    { "color0", GPRT_FLOAT3, GPRT_OFFSETOF(MissProgData,color0)},
    { "color1", GPRT_FLOAT3, GPRT_OFFSETOF(MissProgData,color1)},
    { /* sentinel to mark end of list */ }
  };
  // ----------- create object  ----------------------------
  GPRTMiss miss
    = gprtMissCreate(context,module,"miss",sizeof(MissProgData),
                        missProgVars,-1);

  // ----------- set variables  ----------------------------
  gprtMissSet3f(miss,"color0",0.1f,0.1f,0.1f);
  gprtMissSet3f(miss,"color1",.0f,.0f,.0f);

  // -------------------------------------------------------
  // set up ray gen program
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

  // ----------- create object  ----------------------------
  GPRTRayGen rayGen
    = gprtRayGenCreate(context,module,"raygen",
                      sizeof(RayGenData),
                      rayGenVars,-1);

  // ----------- set variables  ----------------------------
  gprtRayGenSetBuffer(rayGen,"fbPtr", frameBuffer);
  gprtRayGenSet2iv(rayGen,"fbSize", (int32_t*)&fbSize);
  gprtRayGenSetAccel(rayGen,"world", trianglesTLAS);

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
  gprtBuildPipeline(context);
  gprtBuildShaderBindingTable(context);

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

  torusMesh1.cleanup();
  torusMesh2.cleanup();
  torusMesh3.cleanup();
  floorMesh.cleanup();
  gprtBufferDestroy(frameBuffer);
  gprtRayGenDestroy(rayGen);
  gprtMissDestroy(miss);
  gprtAccelDestroy(trianglesBLAS);
  gprtAccelDestroy(trianglesTLAS);
  gprtGeomTypeDestroy(trianglesGeomType);
  gprtModuleDestroy(module);
  gprtContextDestroy(context);

  LOG_OK("seems all went OK; app is done, this should be the last output ...");
}