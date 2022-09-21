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

#define LOG(message)                                            \
  std::cout << GPRT_TERMINAL_BLUE;                               \
  std::cout << "#gprt.sample(main): " << message << std::endl;   \
  std::cout << GPRT_TERMINAL_DEFAULT;
#define LOG_OK(message)                                         \
  std::cout << GPRT_TERMINAL_LIGHT_BLUE;                         \
  std::cout << "#gprt.sample(main): " << message << std::endl;   \
  std::cout << GPRT_TERMINAL_DEFAULT;

extern std::map<std::string, std::vector<uint8_t>> sample01_deviceCode;

const int NUM_VERTICES = 8;
float3 vertices[NUM_VERTICES] =
  {
    { -1.f,-1.f,-1.f },
    { +1.f,-1.f,-1.f },
    { -1.f,+1.f,-1.f },
    { +1.f,+1.f,-1.f },
    { -1.f,-1.f,+1.f },
    { +1.f,-1.f,+1.f },
    { -1.f,+1.f,+1.f },
    { +1.f,+1.f,+1.f }
  };

const int NUM_INDICES = 12;
int3 indices[NUM_INDICES] =
  {
    { 0,1,3 }, { 2,3,0 },
    { 5,7,6 }, { 5,6,4 },
    { 0,4,5 }, { 0,5,1 },
    { 2,3,7 }, { 2,7,6 },
    { 1,5,7 }, { 1,7,3 },
    { 4,0,2 }, { 4,2,6 }
  };

float geometryTransform[3][4] = 
  {
    1.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 1.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 1.0f, 0.0f
  };

float instanceTransform[3][4] = 
  {
    1.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 1.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 1.0f, 0.0f
  };

const char *outFileName = "s01-simpleTriangles.png";
const int2 fbSize = {800,600};
const float3 lookFrom = {-4.f,-3.f,-2.f};
const float3 lookAt = {0.f,0.f,0.f};
const float3 lookUp = {0.f,1.f,0.f};
const float cosFovy = 0.66f;

#include <iostream>
int main(int ac, char **av)
{
  LOG("gprt example '" << av[0] << "' starting up");

  // create a context on the first device:
  GPRTContext context = gprtContextCreate(nullptr,1);
  GPRTModule module = gprtModuleCreate(context,sample01_deviceCode);

  // ##################################################################
  // set up all the *GEOMETRY* graph we want to render
  // ##################################################################

  // -------------------------------------------------------
  // declare geometry type
  // -------------------------------------------------------
  GPRTVarDecl trianglesGeomVars[] = {
    { "index",  GPRT_BUFPTR, GPRT_OFFSETOF(TrianglesGeomData,index)},
    { "vertex", GPRT_BUFPTR, GPRT_OFFSETOF(TrianglesGeomData,vertex)},
    { "color",  GPRT_FLOAT3, GPRT_OFFSETOF(TrianglesGeomData,color)},
    { /* sentinel to mark end of list */ }
  };
  GPRTGeomType trianglesGeomType
    = gprtGeomTypeCreate(context,
                        GPRT_TRIANGLES,
                        sizeof(TrianglesGeomData),
                        trianglesGeomVars,-1);
  gprtGeomTypeSetClosestHit(trianglesGeomType,0,
                           module,"TriangleMesh");

  // ##################################################################
  // set up all the *GEOMS* we want to run that code on
  // ##################################################################

  LOG("building geometries ...");

  // ------------------------------------------------------------------
  // triangle mesh
  // ------------------------------------------------------------------
  GPRTBuffer vertexBuffer
    = gprtHostPinnedBufferCreate(context,GPRT_FLOAT3,NUM_VERTICES,vertices);
  GPRTBuffer indexBuffer
    = gprtDeviceBufferCreate(context,GPRT_INT3,NUM_INDICES,indices);
  GPRTBuffer geometryTransformBuffer
    = gprtDeviceBufferCreate(context,GPRT_TRANSFORM,1,geometryTransform);
  GPRTBuffer instanceTransformBuffer
    = gprtDeviceBufferCreate(context,GPRT_TRANSFORM,1,instanceTransform);
  GPRTBuffer frameBuffer
    = gprtHostPinnedBufferCreate(context,GPRT_INT,fbSize.x*fbSize.y);

  GPRTGeom trianglesGeom
    = gprtGeomCreate(context,trianglesGeomType);

  gprtTrianglesSetVertices(trianglesGeom,vertexBuffer,
                           NUM_VERTICES,sizeof(float3),0);
  gprtTrianglesSetIndices(trianglesGeom,indexBuffer,
                          NUM_INDICES,sizeof(int3),0);

  gprtGeomSetBuffer(trianglesGeom,"vertex",vertexBuffer);
  gprtGeomSetBuffer(trianglesGeom,"index",indexBuffer);
  gprtGeomSet3f(trianglesGeom,"color",0,1,0);

  // ------------------------------------------------------------------
  // the group/accel for that mesh
  // ------------------------------------------------------------------
  GPRTAccel trianglesAccel = gprtTrianglesAccelCreate(context,1,&trianglesGeom);
  gprtTrianglesAccelSetTransforms(trianglesAccel, geometryTransformBuffer);
  gprtAccelBuild(context, trianglesAccel);
  
  GPRTAccel world = gprtInstanceAccelCreate(context,1,&trianglesAccel);
  gprtInstanceAccelSetTransforms(world, instanceTransformBuffer);
  gprtAccelBuild(context, world);

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
  GPRTMissProg missProg
    = gprtMissProgCreate(context,module,"miss",sizeof(MissProgData),
                        missProgVars,-1);

  // ----------- set variables  ----------------------------
  gprtMissProgSet3f(missProg,"color0",.8f,0.f,0.f);
  gprtMissProgSet3f(missProg,"color1",.8f,.8f,.8f);

  // -------------------------------------------------------
  // set up ray gen program
  // -------------------------------------------------------
  GPRTVarDecl rayGenVars[] = {
    { "fbSize",        GPRT_INT2,   GPRT_OFFSETOF(RayGenData,fbSize)},
    { "fbPtr",         GPRT_BUFPTR, GPRT_OFFSETOF(RayGenData,fbPtr)},
    { "world",         GPRT_ACCEL,  GPRT_OFFSETOF(RayGenData,world)},
    { "camera.pos",    GPRT_FLOAT3, GPRT_OFFSETOF(RayGenData,camera.pos)},
    { "camera.dir_00", GPRT_FLOAT3, GPRT_OFFSETOF(RayGenData,camera.dir_00)},
    { "camera.dir_du", GPRT_FLOAT3, GPRT_OFFSETOF(RayGenData,camera.dir_du)},
    { "camera.dir_dv", GPRT_FLOAT3, GPRT_OFFSETOF(RayGenData,camera.dir_dv)},
    { /* sentinel to mark end of list */ }
  };

  // ----------- create object  ----------------------------
  GPRTRayGen rayGen
    = gprtRayGenCreate(context,module,"simpleRayGen",
                      sizeof(RayGenData),
                      rayGenVars,-1);

  // ----------- compute variable values  ------------------
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

  // ----------- set variables  ----------------------------
  gprtRayGenSetBuffer(rayGen,"fbPtr",        frameBuffer);
  // gprtRayGenSet2i    (rayGen,"fbSize",       (const int2&)fbSize);
  gprtRayGenSet2i    (rayGen,"fbSize",       fbSize.x, fbSize.y);
  gprtRayGenSetAccel (rayGen,"world",        world);
  // gprtRayGenSet3f    (rayGen,"camera.pos",   (const float3&)camera_pos);
  // gprtRayGenSet3f    (rayGen,"camera.dir_00",(const float3&)camera_d00);
  // gprtRayGenSet3f    (rayGen,"camera.dir_du",(const float3&)camera_ddu);
  // gprtRayGenSet3f    (rayGen,"camera.dir_dv",(const float3&)camera_ddv);
  gprtRayGenSet3f    (rayGen,"camera.pos",   camera_pos.x, camera_pos.y, camera_pos.z);
  gprtRayGenSet3f    (rayGen,"camera.dir_00",camera_d00.x, camera_d00.y, camera_d00.z);
  gprtRayGenSet3f    (rayGen,"camera.dir_du",camera_ddu.x, camera_ddu.y, camera_ddu.z);
  gprtRayGenSet3f    (rayGen,"camera.dir_dv",camera_ddv.x, camera_ddv.y, camera_ddv.z);

  // ##################################################################
  // build *SBT* required to trace the groups
  // ##################################################################
  gprtBuildPrograms(context);
  gprtBuildPipeline(context);
  gprtBuildSBT(context);

  // ##################################################################
  // now that everything is ready: launch it ....
  // ##################################################################

  LOG("launching ...");

  gprtRayGenLaunch2D(context,rayGen,fbSize.x,fbSize.y);

  LOG("done with launch, writing picture ...");
  // for host pinned mem it doesn't matter which device we query...
  const uint32_t *fb
    = (const uint32_t*)gprtBufferGetPointer(frameBuffer,0);
  assert(fb);
  stbi_write_png(outFileName,fbSize.x,fbSize.y,4,
                 fb,int(fbSize.x) * sizeof(uint32_t));
  LOG_OK("written rendered frame buffer to file "<<outFileName);


  // ##################################################################
  // and finally, clean up
  // ##################################################################

  LOG("cleaning up ...");

  gprtBufferDestroy(vertexBuffer);
  gprtBufferDestroy(indexBuffer);
  gprtBufferDestroy(frameBuffer);
  gprtBufferDestroy(geometryTransformBuffer);
  gprtBufferDestroy(instanceTransformBuffer);
  gprtRayGenDestroy(rayGen);
  gprtMissProgDestroy(missProg);
  gprtAccelDestroy(trianglesAccel);
  gprtAccelDestroy(world);
  gprtGeomDestroy(trianglesGeom);
  gprtGeomTypeDestroy(trianglesGeomType);
  gprtModuleDestroy(module);
  gprtContextDestroy(context);

  LOG_OK("seems all went OK; app is done, this should be the last output ...");
}
