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

extern GPRTProgram cmd04_deviceCode;

const int NUM_VERTICES = 11;
float3 vertices[NUM_VERTICES] =
  {
    { 0.0f, 0.0f, 0.0f },
    { 0.1f, 0.0f, 0.0f },
    { 0.2f, 0.0f, 0.0f },
    { 0.3f, 0.0f, 0.0f },
    { 0.4f, 0.0f, 0.0f },
    { 0.5f, 0.0f, 0.0f },
    { 0.6f, 0.0f, 0.0f },
    { 0.7f, 0.0f, 0.0f },
    { 0.8f, 0.0f, 0.0f },
    { 0.9f, 0.0f, 0.0f },
    { 1.0f, 0.0f, 0.0f },
  };

float radii[NUM_VERTICES] =
  {
    .015, .025, .035, .045, .055, .065, .055, .045, .035, .025, .015
  };

float transform[3][4] =
  {
    1.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 1.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 1.0f, 0.0f
  };

const char *outFileName = "s04-computeAABBs.png";
const int2 fbSize = {700,230};
const float3 lookFrom = {0.5f,0.0f,0.6f};
const float3 lookAt = {0.5f,0.f,0.f};
const float3 lookUp = {0.f,1.f,0.f};
const float cosFovy = 0.66f;

#include <iostream>
int main(int ac, char **av)
{
  LOG("gprt example '" << av[0] << "' starting up");

  // create a context on the first device:
  GPRTContext context = gprtContextCreate(nullptr,1);
  GPRTModule module = gprtModuleCreate(context,cmd04_deviceCode);

  // -------------------------------------------------------
  // Setup programs and geometry types
  // -------------------------------------------------------
  GPRTVarDecl aabbGeomVars[] = {
    { "vertex",  GPRT_BUFFER, GPRT_OFFSETOF(AABBGeomData,vertex)},
    { "radius",  GPRT_BUFFER, GPRT_OFFSETOF(AABBGeomData,radius)},
    { "color",  GPRT_FLOAT3, GPRT_OFFSETOF(AABBGeomData,color)},
    { /* sentinel to mark end of list */ }
  };
  GPRTGeomType aabbGeomType
    = gprtGeomTypeCreate(context,
                        GPRT_AABBS,
                        sizeof(AABBGeomData),
                        aabbGeomVars,-1);
  gprtGeomTypeSetClosestHitProg(aabbGeomType,0,
                           module,"AABBClosestHit");
  gprtGeomTypeSetIntersectionProg(aabbGeomType,0,
                           module,"AABBIntersection");

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
    = gprtRayGenCreate(context,module,"AABBRayGen",
                      sizeof(RayGenData),
                      rayGenVars,-1);

  GPRTVarDecl missVars[]
    = {
    { "color0", GPRT_FLOAT3, GPRT_OFFSETOF(MissProgData,color0)},
    { "color1", GPRT_FLOAT3, GPRT_OFFSETOF(MissProgData,color1)},
    { /* sentinel to mark end of list */ }
  };
  GPRTMiss miss
    = gprtMissCreate(context,module,"miss",sizeof(MissProgData),
                        missVars,-1);

  GPRTVarDecl computeVars[] = {
    { "vertex", GPRT_BUFFER, GPRT_OFFSETOF(AABBBoundsData,vertex)},
    { "radius", GPRT_BUFFER, GPRT_OFFSETOF(AABBBoundsData,radius)},
    { "aabbs",  GPRT_BUFFER, GPRT_OFFSETOF(AABBBoundsData,aabbs)},
    { /* sentinel to mark end of list */ }
  };
  GPRTCompute boundsProgram
    = gprtComputeCreate(context,module,"AABBBounds",
                      sizeof(AABBBoundsData),
                      computeVars,-1);

  // Note, we'll need to call this again after creating our acceleration 
  // structures, as acceleration structures will introduce new shader 
  // binding table records to the pipeline.
  gprtBuildPipeline(context);

  // ------------------------------------------------------------------
  // aabb mesh
  // ------------------------------------------------------------------
  GPRTBuffer vertexBuffer
    = gprtDeviceBufferCreate(context,GPRT_FLOAT3,NUM_VERTICES,vertices);
  GPRTBuffer radiusBuffer
    = gprtDeviceBufferCreate(context,GPRT_FLOAT,NUM_VERTICES,radii);
  GPRTBuffer aabbPositionsBuffer
    = gprtDeviceBufferCreate(context,GPRT_FLOAT3,NUM_VERTICES * 2,nullptr);

  GPRTGeom aabbGeom
    = gprtGeomCreate(context,aabbGeomType);
  gprtAABBsSetPositions(aabbGeom, aabbPositionsBuffer,
                        NUM_VERTICES, 2 * sizeof(float3), 0);

  gprtGeomSetBuffer(aabbGeom,"vertex",vertexBuffer);
  gprtGeomSetBuffer(aabbGeom,"radius",radiusBuffer);
  gprtGeomSet3f(aabbGeom,"color",0,0,1);

  gprtComputeSetBuffer(boundsProgram, "vertex", vertexBuffer);
  gprtComputeSetBuffer(boundsProgram, "radius", radiusBuffer);
  gprtComputeSetBuffer(boundsProgram, "aabbs", aabbPositionsBuffer);

  // compute AABBs in parallel with a compute shader
  gprtBuildShaderBindingTable(context, GPRT_SBT_COMPUTE);
  gprtComputeLaunch1D(context,boundsProgram,NUM_VERTICES);

  GPRTAccel aabbAccel = gprtAABBAccelCreate(context,1,&aabbGeom);
  gprtAccelBuild(context, aabbAccel);

  // ------------------------------------------------------------------
  // the group/accel for that mesh
  // ------------------------------------------------------------------
  GPRTBuffer transformBuffer
    = gprtDeviceBufferCreate(context,GPRT_TRANSFORM,1,transform);
  GPRTAccel world = gprtInstanceAccelCreate(context,1,&aabbAccel);
  gprtInstanceAccelSetTransforms(world, transformBuffer);
  gprtAccelBuild(context, world);

  // ----------- set variables  ----------------------------
  gprtMissSet3f(miss,"color0",.1f,0.1f,0.1f);
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
  // now that everything is ready: launch it ....
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

  gprtBufferDestroy(vertexBuffer);
  gprtBufferDestroy(radiusBuffer);
  gprtBufferDestroy(aabbPositionsBuffer);
  gprtBufferDestroy(frameBuffer);
  gprtBufferDestroy(transformBuffer);
  gprtRayGenDestroy(rayGen);
  gprtMissDestroy(miss);
  gprtAccelDestroy(aabbAccel);
  gprtAccelDestroy(world);
  gprtGeomDestroy(aabbGeom);
  gprtGeomTypeDestroy(aabbGeomType);
  gprtModuleDestroy(module);
  gprtContextDestroy(context);

  LOG_OK("seems all went OK; app is done, this should be the last output ...");
}