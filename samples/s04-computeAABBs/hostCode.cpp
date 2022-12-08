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

#define LOG(message)                                            \
  std::cout << GPRT_TERMINAL_BLUE;                               \
  std::cout << "#gprt.sample(main): " << message << std::endl;   \
  std::cout << GPRT_TERMINAL_DEFAULT;
#define LOG_OK(message)                                         \
  std::cout << GPRT_TERMINAL_LIGHT_BLUE;                         \
  std::cout << "#gprt.sample(main): " << message << std::endl;   \
  std::cout << GPRT_TERMINAL_DEFAULT;

extern GPRTProgram s04_deviceCode;

// Vertices and radii that will be used to define spheres
const int NUM_VERTICES = 4;
float3 vertices[NUM_VERTICES] =
  {
    { 0.1f,0.2f,0.3f },
    { -1.f,-1.f,-1.f },
    { +1.f,-1.f,-1.f },
    { -1.f,+1.f,-1.f },
  };

float radii[NUM_VERTICES] =
  {
    1.f, .5f, .25f, .1f
  };

// initial image resolution
const int2 fbSize = {1400, 460};

// final image output
const char *outFileName = "s04-computeAABBs.png";

// Initial camera parameters
float3 lookFrom = {-4.f,-3.f,-2.f};
float3 lookAt = {0.f,0.f,0.f};
float3 lookUp = {0.f,1.f,0.f};
float cosFovy = 0.66f;

#include <iostream>
int main(int ac, char **av)
{
  // In this example, we'll use a compute shader to generate a set of
  // procedural axis aligned bounding boxes in parallel on the GPU. Each
  // AABB will contain a single sphere.
  LOG("gprt example '" << av[0] << "' starting up");

  // create a context on the first device:
  gprtRequestWindow(fbSize.x, fbSize.y, "S04 Compute AABB");
  GPRTContext context = gprtContextCreate(nullptr,1);
  GPRTModule module = gprtModuleCreate(context,s04_deviceCode);

  // ##################################################################
  // set up all the GPU kernels we want to run
  // ##################################################################

  // -------------------------------------------------------
  // declare geometry type
  // -------------------------------------------------------
  GPRTVarDecl sphereGeomVars[] = {
    { "vertex",  GPRT_BUFFER, GPRT_OFFSETOF(SphereGeomData,vertex)},
    { "radius",  GPRT_BUFFER, GPRT_OFFSETOF(SphereGeomData,radius)},
    { "color",  GPRT_FLOAT3, GPRT_OFFSETOF(SphereGeomData,color)},
    { /* sentinel to mark end of list */ }
  };
  GPRTGeomType sphereGeomType
    = gprtGeomTypeCreate(context,
                        GPRT_AABBS,
                        sizeof(SphereGeomData),
                        sphereGeomVars,-1);
  gprtGeomTypeSetClosestHitProg(sphereGeomType,0,
                           module,"SphereClosestHit");
  gprtGeomTypeSetIntersectionProg(sphereGeomType,0,
                           module,"SphereIntersection");

  // -------------------------------------------------------
  // set up sphere bounding box compute program
  // -------------------------------------------------------
  GPRTVarDecl computeVars[] = {
    { "vertex", GPRT_BUFFER, GPRT_OFFSETOF(SphereBoundsData,vertex)},
    { "radius", GPRT_BUFFER, GPRT_OFFSETOF(SphereBoundsData,radius)},
    { "aabbs",  GPRT_BUFFER, GPRT_OFFSETOF(SphereBoundsData,aabbs)},
    { /* sentinel to mark end of list */ }
  };
  GPRTCompute boundsProgram
    = gprtComputeCreate(context,module,"SphereBounds",
                      sizeof(SphereBoundsData),
                      computeVars,-1);

  // -------------------------------------------------------
  // set up miss
  // -------------------------------------------------------
  GPRTVarDecl missVars[]
    = {
    { "color0", GPRT_FLOAT3, GPRT_OFFSETOF(MissProgData,color0)},
    { "color1", GPRT_FLOAT3, GPRT_OFFSETOF(MissProgData,color1)},
    { /* sentinel to mark end of list */ }
  };
  GPRTMiss miss
    = gprtMissCreate(context,module,"miss",sizeof(MissProgData),
                        missVars,-1);

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
  GPRTRayGen rayGen
    = gprtRayGenCreate(context,module,"simpleRayGen",
                      sizeof(RayGenData),
                      rayGenVars,-1);

  // Note, we'll need to call this again after creating our acceleration 
  // structures, as acceleration structures will introduce new shader 
  // binding table records to the pipeline.
  gprtBuildPipeline(context);

  // ##################################################################
  // set the parameters for our compute kernel
  // ##################################################################

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
    = gprtGeomCreate(context,sphereGeomType);
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

  // Launch the compute kernel, which will populate our aabbPositionsBuffer
  gprtComputeLaunch1D(context,boundsProgram,NUM_VERTICES); 

  // Now that the aabbPositionsBuffer is filled, we can compute our AABB 
  // acceleration structure
  GPRTAccel aabbAccel = gprtAABBAccelCreate(context,1,&aabbGeom);
  gprtAccelBuild(context, aabbAccel);

  GPRTAccel world = gprtInstanceAccelCreate(context,1,&aabbAccel);
  gprtAccelBuild(context, world);

  // ##################################################################
  // set the parameters for the rest of our kernels
  // ##################################################################

  // Setup pixel frame buffer
  GPRTBuffer frameBuffer
    = gprtHostBufferCreate(context,GPRT_INT,fbSize.x*fbSize.y);
  gprtRayGenSetBuffer(rayGen,"fbPtr", frameBuffer);
  gprtRayGenSet2iv(rayGen,"fbSize", (int32_t*)&fbSize);
  gprtRayGenSetAccel(rayGen,"world", world);

  // Miss program checkerboard background colors
  gprtMissSet3f(miss,"color0",0.1f,0.1f,0.1f);
  gprtMissSet3f(miss,"color1",.0f,.0f,.0f);

  // ##################################################################
  // build the pipeline and shader binding table
  // ##################################################################

  // re-build the pipeline to account for newly introduced geometry
  gprtBuildPipeline(context);
  gprtBuildShaderBindingTable(context, GPRT_SBT_ALL);

// ##################################################################
  // now that everything is ready: launch it ....
  // ##################################################################

  LOG("launching ...");

  bool firstFrame = true;
  double xpos = 0.f, ypos = 0.f;
  double lastxpos, lastypos;
  do 
  {
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
    if (state == GPRT_PRESS || firstFrame)
    {
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
      gprtRayGenSet3fv    (rayGen,"camera.pos",   (float*)&camera_pos);
      gprtRayGenSet3fv    (rayGen,"camera.dir_00",(float*)&camera_d00);
      gprtRayGenSet3fv    (rayGen,"camera.dir_du",(float*)&camera_ddu);
      gprtRayGenSet3fv    (rayGen,"camera.dir_dv",(float*)&camera_ddv);

      gprtBuildShaderBindingTable(context, GPRT_SBT_RAYGEN);
    }

    // Calls the GPU raygen kernel function
    gprtRayGenLaunch2D(context,rayGen,fbSize.x,fbSize.y);
    
    // If a window exists, presents the framebuffer here to that window
    gprtBufferPresent(context, frameBuffer); 
  }
  // returns true if "X" pressed or if in "headless" mode
  while (!gprtWindowShouldClose(context));

  // Save final frame to an image
  LOG("done with launch, writing frame buffer to " << outFileName);
  gprtBufferSaveImage(frameBuffer, fbSize.x, fbSize.y, outFileName);
  LOG_OK("written rendered frame buffer to file "<<outFileName);

  // ##################################################################
  // and finally, clean up
  // ##################################################################

  LOG("cleaning up ...");

  gprtBufferDestroy(vertexBuffer);
  gprtBufferDestroy(radiusBuffer);
  gprtBufferDestroy(aabbPositionsBuffer);
  gprtBufferDestroy(frameBuffer);
  gprtRayGenDestroy(rayGen);
  gprtMissDestroy(miss);
  gprtAccelDestroy(aabbAccel);
  gprtAccelDestroy(world);
  gprtGeomDestroy(aabbGeom);
  gprtGeomTypeDestroy(sphereGeomType);
  gprtModuleDestroy(module);
  gprtContextDestroy(context);

  LOG_OK("seems all went OK; app is done, this should be the last output ...");
}
