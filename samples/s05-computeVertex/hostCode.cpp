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

// library for windowing
#include <GLFW/glfw3.h>

#define LOG(message)                                            \
  std::cout << GPRT_TERMINAL_BLUE;                               \
  std::cout << "#gprt.sample(main): " << message << std::endl;   \
  std::cout << GPRT_TERMINAL_DEFAULT;
#define LOG_OK(message)                                         \
  std::cout << GPRT_TERMINAL_LIGHT_BLUE;                         \
  std::cout << "#gprt.sample(main): " << message << std::endl;   \
  std::cout << GPRT_TERMINAL_DEFAULT;

const int GRID_SIDE_LENGTH = 100;

extern GPRTProgram s05_deviceCode;

// initial image resolution
const int2 fbSize = {1400, 460};

const char *outFileName = "s04-computeAABBs.png";

float3 lookFrom = {1.5f,-1.5f,1.f};
float3 lookAt = {0.5f,0.5f,0.f};
float3 lookUp = {0.f,0.f,-1.f};
float cosFovy = 0.45f;

#include <iostream>
int main(int ac, char **av)
{
  // create a context on the first device:
  gprtRequestWindow(fbSize.x, fbSize.y, "S05 Compute Vertex");
  GPRTContext context = gprtContextCreate(nullptr,1);
  GPRTModule module = gprtModuleCreate(context,s05_deviceCode);

  // ##################################################################
  // set up all the GPU kernels we want to run
  // ##################################################################

  // -------------------------------------------------------
  // Setup programs and geometry types
  // -------------------------------------------------------
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
                        trianglesGeomVars,-1);
  gprtGeomTypeSetClosestHitProg(trianglesGeomType,0,
                           module,"ClosestHit");
  
  // -------------------------------------------------------
  // set up vertex program to animate vertices
  // -------------------------------------------------------
  GPRTCompute vertexProgram
    = gprtComputeCreate(context,module,"Vertex",
                      sizeof(TrianglesGeomData),
                      trianglesGeomVars,-1);
  
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
    = gprtRayGenCreate(context,module,"RayGen",
                      sizeof(RayGenData),
                      rayGenVars,-1);

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

  // Note, we'll need to call this again after creating our acceleration 
  // structures, as acceleration structures will introduce new shader 
  // binding table records to the pipeline.
  gprtBuildPipeline(context);

  // ##################################################################
  // set the parameters for our triangle mesh and compute kernel
  // ##################################################################
  
  // Note, the vertex and index buffers are empty here. We will fill 
  // them in on the device by using our vertex program. 
  unsigned int numTriangles = 2 * GRID_SIDE_LENGTH * GRID_SIDE_LENGTH;
  unsigned int numVertices = 3 * numTriangles;
  GPRTBuffer vertexBuffer
    = gprtDeviceBufferCreate(context,GPRT_FLOAT3,numVertices,nullptr);
  GPRTBuffer indexBuffer
    = gprtDeviceBufferCreate(context,GPRT_UINT3,numTriangles,nullptr);
  
  GPRTGeom trianglesGeom
    = gprtGeomCreate(context,trianglesGeomType);

  // It is _okay_ to give our triangles geometry unpopulated buffers here
  // for the vertices and indices, so long as they're filled in before 
  // we go to build our acceleration structure.
  gprtTrianglesSetVertices(trianglesGeom, vertexBuffer, 
                           numVertices, sizeof(float3), 0);
  gprtTrianglesSetIndices(trianglesGeom, indexBuffer, 
                           numTriangles, sizeof(uint3), 0);

  // Parameters for the geometry when a ray hits it.
  gprtGeomSetBuffer(trianglesGeom,"vertex",vertexBuffer);
  gprtGeomSetBuffer(trianglesGeom,"index",indexBuffer);
  gprtGeomSet1f(trianglesGeom,"now",0.0f);
  gprtGeomSet1ui(trianglesGeom,"gridSize",GRID_SIDE_LENGTH);

  // Parameters for our vertex program that'll animate our vertices
  gprtComputeSetBuffer(vertexProgram,"vertex",vertexBuffer);
  gprtComputeSetBuffer(vertexProgram,"index",indexBuffer);
  gprtComputeSet1f(vertexProgram,"now",0.0f);
  gprtComputeSet1ui(vertexProgram,"gridSize",GRID_SIDE_LENGTH);
  
  // Build the shader binding table to upload parameters to the device
  gprtBuildShaderBindingTable(context, GPRT_SBT_COMPUTE);
   
  // Now, compute triangles in parallel with a vertex compute shader
  gprtComputeLaunch1D(context,vertexProgram,numTriangles);

  // Now that our vertex buffer and index buffer are filled, we can compute
  // our triangles acceleration structure.
  GPRTAccel trianglesAccel = gprtTrianglesAccelCreate(context,1,&trianglesGeom);
  gprtAccelBuild(context, trianglesAccel);

  GPRTAccel world = gprtInstanceAccelCreate(context,1,&trianglesAccel);
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
  // build *SBT* required to trace the groups
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

    // update time to move primitives. then, rebuild accel.
    gprtComputeSet1f(vertexProgram, "now", float(glfwGetTime()));
    gprtBuildShaderBindingTable(context, GPRT_SBT_COMPUTE);
    gprtComputeLaunch1D(context,vertexProgram,numTriangles);
    gprtAccelBuild(context, trianglesAccel);
    gprtAccelBuild(context, world);
    gprtRayGenSetAccel(rayGen, "world", world);
    gprtBuildShaderBindingTable(context, GPRT_SBT_HITGROUP);
    
    // Note! we don't need to rebuild the pipeline here, since no geometry was
    // made or destroyed, only updated.

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
  gprtBufferDestroy(indexBuffer);
  gprtBufferDestroy(frameBuffer);
  gprtRayGenDestroy(rayGen);
  gprtMissDestroy(miss);
  gprtComputeDestroy(vertexProgram);
  gprtAccelDestroy(trianglesAccel);
  gprtAccelDestroy(world);
  gprtGeomDestroy(trianglesGeom);
  gprtGeomTypeDestroy(trianglesGeomType);
  gprtModuleDestroy(module);
  gprtContextDestroy(context);

  LOG_OK("seems all went OK; app is done, this should be the last output ...");
}
