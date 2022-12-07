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
#define NOMINMAX
#include <GLFW/glfw3.h>

#define LOG(message)                                            \
  std::cout << GPRT_TERMINAL_BLUE;                               \
  std::cout << "#gprt.sample(main): " << message << std::endl;   \
  std::cout << GPRT_TERMINAL_DEFAULT;
#define LOG_OK(message)                                         \
  std::cout << GPRT_TERMINAL_LIGHT_BLUE;                         \
  std::cout << "#gprt.sample(main): " << message << std::endl;   \
  std::cout << GPRT_TERMINAL_DEFAULT;

extern GPRTProgram int10_deviceCode;


/* forward declarations to double precision cube. 
  See gprt_data/double_cube.cpp for details */
extern uint32_t double_cube_indices[];
extern double double_cube_vertices[];
extern int double_cube_num_indices;
extern int double_cube_num_vertices;

float transform[3][4] = 
  {
    1.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 1.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 1.0f, 0.0f
  };

// initial image resolution
const int2 fbSize = {1400, 460};
GLuint fbTexture {0};

#include <iostream>
int main(int ac, char **av)
{
  // create a context on the first device:
  GPRTContext context = gprtContextCreate(nullptr,1);
  GPRTModule module = gprtModuleCreate(context,int10_deviceCode);

  // -------------------------------------------------------
  // Setup programs and geometry types
  // -------------------------------------------------------
  GPRTVarDecl DPTriangleVars[] = {
    { "vertex",  GPRT_BUFFER, GPRT_OFFSETOF(DPTriangleData,vertex)},
    { "index" ,  GPRT_BUFFER, GPRT_OFFSETOF(DPTriangleData,index)},
    { "aabbs" ,  GPRT_BUFFER, GPRT_OFFSETOF(DPTriangleData,aabbs)},
    { "dpRays" , GPRT_BUFFER, GPRT_OFFSETOF(DPTriangleData,dpRays)},
    { "fbSize",  GPRT_INT2,   GPRT_OFFSETOF(DPTriangleData,fbSize)},
    { /* sentinel to mark end of list */ }
  };
  GPRTGeomType DPTriangleType
    = gprtGeomTypeCreate(context,
                        GPRT_AABBS,
                        sizeof(DPTriangleVars),
                        DPTriangleVars);
  GPRTCompute DPTriangleBoundsProgram
    = gprtComputeCreate(context,module,"DPTriangle",
                        sizeof(DPTriangleVars),
                        DPTriangleVars);
  gprtGeomTypeSetClosestHitProg(DPTriangleType,0,
                           module,"DPTriangle");
  gprtGeomTypeSetIntersectionProg(DPTriangleType,0,
                           module,"DPTrianglePlucker");

  GPRTVarDecl rayGenVars[] = {
    { "fbSize",        GPRT_INT2,   GPRT_OFFSETOF(RayGenData,fbSize)},
    { "fbPtr",         GPRT_BUFFER, GPRT_OFFSETOF(RayGenData,fbPtr)},
    { "dpRays",        GPRT_BUFFER, GPRT_OFFSETOF(RayGenData,dpRays)},
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

  gprtBuildPipeline(context);

  // ------------------------------------------------------------------
  // aabb mesh
  // ------------------------------------------------------------------
  GPRTBuffer vertexBuffer
    = gprtDeviceBufferCreate(context,GPRT_DOUBLE3,double_cube_num_vertices,double_cube_vertices);
  GPRTBuffer indexBuffer
    = gprtDeviceBufferCreate(context,GPRT_INT3,double_cube_num_indices,double_cube_indices);
  GPRTBuffer aabbPositionsBuffer
    = gprtDeviceBufferCreate(context,GPRT_FLOAT3,double_cube_num_indices * 2,nullptr);

  GPRTGeom dpCubeGeom
    = gprtGeomCreate(context,DPTriangleType);
  gprtAABBsSetPositions(dpCubeGeom, aabbPositionsBuffer, 
                        double_cube_num_indices, 2 * sizeof(float3), 0);

  gprtGeomSetBuffer(dpCubeGeom,"vertex",vertexBuffer);
  gprtGeomSetBuffer(dpCubeGeom,"index",indexBuffer);
  gprtGeomSetBuffer(dpCubeGeom, "aabbs", aabbPositionsBuffer);

  gprtComputeSetBuffer(DPTriangleBoundsProgram, "vertex", vertexBuffer);
  gprtComputeSetBuffer(DPTriangleBoundsProgram, "index", indexBuffer);
  gprtComputeSetBuffer(DPTriangleBoundsProgram, "aabbs", aabbPositionsBuffer);
  
  // compute AABBs in parallel with a compute shader
  gprtBuildShaderBindingTable(context, GPRT_SBT_COMPUTE);
  gprtComputeLaunch1D(context,DPTriangleBoundsProgram,double_cube_num_indices);

  GPRTAccel aabbAccel = gprtAABBAccelCreate(context,1,&dpCubeGeom);
  gprtAccelBuild(context, aabbAccel);

  // compute centroid to look at
  double3 aabbmin = double3(double_cube_vertices[0],double_cube_vertices[1],double_cube_vertices[2]);
  double3 aabbmax = aabbmin;
  for (uint32_t i = 1; i < double_cube_num_vertices; ++i) {
    aabbmin = linalg::min(aabbmin, double3(double_cube_vertices[i * 3 + 0],
                                           double_cube_vertices[i * 3 + 1],
                                           double_cube_vertices[i * 3 + 2]));
    aabbmax = linalg::max(aabbmax, double3(double_cube_vertices[i * 3 + 0],
                                           double_cube_vertices[i * 3 + 1],
                                           double_cube_vertices[i * 3 + 2]));
  }
  double3 aabbCentroid = aabbmin + (aabbmax - aabbmin) * 0.5;

  float3 lookFrom = float3(float(aabbCentroid.x), float(aabbCentroid.y)  - 50.f, float(aabbCentroid.z));
  float3 lookAt = float3(float(aabbCentroid.x),float(aabbCentroid.y),float(aabbCentroid.z));
  float3 lookUp = {0.f,0.f,-1.f};
  float cosFovy = 0.66f;

  // ------------------------------------------------------------------
  // the group/accel for that mesh
  // ------------------------------------------------------------------
  GPRTBuffer transformBuffer
    = gprtDeviceBufferCreate(context,GPRT_TRANSFORM,1,transform);
  GPRTAccel world = gprtInstanceAccelCreate(context,1,&aabbAccel);
  gprtInstanceAccelSet3x4Transforms(world, transformBuffer);
  gprtAccelBuild(context, world);

  // ----------- set variables  ----------------------------
  gprtMissSet3f(miss,"color0",0.1f,0.1f,0.1f);
  gprtMissSet3f(miss,"color1",.0f,.0f,.0f);

  // ----------- set raygen variables  ----------------------------
  GPRTBuffer frameBuffer
    = gprtHostBufferCreate(context,GPRT_INT,fbSize.x*fbSize.y);
  
  // need this to communicate double precision rays to intersection program
  // ray origin xyz + tmin, then ray direction xyz + tmax
  GPRTBuffer doubleRayBuffer
    = gprtDeviceBufferCreate(context,GPRT_DOUBLE,fbSize.x*fbSize.y*8);
  gprtRayGenSetBuffer(rayGen,"fbPtr", frameBuffer);
  gprtRayGenSetBuffer(rayGen,"dpRays", doubleRayBuffer);
  gprtRayGenSet2iv(rayGen,"fbSize", (int32_t*)&fbSize);
  gprtRayGenSetAccel(rayGen,"world", world);
  
  // Also set on geometry for intersection program
  gprtGeomSetBuffer(dpCubeGeom,"dpRays", doubleRayBuffer);
  gprtGeomSet2iv(dpCubeGeom,"fbSize", (int32_t*)&fbSize);

  // ##################################################################
  // build *SBT* required to trace the groups
  // ##################################################################
  
  // re-build the pipeline to account for newly introduced geometry
  gprtBuildPipeline(context);
  gprtBuildShaderBindingTable(context, GPRT_SBT_ALL);

  // ##################################################################
  // create a window we can use to display and interact with the image
  // ##################################################################
  if (!glfwInit())
    // Initialization failed
    throw std::runtime_error("Can't initialize GLFW");

  auto error_callback = [](int error, const char* description)
  {
    fprintf(stderr, "Error: %s\n", description);
  };
  glfwSetErrorCallback(error_callback);

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
  glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);
  GLFWwindow* window = glfwCreateWindow(fbSize.x, fbSize.y, 
    "Int02 Simple AABBs", NULL, NULL);
  if (!window) throw std::runtime_error("Window or OpenGL context creation failed");
  glfwMakeContextCurrent(window);

  // ##################################################################
  // now that everything is ready: launch it ....
  // ##################################################################

  LOG("launching ...");

  bool firstFrame = true;
  double xpos = 0.f, ypos = 0.f;
  double lastxpos, lastypos;
  while (!glfwWindowShouldClose(window))
  {
    float speed = .001f;
    lastxpos = xpos;
    lastypos = ypos;
    glfwGetCursorPos(window, &xpos, &ypos);
    if (firstFrame) {
      lastxpos = xpos;
      lastypos = ypos;
    }
    int state = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT);

    // If we click the mouse, we should rotate the camera
    if (state == GLFW_PRESS || firstFrame)
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

    // Now, trace rays
    gprtBeginProfile(context);
    gprtRayGenLaunch2D(context,rayGen,fbSize.x,fbSize.y);
    float ms = gprtEndProfile(context) * 0.000001;
    std::cout<<"time " << ms << " ms" << std::endl;

    // Render results to screen
    void* pixels = gprtBufferGetPointer(frameBuffer);
    if (fbTexture == 0)
      glGenTextures(1, &fbTexture);
    
    glBindTexture(GL_TEXTURE_2D, fbTexture);
    GLenum texFormat = GL_RGBA;
    GLenum texelType = GL_UNSIGNED_BYTE;
    glTexImage2D(GL_TEXTURE_2D, 0, texFormat, fbSize.x, fbSize.y, 0, GL_RGBA,
                  texelType, pixels);

    glDisable(GL_LIGHTING);
    glColor3f(1, 1, 1);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, fbTexture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    
    glDisable(GL_DEPTH_TEST);

    glViewport(0, 0, fbSize.x, fbSize.y);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0.f, (float)fbSize.x, (float)fbSize.y, 0.f, -1.f, 1.f);

    glBegin(GL_QUADS);
    {
      glTexCoord2f(0.f, 0.f);
      glVertex3f(0.f, 0.f, 0.f);
    
      glTexCoord2f(0.f, 1.f);
      glVertex3f(0.f, (float)fbSize.y, 0.f);
    
      glTexCoord2f(1.f, 1.f);
      glVertex3f((float)fbSize.x, (float)fbSize.y, 0.f);
    
      glTexCoord2f(1.f, 0.f);
      glVertex3f((float)fbSize.x, 0.f, 0.f);
    }
    glEnd();
    
    glfwSwapBuffers(window);
    glfwPollEvents();
    
  }

  // ##################################################################
  // and finally, clean up
  // ##################################################################

  LOG("cleaning up ...");

  glfwDestroyWindow(window);
  glfwTerminate();

  gprtBufferDestroy(vertexBuffer);
  gprtBufferDestroy(indexBuffer);
  gprtBufferDestroy(aabbPositionsBuffer);
  gprtBufferDestroy(frameBuffer);
  gprtBufferDestroy(doubleRayBuffer);  
  gprtBufferDestroy(transformBuffer);
  gprtRayGenDestroy(rayGen);
  gprtMissDestroy(miss);
  gprtComputeDestroy(DPTriangleBoundsProgram);
  gprtAccelDestroy(aabbAccel);
  gprtAccelDestroy(world);
  gprtGeomDestroy(dpCubeGeom);
  gprtGeomTypeDestroy(DPTriangleType);
  gprtModuleDestroy(module);
  gprtContextDestroy(context);

  LOG_OK("seems all went OK; app is done, this should be the last output ...");
}
