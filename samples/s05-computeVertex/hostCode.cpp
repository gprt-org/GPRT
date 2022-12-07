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

extern GPRTProgram int05_deviceCode;

const int GRID_SIDE_LENGTH = 100;
float transform[3][4] = 
  {
    1.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 1.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 1.0f, 0.0f
  };

// initial image resolution
const int2 fbSize = {1400, 460};
GLuint fbTexture {0};

float3 lookFrom = {1.5f,-1.5f,1.f};
float3 lookAt = {0.5f,0.5f,0.f};
float3 lookUp = {0.f,0.f,1.f};
float cosFovy = 0.45f;

#include <iostream>
int main(int ac, char **av)
{
  // create a context on the first device:
  GPRTContext context = gprtContextCreate(nullptr,1);
  GPRTModule module = gprtModuleCreate(context,int05_deviceCode);

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
  
  GPRTCompute vertexProgram
    = gprtComputeCreate(context,module,"Vertex",
                      sizeof(TrianglesGeomData),
                      trianglesGeomVars,-1);

  gprtBuildPipeline(context);

  // ------------------------------------------------------------------
  // aabb mesh
  // ------------------------------------------------------------------
  unsigned int numTriangles = 2 * GRID_SIDE_LENGTH * GRID_SIDE_LENGTH;
  unsigned int numVertices = 3 * numTriangles;
  GPRTBuffer vertexBuffer
    = gprtDeviceBufferCreate(context,GPRT_FLOAT3,numVertices,nullptr);
  GPRTBuffer indexBuffer
    = gprtDeviceBufferCreate(context,GPRT_UINT3,numTriangles,nullptr);
  
  GPRTGeom trianglesGeom
    = gprtGeomCreate(context,trianglesGeomType);
  gprtTrianglesSetVertices(trianglesGeom, vertexBuffer, 
                           numVertices, sizeof(float3), 0);
  gprtTrianglesSetIndices(trianglesGeom, indexBuffer, 
                           numTriangles, sizeof(uint3), 0);

  gprtGeomSetBuffer(trianglesGeom,"vertex",vertexBuffer);
  gprtGeomSetBuffer(trianglesGeom,"index",indexBuffer);
  gprtGeomSet1f(trianglesGeom,"now",0.0f);
  gprtGeomSet1ui(trianglesGeom,"gridSize",GRID_SIDE_LENGTH);

  gprtComputeSetBuffer(vertexProgram,"vertex",vertexBuffer);
  gprtComputeSetBuffer(vertexProgram,"index",indexBuffer);
  gprtComputeSet1f(vertexProgram,"now",0.0f);
  gprtComputeSet1ui(vertexProgram,"gridSize",GRID_SIDE_LENGTH);
   
  // compute triangles in parallel with a compute shader
  gprtBuildShaderBindingTable(context, GPRT_SBT_COMPUTE);
  gprtComputeLaunch1D(context,vertexProgram,numTriangles);

  GPRTAccel trianglesAccel = gprtTrianglesAccelCreate(context,1,&trianglesGeom);
  gprtAccelBuild(context, trianglesAccel);

  // ------------------------------------------------------------------
  // the group/accel for that mesh
  // ------------------------------------------------------------------
  GPRTBuffer transformBuffer
    = gprtDeviceBufferCreate(context,GPRT_TRANSFORM,1,transform);
  GPRTAccel world = gprtInstanceAccelCreate(context,1,&trianglesAccel);
  gprtInstanceAccelSet3x4Transforms(world, transformBuffer);
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
      float deltaAngleY = -(M_PI / fbSize.y);
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

    // Now, trace rays
    gprtRayGenLaunch2D(context,rayGen,fbSize.x,fbSize.y);

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
    glOrtho(0.f, (float)fbSize.x, 0.0, (float)fbSize.y, -1.f, 1.f);

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
  gprtBufferDestroy(frameBuffer);
  gprtBufferDestroy(transformBuffer);
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
