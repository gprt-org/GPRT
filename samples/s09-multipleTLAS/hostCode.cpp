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

// for generating meshes
#include <generator.hpp>
using namespace generator;

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

extern GPRTProgram int09_deviceCode;

template <typename T>
struct Mesh {
  std::vector<float3> vertices;
  std::vector<uint3> indices;
  GPRTBuffer vertexBuffer;
  GPRTBuffer indexBuffer;
  GPRTGeom geometry;
  GPRTAccel accel;
  float4x4 transform;

  Mesh() {};
  Mesh(GPRTContext context, GPRTGeomType geomType, T generator, float3 color)
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
    gprtGeomSet3f(geometry,"color",color.x, color.y, color.z);
    accel = gprtTrianglesAccelCreate(context,1,&geometry);
    gprtAccelBuild(context, accel);

    transform = identity;
  };

  void cleanupMesh() {
    gprtAccelDestroy(accel);
    gprtGeomDestroy(geometry);
    gprtBufferDestroy(vertexBuffer);
    gprtBufferDestroy(indexBuffer);
  };
};

struct TetrahedralMesh {
  const int NUM_VERTICES = 4;
  float3 vertices[8] =
    {
      { cosf(2.0944 * 0) * .8f, sinf(2.0944 * 0) * .8f,-0.25f },
      { cosf(2.0944 * 1) * .8f, sinf(2.0944 * 1) * .8f,-0.25f },
      { cosf(2.0944 * 2) * .8f, sinf(2.0944 * 2) * .8f,-0.25f },
      { 0.f,0.f,1.f },
    };

  const int NUM_INDICES = 1;
  int4 indices[1] =
    {
      { 0,1,2,3}
    };

  float3 aabbs[2] = 
    {
      {-1.0, -1.0, -1.0}, {1.0, 1.0, 1.0}
    };

  float transform[3][4] =
    {
      1.0f, 0.0f, 0.0f, 0.0f,
      0.0f, 1.0f, 0.0f, 0.0f,
      0.0f, 0.0f, 1.0f, 0.0f
    };
} tetrahedralMesh;

// initial image resolution
const int2 fbSize = {1400, 460};
GLuint fbTexture {0};

float3 lookFrom = {4.f,0.0f,0.2f};
float3 lookAt = {0.f,0.f,0.2f};
float3 lookUp = {0.f,0.f,1.f};
float cosFovy = 0.3f;

#include <iostream>
int main(int ac, char **av)
{
  LOG("gprt example '" << av[0] << "' starting up");

  // create a context on the first device:
  GPRTContext context = gprtContextCreate(nullptr,1);
  GPRTModule module = gprtModuleCreate(context,int09_deviceCode);

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
                           module,"TriangleMesh");

  GPRTVarDecl tetrahedraGeomVars[] = {
    { "index",  GPRT_BUFFER, GPRT_OFFSETOF(TetrahedraGeomData,index)},
    { "vertex", GPRT_BUFFER, GPRT_OFFSETOF(TetrahedraGeomData,vertex)},
    { /* sentinel to mark end of list */ }
  };
  GPRTGeomType tetrahedraGeomType
    = gprtGeomTypeCreate(context,
                        GPRT_AABBS,
                        sizeof(TrianglesGeomData),
                        tetrahedraGeomVars,-1);
  gprtGeomTypeSetIntersectionProg(tetrahedraGeomType,0,
                           module,"TetrahedralMesh");
  gprtGeomTypeSetClosestHitProg(tetrahedraGeomType,0,
                           module,"TetrahedralMesh");

  // ##################################################################
  // set up all the *GEOMS* we want to run that code on
  // ##################################################################

  LOG("building geometries ...");

  // ------------------------------------------------------------------
  // Meshes
  // ------------------------------------------------------------------
  Mesh<TeapotMesh> teapotMesh(context, trianglesGeomType, TeapotMesh{16}, float3(1,0,0));
  teapotMesh.transform = mul(scaling_matrix(float3(.1,.1,.1)), teapotMesh.transform);
  teapotMesh.transform = mul(rotation_matrix(rotation_quat(float3(0,0,1), 3.14f * .5f)), teapotMesh.transform);
  GPRTBuffer triangleTransformBuffer
    = gprtDeviceBufferCreate(context,GPRT_TRANSFORM_4X4, 1, &teapotMesh.transform);
  GPRTAccel trianglesTLAS = gprtInstanceAccelCreate(context,1,&teapotMesh.accel);
  gprtInstanceAccelSet4x4Transforms(trianglesTLAS, triangleTransformBuffer);
  gprtAccelBuild(context, trianglesTLAS);

  GPRTBuffer tetrahedraVertexBuffer
    = gprtHostBufferCreate(context,GPRT_FLOAT3,
        tetrahedralMesh.NUM_VERTICES,tetrahedralMesh.vertices);
  GPRTBuffer tetrahedraIndexBuffer
    = gprtDeviceBufferCreate(context,GPRT_INT4,
        tetrahedralMesh.NUM_INDICES,tetrahedralMesh.indices);
  GPRTBuffer tetrahedraAABBBuffer
    = gprtHostBufferCreate(context,GPRT_FLOAT3,
        tetrahedralMesh.NUM_INDICES * 2,tetrahedralMesh.aabbs);
  GPRTBuffer tetrahedraTransformBuffer
    = gprtDeviceBufferCreate(context,GPRT_TRANSFORM,
        1,tetrahedralMesh.transform);

  GPRTBuffer frameBuffer
    = gprtHostBufferCreate(context,GPRT_INT,fbSize.x*fbSize.y);

  GPRTGeom tetrahedraGeom
    = gprtGeomCreate(context,tetrahedraGeomType);
  gprtAABBsSetPositions(tetrahedraGeom,tetrahedraAABBBuffer,
                           tetrahedralMesh.NUM_INDICES,sizeof(float3) * 2,0);
  gprtGeomSetBuffer(tetrahedraGeom,"vertex",tetrahedraVertexBuffer);
  gprtGeomSetBuffer(tetrahedraGeom,"index",tetrahedraIndexBuffer);

  GPRTAccel tetrahedraAccel = gprtAABBAccelCreate(context,1,&tetrahedraGeom);
  gprtAccelBuild(context, tetrahedraAccel);
  GPRTAccel tetrahedraTLAS = gprtInstanceAccelCreate(context,1,&tetrahedraAccel);
  gprtInstanceAccelSet3x4Transforms(tetrahedraTLAS, tetrahedraTransformBuffer);
  gprtAccelBuild(context, tetrahedraTLAS);

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
    { "meshes",        GPRT_ACCEL,  GPRT_OFFSETOF(RayGenData,meshes)},
    { "cells",         GPRT_ACCEL,  GPRT_OFFSETOF(RayGenData,cells)},
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

  // ----------- set variables  ----------------------------
  gprtRayGenSetBuffer(rayGen,"fbPtr", frameBuffer);
  gprtRayGenSet2iv(rayGen,"fbSize", (int32_t*)&fbSize);
  gprtRayGenSetAccel(rayGen,"meshes", trianglesTLAS);
  gprtRayGenSetAccel(rayGen,"cells", tetrahedraTLAS);

  // ##################################################################
  // build *SBT* required to trace the groups
  // ##################################################################
  gprtBuildPipeline(context);
  gprtBuildShaderBindingTable(context);

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
    "Int08 Multiple TLAS", NULL, NULL);
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

      gprtBuildShaderBindingTable(context);
    }

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

  teapotMesh.cleanupMesh();
  gprtBufferDestroy(triangleTransformBuffer);
  gprtBufferDestroy(tetrahedraVertexBuffer);
  gprtBufferDestroy(tetrahedraIndexBuffer);
  gprtBufferDestroy(tetrahedraAABBBuffer);
  gprtBufferDestroy(tetrahedraTransformBuffer);
  gprtBufferDestroy(frameBuffer);
  gprtRayGenDestroy(rayGen);
  gprtMissDestroy(miss);
  gprtAccelDestroy(tetrahedraAccel);
  gprtAccelDestroy(trianglesTLAS);
  gprtAccelDestroy(tetrahedraTLAS);
  gprtGeomTypeDestroy(trianglesGeomType);
  gprtGeomDestroy(tetrahedraGeom);
  gprtGeomTypeDestroy(tetrahedraGeomType);
  gprtModuleDestroy(module);
  gprtContextDestroy(context);

  LOG_OK("seems all went OK; app is done, this should be the last output ...");
}
