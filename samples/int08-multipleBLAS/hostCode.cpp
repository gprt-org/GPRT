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

extern GPRTProgram int08_deviceCode;

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

template <typename T>
struct Spheres {
  std::vector<float3> vertices;
  std::vector<float> radius;
  std::vector<float3> aabbs;
  GPRTBuffer vertexBuffer;
  GPRTBuffer radiusBuffer;
  GPRTBuffer aabbBuffer;
  GPRTGeom geometry;

  Spheres() {};
  Spheres(GPRTContext context, GPRTGeomType geomType, T generator, float r, float3 color, float4x4 transform)
  {
    auto vertGenerator = generator.vertices();
    
    while (!vertGenerator.done()) {
      auto vertex = vertGenerator.generate();
      auto position = vertex.position;
      float4 p = mul(transform, float4(vertex.position[0], vertex.position[1], vertex.position[2], 1.0));
      vertices.push_back(p.xyz());
      radius.push_back(r);
      aabbs.push_back((p - r).xyz());
      aabbs.push_back((p + r).xyz());
      vertGenerator.next();
    }

    vertexBuffer
      = gprtDeviceBufferCreate(context,GPRT_FLOAT3,vertices.size(),vertices.data());
    radiusBuffer
      = gprtDeviceBufferCreate(context,GPRT_FLOAT,radius.size(),radius.data());
    aabbBuffer
      = gprtDeviceBufferCreate(context,GPRT_FLOAT3,aabbs.size(),aabbs.data());
    geometry
      = gprtGeomCreate(context,geomType);
    gprtAABBsSetPositions(geometry, aabbBuffer, aabbs.size() / 2, 6 * sizeof(float), 0);
    
    gprtGeomSetBuffer(geometry,"vertex",vertexBuffer);
    gprtGeomSetBuffer(geometry,"radius",radiusBuffer);
    gprtGeomSet3f(geometry,"color",color.x, color.y, color.z);
  };

  void cleanup() {
    gprtGeomDestroy(geometry);
    gprtBufferDestroy(vertexBuffer);
    gprtBufferDestroy(radiusBuffer);
    gprtBufferDestroy(aabbBuffer);
  };
};

// initial image resolution
const int2 fbSize = {800,600};
GLuint fbTexture {0};

float3 lookFrom = {10.f,10.0f,10.f};
float3 lookAt = {0.f,0.f,1.f};
float3 lookUp = {0.f,0.f,1.f};
float cosFovy = 0.4f;

#include <iostream>
int main(int ac, char **av)
{
  LOG("gprt example '" << av[0] << "' starting up");

  // create a context on the first device:
  GPRTContext context = gprtContextCreate(nullptr,1);
  GPRTModule module = gprtModuleCreate(context,int08_deviceCode);

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
                           module,"trianglesClosestHit");

  GPRTVarDecl spheresGeomVars[] = {
    { "vertex", GPRT_BUFFER, GPRT_OFFSETOF(SpheresGeomData,vertex)},
    { "radius", GPRT_BUFFER, GPRT_OFFSETOF(SpheresGeomData,radius)},
    { "color",  GPRT_FLOAT3, GPRT_OFFSETOF(SpheresGeomData,color)},
    { /* sentinel to mark end of list */ }
  };
  GPRTGeomType spheresGeomType
    = gprtGeomTypeCreate(context,
                        GPRT_AABBS,
                        sizeof(SpheresGeomData),
                        spheresGeomVars,-1);
  gprtGeomTypeSetClosestHitProg(spheresGeomType,0,
                           module,"spheresClosestHit");
  gprtGeomTypeSetIntersectionProg(spheresGeomType,0,
                           module,"spheresIntersection");

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

  Mesh<DodecahedronMesh> dodecahedronMesh(context, trianglesGeomType, DodecahedronMesh{}, float3(1,0.,0.0), 
    translation_matrix(float3(2*sin(2*M_PI*.33), 2*cos(2*M_PI*.33), 1.5f)));
  Mesh<IcosahedronMesh> icosahedronMesh(context, trianglesGeomType, IcosahedronMesh{}, float3(0,1.0,0.0), 
    translation_matrix(float3(2*sin(2*M_PI*.66), 2*cos(2*M_PI*.66), 1.5f)));
  Mesh<IcoSphereMesh> icosphereMesh(context, trianglesGeomType, IcoSphereMesh{}, float3(0,0.0,1.0), 
    translation_matrix(float3(2*sin(2*M_PI*1.0), 2*cos(2*M_PI*1.0), 1.5f)));
  
  Spheres<HelixPath> helixSpheresA(context, spheresGeomType, HelixPath{1.0, 1.0, 8}, 0.4, float3(1.0, 0.0, 0.0), 
    mul(translation_matrix(float3(0.0f, 0.0f, 1.0f)), scaling_matrix(float3(4.0f, 4.0f, 4.0f)) ));
  Spheres<HelixPath> helixSpheresB(context, spheresGeomType, HelixPath{1.0, 1.0, 16}, 0.3, float3(0.0, 1.0, 0.0), 
    mul(translation_matrix(float3(0.0f, 0.0f, 1.0f)), scaling_matrix(float3(5.0f, 5.0f, 3.0f)) ));
  Spheres<HelixPath> helixSpheresC(context, spheresGeomType, HelixPath{1.0, 1.0, 32}, 0.2, float3(0.0, 0.0, 1.0), 
    mul(translation_matrix(float3(0.0f, 0.0f, 1.0f)), scaling_matrix(float3(6.0f, 6.0f, 2.0f)) ));

  std::vector<GPRTGeom> triangleGeoms = {dodecahedronMesh.geometry, icosahedronMesh.geometry, icosphereMesh.geometry};
  std::vector<GPRTGeom> pointGeoms = {helixSpheresA.geometry, helixSpheresB.geometry, helixSpheresC.geometry};
  GPRTAccel trianglesBLAS = gprtTrianglesAccelCreate(context,triangleGeoms.size(),triangleGeoms.data());
  GPRTAccel spheresBLAS = gprtAABBAccelCreate(context,pointGeoms.size(),pointGeoms.data());
  std::vector<GPRTAccel> blas = {trianglesBLAS, spheresBLAS};
  GPRTAccel TLAS = gprtInstanceAccelCreate(context,blas.size(),blas.data());
  gprtAccelBuild(context, trianglesBLAS);
  gprtAccelBuild(context, spheresBLAS);
  gprtAccelBuild(context, TLAS);

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
  gprtRayGenSetAccel(rayGen,"world", TLAS);

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
    "Int08 Multiple BLAS in one TLAS", NULL, NULL);
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

  dodecahedronMesh.cleanup();
  icosahedronMesh.cleanup();
  icosphereMesh.cleanup();
  helixSpheresA.cleanup();
  helixSpheresB.cleanup();
  helixSpheresC.cleanup();
  gprtBufferDestroy(frameBuffer);
  gprtRayGenDestroy(rayGen);
  gprtMissDestroy(miss);
  gprtAccelDestroy(trianglesBLAS);
  gprtAccelDestroy(spheresBLAS);
  gprtAccelDestroy(TLAS);
  gprtGeomTypeDestroy(trianglesGeomType);
  gprtGeomTypeDestroy(spheresGeomType);
  gprtModuleDestroy(module);
  gprtContextDestroy(context);

  LOG_OK("seems all went OK; app is done, this should be the last output ...");
}