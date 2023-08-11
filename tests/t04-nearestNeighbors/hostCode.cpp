// MIT License

// Copyright (c) 2022 Nathan V. Morrical

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

// public GPRT API
#include <gprt.h>

// for generating meshes
#include <generator.hpp>
using namespace generator;

// For querying nearest neighbors
#include "gprt_knn.h"

// Shared GPU data structures
#include "sharedCode.h"

#include <iostream>
#include <stdexcept>
#include <random>
#include <algorithm>

template <typename T> struct Mesh {
  std::vector<float3> vertices;
  std::vector<uint2> edges;
  std::vector<uint3> triangles;
  
  Mesh(){};
  Mesh(T generator) {
    // Use the generator to generate vertices and indices
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
      triangles.push_back(uint3(vertices[0], vertices[1], vertices[2]));
      edges.push_back(uint2(vertices[0], vertices[1]));
      edges.push_back(uint2(vertices[1], vertices[2]));
      edges.push_back(uint2(vertices[2], vertices[0]));
      triGenerator.next();
    }
  };
};

// Common device code for testing different nearest neighbor queries
extern GPRTProgram t04_deviceCode;

int
main(int ac, char **av) {

  // Generate a common mesh for testing
  Mesh<TeapotMesh> highResTeapot(TeapotMesh{12});
  Mesh<TeapotMesh> lowResTeapot(TeapotMesh{6});

  // Test closest point query
  {
    GPRTContext context = gprtContextCreate(nullptr, 1);
    GPRTModule module = gprtModuleCreate(context, t04_deviceCode);
    GPRTRayGenOf<RayGenData> rayGen = gprtRayGenCreate<RayGenData>(context, module, "closestPoint");

    // Treat vertices of high res teapot as our data points.
    GPRTBufferOf<float3> dataPoints = gprtDeviceBufferCreate<float3>(context, highResTeapot.vertices.size(), highResTeapot.vertices.data());

    // Treat vertices of low res teapot as our query points.
    GPRTBufferOf<float3> queryPoints = gprtDeviceBufferCreate<float3>(context, lowResTeapot.vertices.size(), lowResTeapot.vertices.data());
    
    // Buffer that will contain results for K=1
    GPRTBufferOf<int2> resultBuffer = gprtDeviceBufferCreate<int2>(context, lowResTeapot.vertices.size());

    // Create the accel
    GPRTKNNAccel knnAccel = gprtKNNAccelCreate(context, GPRT_KNN_TYPE_POINTS, dataPoints, nullptr, nullptr, highResTeapot.vertices.size(), .1f);

    // enabling internal testing variable
    knnAccel._testing = true;
    
    // Build the accel
    gprtKNNAccelBuild(context, knnAccel);
    
    // // Now, upload accel to record
    // {
    //   RayGenData* data = gprtRayGenGetParameters(rayGen);
    //   data->knnAccel = knnAccel.handle;
    //   data->queryBuffer = gprtBufferGetHandle(queryPoints);
    //   data->resultBuffer = gprtBufferGetHandle(resultBuffer);
    // }

    // // Launch
    // gprtRayGenLaunch1D(context, rayGen, lowResTeapot.vertices.size());

    // // todo: Verify results are correct
    // std::vector<int2> gpuResults(lowResTeapot.vertices.size());
    // std::vector<int2> cpuResults(lowResTeapot.vertices.size());
    // {
    //   int2* data = gprtBufferGetPointer(resultBuffer);
    //   memcpy(gpuResults.data(), data, sizeof(int2) * lowResTeapot.vertices.size());
    // }

    // // find results on CPU with a brute force search
    // for (uint32_t i = 0; i < lowResTeapot.vertices.size(); ++i) {
    //   float3 queryPoint = lowResTeapot.vertices[i];
    //   for (uint32_t j = 0; j < highResTeapot.vertices.size(); ++j) {
         
    //   }
    // }

    // Cleanup

    gprtBufferDestroy(dataPoints);
    gprtBufferDestroy(queryPoints);
    gprtBufferDestroy(resultBuffer);
    gprtKNNAccelDestroy(knnAccel);  
    gprtModuleDestroy(module);
    gprtContextDestroy(context);
  }

  // Test closest point on edge query
  {
    GPRTContext context = gprtContextCreate(nullptr, 1);
    GPRTModule module = gprtModuleCreate(context, t04_deviceCode);
    GPRTRayGenOf<RayGenData> rayGen = gprtRayGenCreate<RayGenData>(context, module, "closestPointOnEdge");

    // Treat edges of high res teapot as our primitives.
    GPRTBufferOf<float3> vertices = gprtDeviceBufferCreate<float3>(context, highResTeapot.vertices.size(), highResTeapot.vertices.data());
    GPRTBufferOf<uint2> edges = gprtDeviceBufferCreate<uint2>(context, highResTeapot.edges.size(), highResTeapot.edges.data());

    // Treat vertices of low res teapot as our query points.
    GPRTBufferOf<float3> queryPoints = gprtDeviceBufferCreate<float3>(context, lowResTeapot.vertices.size(), lowResTeapot.vertices.data());
    
    // Buffer that will contain results for K=1
    GPRTBufferOf<int2> resultBuffer = gprtDeviceBufferCreate<int2>(context, lowResTeapot.vertices.size());

    // Create the accel
    GPRTKNNAccel knnAccel = gprtKNNAccelCreate(context, GPRT_KNN_TYPE_EDGES, vertices, edges, nullptr, highResTeapot.edges.size(), .1f);

    // enabling internal testing variable
    knnAccel._testing = true;
    
    // Build the accel
    gprtKNNAccelBuild(context, knnAccel);
    
    // // Now, upload accel to record
    // {
    //   RayGenData* data = gprtRayGenGetParameters(rayGen);
    //   data->knnAccel = knnAccel.handle;
    //   data->queryBuffer = gprtBufferGetHandle(queryPoints);
    //   data->resultBuffer = gprtBufferGetHandle(resultBuffer);
    // }

    // // Launch
    // gprtRayGenLaunch1D(context, rayGen, lowResTeapot.vertices.size());

    // // todo: Verify results are correct
    // std::vector<int2> gpuResults(lowResTeapot.vertices.size());
    // std::vector<int2> cpuResults(lowResTeapot.vertices.size());
    // {
    //   int2* data = gprtBufferGetPointer(resultBuffer);
    //   memcpy(gpuResults.data(), data, sizeof(int2) * lowResTeapot.vertices.size());
    // }

    // // find results on CPU with a brute force search
    // for (uint32_t i = 0; i < lowResTeapot.vertices.size(); ++i) {
    //   float3 queryPoint = lowResTeapot.vertices[i];
    //   for (uint32_t j = 0; j < highResTeapot.edges.size(); ++j) {
         
    //   }
    // }

    // Cleanup
    gprtBufferDestroy(vertices);
    gprtBufferDestroy(edges);
    gprtBufferDestroy(queryPoints);
    gprtBufferDestroy(resultBuffer);
    gprtKNNAccelDestroy(knnAccel);  
    gprtModuleDestroy(module);
    gprtContextDestroy(context);
  }

  // Test closest point on triangle query
  {
    GPRTContext context = gprtContextCreate(nullptr, 1);
    GPRTModule module = gprtModuleCreate(context, t04_deviceCode);
    GPRTRayGenOf<RayGenData> rayGen = gprtRayGenCreate<RayGenData>(context, module, "closestPointOnTriangle");

    // Treat triangles of high res teapot as our primitives.
    GPRTBufferOf<float3> vertices = gprtDeviceBufferCreate<float3>(context, highResTeapot.vertices.size(), highResTeapot.vertices.data());
    GPRTBufferOf<uint3> triangles = gprtDeviceBufferCreate<uint3>(context, highResTeapot.triangles.size(), highResTeapot.triangles.data());

    // Treat vertices of low res teapot as our query points.
    GPRTBufferOf<float3> queryPoints = gprtDeviceBufferCreate<float3>(context, lowResTeapot.vertices.size(), lowResTeapot.vertices.data());
    
    // Buffer that will contain results for K=1
    GPRTBufferOf<int2> resultBuffer = gprtDeviceBufferCreate<int2>(context, lowResTeapot.vertices.size());

    // Create the accel
    GPRTKNNAccel knnAccel = gprtKNNAccelCreate(context, GPRT_KNN_TYPE_TRIANGLES, vertices, nullptr, triangles, highResTeapot.triangles.size(), .1f);

    // enabling internal testing variable
    knnAccel._testing = true;
    
    // Build the accel
    gprtKNNAccelBuild(context, knnAccel);
    
    // // Now, upload accel to record
    // {
    //   RayGenData* data = gprtRayGenGetParameters(rayGen);
    //   data->knnAccel = knnAccel.handle;
    //   data->queryBuffer = gprtBufferGetHandle(queryPoints);
    //   data->resultBuffer = gprtBufferGetHandle(resultBuffer);
    // }

    // // Launch
    // gprtRayGenLaunch1D(context, rayGen, lowResTeapot.vertices.size());

    // // todo: Verify results are correct
    // std::vector<int2> gpuResults(lowResTeapot.vertices.size());
    // std::vector<int2> cpuResults(lowResTeapot.vertices.size());
    // {
    //   int2* data = gprtBufferGetPointer(resultBuffer);
    //   memcpy(gpuResults.data(), data, sizeof(int2) * lowResTeapot.vertices.size());
    // }

    // // find results on CPU with a brute force search
    // for (uint32_t i = 0; i < lowResTeapot.vertices.size(); ++i) {
    //   float3 queryPoint = lowResTeapot.vertices[i];
    //   for (uint32_t j = 0; j < highResTeapot.edges.size(); ++j) {
         
    //   }
    // }

    // Cleanup
    gprtBufferDestroy(vertices);
    gprtBufferDestroy(triangles);
    gprtBufferDestroy(queryPoints);
    gprtBufferDestroy(resultBuffer);
    gprtKNNAccelDestroy(knnAccel);  
    gprtModuleDestroy(module);
    gprtContextDestroy(context);
  }
}