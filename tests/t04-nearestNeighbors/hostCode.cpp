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

    // -------------------------------------------------------
    // declare geometry type
    // -------------------------------------------------------
    GPRTKNNGeomType pointGeomType =
      gprtKNNGeomTypeCreate(context, GPRT_KNN_GEOM_KIND_POINTS /* <- This is new! */);
    gprtKNNGeomTypeSetClosestNeighborProg(pointGeomType, 0, module, "closestPoint");

    // -------------------------------------------------------
    // set up ray gen program
    // -------------------------------------------------------
    GPRTRayGenOf<RayGenData> rayGen = gprtRayGenCreate<RayGenData>(context, module, "NearestNeighbor");

    // ##################################################################
    // set the parameters for those kernels
    // ##################################################################
    
    // Buffer that will contain results for K=1
    GPRTBufferOf<int> resultIDsBuffer = gprtDeviceBufferCreate<int>(context, lowResTeapot.vertices.size());
    GPRTBufferOf<float> resultDistsBuffer = gprtDeviceBufferCreate<float>(context, lowResTeapot.vertices.size());
    
    // Treat vertices of low res teapot as our query points.
    GPRTBufferOf<float3> queryPoints = gprtDeviceBufferCreate<float3>(context, lowResTeapot.vertices.size(), lowResTeapot.vertices.data());
    
    RayGenData* rayGenData = gprtRayGenGetParameters(rayGen);
    rayGenData->resultIDsBuffer = gprtBufferGetHandle(resultIDsBuffer);
    rayGenData->resultDistsBuffer = gprtBufferGetHandle(resultDistsBuffer);
    rayGenData->queryBuffer = gprtBufferGetHandle(queryPoints);

    // Create geometry
    // Treat vertices of high res teapot as our data points.
    GPRTBufferOf<float3> dataPoints = gprtDeviceBufferCreate<float3>(context, highResTeapot.vertices.size(), highResTeapot.vertices.data());
    GPRTKNNGeom geom = gprtKNNGeomCreate(context, pointGeomType);
    gprtKNNGeomSetPositions(geom, dataPoints, highResTeapot.vertices.size());

    // Create the accel
    GPRTKNNAccel knnAccel = gprtKNNPointsAccelCreate(context, 1, &geom);
    gprtKNNAccelSetSearchRange(knnAccel, .1f);

    // enabling internal testing variable
    knnAccel._testing = true;
    
    // Build the accel
    gprtKNNAccelBuild(context, knnAccel, GPRT_KNN_BUILD_MODE_FAST_TRACE_NO_UPDATE);
    
    // Now, upload accel to record
    rayGenData->knnAccel = knnAccel.handle;

    gprtBuildShaderBindingTable(context, GPRT_SBT_ALL);

    // Launch
    gprtRayGenLaunch1D(context, rayGen, lowResTeapot.vertices.size());

    // Verify results are correct
    gprtBufferMap(resultIDsBuffer);
    gprtBufferMap(resultDistsBuffer);
    
    int* IDs = gprtBufferGetPointer(resultIDsBuffer);
    float* dists = gprtBufferGetPointer(resultDistsBuffer);
    
    // Confirm results on CPU with a brute force search
    for (uint32_t i = 0; i < lowResTeapot.vertices.size(); ++i) {
      float3 queryPoint = lowResTeapot.vertices[i];

      /* First, confirm distance for the given primitive is correct */
      float gpuDist = dists[i];
      int gpuPrim = IDs[i];

      float actualDist = getPointDist2(queryPoint, highResTeapot.vertices[gpuPrim]);
      if (abs(gpuDist - actualDist) > .0001f) { 
        throw std::runtime_error("Incorrect distance for closest found primitive!");
      }
      
      /* Second, confirm this is indeed a candidate closest primitive */
      float closestDistance = 1e20f;
      for (uint32_t j = 0; j < highResTeapot.vertices.size(); ++j) {
        float distance = getPointDist2(queryPoint, highResTeapot.vertices[j]);
        if (distance < closestDistance && distance <= pow(knnAccel.handle.maxSearchRange, 2.f)) {
          closestDistance = distance;
        }
      }

      if (abs(gpuDist - closestDistance) > .0001f) { 
        throw std::runtime_error("Found primitive isn't closest!");
      }
    }

    gprtBufferUnmap(resultIDsBuffer);
    gprtBufferUnmap(resultDistsBuffer);

    // Cleanup
    gprtBufferDestroy(dataPoints);
    gprtBufferDestroy(queryPoints);
    gprtBufferDestroy(resultIDsBuffer);
    gprtBufferDestroy(resultDistsBuffer);
    gprtKNNAccelDestroy(knnAccel);  
    gprtModuleDestroy(module);
    gprtContextDestroy(context);
  }

  // Test closest point on edge query
  {
    GPRTContext context = gprtContextCreate(nullptr, 1);
    GPRTModule module = gprtModuleCreate(context, t04_deviceCode);
    
    // -------------------------------------------------------
    // declare geometry type
    // -------------------------------------------------------
    GPRTKNNGeomType edgeGeomType =
      gprtKNNGeomTypeCreate(context, GPRT_KNN_GEOM_KIND_EDGES /* <- This is new! */);
    gprtKNNGeomTypeSetClosestNeighborProg(edgeGeomType, 0, module, "closestEdge");

    // -------------------------------------------------------
    // set up ray gen program
    // -------------------------------------------------------
    GPRTRayGenOf<RayGenData> rayGen = gprtRayGenCreate<RayGenData>(context, module, "NearestNeighbor");

    // ##################################################################
    // set the parameters for those kernels
    // ##################################################################
    
    // Buffer that will contain results for K=1
    GPRTBufferOf<int> resultIDsBuffer = gprtDeviceBufferCreate<int>(context, lowResTeapot.vertices.size());
    GPRTBufferOf<float> resultDistsBuffer = gprtDeviceBufferCreate<float>(context, lowResTeapot.vertices.size());
    
    // Treat vertices of low res teapot as our query points.
    GPRTBufferOf<float3> queryPoints = gprtDeviceBufferCreate<float3>(context, lowResTeapot.vertices.size(), lowResTeapot.vertices.data());
    
    RayGenData* rayGenData = gprtRayGenGetParameters(rayGen);
    rayGenData->resultIDsBuffer = gprtBufferGetHandle(resultIDsBuffer);
    rayGenData->resultDistsBuffer = gprtBufferGetHandle(resultDistsBuffer);
    rayGenData->queryBuffer = gprtBufferGetHandle(queryPoints);

    // Create geometry
    // Treat edges of high res teapot as our primitives.
    GPRTBufferOf<float3> vertices = gprtDeviceBufferCreate<float3>(context, highResTeapot.vertices.size(), highResTeapot.vertices.data());
    GPRTBufferOf<uint2> edges = gprtDeviceBufferCreate<uint2>(context, highResTeapot.edges.size(), highResTeapot.edges.data());
    GPRTKNNGeom geom = gprtKNNGeomCreate(context, edgeGeomType);
    gprtKNNGeomSetPositions(geom, vertices, highResTeapot.vertices.size());
    gprtKNNEdgesSetIndices(geom, edges, highResTeapot.edges.size());
    
    // Create the accel
    GPRTKNNAccel knnAccel = gprtKNNEdgesAccelCreate(context, 1, &geom);
    gprtKNNAccelSetSearchRange(knnAccel, .1f);

    // enabling internal testing variable
    knnAccel._testing = true;
    
    // Build the accel
    gprtKNNAccelBuild(context, knnAccel, GPRT_KNN_BUILD_MODE_FAST_TRACE_NO_UPDATE);

    // Now, upload accel to record
    rayGenData->knnAccel = knnAccel.handle;

    gprtBuildShaderBindingTable(context, GPRT_SBT_ALL);

    // Launch
    gprtRayGenLaunch1D(context, rayGen, lowResTeapot.vertices.size());

    // Verify results are correct
    gprtBufferMap(resultIDsBuffer);
    gprtBufferMap(resultDistsBuffer);
    
    int* IDs = gprtBufferGetPointer(resultIDsBuffer);
    float* dists = gprtBufferGetPointer(resultDistsBuffer);
    
    // Confirm results on CPU with a brute force search
    for (uint32_t i = 0; i < lowResTeapot.vertices.size(); ++i) {
      float3 queryPoint = lowResTeapot.vertices[i];

      /* First, confirm distance for the given primitive is correct */
      float gpuDist = dists[i];
      int gpuPrim = IDs[i];
      uint2 gpuEdge = highResTeapot.edges[gpuPrim];

      float actualDist = getEdgeDist2(queryPoint, highResTeapot.vertices[gpuEdge.x], highResTeapot.vertices[gpuEdge.y]);
      if (abs(gpuDist - actualDist) > .0001f) { 
        throw std::runtime_error("Incorrect distance for closest found primitive!");
      }
      
      /* Second, confirm this is indeed a candidate closest primitive */
      float closestDistance = 1e20f;
      for (uint32_t j = 0; j < highResTeapot.edges.size(); ++j) {
        uint2 edge = highResTeapot.edges[j];
        float distance = getEdgeDist2(queryPoint, highResTeapot.vertices[edge.x], highResTeapot.vertices[edge.y]);
        if (distance < closestDistance && distance <= pow(knnAccel.handle.maxSearchRange, 2.f)) {
          closestDistance = distance;
        }
      }

      if (abs(gpuDist - closestDistance) > .0001f) { 
        throw std::runtime_error("Found primitive isn't closest!");
      }
    }

    gprtBufferUnmap(resultIDsBuffer);
    gprtBufferUnmap(resultDistsBuffer);

    // Cleanup
    gprtBufferDestroy(vertices);
    gprtBufferDestroy(edges);
    gprtBufferDestroy(queryPoints);
    gprtBufferDestroy(resultIDsBuffer);
    gprtBufferDestroy(resultDistsBuffer);
    gprtKNNAccelDestroy(knnAccel);  
    gprtModuleDestroy(module);
    gprtContextDestroy(context);
  }

  // Test closest point on triangle query
  {
    GPRTContext context = gprtContextCreate(nullptr, 1);
    GPRTModule module = gprtModuleCreate(context, t04_deviceCode);
    
    // -------------------------------------------------------
    // declare geometry type
    // -------------------------------------------------------
    GPRTKNNGeomType triangleGeomType =
      gprtKNNGeomTypeCreate(context, GPRT_KNN_GEOM_KIND_TRIANGLES /* <- This is new! */);
    gprtKNNGeomTypeSetClosestNeighborProg(triangleGeomType, 0, module, "closestTriangle");

    // -------------------------------------------------------
    // set up ray gen program
    // -------------------------------------------------------
    GPRTRayGenOf<RayGenData> rayGen = gprtRayGenCreate<RayGenData>(context, module, "NearestNeighbor");

    // ##################################################################
    // set the parameters for those kernels
    // ##################################################################
    
    // Buffer that will contain results for K=1
    GPRTBufferOf<int> resultIDsBuffer = gprtDeviceBufferCreate<int>(context, lowResTeapot.vertices.size());
    GPRTBufferOf<float> resultDistsBuffer = gprtDeviceBufferCreate<float>(context, lowResTeapot.vertices.size());
    
    // Treat vertices of low res teapot as our query points.
    GPRTBufferOf<float3> queryPoints = gprtDeviceBufferCreate<float3>(context, lowResTeapot.vertices.size(), lowResTeapot.vertices.data());
    
    RayGenData* rayGenData = gprtRayGenGetParameters(rayGen);
    rayGenData->resultIDsBuffer = gprtBufferGetHandle(resultIDsBuffer);
    rayGenData->resultDistsBuffer = gprtBufferGetHandle(resultDistsBuffer);
    rayGenData->queryBuffer = gprtBufferGetHandle(queryPoints);

    // Create geometry
    // Treat triangles of high res teapot as our primitives.
    GPRTBufferOf<float3> vertices = gprtDeviceBufferCreate<float3>(context, highResTeapot.vertices.size(), highResTeapot.vertices.data());
    GPRTBufferOf<uint3> triangles = gprtDeviceBufferCreate<uint3>(context, highResTeapot.triangles.size(), highResTeapot.triangles.data());
    GPRTKNNGeom geom = gprtKNNGeomCreate(context, triangleGeomType);
    gprtKNNGeomSetPositions(geom, vertices, highResTeapot.vertices.size());
    gprtKNNTrianglesSetIndices(geom, triangles, highResTeapot.triangles.size());

    // Create the accel
    GPRTKNNAccel knnAccel = gprtKNNTrianglesAccelCreate(context, 1, &geom);
    gprtKNNAccelSetSearchRange(knnAccel, .1f);
    
    // enabling internal testing variable
    knnAccel._testing = true;
    
    // Build the accel
    gprtKNNAccelBuild(context, knnAccel, GPRT_KNN_BUILD_MODE_FAST_TRACE_NO_UPDATE);

    // Now, upload accel to record
    rayGenData->knnAccel = knnAccel.handle;

    gprtBuildShaderBindingTable(context, GPRT_SBT_ALL);

    // Launch
    gprtRayGenLaunch1D(context, rayGen, lowResTeapot.vertices.size());

    // Verify results are correct
    gprtBufferMap(resultIDsBuffer);
    gprtBufferMap(resultDistsBuffer);
    
    int* IDs = gprtBufferGetPointer(resultIDsBuffer);
    float* dists = gprtBufferGetPointer(resultDistsBuffer);
    
    // Confirm results on CPU with a brute force search
    for (uint32_t i = 0; i < lowResTeapot.vertices.size(); ++i) {
      float3 queryPoint = lowResTeapot.vertices[i];

      /* First, confirm distance for the given primitive is correct */
      float gpuDist = dists[i];
      int gpuPrim = IDs[i];
      uint3 gpuTriangle = highResTeapot.triangles[gpuPrim];
      float actualDist = getTriangleDist2(queryPoint, highResTeapot.vertices[gpuTriangle.x], highResTeapot.vertices[gpuTriangle.y], highResTeapot.vertices[gpuTriangle.z]);
      if (abs(gpuDist - actualDist) > .0001f) { 
        throw std::runtime_error("Incorrect distance for closest found primitive!");
      }
      
      /* Second, confirm this is indeed a candidate closest primitive */
      float closestDistance = 1e20f;
      for (uint32_t j = 0; j < highResTeapot.triangles.size(); ++j) {
        uint3 triangle = highResTeapot.triangles[j];
        float distance = getTriangleDist2(queryPoint, highResTeapot.vertices[triangle.x], highResTeapot.vertices[triangle.y], highResTeapot.vertices[triangle.z]);
        if (distance < closestDistance && distance <= pow(knnAccel.handle.maxSearchRange, 2.f)) {
          closestDistance = distance;
        }
      }

      if (abs(gpuDist - closestDistance) > .0001f) { 
        throw std::runtime_error("Found primitive isn't closest!");
      }
    }

    gprtBufferUnmap(resultIDsBuffer);
    gprtBufferUnmap(resultDistsBuffer);

    // Cleanup
    gprtBufferDestroy(vertices);
    gprtBufferDestroy(triangles);
    gprtBufferDestroy(queryPoints);
    gprtBufferDestroy(resultIDsBuffer);
    gprtBufferDestroy(resultDistsBuffer);
    gprtKNNAccelDestroy(knnAccel);  
    gprtModuleDestroy(module);
    gprtContextDestroy(context);
  }
}