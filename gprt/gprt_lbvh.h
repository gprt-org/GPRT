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
#pragma once

#include "gprt.h"

struct LBVHData {
  // input
  alignas(4) uint32_t numPrims;
  alignas(4) uint32_t numNodes;
  alignas(4) uint32_t numInner;
  alignas(4) uint32_t tmp;
  alignas(16) gprt::Buffer positions;
  alignas(16) gprt::Buffer edges;
  alignas(16) gprt::Buffer triangles;

  // Morton codes of quantized primitive centroids
  // One uint32_t per primitive
  alignas(16) gprt::Buffer mortonCodes;

  // Primitive IDs that correspond to sorted morton codes. 
  // One uint32_t per primitive
  alignas(16) gprt::Buffer ids;
  
  // numPrims-1 + numPrims long. 
  // The "numPrims-1" section contains inner nodes
  // The "numPrims" section contains leaves
  // Each node is an int4. 
  // "X" is left, "Y" is right, "Z" is parent, and "W" is leaf or -1 if internal node.
  alignas(16) gprt::Buffer nodes;

  // numPrims-1 + numPrims long. Each aabb is a pair of float3.
  alignas(16) gprt::Buffer aabbs;
};

#ifndef GPRT_DEVICE
#include <limits.h>
extern GPRTProgram lbvhDeviceCode;

typedef enum {
    GPRT_LBVH_POINTS = 1,
    GPRT_LBVH_EDGES = 2,
    GPRT_LBVH_TRIANGLES = 4
} GPRTLBVHType;

struct GPRTLBVH {
    LBVHData handle;
    GPRTModule module;

    GPRTLBVHType type;

    GPRTBufferOf<uint32_t> mortonCodes;
    GPRTBufferOf<uint32_t> ids;
    GPRTBufferOf<int4> nodes;
    GPRTBufferOf<float3> aabbs;

    GPRTBufferOf<uint8_t> scratch;

    GPRTComputeOf<LBVHData> computePointBounds;
    GPRTComputeOf<LBVHData> computeEdgeBounds;
    GPRTComputeOf<LBVHData> computeTriangleBounds;
    GPRTComputeOf<LBVHData> computePointMortonCodes;
    GPRTComputeOf<LBVHData> computeEdgeMortonCodes;
    GPRTComputeOf<LBVHData> computeTriangleMortonCodes;
    GPRTComputeOf<LBVHData> makeNodes;
    GPRTComputeOf<LBVHData> splitNodes;
    GPRTComputeOf<LBVHData> buildHierarchy;
};

inline 
GPRTLBVH gprtPointsLBVHCreate(GPRTContext context, GPRTBufferOf<float3> vertices, uint32_t count) {
    GPRTLBVH lbvh;
    lbvh.type = GPRT_LBVH_POINTS;
    lbvh.module = gprtModuleCreate(context, lbvhDeviceCode);

    lbvh.computePointBounds = gprtComputeCreate<LBVHData>(context, lbvh.module, "ComputePointBounds");
    lbvh.computePointMortonCodes = gprtComputeCreate<LBVHData>(context, lbvh.module, "ComputePointMortonCodes");
    lbvh.makeNodes = gprtComputeCreate<LBVHData>(context, lbvh.module, "MakeNodes");
    lbvh.splitNodes = gprtComputeCreate<LBVHData>(context, lbvh.module, "SplitNodes");
    lbvh.buildHierarchy = gprtComputeCreate<LBVHData>(context, lbvh.module, "BuildPointHierarchy");
    
    lbvh.handle.numPrims = count;
    lbvh.handle.numNodes = lbvh.handle.numPrims * 2 - 1;
    lbvh.handle.numInner = lbvh.handle.numPrims - 1;
    lbvh.handle.positions = gprtBufferGetHandle(vertices);

    lbvh.mortonCodes = gprtDeviceBufferCreate<uint32_t>(context, lbvh.handle.numPrims);
    lbvh.ids = gprtDeviceBufferCreate<uint32_t>(context, lbvh.handle.numPrims);
    lbvh.nodes = gprtDeviceBufferCreate<int4>(context, lbvh.handle.numNodes);
    lbvh.aabbs = gprtDeviceBufferCreate<float3>(context, 2 * lbvh.handle.numNodes);
    lbvh.scratch = gprtDeviceBufferCreate<uint8_t>(context);

    lbvh.handle.mortonCodes = gprtBufferGetHandle(lbvh.mortonCodes);
    lbvh.handle.ids = gprtBufferGetHandle(lbvh.ids);
    lbvh.handle.nodes = gprtBufferGetHandle(lbvh.nodes);
    lbvh.handle.aabbs = gprtBufferGetHandle(lbvh.aabbs);

    gprtComputeSetParameters(lbvh.computePointBounds, &lbvh.handle);
    gprtComputeSetParameters(lbvh.computePointMortonCodes, &lbvh.handle);
    gprtComputeSetParameters(lbvh.makeNodes, &lbvh.handle);
    gprtComputeSetParameters(lbvh.splitNodes, &lbvh.handle);
    gprtComputeSetParameters(lbvh.buildHierarchy, &lbvh.handle);

    gprtBuildShaderBindingTable(context, GPRT_SBT_COMPUTE);

    return lbvh;
}

inline 
GPRTLBVH gprtTriangleLBVHCreate(GPRTContext context, GPRTBufferOf<float3> vertices, GPRTBufferOf<uint3> indices, uint32_t count) {
    GPRTLBVH lbvh;
    lbvh.type = GPRT_LBVH_TRIANGLES;
    lbvh.module = gprtModuleCreate(context, lbvhDeviceCode);

    lbvh.computeTriangleBounds = gprtComputeCreate<LBVHData>(context, lbvh.module, "ComputeTriangleBounds");
    lbvh.computeTriangleMortonCodes = gprtComputeCreate<LBVHData>(context, lbvh.module, "ComputeTriangleMortonCodes");
    lbvh.makeNodes = gprtComputeCreate<LBVHData>(context, lbvh.module, "MakeNodes");
    lbvh.splitNodes = gprtComputeCreate<LBVHData>(context, lbvh.module, "SplitNodes");
    lbvh.buildHierarchy = gprtComputeCreate<LBVHData>(context, lbvh.module, "BuildTriangleHierarchy");

    lbvh.handle.numPrims = count;
    lbvh.handle.numNodes = lbvh.handle.numPrims * 2 - 1;
    lbvh.handle.numInner = lbvh.handle.numPrims - 1;
    lbvh.handle.positions = gprtBufferGetHandle(vertices);
    lbvh.handle.triangles = gprtBufferGetHandle(indices);

    lbvh.mortonCodes = gprtDeviceBufferCreate<uint32_t>(context, lbvh.handle.numPrims);
    lbvh.ids = gprtDeviceBufferCreate<uint32_t>(context, lbvh.handle.numPrims);
    lbvh.nodes = gprtDeviceBufferCreate<int4>(context, lbvh.handle.numNodes);
    lbvh.aabbs = gprtDeviceBufferCreate<float3>(context, 2 * lbvh.handle.numNodes);
    lbvh.scratch = gprtDeviceBufferCreate<uint8_t>(context);

    lbvh.handle.mortonCodes = gprtBufferGetHandle(lbvh.mortonCodes);
    lbvh.handle.ids = gprtBufferGetHandle(lbvh.ids);
    lbvh.handle.nodes = gprtBufferGetHandle(lbvh.nodes);
    lbvh.handle.aabbs = gprtBufferGetHandle(lbvh.aabbs);
    
    gprtComputeSetParameters(lbvh.computeTriangleBounds, &lbvh.handle);
    gprtComputeSetParameters(lbvh.computeTriangleMortonCodes, &lbvh.handle);
    gprtComputeSetParameters(lbvh.makeNodes, &lbvh.handle);
    gprtComputeSetParameters(lbvh.splitNodes, &lbvh.handle);
    gprtComputeSetParameters(lbvh.buildHierarchy, &lbvh.handle);

    gprtBuildShaderBindingTable(context, GPRT_SBT_COMPUTE);

    return lbvh;
}

inline void gprtLBVHBuild(GPRTContext context, GPRTLBVH &lbvh) {
    typedef uint32_t uint;

    // initialize root AABB
    gprtBufferMap(lbvh.aabbs);
    float3* aabbPtr = gprtBufferGetPointer(lbvh.aabbs);
    aabbPtr[0].x = aabbPtr[0].y = aabbPtr[0].z = std::numeric_limits<float>::max();
    aabbPtr[1].x = aabbPtr[1].y = aabbPtr[1].z = -std::numeric_limits<float>::max();
    gprtBufferUnmap(lbvh.aabbs);

    // Now compute bounding boxes over the primitives
    if (lbvh.type == GPRT_LBVH_POINTS) {
        gprtComputeLaunch1D(context, lbvh.computePointBounds, lbvh.handle.numPrims);
        gprtComputeLaunch1D(context, lbvh.computePointMortonCodes, lbvh.handle.numPrims);
    } else if (lbvh.type == GPRT_LBVH_EDGES) {
        gprtComputeLaunch1D(context, lbvh.computeEdgeBounds, lbvh.handle.numPrims);
        gprtComputeLaunch1D(context, lbvh.computeEdgeMortonCodes, lbvh.handle.numPrims);
    } else if (lbvh.type == GPRT_LBVH_TRIANGLES) {
        gprtComputeLaunch1D(context, lbvh.computeTriangleBounds, lbvh.handle.numPrims);
        gprtComputeLaunch1D(context, lbvh.computeTriangleMortonCodes, lbvh.handle.numPrims);
    }
    gprtBufferSortPayload(context, lbvh.mortonCodes, lbvh.ids, lbvh.scratch);
    gprtComputeLaunch1D(context, lbvh.makeNodes, lbvh.handle.numNodes);
    gprtComputeLaunch1D(context, lbvh.splitNodes, lbvh.handle.numInner);
    gprtComputeLaunch1D(context, lbvh.buildHierarchy, lbvh.handle.numPrims);
}

inline void gprtLBVHDestroy(GPRTLBVH &lbvh) {
    gprtBufferDestroy(lbvh.mortonCodes);
    gprtBufferDestroy(lbvh.ids);
    gprtBufferDestroy(lbvh.nodes);
    gprtBufferDestroy(lbvh.aabbs);
    gprtBufferDestroy(lbvh.scratch);

    gprtComputeDestroy(lbvh.computePointBounds);
    gprtComputeDestroy(lbvh.computePointMortonCodes);
    gprtComputeDestroy(lbvh.makeNodes);
    gprtComputeDestroy(lbvh.splitNodes);
    gprtComputeDestroy(lbvh.buildHierarchy);
    gprtModuleDestroy(lbvh.module);
}

#endif