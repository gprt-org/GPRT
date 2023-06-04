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
extern GPRTProgram lbvhDeviceCode;

struct GPRTLBVH {
    LBVHData params;
    GPRTModule module;

    GPRTBufferOf<uint32_t> mortonCodes;
    GPRTBufferOf<uint32_t> ids;
    GPRTBufferOf<int4> nodes;
    GPRTBufferOf<float3> aabbs;

    GPRTBufferOf<uint8_t> scratch;

    GPRTComputeOf<LBVHData> computePointBounds;
    GPRTComputeOf<LBVHData> computePointMortonCodes;
    GPRTComputeOf<LBVHData> makeNodes;
    GPRTComputeOf<LBVHData> splitNodes;
    GPRTComputeOf<LBVHData> buildHierarchy;
};

inline 
GPRTLBVH gprtPointsLBVHCreate(GPRTContext context, GPRTBufferOf<float3> vertices, size_t count) {
    GPRTLBVH lbvh;
    lbvh.module = gprtModuleCreate(context, lbvhDeviceCode);

    lbvh.computePointBounds = gprtComputeCreate<LBVHData>(context, lbvh.module, "ComputePointBounds");
    lbvh.computePointMortonCodes = gprtComputeCreate<LBVHData>(context, lbvh.module, "ComputePointMortonCodes");
    lbvh.makeNodes = gprtComputeCreate<LBVHData>(context, lbvh.module, "MakeNodes");
    lbvh.splitNodes = gprtComputeCreate<LBVHData>(context, lbvh.module, "SplitNodes");
    lbvh.buildHierarchy = gprtComputeCreate<LBVHData>(context, lbvh.module, "BuildPointHierarchy");

    lbvh.params.numPrims = count;
    lbvh.params.numNodes = lbvh.params.numPrims * 2 - 1;
    lbvh.params.numInner = lbvh.params.numPrims - 1;
    lbvh.params.positions = gprtBufferGetHandle(vertices);

    lbvh.mortonCodes = gprtDeviceBufferCreate<uint32_t>(context, lbvh.params.numPrims);
    lbvh.ids = gprtDeviceBufferCreate<uint32_t>(context, lbvh.params.numPrims);
    lbvh.nodes = gprtDeviceBufferCreate<int4>(context, lbvh.params.numNodes);
    lbvh.aabbs = gprtDeviceBufferCreate<float3>(context, 2 * lbvh.params.numNodes);
    lbvh.scratch = gprtDeviceBufferCreate<uint8_t>(context);

    gprtComputeSetParameters(lbvh.computePointBounds, &lbvh.params);
    gprtComputeSetParameters(lbvh.computePointMortonCodes, &lbvh.params);
    gprtComputeSetParameters(lbvh.makeNodes, &lbvh.params);
    gprtComputeSetParameters(lbvh.splitNodes, &lbvh.params);
    gprtComputeSetParameters(lbvh.buildHierarchy, &lbvh.params);

    gprtBuildShaderBindingTable(context, GPRT_SBT_COMPUTE);

    return lbvh;
}

inline void gprtLBVHBuild(GPRTContext context, GPRTLBVH &lbvh) {
    gprtComputeLaunch1D(context, lbvh.computePointBounds, lbvh.params.numPrims);
    gprtComputeLaunch1D(context, lbvh.computePointMortonCodes, lbvh.params.numPrims);
    gprtBufferSortPayload(context, lbvh.mortonCodes, lbvh.ids, lbvh.scratch);
    gprtComputeLaunch1D(context, lbvh.makeNodes, lbvh.params.numNodes);
    gprtComputeLaunch1D(context, lbvh.splitNodes, lbvh.params.numInner);
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