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

#include "sharedCode.h"

[shader("compute")]
[numthreads(32, 1, 1)]
void PLOCComputeCodes(uniform PLOCComputeCodesArgs args, uint3 dispatchThreadID : SV_DispatchThreadID)
{
    uint primitiveIndex = dispatchThreadID.x;
    if (primitiveIndex >= args.primitiveCount) return;

    uint3 indices = gprt::load<uint3>(args.indices, primitiveIndex);
    float3 V0 = gprt::load<float3>(args.vertices, indices.x);
    float3 V1 = gprt::load<float3>(args.vertices, indices.y);
    float3 V2 = gprt::load<float3>(args.vertices, indices.z);
    
    float3 center = (V0 + V1 + V2) / 3.0f;
    center = (center - args.root.bounds[0]) / (args.root.bounds[1] - args.root.bounds[0]);
    
    // Compute the Morton code for the primitive
    uint64_t code = sfc::hilbert64_encode(center);

    // Store the code
    gprt::store<uint64_t>(args.codes, primitiveIndex, code);
    gprt::store<uint64_t>(args.primIDs, primitiveIndex, primitiveIndex);
}

[shader("compute")]
[numthreads(32, 1, 1)]
void PLOCCreateLeaves(uniform PLOCCreateLeavesArgs args, uint3 dispatchThreadID : SV_DispatchThreadID)
{
    uint dispatchIndex = dispatchThreadID.x;
    if (dispatchIndex >= args.primitiveCount) return;
    
    uint64_t primitiveIndex = gprt::load<uint64_t>(args.primIDs, dispatchIndex);

    uint3 indices = gprt::load<uint3>(args.indices, primitiveIndex);
    float3 V0 = gprt::load<float3>(args.vertices, indices.x);
    float3 V1 = gprt::load<float3>(args.vertices, indices.y);
    float3 V2 = gprt::load<float3>(args.vertices, indices.z);

    float3 VMin = min(V0, min(V1, V2));
    float3 VMax = max(V0, max(V1, V2));

    Node node;
    node.primitive_count = 1;
    node.first_child_or_primitive = dispatchIndex;
    node.aabb.bounds[0] = VMin;
    node.aabb.bounds[1] = VMax;

    // Binary trees have 2 * numPrims - 1 nodes, where leaves start at numPrims - 1
    int offset = args.primitiveCount - 1;
    gprt::store<Node>(args.nodes, dispatchIndex + offset, node);
}

[shader("compute")]
[numthreads(32, 1, 1)]
void PLOCMergeNodes(uniform PLOCMergeNodesArgs args, uint3 dispatchThreadID : SV_DispatchThreadID)
{
    uint begin = args.begin;
    uint end = args.end;
    uint unmerged_begin = args.unmerged_begin;
    uint children_begin = args.children_begin;

    uint dispatchIndex = dispatchThreadID.x;
    if (dispatchIndex >= args.end - args.begin) return;

    uint32_t original_i = begin + dispatchIndex;
    uint32_t j = gprt::load<uint32_t>(args.neighbors, original_i);
    uint32_t i = gprt::load<uint32_t>(args.neighbors, j);
    if (i == original_i) {
        if (original_i < j) {
            uint32_t merged_index_i = gprt::load<uint32_t>(args.mergedIndex, i);
            uint32_t merged_index_j = gprt::load<uint32_t>(args.mergedIndex, j);
            Node unmerged_node;
            Node input_i = gprt::load<Node>(args.input, i);
            Node input_j = gprt::load<Node>(args.input, j);
            uint32_t first_child = children_begin + (merged_index_i - 1) * 2;
            unmerged_node.aabb = AABB::GetUnion(input_j.aabb, input_i.aabb);
            unmerged_node.primitive_count = 0;
            unmerged_node.first_child_or_primitive = first_child;
            gprt::store<Node>(args.output, unmerged_begin + j - begin - merged_index_j, unmerged_node);
            gprt::store<Node>(args.output, first_child + 0, input_i);
            gprt::store<Node>(args.output, first_child + 1, input_j);
        }
    } else {
        uint32_t merged_index_original_i = gprt::load<uint32_t>(args.mergedIndex, original_i);
        Node input_original_i = gprt::load<Node>(args.input, original_i);
        gprt::store<Node>(args.output, unmerged_begin + original_i - begin - merged_index_original_i, input_original_i);
    }
}

[shader("compute")]
[numthreads(32, 1, 1)]
void PLOCCopyNodes(uniform PLOCCopyNodesArgs args, uint3 dispatchThreadID : SV_DispatchThreadID)
{
    // Copy the nodes of the previous level into the current array of nodes.
    uint dispatchIndex = dispatchThreadID.x;
    if (dispatchIndex >= args.previousEnd - args.end) return;

    Node node = gprt::load<Node>(args.input, args.end + dispatchIndex);
    gprt::store<Node>(args.output, args.end + dispatchIndex, node);
}

[shader("raygeneration")] 
void simpleRayGen(uniform RayGenData record)
{
  uint2 pixelID = DispatchRaysIndex().xy;
  uint2 fbSize = DispatchRaysDimensions().xy;

  if (pixelID.x == 0 && pixelID.y == 0) {
    printf("Hello from your first raygen program!\n");
  }

  // Generate a simple checkerboard pattern as a test.
  int pattern = (pixelID.x / 32) ^ (pixelID.y / 32);
  // alternate pattern, showing that pixel (0,0) is in the upper left corner
  // pattern = (pixelID.x*pixelID.x + pixelID.y*pixelID.y) / 100000;
  const float3 color = bool(pattern & 1) ? record.color1 : record.color0;

  // find the frame buffer location (x + width*y) and put the result there
  const int fbOfs = pixelID.x + fbSize.x * pixelID.y;
  gprt::store(record.frameBuffer, fbOfs, gprt::make_bgra(color));
}