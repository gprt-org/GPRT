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

#include "gprt.h"

struct AABB {
    float3 bounds[2];    
    
    static AABB GetUnion(AABB bbox1, AABB bbox2) {
        AABB bbox;
        bbox.bounds[0] = min(bbox1.bounds[0], bbox2.bounds[0]);
        bbox.bounds[1] = max(bbox1.bounds[1], bbox2.bounds[1]);
        return bbox;
    }

    // Returns the surface area of the XY, XZ, and YZ planes.
    static float HalfArea(AABB bbox) {
        float3 d = bbox.bounds[1] - bbox.bounds[0];
        return d.x * (d.y + d.z) + d.y * d.z;
    }
};

// Size is 4 * 8 = 32 bytes. Assuming a scalar layout.
struct Node {
    AABB aabb;
    uint32_t first_child_or_primitive;
    
    // if primitive_count != 0, this is a leaf, in which case 
    // first_child_or_primitive is the index into the array of primitives.

    // if primitive_count == 0, this is an interior node, in which case
    // first_child_or_primitive is the index of the first child into the array of nodes. 
    // The second child is just first_child_or_primitive + 1.This ensures that both children 
    // are in the same cache line. It is not possible for a BVH node to have only one child.
    uint32_t primitive_count;
};

// Search radius of 14 is optimal
#ifdef __SLANG_COMPILER__
#include "sfc.slang"

uint32_t find_best_node(uint32_t node_id, gprt::Buffer nodes, uint32_t node_count, uint32_t search_radius) {
    uint32_t best_id = node_id;
    float best_distance = FLT_MAX;

    uint32_t begin = node_id > search_radius ? node_id - search_radius : 0;
    uint32_t end   = node_id + search_radius < node_count ? node_id + search_radius : node_count - 1;
    for (uint32_t other_id = begin; other_id < end; ++other_id) {
        Node A = gprt::load<Node>(nodes, node_id);
        Node B = gprt::load<Node>(nodes, other_id);
        float distance = AABB::HalfArea(AABB::GetUnion(A.aabb, B.aabb));
        if (distance < best_distance) {
            best_distance = distance;
            best_id = other_id;
        }
    }
    return best_id;
}

#endif

struct PLOCComputeCodesArgs {
    // vertices and indices of the triangle mesh
    gprt::Buffer vertices;
    gprt::Buffer indices;
    
    // number of triangles
    uint32_t primitiveCount;

    // A bounding box that encloses the mesh
    AABB root;

    // outputs, two 64-bit uint buffers
    gprt::Buffer primIDs;
    gprt::Buffer codes;
};

struct PLOCCreateLeavesArgs {
    // vertices and indices of the triangle mesh
    gprt::Buffer vertices;
    gprt::Buffer indices;
    uint32_t primitiveCount;

    // These should be sorted
    gprt::Buffer primIDs;
    gprt::Buffer codes;

    // input/output nodes buffer
    gprt::Buffer nodes;
};

struct PLOCMergeNodesArgs {
    // input nodes buffer
    gprt::Buffer input;
    
    // output nodes buffer
    gprt::Buffer output;

    // a buffer containing "best neighbor" indices for each node
    gprt::Buffer neighbors;

    // not sure I follow the purpose of this buffer yet. involved in addressing calculations for new nodes
    gprt::Buffer mergedIndex;

    uint32_t children_begin;
    uint32_t unmerged_begin;

    uint32_t begin;
    uint32_t end;
};

struct PLOCCopyNodesArgs {
    // input nodes buffer
    gprt::Buffer input;

    // output nodes buffer
    gprt::Buffer output;

    // The new end of the nodes buffer
    uint32_t end;

    // The previous end of the nodes buffer
    uint32_t previousEnd;
};

struct RayGenData {
    float3 color0;
    float3 color1;
    gprt::Buffer frameBuffer;
};

struct PushConstants {
    float3 color0;
    float3 color1;
    gprt::Buffer frameBuffer;
};