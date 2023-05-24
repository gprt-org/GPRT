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

#include "lbvh.h"

#define FLT_MAX 3.402823466e+38
#define FLT_MIN 1.175494351e-38

uint separate_bits(uint n)
{
    n &= 0x000003FF;
    n = (n ^ (n << 16)) & 0xFF0000FF;
    n = (n ^ (n <<  8)) & 0x0300F00F;
    n = (n ^ (n <<  4)) & 0x030C30C3;
    n = (n ^ (n <<  2)) & 0x09249249;
    return n;
};

inline uint morton_encode3D(uint x, uint y, uint z)
{
  return separate_bits(x) | (separate_bits(y) << 1) | (separate_bits(z) << 2); 
}

GPRT_COMPUTE_PROGRAM(ComputeMortonCodes, (LBVHData, record), (1, 1, 1)) {
  int primID = DispatchThreadID.x;
  float3 pt = gprt::load<float3>(record.centroids, primID);
  float3 aabbMin = record.aabbMin;
  float3 aabbMax = record.aabbMax;

  pt = (pt - aabbMin) / (aabbMax - aabbMin);

  // Quantize to 10 bit
  pt = min(max(pt * 1024.f, float3(0.f, 0.f, 0.f)), float3(1023.f, 1023.f, 1023.f));
  
  uint code = morton_encode3D(pt.x, pt.y, pt.z);
  gprt::store<uint>(record.mortonCodes, primID, code);
}

// Workaround for bug with vk::RawBufferStore and storing structs of data
void storeNode(gprt::Buffer buffer, int address, LBVHNode node) {
  vk::RawBufferStore<float3>(buffer.x + sizeof(LBVHNode) * address + 0 * sizeof(float3) + 0 * sizeof(int), node.aabbMin);
  vk::RawBufferStore<float3>(buffer.x + sizeof(LBVHNode) * address + 1 * sizeof(float3) + 0 * sizeof(int), node.aabbMax);
  vk::RawBufferStore<int>(buffer.x + sizeof(LBVHNode) * address + 2 * sizeof(float3) + 0 * sizeof(int), node.left);
  vk::RawBufferStore<int>(buffer.x + sizeof(LBVHNode) * address + 2 * sizeof(float3) + 1 * sizeof(int), node.right);
  vk::RawBufferStore<int>(buffer.x + sizeof(LBVHNode) * address + 2 * sizeof(float3) + 2 * sizeof(int), node.parent);
  vk::RawBufferStore<int>(buffer.x + sizeof(LBVHNode) * address + 2 * sizeof(float3) + 3 * sizeof(int), node.leaf);
}

void storeNodeLeft(gprt::Buffer buffer, int address, int left) {
  vk::RawBufferStore<int>(buffer.x + sizeof(LBVHNode) * address + 2 * sizeof(float3) + 0 * sizeof(int), left);
}

void storeNodeRight(gprt::Buffer buffer, int address, int right) {
  vk::RawBufferStore<int>(buffer.x + sizeof(LBVHNode) * address + 2 * sizeof(float3) + 1 * sizeof(int), right);
}

void storeNodeParent(gprt::Buffer buffer, int address, int parent) {
  vk::RawBufferStore<int>(buffer.x + sizeof(LBVHNode) * address + 2 * sizeof(float3) + 2 * sizeof(int), parent);
}

//-------------------------------------------------------------------------------------------------
// Find node range that an inner node overlaps
//

int delta(uint num_codes, uint i, uint j, uint i_code, uint j_code) {
  // Karras' delta(i,j) function
  // Denotes the length of the longest common
  // prefix between keys k_i and k_j

  // Cf. Figure 4: "for simplicity, we define that
  // delta(i,j) = -1 when j not in [0,n-1]"
  if (j < 0 || j >= num_codes)
    return -1;
  uint xord = i_code ^ j_code;
  if (xord == 0)
    return firstbithigh(i ^ j) + 32;
  else
    return firstbithigh(xord);
}

int2 determine_range(
  gprt::Buffer morton_codes,
  int num_codes, int i, out int split)
{
  // Determine direction of the range (+1 or -1)
  uint ch = gprt::load<uint>(morton_codes, i - 1);
  uint ci = gprt::load<uint>(morton_codes, i + 0);
  uint cj = gprt::load<uint>(morton_codes, i + 1);
  int d = delta(num_codes, i, i + 1, ci, cj) >= delta(num_codes, i, i - 1, ci, ch) ? 1 : -1;

  // Compute upper bound for the length of the range
  uint cd = gprt::load<uint>(morton_codes, i - d);
  int delta_min = delta(num_codes, i, i - d, ci, cd);
  int l_max = 2;
  uint cub = gprt::load<uint>(morton_codes, i + l_max * d);
  while (delta(num_codes, i, i + l_max * d, ci, cub) > delta_min)
  {
    l_max *= 2;
    cub = gprt::load<uint>(morton_codes, i + l_max * d);
  }

  // Find the other end using binary search
  int l = 0;
  for (int t = l_max >> 1; t >= 1; t >>= 1)
  {
    uint clb = gprt::load<uint>(morton_codes, i + (l + t) * d);
    if (delta(num_codes, i, i + (l + t) * d, ci, clb) > delta_min)
      l += t;
  }

  int j = i + l * d;
  cj = gprt::load<uint>(morton_codes, j);

  // Find the split position using binary search
  int delta_node = delta(num_codes, i, j, ci, cj);
  int s = 0;
  float divf = 2.f;
  int t = ceil(l / divf);
  for(; t >= 1; divf *= 2.f, t = ceil(l / divf))
  {
    uint cs = gprt::load<uint>(morton_codes, i + (s + t) * d);
    if (delta(num_codes, i, i + (s + t) * d, ci, cs) > delta_node)
      s += t;
  }

  split = i + s * d + min(d, 0);

  if (d == 1)
    return int2(i, j);
  else
    return int2(j, i);
}

GPRT_COMPUTE_PROGRAM(MakeNodes, (LBVHData, record), (1, 1, 1)) {
  int nodeID = DispatchThreadID.x;
  int nodeStart = nodeID * sizeof(LBVHNode);

  LBVHNode node;
  node.aabbMin = float3(FLT_MAX, FLT_MAX, FLT_MAX);
  node.aabbMax = -float3(FLT_MAX, FLT_MAX, FLT_MAX);
  node.left = node.right = node.parent = node.leaf = -1;

  if (nodeID < record.numCentroids - 1) {
    storeNode(record.inner, nodeID, node); 
  }

  storeNode(record.leaves, nodeID, node);

  int2 leafRange = int2(-1, -1);
  gprt::store<int2>(record.primRanges, nodeID, leafRange);
}

GPRT_COMPUTE_PROGRAM(SplitNodes, (LBVHData, record), (1, 1, 1)) {
  int num_leaves = record.numCentroids;
  int num_codes = record.numCentroids;
  int num_inner = num_leaves - 1;
  gprt::Buffer morton_codes = record.mortonCodes;
  int index = DispatchThreadID.x;

  if (index < num_inner)
  {
    // NOTE: This is [first..last], not [first..last)!!
    int split = -1;
    int2 range = determine_range(morton_codes, num_codes, index, split);
    int first = range.x;
    int last = range.y;

    int left = split;
    int right = split + 1;

    if (left == first)
    {
      // left child is leaf
      // inner[index].left = num_inner + left;
      storeNodeLeft(record.inner, index, num_inner + left);
      // leaves[left].parent = index;
      storeNodeParent(record.leaves, left, index);
    }
    else
    {
      // left child is inner
      // inner[index].left = left;
      storeNodeLeft(record.inner, index, left);
      // inner[left].parent = index;
      storeNodeParent(record.inner, left, index);
    }

    if (right == last)
    {
      // right child is leaf
      // inner[index].right = num_inner + right;
      storeNodeRight(record.inner, index, num_inner + right);
      // leaves[right].parent = index;
      storeNodeParent(record.leaves, right, index);
    }
    else
    {
      // right child is inner
      // inner[index].right = right;
      storeNodeRight(record.inner, index, right);
      // inner[right].parent = index;
      storeNodeParent(record.inner, right, index);
    }
  }
}

GPRT_COMPUTE_PROGRAM(BuildHierarchy, (LBVHData, record), (1, 1, 1)) {
}

GPRT_COMPUTE_PROGRAM(CombineHierarchy, (LBVHData, record), (1, 1, 1)) {
}



