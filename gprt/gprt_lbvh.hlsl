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

#include "gprt_lbvh.h"

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

uint clz(uint x) {
  if (firstbithigh(x) == -1) return 32;
  return 31 - firstbithigh(x);
}

int delta(gprt::Buffer morton_codes, uint num_codes, uint i, uint j) {
  // Karras' delta(i,j) function
  // Denotes the length of the longest common
  // prefix between keys k_i and k_j

  // Cf. Figure 4: "for simplicity, we define that
  // delta(i,j) = -1 when j not in [0,n-1]"
  if (j < 0 || j >= num_codes)
    return -1;
  uint i_code = gprt::load<uint>(morton_codes, i);
  uint j_code = gprt::load<uint>(morton_codes, j);
  uint xord = i_code ^ j_code;
  if (xord == 0)
    return clz(i ^ j) + 32;
  else
    return clz(xord);
}

// Find node range that an inner node overlaps
int2 determine_range(
  gprt::Buffer morton_codes,
  int num_codes, int i, inout int split) {

  // Determine direction of the range (+1 or -1)
  int d = delta(morton_codes, num_codes, i, i + 1) >= delta(morton_codes, num_codes, i, i - 1) ? 1 : -1;

  // Compute upper bound for the length of the range
  int delta_min = delta(morton_codes, num_codes, i, i - d);
  int l_max = 2;
  while (delta(morton_codes, num_codes, i, i + l_max * d) > delta_min)
    l_max *= 2;

  // Find the other end using binary search
  int l = 0;
  for (int t = l_max >> 1; t >= 1; t >>= 1)
    if (delta(morton_codes, num_codes, i, i + (l + t) * d) > delta_min)
      l += t;

  int j = i + l * d;

  // Find the split position using binary search
  int delta_node = delta(morton_codes, num_codes, i, j);
  int s = 0;
  float divf = 2.f;
  int t = ceil(l / divf);
  for(; t >= 1; divf *= 2.f, t = ceil(l / divf))
  {
    uint cs = gprt::load<uint>(morton_codes, i + (s + t) * d);
    if (delta(morton_codes, num_codes, i, i + (s + t) * d) > delta_node)
      s += t;
  }

  split = i + s * d + min(d, 0);

  if (d == 1)
    return int2(i, j);
  else
    return int2(j, i);
}

void getPointBounds(gprt::Buffer positions, uint primID, out float3 aabbMin, out float3 aabbMax)
{
  float3 p = gprt::load<float3>(positions, primID);
  aabbMin = p;
  aabbMax = p;
}

void getEdgeBounds(gprt::Buffer edges, gprt::Buffer positions, uint primID, out float3 aabbMin, out float3 aabbMax)
{
  uint2 edge = gprt::load<uint2>(edges, primID);
  float3 p1 = gprt::load<float3>(positions, edge.x);
  float3 p2 = gprt::load<float3>(positions, edge.y);
  aabbMin = float3(min(p1.x,p2.x),min(p1.y,p2.y),min(p1.z,p2.z));
  aabbMax = float3(max(p1.x,p2.x),max(p1.y,p2.y),max(p1.z,p2.z));
}

void getTriangleBounds(gprt::Buffer triangles, gprt::Buffer positions, uint primID, out float3 aabbMin, out float3 aabbMax)
{
  uint3 tri = gprt::load<uint3>(triangles, primID);
  float3 p1 = gprt::load<float3>(positions, tri.x);
  float3 p2 = gprt::load<float3>(positions, tri.y);
  float3 p3 = gprt::load<float3>(positions, tri.z);
  aabbMin = float3(min(p1.x,min(p2.x,p3.x)),min(p1.y,min(p2.y,p3.y)),min(p1.z,min(p2.z,p3.z)));
  aabbMax = float3(max(p1.x,max(p2.x,p3.x)),max(p1.y,max(p2.y,p3.y)),max(p1.z,max(p2.z,p3.z)));
}

float3 getEdgeCentroid(gprt::Buffer edges, gprt::Buffer positions, uint primID) 
{
  uint2 edge = gprt::load<uint2>(edges, primID);
  float3 p1 = gprt::load<float3>(positions, edge.x);
  float3 p2 = gprt::load<float3>(positions, edge.y);
  return (p1 + p2) / 2.f;
}

float3 getTriangleCentroid(gprt::Buffer triangles, gprt::Buffer positions, uint primID) 
{
  uint3 tri = gprt::load<uint3>(triangles, primID);
  float3 p1 = gprt::load<float3>(positions, tri.x);
  float3 p2 = gprt::load<float3>(positions, tri.y);
  float3 p3 = gprt::load<float3>(positions, tri.z);
  return (p1 + p2 + p3) / 3.f;
}

void storeNode(gprt::Buffer buffer, int address, int4 node) {
  gprt::store<int4>(buffer, address, node);
}

void storeNodeLeft(gprt::Buffer buffer, int address, int left) {
  gprt::store<int>(buffer, address * 4 + 0, left);
}

void storeNodeRight(gprt::Buffer buffer, int address, int right) {
  gprt::store<int>(buffer, address * 4 + 1, right);
}

void storeNodeParent(gprt::Buffer buffer, int address, int parent) {
  gprt::store<int>(buffer, address * 4 + 2, parent);
}

void storeNodeLeaf(gprt::Buffer buffer, int address, int leaf) {
  gprt::store<int>(buffer, address * 4 + 3, leaf);
}

int4 loadNode(gprt::Buffer buffer, int address) {
  return gprt::load<int4>(buffer, address);
}

int loadNodeLeft(gprt::Buffer buffer, int address) {
  return gprt::load<int>(buffer, address * 4 + 0);
}

int loadNodeRight(gprt::Buffer buffer, int address) {
  return gprt::load<int>(buffer, address * 4 + 1);
}

int loadNodeParent(gprt::Buffer buffer, int address) {
  return gprt::load<int>(buffer, address * 4 + 2);
}

int loadNodeLeaf(gprt::Buffer buffer, int address) {
  return gprt::load<int>(buffer, address * 4 + 3);
}

GPRT_COMPUTE_PROGRAM(ComputePointBounds, (LBVHData, record), (1,1,1)) {
  int primID = DispatchThreadID.x;
  if (primID >= record.numPrims) return;
  float3 aabbMin, aabbMax;
  getPointBounds(record.positions, primID, aabbMin, aabbMax);

  aabbMin.x = gprt::atomicMin32f(record.aabbs, 0, aabbMin.x);
  aabbMin.y = gprt::atomicMin32f(record.aabbs, 1, aabbMin.y);
  aabbMin.z = gprt::atomicMin32f(record.aabbs, 2, aabbMin.z);
  aabbMax.x = gprt::atomicMax32f(record.aabbs, 3, aabbMax.x);
  aabbMax.y = gprt::atomicMax32f(record.aabbs, 4, aabbMax.y);
  aabbMax.z = gprt::atomicMax32f(record.aabbs, 5, aabbMax.z);
}

GPRT_COMPUTE_PROGRAM(ComputeEdgeBounds, (LBVHData, record), (1,1,1)) {
  int primID = DispatchThreadID.x;
  if (primID >= record.numPrims) return;
  float3 aabbMin, aabbMax;
  getEdgeBounds(record.edges, record.positions, primID, aabbMin, aabbMax);
  gprt::atomicMin32f(record.aabbs, 0, aabbMin.x);
  gprt::atomicMin32f(record.aabbs, 1, aabbMin.y);
  gprt::atomicMin32f(record.aabbs, 2, aabbMin.z);
  gprt::atomicMax32f(record.aabbs, 3, aabbMax.x);
  gprt::atomicMax32f(record.aabbs, 4, aabbMax.y);
  gprt::atomicMax32f(record.aabbs, 5, aabbMax.z);
}

GPRT_COMPUTE_PROGRAM(ComputeTriangleBounds, (LBVHData, record), (1,1,1)) {
  int primID = DispatchThreadID.x;
  if (primID >= record.numPrims) return;
  float3 aabbMin, aabbMax;
  getTriangleBounds(record.triangles, record.positions, primID, aabbMin, aabbMax);
  gprt::atomicMin32f(record.aabbs, 0, aabbMin.x);
  gprt::atomicMin32f(record.aabbs, 1, aabbMin.y);
  gprt::atomicMin32f(record.aabbs, 2, aabbMin.z);
  gprt::atomicMax32f(record.aabbs, 3, aabbMax.x);
  gprt::atomicMax32f(record.aabbs, 4, aabbMax.y);
  gprt::atomicMax32f(record.aabbs, 5, aabbMax.z);
}

GPRT_COMPUTE_PROGRAM(ComputePointMortonCodes, (LBVHData, record), (1, 1, 1)) {
  int primID = DispatchThreadID.x;
  if (primID >= record.numPrims) return;
  float3 pt = gprt::load<float3>(record.positions, primID);
  float3 aabbMin = gprt::load<float3>(record.aabbs, 0);
  float3 aabbMax = gprt::load<float3>(record.aabbs, 1);

  pt = (pt - aabbMin) / (aabbMax - aabbMin);

  // Quantize to 10 bit
  pt = min(max(pt * 1024.f, float3(0.f, 0.f, 0.f)), float3(1023.f, 1023.f, 1023.f));
  
  uint code = morton_encode3D(pt.x, pt.y, pt.z);
  gprt::store<uint>(record.mortonCodes, primID, code);
  gprt::store<uint>(record.ids, primID, primID);
}

GPRT_COMPUTE_PROGRAM(ComputeEdgeMortonCodes, (LBVHData, record), (1, 1, 1)) {
  int primID = DispatchThreadID.x;
  if (primID >= record.numPrims) return;
  float3 pt = getEdgeCentroid(record.edges, record.positions, primID);
  float3 aabbMin = gprt::load<float3>(record.aabbs, 0);
  float3 aabbMax = gprt::load<float3>(record.aabbs, 1);

  pt = (pt - aabbMin) / (aabbMax - aabbMin);

  // Quantize to 10 bit
  pt = min(max(pt * 1024.f, float3(0.f, 0.f, 0.f)), float3(1023.f, 1023.f, 1023.f));
  
  uint code = morton_encode3D(pt.x, pt.y, pt.z);
  gprt::store<uint>(record.mortonCodes, primID, code);
  gprt::store<uint>(record.ids, primID, primID);
}

GPRT_COMPUTE_PROGRAM(ComputeTriangleMortonCodes, (LBVHData, record), (1, 1, 1)) {
  int primID = DispatchThreadID.x;
  if (primID >= record.numPrims) return;
  float3 pt = getTriangleCentroid(record.triangles, record.positions, primID);
  float3 aabbMin = gprt::load<float3>(record.aabbs, 0);
  float3 aabbMax = gprt::load<float3>(record.aabbs, 1);

  pt = (pt - aabbMin) / (aabbMax - aabbMin);

  // Quantize to 10 bit
  pt = min(max(pt * 1024.f, float3(0.f, 0.f, 0.f)), float3(1023.f, 1023.f, 1023.f));
  
  uint code = morton_encode3D(pt.x, pt.y, pt.z);
  gprt::store<uint>(record.mortonCodes, primID, code);
  gprt::store<uint>(record.ids, primID, primID);
}

GPRT_COMPUTE_PROGRAM(MakeNodes, (LBVHData, record), (1, 1, 1)) {
  int nodeID = DispatchThreadID.x;
  if (nodeID >= record.numNodes) return;
  storeNode(record.nodes, nodeID, int4(-1,-1,-1,-1));
  gprt::store<float3>(record.aabbs, nodeID * 2 + 0,  float3(1e20f, 1e20f, 1e20f));
  gprt::store<float3>(record.aabbs, nodeID * 2 + 1, -float3(1e20f, 1e20f, 1e20f));
}

GPRT_COMPUTE_PROGRAM(SplitNodes, (LBVHData, record), (1, 1, 1)) {
  int num_leaves = record.numPrims;
  int num_codes = record.numPrims;
  gprt::Buffer morton_codes = record.mortonCodes;
  int index = DispatchThreadID.x;

  if (index < record.numInner)
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
      storeNodeLeft(record.nodes, index, record.numInner + left);
      // leaves[left].parent = index;
      storeNodeParent(record.nodes, record.numInner + left, index);
    }
    else
    {
      // left child is inner
      // inner[index].left = left;
      storeNodeLeft(record.nodes, index, left);
      // inner[left].parent = index;
      storeNodeParent(record.nodes, left, index);
    }

    if (right == last)
    {
      // right child is leaf
      // inner[index].right = num_inner + right;
      storeNodeRight(record.nodes, index, record.numInner + right);
      // leaves[right].parent = index;
      storeNodeParent(record.nodes, record.numInner + right, index);
    }
    else
    {
      // right child is inner
      // inner[index].right = right;
      storeNodeRight(record.nodes, index, right);
      // inner[right].parent = index;
      storeNodeParent(record.nodes, right, index);
    }
  }
}

GPRT_COMPUTE_PROGRAM(BuildPointHierarchy, (LBVHData, record), (1, 1, 1)) {
  int index = DispatchThreadID.x;
  uint numPrims = record.numPrims;
  if (index >= numPrims) return;

  float3 aabbMin, aabbMax;
  uint id = gprt::load<uint>(record.ids, index);
  getPointBounds(record.positions, id, aabbMin, aabbMax);

  // Leaf's bounding box
  int leafAddr = index + (record.numPrims - 1);
  gprt::store<float3>(record.aabbs, leafAddr * 2 + 0, aabbMin);
  gprt::store<float3>(record.aabbs, leafAddr * 2 + 1, aabbMax);

  // Leaf's object
  storeNodeLeaf(record.nodes, leafAddr, id);
  
  // Atomically combine child bounding boxes and update parents
  int next = loadNodeParent(record.nodes, leafAddr);
  while (next >= 0) {
    gprt::atomicMin32f(record.aabbs, next * 6 + 0, aabbMin.x);
    gprt::atomicMin32f(record.aabbs, next * 6 + 1, aabbMin.y);
    gprt::atomicMin32f(record.aabbs, next * 6 + 2, aabbMin.z);
    gprt::atomicMax32f(record.aabbs, next * 6 + 3, aabbMax.x);
    gprt::atomicMax32f(record.aabbs, next * 6 + 4, aabbMax.y);
    gprt::atomicMax32f(record.aabbs, next * 6 + 5, aabbMax.z);
    next = loadNodeParent(record.nodes, next);   
  }
}

GPRT_COMPUTE_PROGRAM(BuildEdgeHierarchy, (LBVHData, record), (1, 1, 1)) {
  int index = DispatchThreadID.x;
  uint numPrims = record.numPrims;
  if (index >= numPrims) return;

  float3 aabbMin, aabbMax;
  uint id = gprt::load<uint>(record.ids, index);
  getEdgeBounds(record.edges, record.positions, id, aabbMin, aabbMax);

  // Leaf's bounding box
  int leafAddr = index + (record.numPrims - 1);
  gprt::store<float3>(record.aabbs, leafAddr * 2 + 0, aabbMin);
  gprt::store<float3>(record.aabbs, leafAddr * 2 + 1, aabbMax);

  // Leaf's object
  storeNodeLeaf(record.nodes, leafAddr, id);
  
  // Atomically combine child bounding boxes and update parents
  int next = loadNodeParent(record.nodes, leafAddr);
  while (next >= 0) {
    gprt::atomicMin32f(record.aabbs, next * 6 + 0, aabbMin.x);
    gprt::atomicMin32f(record.aabbs, next * 6 + 1, aabbMin.y);
    gprt::atomicMin32f(record.aabbs, next * 6 + 2, aabbMin.z);
    gprt::atomicMax32f(record.aabbs, next * 6 + 3, aabbMax.x);
    gprt::atomicMax32f(record.aabbs, next * 6 + 4, aabbMax.y);
    gprt::atomicMax32f(record.aabbs, next * 6 + 5, aabbMax.z);
    next = loadNodeParent(record.nodes, next);   
  }
}

GPRT_COMPUTE_PROGRAM(BuildTriangleHierarchy, (LBVHData, record), (1, 1, 1)) {
  int index = DispatchThreadID.x;
  uint numPrims = record.numPrims;
  if (index >= numPrims) return;

  float3 aabbMin, aabbMax;
  uint id = gprt::load<uint>(record.ids, index);
  getTriangleBounds(record.triangles, record.positions, id, aabbMin, aabbMax);

  // Leaf's bounding box
  int leafAddr = index + (record.numPrims - 1);
  gprt::store<float3>(record.aabbs, leafAddr * 2 + 0, aabbMin);
  gprt::store<float3>(record.aabbs, leafAddr * 2 + 1, aabbMax);

  // Leaf's object
  storeNodeLeaf(record.nodes, leafAddr, id);
  
  // Atomically combine child bounding boxes and update parents
  int next = loadNodeParent(record.nodes, leafAddr);
  while (next >= 0) {
    gprt::atomicMin32f(record.aabbs, next * 6 + 0, aabbMin.x);
    gprt::atomicMin32f(record.aabbs, next * 6 + 1, aabbMin.y);
    gprt::atomicMin32f(record.aabbs, next * 6 + 2, aabbMin.z);
    gprt::atomicMax32f(record.aabbs, next * 6 + 3, aabbMax.x);
    gprt::atomicMax32f(record.aabbs, next * 6 + 4, aabbMax.y);
    gprt::atomicMax32f(record.aabbs, next * 6 + 5, aabbMax.z);
    next = loadNodeParent(record.nodes, next);   
  }
}
