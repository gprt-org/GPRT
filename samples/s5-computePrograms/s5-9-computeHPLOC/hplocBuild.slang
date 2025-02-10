#include "hplocShared.h"
#include "sfc.slang"

#define HPLOC_WAVE_SIZE 32
#define UINT64_MAX      18446744073709551615ULL

// Todo... try to remove this shared memory dependency
groupshared uint cached_neighbor[HPLOC_WAVE_SIZE];

int atomicExchange(RWStructuredBuffer<uint32_t> buffer, int index, int value) {
  int original_value = -1;
  InterlockedExchange(buffer[index], value, original_value);
  return original_value;
}

int atomicAdd(RWStructuredBuffer<uint32_t> buffer, int index, int value) {
  uint32_t old;
  InterlockedAdd(buffer[index], value, old);
  return old;
}

float atomicMin32f(RWStructuredBuffer<uint> buffer, int index, float value) {
  uint ret_i = buffer[index];
  while (value < asfloat(ret_i)) {
    uint old = ret_i;
    InterlockedCompareExchange(buffer[index], old, asuint(value), ret_i);
    if (ret_i == old)
      break;
  }
  return asfloat(ret_i);
}

float atomicMax32f(RWStructuredBuffer<uint> buffer, int index, float value) {
  uint ret_i = buffer[index];
  while (value > asfloat(ret_i)) {
    uint old = ret_i;
    InterlockedCompareExchange(buffer[index], old, asuint(value), ret_i);
    if (ret_i == old)
      break;
  }
  return asfloat(ret_i);
}

#define INVALID_ID UINT32_MAX

// Shouldn't happen, but useful for debugging purposes.
#define ERROR_OUT_OF_BOUNDS -1
#define ERROR_TIMEOUT       -2

// Results in a search radius of 8, which is what's recommended in the paper.
// Larger radii than this aren't helpful.
// #define SEARCH_RADIUS_SHIFT 3

// Edit: with hilbert, it seems that a shift of 5 still helps a bit...
#define SEARCH_RADIUS_SHIFT 5

// Assuming at most 254 geometries for now, with 255 marking an inner node type.
#define GEOM_ID_BVH2 255
inline bool isLeaf(uint32_t clusterid) { return (clusterid >> 24) != GEOM_ID_BVH2; }
inline bool isNode(uint32_t clusterid) { return (clusterid >> 24) == GEOM_ID_BVH2; }
inline uint8_t getClusterMeta(uint32_t clusterid) { return uint8_t(clusterid >> 24); }
inline uint32_t getClusterIndex(uint32_t clusterid) { return clusterid & 0x00FFFFFF; }

// delta function in sec3 of the paper
// "Fast and Simple Agglomerative LBVH Construction"
// Effectively, this returns where the code at "id" and code at "id+1" differ.
// We do so by xor-ing the two codes. Larger values indicate a larger difference.
uint64_t delta(const int a, const int b, const uint n, uint64_t * mc)
{
  if (a < 0 || b >= n) return (uint64_t) (-1);
  uint64_t c = mc[a] ^ mc[b];
  if (c == 0) return a ^ (a + 1ull);
  return c;
}

// Returns either L-1 or R depending on which parent has the smaller delta value to L or R+1 respectively.
// In effect, this function returns the element which minimizes the expansion of the AABB of the current
// morton range.
uint findParentID(const int a, const int b, const uint n, uint64_t *sfc_codes)
{
  if (a == 0 || (b != n && (delta(b, b + 1, n, sfc_codes) < delta(a - 1, a, n, sfc_codes))))
    return b;
  else
    return a - 1;
}

float getSurfaceArea(float2x3 bounds) {
  float3 d = bounds[1] - bounds[0];
  float sa = max(2.0f * (d.x * d.y + d.x * d.z + d.y * d.z), 0.0f);
  return sa;
}

// Returns the surface area of the AABB containing A and B as a "distance" for the PLOC nearest neighbor search.
// Returned values are guaranteed to be non-negative.
float distanceFct(float2x3 A, float2x3 B) {
  if (any(A[1] < A[0]) || any(B[1] < B[0])) return 1e38f;   // one box is invalid
  float3 d = max(A[1], B[1]) - min(A[0], B[0]);
  float sa = max(2.0f * (d.x * d.y + d.x * d.z + d.y * d.z), 0.0);
  return sa;
}

// Counts the trailing zeros in a 32-bit integer
uint countTrailingZero(uint x) {
  // Check for the special case when x is 0
  if (x == 0) {
    return 32;   // or use a special value to indicate no bits are set
  }
  return firstbitlow(x);
}

// Finds the "Nth" set bit in a 32-bit integer.
uint findNthSetBit(uint mask, uint n) {
  uint count = 0;
  for (uint i = 0; i < 32; ++i) {
    if ((mask & (1 << i)) != 0) {
      count++;
      if (count == n) {
        return i;   // Return the position of the nth set bit
      }
    }
  }
  return uint(-1);   // Return an invalid position if there aren't enough bits set
}

// Encodes the lane ID and neighbor ID into a single integer in a way that allows for unified atomic min/max operations.
// Taken from here: https://github.com/RenderKit/embree/blob/8ced524ed3066adfe8175ac15a187f3d6bd26f14/kernels/rthwif/rtbuild/rthwif_embree_builder_ploc.h#L1658
uint encodeRelativeOffset(const uint ID, const uint neighbor)
{
  const uint uOffset = neighbor - ID - 1;
  return uOffset << 1;
}

// Extracts the lane ID from the encoded value above.
// Taken from here: https://github.com/RenderKit/embree/blob/8ced524ed3066adfe8175ac15a187f3d6bd26f14/kernels/rthwif/rtbuild/rthwif_embree_builder_ploc.h#L1664
int decodeRelativeOffset(const int localID, const uint offset, const uint ID)
{
  const uint off = (offset >> 1) + 1;
  return localID + (((offset ^ ID) % 2 == 0) ? (int) off : -(int) off);
}

// Loads cluster indices from the buffer into the lanes of the wave.
// If the lane ID is less than offset or more than end-start, CI is returned
// unmodified.
// Returns the number of valid indices loaded.
uint loadIndices(uint start, uint end, uint2* I, inout uint2 CI, uint offset)
{
  uint localWaveID = WaveGetLaneIndex();
  uint laneCount = WaveGetLaneCount();
  uint numIndices = min(end - start, uint(laneCount / 2));
  uint indexID = localWaveID - offset;
  bool laneActive = (indexID >= 0 && indexID < numIndices);
  if (laneActive)
    CI = I[start + indexID];
  uint numValid = WaveActiveCountBits(laneActive && CI.x != INVALID_ID);
  numIndices = min(numIndices, numValid);
  return numIndices;
}

// Note, call this for the total number of primitives *before* PLOC merged them.
// This needs to store back the cleared "CI" values.
// When calling this function, CI registers should be compact across the wave:
//    size([CI0, CI1, ... CI_(numPrims-1), -1, -1, ... -1]) = origNumPrims
void storeIndices(int origNumPrims, uint2 CI, uint2* I, uint LStart) {
  uint localWaveID = WaveGetLaneIndex();
  if (localWaveID < origNumPrims) {
    I[LStart + localWaveID] = CI;
  }
}

// Returns the geometry ID assigned to the given thread, or -1 if out of bounds
uint GetGeometryID(in HPLOCParams params, uint3 DispatchThreadID) {
  // Might need to account for multi-dimensional dispatches later...
  uint threadID = DispatchThreadID.x;

  // This is a bit naive at the moment, just doing a linear search...
  uint numGeoms = params.M;
  uint geomID = INVALID_ID;
  for (int i = 0; i < numGeoms; ++i) {
    uint geomIDPrefixSum = params.primPrefix[i];
    if (geomIDPrefixSum <= threadID) geomID++;
    else break;
  }
  if (geomID >= numGeoms) return INVALID_ID;
  else return geomID;
}

uint GetPrimitiveID(in HPLOCParams params, uint3 DispatchThreadID, uint geomID) {
  // Might need to account for multi-dimensional dispatches later...
  uint threadID = DispatchThreadID.x;

  // If the current geometry ID represents an inner node, just assume a direct
  // correspondance between the thread ID and the BVH2 node ID.
  if (geomID == GEOM_ID_BVH2) return threadID;

  // Invalid geometry ID
  if (geomID < 0 || geomID >= params.M) return INVALID_ID;

  // Otherwise, return the primitive ID by subtracting the prefix sum of the previous geometry
  uint primOffset = params.primPrefix[geomID]; 
  return threadID - primOffset;
}

// Loads the triangle at the given primitive ID and geometry ID.
bool LoadTriangle(in HPLOCParams params, uint primID, uint geomID, out float3 v[3]) {
  v = float3[3](float3(0.0), float3(0.0), float3(0.0));

  // Check for invalid geometry id
  if (geomID < 0 || geomID >= params.M /*params.primPrefix.size*/) return false;

  uint3 *triangles = params.triangles[geomID];
  float3* vertices = params.vertices[geomID];

  // For the moment, assuming all primitive IDs are valid...
  // // Check for invalid primitive ID
  // if (primID >= triangles.size) return false;

  uint3 tri = triangles[primID];
  v[0] = vertices[tri.x];
  v[1] = vertices[tri.y];
  v[2] = vertices[tri.z];

  // Check for invalid vertex data
  if (any(isnan(v[0])) || any(isnan(v[1])) || any(isnan(v[2]))) return false;
  return true;
}

// Wrapper for the function above, decompresses clusterID into geomID / primID
bool LoadTriangle(in HPLOCParams params, uint clusterID, out float3 v[3]) {
  uint geomID = clusterID >> 24;
  uint primID = clusterID & 0x00FFFFFF;
  return LoadTriangle(params, primID, geomID, v);
};

// Loads the triangle bounds at the given primitive ID and geometry ID.
bool LoadTriangleBounds(in HPLOCParams params, uint primID, uint geomID, out float2x3 bounds) {
  bounds = float2x3(float3(FLT_MAX), -float3(FLT_MAX));
  float3 v[3];
  if (LoadTriangle(params, primID, geomID, v)) {
    bounds = float2x3(min(min(v[0], v[1]), v[2]), max(max(v[0], v[1]), v[2]));
    return true;
  }
  // Triangle is either degenerate or geomID/primID is invalid.
  return false;
}

// Loads the BVH2 bounds at the given node ID.
bool LoadBvh2Bounds(in HPLOCParams params, uint clusterID, out float2x3 bounds) {
  bounds = float2x3(float3(FLT_MAX), -float3(FLT_MAX));
  uint index = getClusterIndex(clusterID);
  if (index >= (params.N - 1)/*params.BVH2.size*/) return false;
  BVH2Node node = params.BVH2[index];
  bounds[0] = node.aabbMinAndL.xyz; // load<float4>(params.BVH2, index * 2 + 0).xyz;
  bounds[1] = node.aabbMaxAndR.xyz; // load<float4>(params.BVH2, index * 2 + 1).xyz;
  return true;
}

// Derives the AABB of either a triangle or an inner node on-the-fly.
// If either the node or primitive is invalid, returns false and an empty AABB.
bool LoadAABB(in HPLOCParams params, uint2 clusterIndex, out float2x3 bounds) {
  // Default to an empty AABB
  bounds = float2x3(float3(FLT_MAX), -float3(FLT_MAX));

  // Cluster index represents an internal BVH2 node
  if (clusterIndex.y == GEOM_ID_BVH2) {
    return LoadBvh2Bounds(params, clusterIndex.x, bounds);
  }

  // Cluster index represents a leaf belonging to a geometry
  else if (clusterIndex.y < params.M /*params.primPrefix.size*/) {
    return LoadTriangleBounds(params, clusterIndex.x, clusterIndex.y, bounds);
  }

  // Could also check here somehow for other types of geometry...

  // cluster is invalid
  else return false;
}

// Loads the center of a triangle at the given primitive ID and geometry ID.
bool LoadTriangleCenter(in HPLOCParams params, uint primID, uint geomID, out float3 center) {
  center = float3(0.0);
  float3 v[3];
  if (LoadTriangle(params, primID, geomID, v)) {
    center = (v[0] + v[1] + v[2]) / 3.f;
    return true;
  }
  return false;
}

// Loads the center of a BVH2 node at the given node ID.
bool LoadBVH2Center(in HPLOCParams params, uint nodeID, out float3 center) {
  center = float3(0.0);
  float2x3 bounds;
  if (LoadBvh2Bounds(params, nodeID, bounds)) {
    center = (bounds[0] + bounds[1]) * 0.5f;
    return true;
  }
  return false;
}

// Loads the center of either a triangle or an inner node.
// If either the node or primitive is invalid, returns false and an empty center.
bool LoadCenter(in HPLOCParams params, uint2 clusterIndex, out float3 center) {
  // Default center as 0
  center = float3(0.0);

  // Cluster index represents an internal BVH2 node
  if (clusterIndex.y == GEOM_ID_BVH2) {
    return LoadBVH2Center(params, clusterIndex.x, center);
  }

  // Cluster index represents a leaf belonging to a geometry
  else if (clusterIndex.y < params.M) {
    return LoadTriangleCenter(params, clusterIndex.x, clusterIndex.y, center);
  }

  // Could also check here somehow for other types of geometry...

  // cluster is invalid
  else return false;
}

// NN < -findNearestNeighbor(numPrims, CI, B); // inlined below
uint findNearestNeighbor(uint numPrims, float2x3 bounds) {
  uint localWaveID = WaveGetLaneIndex();
  cached_neighbor[localWaveID] = INVALID_ID;
  GroupMemoryBarrierWithWaveSync();

  const uint SEARCH_RADIUS = (uint) 1 << SEARCH_RADIUS_SHIFT;
  const uint encode_mask = ~(((uint) 1 << (SEARCH_RADIUS_SHIFT + 1)) - 1);
  uint min_area_index = -1;
  for (uint r = 1; r <= SEARCH_RADIUS; r++)
    {
    // float2x3 bounds1 = wave_shuffle_down(cached_bounds, r);
    float2x3 neighborBounds = WaveReadLaneAt(bounds, localWaveID + r);   // min(localWaveID + r, laneCount - 1));
    if ((localWaveID + r) < numPrims)
        {
      float new_area = distanceFct(bounds, neighborBounds);   // note, currently returns infinite "cost" for invalid bounds
      uint new_area_i = ((asuint(new_area) << 1) & encode_mask);
      const uint encode0 = encodeRelativeOffset(localWaveID, localWaveID + r);
      const uint new_area_index0 = new_area_i | encode0 | (localWaveID & 1);
      const uint new_area_index1 = new_area_i | encode0 | (((localWaveID + r) & 1) ^ 1);
      min_area_index = min(min_area_index, new_area_index0);
      InterlockedMin(cached_neighbor[localWaveID + r], new_area_index1);   // WaveMaskMin?
    }
  }
  InterlockedMin(cached_neighbor[localWaveID], min_area_index);
  uint neighbor = cached_neighbor[localWaveID];
  return neighbor;
}

// numPrims <- mergeClustersCreateBVH2Node(numPrims, NN, CI, B) // Inlined below...
uint mergeClustersCreateBVH2Node(uint numPrims, uint NN, inout uint2 CI, inout float2x3 bounds, HPLOCParams params) {
  uint localWaveID = WaveGetLaneIndex();

  const uint decode_mask = (((uint) 1 << (SEARCH_RADIUS_SHIFT + 1)) - 1);
  const uint n_i = decodeRelativeOffset(localWaveID, WaveReadLaneAt(NN, localWaveID) & decode_mask, localWaveID);
  const uint n_i_n_i = decodeRelativeOffset(n_i, WaveReadLaneAt(NN, n_i) & decode_mask, n_i);
  bool symmetricMatchFound = (localWaveID == n_i_n_i);   // true if our neighbor thinks we're their nearest neighbor too
  bool laneIsLeftNeighbor = localWaveID < n_i;           // true if we're the smaller of the pair

  const uint2 leftCluster = CI;
  const uint2 rightCluster = WaveReadLaneAt(CI, n_i);
  const float2x3 leftBounds = bounds;
  const float2x3 rightBounds = WaveReadLaneAt(bounds, n_i);

  // First, use an atomic counter to allocate the new number of BVH2 nodes.
  // We will insert from right to left, such that the resulting BVH2 nodes are depth first from left to right
  // and the final node (the root node) will be located at 0.

  // First, determine how many nodes we'll be generating in this wave.
  // Only the left of neighbor pairs will generate nodes. The right of the pair will clear.
  bool laneHasCluster = (localWaveID < numPrims);
  bool laneIsCreatingNode = ((laneHasCluster) && (symmetricMatchFound) && (laneIsLeftNeighbor));
  uint numToAppend = WaveActiveCountBits(laneIsCreatingNode);
  uint numNodesAllocated = INVALID_ID;
  if (localWaveID == 0) numNodesAllocated = atomicAdd(params.AC, 1, numToAppend);
  numNodesAllocated = WaveReadLaneFirst(numNodesAllocated);

  // Because BVH2 nodes are generated from the bottom up, we reverse the address here
  // to ensure the root node appears first.
  int baseNodeOffset = (((params.N - 1) - numToAppend) - numNodesAllocated);

  // This should never happen, but if it does, we need to set the error bit and stop.
  if (numNodesAllocated >= params.N - 1) {
    params.AC[5] = ERROR_OUT_OF_BOUNDS;
  }

  // New nodes are appended after leaves and previous appended nodes
  uint bvh2IndexPrefix = WavePrefixCountBits(laneIsCreatingNode);
  uint bvh2Index = baseNodeOffset + bvh2IndexPrefix;

  uint2 newCI = uint2(INVALID_ID, INVALID_ID);
  if (laneHasCluster) {
    newCI = CI;
    if (symmetricMatchFound) {
      if (laneIsLeftNeighbor) {
        // Merge the two clusters into a new BVH2 node
        float2x3 new_bounds;
        new_bounds[0] = min(leftBounds[0], rightBounds[0]);
        new_bounds[1] = max(leftBounds[1], rightBounds[1]);

        const BVH2Node newNode = BVH2Node(leftCluster, rightCluster, new_bounds);
        params.BVH2[bvh2Index] = newNode;
        bounds = new_bounds;
        newCI = uint2(bvh2Index, GEOM_ID_BVH2);   // Note, marking the merged cluster type here as a BVH2 node
      }
            else
        newCI = uint2(INVALID_ID, INVALID_ID);   // Second item of pair with the larger index disables the slot
    }
  }

  // Compact away cleared nodes
  const uint flag = (newCI.x != INVALID_ID) ? 1 : 0;
  const uint ps = laneHasCluster ? flag : 0;
  const uint total_reduction = WaveActiveCountBits(bool(ps));   //__popc(__ballot(ps));
  const uint compactionShuffle = findNthSetBit(WaveActiveBallot(bool(ps)).x, localWaveID + 1);   // findNthSetBit(__ballot(ps), 0, localWaveID + 1);
  bounds = WaveReadLaneAt(bounds, compactionShuffle);         // bounds = wave_shuffle(bounds, compactionShuffle);

  CI = newCI.x != INVALID_ID ? newCI : CI;
  CI = WaveReadLaneAt(newCI, compactionShuffle);   // CI = wave_shuffle(CI, compactionShuffle);
  return total_reduction;
}

void plocMerge(uint selectedLaneID, uint L, uint R, uint S, bool final, uniform HPLOCParams params) {
  uint localWaveID = WaveGetLaneIndex();
  uint laneCount = WaveGetLaneCount();

  // Broadcast the range and split provided by "laneID" to the entire wave
  uint LStart = WaveReadLaneAt(L, selectedLaneID);
  uint REnd = WaveReadLaneAt(R, selectedLaneID) + 1;   // (need to add +1 to make the interval half-open, see page 7)
  uint LEnd = WaveReadLaneAt(S, selectedLaneID);
  uint RStart = LEnd;   // (same as LEnd)

  // Per-wave cluster indices, initialized as invalid
  // (In this implementation, the "Y" component stores the type (leaf, inner node) as well as the geometry ID.)
  uint2 CI = uint2(INVALID_ID, INVALID_ID);   // [CI0, CI1, ... CI_(WAVE_SIZE-1)];
  uint numLeft = loadIndices(LStart, LEnd, params.I, CI, 0);
  uint numRight = loadIndices(RStart, REnd, params.I, CI, numLeft);
  uint numPrims = numLeft + numRight;

  // (Caching the AABB bounds here to avoid redundant loads)
  float2x3 bounds;
  bool validBounds = LoadAABB(params, CI, bounds);

  uint THRESHOLD = WaveReadLaneAt(final, selectedLaneID) == true ? 1 : laneCount / 2;
  while (numPrims > THRESHOLD) {
    uint NN = findNearestNeighbor(numPrims, bounds);
    numPrims = mergeClustersCreateBVH2Node(numPrims, NN, CI, bounds, params);
  }
  /* (small typo in paper, this should be numLeft + numRight, not numPrims) */
  storeIndices(numLeft + numRight, CI, params.I, LStart);
}

// Computes the bounding box containing all geometry in the tree.
// Run N threads, where N is the sum of all prims of all geometries.
[shader("compute")]
[numthreads(128, 1, 1)]
void HPLOC_Bounds(uint3 DispatchThreadID: SV_DispatchThreadID, uniform HPLOCParams params)
{
  uint threadID = DispatchThreadID.x;
  if (threadID >= params.N) return;

  // Determine the geometry and primitive ID for the current thread
  uint geomID = GetGeometryID(params, DispatchThreadID);
  uint primID = GetPrimitiveID(params, DispatchThreadID, geomID);

  // Fetch the bounding box for the current primitive
  // (if not valid, bounds will be set to an empty AABB)
  float2x3 bounds;
  bool valid = LoadAABB(params, uint2(primID, geomID), bounds);

  // Use local wave reduction to reduce global atomic contention
  uint numValidPrims = WaveActiveCountBits(valid);
  bounds[0] = WaveActiveMin(bounds[0]);
  bounds[1] = WaveActiveMax(bounds[1]);

  // Only the first lane of each wave will update the global bounds
  if (WaveIsFirstLane()) {
    // Global atomic min/max to update global root bounds
    atomicMin32f(params.rootBounds, 0, bounds[0].x);
    atomicMin32f(params.rootBounds, 1, bounds[0].y);
    atomicMin32f(params.rootBounds, 2, bounds[0].z);
    atomicMax32f(params.rootBounds, 3, bounds[1].x);
    atomicMax32f(params.rootBounds, 4, bounds[1].y);
    atomicMax32f(params.rootBounds, 5, bounds[1].z);

    // Count the number of valid primitives.
    atomicAdd(params.AC, 0, numValidPrims);
  }
}

// Computes the morton code for each primitive of each geometry.
// If a primitive is invalid, it will receive a code of UINT32_MAX, causing the
// code to be reordered to the end of the list during the radix sort.
[shader("compute")]
[numthreads(128, 1, 1)]
void HPLOC_SFC(uint3 DispatchThreadID: SV_DispatchThreadID, uint3 GroupID: SV_GroupID, uniform HPLOCParams params)
{
  uint threadID = DispatchThreadID.x;
  if (threadID >= params.N) return;

  // Determine the geometry and primitive ID for the current thread
  uint geomID = GetGeometryID(params, DispatchThreadID);
  uint primID = GetPrimitiveID(params, DispatchThreadID, geomID);

  uint2 clusterIndex = uint2(primID, geomID);
  if (primID >= params.N) return;

  // Transform center into 0-1 range
  float3 aabbMin, aabbMax;
  aabbMin.x = asfloat(params.rootBounds[0]);
  aabbMin.y = asfloat(params.rootBounds[1]);
  aabbMin.z = asfloat(params.rootBounds[2]);
  aabbMax.x = asfloat(params.rootBounds[3]);
  aabbMax.y = asfloat(params.rootBounds[4]);
  aabbMax.z = asfloat(params.rootBounds[5]);

  float3 center;
  bool valid = LoadCenter(params, clusterIndex, center);

  // Default to a code that throws invalid primitives to the end
  uint64_t code = (valid) ? hilbert64_encode(center) : UINT64_MAX;

  // Store the sfc code as the key, and the cluster index as the value
  params.C[threadID] = code;
  params.I[threadID] = clusterIndex;

  // Also, initialize the scratch "parent IDs" buffer to an invalid state.
  // As we walk up the tree from the leaves, this will be used to deactivate lanes.

  // While here, clear our parent ID buffer
  params.pID[threadID] = INVALID_ID;

  // Array of index pairs for BVH2 to BVH8 conversion
  if (threadID == 0) {
    // Allocate the wide root node
    uint64_t pair = /*root bvh2 cluster ID*/ (uint64_t(GEOM_ID_BVH2 << 24) << 32ull) | /*where bvh8 root will go*/ (0ull);
    params.indexPairs[threadID] = pair;
    params.AC[3] = 1;   // global counter of allocated BVH-N nodes
    params.AC[4] = 0;   // global counter of allocated BVH-N leaves
  }
  else {
    uint64_t pair = UINT64_MAX;   // invalid pair
    params.indexPairs[threadID] = pair;
  }
}

// Builds a binary BVH from the sorted (mortoncode / clusterIndex) pairs, following algorithm 1 in the H-PLOC paper. 
// Run for N threads, where N is the number of primitives.
[shader("compute")]
[numthreads(HPLOC_WAVE_SIZE, 1, 1)]
void HPLOC_Build(uint3 DispatchThreadID: SV_DispatchThreadID, uint3 GroupThreadID: SV_GroupThreadID, uint3 GroupID: SV_GroupID, uniform HPLOCParams params)
{
  using namespace gprt;
  
  uint i = DispatchThreadID.x;

  // The total number of leaves in the tree
  uint N = params.N;
  
  // The current node's (inclusive) child node range [L;R].
  uint L = i, R = i;

  // Initially, all lanes that correspond to codes are active.
  // (Note, we will have some helper lanes beyond this range, rounded up to the wave size)
  bool laneActive = i < N;

  // Do bottom-up traversal as long as there are active lanes in wave
  while (WaveActiveAnyTrue(laneActive)) {
    uint split = INVALID_ID;
    if (laneActive) {
      int previousID = INVALID_ID;
      if (findParentID(L, R, N, params.C) == R) {
        previousID = atomicExchange(params.pID, R, L);
        if (previousID != INVALID_ID) {
          split = R + 1;
          R = previousID;
        }
      }
      else {
        previousID = atomicExchange(params.pID, L - 1, R);
        if (previousID != INVALID_ID) {
          split = L;
          L = previousID;
        }
      }
      if (previousID == INVALID_ID) laneActive = false;
    }

    uint size = R - L + 1;
    bool final = laneActive && size == N; // Reached top of Morton tree, need to finish BVH2
    uint waveMask = WaveActiveBallot(laneActive && (size > HPLOC_WAVE_SIZE / 2) || final).x;
    while (waveMask != 0) {
      uint laneID = countTrailingZero(waveMask);
      plocMerge(laneID, L, R, split, final, params); // Wave-based PLOC++ (Algorithm 2)
      waveMask = waveMask & (waveMask - 1); // Done with current lane
    }
  }
}

// Implements the HPLOC BVH2->BVHN conversion algorithm.
[shader("compute")]
[numthreads(1024, 1, 1)]
void HPLOC_ToBVHN(uint3 DispatchThreadID: SV_DispatchThreadID, uniform HPLOCParams params)
{
}