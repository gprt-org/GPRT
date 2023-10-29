#pragma once

// #define COLLECT_STATS

// The higher this number is, the more primitives we can store in our tree.
#define MAX_LEVELS 10

// The higher this number is, the more clusters we're going to touch
// relative to the number of primitives. 
#define BRANCHING_FACTOR 4

// With a branching factor of 4 and 12 levels (10 excluding the leaves),
// we can support up to 8 million primitives per nearest neighbor tree.

// Edit: nevermind... I had a bug with my previous minMaxDist function which was giving me some 
// incorrect intuition. I'm finding now that this is very helpful for the utah teapot.
// Edit edit: Downward pruning seems to fall apart when nodes are quantized. The reason 
// for why is because the nearest face might differ between the quantized and unquantized boxes.
// Nearest face only matches when quantized box is just a scaling up or down of the original. 
// It's very rare that this happens, but also, the performance improvements from this aren't very good.
//
// Edit edit edit: When using OBBs, it seems like the maximal distance to the box gives good performance 
// improvements.
// More updates, the "minMaxDistance" metric doesn't work for internal OBB nodes, since internal faces aren't guaranteed
// to contain primitives. Only OBB leaves can guarantee this. So, for internal nodes, we instead use the "maxDistance"
// for downward culling.
// More more updates... So long as internal OBBs are built to tightly fit the underlying primitives, then minMaxDist is
// "free game". However, it introduces some compute complexity which doesn't seem worth the effort
// #define ENABLE_DOWNAWARD_CULLING 

// Enables an LBVH reference, similar to Jakob and Guthe's knn.
// Note, we use this LBVH as a top level tree for our method, so 
// this is just about if we're using RT cores or not
#define ENABLE_LBVH_REFERENCE

// Uses a quantized representation for the active cluster queue
// I've bounced back and forth on if this helps or not. 
//   For untruncated queries, the ultimate bottleneck is when tree traversal fails and many prims 
//   are all approximately the same distance. There, its better to have an accurate queue for culling. 
//   However, I suspect the issue with culling has more to do with using AABBs over something tigher fitting
// #define ENABLE_QUEUE_QUANTIZATION


// Enables oriented bounding boxes in an attempt to improve culling performance. 
// Prior experiments seem to suggest that the minimum distance to an axis aligned bounding 
// box tends to be a poor fit for the minimum distance to the contained primitive. This
// issue seems to become more and more extreme as primitives are clustered together.
// This mode uses a singular value decomposition to compute the orientation of a bounding box 
// that best fits the underlying primitive distribution
// #define ENABLE_OBBS

// #define ENABLE_AABBS

// #define ENABLE_6DOPS

// Honestly I don't see much improvement by avoiding square roots. I don't think this is what the bottleneck is
// at least on my problems... Just going to take square roots for simplicity...
// #define USE_SQUARED_DISTS

// NOTE, struct must be synchronized with declaration in gprt_host.h
namespace gprt{

    typedef uint32_t Texture;
    typedef uint32_t Sampler;

    #ifdef GPRT_DEVICE
    // ideally this buffer type would be a struct on the GPU...
    // but I'm running into a compiler bug reading one struct inside another one.
    // x stores pointer, y stores size.
    typedef uint64_t2 Buffer;
    typedef uint64_t2 Accel;
    

    #else
    struct Buffer {
        uint64_t x;
        uint64_t y;
    };
    struct Accel {
        uint64_t x;
        uint64_t y;
    };
    #endif

    struct NNAccel {
        // input
        uint32_t numPrims;
        uint32_t numLevels;
        uint32_t numClusters[MAX_LEVELS];
        float maxSearchRange;

        gprt::Buffer points; 

        // Why are we separating these again?... 
        // They could just be "primitives"...
        gprt::Buffer edges; 
        gprt::Buffer triangles;

        // A sorted list of triangle indices, meant to be traversed
        // linearly from the leaves
        gprt::Buffer triangleLists;

        // Hilbert codes of quantized primitive centroids
        // One uint64_t per primitive
        gprt::Buffer codes;
        gprt::Buffer ids;

        // Buffer containing the global AABB. Pair of two floats
        gprt::Buffer aabb;        

        // Buffers of bounding primitives. 
        // If axis aligned bounding boxes, each is a pair of float3.
        // If bounding balls, each is a single float4 (xyzr).
        // If oriented bounding boxes, each is a triplet of float3s.
        //   - note, also reused for temporarily storing covariance matrices
        gprt::Buffer aabbs[MAX_LEVELS];
        gprt::Buffer oobbs[MAX_LEVELS];
        gprt::Buffer centers[MAX_LEVELS];
 
        // 3 floats for treelet aabb min, 
        // 3 bytes for scale exponent, one unused 
        // byte   15   14    13    12    11 10 9 8   7  6  5  4  3  2  1  0
        //       [??]  [sz]  [sy]  [sx]  [  zmin  ]  [  ymin  ]  [  xmin  ]
        // gprt::Buffer treelets;

        // If an "axis aligned bounding box", 64-bit integers, 6 bytes for bounding box, 2 unused.
        // byte   8    7   6     5     4     3     2     1           
        //       [?]  [?]  [zh]  [yh]  [xh]  [zl]  [yl]  [xl]        

        // If a "bounding ball", 32-bit integers, 3 bytes for center, 1 for radius.
        // byte   4    3    2    1           
        //       [r]  [z]  [y]  [x]        

        // If an "oriented bounding box", we use a 128-bit integer. 
        // 6 bytes for bounding box, 2 unused. 
        // Then 20 bits for Euler rotations in X, Y, then Z. 4 bits unused.
        //        15 14 13 12 11 10 9 8   7   6   5     4     3    2    1    0
        //       [ zr ]  [ yr ]  [ xr ]   [?] [?] [zh]  [yh]  [xh]  [zl]  [yl]  [xl]
        // gprt::Buffer children; 

        // An RT core tree
        gprt::Accel accel;

        // An LBVH tree
        // gprt::Buffer lbvhMortonCodes;

        // Primitive IDs that correspond to sorted morton codes. 
        // One uint32_t per primitive
        // gprt::Buffer lbvhIds;

        // numPrims-1 + numPrims long. 
        // The "numPrims-1" section contains inner nodes
        // The "numPrims" section contains leaves
        // Each node is an int4. 
        // "X" is left, "Y" is right, "Z" is parent, and "W" is leaf or -1 if internal node.
        // gprt::Buffer lbvhNodes;

        // numPrims-1 + numPrims long. Each aabb is a pair of float3.
        // gprt::Buffer lbvhAabbs;

    };

    struct NNConstants {
        // An internal iteration parameter for construction
        uint32_t iteration;
        uint32_t stride;
        uint32_t numLevels;
        uint32_t level;
        uint32_t numPrims;
        gprt::Buffer triangles;

        gprt::Buffer buffer1;
        gprt::Buffer buffer2;
        gprt::Buffer buffer3;
        gprt::Buffer buffer4;
        gprt::Buffer buffer5;
    };
};